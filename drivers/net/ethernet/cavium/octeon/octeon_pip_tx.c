// SPDX-License-Identifier: GPL-2.0
/*
 * The transmit side of octeon_pip.
 *
 * Each port has one PKO output queue per CPU, as far as the chip has
 * them, and one netdev TX queue per PKO queue. XPS gives every CPU a
 * queue of its own as far as there are queues; the TX queue lock is all
 * the locking the command chunks of a queue need.
 *
 * Every command is two words: the command word and one buffer pointer, or
 * the pointer to a gather list. The PKO fetches commands in 64 byte units,
 * and a command that crosses a unit boundary is, intermittently and under
 * load, assembled from words the PKO already held for that unit, the
 * chunk's previous contents, instead of what the driver wrote: a wrong
 * parse, a buffer pointer of size zero, PKO_REG_ERROR[CURRZERO], the port
 * dead for good. Two word commands from the start of a 128 byte aligned
 * chunk never cross a unit, and that is the stream the vendor's software
 * writes. Three word commands (a completion through a work queue entry)
 * do cross, at one in three unit boundaries, and were the stall of this
 * driver's first design.
 *
 * So a packet does not ask for a completion. The PKO counts the packets
 * of a queue it has sent in the queue's FAU register (word 0: REG0 and
 * SUBONE0), and the completion reads the counter and frees that many
 * packets from the head of the queue's ring; the PKO sends a queue's
 * packets in the order of their commands. The counter is read in a NAPI
 * poll, under the budget the core gave it: every queue has a NAPI of its
 * own on the port, which a transmit schedules once in 8 packets, and so
 * does a timer per queue for whatever is left when nothing else comes
 * along. There is no completion interrupt: a packet that asks for one is
 * a three word command.
 */

#include <linux/atomic.h>
#include <linux/etherdevice.h>
#include <linux/hrtimer.h>
#include <linux/if_vlan.h>
#include <linux/iopoll.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/netdevice.h>
#include <linux/netpoll.h>
#include <linux/prefetch.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip.h>
#include <net/netdev_queues.h>

#include "octeon_pip.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_pko.h"
#include "octeon_pip_regs.h"
#include "octeon_pip_tx.h"

/* Free slots a stopped transmit queue needs before it is woken */
#define OPIP_TX_WAKE_THRESH	(OPIP_TXQ_DEPTH / 4)

/* Packets written between two reads of the counter by the transmit paths */
#define OPIP_TX_COMPLETE_BATCH	8

/* The timer for what xmit and the polls leave in flight */
#define OPIP_TX_TIMER_NS	(250 * NSEC_PER_USEC)

/*
 * A packet in the ring is an skb, or an skb with a gather list (bit 1,
 * the list's address is in skb->cb)
 */
#define OPIP_TX_GATHER		2UL
#define OPIP_TX_FLAGS		OPIP_TX_GATHER

static struct sk_buff *opip_tx_entry_skb(unsigned long entry)
{
	return (struct sk_buff *)(entry & ~OPIP_TX_FLAGS);
}

/* The gather list of a fragmented skb sits in a buffer from the WQE pool */
static u64 **opip_tx_skb_list(struct sk_buff *skb)
{
	return (u64 **)skb->cb;
}

/* The largest IP header offset IPOFFP1 (offset + 1, 7 bits) can name */
#define OPIP_TX_MAX_IPOFF	(FIELD_MAX(OPIP_PKO_CMD_IPOFFP1) - 1)

/* Packets handed to the PKO whose completion is still due */
static unsigned int opip_txq_inflight(struct opip_txq *q)
{
	return READ_ONCE(q->tail) - READ_ONCE(q->head);
}

static int opip_txq_free_slots(struct opip_txq *q)
{
	return OPIP_TXQ_DEPTH - opip_txq_inflight(q);
}

/*
 * The command word for a packet of one buffer on queue @q: the PKO
 * subtracts one from the queue's FAU register once the packet is out,
 * and leaves the buffer alone. Always inline: at -Os gcc calls it out
 * of line, per packet.
 */
static __always_inline u64 opip_pko_cmd(struct opip_txq *q, unsigned int len)
{
	u64 cmd = FIELD_PREP(OPIP_PKO_CMD_TOTAL_BYTES, len) |
		  FIELD_PREP(OPIP_PKO_CMD_SEGS, 1) |
		  FIELD_PREP(OPIP_PKO_CMD_REG0, opip_pko_queue_fau(q->queue)) |
		  FIELD_PREP(OPIP_PKO_CMD_SIZE0, OPIP_FAU_SIZE_32) |
		  OPIP_PKO_CMD_SUBONE0 |
		  /* don't pollute L2 with the outgoing packet */
		  OPIP_PKO_CMD_N2;

	if (IS_ENABLED(CONFIG_CPU_LITTLE_ENDIAN))
		cmd |= OPIP_PKO_CMD_LE;
	return cmd | OPIP_PKO_CMD_DONTFREE;
}

/* The buffer pointer for a packet the CPU built (always inline: as above) */
static __always_inline u64 opip_pko_buf(const void *data, unsigned int len)
{
	return opip_buf_build(opip_xkphys_to_phys(data), len, 0, 0, false);
}

/*
 * Stop the queue while it has less room than a packet and one more.
 * Always inline: at -Os gcc calls it out of line, per packet.
 */
static __always_inline void opip_tx_maybe_stop(struct netdev_queue *txq,
					       struct opip_txq *q)
{
	if (!netif_txq_maybe_stop(txq, opip_txq_free_slots(q), 2,
				  OPIP_TX_WAKE_THRESH))
		WRITE_ONCE(q->stopped, q->stopped + 1);
}

/* Tell the PKO about the command words written since the last doorbell */
static void opip_tx_doorbell(struct opip_port *p, struct opip_txq *q)
{
	if (q->doorbell) {
		opip_pko_doorbell(p->port, q->queue, q->doorbell);
		q->doorbell = 0;
	}
}

/* ------------------------------------------------------------------ */
/* Completions */

/*
 * Free the packets the PKO has sent since the last time, from the head of
 * the ring, and report them to BQL. Under the queue lock. The counter is
 * what the PKO subtracted, so its negation is the number of packets ever
 * sent on the queue; the head counts the same way. The read is an I/O
 * round trip: the transmit paths do it once in OPIP_TX_COMPLETE_BATCH
 * packets.
 *
 * @budget is the budget the core passed to the queue's NAPI poll, which
 * this runs in: the skbs go into the per-CPU NAPI cache. A drain, from
 * process context, passes 0 and frees to the slab.
 *
 * Return: the number of packets completed
 */
static unsigned int opip_tx_complete(struct net_device *dev, unsigned int qid,
				     struct opip_txq *q, int budget)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, qid);
	unsigned int head = q->head, n, pkts;
	unsigned int bytes = 0;
	u32 sent;

	sent = -opip_fau_rd32(q->fau);

	q->written = 0;
	n = sent - head;
	if (!n)
		return 0;
	/*
	 * Not without a fault: the PKO counts only the packets it was given,
	 * and only the reset at teardown takes any back from it
	 */
	if (WARN_ON_ONCE(n > q->tail - head))
		n = q->tail - head;
	pkts = n;

	while (n--) {
		unsigned long entry = q->ring[head++ & (OPIP_TXQ_RING - 1)];
		struct sk_buff *skb = opip_tx_entry_skb(entry);

		/* The next one was sent long ago, its lines have gone */
		if (n)
			prefetch((void *)(q->ring[head & (OPIP_TXQ_RING - 1)] &
					  ~OPIP_TX_FLAGS));
		if (skb) {
			if (entry & OPIP_TX_GATHER)
				opip_fpa_free_nosync(*opip_tx_skb_list(skb),
						     OPIP_POOL_WQE, 0);
			bytes += skb->len;
			napi_consume_skb(skb, budget);
		}
	}
	WRITE_ONCE(q->head, head);

	netif_txq_completed_wake(txq, pkts, bytes, opip_txq_free_slots(q),
				 OPIP_TX_WAKE_THRESH);
	return pkts;
}

/*
 * Have the timer schedule the queue's NAPI for what is in flight if
 * nothing else completes it first. Under the queue lock, which is what
 * keeps the NAPI's re-arming and an arming from another CPU apart; the
 * timer is never forwarded. It is pinned: it fires on the CPU that sent
 * last, through XPS the CPU of the queue, and the NAPI runs there.
 * Always inline: at -Os gcc calls it out of line, per stack packet.
 */
static __always_inline void opip_tx_arm(struct opip_txq *q)
{
	if (opip_txq_inflight(q) && !hrtimer_is_queued(&q->timer))
		hrtimer_start(&q->timer, OPIP_TX_TIMER_NS,
			      HRTIMER_MODE_REL_PINNED_SOFT);
}

/*
 * A transmit has written a batch: the queue's NAPI completes it, on this
 * CPU, once bottom halves are back on. Under the queue lock. The batch
 * starts over: with the NAPI still pending the next one has it polled
 * once more.
 */
static void opip_tx_kick(struct opip_txq *q)
{
	q->written = 0;
	napi_schedule(&q->napi);
}

/*
 * The queue's NAPI: complete what the PKO has sent and leave the rest to
 * the timer, under the budget the core gave the poll. netpoll's budget
 * of 0 allows no page pool skb to be freed: the NAPI stays scheduled
 * for the next real poll.
 */
static int opip_tx_napi_poll(struct napi_struct *napi, int budget)
{
	struct opip_txq *q = container_of(napi, struct opip_txq, napi);
	struct netdev_queue *txq = netdev_get_tx_queue(q->dev, q->qid);

	if (!budget)
		return 0;

	__netif_tx_lock(txq, smp_processor_id());
	opip_tx_complete(q->dev, q->qid, q, budget);
	/* A closed device's MAC sends nothing: opip_tx_resume() rearms */
	if (netif_running(q->dev))
		opip_tx_arm(q);
	__netif_tx_unlock(txq);

	/* A completion is no receive work: nothing counts against @budget */
	napi_complete_done(napi, 0);
	return 0;
}

static enum hrtimer_restart opip_tx_timer(struct hrtimer *timer)
{
	struct opip_txq *q = container_of(timer, struct opip_txq, timer);

	/* A closed device's NAPI is disabled: this does nothing then */
	napi_schedule(&q->napi);
	return HRTIMER_NORESTART;
}

/**
 * opip_txq_init - the driver's side of a transmit queue, before it is used
 * @dev: Port
 * @qid: The queue
 */
void opip_txq_init(struct net_device *dev, unsigned int qid)
{
	struct opip_txq *q = &opip_port(dev)->txq[qid];

	q->dev = dev;
	q->qid = qid;
	hrtimer_setup(&q->timer, opip_tx_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_PINNED_SOFT);
	netif_napi_add_tx(dev, &q->napi, opip_tx_napi_poll);
}

/**
 * opip_txq_exit - the driver's side of a transmit queue, when it goes away
 * @q: The queue
 *
 * The queue is going away, or its device closes: no timer may run on it
 * any more. A queue that opip_txq_init() never set up has no timer.
 */
void opip_txq_exit(struct opip_txq *q)
{
	if (!q->dev)
		return;
	hrtimer_cancel(&q->timer);
}

/**
 * opip_txq_free - release the driver's side of a transmit queue
 * @q: The queue, of a device that is closed or never was opened
 *
 * The device goes away: no timer may run on the queue and its NAPI is
 * deleted, before the packets left in the queue are freed.
 */
void opip_txq_free(struct opip_txq *q)
{
	if (!q->dev)
		return;
	opip_txq_exit(q);
	netif_napi_del(&q->napi);
}

/**
 * opip_txq_purge - free the packets of a queue the PKO will not send
 * @q: The queue, the PKO reset
 *
 * Only for the teardown: while the PKO runs, its engine may hold a packet
 * of the queue that no count shows.
 *
 * Return: how many packets were freed
 */
unsigned int opip_txq_purge(struct opip_txq *q)
{
	unsigned int head, freed = 0;

	for (head = q->head; head != q->tail; head++, freed++) {
		unsigned long entry = q->ring[head & (OPIP_TXQ_RING - 1)];
		struct sk_buff *skb = opip_tx_entry_skb(entry);

		if (skb) {
			if (entry & OPIP_TX_GATHER)
				opip_fpa_free(*opip_tx_skb_list(skb),
					      OPIP_POOL_WQE, 0);
			dev_kfree_skb_any(skb);
		}
	}
	WRITE_ONCE(q->head, head);
	return freed;
}

/* ------------------------------------------------------------------ */
/* The commands */

/*
 * The PKO inserts the TCP/UDP checksum behind the IP header at ipoffp1 - 1.
 * Only use it for packets that ask for exactly that: rewriting the
 * checksum of a forwarded packet would hide corruption that happened
 * before it reached us, and a CHECKSUM_PARTIAL that points elsewhere (a
 * virtio_net_hdr from userspace) would get a wrong one. The header may be
 * the inner one of a tunnel packet (the stack fills an outer UDP checksum
 * itself), an IPv4 header without options or an IPv6 header without
 * extension headers.
 *
 * Called for a CHECKSUM_PARTIAL packet. Returns the offset of that IP
 * header, or -1.
 */
static int opip_hw_csum_off(struct sk_buff *skb)
{
	unsigned int start, nh;
	u8 proto;

	start = skb_checksum_start_offset(skb);
	nh = skb->encapsulation ? skb_inner_network_offset(skb) :
				  skb_network_offset(skb);
	if (nh > OPIP_TX_MAX_IPOFF ||
	    nh + sizeof(struct iphdr) > skb_headlen(skb))
		return -1;

	switch (skb->data[nh] >> 4) {
	case 4: {
		const struct iphdr *iph =
			(const struct iphdr *)(skb->data + nh);

		if (iph->ihl != 5 || start != nh + sizeof(struct iphdr) ||
		    (iph->frag_off != 0 && iph->frag_off != htons(IP_DF)))
			return -1;
		proto = iph->protocol;
		break;
	}
	case 6: {
		const struct ipv6hdr *ip6h =
			(const struct ipv6hdr *)(skb->data + nh);

		if (start != nh + sizeof(struct ipv6hdr))
			return -1;
		proto = ip6h->nexthdr;
		break;
	}
	default:
		return -1;
	}

	switch (proto) {
	case IPPROTO_TCP:
		return skb->csum_offset == offsetof(struct tcphdr, check) ?
		       nh : -1;
	case IPPROTO_UDP:
		return skb->csum_offset == offsetof(struct udphdr, check) ?
		       nh : -1;
	default:
		return -1;
	}
}

/**
 * opip_features_check - leave checksums the PKO cannot insert to the stack
 * @skb:      The packet
 * @dev:      Port
 * @features: Features the stack offers for @skb
 *
 * Return: @features less what vlan_features_check() takes away, and less
 * the checksum offloads if the PKO cannot insert @skb's checksum
 */
netdev_features_t opip_features_check(struct sk_buff *skb,
				      struct net_device *dev,
				      netdev_features_t features)
{
	features = vlan_features_check(skb, features);
	if (skb->ip_summed == CHECKSUM_PARTIAL && opip_hw_csum_off(skb) < 0)
		features &= ~NETIF_F_CSUM_MASK;
	return features;
}

/* Segments of one skb, ignoring its frag_list */
static unsigned int opip_tx_skb_segs(const struct sk_buff *skb)
{
	return !!skb_headlen(skb) + skb_shinfo(skb)->nr_frags;
}

/*
 * Number of gather list entries a packet needs, or more than
 * OPIP_TX_MAX_SEGS if it has to be linearized. The frag_list is only
 * followed one level deep.
 */
static unsigned int opip_tx_count_segs(struct sk_buff *skb)
{
	unsigned int segs = opip_tx_skb_segs(skb);
	struct sk_buff *frag;

	skb_walk_frags(skb, frag) {
		if (skb_has_frag_list(frag))
			return OPIP_TX_MAX_SEGS + 1;
		segs += opip_tx_skb_segs(frag);
	}
	return segs;
}

/* Add the head and the page fragments of one skb to a gather list */
static unsigned int opip_tx_map_skb(const struct sk_buff *skb, u64 *list,
				    unsigned int n)
{
	int i;

	if (skb_headlen(skb))
		list[n++] = opip_pko_buf(skb->data, skb_headlen(skb));
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		const skb_frag_t *f = &skb_shinfo(skb)->frags[i];

		list[n++] = opip_pko_buf(skb_frag_address(f), skb_frag_size(f));
	}
	return n;
}

/*
 * The gather list of a fragmented packet, in a buffer from the WQE pool
 * that the completion frees; @cmd and @buf become the command for it.
 *
 * Return: false without a buffer for the list
 */
static bool opip_tx_map_frags(struct sk_buff *skb, u64 *cmd, u64 *buf)
{
	u64 *list = opip_fpa_alloc(OPIP_POOL_WQE);
	struct sk_buff *frag;
	unsigned int n;

	if (unlikely(!list))
		return false;
	n = opip_tx_map_skb(skb, list, 0);
	skb_walk_frags(skb, frag)
		n = opip_tx_map_skb(frag, list, n);
	*opip_tx_skb_list(skb) = list;
	*cmd = u64_replace_bits(*cmd, n, OPIP_PKO_CMD_SEGS) |
	       OPIP_PKO_CMD_GATHER;
	*buf = opip_buf_build(opip_phys(list), n, 0, 0, false);
	return true;
}

/*
 * Write the command of a packet and put the packet into the ring of its
 * queue; @len is what BQL is told. Called under the transmit queue lock.
 *
 * Return: false if the command was not written, the caller frees the
 * packet
 */
static bool opip_tx_write(struct net_device *dev, struct opip_txq *q,
			  unsigned int qid, unsigned long entry,
			  unsigned int len, u64 cmd, u64 buf)
{
	struct sk_buff *skb = opip_tx_entry_skb(entry);

	/* Cannot happen: the queue is stopped long before the ring is full */
	if (WARN_ON_ONCE(q->tail - READ_ONCE(q->head) >= OPIP_TXQ_RING))
		return false;

	if (skb_is_nonlinear(skb)) {
		if (unlikely(!opip_tx_map_frags(skb, &cmd, &buf))) {
			WRITE_ONCE(q->no_wqe, q->no_wqe + 1);
			return false;
		}
		entry |= OPIP_TX_GATHER;
	}
	if (unlikely(!opip_pko_write(q, cmd, buf))) {
		/* No command chunk for the queue: the pool is empty */
		WRITE_ONCE(q->no_chunk, q->no_chunk + 1);
		if (entry & OPIP_TX_GATHER)
			opip_fpa_free(*opip_tx_skb_list(skb), OPIP_POOL_WQE, 0);
		return false;
	}

	/* Once the hardware has it, the completion may run on another CPU */
	q->ring[q->tail & (OPIP_TXQ_RING - 1)] = entry;
	WRITE_ONCE(q->tail, q->tail + 1);
	netdev_tx_sent_queue(netdev_get_tx_queue(dev, qid), len);
	q->doorbell += 2;
	q->written++;
	return true;
}

/* Drop a packet whose command was not written */
static noinline __cold void opip_tx_drop(struct net_device *dev,
					 struct sk_buff *skb)
{
	dev_core_stats_tx_dropped_inc(dev);
	dev_kfree_skb_any(skb);
}

/**
 * opip_xmit - transmit a packet
 * @skb: Packet to send
 * @dev: Port
 *
 * Return: NETDEV_TX_OK, always
 */
netdev_tx_t opip_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int qid = skb_get_queue_mapping(skb);
	struct opip_port *p = opip_port(dev);
	struct netdev_queue *txq;
	struct opip_txq *q;
	unsigned int len;
	u64 cmd, buf;
	int hw_csum;

	txq = netdev_get_tx_queue(dev, qid);
	q = &p->txq[qid];

	/* The gather list must fit its buffer */
	if (unlikely(skb_is_nonlinear(skb) &&
		     opip_tx_count_segs(skb) > OPIP_TX_MAX_SEGS)) {
		if (__skb_linearize(skb))
			goto drop;
		WRITE_ONCE(q->linearized, q->linearized + 1);
	}

	/*
	 * opip_features_check() has left the checksums the hardware cannot
	 * insert to the stack on every path through validate_xmit_skb(): no
	 * second parse here. What bypasses it (pktgen) still has to fit
	 * IPOFFP1.
	 */
	hw_csum = -1;
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		hw_csum = skb->encapsulation ? skb_inner_network_offset(skb) :
					       skb_network_offset(skb);
		if (unlikely(hw_csum > OPIP_TX_MAX_IPOFF)) {
			if (skb_checksum_help(skb))
				goto drop;
			hw_csum = -1;
		}
		DEBUG_NET_WARN_ON_ONCE(hw_csum >= 0 &&
				       opip_hw_csum_off(skb) != hw_csum);
	}

	/*
	 * A frame over the port's limit is dropped: a tc or BPF redirect
	 * does not check the MTU, and the PKO's lengths are 16 bits
	 */
	if (unlikely(skb->len > OPIP_MAX_FRAME))
		goto drop;

	/* The packets sent a batch ago are out: make room before adding */
	if (q->written >= OPIP_TX_COMPLETE_BATCH && !netpoll_tx_running(dev))
		opip_tx_kick(q);

	/*
	 * Before the command is written: a completion may free the skb then.
	 * A forwarded skb has no socket and cannot ask for a timestamp;
	 * testing skb->sk first avoids reading skb_shinfo()->tx_flags, a
	 * cache miss per forwarded packet here (the skb waits in the qdisc
	 * after the core's read).
	 */
	if (skb->sk)
		skb_tx_timestamp(skb);
	len = skb->len;
	cmd = opip_pko_cmd(q, len);
	if (hw_csum >= 0)
		cmd |= FIELD_PREP(OPIP_PKO_CMD_IPOFFP1, hw_csum + 1);
	buf = opip_pko_buf(skb->data, len);
	if (!opip_tx_write(dev, q, qid, (unsigned long)skb, len, cmd, buf))
		opip_tx_drop(dev, skb);
	opip_tx_maybe_stop(txq, q);

	if (!netdev_xmit_more() || netif_xmit_stopped(txq)) {
		opip_tx_doorbell(p, q);
		opip_tx_arm(q);
	}
	return NETDEV_TX_OK;

drop:
	opip_tx_drop(dev, skb);
	/* The batch's earlier packets still need their timer */
	if (!netdev_xmit_more()) {
		opip_tx_doorbell(p, q);
		opip_tx_arm(q);
	}
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------ */
/* Drain */

/*
 * Complete what the PKO has sent on all queues of a device; returns what
 * is still in flight. The queues are stopped or the device closed: the
 * lock is taken against the NAPIs.
 */
static unsigned int opip_tx_complete_dev(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int qid, inflight = 0;

	for (qid = 0; qid < dev->real_num_tx_queues; qid++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, qid);
		struct opip_txq *q = &p->txq[qid];

		__netif_tx_lock_bh(txq);
		opip_tx_complete(dev, qid, q, 0);
		inflight += opip_txq_inflight(q);
		__netif_tx_unlock_bh(txq);
	}
	return inflight;
}

/**
 * opip_tx_drain - wait for the packets in flight on a stopped device
 * @dev: Device whose transmit queues the stack has stopped
 *
 * The skbs and the device must stay until the PKO has sent the packets.
 * The MAC must still be enabled for the PKO to get rid of them. All
 * queues share one time budget, enough for full queues on a 10 Mbit/s
 * link. Only a fully drained device has its BQL state reset; what a drain
 * that gave up leaves waits for opip_tx_resume() or the teardown.
 *
 * Return: true if no packet is in flight any more
 */
bool opip_tx_drain(struct net_device *dev)
{
	unsigned int inflight, qid;

	if (read_poll_timeout(opip_tx_complete_dev, inflight, !inflight,
			      USEC_PER_MSEC, 2 * USEC_PER_SEC, false, dev)) {
		netdev_warn(dev, "%u transmitted packets not completed\n",
			    inflight);
		return false;
	}
	for (qid = 0; qid < dev->real_num_tx_queues; qid++) {
		opip_txq_exit(&opip_port(dev)->txq[qid]);
		netdev_tx_reset_subqueue(dev, qid);
	}
	return true;
}

/**
 * opip_tx_resume - start the completions of a device being opened
 * @dev: Device being opened
 *
 * Enable the queues' NAPIs. The timers of a closed device stop rearming;
 * an open starts them again for the packets a drain that gave up left in
 * flight, which the stack may not send behind.
 */
void opip_tx_resume(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int qid;

	for (qid = 0; qid < dev->real_num_tx_queues; qid++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, qid);

		napi_enable(&p->txq[qid].napi);
		__netif_tx_lock_bh(txq);
		opip_tx_arm(&p->txq[qid]);
		__netif_tx_unlock_bh(txq);
	}
}

/**
 * opip_tx_stop - stop the completions of a device being closed
 * @dev: Device that no longer transmits, drained or not
 *
 * Disable the queues' NAPIs: none runs on a closed device. A timer still
 * pending schedules nothing; opip_tx_resume() takes over what is left.
 */
void opip_tx_stop(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int qid;

	for (qid = 0; qid < dev->real_num_tx_queues; qid++)
		napi_disable(&p->txq[qid].napi);
}
