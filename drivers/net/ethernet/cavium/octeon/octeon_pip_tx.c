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
 * packets in the order of their commands. The counter is read where the
 * packets come from, and always in a NAPI poll, under the budget the core
 * gave it: in a receive poll once in 16 received packets for the queues
 * with skbs in flight, at its end for the queues it sent on, and in xmit
 * from the poll for the queue it is about to use (once in 8 packets, not
 * for every one). Every queue has a NAPI of its own on the port for the
 * rest: a transmit outside of a receive poll schedules it once in 8
 * packets, unless a poll has the queue on its list, and so does a timer
 * per queue for whatever is left when nothing else comes along. That NAPI
 * waits for the one its CPU is in (the RPS backlog, another driver's), so
 * from half full on a transmit outside of a receive poll reads the counter
 * itself. There is no completion interrupt: a packet that asks for one is
 * a three word command.
 *
 * Derived in part from the staging octeon driver (ethernet-tx.c: the
 * short frame padding of CN3XXX),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
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
#include <linux/preempt.h>
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

/*
 * Packets in flight from which a transmit outside of a receive poll
 * completes the queue itself instead of scheduling its NAPI, which runs
 * only once the NAPI its CPU is in returns, see opip_tx_batch()
 */
#define OPIP_TX_COMPLETE_THRESH	(OPIP_TXQ_DEPTH / 2)

/*
 * Packets written between two reads of the counter by the transmit paths.
 * In a receive poll this interleaves the frees with the skb builds, so the
 * per-CPU skb cache recycles them instead of overflowing to the slab.
 */
#define OPIP_TX_COMPLETE_BATCH	8

/* Command words a receive poll writes before it rings the doorbell */
#define OPIP_TX_POLL_DOORBELL	24

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

/* The list of the receive poll running on a CPU, NULL outside of one */
static DEFINE_PER_CPU(struct opip_tx_poll *, opip_tx_poll_cur);

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
 * packets, the poll end once per poll.
 *
 * @budget is the budget the core passed to the NAPI poll this runs in: a
 * receive poll (its ticks, its end, what it transmits) or the queue's own
 * NAPI, or any non-zero one for a transmit in a softirq outside of a
 * receive poll. The skbs go into the per-CPU NAPI cache they were built
 * from. A drain from process context, the watchdog and a transmit outside
 * of a softirq pass 0 and free to the slab. netpoll transmits with
 * interrupts off and bottom halves on, so it leaves the completion to the
 * NAPI and to opip_tx_netpoll(). The stack delivers the last packets of
 * every poll (its GRO batch) after the poll has returned, so a third of a
 * forwarding load's packets are sent from outside the poll on the poll's
 * CPU; freeing those to the slab starved the cache, and the next poll
 * built its skbs cold from the slab. A transmit outside a receive poll
 * leaves them to the next poll that has the queue on its list, or
 * schedules the queue's NAPI, see opip_tx_kick(), until the ring is half
 * full, see opip_tx_batch().
 *
 * Return: the number of packets completed
 */
static unsigned int opip_tx_complete(struct net_device *dev, unsigned int qid,
				     struct opip_txq *q, int budget)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, qid);
	unsigned int head = q->head, n, pkts;
	unsigned int bytes = 0, skbs = 0;
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
			skbs++;
			napi_consume_skb(skb, budget);
		}
	}
	WRITE_ONCE(q->skbs, q->skbs - skbs);
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
 * A transmit outside of a receive poll has written a batch: the poll that
 * has the queue on its list completes it soon (see opip_tx_poll_end()),
 * otherwise the queue's NAPI does, on this CPU, once bottom halves are
 * back on. Under the queue lock. The batch starts over: with the NAPI
 * still pending the next one has it polled once more.
 */
static void opip_tx_kick(struct opip_txq *q)
{
	q->written = 0;
	if (!q->poll_dirty)
		napi_schedule(&q->napi);
}

/*
 * A transmit has written OPIP_TX_COMPLETE_BATCH packets: complete them in
 * a receive poll, under the poll's budget. Outside of one the queue's NAPI
 * does, but the RPS backlog's poll of 64 GRO super-skbs fills the ring
 * several times over before the NAPI gets to run, and the PKO idles on a
 * ring of sent packets meanwhile: from OPIP_TX_COMPLETE_THRESH on the
 * transmit completes them itself, with a NAPI budget in a softirq, to the
 * slab from process context. Below it the FAU read is not worth it. Under
 * the queue lock, never for netpoll.
 */
static __always_inline void opip_tx_batch(struct net_device *dev,
					  unsigned int qid, struct opip_txq *q,
					  struct opip_tx_poll *poll)
{
	if (poll)
		opip_tx_complete(dev, qid, q, poll->budget);
	else if (opip_txq_inflight(q) >= OPIP_TX_COMPLETE_THRESH)
		opip_tx_complete(dev, qid, q,
				 in_serving_softirq() ? NAPI_POLL_WEIGHT : 0);
	else
		opip_tx_kick(q);
}

/*
 * Free the page pool skbs opip_tx_netpoll() took off the ring with
 * interrupts off. Under the queue lock, with interrupts on: the NAPI,
 * or a purge.
 */
static void opip_tx_netpoll_flush(struct opip_txq *q)
{
	struct sk_buff *skb;

	while ((skb = q->netpoll_skbs)) {
		q->netpoll_skbs = skb->next;
		skb->next = NULL;
		dev_consume_skb_any(skb);
	}
}

/*
 * The queue's NAPI: complete what the PKO has sent and leave the rest to
 * the timer. What it completes is mostly the tail of a receive poll on
 * this CPU; freed to the slab, that made the next poll refill the per-CPU
 * NAPI skb cache from the slab too, so it is a NAPI and frees under its
 * budget. netpoll's budget of
 * 0 allows no page pool skb to be freed: the NAPI stays scheduled for
 * the next real poll, and opip_tx_netpoll() makes netpoll's room.
 */
static int opip_tx_napi_poll(struct napi_struct *napi, int budget)
{
	struct opip_txq *q = container_of(napi, struct opip_txq, napi);
	struct netdev_queue *txq = netdev_get_tx_queue(q->dev, q->qid);

	if (!budget)
		return 0;

	__netif_tx_lock(txq, smp_processor_id());
	if (unlikely(q->netpoll_skbs))
		opip_tx_netpoll_flush(q);
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
	struct netdev_queue *txq;

	if (!q->dev)
		return;
	hrtimer_cancel(&q->timer);
	/*
	 * No timer any more: what netpoll left is freed here, under the
	 * queue lock, as the queue's NAPI may still run (a close drains
	 * before it disables the NAPIs)
	 */
	txq = netdev_get_tx_queue(q->dev, q->qid);
	__netif_tx_lock_bh(txq);
	opip_tx_netpoll_flush(q);
	__netif_tx_unlock_bh(txq);
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
 * of the queue that no count shows, see opip_tx_timeout().
 *
 * Return: how many packets were freed
 */
unsigned int opip_txq_purge(struct opip_txq *q)
{
	unsigned int head, freed = 0, skbs = 0;

	for (head = q->head; head != q->tail; head++, freed++) {
		unsigned long entry = q->ring[head & (OPIP_TXQ_RING - 1)];
		struct sk_buff *skb = opip_tx_entry_skb(entry);

		if (skb) {
			if (entry & OPIP_TX_GATHER)
				opip_fpa_free(*opip_tx_skb_list(skb),
					      OPIP_POOL_WQE, 0);
			skbs++;
			dev_kfree_skb_any(skb);
		}
	}
	WRITE_ONCE(q->skbs, q->skbs - skbs);
	WRITE_ONCE(q->head, head);
	opip_tx_netpoll_flush(q);
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

/*
 * Errata GMX-401 of CN3XXX: the GMX hangs on a collision towards the end
 * of a frame under 68 bytes. A GMX port in half duplex has p->pad_runts
 * set (with the link, see opip_link_change()) and pads frames under 64
 * bytes: an skb with __skb_put_padto(), an XDP packet in its buffer. The
 * tests sit behind a static key that probe enables on CN3XXX only: on the
 * other chips each send path runs a nop for it.
 */
DEFINE_STATIC_KEY_FALSE(opip_gmx401);

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
	if (skb)
		WRITE_ONCE(q->skbs, q->skbs + 1);
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

/* Have the receive poll of this CPU ring the doorbell of a queue */
static bool opip_tx_poll_mark(struct opip_tx_poll *poll, struct opip_txq *q,
			      struct net_device *dev, unsigned int qid)
{
	if (q->poll_dirty)
		return true;
	if (poll->num == ARRAY_SIZE(poll->dirty))
		return false;
	poll->dirty[poll->num].dev = dev;
	poll->dirty[poll->num].qid = qid;
	poll->num++;
	q->poll_dirty = true;
	return true;
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
	struct opip_tx_poll *poll;
	struct netdev_queue *txq;
	struct opip_txq *q;
	unsigned int len;
	u64 cmd, buf;
	int hw_csum;

	txq = netdev_get_tx_queue(dev, qid);
	q = &p->txq[qid];

	/*
	 * netpoll may send from an interrupt in the middle of this CPU's
	 * receive poll: not onto the poll's list. A preempt count test, the
	 * CP0 Status read of irqs_disabled() costs the forwarding rate;
	 * without netpoll nothing sends from an interrupt. Bottom halves
	 * are off (interrupts for netpoll): no preemption to guard the
	 * per-CPU read against.
	 */
	poll = IS_ENABLED(CONFIG_NETPOLL) && (in_hardirq() || in_nmi()) ?
	       NULL : __this_cpu_read(opip_tx_poll_cur);

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
	 * A frame over the port's limit is dropped (a tc or BPF redirect
	 * does not check the MTU, and the PKO's lengths are 16 bits); a runt
	 * is padded on a CN3XXX half duplex port (errata GMX-401; unclones
	 * and linearizes as needed, the caller frees on error).
	 */
	if (unlikely(skb->len > OPIP_MAX_FRAME))
		goto drop;
	if (static_branch_unlikely(&opip_gmx401) && skb->len < 64 &&
	    READ_ONCE(p->pad_runts) && __skb_put_padto(skb, 64, false))
		goto drop;

	/* The packets sent a batch ago are out: make room before adding */
	if (q->written >= OPIP_TX_COMPLETE_BATCH && !netpoll_tx_running(dev))
		opip_tx_batch(dev, qid, q, poll);

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

	/*
	 * From a receive poll the queue goes on the poll's list, whose end
	 * completes it and arms the timer. The doorbell rings at the end of
	 * the stack's batch as outside of a poll, or once the poll's words on
	 * the queue reach OPIP_TX_POLL_DOORBELL or the queue stops.
	 */
	if (poll && opip_tx_poll_mark(poll, q, dev, qid)) {
		if (!netdev_xmit_more() ||
		    q->doorbell >= OPIP_TX_POLL_DOORBELL ||
		    netif_xmit_stopped(txq))
			opip_tx_doorbell(p, q);
		return NETDEV_TX_OK;
	}
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

/**
 * opip_tx_poll_begin - the start of a receive poll's transmit side
 * @poll:   The poll's list
 * @budget: The budget the core passed to the poll, not 0
 *
 * A receive poll starts on this CPU: the queues it sends on go on its
 * list, see opip_tx_poll_end(), and what it completes is freed under
 * @budget.
 */
void opip_tx_poll_begin(struct opip_tx_poll *poll, int budget)
{
	poll->budget = budget;
	this_cpu_write(opip_tx_poll_cur, poll);
}

/**
 * opip_tx_poll_tick - complete in flight packets in the middle of a poll
 * @poll: The poll's list
 *
 * Called every few received packets. The packets a receive poll forwards
 * leave the PKO after the batch in xmit has looked for them, and the ones
 * the stack sends after the poll has returned (the GRO tail, a tunnel's
 * workers) leave while the next poll receives: nothing completed either
 * before a poll's end, in one lump, after all of the poll's skb builds
 * had drawn from the per-CPU NAPI skb cache, which the lump then pushed
 * over its edge, so a part of every poll's skbs went through the slab and
 * came back cold (the stateful IPv6 forwarding rate). Completing them
 * here, spread over the builds, keeps the cache in the middle. A queue is
 * read while it has skbs in flight. The queues are the ones the polls
 * kept on the list, see opip_tx_poll_end().
 */
void opip_tx_poll_tick(struct opip_tx_poll *poll)
{
	unsigned int cpu = smp_processor_id();
	unsigned int i;

	for (i = 0; i < poll->num; i++) {
		struct net_device *dev = poll->dirty[i].dev;
		unsigned int qid = poll->dirty[i].qid;
		struct netdev_queue *txq;
		struct opip_txq *q;

		q = &opip_port(dev)->txq[qid];
		if (!READ_ONCE(q->skbs))
			continue;
		txq = netdev_get_tx_queue(dev, qid);
		__netif_tx_lock(txq, cpu);
		opip_tx_complete(dev, qid, q, poll->budget);
		__netif_tx_unlock(txq);
	}
}

/**
 * opip_tx_poll_end - the end of a receive poll's transmit side
 * @poll: The poll's list
 * @idle: The poll found the group empty; otherwise it is polled again at once
 *
 * Ring the doorbells and complete the queues the poll sent on. A queue
 * with packets still in flight stays on the group's list as long as the
 * polls go on, so that the next poll's end completes it whether it sends
 * on it or not; only when the poll goes idle, and the next one may be
 * far, the queue's timer and NAPI take over. Under load that is one timer
 * less per queue firing every 250 us; arming it only at idle without
 * keeping the list left queues without completion for seconds.
 */
void opip_tx_poll_end(struct opip_tx_poll *poll, bool idle)
{
	unsigned int i, kept = 0;

	this_cpu_write(opip_tx_poll_cur, NULL);
	for (i = 0; i < poll->num; i++) {
		struct net_device *dev = poll->dirty[i].dev;
		unsigned int qid = poll->dirty[i].qid;
		struct opip_port *p = opip_port(dev);
		struct netdev_queue *txq;
		struct opip_txq *q;
		bool inflight;

		txq = netdev_get_tx_queue(dev, qid);
		q = &p->txq[qid];

		__netif_tx_lock(txq, smp_processor_id());
		opip_tx_doorbell(p, q);
		opip_tx_complete(dev, qid, q, poll->budget);
		opip_tx_maybe_stop(txq, q);
		inflight = opip_txq_inflight(q);
		if (idle || !inflight) {
			q->poll_dirty = false;
			if (inflight)
				opip_tx_arm(q);
		} else {
			poll->dirty[kept++] = poll->dirty[i];
		}
		__netif_tx_unlock(txq);
	}
	poll->num = kept;
}

/* ------------------------------------------------------------------ */
/* Drain and watchdog */

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

#ifdef CONFIG_NET_POLL_CONTROLLER
/**
 * opip_tx_netpoll - complete the sent skbs of a device, for netpoll
 * @dev: Port
 *
 * The ports' ndo_poll_controller: netpoll asks for this when a queue is
 * stopped or busy. Room on the port's queues is what it needs; the
 * receive polls are not scheduled from here (their NAPI instances hang
 * off a dummy device, none could run before netpoll returns, and their
 * interrupts schedule them afterwards; nor are the queues' NAPIs, which
 * netpoll polls with a budget of 0). With interrupts off the timer cannot
 * run, and in a panic nothing else does: without this every later message
 * is lost. The skbs go to dev_consume_skb_any(), but a page pool skb (a
 * forwarded one) that netpoll would free right away from its completion
 * queue, with interrupts off, which the pool's return does not allow,
 * comes off the ring onto the queue's list for the NAPI (a panic never
 * frees it, which is fine).
 */
void opip_tx_netpoll(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int qid;

	for (qid = 0; qid < dev->real_num_tx_queues; qid++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, qid);
		unsigned int head, n, pkts = 0, bytes = 0, skbs = 0;
		struct opip_txq *q = &p->txq[qid];

		if (!__netif_tx_trylock(txq))
			continue;
		q->written = 0;
		head = q->head;
		n = min(-opip_fau_rd32(q->fau) - head, q->tail - head);
		while (n--) {
			unsigned long entry =
				q->ring[head & (OPIP_TXQ_RING - 1)];
			struct sk_buff *skb = opip_tx_entry_skb(entry);

			if (skb) {
				if (entry & OPIP_TX_GATHER)
					opip_fpa_free(*opip_tx_skb_list(skb),
						      OPIP_POOL_WQE, 0);
				bytes += skb->len;
				skbs++;
				if (skb->pp_recycle) {
					skb->next = q->netpoll_skbs;
					q->netpoll_skbs = skb;
				} else {
					dev_consume_skb_any(skb);
				}
			}
			head++;
			pkts++;
		}
		WRITE_ONCE(q->skbs, q->skbs - skbs);
		WRITE_ONCE(q->head, head);
		netif_txq_completed_wake(txq, pkts, bytes,
					 opip_txq_free_slots(q),
					 OPIP_TX_WAKE_THRESH);
		/* Nothing in flight arms the timer: the list needs it */
		if (q->netpoll_skbs && !hrtimer_is_queued(&q->timer))
			hrtimer_start(&q->timer, OPIP_TX_TIMER_NS,
				      HRTIMER_MODE_REL_PINNED_SOFT);
		__netif_tx_unlock(txq);
	}
}
#endif

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

/**
 * opip_tx_timeout - the stack's watchdog found a queue stopped too long
 * @dev:     Device
 * @txqueue: The transmit queue
 *
 * Diagnostics, no recovery. What the PKO has sent is completed first, so
 * that a queue only short of a completion is woken and the rest is what
 * the PKO really holds. A queue stays stopped while the PKO does not send
 * its packets: a link partner that pauses the port, a MAC that does not
 * send, or a PKO error, which stops the port for good. Nothing can be
 * taken back from the PKO while it runs: with no command word of the
 * queue left in its storage, the port's engine still holds the queue's
 * newest packet, counted nowhere, and freeing it would leave the queue's
 * FAU counter one ahead for good, a later completion freeing a packet the
 * PKO has not sent. The queue waits for the PKO; its completions wake it.
 * Runs in the watchdog's timer.
 */
void opip_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, txqueue);
	unsigned int inflight, pending, doorbell;
	struct opip_port *p = opip_port(dev);
	struct opip_txq *q;
	u64 error;

	q = &p->txq[txqueue];

	__netif_tx_lock(txq, smp_processor_id());
	opip_tx_complete(dev, txqueue, q, 0);
	inflight = opip_txq_inflight(q);
	/* Written, not announced yet: a receive poll on another CPU */
	doorbell = q->doorbell;
	__netif_tx_unlock(txq);
	pending = opip_pko_queue_pending(p->opip, q->queue);

	netdev_warn(dev, "transmit queue %u stopped: %u packets in flight, the PKO holds %u command words of queue %d\n",
		    txqueue, inflight, pending, q->queue);
	if (inflight && !pending && !doorbell)
		net_warn_ratelimited("%s: transmit queue %u: the engine holds its last packet; waiting\n",
				     netdev_name(dev), txqueue);

	/*
	 * A PKO error stops the port for good; name it. The bits are
	 * write-one-to-clear, so each error is reported once.
	 */
	error = opip_rd(OPIP_PKO_REG_ERROR);
	if (error) {
		opip_wr(OPIP_PKO_REG_ERROR, error);
		netdev_err(dev, "PKO error%s%s%s%s (PKO_REG_ERROR %#llx)\n",
			   error & OPIP_PKO_REG_ERROR_PARITY ?
			   " parity" : "",
			   error & OPIP_PKO_REG_ERROR_DOORBELL ?
			   " bad-doorbell" : "",
			   error & OPIP_PKO_REG_ERROR_CURRZERO ?
			   " zero-buffer-pointer" : "",
			   error & OPIP_PKO_REG_ERROR_LOOPBACK ?
			   " loopback" : "",
			   (unsigned long long)error);
	}
}
