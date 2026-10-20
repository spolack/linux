// SPDX-License-Identifier: GPL-2.0
/*
 * The receive side of octeon_pip: the POW groups, one per CPU, each with
 * its interrupt, NAPI instance and page pool. A group delivers the
 * received packets the PIP tagged for it.
 */

#include <linux/atomic.h>
#include <linux/cpumask.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/prefetch.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <net/page_pool/helpers.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_ipd.h"
#include "octeon_pip_pow.h"
#include "octeon_pip_rx.h"
#include "octeon_pip_tx.h"

/*
 * Asynchronous work requests: the POW answers into the core's scratch
 * memory, which needs CVMSEG. Without it the requests are loads.
 */
#if defined(CONFIG_CAVIUM_OCTEON_CVMSEG_SIZE) && \
	CONFIG_CAVIUM_OCTEON_CVMSEG_SIZE > 0
#define OPIP_ASYNC_POW		true
#else
#define OPIP_ASYNC_POW		false
#endif

/* Receives between two transmit completions in a poll, opip_tx_poll_tick() */
#define OPIP_TX_POLL_TICK	16

/* The refill worker's retry after an allocation failure of an idle poll */
#define OPIP_REFILL_RETRY	msecs_to_jiffies(20)

static irqreturn_t opip_group_irq(int irq, void *data)
{
	struct opip_group *group = data;

	if (!test_and_set_bit(OPIP_GROUP_IRQ_DISABLED, &group->state))
		disable_irq_nosync(irq);
	napi_schedule(&group->napi);
	return IRQ_HANDLED;
}

/**
 * opip_rx_free_work - free a received work queue entry and its buffers
 * @wqe: The work queue entry
 *
 * The buffers go back to the pools they came from, except those with the
 * I bit set.
 */
void opip_rx_free_work(struct opip_wqe *wqe)
{
	unsigned int segments = opip_wqe_bufs(wqe);
	u64 seg = wqe->packet_ptr;

	while (segments--) {
		u64 next = opip_buf_next(seg);

		if (unlikely(!(seg & OPIP_BUF_I)))
			opip_fpa_free(opip_virt(opip_buf_start(seg)),
				      FIELD_GET(OPIP_BUF_POOL, seg),
				      OPIP_POOL_PACKET_SIZE / 128);
		seg = next;
	}
	/*
	 * No barrier: only the preamble repair writes a received work
	 * entry, and it orders its writes itself
	 */
	opip_fpa_free_nosync(wqe, OPIP_POOL_WQE, 1);
}

/*
 * A dropped packet, at debug level: the statistics count it, this says
 * why. The rate limit is asked only once the message is enabled.
 */
#define opip_rx_dbg(dev, port, fmt, ...)				\
do {									\
	const struct net_device *__dev = (dev);				\
									\
	net_dbg_ratelimited("%s port %d: " fmt,				\
			    __dev ? netdev_name(__dev) : KBUILD_MODNAME, \
			    port, ##__VA_ARGS__);			\
} while (0)

/*
 * A packet with a receive error. Minimum size frames with a length error
 * pass: some equipment pads to 64 + 4 FCS instead of 60 + 4.
 * A frame longer than the port's jabber limit arrives cut there, with the
 * jabber error (GMX_RX_FRM_CHK[JABBER]).
 *
 * Return: true if the packet was dropped
 */
static noinline __cold bool opip_rx_check_error(struct opip_wqe *wqe,
						struct net_device *dev,
						int port)
{
	unsigned int err = opip_wqe_err_code(wqe);

	if (err == OPIP_RCV_ERR_LENGTH && opip_wqe_len(wqe) <= 64)
		return false;

	opip_rx_dbg(dev, port, "receive error code %u, packet dropped\n", err);
	opip_rx_free_work(wqe);
	return true;
}

/* Copy a packet that spans buffers into an skb */
static void opip_rx_copy_segments(struct opip_wqe *wqe, struct sk_buff *skb)
{
	unsigned int segments = opip_wqe_bufs(wqe);
	unsigned int len = opip_wqe_len(wqe);
	u64 seg = wqe->packet_ptr;

	while (segments--) {
		u64 next = opip_buf_next(seg);
		/*
		 * Errata PKI-100: the segment size of the pointer is wrong,
		 * so it is what is left of the buffer from the data on.
		 */
		unsigned int size = OPIP_POOL_PACKET_SIZE -
				    (opip_buf_addr(seg) - opip_buf_start(seg));

		size = min(size, len);
		skb_put_data(skb, opip_virt(opip_buf_addr(seg)), size);
		len -= size;
		seg = next;
	}
}

/*
 * The zero copy path: a packet that fits in one buffer gets an skb built
 * around it. The buffer is a page (or a slice) of any group's page pool
 * (the packet pool mixes them) and goes back to that pool with its last
 * skb: into its cache when freed in its own group's poll, else its ring.
 * Always inline: with two callers gcc calls it out of line, per packet.
 */
static __always_inline struct sk_buff *opip_rx_build_skb(void *buffer,
							 void *data,
							 unsigned int len)
{
	struct sk_buff *skb = napi_build_skb(buffer, OPIP_RX_FRAG_SIZE);

	if (unlikely(!skb))
		return NULL;
	skb_mark_for_recycle(skb);
	skb_reserve(skb, data - buffer);
	__skb_put(skb, len);
	return skb;
}

/* A packet the PIP left in the entry, or one over several buffers */
static struct sk_buff *opip_rx_copy_skb(struct opip_group *group,
					struct opip_wqe *wqe)
{
	unsigned int len = opip_wqe_len(wqe);
	struct sk_buff *skb;

	skb = napi_alloc_skb(&group->napi, len);
	if (!skb)
		return NULL;
	if (unlikely(!opip_wqe_bufs(wqe))) {
		u8 *ptr = wqe->packet_data;

		/* The beginning of an IP packet moves in the entry */
		if (likely(!(wqe->word2 & OPIP_WQE2_NOT_IP)))
			ptr += wqe->word2 & OPIP_WQE2_IS_V6 ? 2 : 6;
		skb_put_data(skb, ptr, len);
	} else {
		opip_rx_copy_segments(wqe, skb);
	}
	return skb;
}

/* Always inline: with two callers gcc calls it out of line, per packet */
static __always_inline void opip_rx_deliver(struct opip_group *group,
					    struct net_device *dev,
					    struct opip_wqe *wqe,
					    struct sk_buff *skb)
{
	/* Read once per packet, for both tests */
	netdev_features_t features = dev->features;

	/* A port has a receive queue per group, as XDP has */
	skb_record_rx_queue(skb, group->group);
	skb->protocol = eth_type_trans(skb, dev);
	/*
	 * The PIP's flow tag of an IP packet is the stack's flow hash: GRO
	 * buckets by it, the queueing disciplines classify by it, and a
	 * tunnel keeps it for the outer packet. Its low bits (up to four)
	 * picked the group, so they are the same for all of a NAPI's
	 * packets: an xorshift, which loses nothing, fills them for GRO's
	 * buckets (hash & 7) and leaves the top bits that RPS scales by.
	 */
	if (likely(features & NETIF_F_RXHASH) &&
	    likely(!(wqe->word2 & OPIP_WQE2_NOT_IP))) {
		u32 tag = opip_wqe_tag(wqe);

		skb_set_hash(skb, tag ^ (tag >> 4),
			     wqe->word2 & OPIP_WQE2_TCP_OR_UDP ?
			     PKT_HASH_TYPE_L4 : PKT_HASH_TYPE_L3);
	}
	/* The PIP checks no L4 checksum of a fragment */
	if (unlikely((wqe->word2 & (OPIP_WQE2_NOT_IP | OPIP_WQE2_IP_EXC |
				    OPIP_WQE2_IS_FRAG | OPIP_WQE2_L4_ERROR)) ||
		     !(wqe->word2 & OPIP_WQE2_TCP_OR_UDP) ||
		     !(features & NETIF_F_RXCSUM)))
		skb->ip_summed = CHECKSUM_NONE;
	else
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	napi_gro_receive(&group->napi, skb);
}

/* A packet not in one buffer, or in the entry alone: it is copied */
static noinline void opip_rx_slow_packet(struct opip_group *group,
					 struct opip_wqe *wqe,
					 struct net_device *dev)
{
	struct sk_buff *skb;

	skb = opip_rx_copy_skb(group, wqe);
	if (skb)
		opip_rx_deliver(group, dev, wqe, skb);
	opip_rx_free_work(wqe);
}

/* A packet for a port without a device, or for a device that is not open */
static noinline __cold void opip_rx_drop_closed(struct opip_wqe *wqe,
						struct net_device *dev,
						int port)
{
	if (dev)
		dev_core_stats_rx_dropped_inc(dev);
	else
		opip_rx_dbg(dev, port, "not controlled by Linux, packet dropped\n");
	opip_rx_free_work(wqe);
}

/* One received packet: receive errors, the skb and its delivery */
static void opip_rx_packet(struct opip_group *group, struct opip_wqe *wqe,
			   struct net_device *dev, int port)
{
	bool skb_in_hw = opip_wqe_bufs(wqe) == 1;
	void *buffer = NULL;
	struct sk_buff *skb;
	unsigned int len;
	void *data;

	if (likely(skb_in_hw))
		buffer = opip_virt(opip_buf_start(wqe->packet_ptr));

	if (unlikely(wqe->word2 & OPIP_WQE2_RCV_ERROR) &&
	    opip_rx_check_error(wqe, dev, port))
		return;

	/*
	 * A device being closed (stopped already, IFF_UP not cleared yet)
	 * delivers nothing
	 */
	if (unlikely(!dev || !netif_running(dev))) {
		opip_rx_drop_closed(wqe, dev, port);
		return;
	}

	data = opip_virt(opip_buf_addr(wqe->packet_ptr));
	len = opip_wqe_len(wqe);

	if (likely(skb_in_hw)) {
		skb = opip_rx_build_skb(buffer, data, len);
	} else {
		opip_rx_slow_packet(group, wqe, dev);
		return;
	}

	if (unlikely(!skb)) {
		opip_rx_free_work(wqe);
		return;
	}

	opip_rx_deliver(group, dev, wqe, skb);

	/* The entry goes back; the skb's buffer is replaced later */
	WRITE_ONCE(group->to_refill, group->to_refill + 1);
	opip_fpa_free_nosync(wqe, OPIP_POOL_WQE, 1);
}

/*
 * The input port: a port kind on the chips with them (@pknd, CN68XX), the
 * IPD port elsewhere. The poll knows the chip once, not per packet.
 */
static unsigned int opip_rx_port(const struct opip_wqe *wqe, bool pknd)
{
	if (pknd)
		return FIELD_GET(OPIP_WQE0_PKND, wqe->word0);
	return FIELD_GET(OPIP_WQE1_IPPRT, wqe->word1);
}

/*
 * The next work entry, with its cache line on the way. The POW answers a
 * synchronous request right here; with the asynchronous requests the
 * request went out an iteration ago and the answer is in the scratch
 * memory already. Always inline: at -Os gcc calls it out of line.
 */
static __always_inline struct opip_wqe *opip_rx_next_work(void)
{
	struct opip_wqe *wqe;

	if (OPIP_ASYNC_POW)
		wqe = opip_pow_response_async();
	else
		wqe = opip_pow_request_sync(false);
	if (wqe)
		prefetch(wqe);
	return wqe;
}

/*
 * Replace the packet buffers the poll handed to skbs or XDP. Only the poll
 * of the group allocates from its page pool, so this runs in the poll;
 * opip_rx_refill_check() schedules a poll if the pool ran low with the
 * polls idle. Returns what is still owed: an allocation failed.
 */
static unsigned int opip_rx_refill(struct opip_group *group)
{
	unsigned int n = group->to_refill;

	if (!n)
		return 0;
	n -= opip_fpa_fill_packet_pool(group->pp, n, GFP_ATOMIC);
	WRITE_ONCE(group->to_refill, n);
	return n;
}

static int opip_poll(struct opip_group *group, int budget)
{
	/*
	 * Per poll: the chips with port kinds deliver those in place of the
	 * input port, and count per backpressure ID instead
	 */
	const bool pknd = opip_chip->cn68xx;
	const bool bp_count = !pknd;
	struct net_device **ports = group->opip->ports;
	const unsigned int core = opip_core();
	struct opip_wqe *wqe;
	u64 old_mask, old_scratch = 0;
	int bp_port = 0, bp_bufs = 0;
	int rx_count = 0;
	bool requested = false, idle = false;

	prefetch(ports);

	if (OPIP_ASYNC_POW) {
		/* Userspace may use the scratch word: save it */
		opip_iobdma_wait();
		old_scratch = opip_scratch_rd(OPIP_POW_SCRATCH);
	}

	old_mask = opip_pow_select_group(core, group->group);
	opip_tx_poll_begin(&group->tx_poll, budget);
	/*
	 * The loop runs two work entries ahead of the stack: the one in
	 * hand had its cache line fetched while the stack handled the one
	 * before it, and with the asynchronous requests the request for
	 * the one after it is out. An entry is requested only when the
	 * budget has room for it, so that none is taken from the POW and
	 * left behind.
	 *
	 * Why: on CN70XX the first touch of an entry taken and read at once
	 * stalled the poll, and the first touch of the packet data longer.
	 * The prefetches pay only when they fill L1: with
	 * CONFIG_CAVIUM_CN63XXP1 the assembler turns every prefetch into
	 * the L2-only hint 28 and the first touch misses L1 again.
	 */
	if (OPIP_ASYNC_POW)
		opip_pow_request_async(false);
	wqe = opip_rx_next_work();
	if (!wqe) {
		idle = true;
	} else if (OPIP_ASYNC_POW && rx_count + 1 < budget) {
		opip_pow_request_async(false);
		requested = true;
	}

	while (wqe) {
		struct net_device *dev = NULL;
		struct opip_wqe *next = NULL;
		int port;

		if (rx_count + 1 < budget) {
			/* Not requested ahead: the last one of the budget */
			if (OPIP_ASYNC_POW && !requested)
				opip_pow_request_async(false);
			next = opip_rx_next_work();
			requested = false;
			if (!next) {
				idle = true;
			} else if (OPIP_ASYNC_POW && rx_count + 2 < budget) {
				opip_pow_request_async(false);
				requested = true;
			}
		}

		/* The packet data, the stack reads its headers first */
		prefetch(opip_virt(opip_buf_addr(wqe->packet_ptr)));
		rx_count++;

		port = opip_rx_port(wqe, pknd);
		if (likely(port < OPIP_MAX_PORTS)) {
			dev = READ_ONCE(ports[port]);
			prefetch(dev);
		}

		/* One write per run of packets from a port */
		if (bp_count) {
			if (unlikely(port != bp_port)) {
				opip_ipd_bp_sub(bp_port, bp_bufs);
				bp_port = port;
				bp_bufs = 0;
			}
			bp_bufs += opip_wqe_bufs(wqe);
		}

		opip_rx_packet(group, wqe, dev, port);

		/*
		 * Complete what the PKO has sent on the poll's queues now and
		 * then, so the freed skbs refill the cache the builds draw
		 * from, see opip_tx_poll_tick().
		 */
		if (!(rx_count & (OPIP_TX_POLL_TICK - 1)))
			opip_tx_poll_tick(&group->tx_poll);

		/* Its line is in by now: have the next packet's data follow */
		if (next)
			prefetch(opip_virt(opip_buf_addr(next->packet_ptr)));
		wqe = next;
	}
	if (idle)
		opip_pow_group_ack(group->group);
	if (bp_count)
		opip_ipd_bp_sub(bp_port, bp_bufs);
	opip_tx_poll_end(&group->tx_poll, idle);
	opip_pow_restore_groups(core, old_mask);
	if (OPIP_ASYNC_POW)
		opip_scratch_wr(OPIP_POW_SCRATCH, old_scratch);
	/*
	 * An idle poll that could not refill may be the last for long: with
	 * the pool empty nothing comes in to interrupt. Have the worker look
	 * soon, not in up to a second.
	 */
	if (unlikely(opip_rx_refill(group)) && idle &&
	    !READ_ONCE(group->opip->stopping))
		mod_delayed_work(system_percpu_wq, &group->opip->refill_work,
				 OPIP_REFILL_RETRY);

	/* The budget ended the poll: poll again */
	if (!idle)
		return budget;
	return rx_count;
}

static int opip_napi_poll(struct napi_struct *napi, int budget)
{
	struct opip_group *group = container_of(napi, struct opip_group, napi);
	int rx_count;

	/* A budget of 0 (netpoll) allows no receive work */
	if (!budget)
		return 0;

	rx_count = opip_poll(group, budget);

	/*
	 * Only re-enable the interrupt if NAPI is really done: with
	 * napi_defer_hard_irqs or busy polling the core keeps polling and
	 * napi_complete_done() returns false. The state bit pairs each
	 * enable_irq() with the disable_irq_nosync() before it.
	 */
	if (rx_count < budget && napi_complete_done(napi, rx_count) &&
	    test_and_clear_bit(OPIP_GROUP_IRQ_DISABLED, &group->state))
		enable_irq(group->irq);
	return rx_count;
}

static void opip_group_kick(void *data)
{
	struct opip_group *group = data;

	__napi_schedule(&group->napi);
}

/**
 * opip_group_schedule - have a group's poll run, from any context
 * @group: The group
 *
 * On the CPU of the group's interrupt: a NAPI scheduled from another CPU
 * polls on that CPU, and stays there as long as it finds work. With a
 * group kicked once a second from a worker, one CPU ended up polling all
 * groups while the others idled with their interrupts disabled. A kick
 * still on its way to the CPU is enough; a CPU that is gone gets none,
 * any CPU will do then.
 *
 * The sender takes the NAPI's SCHED bit before it sends: only one kick is
 * on its way at a time, none is sent to a disabled NAPI, and
 * napi_disable() waits for one still on its way, so the csd is idle
 * before the group goes away.
 */
void opip_group_schedule(struct opip_group *group)
{
	if (napi_if_scheduled_mark_missed(&group->napi) ||
	    !napi_schedule_prep(&group->napi))
		return;
	if (smp_call_function_single_async(group->cpu, &group->csd)) {
		local_bh_disable();
		__napi_schedule(&group->napi);
		local_bh_enable();
	}
}

/**
 * opip_rx_refill_check - have the polls run if they owe the packet pool
 * @opip: The complex
 *
 * The packet pool may have been drained with no poll running to refill
 * it (the polls refill what they took, but only when they run, and an
 * allocation may have failed): if the polls owe more than half of it, or
 * anything while it is empty, have the owing ones run. Not on the pool's
 * level alone: under load most of the pool is in flight, and the polls
 * run anyway. The periodic worker calls this once a second, and soon
 * after an idle poll could not refill.
 */
void opip_rx_refill_check(struct opip *opip)
{
	unsigned int owed = 0;
	int i;

	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		if (opip->groups_mask & BIT(i))
			owed += READ_ONCE(opip->groups[i].to_refill);
	if (!owed || (owed <= READ_ONCE(opip->num_packet_buffers) / 2 &&
		      opip_fpa_level(OPIP_POOL_PACKET)))
		return;

	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		if ((opip->groups_mask & BIT(i)) &&
		    READ_ONCE(opip->groups[i].to_refill))
			opip_group_schedule(&opip->groups[i]);
}

static void opip_group_shutdown(struct opip_group *group)
{
	opip_pow_group_irq(group->group, false);
	napi_disable(&group->napi);
	irq_update_affinity_hint(group->irq, NULL);
	free_irq(group->irq, group);
}

/* The NAPI instances of the groups in use, see opip_rx_create_pools() */
static void opip_rx_del_napis(struct opip *opip)
{
	int i;

	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		if (opip->groups_mask & BIT(i))
			netif_napi_del(&opip->groups[i].napi);
}

/*
 * Free what is left in a group that was shut down: packets that came in
 * before input was disabled. Left in the group they would reach the next
 * driver instance.
 */
static int opip_group_flush(struct opip_group *group)
{
	const unsigned int core = opip_core();
	struct opip_wqe *wqe;
	u64 old_mask;
	int left = 0;

	old_mask = opip_pow_select_group(core, group->group);
	while ((wqe = opip_pow_request_sync(false))) {
		opip_rx_free_work(wqe);
		left++;
	}
	opip_pow_group_ack(group->group);
	opip_pow_restore_groups(core, old_mask);
	return left;
}

/*
 * A page pool per group: its poll alone allocates from it, any poll may
 * return pages to it. The packet pool is filled from the first one before
 * the polls run; the refills then keep the buffers moving. The NAPI
 * instances are added first: a page pool recycles directly on the CPU its
 * NAPI is listed on, and a NAPI not yet added claims CPU 0.
 */
static int opip_rx_create_pools(struct opip *opip)
{
	struct opip_group *first = NULL;
	int i, ret;

	for (i = 0; i < OPIP_MAX_GROUPS; i++) {
		struct opip_group *group = &opip->groups[i];
		/*
		 * The size bounds the ring of returned pages only: one that
		 * holds any size ethtool -G can set (128 KB a group)
		 */
		struct page_pool_params pp_params = {
			.pool_size = OPIP_MAX_PACKET_BUFFERS,
			.nid = NUMA_NO_NODE,
			.napi = &group->napi,
		};

		if (!(opip->groups_mask & BIT(i)))
			continue;
		group->pp = page_pool_create(&pp_params);
		if (IS_ERR(group->pp)) {
			ret = PTR_ERR(group->pp);
			group->pp = NULL;
			dev_err(&opip->pdev->dev, "could not create the page pool of group %d: %d\n",
				i, ret);
			/* Nothing went into the packet pool */
			opip->num_packet_buffers = 0;
			return ret;
		}
		if (!first)
			first = group;
	}

	/* What could not be had now, the first poll owes */
	first->to_refill = opip->num_packet_buffers -
		opip_fpa_fill_packet_pool(first->pp, opip->num_packet_buffers,
					  GFP_KERNEL);
	return 0;
}

/* The mirror of opip_group_shutdown(), with the interrupt on @cpu */
static int opip_group_start(struct opip_group *group, int cpu)
{
	struct opip *opip = group->opip;
	int ret;

	napi_enable(&group->napi);

	/* The interrupt starts out disabled, the first poll enables it */
	set_bit(OPIP_GROUP_IRQ_DISABLED, &group->state);
	ret = request_irq(group->irq, opip_group_irq, IRQF_NO_AUTOEN,
			  KBUILD_MODNAME, group);
	if (ret) {
		dev_err(&opip->pdev->dev, "could not get the interrupt %d of group %d\n",
			group->irq, group->group);
		napi_disable(&group->napi);
		return ret;
	}
	group->cpu = cpu;
	INIT_CSD(&group->csd, opip_group_kick, group);
	if (hweight32(opip->groups_mask) > 1)
		irq_set_affinity_and_hint(group->irq, cpumask_of(cpu));
	opip_pow_group_irq(group->group, true);
	/* On the interrupt's CPU, not the one probing */
	opip_group_schedule(group);
	return 0;
}

/**
 * opip_rx_init - start receive processing on the groups in use
 * @opip: The complex, its groups chosen
 *
 * Return: 0 on success, a negative error code otherwise
 */
int opip_rx_init(struct opip *opip)
{
	int i, ret, cpu = -1;

	for (i = 0; i < OPIP_MAX_GROUPS; i++) {
		struct opip_group *group = &opip->groups[i];

		if (!(opip->groups_mask & BIT(i)))
			continue;
		group->irq = OCTEON_IRQ_WORKQ0 + i;
		group->group = i;
		group->opip = opip;
		netif_napi_add(opip->napi_dev, &group->napi, opip_napi_poll);
	}

	ret = opip_rx_create_pools(opip);
	if (ret) {
		opip_rx_del_napis(opip);
		return ret;
	}

	/* The online CPUs in turn: their numbers may have gaps */
	for (i = 0; i < OPIP_MAX_GROUPS; i++) {
		if (!(opip->groups_mask & BIT(i)))
			continue;
		cpu = cpumask_next_wrap(cpu, cpu_online_mask);
		ret = opip_group_start(&opip->groups[i], cpu);
		if (ret)
			goto err;
	}
	return 0;

err:
	while (--i >= 0)
		if (opip->groups_mask & BIT(i))
			opip_group_shutdown(&opip->groups[i]);
	opip_rx_del_napis(opip);
	return ret;
}

/**
 * opip_rx_free_pools - destroy the page pools
 * @opip: The complex
 *
 * After the packet pool was emptied: a page pool waits for the pages
 * still out in skbs, the ones in the FPA would never come back.
 */
void opip_rx_free_pools(struct opip *opip)
{
	int i;

	for (i = 0; i < OPIP_MAX_GROUPS; i++) {
		if (opip->groups[i].pp)
			page_pool_destroy(opip->groups[i].pp);
		opip->groups[i].pp = NULL;
	}
}

/**
 * opip_rx_shutdown - take the receive groups down
 * @opip: The complex, input disabled
 *
 * Each group's interrupt is turned off and freed, then the work left in
 * the group is freed, with a message if there was any, and the NAPI
 * instances are deleted.
 */
void opip_rx_shutdown(struct opip *opip)
{
	int i, left = 0;

	for (i = 0; i < OPIP_MAX_GROUPS; i++) {
		if (!(opip->groups_mask & BIT(i)))
			continue;
		opip_group_shutdown(&opip->groups[i]);
		/* Per core group mask and NAPI-like context */
		local_bh_disable();
		left += opip_group_flush(&opip->groups[i]);
		local_bh_enable();
	}
	opip_rx_del_napis(opip);
	if (left)
		dev_dbg(&opip->pdev->dev, "%d packets left in the groups at shutdown\n",
			left);
}
