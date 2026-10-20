// SPDX-License-Identifier: GPL-2.0
/*
 * The receive side of octeon_pip: the POW groups, one per CPU, each with
 * its interrupt, NAPI instance and page pool. A group delivers the
 * received packets the PIP tagged for it.
 */

#include <linux/atomic.h>
#include <linux/bpf.h>
#include <linux/bpf_trace.h>
#include <linux/cpumask.h>
#include <linux/etherdevice.h>
#include <linux/filter.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/prefetch.h>
#include <linux/skbuff_ref.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <net/page_pool/helpers.h>
#include <net/xdp.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_gmx.h"
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

/*
 * On while a port has an XDP program: the polls run their instance with
 * the program's tests, see opip_poll(). Counted by opip_xdp_setup().
 */
DEFINE_STATIC_KEY_FALSE(opip_rx_xdp_key);

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
 * With the preamble check off (10 Mbit/s RGMII, see
 * opip_check_preamble_errors()), a PHY may deliver the preamble in front
 * of the packet, also shifted by a nibble. Take it off. Only a packet in
 * one buffer: moving the data of a later one loses the link to the next
 * segment. The preamble and SFD are eight bytes.
 *
 * Return: true if the packet was repaired
 */
static bool opip_rx_fix_preamble(struct opip_wqe *wqe)
{
	u8 *ptr = opip_virt(opip_buf_addr(wqe->packet_ptr));
	unsigned int len = opip_wqe_len(wqe);
	unsigned int i = 0;

	if (opip_wqe_bufs(wqe) != 1 || len < 8 + ETH_HLEN + ETH_FCS_LEN)
		return false;

	while (i < 7 && *ptr == 0x55) {
		ptr++;
		i++;
	}

	if (*ptr == 0xd5) {
		wqe->packet_ptr += i + 1;
		wqe->word1 = u64_replace_bits(wqe->word1, len - i - 5,
					      OPIP_WQE1_LEN);
		return true;
	}
	if ((*ptr & 0xf) == 0xd) {
		wqe->packet_ptr += i;
		len -= i + 4;
		wqe->word1 = u64_replace_bits(wqe->word1, len, OPIP_WQE1_LEN);
		for (i = 0; i < len; i++) {
			*ptr = (*ptr >> 4) | (*(ptr + 1) << 4);
			ptr++;
		}
		return true;
	}
	return false;
}

/* Count a dropped packet whose error the PIP's counters do not have */
static void opip_rx_count_error(struct net_device *dev, unsigned int err)
{
	struct opip_port *p;

	if (!dev || (err >= OPIP_RCV_ERR_OVER_FCS && err <= OPIP_RCV_ERR_UNDER))
		return;
	p = opip_port(dev);
	if (err == OPIP_RCV_ERR_JABBER)
		atomic_long_inc(&p->rx_jabber);
	else if (err == OPIP_RCV_ERR_LENGTH)
		atomic_long_inc(&p->rx_l2_length);
	else
		atomic_long_inc(&p->rx_other_errors);
}

/*
 * A packet with a receive error. Minimum size frames with a length error
 * pass: some equipment pads to 64 + 4 FCS instead of 60 + 4. An alignment
 * or FCS error with the preamble check off may be a preamble in the data.
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

	if ((err == OPIP_RCV_ERR_ALIGN || err == OPIP_RCV_ERR_FCS) && dev) {
		struct opip_port *p = opip_port(dev);

		if (opip_port_rgmii_preamble(p) &&
		    opip_gmx_preamble_check_off(p->iface->num, p->index)) {
			/*
			 * The FCS is not checked, and the PIP parsed the
			 * preamble: the stack checks the L4 checksum.
			 */
			if (opip_rx_fix_preamble(wqe)) {
				wqe->word2 &= ~OPIP_WQE2_TCP_OR_UDP;
				/*
				 * The entry goes back to its pool without a
				 * barrier: these writes are out first
				 */
				wmb();
				return false;
			}
			opip_rx_dbg(dev, port, "unknown preamble, packet dropped\n");
			opip_rx_free_work(wqe);
			return true;
		}
	}

	opip_rx_dbg(dev, port, "receive error code %u, packet dropped\n", err);
	opip_rx_count_error(dev, err);
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
 * The packet pool buffer of a frag of a packet over several buffers: a
 * whole page, or the slice the frag is in (a program may have moved the
 * frag's start, bpf_xdp_pull_data())
 */
static void *opip_rx_frag_buffer(const skb_frag_t *frag)
{
	unsigned int off = skb_frag_off(frag);

	return page_address(skb_frag_page(frag)) +
	       (OPIP_RX_SPLIT ? off - off % OPIP_RX_SLICE_SIZE : 0);
}

/*
 * Give the frags of a packet back to the hardware, as an XDP program left
 * them: what it took off the tail went back to the page pool already.
 * Before the first buffer's free: its shared info lists them, and the
 * hardware may fill that buffer again as soon as it has it.
 */
static void opip_rx_xdp_free_frags(const struct xdp_buff *xdp)
{
	const struct skb_shared_info *sinfo =
		xdp_get_shared_info_from_buff(xdp);
	unsigned int i;

	/* What the program wrote into the frags out before the FPA has them */
	wmb();
	for (i = 0; i < sinfo->nr_frags; i++)
		opip_fpa_free_nosync(opip_rx_frag_buffer(&sinfo->frags[i]),
				     OPIP_POOL_PACKET,
				     OPIP_POOL_PACKET_SIZE / 128);
}

/* A segment's PKO buffer pointer, 0 if BACK cannot name its buffer */
static u64 opip_rx_xdp_tx_seg(const void *buffer, const void *data,
			      unsigned int size)
{
	u64 addr = opip_xkphys_to_phys(data);
	unsigned int back = (addr >> 7) - (opip_xkphys_to_phys(buffer) >> 7);

	if (back > FIELD_MAX(OPIP_BUF_BACK) || !size)
		return 0;
	return opip_buf_build(addr, size, OPIP_POOL_PACKET, back, false);
}

/*
 * XDP_TX of a packet over several buffers: the gather list goes into the
 * packet's work entry, which the completion frees with it. A segment
 * needs a BACK of four bits like a single buffer (a CN3XXX limit of the
 * pointer, the later chips have the same field) and a size; the list
 * holds OPIP_TX_MAX_SEGS of them. Out of line: the jumbo frames only.
 *
 * Return: 0, or an error: the caller frees the buffers and the entry
 */
static noinline int opip_rx_xdp_tx_frags(struct opip_group *group,
					 struct net_device *dev,
					 struct opip_wqe *wqe,
					 const struct xdp_buff *xdp)
{
	const struct skb_shared_info *sinfo =
		xdp_get_shared_info_from_buff(xdp);
	unsigned int segs = sinfo->nr_frags + 1, i;
	u64 *list = (u64 *)wqe;

	if (segs > OPIP_TX_MAX_SEGS)
		return -E2BIG;
	list[0] = opip_rx_xdp_tx_seg(xdp->data_hard_start, xdp->data,
				     xdp->data_end - xdp->data);
	if (!list[0])
		return -EINVAL;
	for (i = 1; i < segs; i++) {
		const skb_frag_t *frag = &sinfo->frags[i - 1];

		list[i] = opip_rx_xdp_tx_seg(opip_rx_frag_buffer(frag),
					     skb_frag_address(frag),
					     skb_frag_size(frag));
		if (!list[i])
			return -EINVAL;
	}
	return opip_xdp_tx_list(dev, &group->tx_poll, list, segs,
				xdp_get_buff_len(xdp));
}

/*
 * XDP_TX of a packet in one buffer, as the program left it. BACK has four
 * bits: data moved 2048 bytes or more into a whole page cannot name its
 * buffer (a slice ends before that). Always inline: per packet.
 */
static __always_inline int opip_rx_xdp_tx_one(struct opip_group *group,
					      struct net_device *dev,
					      void *buffer,
					      const struct xdp_buff *xdp)
{
	unsigned int len = xdp->data_end - xdp->data;
	u64 addr = opip_xkphys_to_phys(xdp->data);
	unsigned int back;

	back = (addr >> 7) - (opip_xkphys_to_phys(buffer) >> 7);
	if (back > FIELD_MAX(OPIP_BUF_BACK))
		return -EINVAL;
	return opip_xdp_tx(dev, &group->tx_poll,
			   opip_buf_build(addr, len, OPIP_POOL_PACKET, back,
					  false), len);
}

/*
 * Run the XDP program of the port on a packet, as @xdp has it: the
 * caller prepared it, the whole first buffer at @buffer and with @chain
 * the frags of a packet over @segments buffers. For XDP_TX the buffers
 * stay the hardware's, for a drop they go back to the hardware, for
 * XDP_REDIRECT they become the frame's and the pool is refilled for them;
 * for XDP_PASS the caller builds the skb from @xdp. What the program
 * took off a chain's tail went back to the page pool: refilled too. The
 * hardware's chain is not walked again, the program may have changed
 * the link words in the headroom. Always inline: two callers, the
 * single buffer one per packet and without the chain's tests.
 *
 * Return: the verdict
 */
static __always_inline u32 opip_rx_xdp(struct opip_group *group,
				       struct net_device *dev,
				       struct bpf_prog *prog,
				       struct opip_wqe *wqe, void *buffer,
				       struct xdp_buff *xdp, const bool chain,
				       unsigned int segments)
{
	unsigned int frags = 0;
	u32 act;

	act = bpf_prog_run_xdp(prog, xdp);
	switch (act) {
	case XDP_PASS:
		return act;
	case XDP_TX: {
		int err;

		/* Before the PKO has the buffers: the count is in the first */
		if (chain && xdp_buff_has_frags(xdp)) {
			frags = xdp_get_shared_info_from_buff(xdp)->nr_frags;
			err = opip_rx_xdp_tx_frags(group, dev, wqe, xdp);
		} else {
			err = opip_rx_xdp_tx_one(group, dev, buffer, xdp);
		}
		if (err) {
			trace_xdp_exception(dev, prog, act);
			dev_core_stats_tx_dropped_inc(dev);
			goto free;
		}
		if (chain)
			WRITE_ONCE(group->to_refill,
				   group->to_refill + segments - 1 - frags);
		/* The entry of a packet over several buffers is its list */
		if (frags)
			return act;
		break;
	}
	case XDP_REDIRECT:
		if (xdp_do_redirect(dev, xdp, prog)) {
			trace_xdp_exception(dev, prog, act);
			goto free;
		}
		WRITE_ONCE(group->to_refill, group->to_refill + segments);
		break;
	default:
		bpf_warn_invalid_xdp_action(dev, prog, act);
		fallthrough;
	case XDP_ABORTED:
		trace_xdp_exception(dev, prog, act);
		fallthrough;
	case XDP_DROP:
free:
		if (chain) {
			if (xdp_buff_has_frags(xdp)) {
				frags = xdp_get_shared_info_from_buff(xdp)->
					nr_frags;
				opip_rx_xdp_free_frags(xdp);
			}
			WRITE_ONCE(group->to_refill,
				   group->to_refill + segments - 1 - frags);
		}
		opip_fpa_free(buffer, OPIP_POOL_PACKET,
			      OPIP_POOL_PACKET_SIZE / 128);
		act = XDP_DROP;
		break;
	}
	opip_fpa_free_nosync(wqe, OPIP_POOL_WQE, 1);
	return act;
}

/*
 * The zero copy path: a packet that fits in one buffer gets an skb built
 * around it. The buffer is a page (or a slice) of any group's page pool
 * (the packet pool mixes them) and goes back to that pool with its last
 * skb: into its cache when freed in its own group's poll, else its ring.
 * The skb comes from the packet's xdp_buff, as an XDP program left it:
 * the head and the metadata where it moved them, the buffer the same;
 * with @chain the frags of a packet over several buffers too
 * (opip_rx_prepare_chain()), whose descriptors sit in the
 * skb_shared_info of the first buffer already. Not
 * xdp_build_skb_from_buff(): a call per packet, the receive queue and
 * the protocol are opip_rx_deliver()'s, and the frame size and the page
 * pool are constants here. Always inline: with two callers gcc calls it
 * out of line, per packet.
 */
static __always_inline struct sk_buff *
opip_rx_build_xdp_skb(const struct xdp_buff *xdp, const bool chain)
{
	unsigned int nr_frags = 0, frags_size = 0;
	struct sk_buff *skb;
	int meta;

	/* The build clears the counts of the skb_shared_info */
	if (chain && xdp_buff_has_frags(xdp)) {
		const struct skb_shared_info *sinfo;

		sinfo = xdp_get_shared_info_from_buff(xdp);
		nr_frags = sinfo->nr_frags;
		frags_size = sinfo->xdp_frags_size;
	}
	skb = napi_build_skb(xdp->data_hard_start, OPIP_RX_FRAG_SIZE);
	if (unlikely(!skb))
		return NULL;
	skb_mark_for_recycle(skb);
	skb_reserve(skb, xdp->data - xdp->data_hard_start);
	__skb_put(skb, xdp->data_end - xdp->data);
	meta = xdp->data - xdp->data_meta;
	if (meta > 0)
		skb_metadata_set(skb, meta);
	/* Each frag is a whole buffer, however much a program took off it */
	if (chain && nr_frags)
		xdp_update_skb_frags_info(skb, nr_frags, frags_size,
					  nr_frags * OPIP_RX_FRAG_SIZE,
					  xdp_buff_get_skb_flags(xdp));
	return skb;
}

/*
 * A packet over several buffers as an xdp_buff: the first buffer its
 * linear part, the others its frags, page_pool buffers too: no high
 * order allocation for a jumbo frame. The caller checked that the frags
 * fit and initialized @xdp. Two buffers of a chain never coalesce into
 * one frag: the data of a later buffer starts behind its link word,
 * after the end of the buffer before it.
 */
static void opip_rx_prepare_chain(struct xdp_buff *xdp,
				  const struct opip_wqe *wqe,
				  unsigned int segments)
{
	unsigned int len = opip_wqe_len(wqe), size;
	u64 seg = wqe->packet_ptr;
	void *start, *data;

	start = opip_virt(opip_buf_start(seg));
	data = opip_virt(opip_buf_addr(seg));
	/* Errata PKI-100, see opip_rx_copy_segments() */
	size = min_t(unsigned int, len, OPIP_POOL_PACKET_SIZE - (data - start));
	xdp_prepare_buff(xdp, start, data - start, size, true);

	while (--segments) {
		len -= size;
		seg = opip_buf_next(seg);
		start = opip_virt(opip_buf_start(seg));
		data = opip_virt(opip_buf_addr(seg));
		size = min_t(unsigned int, len,
			     OPIP_POOL_PACKET_SIZE - (data - start));
		xdp_buff_add_frag(xdp, page_to_netmem(virt_to_page(start)),
				  opip_rx_page_offset(start, data), size,
				  OPIP_RX_FRAG_SIZE);
	}
}

/* A packet the PIP left in the entry, or over more buffers than frags */
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
					    struct sk_buff *skb,
					    bool csum_valid)
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
		     !(wqe->word2 & OPIP_WQE2_TCP_OR_UDP) || !csum_valid ||
		     !(features & NETIF_F_RXCSUM)))
		skb->ip_summed = CHECKSUM_NONE;
	else
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	napi_gro_receive(&group->napi, skb);
}

/*
 * A packet not in one buffer: its buffers become an xdp_buff with frags,
 * which a program for multi-buffer packets sees, and an skb from that, or
 * it is copied. Returns true if the program redirected the packet.
 */
static noinline bool opip_rx_slow_packet(struct opip_group *group,
					 struct opip_wqe *wqe,
					 struct net_device *dev)
{
	struct bpf_prog *prog = rcu_dereference_bh(opip_port(dev)->xdp_prog);
	unsigned int segments = opip_wqe_bufs(wqe);
	bool csum_valid = true;
	struct xdp_buff xdp;
	struct sk_buff *skb;

	/* A packet the program cannot see does not pass it */
	if (unlikely(prog && (!segments || segments > MAX_SKB_FRAGS + 1 ||
			      !prog->aux->xdp_has_frags))) {
		dev_core_stats_rx_dropped_inc(dev);
		opip_rx_free_work(wqe);
		return false;
	}

	if (!segments || segments > MAX_SKB_FRAGS + 1) {
		skb = opip_rx_copy_skb(group, wqe);
		if (skb)
			opip_rx_deliver(group, dev, wqe, skb, true);
		opip_rx_free_work(wqe);
		return false;
	}

	xdp_init_buff(&xdp, OPIP_RX_FRAG_SIZE,
		      &opip_port(dev)->xdp_rxq[group->group]);
	opip_rx_prepare_chain(&xdp, wqe, segments);
	if (prog) {
		unsigned int len = opip_wqe_len(wqe);
		void *data = xdp.data;
		u32 act;

		act = opip_rx_xdp(group, dev, prog, wqe, xdp.data_hard_start,
				  &xdp, true, segments);
		if (act != XDP_PASS)
			return act == XDP_REDIRECT;
		csum_valid = xdp.data == data && xdp_get_buff_len(&xdp) == len;
	}

	skb = opip_rx_build_xdp_skb(&xdp, true);
	if (!skb) {
		/* As the program left it: not along the hardware's chain */
		if (xdp_buff_has_frags(&xdp)) {
			segments -=
				xdp_get_shared_info_from_buff(&xdp)->nr_frags;
			opip_rx_xdp_free_frags(&xdp);
		}
		opip_fpa_free(xdp.data_hard_start, OPIP_POOL_PACKET,
			      OPIP_POOL_PACKET_SIZE / 128);
		WRITE_ONCE(group->to_refill, group->to_refill + segments - 1);
		opip_fpa_free_nosync(wqe, OPIP_POOL_WQE, 1);
		return false;
	}
	opip_rx_deliver(group, dev, wqe, skb, csum_valid);
	/* The buffers are the skb's, replaced later */
	WRITE_ONCE(group->to_refill, group->to_refill + segments);
	opip_fpa_free_nosync(wqe, OPIP_POOL_WQE, 1);
	return false;
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

/*
 * One received packet: receive errors, the XDP program, the skb and its
 * delivery. Returns true if the program redirected the packet. Without
 * @xdp no port has a program (in a packet in one buffer; the chains look
 * for themselves). Always inline: the two instances of the poll need
 * their own, one without the program's code.
 */
static __always_inline bool opip_rx_packet(struct opip_group *group,
					   struct opip_wqe *wqe,
					   struct net_device *dev, int port,
					   const bool xdp_on)
{
	bool skb_in_hw = opip_wqe_bufs(wqe) == 1;
	bool csum_valid = true;
	void *buffer = NULL;
	struct sk_buff *skb;
	unsigned int len;
	void *data;

	if (likely(skb_in_hw))
		buffer = opip_virt(opip_buf_start(wqe->packet_ptr));

	if (unlikely(wqe->word2 & OPIP_WQE2_RCV_ERROR) &&
	    opip_rx_check_error(wqe, dev, port))
		return false;

	/*
	 * One test for the skbs and XDP: a device being closed (stopped
	 * already, IFF_UP not cleared yet) neither delivers nor sends
	 */
	if (unlikely(!dev || !netif_running(dev))) {
		opip_rx_drop_closed(wqe, dev, port);
		return false;
	}

	data = opip_virt(opip_buf_addr(wqe->packet_ptr));
	len = opip_wqe_len(wqe);

	/*
	 * An XDP program sees the packets that sit in one buffer. With a
	 * program or without, the packet is an xdp_buff of the whole
	 * buffer and the skb is built from it.
	 */
	if (likely(skb_in_hw)) {
		struct bpf_prog *prog = NULL;
		struct xdp_buff xdp;

		if (xdp_on)
			prog = rcu_dereference_bh(opip_port(dev)->xdp_prog);

		xdp_init_buff(&xdp, OPIP_RX_FRAG_SIZE, NULL);
		xdp_prepare_buff(&xdp, buffer, data - buffer, len, true);
		if (prog) {
			u32 act;

			/* The port's queue of the group */
			xdp.rxq = &opip_port(dev)->xdp_rxq[group->group];
			act = opip_rx_xdp(group, dev, prog, wqe, buffer, &xdp,
					  false, 1);
			if (act != XDP_PASS)
				return act == XDP_REDIRECT;
			csum_valid = xdp.data == data &&
				     xdp.data_end == data + len;
		}
		skb = opip_rx_build_xdp_skb(&xdp, false);
	} else {
		return opip_rx_slow_packet(group, wqe, dev);
	}

	if (unlikely(!skb)) {
		opip_rx_free_work(wqe);
		return false;
	}

	opip_rx_deliver(group, dev, wqe, skb, csum_valid);

	/* The entry goes back; the skb's buffer is replaced later */
	WRITE_ONCE(group->to_refill, group->to_refill + 1);
	opip_fpa_free_nosync(wqe, OPIP_POOL_WQE, 1);
	return false;
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
 * Pay off what a shrink of the packet pool could not take out, from @n
 * buffers a poll would replace: they are out of the pool already (with
 * the stack), and not replacing them takes them out for good. Returns
 * the number paid.
 */
static noinline unsigned int opip_rx_pay_excess(atomic_t *excess,
						unsigned int n)
{
	int old = atomic_read(excess);
	unsigned int pay;

	do {
		pay = min_t(unsigned int, old, n);
		if (!pay)
			break;
	} while (!atomic_try_cmpxchg(excess, &old, old - pay));
	return pay;
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
	/* The pool was made smaller: replace fewer */
	if (unlikely(atomic_read(&group->opip->rx_excess)))
		n -= opip_rx_pay_excess(&group->opip->rx_excess, n);
	n -= opip_fpa_fill_packet_pool(group->pp, n, GFP_ATOMIC);
	WRITE_ONCE(group->to_refill, n);
	return n;
}

static __always_inline int __opip_poll(struct opip_group *group, int budget,
				       const bool xdp_on)
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
	bool requested = false, idle = false, redirected = false;

	prefetch(ports);

	if (OPIP_ASYNC_POW) {
		/* Userspace may use the scratch word: save it */
		opip_iobdma_wait();
		old_scratch = opip_scratch_rd(OPIP_POW_SCRATCH);
	}

	old_mask = opip_pow_select_group(core, group->group);
	opip_tx_poll_begin(&group->tx_poll, budget);
	/*
	 * The packet pool mixes the pages of all groups' page pools: what XDP
	 * returns in this poll (a frame a devmap flush could not send, a
	 * packet AF_XDP copied) may be another group's page, and only that
	 * group's poll may use its pool's cache. Without the direct returns
	 * the page pool still puts a page of its own into the cache, by the
	 * CPU of its NAPI, and the others into their pools' rings.
	 */
	xdp_set_return_frame_no_direct();

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

		redirected |= opip_rx_packet(group, wqe, dev, port, xdp_on);

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
	if (redirected)
		xdp_do_flush();
	xdp_clear_return_frame_no_direct();
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

/*
 * The poll in two instances: with the tests for an XDP program, and
 * without, while no port has one. The packet's xdp_buff never leaves
 * the second, so it lives in registers there, and its skb path is the
 * one of a driver without XDP. opip_xdp_setup() turns the key on before
 * a first program is visible to a poll and off after the last is gone.
 */
static noinline int opip_poll_xdp(struct opip_group *group, int budget)
{
	return __opip_poll(group, budget, true);
}

static noinline int opip_poll_skb(struct opip_group *group, int budget)
{
	return __opip_poll(group, budget, false);
}

static int opip_poll(struct opip_group *group, int budget)
{
	if (static_branch_unlikely(&opip_rx_xdp_key))
		return opip_poll_xdp(group, budget);
	return opip_poll_skb(group, budget);
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

/*
 * Grow the packet pool from @have to @count buffers, all or nothing. The
 * entries the buffers need come first: more than needed are harmless,
 * the receive share of the WQE pool grows to the largest size asked for.
 * What a shrink could not take out yet is still in the hardware's hands
 * or owed: it counts against the growth. The rest comes from the first
 * group's page pool, with its poll held off (only the poll allocates
 * from it), into a staging array, and goes into the pool only once it is
 * all there; putting it there cannot fail.
 */
static int opip_rx_grow(struct opip *opip, unsigned int have,
			unsigned int count)
{
	unsigned int add = count - have, pending, n, i;
	struct opip_group *group = NULL;
	void **bufs;
	int ret = 0;

	if (count > opip->rx_wqe_bufs) {
		n = opip_fpa_fill(opip, OPIP_POOL_WQE, OPIP_POOL_WQE_SIZE,
				  count - opip->rx_wqe_bufs);
		opip->wqe_bufs += n;
		opip->rx_wqe_bufs += n;
		if (opip->rx_wqe_bufs < count)
			return -ENOMEM;
	}

	pending = atomic_xchg(&opip->rx_excess, 0);
	if (pending >= add) {
		atomic_add(pending - add, &opip->rx_excess);
		return 0;
	}
	add -= pending;

	bufs = kvmalloc_array(add, sizeof(*bufs), GFP_KERNEL);
	if (!bufs) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < OPIP_MAX_GROUPS && !group; i++)
		if (opip->groups[i].pp)
			group = &opip->groups[i];

	napi_disable(&group->napi);
	for (n = 0; n < add; n++) {
		bufs[n] = opip_fpa_alloc_packet_buf(group->pp, GFP_KERNEL);
		if (!bufs[n])
			break;
	}
	if (n == add) {
		/* The pages' writes out before the IPD may fill them */
		wmb();
		for (i = 0; i < n; i++)
			opip_fpa_free_nosync(bufs[i], OPIP_POOL_PACKET,
					     OPIP_POOL_PACKET_SIZE / 128);
	} else {
		while (n--)
			opip_fpa_put_packet_buf(bufs[n]);
		ret = -ENOMEM;
	}
	/* Have the poll run for an interrupt that found it disabled */
	napi_enable(&group->napi);
	opip_group_schedule(group);
	kvfree(bufs);
out:
	if (ret)
		atomic_add(pending, &opip->rx_excess);
	return ret;
}

/*
 * Take @take buffers out of the packet pool, which is to have @count:
 * the free ones go back to their page pools now, as long as the pool
 * stays above the RED pass mark of the new size, so that a shrink under
 * load makes RED drop nothing; the rest, and what an earlier shrink left
 * over, the polls pay off as they replace the buffers the stack took
 * (opip_rx_refill()). Under load most of the pool is in flight, so this
 * does not wait for it.
 */
static void opip_rx_shrink(struct opip *opip, unsigned int count,
			   unsigned int take)
{
	unsigned int taken = 0;
	void *buf;

	take += atomic_xchg(&opip->rx_excess, 0);
	/* The level read each time: the IPD keeps taking buffers meanwhile */
	while (taken < take &&
	       opip_fpa_level(OPIP_POOL_PACKET) > count / 4) {
		buf = opip_fpa_alloc(OPIP_POOL_PACKET);
		if (!buf)
			break;
		opip_fpa_put_packet_buf(buf);
		taken++;
	}
	atomic_add(take - taken, &opip->rx_excess);
}

/**
 * opip_rx_resize - change the size of the packet pool, the receive ring
 * @opip:  The complex, under rtnl
 * @count: Buffers the pool is to have: free ones, ones the hardware
 *         holds and ones the polls owe it, not the ones skbs hold
 *
 * Growing either brings the pool to @count or changes nothing, see
 * opip_rx_grow(). Shrinking cannot fail: what the pool cannot give back
 * now, the polls give back later, see opip_rx_shrink(). The size is
 * @count from here on, for ethtool -g too, and RED and the backpressure
 * thresholds follow it; on a shrink before the buffers are taken out,
 * so that the old drop mark does not drop in the meantime.
 *
 * Return: 0, or -ENOMEM if the pool could not grow; it is as it was
 */
int opip_rx_resize(struct opip *opip, unsigned int count)
{
	unsigned int have = opip->num_packet_buffers;
	int ret, i;

	if (count == have)
		return 0;
	if (count > have) {
		ret = opip_rx_grow(opip, have, count);
		if (ret)
			return ret;
	}
	WRITE_ONCE(opip->num_packet_buffers, count);

	mutex_lock(&opip->hw_lock);
	opip_ipd_red_setup(count / 4, count / 8);
	mutex_unlock(&opip->hw_lock);
	for (i = 0; i < OPIP_MAX_PORTS; i++)
		if (opip->ports[i])
			opip_pause_refresh(opip_port(opip->ports[i]));

	if (count < have)
		opip_rx_shrink(opip, count, have - count);
	return 0;
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
