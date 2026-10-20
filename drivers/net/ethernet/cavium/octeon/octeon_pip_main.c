// SPDX-License-Identifier: GPL-2.0
/*
 * octeon_pip: the Ethernet driver of the OCTEON packet I/O blocks.
 *
 * The probe brings the blocks up in the order they depend on each other:
 * the pools, the interfaces and their ports, the output queues, the POW
 * groups that deliver the packets, then packet input, and the network
 * devices last.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper.c, cvmx-helper-errata.c) and the staging octeon
 * driver (ethernet.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include <linux/bpf.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <net/netdev_queues.h>
#include <net/xdp.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_ipd.h"
#include "octeon_pip_pcs.h"
#include "octeon_pip_pko.h"
#include "octeon_pip_pow.h"
#include "octeon_pip_regs.h"
#include "octeon_pip_rx.h"
#include "octeon_pip_tx.h"

/* PKO command chunks put in the chunk pool */
#define OPIP_NUM_CMD_CHUNKS	1024

/*
 * Command chunks a full transmit queue takes: two words a packet, the
 * usable words of a chunk, and one more for the chunk the PKO still reads.
 */
#define OPIP_TXQ_CMD_CHUNKS \
	(DIV_ROUND_UP(OPIP_TXQ_DEPTH * 2, OPIP_PKO_CHUNK_WORDS) + 1)

/* ------------------------------------------------------------------ */
/* The link of a port, from the blocks that make it */

/**
 * opip_link_set - set up a port for a new link state
 * @p:    Port
 * @link: The new link state
 *
 * Return: zero on success
 */
int opip_link_set(struct opip_port *p, const struct opip_link *link)
{
	int iface = p->iface->num;
	int ret = 0;

	mutex_lock(&p->opip->hw_lock);
	switch (p->iface->mode) {
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		ret = opip_pcs_link_set(iface, p->index, link);
		break;
	case OPIP_MODE_RGMII:
	case OPIP_MODE_GMII:
		opip_gmx_rgmii_link_set(p->opip, iface, p->index, p->port,
					link);
		break;
	case OPIP_MODE_XAUI:
		ret = opip_xaui_link_set(iface, p->port, link);
		break;
	default:
		break;
	}
	mutex_unlock(&p->opip->hw_lock);
	return ret;
}

/**
 * opip_pcs_setup - set the PCS of an SGMII, QSGMII or 1000BASE-X port up
 * @p:      Port
 * @inband: In-band autonegotiation, or the PCS forced out of band
 *
 * Set the PCS of an SGMII, QSGMII or 1000BASE-X port up for in-band or
 * out-of-band operation, under the hardware lock. Ports of other modes
 * are left alone.
 *
 * Return: 0, or the error of opip_pcs_config()
 */
int opip_pcs_setup(struct opip_port *p, bool inband)
{
	int ret = 0;

	mutex_lock(&p->opip->hw_lock);
	if (opip_port_has_pcs(p))
		ret = opip_pcs_config(p->iface->num, p->index, inband);
	mutex_unlock(&p->opip->hw_lock);
	return ret;
}

/**
 * opip_pcs_restart_an - restart the PCS autonegotiation of a PCS port
 * @p: Port
 *
 * Under the hardware lock. Ports of other modes are left alone.
 */
void opip_pcs_restart_an(struct opip_port *p)
{
	mutex_lock(&p->opip->hw_lock);
	if (opip_port_has_pcs(p))
		opip_pcs_an_restart(p->iface->num, p->index);
	mutex_unlock(&p->opip->hw_lock);
}

/**
 * opip_port_enable - enable or disable the MAC of a port that has one
 * @p:  Port
 * @en: Enable
 *
 * Only ports of a GMX interface have one; the port number is the port
 * kind on the chips that have them.
 */
void opip_port_enable(struct opip_port *p, bool en)
{
	if (!opip_iface_is_gmx(p->iface))
		return;
	mutex_lock(&p->opip->hw_lock);
	opip_gmx_port_enable(p->iface->num, p->index, en, p->port);
	mutex_unlock(&p->opip->hw_lock);
}

/* ------------------------------------------------------------------ */
/* Statistics */

/**
 * opip_update_hw_stats - accumulate the hardware counters of a port
 * @dev: The port's network device
 *
 * The counters of the PIP and the GMX are cleared on read; the PKO's keep
 * running and give what they moved since the last read. Called from
 * process context, once a second by the periodic worker, and by ethtool.
 */
void opip_update_hw_stats(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	struct opip_gmx_stats gmx = {};
	u32 tx_packets, pko_packets;
	u64 tx_octets, pko_octets;
	struct opip_hw_stats *hw;
	struct opip_pip_stats rx;
	unsigned long flags;
	int i;

	hw = &p->hw_stats;

	/* The simulator has no counters */
	if (opip_on_simulator())
		return;

	mutex_lock(&p->opip->hw_lock);
	opip_pip_read_stats(p->port, &rx);
	opip_pko_read_stats(p->opip, p->port, &pko_packets, &pko_octets);
	tx_packets = pko_packets - p->pko_packets;
	tx_octets = (pko_octets - p->pko_octets) & OPIP_PKO_MEM_COUNT1_COUNT;
	p->pko_packets = pko_packets;
	p->pko_octets = pko_octets;
	/* Wrap or saturate, the counters are cleared long before either */
	if (pko_packets >= BIT(31) || pko_octets >= BIT_ULL(47)) {
		opip_pko_clear_stats(p->opip, p->port);
		p->pko_packets = 0;
		p->pko_octets = 0;
	}
	if (opip_port_has_gmx(p))
		opip_gmx_read_stats(p->iface->num, p->index, &gmx);
	mutex_unlock(&p->opip->hw_lock);

	spin_lock_irqsave(&p->stats_lock, flags);
	p->gmx_stats.rx_ctl_packets += gmx.rx_ctl_packets;
	p->gmx_stats.rx_dmac_packets += gmx.rx_dmac_packets;
	p->gmx_stats.rx_drop_packets += gmx.rx_drop_packets;
	p->gmx_stats.tx_packets += gmx.tx_packets;
	p->gmx_stats.tx_octets += gmx.tx_octets;
	p->gmx_stats.tx_ctl_packets += gmx.tx_ctl_packets;
	p->gmx_stats.tx_underflow += gmx.tx_underflow;
	p->gmx_stats.tx_single_collisions += gmx.tx_single_collisions;
	p->gmx_stats.tx_multiple_collisions += gmx.tx_multiple_collisions;
	p->gmx_stats.tx_excessive_collisions += gmx.tx_excessive_collisions;
	p->gmx_stats.tx_excessive_deferrals += gmx.tx_excessive_deferrals;
	p->gmx_stats.tx_multicast += gmx.tx_multicast;
	p->gmx_stats.tx_broadcast += gmx.tx_broadcast;
	for (i = 0; i < OPIP_GMX_HIST_BINS; i++)
		p->gmx_stats.tx_hist[i] += gmx.tx_hist[i];
	hw->rx_packets += rx.inb_packets;
	hw->rx_octets += rx.inb_octets;
	hw->rx_parsed_packets += rx.packets;
	hw->rx_parsed_octets += rx.octets;
	hw->rx_dropped_packets += rx.dropped_packets;
	hw->rx_dropped_octets += rx.dropped_octets;
	hw->rx_fcs_align_errors += rx.fcs_align_errors;
	hw->rx_runts += rx.runts;
	hw->rx_runts_fcs += rx.runts_fcs;
	hw->rx_oversize += rx.oversize;
	hw->rx_oversize_fcs += rx.oversize_fcs;
	hw->rx_broadcast += rx.broadcast;
	hw->rx_multicast += rx.multicast;
	for (i = 0; i < OPIP_PIP_HIST_BINS; i++)
		hw->rx_hist[i] += rx.hist[i];
	hw->tx_packets += tx_packets;
	hw->tx_octets += tx_octets;
	spin_unlock_irqrestore(&p->stats_lock, flags);
}

/**
 * opip_get_hw_stats - a consistent copy of the accumulated counters
 * @dev: The port's network device
 * @hw:  The copy
 */
void opip_get_hw_stats(struct net_device *dev, struct opip_hw_stats *hw)
{
	struct opip_port *p = opip_port(dev);
	unsigned long flags;

	spin_lock_irqsave(&p->stats_lock, flags);
	*hw = p->hw_stats;
	spin_unlock_irqrestore(&p->stats_lock, flags);
}

/**
 * opip_get_gmx_stats - a consistent copy of the accumulated MAC counters
 * @dev: The port's network device
 * @gmx: The copy
 */
void opip_get_gmx_stats(struct net_device *dev, struct opip_gmx_stats *gmx)
{
	struct opip_port *p = opip_port(dev);
	unsigned long flags;

	spin_lock_irqsave(&p->stats_lock, flags);
	*gmx = p->gmx_stats;
	spin_unlock_irqrestore(&p->stats_lock, flags);
}

/*
 * May be called in atomic context, so it reports what the periodic worker
 * collected, up to a second old. The core adds the drops counted in
 * software.
 */
static void opip_get_stats64(struct net_device *dev,
			     struct rtnl_link_stats64 *stats)
{
	struct opip_port *p = opip_port(dev);
	unsigned long jabber, l2_length;
	struct opip_gmx_stats gmx;
	struct opip_hw_stats hw;

	opip_get_hw_stats(dev, &hw);
	opip_get_gmx_stats(dev, &gmx);
	jabber = atomic_long_read(&p->rx_jabber);
	l2_length = atomic_long_read(&p->rx_l2_length);
	/* What the PIP passed on; the RED drops are missed */
	stats->rx_packets = hw.rx_parsed_packets;
	stats->rx_bytes = hw.rx_parsed_octets;
	stats->rx_otherhost_dropped = gmx.rx_dmac_packets;
	stats->tx_packets = hw.tx_packets;
	stats->tx_bytes = hw.tx_octets;
	stats->multicast = hw.rx_multicast;
	stats->rx_crc_errors = hw.rx_fcs_align_errors;
	/* Fragments and jabbers have a wrong length and a bad FCS: length */
	stats->rx_length_errors = hw.rx_runts + hw.rx_runts_fcs +
				  hw.rx_oversize + hw.rx_oversize_fcs +
				  jabber + l2_length;
	stats->rx_missed_errors = hw.rx_dropped_packets;
	/*
	 * The GMX counts the frames its FIFOs lost; the error bits are events
	 */
	stats->rx_fifo_errors = gmx.rx_drop_packets;
	/* The PIP's error classes are by frame size, each frame in one */
	stats->rx_errors = hw.rx_fcs_align_errors + hw.rx_runts +
			   hw.rx_runts_fcs + hw.rx_oversize +
			   hw.rx_oversize_fcs + stats->rx_fifo_errors +
			   jabber + l2_length +
			   atomic_long_read(&p->rx_other_errors);
	stats->tx_fifo_errors = gmx.tx_underflow;
	stats->tx_aborted_errors = gmx.tx_excessive_collisions;
	stats->tx_errors = stats->tx_fifo_errors + stats->tx_aborted_errors;
	stats->collisions = gmx.tx_single_collisions +
			    gmx.tx_multiple_collisions;
}

static void opip_get_queue_stats_tx(struct net_device *dev, int idx,
				    struct netdev_queue_stats_tx *stats)
{
	stats->stop = READ_ONCE(opip_port(dev)->txq[idx].stopped);
}

static void opip_get_base_stats(struct net_device *dev,
				struct netdev_queue_stats_rx *rx,
				struct netdev_queue_stats_tx *tx)
{
	tx->stop = 0;
}

static const struct netdev_stat_ops opip_stat_ops = {
	.get_queue_stats_tx	= opip_get_queue_stats_tx,
	.get_base_stats		= opip_get_base_stats,
};

/* The pools, and the IPD's view of the packet pool: RED decides on it */
static void opip_debugfs_pools(struct seq_file *m)
{
	unsigned int fpa = opip_fpa_level(OPIP_POOL_PACKET);
	unsigned int ipd = opip_ipd_pool_count();

	seq_printf(m, "fpa packet buffers %u\n", fpa);
	seq_printf(m, "fpa wqe buffers %u\n", opip_fpa_level(OPIP_POOL_WQE));
	seq_printf(m, "fpa output buffers %u\n", opip_fpa_level(OPIP_POOL_CMD));
	seq_printf(m, "ipd packet buffers %u (difference %d)\n",
		   ipd, (int)(fpa - ipd));
}

/*
 * Work the cores have not taken yet: the backlog of each group, the
 * groups whose interrupt is raised, and the input queue of each QOS level
 * (a port's level is port & 7).
 */
static void opip_debugfs_pow(struct seq_file *m, struct opip *opip)
{
	int group, i;

	seq_printf(m, "pow queued %u groups", opip_pow_queued(-1));
	for (group = 0; group < OPIP_MAX_GROUPS; group++)
		if (opip->groups_mask & BIT(group))
			seq_printf(m, " %d:%u", group, opip_pow_queued(group));
	seq_printf(m, " int %#x qos", opip_pow_int_pending());
	for (i = 0; i < 8; i++)
		seq_printf(m, " %d:%u", i, opip_pow_qos_queued(i));
	seq_puts(m, "\n");
}

/* The backpressure state of the GMX interfaces: a bit per port in each field */
static void opip_debugfs_gmx(struct seq_file *m, struct opip *opip)
{
	int i, index;

	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];
		struct opip_gmx_bp bp;

		if (iface->num >= 2 || !opip_iface_is_gmx(iface))
			continue;
		opip_gmx_bp_state(iface->num, iface->num_ports, &bp);
		seq_printf(m, "gmx%d tx_ovr_bp en %#x bp %#x ign_full %#x rx_prt_info commit %#x drop %#x marks",
			   iface->num, bp.ovr_en, bp.ovr_bp, bp.ign_full,
			   bp.rx_commit, bp.rx_drop);
		for (index = 0; index < min(iface->num_ports, 4); index++)
			seq_printf(m, " %u/%u/%u", bp.bp_on[index],
				   bp.bp_off[index], bp.bp_drop[index]);
		seq_puts(m, "\n");
	}
}

/*
 * The PCS of a port: in band or out of band (PCS_MR_CONTROL[AN_EN]) and
 * the MAC side. The link status only for SGMII and QSGMII out of band:
 * PCS_MR_STATUS[LNK_ST] latches low and a read clears it, which phylink's
 * poll must see in band and on 1000BASE-X. In band the autonegotiation's
 * result and what the partner sent; out of band PCS_AN_RESULTS is
 * undefined.
 */
static void opip_debugfs_pcs(struct seq_file *m, struct net_device *dev)
{
	static const unsigned int speeds[] = { 10, 100, 1000, 0 };
	u64 ctl, st = 0, res = 0, lp = 0, misc;
	struct opip_port *p = opip_port(dev);
	int iface = p->iface->num;
	int index = p->index;
	bool inband, basex;

	mutex_lock(&p->opip->hw_lock);
	ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));
	misc = opip_rd(OPIP_PCS_MISC_CTL(iface, index));
	inband = ctl & OPIP_PCS_MR_CONTROL_AN_EN;
	basex = misc & OPIP_PCS_MISC_CTL_MODE;
	if (inband) {
		res = opip_rd(OPIP_PCS_AN_RESULTS(iface, index));
		lp = opip_rd(OPIP_PCS_AN_LP_ABIL(iface, index));
	} else if (!basex) {
		st = opip_rd(OPIP_PCS_MR_STATUS(iface, index));
	}
	mutex_unlock(&p->opip->hw_lock);

	seq_printf(m, "%s port %d pcs %s %s control %#llx", dev->name, p->port,
		   phy_modes(p->phy_mode), inband ? "in-band" : "out-of-band",
		   ctl);
	if (!inband && !basex)
		seq_printf(m, " status %#llx link %s", st,
			   st & OPIP_PCS_MR_STATUS_LNK_ST ? "up" : "down");
	seq_printf(m, " misc %#llx %s gmxeno %d samp_pt %llu\n", misc,
		   misc & OPIP_PCS_MISC_CTL_MAC_PHY ? "phy-side" : "mac-side",
		   !!(misc & OPIP_PCS_MISC_CTL_GMXENO),
		   FIELD_GET(OPIP_PCS_MISC_CTL_SAMP_PT, misc));
	if (!inband)
		return;
	/* 1000BASE-X: a clause 37 word, no SGMII speed */
	if (basex) {
		seq_printf(m, "%s port %d pcs an results %#llx lp %#llx\n",
			   dev->name, p->port, res, lp);
		return;
	}
	seq_printf(m, "%s port %d pcs an results %#llx link_ok %d an_complete %d speed %u %s lp %#llx\n",
		   dev->name, p->port, res,
		   !!(res & OPIP_PCS_AN_RESULTS_LINK_OK),
		   !!(res & OPIP_PCS_AN_RESULTS_AN_CPT),
		   speeds[FIELD_GET(OPIP_PCS_AN_RESULTS_SPD, res)],
		   res & OPIP_PCS_AN_RESULTS_DUP ? "full" : "half", lp);
}

/*
 * Per port: the packets in flight on each transmit queue, the commands the
 * PKO has been told of and not fetched yet, the IPD's backpressure count
 * of the GMX ports, and the PCS. Under rtnl: the devices and their queue
 * counts.
 */
static void opip_debugfs_port(struct seq_file *m, struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int qid;

	seq_printf(m, "%s port %d queues %d-%d inflight", dev->name, p->port,
		   p->base_queue, p->base_queue + dev->real_num_tx_queues - 1);
	for (qid = 0; qid < dev->real_num_tx_queues; qid++)
		seq_printf(m, " %u", READ_ONCE(p->txq[qid].tail) -
			   READ_ONCE(p->txq[qid].head));
	seq_puts(m, "\n");

	seq_printf(m, "%s port %d pko words", dev->name, p->port);
	for (qid = 0; qid < dev->real_num_tx_queues; qid++)
		seq_printf(m, " %u",
			   opip_pko_queue_pending(p->opip, p->txq[qid].queue));
	seq_puts(m, "\n");

	if (opip_port_has_gmx(p)) {
		unsigned int limit;
		bool on;

		on = opip_ipd_bp_limit(p->port, &limit);

		seq_printf(m, "%s port %d ipd buffers %u backpressure %s limit %u\n",
			   dev->name, p->port, opip_ipd_bp_count(p->port),
			   on ? "on" : "off", limit);
	}
	if (opip_port_has_pcs(p))
		opip_debugfs_pcs(m, dev);
}

static struct dentry *opip_debugfs_root;

/* The state of the blocks, levels rather than counters, for a bug report */
static int opip_debugfs_show(struct seq_file *m, void *v)
{
	struct opip *opip = m->private;
	int port;

	opip_debugfs_pools(m);
	opip_debugfs_pow(m, opip);
	/* PKO_REG_ERROR: parity, bad doorbell, zero buffer pointer */
	seq_printf(m, "pko error %#llx\n", opip_pko_error());
	opip_debugfs_gmx(m, opip);

	rtnl_lock();
	for (port = 0; port < OPIP_MAX_PORTS; port++)
		if (opip->ports[port])
			opip_debugfs_port(m, opip->ports[port]);
	rtnl_unlock();
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(opip_debugfs);

/* ------------------------------------------------------------------ */
/* The periodic worker of a port */

static void opip_count_error(struct opip_port *p, unsigned long *counter,
			     const char *what)
{
	WRITE_ONCE(*counter, *counter + 1);
	if (net_ratelimit())
		netdev_warn(p->netdev, "%s\n", what);
}

/*
 * Error conditions of the GMX and the PCS that should not happen. The
 * FIFO overruns and underflows are not among them: the GMX counts the
 * frames they cost (rx_fifo_errors, tx_fifo_errors).
 */
static void opip_poll_errors(struct opip_port *p)
{
	int i, iface = p->iface->num;
	unsigned int errors;
	bool nxa = true;

	if (!opip_port_has_gmx(p))
		return;

	/* The interface's PKO_NXA goes to its first port with a device */
	for (i = 0; i < p->index; i++)
		if (READ_ONCE(p->opip->ports[opip_port_number(iface, i)]))
			nxa = false;
	errors = opip_gmx_poll_errors(iface, p->index, nxa);
	if (errors & OPIP_GMX_ERR_PKO_NXA)
		opip_count_error(p, &p->tx_pko_nxa,
				 "PKO used a nonexistent queue of the interface");

	if (!opip_port_has_pcs(p))
		return;
	errors = opip_pcs_poll_errors(p->iface->num, p->index);
	if (errors & OPIP_PCS_ERR_AN)
		opip_count_error(p, &p->pcs_an_error,
				 "PCS autonegotiation error");
	if (errors & OPIP_PCS_ERR_TX_FIFO)
		opip_count_error(p, &p->pcs_tx_fifo_error,
				 "PCS transmit FIFO error");
}

static void opip_periodic_worker(struct work_struct *work)
{
	struct opip_port *p = container_of(work, struct opip_port,
					   periodic_work.work);

	if (READ_ONCE(p->preamble_polling))
		opip_check_preamble_errors(p->netdev);
	opip_poll_errors(p);
	opip_update_hw_stats(p->netdev);

	if (!READ_ONCE(p->opip->stopping))
		schedule_delayed_work(&p->periodic_work, HZ);
}

static void opip_refill_worker(struct work_struct *work)
{
	struct opip *opip = container_of(work, struct opip, refill_work.work);

	opip_rx_refill_check(opip);
	if (!READ_ONCE(opip->stopping))
		schedule_delayed_work(&opip->refill_work, HZ);
}

/* ------------------------------------------------------------------ */
/* The network device operations */

/*
 * The receive frame limits of a port for an MTU. The jabber limit is per
 * port. The PIP's length check of the later chips has two checkers: CN68XX
 * picks one per port kind, the interface's (opip_pip_port_config()); on
 * the other chips the vendor's code takes checker 0 to apply to all
 * incoming traffic and leaves checker 1 unused. Both get the largest MTU
 * of the ports they may check: the interface's on CN68XX, all GMX ports
 * otherwise.
 */
static void opip_set_hw_mtu(struct net_device *dev, int new_mtu)
{
	unsigned int max_frame = new_mtu + OPIP_MTU_OVERHEAD;
	struct opip_port *p = opip_port(dev);

	if (!opip_port_has_gmx(p))
		return;

	opip_gmx_set_frame_max(p->iface->num, p->index, max_frame);

	if (!opip_chip->gmx_frm_max) {
		unsigned int max_len = max_frame;
		int port;

		for (port = 0; port < OPIP_MAX_PORTS; port++) {
			struct net_device *other = p->opip->ports[port];

			if (!other || other == dev ||
			    !opip_port_has_gmx(opip_port(other)) ||
			    (opip_chip->cn68xx &&
			     opip_port(other)->iface != p->iface))
				continue;
			max_len = max(max_len, READ_ONCE(other->mtu) +
				      OPIP_MTU_OVERHEAD);
		}
		if (opip_chip->cn68xx) {
			opip_pip_frame_length(p->iface->num, VLAN_ETH_ZLEN,
					      max_len);
		} else {
			opip_pip_frame_length(0, VLAN_ETH_ZLEN, max_len);
			opip_pip_frame_length(1, VLAN_ETH_ZLEN, max_len);
		}
	}
}

/*
 * The largest MTU for an XDP program: a packet in one buffer, or for a
 * program that handles multi-buffer packets one that every verdict can
 * take: the first buffer after its skip and the later ones whole, as
 * many as XDP_TX's gather list holds (16 buffers, 32362 bytes) and an
 * xdp_buff has frags for
 */
static unsigned int opip_xdp_max_mtu(const struct bpf_prog *prog)
{
	unsigned int buf = OPIP_POOL_PACKET_SIZE - OPIP_BUF_LINK_SIZE;
	unsigned int later = min_t(unsigned int, MAX_SKB_FRAGS,
				   OPIP_TX_MAX_SEGS - 1);

	if (!prog->aux->xdp_has_frags)
		return OPIP_XDP_MAX_MTU;
	return min(OPIP_XDP_MAX_MTU + later * buf,
		   OPIP_MAX_FRAME - OPIP_MTU_OVERHEAD);
}

static int opip_change_mtu(struct net_device *dev, int new_mtu)
{
	struct bpf_prog *prog = rtnl_dereference(opip_port(dev)->xdp_prog);

	if (prog && new_mtu > opip_xdp_max_mtu(prog)) {
		netdev_warn(dev, "MTU %d is too large for the XDP program, %u at most\n",
			    new_mtu, opip_xdp_max_mtu(prog));
		return -EINVAL;
	}
	WRITE_ONCE(dev->mtu, new_mtu);
	opip_set_hw_mtu(dev, new_mtu);
	return 0;
}

/* Attach or detach the XDP program of a port, under rtnl */
static int opip_xdp_setup(struct net_device *dev, struct netdev_bpf *bpf)
{
	struct opip_port *p = opip_port(dev);
	struct bpf_prog *old;

	if (bpf->command != XDP_SETUP_PROG)
		return -EINVAL;
	if (bpf->prog && dev->mtu > opip_xdp_max_mtu(bpf->prog)) {
		unsigned int max = opip_xdp_max_mtu(bpf->prog);

		if (bpf->prog->aux->xdp_has_frags)
			NL_SET_ERR_MSG_FMT_MOD(bpf->extack,
					       "MTU %u too large for XDP, %u at most (XDP_TX: %zu buffers)",
					       dev->mtu, max,
					       OPIP_TX_MAX_SEGS);
		else
			NL_SET_ERR_MSG_FMT_MOD(bpf->extack,
					       "MTU %u too large for single-buffer XDP, %u at most",
					       dev->mtu, max);
		return -EOPNOTSUPP;
	}
	old = rtnl_dereference(p->xdp_prog);
	/*
	 * The polls run their XDP instance (opip_poll()) from before a first
	 * program is visible to them until after it is gone: the key goes on
	 * and the polls running without it end before the program is set.
	 */
	if (bpf->prog && !old) {
		static_branch_inc(&opip_rx_xdp_key);
		synchronize_net();
	}
	rcu_assign_pointer(p->xdp_prog, bpf->prog);
	/* bpf_prog_put() frees after a grace period, past the polls (BH off) */
	if (old)
		bpf_prog_put(old);
	/*
	 * The key may go off at once: a poll still in the XDP instance runs
	 * the old program or none, and bpf_prog_put() waits for it
	 */
	if (old && !bpf->prog)
		static_branch_dec(&opip_rx_xdp_key);
	return 0;
}

/* The receive groups are the XDP receive queues of every port */
static void opip_xdp_rxq_unregister(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	int i;

	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		if (xdp_rxq_info_is_reg(&p->xdp_rxq[i]))
			xdp_rxq_info_unreg(&p->xdp_rxq[i]);
}

static int opip_xdp_rxq_register(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	struct opip *opip = p->opip;
	int i, ret;

	for (i = 0; i < OPIP_MAX_GROUPS; i++) {
		struct opip_group *group = &opip->groups[i];

		if (!(opip->groups_mask & BIT(i)))
			continue;
		/* bpf_xdp_adjust_tail() may grow a frag to its whole buffer */
		ret = __xdp_rxq_info_reg(&p->xdp_rxq[i], dev, i,
					 group->napi.napi_id,
					 OPIP_RX_FRAG_SIZE);
		if (!ret)
			ret = xdp_rxq_info_reg_mem_model(&p->xdp_rxq[i],
							 MEM_TYPE_PAGE_POOL,
							 group->pp);
		if (ret) {
			opip_xdp_rxq_unregister(dev);
			return ret;
		}
	}
	return 0;
}

static void opip_rx_filter_flags(struct net_device *dev, bool *promisc,
				 bool *allmulti)
{
	netif_addr_lock_bh(dev);
	*promisc = dev->flags & IFF_PROMISC;
	*allmulti = (dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev);
	netif_addr_unlock_bh(dev);
}

static void opip_rx_mode_worker(struct work_struct *work)
{
	struct opip_port *p = container_of(work, struct opip_port,
					   rx_mode_work);
	bool promisc, allmulti;

	mutex_lock(&p->opip->hw_lock);
	opip_rx_filter_flags(p->netdev, &promisc, &allmulti);
	opip_gmx_set_rx_filter(p->iface->num, p->index, promisc, allmulti);
	mutex_unlock(&p->opip->hw_lock);
}

/* Called in atomic context: a work item programs the registers */
static void opip_set_rx_mode(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);

	if (opip_port_has_gmx(p))
		schedule_work(&p->rx_mode_work);
}

static void opip_set_mac_filter(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	bool promisc, allmulti;

	if (!opip_port_has_gmx(p))
		return;
	/*
	 * The flags are read under hw_lock, here and in the worker: an
	 * address list change between reading and writing them is then
	 * written by whichever of the two comes second
	 */
	mutex_lock(&p->opip->hw_lock);
	opip_rx_filter_flags(dev, &promisc, &allmulti);
	opip_gmx_set_mac(p->iface->num, p->index, dev->dev_addr);
	opip_gmx_set_rx_filter(p->iface->num, p->index, promisc, allmulti);
	mutex_unlock(&p->opip->hw_lock);
}

static int opip_set_mac_address(struct net_device *dev, void *addr)
{
	int ret = eth_mac_addr(dev, addr);

	if (ret)
		return ret;
	opip_set_mac_filter(dev);
	return 0;
}

/*
 * The port's hardware is set up for the device; it stays disabled until opened
 */
static int opip_ndo_init(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int i;
	int ret;

	ret = opip_xdp_rxq_register(dev);
	if (ret)
		return ret;

	/*
	 * Receive queue i is group i, whose NAPI is on opip->napi_dev; each
	 * transmit queue has its own. Both outlive the registration.
	 */
	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		if (p->opip->groups_mask & BIT(i))
			netif_queue_set_napi(dev, i, NETDEV_QUEUE_TYPE_RX,
					     &p->opip->groups[i].napi);
	for (i = 0; i < dev->real_num_tx_queues; i++)
		netif_queue_set_napi(dev, i, NETDEV_QUEUE_TYPE_TX,
				     &p->txq[i].napi);

	opip_set_mac_filter(dev);
	opip_set_hw_mtu(dev, dev->mtu);

	/* Don't show the counters of the bootloader: read and drop them */
	if (opip_port_has_gmx(p) && !opip_on_simulator()) {
		mutex_lock(&p->opip->hw_lock);
		opip_gmx_stats_init(p->iface->num, p->index);
		mutex_unlock(&p->opip->hw_lock);
	}
	opip_update_hw_stats(dev);
	memset(&p->hw_stats, 0, sizeof(p->hw_stats));
	memset(&p->gmx_stats, 0, sizeof(p->gmx_stats));

	opip_port_enable(p, false);
	return 0;
}

static void opip_ndo_uninit(struct net_device *dev)
{
	opip_xdp_rxq_unregister(dev);
}

static int opip_ndo_open(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	int ret;

	opip_tx_resume(dev);

	/* NPI and LOOP: no MAC or link to bring up */
	if (!opip_iface_is_gmx(p->iface))
		return 0;

	/* The simulator has no link: the port keeps the carrier on */
	if (!p->phylink) {
		opip_port_enable(p, true);
		return 0;
	}

	ret = phylink_of_phy_connect(p->phylink, p->of_node, 0);
	if (ret) {
		netdev_err(dev, "could not attach the PHY: %d\n", ret);
		opip_tx_stop(dev);
		return ret;
	}
	opip_port_enable(p, true);
	phylink_start(p->phylink);
	/* Only then the periodic worker may check for preamble errors */
	if (opip_port_rgmii_preamble(p))
		WRITE_ONCE(p->preamble_polling, true);
	return 0;
}

static int opip_ndo_stop(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);

	/*
	 * XDP sends (XDP_TX from the shared receive polls, ndo_xdp_xmit
	 * from other devices) test netif_running() without the qdisc: wait
	 * for those that saw it set before the queues are drained and reset
	 */
	synchronize_net();

	if (!opip_iface_is_gmx(p->iface)) {
		/* The packets in flight reference the device */
		opip_tx_drain(dev);
		opip_tx_stop(dev);
		return 0;
	}

	/* No preamble check while the port goes down */
	WRITE_ONCE(p->preamble_polling, false);
	flush_delayed_work(&p->periodic_work);

	/* The stack no longer transmits: the PKO sends what it has */
	opip_tx_drain(dev);
	opip_tx_stop(dev);

	/*
	 * With the MAC still on: the link goes down through mac_link_down(),
	 * which also stops the PAUSE frames that hold the partner off
	 */
	if (p->phylink) {
		phylink_stop(p->phylink);
		phylink_disconnect_phy(p->phylink);
	}
	opip_port_enable(p, false);
	return 0;
}

static int opip_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct opip_port *p = opip_port(dev);

	if (!netif_running(dev) || !p->phylink)
		return -ENODEV;
	return phylink_mii_ioctl(p->phylink, rq, cmd);
}

static const struct net_device_ops opip_netdev_ops = {
	.ndo_init		= opip_ndo_init,
	.ndo_uninit		= opip_ndo_uninit,
	.ndo_open		= opip_ndo_open,
	.ndo_stop		= opip_ndo_stop,
	.ndo_start_xmit		= opip_xmit,
	.ndo_tx_timeout		= opip_tx_timeout,
	.ndo_features_check	= opip_features_check,
	.ndo_set_rx_mode	= opip_set_rx_mode,
	.ndo_set_mac_address	= opip_set_mac_address,
	.ndo_eth_ioctl		= opip_ioctl,
	.ndo_change_mtu		= opip_change_mtu,
	.ndo_bpf		= opip_xdp_setup,
	.ndo_xdp_xmit		= opip_xdp_xmit,
	.ndo_get_stats64	= opip_get_stats64,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= opip_tx_netpoll,
#endif
};

/* ------------------------------------------------------------------ */
/* The interfaces and their ports */

static struct device_node *opip_of_child(const struct device_node *parent,
					 int reg)
{
	struct device_node *node;

	for_each_child_of_node(parent, node) {
		u32 val;

		if (!of_property_read_u32(node, "reg", &val) && val == reg)
			return node;
	}
	return NULL;
}

/* The device tree node of a port: pip -> interface -> port */
static struct device_node *opip_port_of_node(struct device_node *pip, int iface,
					     int index)
{
	struct device_node *ni = opip_of_child(pip, iface), *np;

	if (!ni)
		return NULL;
	np = opip_of_child(ni, index);
	of_node_put(ni);
	return np;
}

/* The ASX's clock delay steps: 0 to 31 */
#define OPIP_ASX_CLK_SET_MAX	31

/* A clock delay of an RGMII port from the device tree; -1 if none */
static int opip_rgmii_delay(struct opip_port *p, const char *prop)
{
	u32 val;

	if (!p->of_node || of_property_read_u32(p->of_node, prop, &val))
		return -1;
	return min(val, OPIP_ASX_CLK_SET_MAX);
}

/* Whether a port's node has a phy-mode, valid or not */
static bool opip_has_phy_mode(const struct device_node *np)
{
	return np && (of_property_present(np, "phy-mode") ||
		      of_property_present(np, "phy-connection-type"));
}

/*
 * The PHY mode and the clock delays of an RGMII port. phy-mode says which
 * side of the link adds the delays: the PHY those of its mode, the board
 * the others, the MAC none. rx-delay and tx-delay, steps of the ASX, may
 * fine-tune the MAC's delay on a side the PHY leaves alone. A device
 * tree without phy-mode (the platform code's and the bootloaders' are
 * all like that) is read as the staging driver reads it: the MAC adds a
 * delay that the properties do not set to 0, with the chip's default
 * step unless they give one, and the PHY the others.
 *
 * Return: 0, or -EINVAL if the port cannot run what the node asks for
 */
static int opip_rgmii_config(struct opip_port *p)
{
	struct device *dev = &p->opip->pdev->dev;
	int rx = opip_rgmii_delay(p, "rx-delay");
	int tx = opip_rgmii_delay(p, "tx-delay");
	struct device_node *np = p->of_node;
	phy_interface_t mode;

	/* The time a step of the ASX takes is not documented */
	if (np && (of_property_present(np, "rx-internal-delay-ps") ||
		   of_property_present(np, "tx-internal-delay-ps"))) {
		dev_err(dev, "%pOF: rx/tx-internal-delay-ps not supported, the MAC delay is a step of 0 to %d\n",
			np, OPIP_ASX_CLK_SET_MAX);
		return -EINVAL;
	}

	if (!opip_has_phy_mode(np)) {
		if (rx && tx)
			p->phy_mode = PHY_INTERFACE_MODE_RGMII;
		else if (tx)
			p->phy_mode = PHY_INTERFACE_MODE_RGMII_RXID;
		else if (rx)
			p->phy_mode = PHY_INTERFACE_MODE_RGMII_TXID;
		else
			p->phy_mode = PHY_INTERFACE_MODE_RGMII_ID;
		p->rgmii_rx_delay = rx;
		p->rgmii_tx_delay = tx;
		dev_warn(dev, "port %d: no phy-mode, taking %s from rx-delay/tx-delay; update the device tree\n",
			 p->port, phy_modes(p->phy_mode));
		return 0;
	}

	if (of_get_phy_mode(np, &mode) || !phy_interface_mode_is_rgmii(mode)) {
		dev_err(dev, "%pOF: not an RGMII phy-mode on an RGMII port\n",
			np);
		return -EINVAL;
	}
	p->phy_mode = mode;

	/* Not a delay on a side the PHY adds it on too */
	if (rx > 0 && (mode == PHY_INTERFACE_MODE_RGMII_ID ||
		       mode == PHY_INTERFACE_MODE_RGMII_RXID)) {
		dev_warn(dev, "%pOF: rx-delay %d with phy-mode %s, which delays the receive clock in the PHY: using 0\n",
			 np, rx, phy_modes(mode));
		rx = 0;
	}
	if (tx > 0 && (mode == PHY_INTERFACE_MODE_RGMII_ID ||
		       mode == PHY_INTERFACE_MODE_RGMII_TXID)) {
		dev_warn(dev, "%pOF: tx-delay %d with phy-mode %s, which delays the transmit clock in the PHY: using 0\n",
			 np, tx, phy_modes(mode));
		tx = 0;
	}
	/* The MAC adds no delay unless the properties tune one */
	p->rgmii_rx_delay = max(rx, 0);
	p->rgmii_tx_delay = max(tx, 0);
	return 0;
}

/*
 * Port 0 of CN30XX and CN50XX runs MII when GMX_INF_MODE[P0MII] is set,
 * on an RGMII and on a GMII interface; the bit is reserved on the other
 * chips.
 */
static bool opip_port_p0mii(const struct opip_port *p)
{
	return p->index == 0 && opip_chip->rgmii_mii_clk &&
	       (opip_rd(OPIP_GMX_INF_MODE(p->iface->num)) &
		OPIP_GMX_INF_MODE_P0MII);
}

/*
 * A GMII or MII port: hw is what the port runs. Port 0 of a GMII
 * interface is an RGMII port unless P0MII makes it MII. A GMII port runs
 * MII at 10 and 100 Mbit/s, so "mii" describes one wired to an MII-only
 * PHY.
 */
static int opip_mii_config(struct opip_port *p, phy_interface_t hw)
{
	struct device_node *np = p->of_node;
	phy_interface_t mode;

	p->phy_mode = hw;
	if (!opip_has_phy_mode(np))
		return 0;
	if (of_get_phy_mode(np, &mode) ||
	    (mode != hw && mode != PHY_INTERFACE_MODE_MII)) {
		dev_err(&p->opip->pdev->dev, "%pOF: phy-mode not %s%s on a%s port\n",
			np, phy_modes(hw),
			hw == PHY_INTERFACE_MODE_GMII ? " or mii" : "",
			hw == PHY_INTERFACE_MODE_GMII ? " GMII" : "n MII");
		return -EINVAL;
	}
	p->phy_mode = mode;
	return 0;
}

/*
 * The SGMII interface mode covers QSGMII and 1000BASE-X as well. The PHY
 * driver needs to know which: it configures the MAC side of the PHY
 * accordingly. The hardware knows (the bootloader set it up), the device
 * tree is only checked against it.
 */
static void opip_sgmii_phy_mode(struct opip_port *p)
{
	phy_interface_t dt_mode;

	if (opip_pcs_basex(p->iface->num, p->index))
		p->phy_mode = PHY_INTERFACE_MODE_1000BASEX;
	else if (p->iface->qsgmii)
		p->phy_mode = PHY_INTERFACE_MODE_QSGMII;
	else
		p->phy_mode = PHY_INTERFACE_MODE_SGMII;
	if (p->of_node && !of_get_phy_mode(p->of_node, &dt_mode) &&
	    dt_mode != p->phy_mode)
		dev_warn(&p->opip->pdev->dev, "%pOF: phy-mode %s, but interface %d runs %s\n",
			 p->of_node, phy_modes(dt_mode), p->iface->num,
			 phy_modes(p->phy_mode));
}

/*
 * Give every CPU its own transmit queue: queue n gets the CPUs whose
 * number modulo the queue count is n.
 */
static void opip_setup_xps(struct net_device *dev)
{
	unsigned int num_queues = dev->real_num_tx_queues, qid;
	cpumask_var_t mask;
	int cpu;

	if (num_queues < 2 || !zalloc_cpumask_var(&mask, GFP_KERNEL))
		return;
	for (qid = 0; qid < num_queues; qid++) {
		cpumask_clear(mask);
		for_each_possible_cpu(cpu)
			if (cpu % num_queues == qid)
				cpumask_set_cpu(cpu, mask);
		netif_set_xps_queue(dev, mask, qid);
	}
	free_cpumask_var(mask);
}

/*
 * Release what opip_alloc_port() took. The device's own queue count is
 * gone once it is unregistered: the port's is used. A drain that gave up
 * left packets in the queues, the PKO reset since: free them. Lost are
 * the chunks the PKO still held but the current one, and the buffers of
 * XDP_TX packets: only the PKO frees those, and the one its engine had
 * taken it may have freed already. The pools' counts warn of them.
 */
static void opip_put_port(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	int qid;

	if (p->phylink)
		phylink_destroy(p->phylink);
	for (qid = 0; qid < p->num_queues; qid++) {
		opip_txq_free(&p->txq[qid]);
		opip_txq_purge(&p->txq[qid]);
		opip_pko_queue_release(p->opip, &p->txq[qid]);
	}
	of_node_put(p->of_node);
	free_netdev(dev);
}

/*
 * A fragmented packet in flight holds a gather list in a buffer of the
 * WQE pool until the PKO has sent it, so grow the pool by what the
 * transmit queues of the device can hold before it can send, leaving the
 * receive side its share. The same for the command chunks that full
 * queues take up.
 */
static void opip_port_grow_pools(struct opip *opip, struct net_device *dev)
{
	unsigned int queues = dev->real_num_tx_queues;

	opip->wqe_bufs += opip_fpa_fill(opip, OPIP_POOL_WQE, OPIP_POOL_WQE_SIZE,
					queues * OPIP_TXQ_DEPTH);
	opip->cmd_bufs += opip_fpa_fill(opip, OPIP_POOL_CMD, OPIP_POOL_CMD_SIZE,
					queues * OPIP_TXQ_CMD_CHUNKS);
}

/*
 * A net_device for a port, with its queues set up at the PKO (which is
 * still disabled) and phylink created. Failures only cost the port.
 */
static struct net_device *opip_alloc_port(struct opip *opip,
					  struct device_node *pip,
					  struct opip_iface *iface, int index)
{
	int port = opip_port_number(iface->num, index);
	const char *name = "eth%d";
	struct net_device *dev;
	struct device_node *np;
	int num_queues, qid;
	struct opip_port *p;

	/* Sending on a port without queues corrupts memory */
	if (port < 0 || !opip_pko_port_valid(opip, port)) {
		dev_warn(&opip->pdev->dev, "interface %d port %d has no output queues, not used\n",
			 iface->num, index);
		return NULL;
	}

	/* A port the device tree disables has nothing connected */
	np = opip_port_of_node(pip, iface->num, index);
	if (np && !of_device_is_available(np)) {
		of_node_put(np);
		return NULL;
	}

	/* A receive queue per group: the queue index of its XDP packets */
	num_queues = clamp(opip_pko_num_queues(opip, port), 1, OPIP_MAX_TXQS);
	dev = alloc_etherdev_mqs(sizeof(*p), num_queues,
				 hweight32(opip->groups_mask));
	if (!dev) {
		of_node_put(np);
		return NULL;
	}
	SET_NETDEV_DEV(dev, &opip->pdev->dev);

	p = opip_port(dev);
	p->opip = opip;
	p->iface = iface;
	p->netdev = dev;
	p->port = port;
	p->index = index;
	p->base_queue = opip_pko_base_queue(opip, port);
	p->num_queues = num_queues;
	p->of_node = np;
	dev->dev.of_node = np;
	p->phy_mode = PHY_INTERFACE_MODE_NA;
	INIT_DELAYED_WORK(&p->periodic_work, opip_periodic_worker);
	INIT_WORK(&p->rx_mode_work, opip_rx_mode_worker);
	spin_lock_init(&p->stats_lock);
	netif_set_real_num_tx_queues(dev, min_t(int, num_queues,
						num_possible_cpus()));
	dev->min_mtu = VLAN_ETH_ZLEN - OPIP_MTU_OVERHEAD;
	dev->max_mtu = OPIP_MAX_FRAME - OPIP_MTU_OVERHEAD;
	dev->netdev_ops = &opip_netdev_ops;
	dev->ethtool_ops = &opip_ethtool_ops;
	dev->stat_ops = &opip_stat_ops;
	/*
	 * The gather list of a packet sits in its skb->cb and a transmit may
	 * linearize or pad the skb: none may be sent again while in flight
	 * (pktgen's clone_skb and burst)
	 */
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->hw_features = NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_IP_CSUM |
			   NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM | NETIF_F_RXHASH;
	dev->features |= dev->hw_features;
	dev->vlan_features |= dev->hw_features;
	dev->hw_enc_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	xdp_set_features_flag(dev, NETDEV_XDP_ACT_BASIC |
			      NETDEV_XDP_ACT_REDIRECT |
			      NETDEV_XDP_ACT_NDO_XMIT |
			      NETDEV_XDP_ACT_RX_SG);

	/*
	 * opip_probe() has waited for the nvmem cells: a failure here is final
	 */
	if (of_get_ethdev_address(np, dev)) {
		eth_hw_addr_random(dev);
		if (np)
			dev_warn(&opip->pdev->dev, "%pOF: no MAC address, using a random one\n",
				 np);
	}

	switch (iface->mode) {
	case OPIP_MODE_NPI:
		name = "npi%d";
		break;
	case OPIP_MODE_XAUI:
		name = "xaui%d";
		p->phy_mode = PHY_INTERFACE_MODE_XAUI;
		break;
	case OPIP_MODE_LOOP:
		name = "loop%d";
		break;
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		opip_sgmii_phy_mode(p);
		break;
	case OPIP_MODE_GMII:
	case OPIP_MODE_RGMII:
		if (opip_port_p0mii(p)) {
			if (opip_mii_config(p, PHY_INTERFACE_MODE_MII))
				goto err_put;
		} else if (iface->mode == OPIP_MODE_GMII && p->index != 0) {
			if (opip_mii_config(p, PHY_INTERFACE_MODE_GMII))
				goto err_put;
		} else if (opip_rgmii_config(p)) {
			goto err_put;
		}
		break;
	default:
		goto err_put;
	}
	/*
	 * The name templates of the staging driver, which its users have
	 * had since 2009: npi%d, xaui%d and loop%d, eth%d for the others
	 */
	strscpy(dev->name, name, sizeof(dev->name));
	/* The port's node and its IPD/PKO port number, for user space */
	device_set_node(&dev->dev, of_fwnode_handle(np));
	dev->dev_port = port;

	/* The queues: the PKO's view of them and their first chunks */
	for (qid = 0; qid < num_queues; qid++) {
		struct opip_txq *q = &p->txq[qid];

		q->queue = p->base_queue + qid;
		opip_txq_init(dev, qid);
		if (opip_pko_queue_init(opip, q, port, qid, num_queues)) {
			dev_err(&opip->pdev->dev, "no command chunk for queue %d\n",
				q->queue);
			goto err_put;
		}
	}

	if (opip_iface_is_gmx(iface) && !opip_on_simulator() &&
	    opip_phylink_create(p)) {
		dev_err(&opip->pdev->dev, "failed to set up the link of interface %d port %d\n",
			iface->num, index);
		goto err_put;
	}

	opip_port_grow_pools(opip, dev);
	return dev;

err_put:
	opip_put_port(dev);
	return NULL;
}

static const char * const opip_mode_names[] = {
	"DISABLED", "RGMII", "GMII", "SPI", "PCIE", "XAUI", "SGMII", "PICMG",
	"NPI", "LOOP",
};

/*
 * The ports of an interface, counted as the executive's enumeration does
 * but without its board quirks: the platform code removes the device tree
 * nodes of the ports a board lacks, see opip_alloc_ports().
 */
static int opip_iface_ports(int iface, enum opip_mode mode)
{
	switch (mode) {
	case OPIP_MODE_RGMII:
		return opip_chip->rgmii_ports;
	case OPIP_MODE_GMII:
		/* GMII/MII mode of CN30XX, CN31XX and CN50XX */
		return 2;
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		if (!opip_chip->qsgmii)
			return 4;
		/* CN70XX: one SGMII port, or four QSGMII ports */
		switch (FIELD_GET(OPIP_GMX_INF_MODE_MODE,
				  opip_rd(OPIP_GMX_INF_MODE(iface)))) {
		case OPIP_GMX_INF_MODE_MODE_SGMII:
			return 1;
		case OPIP_GMX_INF_MODE_MODE_QSGMII:
			return 4;
		default:
			return 0;
		}
	case OPIP_MODE_XAUI:
		return 1;
	case OPIP_MODE_NPI:
		/* The packet engines came with pass 2 of CN52XX and CN56XX */
		if (OCTEON_IS_MODEL(OCTEON_CN52XX_PASS1_X) ||
		    OCTEON_IS_MODEL(OCTEON_CN56XX_PASS1_X))
			return 0;
		return opip_chip->npi_ports;
	case OPIP_MODE_LOOP:
		return 4;
	default:
		return 0;
	}
}

/*
 * Enumerate the interfaces as the bootloader strapped them, and do what
 * each mode needs before input runs: the interface enable of the GMX
 * modes (errata GMX-700), the length checks and FCS handling of the
 * loopback ports. The SPI-4.2 interfaces of CN38XX and CN58XX are not
 * driven: their bring-up and training could not be tested on any board.
 */
static void opip_probe_ifaces(struct opip *opip)
{
	int i, index;

	opip->num_ifaces = min(opip_num_interfaces(), OPIP_MAX_IFACES);
	dev_dbg(&opip->pdev->dev, "%s, %d interfaces\n", opip_chip->name,
		opip->num_ifaces);
	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];

		iface->num = i;
		iface->first_port = opip_port_number(i, 0);
		iface->mode = opip_iface_mode(i);
		/* CN68XX's interfaces 6 to 8 have no port numbers here */
		if (iface->first_port < 0) {
			iface->num_ports = 0;
			continue;
		}
		iface->num_ports = opip_iface_ports(i, iface->mode);

		switch (iface->mode) {
		case OPIP_MODE_SGMII:
		case OPIP_MODE_PICMG:
			opip_gmx_iface_enable(i);
			iface->qsgmii = opip_gmx_iface_is_qsgmii(i);
			break;
		case OPIP_MODE_XAUI:
			/*
			 * HiGig2 makes its 16 logical ports out of a header
			 * the driver neither parses nor builds: none of them
			 * would work
			 */
			if (opip_rd(OPIP_GMX_HG2_CONTROL(i)) &
			    OPIP_GMX_HG2_CONTROL_HG2TX_EN) {
				dev_warn(&opip->pdev->dev,
					 "interface %d is HiGig2, not supported\n",
					 i);
				iface->num_ports = 0;
				break;
			}
			opip_xaui_probe(i);
			break;
		case OPIP_MODE_SPI:
			dev_dbg(&opip->pdev->dev,
				"interface %d is SPI-4.2, not supported\n", i);
			iface->num_ports = 0;
			break;
		case OPIP_MODE_LOOP:
			for (index = 0; index < 4; index++) {
				int port = opip_port_number(i, index);

				opip_pip_length_checks(port, false);
				opip_ipd_fcs_strip(port, false);
			}
			break;
		default:
			break;
		}
		if (iface->num_ports)
			dev_dbg(&opip->pdev->dev, "interface %d has %d ports (%s)\n",
				i, iface->num_ports,
				opip_mode_names[iface->mode]);
	}
}

/*
 * Bring the ports of an interface up, with the IPD enabled, the PKO not.
 * A failure is reported, not fatal: the ports exist, their links stay
 * down, and the XAUI link setup tries again.
 */
static void opip_iface_enable(struct opip *opip, struct opip_iface *iface)
{
	int index, ret = 0;

	switch (iface->mode) {
	case OPIP_MODE_RGMII:
	case OPIP_MODE_GMII:
		ret = opip_gmx_rgmii_enable(iface->num, iface->num_ports);
		/* The clock delays of opip_rgmii_config(), over the defaults */
		for (index = 0; !ret && index < iface->num_ports; index++) {
			int port = opip_port_number(iface->num, index);
			struct opip_port *p;

			if (!opip->ports[port])
				continue;
			p = opip_port(opip->ports[port]);
			/* A GMII or MII port keeps the default */
			if (!phy_interface_mode_is_rgmii(p->phy_mode))
				continue;
			opip_gmx_rgmii_set_delays(iface->num, index,
						  p->rgmii_rx_delay,
						  p->rgmii_tx_delay);
		}
		break;
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		ret = opip_sgmii_enable(iface->num, iface->num_ports);
		break;
	case OPIP_MODE_XAUI:
		ret = opip_xaui_enable(iface->num, iface->first_port);
		break;
	case OPIP_MODE_NPI:
		/* Frames of any length; the host enables the ports */
		if (opip_chip->gmx_frm_max)
			break;
		for (index = 0; index < iface->num_ports; index++) {
			int port = opip_port_number(iface->num, index);

			opip_pip_length_checks(port, false);
		}
		break;
	default:
		break;
	}
	if (ret)
		dev_err(&opip->pdev->dev, "interface %d (%s) did not come up: %d\n",
			iface->num, opip_mode_names[iface->mode], ret);
}

/* ------------------------------------------------------------------ */
/* Errata: the IPD pointer alignment of CN30XX and CN31XX pass 1 */

/*
 * The IPD of these chips starts with its pointer FIFOs misaligned. Send
 * packets of the right size through port 0 in internal loopback until the
 * counts line up, and take the packets back from the POW. The packets go
 * out on the first queue of port 0, the device's own if it has one: the
 * commands count nothing in the FAU, so its completions are not disturbed.
 * The polls already run: the packets go to a group none of them serves.
 */
static void opip_ipd_ptr_alignment_errata(struct opip *opip)
{
	const unsigned int first_payload =
		OPIP_POOL_PACKET_SIZE - OPIP_BUF_LINK_SIZE - OPIP_FIRST_SKIP;
	const unsigned int later_payload =
		OPIP_POOL_PACKET_SIZE - OPIP_BUF_LINK_SIZE - OPIP_LATER_SKIP;
	u64 prt_cfg, tx_en, rx_en, jabber, frm_max, tag;
	unsigned int num_segs = 0, pending, core;
	/* The groups in use are 8 at most */
	const int group = OPIP_MAX_GROUPS - 1;
	struct opip_txq *txq, *own = NULL;
	u64 old_mask;
	int loop;

	prt_cfg = opip_rd(OPIP_GMX_PRT_CFG(0, 0));
	tx_en = opip_rd(OPIP_ASX_TX_PRT_EN(0));
	rx_en = opip_rd(OPIP_ASX_RX_PRT_EN(0));
	jabber = opip_rd(OPIP_GMX_RX_JABBER(0, 0));
	frm_max = opip_rd(OPIP_GMX_RX_FRM_MAX(0, 0));
	tag = opip_rd(OPIP_PIP_PRT_TAG(0));
	if (!opip_pko_port_valid(opip, 0))
		return;
	if (opip->ports[0]) {
		txq = &opip_port(opip->ports[0])->txq[0];
	} else {
		own = kzalloc_obj(*own);
		if (!own)
			return;
		own->queue = opip_pko_base_queue(opip, 0);
		if (opip_pko_queue_init(opip, own, 0, 0, 1)) {
			kfree(own);
			return;
		}
		txq = own;
	}

	opip_gmx_rgmii_loopback(0, 0);
	opip_wr(OPIP_ASX_RX_PRT_EN(0), 0);
	msleep(100);

	opip_wr(OPIP_PIP_PRT_TAG(0),
		u64_replace_bits(tag & ~OPIP_PIP_PRT_TAG_GRPTAG, group,
				 OPIP_PIP_PRT_TAG_GRP));
	/* The group mask is the core's own */
	migrate_disable();
	core = opip_core();
	old_mask = opip_pow_select_group(core, group);

	for (loop = 0; loop < 10; loop++) {
		u64 count = opip_rd(OPIP_IPD_PTR_COUNT);
		unsigned int wqe_pcnt = FIELD_GET(OPIP_IPD_PTR_COUNT_WQE_PCNT,
						  count);
		unsigned int pkt_pcnt = FIELD_GET(OPIP_IPD_PTR_COUNT_PKT_PCNT,
						  count);
		unsigned int size, i;
		struct opip_wqe *wqe;
		int retry = 100000;
		u64 *list, *data;
		void *buf;

		num_segs = (2 + pkt_pcnt - wqe_pcnt) & 3;
		if (!num_segs)
			break;
		num_segs++;
		size = first_payload + (num_segs - 1) * later_payload -
		       later_payload / 2;

		opip_wr(OPIP_ASX_PRT_LOOP(0), BIT(0));
		mb();	/* the loopback is on before the packet is sent */

		list = opip_fpa_alloc(OPIP_POOL_WQE);
		buf = opip_fpa_alloc(OPIP_POOL_PACKET);
		if (!list || !buf) {
			dev_warn(&opip->pdev->dev, "no buffer for the IPD alignment errata\n");
			if (list)
				opip_fpa_free(list, OPIP_POOL_WQE, 0);
			if (buf)
				opip_fpa_free(buf, OPIP_POOL_PACKET, 0);
			break;
		}

		/*
		 * An IP packet of the size, in one buffer the PKO reads in
		 * segments
		 */
		data = buf;
		data[0] = 0xffffffffffff0000ull;
		data[1] = 0x08004510ull;
		data[2] = ((u64)(size - 14) << 48) | 0x5ae740004000ull;
		data[3] = 0x3a5fc0a81073c0a8ull;
		for (i = 0; i < num_segs; i++)
			list[i] = opip_buf_build(opip_phys(buf),
						 i ? later_payload :
						     first_payload,
						 OPIP_POOL_PACKET, 0,
						 i != num_segs - 1);

		opip_gmx_port_enable(0, 0, true, 0);
		opip_wr(OPIP_ASX_TX_PRT_EN(0), BIT(0));
		opip_wr(OPIP_ASX_RX_PRT_EN(0), BIT(0));
		opip_wr(OPIP_GMX_RX_JABBER(0, 0), OPIP_MAX_FRAME - 14 - 4);
		opip_wr(OPIP_GMX_RX_FRM_MAX(0, 0), OPIP_MAX_FRAME - 14 - 4);

		if (!opip_pko_write(txq,
				    FIELD_PREP(OPIP_PKO_CMD_SEGS, num_segs) |
				    FIELD_PREP(OPIP_PKO_CMD_TOTAL_BYTES, size) |
				    OPIP_PKO_CMD_GATHER,
				    opip_buf_build(opip_phys(list), num_segs,
						   OPIP_POOL_WQE, 0, false))) {
			dev_warn(&opip->pdev->dev, "no command chunk for the IPD alignment errata\n");
			opip_fpa_free(list, OPIP_POOL_WQE, 0);
			opip_fpa_free(buf, OPIP_POOL_PACKET, 0);
			break;
		}
		opip_pko_doorbell(0, txq->queue, 2);
		/* the doorbell is written before the work is requested */
		mb();

		do {
			wqe = opip_pow_request_sync(true);
		} while (!wqe && --retry);
		if (!wqe) {
			dev_warn(&opip->pdev->dev, "IPD alignment errata: no work came back\n");
		} else {
			/* As a poll would */
			opip_ipd_bp_sub(0, opip_wqe_bufs(wqe));
			opip_rx_free_work(wqe);
		}
	}

	/* Work that did not come back may leave the command in the PKO */
	read_poll_timeout(opip_pko_queue_pending, pending, !pending, 200,
			  20 * USEC_PER_MSEC, false, opip, txq->queue);
	/*
	 * What comes in from now on goes to a served group; a frame still
	 * on its way through the loopback and what came in late are in no
	 * served group: take them back.
	 */
	opip_wr(OPIP_PIP_PRT_TAG(0), tag);
	usleep_range(200, 400);
	for (;;) {
		struct opip_wqe *wqe = opip_pow_request_sync(false);

		if (!wqe)
			break;
		opip_ipd_bp_sub(0, opip_wqe_bufs(wqe));
		opip_rx_free_work(wqe);
	}
	opip_pow_restore_groups(core, old_mask);
	migrate_enable();

	opip_wr(OPIP_GMX_PRT_CFG(0, 0), prt_cfg);
	opip_wr(OPIP_ASX_TX_PRT_EN(0), tx_en);
	opip_wr(OPIP_ASX_RX_PRT_EN(0), rx_en);
	opip_wr(OPIP_GMX_RX_JABBER(0, 0), jabber);
	opip_wr(OPIP_GMX_RX_FRM_MAX(0, 0), frm_max);
	opip_wr(OPIP_ASX_PRT_LOOP(0), 0);
	mb();	/* the registers are restored before the port is used */
	if (num_segs)
		dev_warn(&opip->pdev->dev, "the IPD pointer alignment errata workaround failed\n");
	if (pending)
		dev_warn(&opip->pdev->dev, "IPD alignment errata: the PKO holds %u command words of queue %d\n",
			 pending, txq->queue);
	if (own) {
		/* A chunk the PKO may still read is kept, not reused */
		if (!pending)
			opip_pko_queue_release(opip, own);
		kfree(own);
	}
}

/* ------------------------------------------------------------------ */
/* Probe and remove */

/*
 * A kernel that crashed (kdump) or a driver without a shutdown may have
 * left packet I/O running, and its buffers in the pools and the groups.
 * Stop it and take them out without freeing them: they are not this
 * kernel's. After the bootloader there is nothing to take. The IPD's
 * FIFOs are not walked: the reset that follows drops what the IPD holds,
 * and its port FIFO, walked whole, holds what any earlier user left.
 */
static void opip_hw_takeover(struct opip *opip)
{
	unsigned int core, n = 0;
	int pool, group;
	u64 old_mask;

	opip_ipd_disable();
	if (opip_rd(OPIP_PKO_REG_FLAGS) & OPIP_PKO_REG_FLAGS_ENA_PKO) {
		/* It finishes the packets it is sending, see opip_pko_stop() */
		opip_pko_disable();
		msleep(20);
	}
	opip_pko_reset();
	opip_fpa_enable();
	for (pool = OPIP_POOL_PACKET; pool <= OPIP_POOL_CMD; pool++)
		while (opip_fpa_alloc(pool))
			n++;

	/* The group mask is the core's own */
	preempt_disable();
	core = opip_core();
	for (group = 0; group < OPIP_MAX_GROUPS; group++) {
		old_mask = opip_pow_select_group(core, group);
		while (opip_pow_request_sync(false))
			n++;
		opip_pow_restore_groups(core, old_mask);
	}
	preempt_enable();

	if (n)
		dev_warn(&opip->pdev->dev, "dropped %u buffers and work entries of an earlier owner\n",
			 n);
}

const struct opip_chip *opip_chip __read_mostly;

/*
 * The chips, one entry per model family the arch tells apart. A field
 * left out is false or 0: no QSGMII, no SPI, no L2C_CFG, every queue in
 * the layout, no PKO_REG_QUEUE_MODE.
 */
static const struct opip_chip opip_chip_cn30xx = {
	.name = "CN30XX",
	.cn3xxx = true,
	.small = true,
	.gmx_frm_max = true,
	.l2c_cfg = true,
	.rgmii_mii_clk = true,
	.rgmii_delay = 24,
	.rgmii_ports = 3,
	.pko_queues = 32,
	.pko_layout_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn31xx = {
	.name = "CN31XX",
	.cn3xxx = true,
	.small = true,
	.gmx_frm_max = true,
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.rgmii_ports = 3,
	.pko_queues = 32,
	.pko_layout_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn38xx = {
	.name = "CN38XX",
	.cn3xxx = true,
	.gmx_frm_max = true,
	.spi = true,
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.rgmii_ports = 4,
	.npi_ports = 4,
	.pko_queues = 128,
	.pko_layout_queues = 128,
	.pko_mode_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn50xx = {
	.name = "CN50XX",
	.small = true,
	.l2c_cfg = true,
	.rgmii_mii_clk = true,
	.rgmii_delay = 16,
	.rgmii_ports = 3,
	.pko_queues = 32,
	.pko_layout_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn52xx = {
	.name = "CN52XX",
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.npi_ports = 4,
	.pko_queues = 128,
	.pko_layout_queues = 128,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn56xx = {
	.name = "CN56XX",
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.npi_ports = 4,
	.pko_queues = 256,
	.pko_layout_queues = 256,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn58xx = {
	.name = "CN58XX",
	.gmx_frm_max = true,
	.spi = true,
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.rgmii_ports = 4,
	.npi_ports = 4,
	.pko_queues = 256,
	.pko_layout_queues = 256,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

/*
 * OCTEON II and CN70XX: the layout stays within 64 queues, where
 * PKO_REG_QUEUE_MODE 2 gives each queue 32 words of command storage
 */
static const struct opip_chip opip_chip_cn61xx = {
	.name = "CN61XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn63xx = {
	.name = "CN63XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn66xx = {
	.name = "CN66XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn68xx = {
	.name = "CN68XX",
	.cn68xx = true,
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x1000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cnf71xx = {
	.name = "CNF71XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn70xx = {
	.name = "CN70XX",
	.qsgmii = true,
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

/* The arch's model constants; each covers the variants of its family */
static const struct {
	u32 model;
	const struct opip_chip *chip;
} opip_chips[] = {
	{ OCTEON_CN30XX, &opip_chip_cn30xx },
	{ OCTEON_CN31XX, &opip_chip_cn31xx },
	{ OCTEON_CN38XX, &opip_chip_cn38xx },
	{ OCTEON_CN50XX, &opip_chip_cn50xx },
	{ OCTEON_CN52XX, &opip_chip_cn52xx },
	{ OCTEON_CN56XX, &opip_chip_cn56xx },
	{ OCTEON_CN58XX, &opip_chip_cn58xx },
	{ OCTEON_CN61XX, &opip_chip_cn61xx },
	{ OCTEON_CN63XX, &opip_chip_cn63xx },
	{ OCTEON_CN66XX, &opip_chip_cn66xx },
	{ OCTEON_CN68XX, &opip_chip_cn68xx },
	{ OCTEON_CNF71XX, &opip_chip_cnf71xx },
	{ OCTEON_CN70XX, &opip_chip_cn70xx },
};

/* The chip the driver runs on, from the processor ID */
static const struct opip_chip *opip_chip_find(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(opip_chips); i++)
		if (OCTEON_IS_MODEL(opip_chips[i].model))
			return opip_chips[i].chip;
	return NULL;
}

/*
 * The packet I/O hardware the ports share, up to the receive groups.
 * Return: 0, or -ENOMEM with the hardware idle and the pools empty
 */
static int opip_hw_init(struct opip *opip)
{
	int i, order, ports = 0;

	/* CN52XX pass 1: a bug in the second order CDR, disable it */
	if (OCTEON_IS_MODEL(OCTEON_CN52XX_PASS1_0))
		opip_errata_qlm_cdr();

	/*
	 * The I/O blocks get priority over the cores at the L2: no starving.
	 * L2C_CFG is CN3XXX/CN5XXX only: OCTEON II/III arbitrate in L2C_CTL,
	 * which the bootloader sets.
	 */
	if (opip_chip->l2c_cfg)
		opip_wr(OPIP_L2C_CFG, opip_rd(OPIP_L2C_CFG) &
			~(OPIP_L2C_CFG_LRF_ARB_MODE |
			  OPIP_L2C_CFG_RFB_ARB_MODE));

	/*
	 * Whatever the bootloader left: the IPD's count of the packet pool
	 * starts at zero while the pool is empty, see opip_ipd_reset().
	 */
	opip_hw_takeover(opip);
	opip_ipd_reset();
	opip_fpa_enable();
	opip_probe_ifaces(opip);

	/* The packet pool by the MAC ports, see OPIP_RX_RING_PER_PORT */
	for (i = 0; i < opip->num_ifaces; i++)
		if (opip_iface_is_gmx(&opip->ifaces[i]))
			ports += opip->ifaces[i].num_ports;
	opip->num_packet_buffers = clamp(ports * OPIP_RX_RING_PER_PORT,
					 OPIP_DEFAULT_PACKET_BUFFERS,
					 OPIP_MAX_PACKET_BUFFERS);

	/* Every received packet takes an entry; the ports add their share */
	opip->wqe_bufs = opip_fpa_fill(opip, OPIP_POOL_WQE, OPIP_POOL_WQE_SIZE,
				       opip->num_packet_buffers);
	opip->rx_wqe_bufs = opip->wqe_bufs;
	opip->cmd_bufs = opip_fpa_fill(opip, OPIP_POOL_CMD, OPIP_POOL_CMD_SIZE,
				       OPIP_NUM_CMD_CHUNKS);
	if (opip->wqe_bufs < opip->num_packet_buffers ||
	    opip->cmd_bufs < OPIP_NUM_CMD_CHUNKS) {
		opip_fpa_empty(opip, OPIP_POOL_WQE, opip->wqe_bufs);
		opip_fpa_empty(opip, OPIP_POOL_CMD, opip->cmd_bufs);
		return -ENOMEM;
	}

	/* One output queue per CPU and port, as far as the chip has them */
	opip_pko_layout(opip, min_t(int, num_possible_cpus(), OPIP_MAX_TXQS));
	opip_pko_init(opip);

	/*
	 * One receive group per CPU, as far as a power of two goes: the PIP
	 * spreads the flows over them by a mask of the tag.
	 */
	order = min(ilog2(num_online_cpus()), 3);
	opip->groups_mask = GENMASK((1 << order) - 1, 0);

	opip_ipd_config();
	opip_ipd_red_setup(opip->num_packet_buffers / 4,
			   opip->num_packet_buffers / 8);
	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];
		int index;

		for (index = 0; index < iface->num_ports; index++) {
			int port = opip_port_number(i, index);

			opip_pip_port_config(i, port, order);
			/*
			 * The bootloader leaves the backpressure counts
			 * nonzero. Zero them now: with the packet pool empty
			 * the IPD counts nothing and no poll subtracts.
			 */
			if ((iface->mode == OPIP_MODE_SGMII ||
			     iface->mode == OPIP_MODE_PICMG) &&
			    !opip_chip->cn68xx)
				opip_ipd_bp_sub(port, opip_ipd_bp_count(port));
		}
	}
	return 0;
}

/*
 * The interface bring-up enables the MAC of every RGMII, GMII, SGMII,
 * PICMG and XAUI port. A port without a device has nobody to take its
 * packets: off again.
 */
static void opip_iface_disable_unused(struct opip *opip,
				      struct opip_iface *iface)
{
	int index;

	switch (iface->mode) {
	case OPIP_MODE_RGMII:
	case OPIP_MODE_GMII:
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		break;
	case OPIP_MODE_XAUI:
		/* One MAC for the interface: off if its port has no device */
		for (index = 0; index < iface->num_ports; index++)
			if (opip->ports[opip_port_number(iface->num, index)])
				return;
		opip_gmx_port_enable(iface->num, 0, false, 0);
		return;
	default:
		return;
	}
	for (index = 0; index < iface->num_ports; index++)
		if (!opip->ports[opip_port_number(iface->num, index)])
			opip_gmx_port_enable(iface->num, index, false, 0);
}

/* Packet input and output on: the IPD, the interfaces, the PKO */
static void opip_input_enable(struct opip *opip)
{
	int i;

	opip_ipd_enable();
	for (i = 0; i < opip->num_ifaces; i++) {
		if (!opip->ifaces[i].num_ports)
			continue;
		opip_iface_enable(opip, &opip->ifaces[i]);
		opip_iface_disable_unused(opip, &opip->ifaces[i]);
	}
	opip_pko_enable();

	if ((OCTEON_IS_MODEL(OCTEON_CN31XX_PASS1) ||
	     OCTEON_IS_MODEL(OCTEON_CN30XX_PASS1)) && !opip_on_simulator())
		opip_ipd_ptr_alignment_errata(opip);
}

/*
 * Undo opip_hw_init() and what the ports put into the pools. @drain once
 * input ran: before, the IPD's FIFOs hold nothing of this driver, and the
 * port FIFO, walked whole, holds what an earlier user left there.
 */
static void opip_free_hw(struct opip *opip, bool drain)
{
	unsigned int owed = 0;
	int i;

	/* The polls are down: none can queue the refill worker again */
	cancel_delayed_work_sync(&opip->refill_work);
	/*
	 * What the polls still owe the packet pool is not in it; what a
	 * shrink could not take out yet still is
	 */
	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		owed += opip->groups[i].to_refill;
	opip_pko_reset();
	if (drain)
		opip_ipd_drain();
	opip_fpa_empty_packet_pool(opip, opip->num_packet_buffers +
				   atomic_read(&opip->rx_excess) - owed);
	/* The IPD's count of the packet pool is zero again with the pool */
	opip_ipd_reset();
	opip_rx_free_pools(opip);
	opip_fpa_empty(opip, OPIP_POOL_WQE, opip->wqe_bufs);
	opip_fpa_empty(opip, OPIP_POOL_CMD, opip->cmd_bufs);
}

/* A device for every port of every interface; returns how many there are */
static int opip_alloc_ports(struct opip *opip, struct device_node *pip)
{
	int i, index, num = 0;

	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];
		bool available, described, missing;
		struct device_node *ni, *np;

		if (iface->mode == OPIP_MODE_DISABLED ||
		    iface->mode == OPIP_MODE_PCIE)
			continue;
		/* An interface the device tree disables has no ports */
		ni = opip_of_child(pip, iface->num);
		available = !ni || of_device_is_available(ni);
		described = !!ni;
		of_node_put(ni);
		if (!available)
			continue;
		for (index = 0; index < iface->num_ports; index++) {
			struct net_device *dev;

			/*
			 * Under an interface node, a port without a node of its
			 * own is not on the board: the platform code removes
			 * the nodes of the ports the executive's board quirks
			 * take away (built-in device tree only)
			 */
			np = opip_port_of_node(pip, iface->num, index);
			missing = described && !np;
			of_node_put(np);
			if (missing)
				continue;
			dev = opip_alloc_port(opip, pip, iface, index);
			if (dev) {
				opip->ports[opip_port(dev)->port] = dev;
				num++;
			}
		}
	}
	return num;
}

/* Register the devices; a port that fails is freed, the others go on */
static int opip_register_ports(struct opip *opip)
{
	int port, num = 0;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		struct net_device *dev = opip->ports[port];
		struct opip_port *p;

		if (!dev)
			continue;
		p = opip_port(dev);
		if (register_netdev(dev)) {
			dev_err(&opip->pdev->dev, "failed to register the device of interface %d port %d\n",
				p->iface->num, p->index);
			/*
			 * No poll or rtnl user of an earlier port may still
			 * hold it when freed
			 */
			rtnl_lock();
			WRITE_ONCE(opip->ports[port], NULL);
			rtnl_unlock();
			synchronize_net();
			opip_port_enable(p, false);
			opip_put_port(dev);
			continue;
		}
		opip_setup_xps(dev);
		schedule_delayed_work(&p->periodic_work, HZ);
		num++;
	}
	return num;
}

/* Unregistering closes the device and waits for its packets in flight */
static void opip_unregister_ports(struct opip *opip)
{
	int port;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		struct net_device *dev = opip->ports[port];
		struct opip_port *p;

		if (!dev || dev->reg_state != NETREG_REGISTERED)
			continue;
		p = opip_port(dev);
		unregister_netdev(dev);
		cancel_delayed_work_sync(&p->periodic_work);
		cancel_work_sync(&p->rx_mode_work);
	}
}

static void opip_free_ports(struct opip *opip)
{
	int port;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		if (!opip->ports[port])
			continue;
		opip_put_port(opip->ports[port]);
		opip->ports[port] = NULL;
	}
}

/*
 * Disable the PKO. It stops taking new commands but finishes the packets
 * it is sending: a 9 KB frame takes 8 ms at 10 Mbit/s. Give it the time,
 * then reset it: a port held by PAUSE frames would keep it reading the
 * buffers its queues still hold, which are taken away next.
 */
static void opip_pko_stop(void)
{
	opip_pko_disable();
	msleep(20);
	opip_pko_reset();
}

/*
 * Take the driver down in the order its parts depend on each other: input
 * off, the devices unregistered (each drains its queues), the PKO off, the
 * groups down, and only then the devices and the hardware freed.
 */
static void opip_teardown(struct opip *opip)
{
	debugfs_remove_recursive(opip->debugfs);
	opip_ipd_disable();
	WRITE_ONCE(opip->stopping, true);
	cancel_delayed_work_sync(&opip->refill_work);

	opip_unregister_ports(opip);
	opip_pko_stop();
	opip_rx_shutdown(opip);
	opip_free_ports(opip);
	opip_free_hw(opip, true);
	free_netdev(opip->napi_dev);
}

static void opip_remove(struct platform_device *pdev)
{
	struct opip *opip = platform_get_drvdata(pdev);

	if (opip->released)
		return;
	opip->released = true;
	opip_teardown(opip);
}

/*
 * A port's MAC address may sit in an nvmem cell whose provider is not up
 * yet. Find out before the hardware is touched: the probe is deferred as
 * a whole, and a deferred probe that had already set the blocks up would
 * have to take them down again, pools and all.
 *
 * Return: -EPROBE_DEFER if an address is not available yet, else 0
 */
static int opip_wait_for_mac_addresses(struct device_node *pip)
{
	struct device_node *iface, *port;

	for_each_available_child_of_node(pip, iface) {
		for_each_available_child_of_node(iface, port) {
			u8 addr[ETH_ALEN];

			if (of_get_mac_address(port, addr) == -EPROBE_DEFER) {
				of_node_put(port);
				of_node_put(iface);
				return -EPROBE_DEFER;
			}
		}
	}
	return 0;
}

static void opip_free(void *opip)
{
	kfree(opip);
}

static int opip_probe(struct platform_device *pdev)
{
	struct device_node *pip = pdev->dev.of_node;
	struct opip *opip;
	int ret;

	ret = opip_wait_for_mac_addresses(pip);
	if (ret)
		return ret;

	opip_chip = opip_chip_find();
	if (!opip_chip) {
		dev_err(&pdev->dev, "unknown chip\n");
		return -ENODEV;
	}
	/*
	 * Engineering samples: their IPD cannot give back the buffers it
	 * holds, so the driver could never take its pools down again
	 */
	if (OCTEON_IS_MODEL(OCTEON_CN38XX_PASS1) ||
	    OCTEON_IS_MODEL(OCTEON_CN38XX_PASS2)) {
		dev_err(&pdev->dev, "CN38XX pass 1 and 2 are not supported\n");
		return -ENODEV;
	}
	/* Errata GMX-401: the runt padding, only here */
	if (opip_chip->cn3xxx)
		static_branch_enable(&opip_gmx401);

	/*
	 * Not devm_kzalloc(): the devres header in front of it would leave
	 * the groups off the cache line alignment of struct opip_group
	 */
	opip = kzalloc_obj(*opip);
	if (!opip)
		return -ENOMEM;
	ret = devm_add_action_or_reset(&pdev->dev, opip_free, opip);
	if (ret)
		return ret;
	opip->pdev = pdev;
	mutex_init(&opip->hw_lock);
	spin_lock_init(&opip->pko_idx_lock);
	INIT_DELAYED_WORK(&opip->refill_work, opip_refill_worker);
	platform_set_drvdata(pdev, opip);

	/*
	 * The receive NAPIs hang off a dummy device: a POW group delivers
	 * the packets of every port (the PIP's tag picks the group), so its
	 * NAPI is none of the ports'. Nobody can set the dummy's
	 * gro_flush_timeout and napi_defer_hard_irqs from user space, so the
	 * driver sets the software interrupt coalescing r8169, bcmgenet,
	 * bcmasp, macb and ravb default to: a poll that ends with work done
	 * leaves the interrupt off once (napi_defer_hard_irqs 1) and is
	 * polled again from a 20 us timer (gro_flush_timeout), which saves
	 * the interrupt, its disable/enable and the group's ack under a
	 * steady load and lets GRO batches grow over polls.
	 */
	opip->napi_dev = alloc_netdev_dummy(0);
	if (!opip->napi_dev)
		return -ENOMEM;
	netdev_sw_irq_coalesce_default_on(opip->napi_dev);

	ret = opip_hw_init(opip);
	if (ret) {
		free_netdev(opip->napi_dev);
		return ret;
	}

	ret = opip_rx_init(opip);
	if (ret)
		goto err_hw;

	if (!opip_alloc_ports(opip, pip)) {
		dev_err(&pdev->dev, "no usable ports\n");
		ret = -ENODEV;
		goto err_ports;
	}

	opip_input_enable(opip);

	/* The state of the blocks, there when the first port appears */
	opip->debugfs = debugfs_create_dir(dev_name(&pdev->dev),
					   opip_debugfs_root);
	debugfs_create_file("status", 0444, opip->debugfs, opip,
			    &opip_debugfs_fops);

	if (!opip_register_ports(opip)) {
		dev_err(&pdev->dev, "no device could be registered\n");
		ret = -ENODEV;
		goto err_input;
	}

	schedule_delayed_work(&opip->refill_work, HZ);
	return 0;

err_input:
	opip_teardown(opip);
	return ret;
err_ports:
	opip_rx_shutdown(opip);
	opip_free_ports(opip);
err_hw:
	opip_free_hw(opip, false);
	free_netdev(opip->napi_dev);
	return ret;
}

/*
 * Leave the packet hardware idle and its pools empty for whatever runs
 * next, a kernel started by kexec or the bootloader: packet input would
 * go on writing to buffers that belong to it, and the pools would hand it
 * buffers of this kernel.
 */
static void opip_shutdown(struct platform_device *pdev)
{
	opip_remove(pdev);
}

static const struct of_device_id opip_match[] = {
	{ .compatible = "cavium,octeon-3860-pip" },
	{ }
};
MODULE_DEVICE_TABLE(of, opip_match);

static struct platform_driver opip_driver = {
	.probe		= opip_probe,
	.remove		= opip_remove,
	.shutdown	= opip_shutdown,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = opip_match,
	},
};

/* debugfs: octeon_pip/<device>/status */
static int __init opip_init(void)
{
	int ret;

	opip_debugfs_root = debugfs_create_dir(KBUILD_MODNAME, NULL);
	ret = platform_driver_register(&opip_driver);
	if (ret)
		debugfs_remove(opip_debugfs_root);
	return ret;
}
module_init(opip_init);

static void __exit opip_exit(void)
{
	platform_driver_unregister(&opip_driver);
	debugfs_remove(opip_debugfs_root);
}
module_exit(opip_exit);

/* phylink connects the PHY at open: with both modules, the bus must exist */
MODULE_SOFTDEP("pre: mdio-octeon");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Polack <spolack+git@mailbox.org>");
MODULE_DESCRIPTION("Cavium OCTEON PIP/PKO Ethernet driver");
