// SPDX-License-Identifier: GPL-2.0
/*
 * The ethtool operations of octeon_pip.
 *
 * The hardware counters that have a standard home are reported there: the
 * rtnl_link_stats64 through ndo_get_stats64, the MAC, control, pause and
 * RMON counters through their ethtool groups. ethtool -S has what is left:
 * the driver's own counters and the error events of the GMX and the PCS.
 */

#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/phylink.h>
#include <linux/string.h>

#include "octeon_pip.h"
#include "octeon_pip_rx.h"

/*
 * The counters of ethtool -S, in the order opip_get_ethtool_stats() fills them
 */
static const char opip_stat_names[][ETH_GSTRING_LEN] = {
	/* Summed over the transmit queues */
	"tx_linearized",
	"tx_no_wqe",
	"tx_no_chunk",
	/*
	 * Error events, not frames: the times the periodic worker found the
	 * bit set, see opip_poll_errors()
	 */
	"tx_pko_nxa",
	"pcs_an_error",
	"pcs_tx_fifo_error",
	/*
	 * Frames dropped for a receive error the PIP does not count and no
	 * standard counter has (the jabber and L2 length drops are in
	 * FrameTooLongErrors and InRangeLengthErrors)
	 */
	"rx_other_errors",
};

/*
 * The size histograms: the PIP counts received frames of 64 bytes and
 * above, the GMX sent frames below 64 bytes as well, up to the same
 * bounds. The bins the PIP lacks come from the runt counters.
 */
static const struct ethtool_rmon_hist_range opip_rmon_ranges[] = {
	{ 0, 63 },
	{ 64, 64 },
	{ 65, 127 },
	{ 128, 255 },
	{ 256, 511 },
	{ 512, 1023 },
	{ 1024, 1518 },
	{ 1519, OPIP_MAX_FRAME },
	{}
};

static_assert(ARRAY_SIZE(opip_rmon_ranges) - 1 == OPIP_GMX_HIST_BINS);
static_assert(OPIP_PIP_HIST_BINS + 1 == OPIP_GMX_HIST_BINS);

static int opip_get_sset_count(struct net_device *dev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;
	return ARRAY_SIZE(opip_stat_names);
}

static void opip_get_strings(struct net_device *dev, u32 sset, u8 *data)
{
	if (sset != ETH_SS_STATS)
		return;
	memcpy(data, opip_stat_names, sizeof(opip_stat_names));
}

static void opip_get_ethtool_stats(struct net_device *dev,
				   struct ethtool_stats *stats, u64 *data)
{
	u64 linearized = 0, no_wqe = 0, no_chunk = 0;
	struct opip_port *p = opip_port(dev);
	unsigned int qid;
	int i = 0;

	for (qid = 0; qid < dev->real_num_tx_queues; qid++) {
		struct opip_txq *q = &p->txq[qid];

		linearized += READ_ONCE(q->linearized);
		no_wqe += READ_ONCE(q->no_wqe);
		no_chunk += READ_ONCE(q->no_chunk);
	}
	data[i++] = linearized;
	data[i++] = no_wqe;
	data[i++] = no_chunk;
	data[i++] = READ_ONCE(p->tx_pko_nxa);
	data[i++] = READ_ONCE(p->pcs_an_error);
	data[i++] = READ_ONCE(p->pcs_tx_fifo_error);
	data[i++] = atomic_long_read(&p->rx_other_errors);
}

static void opip_get_eth_mac_stats(struct net_device *dev,
				   struct ethtool_eth_mac_stats *mac)
{
	struct opip_gmx_stats gmx;
	struct opip_hw_stats hw;

	opip_update_hw_stats(dev);
	opip_get_hw_stats(dev, &hw);
	opip_get_gmx_stats(dev, &gmx);
	mac->FramesTransmittedOK = hw.tx_packets;
	mac->OctetsTransmittedOK = hw.tx_octets;
	mac->FramesReceivedOK = hw.rx_packets;
	mac->OctetsReceivedOK = hw.rx_octets;
	/* The PIP counts FCS and alignment errors together */
	mac->FrameCheckSequenceErrors = hw.rx_fcs_align_errors;
	mac->FrameTooLongErrors = hw.rx_oversize + hw.rx_oversize_fcs +
				  atomic_long_read(&opip_port(dev)->rx_jabber);
	mac->InRangeLengthErrors =
		atomic_long_read(&opip_port(dev)->rx_l2_length);
	mac->MulticastFramesReceivedOK = hw.rx_multicast;
	mac->BroadcastFramesReceivedOK = hw.rx_broadcast;
	/* NPI and loopback ports have no MAC: its counters stay unset */
	if (!opip_port_has_gmx(opip_port(dev)))
		return;
	mac->SingleCollisionFrames = gmx.tx_single_collisions;
	mac->MultipleCollisionFrames = gmx.tx_multiple_collisions;
	mac->FramesAbortedDueToXSColls = gmx.tx_excessive_collisions;
	mac->FramesWithExcessiveDeferral = gmx.tx_excessive_deferrals;
	mac->FramesLostDueToIntMACXmitError = gmx.tx_underflow;
	mac->FramesLostDueToIntMACRcvError = gmx.rx_drop_packets;
	mac->MulticastFramesXmittedOK = gmx.tx_multicast;
	mac->BroadcastFramesXmittedOK = gmx.tx_broadcast;
}

/* The GMX counts MAC control frames; the only ones it sends are PAUSE */
static void opip_get_eth_ctrl_stats(struct net_device *dev,
				    struct ethtool_eth_ctrl_stats *ctrl)
{
	struct opip_gmx_stats gmx;

	if (!opip_port_has_gmx(opip_port(dev)))
		return;
	opip_update_hw_stats(dev);
	opip_get_gmx_stats(dev, &gmx);
	ctrl->MACControlFramesTransmitted = gmx.tx_ctl_packets;
	ctrl->MACControlFramesReceived = gmx.rx_ctl_packets;
}

/*
 * The GMX sends no MAC control frames but PAUSE; it counts all it
 * receives, which is not a PAUSE count: rx_pause_frames stays unset, the
 * count is MACControlFramesReceived
 */
static void opip_get_pause_stats(struct net_device *dev,
				 struct ethtool_pause_stats *pause)
{
	struct opip_gmx_stats gmx;

	if (!opip_port_has_gmx(opip_port(dev)))
		return;
	opip_update_hw_stats(dev);
	opip_get_gmx_stats(dev, &gmx);
	pause->tx_pause_frames = gmx.tx_ctl_packets;
}

static void opip_get_rmon_stats(struct net_device *dev,
				struct ethtool_rmon_stats *rmon,
				const struct ethtool_rmon_hist_range **ranges)
{
	struct opip_port *p = opip_port(dev);
	struct opip_gmx_stats gmx;
	struct opip_hw_stats hw;
	int i;

	opip_update_hw_stats(dev);
	opip_get_hw_stats(dev, &hw);
	opip_get_gmx_stats(dev, &gmx);
	rmon->undersize_pkts = hw.rx_runts;
	rmon->fragments = hw.rx_runts_fcs;
	rmon->oversize_pkts = hw.rx_oversize;
	rmon->jabbers = hw.rx_oversize_fcs;

	/* The PIP counts from 64 bytes on; below that are the runts */
	rmon->hist[0] = hw.rx_runts + hw.rx_runts_fcs;
	for (i = 0; i < OPIP_PIP_HIST_BINS; i++)
		rmon->hist[i + 1] = hw.rx_hist[i];
	if (opip_port_has_gmx(p))
		for (i = 0; i < OPIP_GMX_HIST_BINS; i++)
			rmon->hist_tx[i] = gmx.tx_hist[i];
	*ranges = opip_rmon_ranges;
}

/*
 * The receive ring is the packet pool, shared by all ports: every port
 * shows and sets the same size. The transmit rings are fixed.
 */
static void opip_get_ringparam(struct net_device *dev,
			       struct ethtool_ringparam *ring,
			       struct kernel_ethtool_ringparam *kring,
			       struct netlink_ext_ack *extack)
{
	struct opip_port *p = opip_port(dev);

	ring->rx_max_pending = OPIP_MAX_PACKET_BUFFERS;
	ring->rx_pending = READ_ONCE(p->opip->num_packet_buffers);
	ring->tx_max_pending = OPIP_TXQ_DEPTH;
	ring->tx_pending = OPIP_TXQ_DEPTH;
}

static int opip_set_ringparam(struct net_device *dev,
			      struct ethtool_ringparam *ring,
			      struct kernel_ethtool_ringparam *kring,
			      struct netlink_ext_ack *extack)
{
	struct opip_port *p = opip_port(dev);

	if (ring->tx_pending != OPIP_TXQ_DEPTH) {
		NL_SET_ERR_MSG_MOD(extack, "the transmit ring size is fixed");
		return -EINVAL;
	}
	if (ring->rx_pending < OPIP_MIN_PACKET_BUFFERS) {
		NL_SET_ERR_MSG_FMT_MOD(extack,
				       "the receive ring has at least %u buffers",
				       OPIP_MIN_PACKET_BUFFERS);
		return -EINVAL;
	}
	return opip_rx_resize(p->opip, ring->rx_pending);
}

static int opip_nway_reset(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);

	if (!p->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_nway_reset(p->phylink);
}

static void opip_get_pauseparam(struct net_device *dev,
				struct ethtool_pauseparam *pause)
{
	struct opip_port *p = opip_port(dev);

	if (p->phylink && opip_port_pause_capable(p))
		phylink_ethtool_get_pauseparam(p->phylink, pause);
}

static int opip_set_pauseparam(struct net_device *dev,
			       struct ethtool_pauseparam *pause)
{
	struct opip_port *p = opip_port(dev);

	if (!p->phylink || !opip_port_pause_capable(p))
		return -EOPNOTSUPP;
	return phylink_ethtool_set_pauseparam(p->phylink, pause);
}

static int opip_get_link_ksettings(struct net_device *dev,
				   struct ethtool_link_ksettings *cmd)
{
	struct opip_port *p = opip_port(dev);

	if (!p->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_ksettings_get(p->phylink, cmd);
}

static int opip_set_link_ksettings(struct net_device *dev,
				   const struct ethtool_link_ksettings *cmd)
{
	struct opip_port *p = opip_port(dev);

	if (!p->phylink)
		return -EOPNOTSUPP;
	return phylink_ethtool_ksettings_set(p->phylink, cmd);
}

const struct ethtool_ops opip_ethtool_ops = {
	.nway_reset = opip_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_ts_info = ethtool_op_get_ts_info,
	.get_pauseparam = opip_get_pauseparam,
	.set_pauseparam = opip_set_pauseparam,
	.get_pause_stats = opip_get_pause_stats,
	.get_link_ksettings = opip_get_link_ksettings,
	.set_link_ksettings = opip_set_link_ksettings,
	.get_ringparam = opip_get_ringparam,
	.set_ringparam = opip_set_ringparam,
	.get_sset_count = opip_get_sset_count,
	.get_strings = opip_get_strings,
	.get_ethtool_stats = opip_get_ethtool_stats,
	.get_eth_mac_stats = opip_get_eth_mac_stats,
	.get_eth_ctrl_stats = opip_get_eth_ctrl_stats,
	.get_rmon_stats = opip_get_rmon_stats,
};
