// SPDX-License-Identifier: GPL-2.0
/*
 * The GMX blocks: the MACs of an interface. One GMX serves up to four
 * RGMII, GMII or (Q)SGMII ports, or one XAUI port.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper-rgmii.c, cvmx-helper-util.c, cvmx-helper-errata.c)
 * and the staging octeon driver (ethernet-rgmii.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "octeon_pip.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_pko.h"
#include "octeon_pip_regs.h"

/**
 * opip_gmx_iface_enable - enable an interface in GMX_INF_MODE
 * @iface: Interface
 *
 * Due to errata GMX-700 on CN56XX pass 1 and CN52XX pass 1, the interface
 * must be enabled before the IPD, or per-port backpressure may not work.
 */
void opip_gmx_iface_enable(int iface)
{
	opip_wr(OPIP_GMX_INF_MODE(iface),
		opip_rd(OPIP_GMX_INF_MODE(iface)) | OPIP_GMX_INF_MODE_EN);
}

/**
 * opip_gmx_iface_is_qsgmii - whether an interface runs QSGMII
 * @iface: Interface
 *
 * CN70XX: GMX_INF_MODE[MODE] is 1 for SGMII and 2 for QSGMII. The other
 * chips, and the interfaces past the first two, have no QSGMII.
 *
 * Return: true for a QSGMII interface
 */
bool opip_gmx_iface_is_qsgmii(int iface)
{
	if (!opip_chip->qsgmii || iface > 1)
		return false;
	return FIELD_GET(OPIP_GMX_INF_MODE_MODE,
			 opip_rd(OPIP_GMX_INF_MODE(iface))) ==
	       OPIP_GMX_INF_MODE_MODE_QSGMII;
}

/**
 * opip_gmx_setup - the common settings of a GMX: its port count
 * @iface:     Interface
 * @num_ports: Ports on it, four at most
 *
 * The port counts on the transmit and receive side, the PKO's view of
 * them, and the transmit threshold: the GMX buffers as much of a packet
 * as it can before it starts sending, so that memory contention cannot
 * underrun a packet that fits the FIFO. CN68XX: the PKO adds padding
 * and FCS, the GMX must not add them again.
 *
 * Return: 0, or -EINVAL for more ports than a GMX has
 */
int opip_gmx_setup(int iface, int num_ports)
{
	unsigned int thresh;
	int index;

	/* opip_iface_ports() counts at most four */
	if (WARN_ON_ONCE(num_ports > 4))
		return -EINVAL;
	opip_wr(OPIP_GMX_TX_PRTS(iface),
		u64_replace_bits(opip_rd(OPIP_GMX_TX_PRTS(iface)), num_ports,
				 OPIP_GMX_TX_PRTS_PRTS));
	opip_wr(OPIP_GMX_RX_PRTS(iface),
		u64_replace_bits(opip_rd(OPIP_GMX_RX_PRTS(iface)), num_ports,
				 OPIP_GMX_RX_PRTS_PRTS));
	opip_pko_gmx_port_mode(iface, num_ports);

	if (opip_chip->small)
		thresh = 0x40;	/* the fixed maximum of these chips */
	else if (num_ports <= 1)
		thresh = 0x100;
	else if (num_ports == 2)
		thresh = 0x100 / 2;
	else
		thresh = 0x100 / 4;
	for (index = 0; index < num_ports; index++) {
		opip_wr(OPIP_GMX_TX_THRESH(iface, index),
			u64_replace_bits(opip_rd(OPIP_GMX_TX_THRESH(iface,
								    index)),
					 thresh, OPIP_GMX_TX_THRESH_CNT));
		if (opip_chip->cn68xx)
			opip_wr(OPIP_GMX_TX_APPEND(iface, index),
				opip_rd(OPIP_GMX_TX_APPEND(iface, index)) &
				~(OPIP_GMX_TX_APPEND_FCS |
				  OPIP_GMX_TX_APPEND_PAD));
		/*
		 * No timestamps: a bootloader's PTP_MODE would have the GMX
		 * put eight bytes in front of every packet, and the PIP
		 * skips none
		 */
		opip_wr(OPIP_GMX_RX_FRM_CTL(iface, index),
			opip_rd(OPIP_GMX_RX_FRM_CTL(iface, index)) &
			~OPIP_GMX_RX_FRM_CTL_PTP_MODE);
	}
	return 0;
}

/**
 * opip_gmx_port_enabled - whether a port's MAC is enabled
 * @iface: Interface
 * @index: Port on it
 *
 * Return: true if GMX_PRT_CFG[EN] is set
 */
bool opip_gmx_port_enabled(int iface, int index)
{
	return opip_rd(OPIP_GMX_PRT_CFG(iface, index)) & OPIP_GMX_PRT_CFG_EN;
}

/**
 * opip_gmx_port_enable - enable or disable a port's MAC
 * @iface: Interface
 * @index: Port on it
 * @en:    Enable
 * @pknd:  The port kind, on the chips that have them, when enabling
 */
void opip_gmx_port_enable(int iface, int index, bool en, int pknd)
{
	u64 cfg = opip_rd(OPIP_GMX_PRT_CFG(iface, index));

	if (en) {
		cfg |= OPIP_GMX_PRT_CFG_EN;
		if (opip_chip->cn68xx)
			cfg = u64_replace_bits(cfg, pknd,
					       OPIP_GMX_PRT_CFG_PKND);
	} else {
		cfg &= ~OPIP_GMX_PRT_CFG_EN;
	}
	opip_wr(OPIP_GMX_PRT_CFG(iface, index), cfg);
}

/**
 * opip_gmx_set_mac - set a port's MAC address
 * @iface: Interface
 * @index: Port on it
 * @addr:  The address
 *
 * The MAC address: the source of PAUSE frames and the one CAM entry.
 */
void opip_gmx_set_mac(int iface, int index, const u8 *addr)
{
	u64 cfg = opip_rd(OPIP_GMX_PRT_CFG(iface, index));
	u64 mac = 0;
	int i;

	for (i = 0; i < ETH_ALEN; i++)
		mac = (mac << 8) | addr[i];

	/* The port is disabled while the filters change */
	opip_wr(OPIP_GMX_PRT_CFG(iface, index), cfg & ~OPIP_GMX_PRT_CFG_EN);
	opip_wr(OPIP_GMX_SMAC(iface, index), mac);
	for (i = 0; i < ETH_ALEN; i++)
		opip_wr(OPIP_GMX_RX_ADR_CAM(iface, index, i), addr[i]);
	opip_wr(OPIP_GMX_PRT_CFG(iface, index), cfg);
}

/**
 * opip_gmx_set_rx_filter - the receive filter of a port
 * @iface:    Interface
 * @index:    Port on it
 * @promisc:  Accept everything
 * @allmulti: Accept all multicast
 *
 * Broadcast is always accepted; multicast is all or nothing, the CAM has
 * no room for a list. The port is disabled while the filter changes.
 */
void opip_gmx_set_rx_filter(int iface, int index, bool promisc, bool allmulti)
{
	u64 cfg = opip_rd(OPIP_GMX_PRT_CFG(iface, index));
	u64 ctl = OPIP_GMX_RX_ADR_CTL_BCST;

	ctl |= FIELD_PREP(OPIP_GMX_RX_ADR_CTL_MCST,
			  allmulti || promisc ?
			  OPIP_GMX_RX_ADR_CTL_MCST_ACCEPT :
			  OPIP_GMX_RX_ADR_CTL_MCST_REJECT);
	/* With the CAM off everything matches; otherwise the CAM decides */
	if (!promisc)
		ctl |= OPIP_GMX_RX_ADR_CTL_CAM_MODE;

	opip_wr(OPIP_GMX_PRT_CFG(iface, index), cfg & ~OPIP_GMX_PRT_CFG_EN);
	opip_wr(OPIP_GMX_RX_ADR_CTL(iface, index), ctl);
	opip_wr(OPIP_GMX_RX_ADR_CAM_EN(iface, index), !promisc);
	opip_wr(OPIP_GMX_PRT_CFG(iface, index), cfg);
}

/**
 * opip_gmx_set_frame_max - the largest frame a port accepts
 * @iface:     Interface
 * @index:     Port on it
 * @max_frame: Bytes, FCS included
 *
 * CN3XXX and CN58XX flag larger frames as errors; the others truncate
 * them at the jabber limit, a multiple of 8, and flag them with the
 * jabber error (GMX_RX_FRM_CHK[JABBER], set at reset, not changed). The
 * PIP's length check of the later chips is for several ports, see
 * opip_set_hw_mtu().
 */
void opip_gmx_set_frame_max(int iface, int index, unsigned int max_frame)
{
	if (opip_chip->gmx_frm_max)
		opip_wr(OPIP_GMX_RX_FRM_MAX(iface, index), max_frame);
	opip_wr(OPIP_GMX_RX_JABBER(iface, index), round_up(max_frame, 8));
}

/**
 * opip_gmx_wait_idle - wait for a disabled port's GMX to go idle
 * @iface: Interface
 * @index: Port on it
 *
 * Wait for the receive and transmit state machines of a disabled port.
 *
 * Return: 0, or -ETIMEDOUT if either does not go idle in 10 ms
 */
int opip_gmx_wait_idle(int iface, int index)
{
	u64 cfg;

	if (read_poll_timeout(opip_rd, cfg, cfg & OPIP_GMX_PRT_CFG_RX_IDLE,
			      10, 10000, false,
			      OPIP_GMX_PRT_CFG(iface, index)) ||
	    read_poll_timeout(opip_rd, cfg, cfg & OPIP_GMX_PRT_CFG_TX_IDLE,
			      10, 10000, false,
			      OPIP_GMX_PRT_CFG(iface, index)))
		return -ETIMEDOUT;
	return 0;
}

/**
 * opip_gmx_set_speed - program a port's MAC for a link's speed and duplex
 * @iface: Interface
 * @index: Port on it
 * @link:  The link; a down link keeps the speed and the duplex
 *
 * The caller has the port disabled.
 */
void opip_gmx_set_speed(int iface, int index, const struct opip_link *link)
{
	u64 cfg = opip_rd(OPIP_GMX_PRT_CFG(iface, index));
	unsigned int speed = link->speed;

	if (link->up) {
		if (link->full_duplex)
			cfg |= OPIP_GMX_PRT_CFG_DUPLEX;
		else
			cfg &= ~OPIP_GMX_PRT_CFG_DUPLEX;
	}

	switch (speed) {
	case 10:
		cfg &= ~(OPIP_GMX_PRT_CFG_SPEED | OPIP_GMX_PRT_CFG_SLOTTIME);
		cfg |= OPIP_GMX_PRT_CFG_SPEED_MSB;
		opip_wr(OPIP_GMX_TX_SLOT(iface, index),
			OPIP_GMX_TX_SLOT_10_100);
		opip_wr(OPIP_GMX_TX_BURST(iface, index),
			OPIP_GMX_TX_BURST_10_100);
		break;
	case 100:
		cfg &= ~(OPIP_GMX_PRT_CFG_SPEED | OPIP_GMX_PRT_CFG_SLOTTIME |
			 OPIP_GMX_PRT_CFG_SPEED_MSB);
		opip_wr(OPIP_GMX_TX_SLOT(iface, index),
			OPIP_GMX_TX_SLOT_10_100);
		opip_wr(OPIP_GMX_TX_BURST(iface, index),
			OPIP_GMX_TX_BURST_10_100);
		break;
	case 1000:
		cfg |= OPIP_GMX_PRT_CFG_SPEED | OPIP_GMX_PRT_CFG_SLOTTIME;
		cfg &= ~OPIP_GMX_PRT_CFG_SPEED_MSB;
		opip_wr(OPIP_GMX_TX_SLOT(iface, index), OPIP_GMX_TX_SLOT_1000);
		opip_wr(OPIP_GMX_TX_BURST(iface, index),
			OPIP_GMX_TX_BURST_1000);
		break;
	default:
		break;
	}

	opip_wr(OPIP_GMX_PRT_CFG(iface, index), cfg);
	opip_rd(OPIP_GMX_PRT_CFG(iface, index));
}

/**
 * opip_gmx_stats_init - set a port's MAC counters to clear on read
 * @iface: Interface
 * @index: Port on it
 *
 * The port's counters clear on read from now on; what the bootloader
 * collected is read and dropped.
 */
void opip_gmx_stats_init(int iface, int index)
{
	struct opip_gmx_stats s;

	opip_wr(OPIP_GMX_RX_STATS_CTL(iface, index), OPIP_GMX_STATS_CTL_RD_CLR);
	opip_wr(OPIP_GMX_TX_STATS_CTL(iface, index), OPIP_GMX_STATS_CTL_RD_CLR);
	opip_gmx_read_stats(iface, index, &s);
}

/**
 * opip_gmx_read_stats - read and clear the MAC counters of a port
 * @iface: Interface
 * @index: Port on it
 * @s:     What was counted since the last read
 *
 * The caller accumulates: the hardware counters are 32 bits wide (48 for
 * octets) and clear on every read.
 */
void opip_gmx_read_stats(int iface, int index, struct opip_gmx_stats *s)
{
	u64 st[OPIP_GMX_TX_STAT_CTL + 1];
	int i;

	s->rx_ctl_packets =
		FIELD_GET(OPIP_GMX_RX_STATS_PKTS_CNT,
			  opip_rd(OPIP_GMX_RX_STATS_PKTS_CTL(iface, index)));
	s->rx_dmac_packets =
		FIELD_GET(OPIP_GMX_RX_STATS_PKTS_CNT,
			  opip_rd(OPIP_GMX_RX_STATS_PKTS_DMAC(iface, index)));
	s->rx_drop_packets =
		FIELD_GET(OPIP_GMX_RX_STATS_PKTS_CNT,
			  opip_rd(OPIP_GMX_RX_STATS_PKTS_DRP(iface, index)));

	for (i = 0; i < ARRAY_SIZE(st); i++)
		st[i] = opip_rd(OPIP_GMX_TX_STAT(iface, index, i));
	s->tx_excessive_deferrals = FIELD_GET(OPIP_GMX_TX_STAT_HI,
					      st[OPIP_GMX_TX_STAT_COLLISIONS]);
	s->tx_excessive_collisions = FIELD_GET(OPIP_GMX_TX_STAT_LO,
					       st[OPIP_GMX_TX_STAT_COLLISIONS]);
	s->tx_single_collisions = FIELD_GET(OPIP_GMX_TX_STAT_HI,
					    st[OPIP_GMX_TX_STAT_COLLISIONS_N]);
	s->tx_multiple_collisions =
		FIELD_GET(OPIP_GMX_TX_STAT_LO,
			  st[OPIP_GMX_TX_STAT_COLLISIONS_N]);
	s->tx_octets = FIELD_GET(OPIP_GMX_TX_STAT2_OCTS,
				 st[OPIP_GMX_TX_STAT_OCTS]);
	s->tx_packets = FIELD_GET(OPIP_GMX_TX_STAT_LO,
				  st[OPIP_GMX_TX_STAT_PKTS]);
	for (i = 0; i < OPIP_GMX_HIST_BINS; i += 2) {
		u64 hist = st[OPIP_GMX_TX_STAT_HIST + i / 2];

		s->tx_hist[i] = FIELD_GET(OPIP_GMX_TX_STAT_LO, hist);
		s->tx_hist[i + 1] = FIELD_GET(OPIP_GMX_TX_STAT_HI, hist);
	}
	s->tx_multicast = FIELD_GET(OPIP_GMX_TX_STAT_HI,
				    st[OPIP_GMX_TX_STAT_CAST]);
	s->tx_broadcast = FIELD_GET(OPIP_GMX_TX_STAT_LO,
				    st[OPIP_GMX_TX_STAT_CAST]);
	s->tx_underflow = FIELD_GET(OPIP_GMX_TX_STAT_HI,
				    st[OPIP_GMX_TX_STAT_CTL]);
	s->tx_ctl_packets = FIELD_GET(OPIP_GMX_TX_STAT_LO,
				      st[OPIP_GMX_TX_STAT_CTL]);
}

/**
 * opip_gmx_bp_state - read the backpressure state of an interface
 * @iface:     Interface
 * @num_ports: Ports on it; the marks of the first four are read
 * @s:         The state: GMX_TX_OVR_BP, GMX_RX_PRT_INFO and the FIFO marks
 *
 * The backpressure state of an interface, for the debugfs status.
 */
void opip_gmx_bp_state(int iface, int num_ports, struct opip_gmx_bp *s)
{
	u64 ovr, info;
	int index;

	ovr = opip_rd(OPIP_GMX_TX_OVR_BP(iface));
	info = opip_rd(OPIP_GMX_RX_PRT_INFO(iface));
	memset(s, 0, sizeof(*s));
	s->ovr_en = FIELD_GET(OPIP_GMX_TX_OVR_BP_EN, ovr);
	s->ovr_bp = FIELD_GET(OPIP_GMX_TX_OVR_BP_BP, ovr);
	s->ign_full = FIELD_GET(OPIP_GMX_TX_OVR_BP_IGN_FULL, ovr);
	s->rx_commit = FIELD_GET(OPIP_GMX_RX_PRT_INFO_COMMIT, info);
	s->rx_drop = FIELD_GET(OPIP_GMX_RX_PRT_INFO_DROP, info);
	for (index = 0; index < min(num_ports, 4); index++) {
		s->bp_on[index] =
			FIELD_GET(OPIP_GMX_RX_BP_ON_MARK,
				  opip_rd(OPIP_GMX_RX_BP_ON(iface, index)));
		s->bp_off[index] =
			FIELD_GET(OPIP_GMX_RX_BP_MARK,
				  opip_rd(OPIP_GMX_RX_BP_OFF(iface, index)));
		s->bp_drop[index] =
			FIELD_GET(OPIP_GMX_RX_BP_MARK,
				  opip_rd(OPIP_GMX_RX_BP_DROP(iface, index)));
	}
}

/**
 * opip_gmx_poll_errors - check and clear the error bits of a port
 * @iface: Interface
 * @index: Port on it
 * @nxa:   Check the interface's PKO_NXA as well
 *
 * The bits latch in the interrupt registers whether the interrupts are
 * enabled or not, and nothing enables them. Reported is the PKO's use of
 * a queue the interface does not have (PKO_NXA, shared by the ports of
 * the interface, checked by one of them), a driver bug. The FIFO overruns and
 * underflows are only cleared: the GMX counts the frames they cost. Only
 * these bits are cleared; the preamble check of RGMII ports reads its
 * own bit.
 *
 * Return: the OPIP_GMX_ERR_* bits that were set
 */
unsigned int opip_gmx_poll_errors(int iface, int index, bool nxa)
{
	u64 rx = opip_rd(OPIP_GMX_RX_INT_REG(iface, index));
	u64 tx = opip_rd(OPIP_GMX_TX_INT_REG(iface));
	unsigned int errors = 0;
	u64 clr = 0;

	if (rx & OPIP_GMX_RX_INT_OVRERR)
		opip_wr(OPIP_GMX_RX_INT_REG(iface, index),
			OPIP_GMX_RX_INT_OVRERR);

	/* The ports of an interface share it: each clears only its own bits */
	if (FIELD_GET(OPIP_GMX_TX_INT_UNDFLW, tx) & BIT(index))
		clr |= FIELD_PREP(OPIP_GMX_TX_INT_UNDFLW, BIT(index));
	if (nxa && (tx & OPIP_GMX_TX_INT_PKO_NXA)) {
		clr |= OPIP_GMX_TX_INT_PKO_NXA;
		errors |= OPIP_GMX_ERR_PKO_NXA;
	}
	if (clr)
		opip_wr(OPIP_GMX_TX_INT_REG(iface), clr);
	return errors;
}
