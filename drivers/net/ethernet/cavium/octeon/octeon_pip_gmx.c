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
#include <linux/kernel.h>

#include "octeon_pip.h"
#include "octeon_pip_gmx.h"
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
