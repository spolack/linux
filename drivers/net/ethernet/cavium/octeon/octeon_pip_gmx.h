/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The GMX blocks of octeon_pip: the Ethernet MACs of an interface, with
 * the ASX pads of the RGMII ports.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper-rgmii.c, cvmx-helper-util.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_GMX_H
#define OCTEON_PIP_GMX_H

#include <linux/types.h>

#include "octeon_pip.h"

/* The error bits opip_gmx_poll_errors() reports */
#define OPIP_GMX_ERR_PKO_NXA		BIT(0)

/* The backpressure state of an interface, a bit per port in each field */
struct opip_gmx_bp {
	/* GMX_TX_OVR_BP: the overrides of what the IPD says */
	unsigned int ovr_en;
	unsigned int ovr_bp;
	unsigned int ign_full;
	/* GMX_RX_PRT_INFO: the receive FIFO holds data, is dropping */
	unsigned int rx_commit;
	unsigned int rx_drop;
	/* GMX_RX_BP_ON/OFF/DROPn: the FIFO marks of each port */
	unsigned int bp_on[4];
	unsigned int bp_off[4];
	unsigned int bp_drop[4];
};

void opip_gmx_iface_enable(int iface);
bool opip_gmx_iface_is_qsgmii(int iface);
int opip_gmx_setup(int iface, int num_ports);
bool opip_gmx_port_enabled(int iface, int index);
void opip_gmx_port_enable(int iface, int index, bool en, int pknd);
void opip_gmx_set_mac(int iface, int index, const u8 *addr);
void opip_gmx_set_rx_filter(int iface, int index, bool promisc, bool allmulti);
void opip_gmx_set_frame_max(int iface, int index, unsigned int max_frame);
void opip_gmx_set_speed(int iface, int index, const struct opip_link *link);
int opip_gmx_wait_idle(int iface, int index);
void opip_gmx_bp_state(int iface, int num_ports, struct opip_gmx_bp *s);
unsigned int opip_gmx_poll_errors(int iface, int index, bool nxa);
void opip_gmx_stats_init(int iface, int index);
void opip_gmx_read_stats(int iface, int index, struct opip_gmx_stats *s);

#endif /* OCTEON_PIP_GMX_H */
