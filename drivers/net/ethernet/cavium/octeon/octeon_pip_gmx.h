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

void opip_gmx_iface_enable(int iface);
bool opip_gmx_iface_is_qsgmii(int iface);
void opip_gmx_port_enable(int iface, int index, bool en, int pknd);
void opip_gmx_set_mac(int iface, int index, const u8 *addr);
void opip_gmx_set_rx_filter(int iface, int index, bool promisc, bool allmulti);
void opip_gmx_set_frame_max(int iface, int index, unsigned int max_frame);

#endif /* OCTEON_PIP_GMX_H */
