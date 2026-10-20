/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The SGMII/QSGMII PCS of a port.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper-sgmii.c, cvmx-helper-xaui.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_PCS_H
#define OCTEON_PIP_PCS_H

#include <linux/phylink.h>
#include <linux/types.h>

#include "octeon_pip.h"

extern const struct phylink_pcs_ops opip_pcs_ops;

bool opip_pcs_basex(int iface, int index);
int opip_sgmii_enable(int iface, int num_ports);
int opip_pcs_config(int iface, int index, bool inband);
void opip_pcs_an_restart(int iface, int index);
struct opip_link opip_pcs_link(int iface, int index);
int opip_pcs_link_up(int iface, int index, const struct opip_link *link,
		     bool inband);
int opip_pcs_link_set(int iface, int index, const struct opip_link *link);
#endif /* OCTEON_PIP_PCS_H */
