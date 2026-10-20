/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The PKO (packet output) block of octeon_pip: the output queues, their
 * command chunks and doorbells.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-pko.c, arch/mips/include/asm/octeon/cvmx-pko.h),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_PKO_H
#define OCTEON_PIP_PKO_H

#include <linux/types.h>

#include "octeon_pip.h"
#include "octeon_pip_regs.h"

void opip_pko_layout(struct opip *opip, int queues_per_port);
int opip_pko_base_queue(const struct opip *opip, int port);
int opip_pko_num_queues(const struct opip *opip, int port);
bool opip_pko_port_valid(const struct opip *opip, int port);
void opip_pko_init(struct opip *opip);
void opip_pko_disable(void);
void opip_pko_reset(void);
void opip_pko_gmx_port_mode(int iface, int num_ports);
#endif /* OCTEON_PIP_PKO_H */
