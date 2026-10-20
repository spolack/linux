/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The IPD (packet data) and PIP (packet parsing) blocks of octeon_pip.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_IPD_H
#define OCTEON_PIP_IPD_H

#include <linux/types.h>

#include "octeon_pip.h"
#include "octeon_pip_regs.h"

void opip_ipd_config(void);
void opip_ipd_disable(void);
void opip_ipd_reset(void);
void opip_ipd_red_setup(unsigned int pass, unsigned int drop);
void opip_ipd_fcs_strip(int port, bool strip);
unsigned int opip_ipd_bp_count(int port);
void opip_pip_port_config(int iface, int port, unsigned int group_order);
void opip_pip_length_checks(int port, bool enable);
void opip_pip_frame_length(int n, unsigned int min, unsigned int max);

/*
 * The IPD counts the packet buffers it fills per port and asserts
 * backpressure above the threshold of IPD_PORTn_BP_PAGE_CNT; it does not
 * count down. The receive poll subtracts the buffers of the packets it
 * took, so the count is that of the packets still waiting.
 */
static inline void opip_ipd_bp_sub(int port, int bufs)
{
	if (!bufs)
		return;
	opip_wr(OPIP_IPD_SUB_PORT_BP_PAGE_CNT,
		FIELD_PREP(OPIP_IPD_SUB_PORT_BP_PAGE_CNT_PORT, port) |
		FIELD_PREP(OPIP_IPD_SUB_PORT_BP_PAGE_CNT_PAGE_CNT,
			   (u32)-bufs &
			   OPIP_IPD_SUB_PORT_BP_PAGE_CNT_PAGE_CNT));
}

#endif /* OCTEON_PIP_IPD_H */
