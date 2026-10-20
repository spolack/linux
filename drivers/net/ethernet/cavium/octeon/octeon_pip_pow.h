/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The POW work scheduler (SSO on CN68XX) as octeon_pip uses it: the groups
 * that deliver received packets and transmit completions, and the work
 * requests of the receive poll.
 *
 * Derived in part from arch/mips/include/asm/octeon/cvmx-pow.h,
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_POW_H
#define OCTEON_PIP_POW_H

#include <linux/prefetch.h>
#include <linux/types.h>

#include "octeon_pip_hw.h"
#include "octeon_pip_regs.h"

/* The scratch word the asynchronous work requests answer into */
#define OPIP_POW_SCRATCH	0

/* The I/O address a work request loads from */
static inline u64 opip_pow_request_addr(bool wait)
{
	return OPIP_CSR(OPIP_POW_LOAD_IS_IO |
			FIELD_PREP(OPIP_POW_LOAD_DID, OPIP_POW_DID_SWTAG) |
			(wait ? OPIP_POW_LOAD_WAIT : 0));
}

static inline struct opip_wqe *opip_pow_response(u64 resp)
{
	if (resp & OPIP_POW_RESP_NO_WORK)
		return NULL;
	return opip_virt(FIELD_GET(OPIP_POW_RESP_ADDR, resp));
}

/* Ask for work and wait for the answer */
static inline struct opip_wqe *opip_pow_request_sync(bool wait)
{
	return opip_pow_response(opip_rd(opip_pow_request_addr(wait)));
}

/* Ask for work; the answer lands in the core's scratch memory */
static inline void opip_pow_request_async(bool wait)
{
	opip_iobdma(FIELD_PREP(OPIP_POW_IOBDMA_SCRADDR, OPIP_POW_SCRATCH >> 3) |
		    FIELD_PREP(OPIP_POW_IOBDMA_LEN, 1) |
		    FIELD_PREP(OPIP_POW_IOBDMA_DID, OPIP_POW_DID_SWTAG) |
		    (wait ? OPIP_POW_IOBDMA_WAIT : 0));
}

/* The answer to opip_pow_request_async() */
static inline struct opip_wqe *opip_pow_response_async(void)
{
	opip_iobdma_wait();
	return opip_pow_response(opip_scratch_rd(OPIP_POW_SCRATCH));
}

u64 opip_pow_select_group(unsigned int core, int group);
void opip_pow_restore_groups(unsigned int core, u64 mask);
void opip_pow_group_ack(int group);
void opip_pow_group_irq(int group, bool enable);

#endif /* OCTEON_PIP_POW_H */
