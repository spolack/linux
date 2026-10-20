/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Registers of the OCTEON packet I/O blocks the octeon_pip driver drives:
 * the addresses, per chip family where they differ, and the fields.
 *
 * The register names follow the hardware reference manuals (GMX_PRTn_CFG,
 * IPD_CTL_STATUS, ...). A register is addressed by its 64-bit CSR address
 * as the hardware decodes it; the simulator model decodes the same
 * addresses, so nothing here depends on the build.
 *
 * Derived in part from the register definitions of the Cavium executive
 * (arch/mips/include/asm/octeon/cvmx-*-defs.h),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_REGS_H
#define OCTEON_PIP_REGS_H

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/types.h>

#include "octeon_pip_hw.h"

/* The I/O segment every CSR lives in */
#define OPIP_CSR(addr)		(0x8000000000000000ull | (addr))

/*
 * The GMX, ASX and PCS blocks have one register set per interface. The
 * interfaces are 0x8000000 apart, except on CN68XX, which has more of
 * them and packs them 0x1000000 apart.
 */
static inline u64 opip_iface_stride(void)
{
	return opip_chip->iface_stride;
}

/* ------------------------------------------------------------------ */
/* GMX: the Ethernet MACs of an interface, up to four ports */

#define OPIP_GMX_BASE		0x0001180008000000ull

/* A register of the GMX of an interface */
static inline u64 opip_gmx_reg(int iface, u64 off)
{
	return OPIP_CSR(OPIP_GMX_BASE + off + (u64)iface * opip_iface_stride());
}

#define OPIP_GMX_INF_MODE(i)		opip_gmx_reg(i, 0x7F8)

/* GMX_INF_MODE */
#define OPIP_GMX_INF_MODE_MODE		GENMASK_ULL(6, 4)
#define OPIP_GMX_INF_MODE_MODE_SGMII	1
#define OPIP_GMX_INF_MODE_MODE_QSGMII	2
#define OPIP_GMX_INF_MODE_P0MII		BIT_ULL(2)
#define OPIP_GMX_INF_MODE_EN		BIT_ULL(1)
#define OPIP_GMX_INF_MODE_TYPE		BIT_ULL(0)

#endif /* OCTEON_PIP_REGS_H */
