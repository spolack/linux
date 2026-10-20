/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The hardware layer of octeon_pip without OCTEON hardware (COMPILE_TEST):
 * the arch definitions the driver uses, and the primitives as no-ops.
 */
#ifndef OCTEON_PIP_HW_STUB_H
#define OCTEON_PIP_HW_STUB_H

#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/types.h>

/* The chip models and features, none of which an x86 build has */
enum {
	OCTEON_CN30XX, OCTEON_CN30XX_PASS1, OCTEON_CN31XX,
	OCTEON_CN31XX_PASS1, OCTEON_CN38XX, OCTEON_CN38XX_PASS1,
	OCTEON_CN38XX_PASS2, OCTEON_CN50XX, OCTEON_CN52XX,
	OCTEON_CN52XX_PASS1_0, OCTEON_CN52XX_PASS1_X, OCTEON_CN56XX,
	OCTEON_CN56XX_PASS1_X, OCTEON_CN58XX, OCTEON_CN61XX,
	OCTEON_CN63XX, OCTEON_CN66XX, OCTEON_CN68XX, OCTEON_CN68XX_PASS1_X,
	OCTEON_CN68XX_PASS2_X, OCTEON_CN70XX, OCTEON_CNF71XX,
};

#define OCTEON_IS_MODEL(x)	((void)(x), false)

static inline u64 opip_rd(u64 addr)
{
	return 0;
}

static inline void opip_wr(u64 addr, u64 val)
{
}

static inline int opip_num_interfaces(void)
{
	return 0;
}

static inline enum opip_mode opip_iface_mode(int iface)
{
	return OPIP_MODE_DISABLED;
}

#endif /* OCTEON_PIP_HW_STUB_H */
