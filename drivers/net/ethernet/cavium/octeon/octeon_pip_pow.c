// SPDX-License-Identifier: GPL-2.0
/*
 * The POW work scheduler (SSO on CN68XX): group masks and interrupts.
 *
 * Derived in part from the staging octeon driver (ethernet-rx.c) and
 * arch/mips/include/asm/octeon/cvmx-pow.h,
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include "octeon_pip_pow.h"

/**
 * opip_pow_select_group - have a core take work of one group only
 * @core:  This core
 * @group: The group
 *
 * Return: the previous mask, for opip_pow_restore_groups()
 */
u64 opip_pow_select_group(unsigned int core, int group)
{
	u64 old;

	if (opip_chip->cn68xx) {
		old = opip_rd(OPIP_SSO_PP_GRP_MSK(core));
		opip_wr(OPIP_SSO_PP_GRP_MSK(core), BIT_ULL(group));
		opip_rd(OPIP_SSO_PP_GRP_MSK(core));	/* flush */
	} else {
		old = opip_rd(OPIP_POW_PP_GRP_MSK(core));
		/* The priorities stay */
		opip_wr(OPIP_POW_PP_GRP_MSK(core),
			u64_replace_bits(old, BIT(group),
					 OPIP_POW_PP_GRP_MSK_GRP_MSK));
	}
	return old;
}

/**
 * opip_pow_restore_groups - give a core back its group mask
 * @core: This core
 * @mask: The mask opip_pow_select_group() returned
 */
void opip_pow_restore_groups(unsigned int core, u64 mask)
{
	if (opip_chip->cn68xx) {
		opip_wr(OPIP_SSO_PP_GRP_MSK(core), mask);
		opip_rd(OPIP_SSO_PP_GRP_MSK(core));	/* flush */
	} else {
		opip_wr(OPIP_POW_PP_GRP_MSK(core), mask);
	}
}
