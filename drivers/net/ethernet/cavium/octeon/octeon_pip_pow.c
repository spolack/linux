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

/**
 * opip_pow_group_ack - acknowledge a group's interrupt
 * @group: The group
 *
 * The group is empty: acknowledge its interrupt.
 */
void opip_pow_group_ack(int group)
{
	if (opip_chip->cn68xx) {
		opip_wr(OPIP_SSO_WQ_IQ_DIS, BIT_ULL(group));
		opip_wr(OPIP_SSO_WQ_INT, BIT_ULL(group));
	} else {
		opip_wr(OPIP_POW_WQ_INT,
			FIELD_PREP(OPIP_POW_WQ_INT_IQ_DIS, BIT(group)) |
			FIELD_PREP(OPIP_POW_WQ_INT_WQ_INT, BIT(group)));
	}
}

/**
 * opip_pow_group_irq - turn a group's work interrupt on or off
 * @group:  The group
 * @enable: Turn it on
 *
 * Interrupt as soon as the group has work: the time counter with a
 * threshold of one, checked every 5 * 1024 cycles.
 */
void opip_pow_group_irq(int group, bool enable)
{
	if (opip_chip->cn68xx) {
		opip_wr(OPIP_SSO_WQ_INT_THR(group),
			enable ? OPIP_SSO_WQ_INT_THR_TC_EN |
				 FIELD_PREP(OPIP_SSO_WQ_INT_THR_TC_THR, 1) : 0);
		opip_wr(OPIP_SSO_WQ_INT_PC,
			FIELD_PREP(OPIP_POW_WQ_INT_PC_PC_THR, 5));
	} else {
		opip_wr(OPIP_POW_WQ_INT_THR(group),
			enable ? OPIP_POW_WQ_INT_THR_TC_EN |
				 FIELD_PREP(OPIP_POW_WQ_INT_THR_TC_THR, 1) : 0);
		opip_wr(OPIP_POW_WQ_INT_PC,
			FIELD_PREP(OPIP_POW_WQ_INT_PC_PC_THR, 5));
	}
}

/**
 * opip_pow_queued - entries waiting for a group
 * @group: The group, or -1 for all groups together
 *
 * Return: how much work (received packets, transmit completions) the
 * cores have not taken yet
 */
unsigned int opip_pow_queued(int group)
{
	if (group < 0)
		return FIELD_GET(OPIP_POW_IQ_CNT_IQ_CNT,
				 opip_rd(opip_chip->cn68xx ?
					 OPIP_SSO_IQ_COM_CNT :
					 OPIP_POW_IQ_COM_CNT));
	return FIELD_GET(OPIP_POW_WQ_INT_CNT_IQ_CNT,
			 opip_rd(opip_chip->cn68xx ?
				 OPIP_SSO_WQ_INT_CNT(group) :
				 OPIP_POW_WQ_INT_CNT(group)));
}

/**
 * opip_pow_qos_queued - entries waiting in a QOS level
 * @qos: The QOS level (a port's is port & 7)
 *
 * Return: the entries in the input queue of @qos
 */
unsigned int opip_pow_qos_queued(int qos)
{
	return FIELD_GET(OPIP_POW_IQ_CNT_IQ_CNT,
			 opip_rd(opip_chip->cn68xx ?
				 OPIP_SSO_IQ_CNT(qos) : OPIP_POW_IQ_CNT(qos)));
}

/**
 * opip_pow_int_pending - groups with a raised interrupt
 *
 * Return: a mask of the groups whose interrupt is raised right now
 */
unsigned int opip_pow_int_pending(void)
{
	if (opip_chip->cn68xx)
		return opip_rd(OPIP_SSO_WQ_INT) & OPIP_POW_WQ_INT_WQ_INT;
	return FIELD_GET(OPIP_POW_WQ_INT_WQ_INT, opip_rd(OPIP_POW_WQ_INT));
}
