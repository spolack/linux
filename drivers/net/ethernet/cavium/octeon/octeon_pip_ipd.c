// SPDX-License-Identifier: GPL-2.0
/*
 * The IPD and PIP blocks: packet input. The IPD stores packets into pool
 * buffers and builds the work queue entries, the PIP parses the packets
 * and tags them for the POW groups.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper.c, cvmx-helper-util.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include <linux/kernel.h>

#include "octeon_pip.h"
#include "octeon_pip_ipd.h"

/**
 * opip_ipd_config - the global input settings, before input is enabled
 *
 * Buffer size and the skips, the pool of the work queue entries, packet
 * data to DRAM rather than L2 (the CPU reads it once), per-port
 * backpressure on but disabled for every port until its link comes up,
 * and on a little endian kernel the entries and packets in little endian
 * words.
 */
void opip_ipd_config(void)
{
	int port;
	u64 ctl;

	for (port = 0; port < 2 * OPIP_IFACE_PORTS; port++)
		opip_wr(OPIP_IPD_PORT_BP_PAGE_CNT(port),
			FIELD_PREP(OPIP_IPD_PORT_BP_PAGE_CNT_PAGE_CNT,
				   OPIP_IPD_PORT_BP_PAGES_INIT));

	opip_wr(OPIP_IPD_1ST_MBUFF_SKIP,
		FIELD_PREP(OPIP_IPD_MBUFF_SKIP_SZ, OPIP_FIRST_SKIP / 8));
	opip_wr(OPIP_IPD_NOT_1ST_MBUFF_SKIP,
		FIELD_PREP(OPIP_IPD_MBUFF_SKIP_SZ, OPIP_LATER_SKIP / 8));
	opip_wr(OPIP_IPD_PACKET_MBUFF_SIZE,
		FIELD_PREP(OPIP_IPD_PACKET_MBUFF_SIZE_MB_SIZE,
			   OPIP_POOL_PACKET_SIZE / 8));
	/* The link pointer takes the 8 bytes before the data */
	opip_wr(OPIP_IPD_1ST_NEXT_PTR_BACK,
		FIELD_PREP(OPIP_IPD_NEXT_PTR_BACK_BACK,
			   (OPIP_FIRST_SKIP + 8) / 128));
	opip_wr(OPIP_IPD_2ND_NEXT_PTR_BACK,
		FIELD_PREP(OPIP_IPD_NEXT_PTR_BACK_BACK,
			   (OPIP_LATER_SKIP + 8) / 128));
	opip_wr(OPIP_IPD_WQE_FPA_QUEUE,
		FIELD_PREP(OPIP_IPD_WQE_FPA_QUEUE_POOL, OPIP_POOL_WQE));

	/* Whatever a previous owner set: the reset may keep it */
	ctl = opip_rd(OPIP_IPD_CTL_STATUS);
	ctl &= ~(OPIP_IPD_CTL_STATUS_NO_WPTR | OPIP_IPD_CTL_STATUS_LEN_M8 |
		 OPIP_IPD_CTL_STATUS_ADDPKT | OPIP_IPD_CTL_STATUS_NADDBUF |
		 OPIP_IPD_CTL_STATUS_IPD_EN);
	ctl = u64_replace_bits(ctl, 0, OPIP_IPD_CTL_STATUS_OPC_MODE);
	ctl |= OPIP_IPD_CTL_STATUS_PBP_EN;
	ctl = u64_replace_bits(ctl, IS_ENABLED(CONFIG_CPU_LITTLE_ENDIAN),
			       OPIP_IPD_CTL_STATUS_PKT_LEND);
	ctl = u64_replace_bits(ctl, IS_ENABLED(CONFIG_CPU_LITTLE_ENDIAN),
			       OPIP_IPD_CTL_STATUS_WQE_LEND);
	opip_wr(OPIP_IPD_CTL_STATUS, ctl);
}

/**
 * opip_ipd_disable - stop packet input
 *
 * Clears IPD_EN in IPD_CTL_STATUS. The buffers the IPD holds stay with
 * it.
 */
void opip_ipd_disable(void)
{
	opip_wr(OPIP_IPD_CTL_STATUS,
		opip_rd(OPIP_IPD_CTL_STATUS) & ~OPIP_IPD_CTL_STATUS_IPD_EN);
}

/**
 * opip_ipd_reset - reset the IPD and the PIP
 *
 * The IPD keeps its own count of the packet pool, IPD_QUE0_FREE_PAGE_CNT,
 * from the allocations and frees it sees, and RED decides on it. The
 * reset zeroes the count whatever the pool holds, and the register cannot
 * be written: reset only while the pool is empty, or RED compares with a
 * count that is off by the pool and never drops, the IPD waits for
 * buffers instead and the ports pause their links. Seen on CN70XX.
 */
void opip_ipd_reset(void)
{
	opip_wr(OPIP_IPD_CTL_STATUS,
		opip_rd(OPIP_IPD_CTL_STATUS) | OPIP_IPD_CTL_STATUS_RESET);
	opip_wr(OPIP_PIP_SFT_RST,
		opip_rd(OPIP_PIP_SFT_RST) | OPIP_PIP_SFT_RST_RST);
}

/**
 * opip_ipd_red_setup - random early drop on the packet pool level
 * @pass: Packets start to be dropped below this many free buffers
 * @drop: All packets are dropped below this many
 *
 * On all eight QoS queues, from the actual count rather than an average.
 * Backpressure by queued buffers is left off.
 */
void opip_ipd_red_setup(unsigned int pass, unsigned int drop)
{
	int queue;

	for (queue = 0; queue < 8; queue++) {
		opip_wr(OPIP_IPD_QOS_RED_MARKS(queue),
			FIELD_PREP(OPIP_IPD_RED_MARKS_DROP, drop) |
			FIELD_PREP(OPIP_IPD_RED_MARKS_PASS, pass));
		opip_wr(OPIP_IPD_RED_QUE_PARAM(queue),
			FIELD_PREP(OPIP_IPD_RED_PARAM_PRB_CON,
				   (255ul << 24) / (pass - drop)) |
			FIELD_PREP(OPIP_IPD_RED_PARAM_AVG_CON, 1) |
			FIELD_PREP(OPIP_IPD_RED_PARAM_NEW_CON, 255) |
			OPIP_IPD_RED_PARAM_USE_PCNT);
	}

	opip_wr(OPIP_IPD_BP_PRT_RED_END, 0);
	opip_wr(OPIP_IPD_RED_PORT_ENABLE,
		OPIP_IPD_RED_PORT_ENABLE_PRT_ENB |
		FIELD_PREP(OPIP_IPD_RED_PORT_ENABLE_AVG_DLY,
			   OPIP_IPD_RED_PORT_DLY) |
		FIELD_PREP(OPIP_IPD_RED_PORT_ENABLE_PRB_DLY,
			   OPIP_IPD_RED_PORT_DLY));
}

/**
 * opip_ipd_fcs_strip - set whether the IPD strips a port's FCS
 * @port:  IPD port
 * @strip: Strip the FCS of the port's packets
 */
void opip_ipd_fcs_strip(int port, bool strip)
{
	u64 reg = opip_rd(OPIP_IPD_SUB_PORT_FCS);
	u64 bit;

	if (port < 32)
		bit = FIELD_PREP(OPIP_IPD_SUB_PORT_FCS_PORT_BIT, BIT(port));
	else if (port >= 36 && port < 40)
		bit = FIELD_PREP(OPIP_IPD_SUB_PORT_FCS_PORT_BIT2,
				 BIT(port - 36));
	else
		return;
	opip_wr(OPIP_IPD_SUB_PORT_FCS, strip ? reg | bit : reg & ~bit);
}

/**
 * opip_ipd_bp_count - the buffers the IPD counts for a port
 * @port: IPD port
 *
 * Return: the buffers the IPD counts for @port right now
 */
unsigned int opip_ipd_bp_count(int port)
{
	return FIELD_GET(OPIP_IPD_PORT_BP_COUNTERS_CNT_VAL,
			 opip_rd(OPIP_IPD_PORT_BP_COUNTERS_PAIR(port)));
}

/**
 * opip_pip_port_config - how the PIP parses and tags a port's packets
 * @iface:       Interface of the port
 * @port:        IPD port
 * @group_order: The packets spread over 2^order POW groups by their flow
 *
 * The parser skips to the L2 header; the QoS queue is the port's. The
 * tag is the hash of the IP addresses and ports, which also picks the
 * group with order > 0; with one group everything goes to group 0.
 */
void opip_pip_port_config(int iface, int port, unsigned int group_order)
{
	u64 cfg = opip_rd(OPIP_PIP_PRT_CFG(port));
	u64 tag;

	/*
	 * No instruction header, no packet left in the entry alone, and
	 * the parse starts at byte 0 with no DSA or HiGig tag: whatever a
	 * previous owner set, the stack reads the frame from there
	 */
	cfg &= ~(OPIP_PIP_PRT_CFG_DYN_RS | OPIP_PIP_PRT_CFG_INST_HDR |
		 OPIP_PIP_PRT_CFG_HIGIG_EN | OPIP_PIP_PRT_CFG_DSA_EN |
		 OPIP_PIP_PRT_CFG_SKIP);
	cfg = u64_replace_bits(cfg, port & 7, OPIP_PIP_PRT_CFG_QOS);
	cfg = u64_replace_bits(cfg, OPIP_PIP_PRT_CFG_MODE_SKIP_TO_L2,
			       OPIP_PIP_PRT_CFG_MODE);
	/* CN68XX picks the length checker per port kind: the interface's */
	if (opip_chip->cn68xx && iface < 2)
		cfg = u64_replace_bits(cfg, iface,
				       OPIP_PIP_PRT_CFG_LEN_CHK_SEL);
	opip_wr(OPIP_PIP_PRT_CFG(port), cfg);

	/*
	 * The tag register is the driver's alone: build it whole, nothing a
	 * previous owner set stays. The flow fields also make the tag the
	 * skb's flow hash.
	 */
	tag = OPIP_PIP_PRT_TAG_INC_PRT_FLAG |
	      OPIP_PIP_PRT_TAG_IP6_SRC_FLAG | OPIP_PIP_PRT_TAG_IP6_DST_FLAG |
	      OPIP_PIP_PRT_TAG_IP6_SPRT_FLAG | OPIP_PIP_PRT_TAG_IP6_DPRT_FLAG |
	      OPIP_PIP_PRT_TAG_IP4_SRC_FLAG | OPIP_PIP_PRT_TAG_IP4_DST_FLAG |
	      OPIP_PIP_PRT_TAG_IP4_SPRT_FLAG | OPIP_PIP_PRT_TAG_IP4_DPRT_FLAG |
	      FIELD_PREP(OPIP_PIP_PRT_TAG_TCP6_TAG_TYPE, OPIP_TAG_ORDERED) |
	      FIELD_PREP(OPIP_PIP_PRT_TAG_TCP4_TAG_TYPE, OPIP_TAG_ORDERED) |
	      FIELD_PREP(OPIP_PIP_PRT_TAG_IP6_TAG_TYPE, OPIP_TAG_ORDERED) |
	      FIELD_PREP(OPIP_PIP_PRT_TAG_IP4_TAG_TYPE, OPIP_TAG_ORDERED) |
	      FIELD_PREP(OPIP_PIP_PRT_TAG_NON_TAG_TYPE, OPIP_TAG_ORDERED);
	if (group_order) {
		/* 16 groups at most: the hidden mask bits of CN68XX are set */
		if (opip_chip->cn68xx)
			tag |= OPIP_PIP_PRT_TAG_GRPTAGMASK_MSB;
		tag |= OPIP_PIP_PRT_TAG_GRPTAG;
		tag |= FIELD_PREP(OPIP_PIP_PRT_TAG_GRPTAGMASK,
				  ~((1 << group_order) - 1) & 0xf);
	}
	opip_wr(OPIP_PIP_PRT_TAG(port), tag);
}

/**
 * opip_pip_length_checks - turn the length errors of a port on or off
 * @port:   IPD port
 * @enable: Check the minimum and maximum length
 *
 * Length errors: off for the LOOP and NPI ports, which carry anything.
 */
void opip_pip_length_checks(int port, bool enable)
{
	u64 cfg = opip_rd(OPIP_PIP_PRT_CFG(port));

	if (enable)
		cfg |= OPIP_PIP_PRT_CFG_MAXERR_EN | OPIP_PIP_PRT_CFG_MINERR_EN;
	else
		cfg &= ~(OPIP_PIP_PRT_CFG_MAXERR_EN |
			 OPIP_PIP_PRT_CFG_MINERR_EN);
	opip_wr(OPIP_PIP_PRT_CFG(port), cfg);
}
