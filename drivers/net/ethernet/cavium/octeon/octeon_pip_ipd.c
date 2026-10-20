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
 * opip_ipd_enable - start packet input
 *
 * Sets IPD_EN in IPD_CTL_STATUS, after opip_ipd_config().
 */
void opip_ipd_enable(void)
{
	opip_wr(OPIP_IPD_CTL_STATUS,
		opip_rd(OPIP_IPD_CTL_STATUS) | OPIP_IPD_CTL_STATUS_IPD_EN);
}

/**
 * opip_ipd_disable - stop packet input
 *
 * Clears IPD_EN in IPD_CTL_STATUS. The buffers the IPD holds stay with
 * it, see opip_ipd_drain().
 */
void opip_ipd_disable(void)
{
	opip_wr(OPIP_IPD_CTL_STATUS,
		opip_rd(OPIP_IPD_CTL_STATUS) & ~OPIP_IPD_CTL_STATUS_IPD_EN);
}

/* The buffer a pointer FIFO entry holds, back to its pool */
static void opip_ipd_free_fifo_buf(u64 ptr_field, int pool)
{
	opip_fpa_free(opip_virt(ptr_field << 7), pool, 0);
}

/*
 * Walk a pointer FIFO of the IPD: select each entry with CENA clear, read
 * its pointer and free the buffer, then put the FIFO back to normal. The
 * pointer is in the control register, or in @val_reg (CN68XX's free FIFO).
 */
static void opip_ipd_free_fifo(u64 reg, unsigned int count, u64 raddr_mask,
			       u64 cena, u64 val_reg, u64 ptr_mask, int pool,
			       unsigned int (*raddr)(u64 ctl, unsigned int i))
{
	u64 ctl = opip_rd(reg);
	unsigned int i;

	for (i = 0; i < count; i++) {
		u64 val;

		ctl &= ~cena;
		ctl = opip_field_set_var(ctl, raddr_mask, raddr(ctl, i));
		opip_wr(reg, ctl);
		ctl = opip_rd(reg);
		val = val_reg ? opip_rd(val_reg) : ctl;
		opip_ipd_free_fifo_buf((val & ptr_mask) >> __ffs64(ptr_mask),
				       pool);
	}
	opip_wr(reg, ctl | cena);
}

static unsigned int opip_ipd_pwp_wqe_raddr(u64 ctl, unsigned int i)
{
	unsigned int max = FIELD_GET(OPIP_IPD_PWP_FIFO_MAX_CNTS, ctl);

	return max + (FIELD_GET(OPIP_IPD_PWP_FIFO_WRADDR, ctl) + i) % max;
}

static unsigned int opip_ipd_pwp_pkt_raddr(u64 ctl, unsigned int i)
{
	unsigned int max = FIELD_GET(OPIP_IPD_PWP_FIFO_MAX_CNTS, ctl);

	return (FIELD_GET(OPIP_IPD_PWP_FIFO_PRADDR, ctl) + i) % max;
}

static unsigned int opip_ipd_prc_port_raddr(u64 ctl, unsigned int i)
{
	return i % FIELD_GET(OPIP_IPD_PRC_PORT_FIFO_MAX_PKT, ctl);
}

static unsigned int opip_ipd_prc_hold_raddr(u64 ctl, unsigned int i)
{
	unsigned int max = FIELD_GET(OPIP_IPD_PRC_HOLD_FIFO_MAX_PKT, ctl);

	return (FIELD_GET(OPIP_IPD_PRC_HOLD_FIFO_PRADDR, ctl) + i) % max;
}

static unsigned int opip_ipd_free_wqe_raddr(u64 ctl, unsigned int i)
{
	unsigned int max = FIELD_GET(OPIP_IPD_FREE_FIFO_MAX_CNTS, ctl);

	return max + (FIELD_GET(OPIP_IPD_FREE_FIFO_WRADDR, ctl) + i) % max;
}

static unsigned int opip_ipd_free_pkt_raddr(u64 ctl, unsigned int i)
{
	unsigned int max = FIELD_GET(OPIP_IPD_FREE_FIFO_MAX_CNTS, ctl);

	return (FIELD_GET(OPIP_IPD_FREE_FIFO_PRADDR, ctl) + i) % max;
}

static unsigned int opip_ipd_port_raddr(u64 ctl, unsigned int i)
{
	return i % FIELD_GET(OPIP_IPD_PORT_FIFO_MAX_PKT, ctl);
}

static unsigned int opip_ipd_hold_raddr(u64 ctl, unsigned int i)
{
	unsigned int max = FIELD_GET(OPIP_IPD_HOLD_FIFO_MAX_PKT, ctl);

	return (FIELD_GET(OPIP_IPD_HOLD_FIFO_PRADDR, ctl) + i) % max;
}

/*
 * CN68XX: the same FIFOs in other registers, 33-bit pointers; the work
 * queue entries and packet pointers share the free FIFO, whose entries
 * are read through IPD_FREE_PTR_VALUE
 */
static void opip_ipd_drain_cn68xx(u64 count, int wqe_pool)
{
	u64 ptr;

	if (count & OPIP_IPD_PTR_COUNT_WQEV_CNT) {
		ptr = opip_rd(OPIP_IPD_NEXT_WQE_PTR);
		opip_ipd_free_fifo_buf(FIELD_GET(OPIP_IPD_PTR_VALUE_PTR, ptr),
				       wqe_pool);
	}
	opip_ipd_free_fifo(OPIP_IPD_FREE_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PTR_COUNT_WQE_PCNT, count),
			   OPIP_IPD_FREE_FIFO_RADDR, OPIP_IPD_FREE_FIFO_CENA,
			   OPIP_IPD_FREE_PTR_VALUE, OPIP_IPD_PTR_VALUE_PTR,
			   wqe_pool, opip_ipd_free_wqe_raddr);

	if (count & OPIP_IPD_PTR_COUNT_PKTV_CNT) {
		ptr = opip_rd(OPIP_IPD_NEXT_PKT_PTR);
		opip_ipd_free_fifo_buf(FIELD_GET(OPIP_IPD_PTR_VALUE_PTR, ptr),
				       OPIP_POOL_PACKET);
	}
	opip_ipd_free_fifo(OPIP_IPD_PORT_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PORT_FIFO_MAX_PKT,
				     opip_rd(OPIP_IPD_PORT_PTR_FIFO_CTL)),
			   OPIP_IPD_PORT_FIFO_RADDR, OPIP_IPD_PORT_FIFO_CENA,
			   0, OPIP_IPD_PORT_FIFO_PTR, OPIP_POOL_PACKET,
			   opip_ipd_port_raddr);
	opip_ipd_free_fifo(OPIP_IPD_HOLD_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PTR_COUNT_PFIF_CNT, count),
			   OPIP_IPD_HOLD_FIFO_RADDR, OPIP_IPD_HOLD_FIFO_CENA,
			   0, OPIP_IPD_HOLD_FIFO_PTR, OPIP_POOL_PACKET,
			   opip_ipd_hold_raddr);
	opip_ipd_free_fifo(OPIP_IPD_FREE_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PTR_COUNT_PKT_PCNT, count),
			   OPIP_IPD_FREE_FIFO_RADDR, OPIP_IPD_FREE_FIFO_CENA,
			   OPIP_IPD_FREE_PTR_VALUE, OPIP_IPD_PTR_VALUE_PTR,
			   OPIP_POOL_PACKET, opip_ipd_free_pkt_raddr);
}

/**
 * opip_ipd_drain - take the buffers out of the disabled IPD and reset it
 *
 * The IPD holds prefetched work queue entries and packet buffers in its
 * FIFOs. They go back to their pools, then the IPD and the PIP are reset,
 * so that the pools can be emptied (CN38XX pass 1 and 2, which cannot
 * read the pointers out, are refused at probe). The caller resets the IPD
 * again once the packet pool is empty, see opip_ipd_reset().
 */
void opip_ipd_drain(void)
{
	int wqe_pool = OPIP_POOL_WQE;
	u64 count, ptr;

	/* CN52XX and CN56XX can take the entries from the packet pool */
	if (octeon_has_feature(OCTEON_FEATURE_NO_WPTR) &&
	    (opip_rd(OPIP_IPD_CTL_STATUS) & OPIP_IPD_CTL_STATUS_NO_WPTR))
		wqe_pool = OPIP_POOL_PACKET;

	count = opip_rd(OPIP_IPD_PTR_COUNT);
	if (opip_chip->cn68xx) {
		opip_ipd_drain_cn68xx(count, wqe_pool);
		opip_ipd_reset();
		return;
	}

	if (count & OPIP_IPD_PTR_COUNT_WQEV_CNT) {
		ptr = FIELD_GET(OPIP_IPD_PTR_VALID_PTR,
				opip_rd(OPIP_IPD_WQE_PTR_VALID));
		opip_ipd_free_fifo_buf(ptr, wqe_pool);
	}
	opip_ipd_free_fifo(OPIP_IPD_PWP_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PTR_COUNT_WQE_PCNT, count),
			   OPIP_IPD_PWP_FIFO_RADDR, OPIP_IPD_PWP_FIFO_CENA,
			   0, OPIP_IPD_PWP_FIFO_PTR, wqe_pool,
			   opip_ipd_pwp_wqe_raddr);

	if (count & OPIP_IPD_PTR_COUNT_PKTV_CNT) {
		ptr = FIELD_GET(OPIP_IPD_PTR_VALID_PTR,
				opip_rd(OPIP_IPD_PKT_PTR_VALID));
		opip_ipd_free_fifo_buf(ptr, OPIP_POOL_PACKET);
	}
	opip_ipd_free_fifo(OPIP_IPD_PRC_PORT_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PRC_PORT_FIFO_MAX_PKT,
				     opip_rd(OPIP_IPD_PRC_PORT_PTR_FIFO_CTL)),
			   OPIP_IPD_PRC_PORT_FIFO_RADDR,
			   OPIP_IPD_PRC_PORT_FIFO_CENA, 0,
			   OPIP_IPD_PRC_PORT_FIFO_PTR, OPIP_POOL_PACKET,
			   opip_ipd_prc_port_raddr);
	opip_ipd_free_fifo(OPIP_IPD_PRC_HOLD_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PTR_COUNT_PFIF_CNT, count),
			   OPIP_IPD_PRC_HOLD_FIFO_RADDR,
			   OPIP_IPD_PRC_HOLD_FIFO_CENA, 0,
			   OPIP_IPD_PRC_HOLD_FIFO_PTR, OPIP_POOL_PACKET,
			   opip_ipd_prc_hold_raddr);
	opip_ipd_free_fifo(OPIP_IPD_PWP_PTR_FIFO_CTL,
			   FIELD_GET(OPIP_IPD_PTR_COUNT_PKT_PCNT, count),
			   OPIP_IPD_PWP_FIFO_RADDR, OPIP_IPD_PWP_FIFO_CENA,
			   0, OPIP_IPD_PWP_FIFO_PTR, OPIP_POOL_PACKET,
			   opip_ipd_pwp_pkt_raddr);

	opip_ipd_reset();
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

/**
 * opip_pip_frame_length - set the limits of a frame length checker
 * @n:   Checker, 0 or 1
 * @min: Shortest frame
 * @max: Longest frame
 *
 * CN5XXX and later; which ports a checker serves, see opip_set_hw_mtu().
 */
void opip_pip_frame_length(int n, unsigned int min, unsigned int max)
{
	opip_wr(OPIP_PIP_FRM_LEN_CHK(n),
		FIELD_PREP(OPIP_PIP_FRM_LEN_CHK_MINLEN, min) |
		FIELD_PREP(OPIP_PIP_FRM_LEN_CHK_MAXLEN, max));
}
