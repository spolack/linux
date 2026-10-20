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
#include <linux/mii.h>
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

/* A register of one port of a GMX: 2048 bytes between the ports */
static inline u64 opip_gmx_port_reg(int iface, int index, u64 off)
{
	return OPIP_CSR(OPIP_GMX_BASE + off + (u64)index * 2048 +
			(u64)iface * opip_iface_stride());
}

/* A register of the GMX of an interface */
static inline u64 opip_gmx_reg(int iface, u64 off)
{
	return OPIP_CSR(OPIP_GMX_BASE + off + (u64)iface * opip_iface_stride());
}

#define OPIP_GMX_PRT_CFG(i, p)		opip_gmx_port_reg(i, p, 0x010)
#define OPIP_GMX_RX_FRM_CTL(i, p)	opip_gmx_port_reg(i, p, 0x018)
#define OPIP_GMX_RX_FRM_MAX(i, p)	opip_gmx_port_reg(i, p, 0x030)
#define OPIP_GMX_RX_JABBER(i, p)	opip_gmx_port_reg(i, p, 0x038)
#define OPIP_GMX_RX_ADR_CTL(i, p)	opip_gmx_port_reg(i, p, 0x100)
#define OPIP_GMX_RX_ADR_CAM_EN(i, p)	opip_gmx_port_reg(i, p, 0x108)
#define OPIP_GMX_RX_ADR_CAM(i, p, n)	opip_gmx_port_reg(i, p, 0x180 + (n) * 8)
#define OPIP_GMX_TX_THRESH(i, p)	opip_gmx_port_reg(i, p, 0x210)
#define OPIP_GMX_TX_APPEND(i, p)	opip_gmx_port_reg(i, p, 0x218)
#define OPIP_GMX_TX_SLOT(i, p)		opip_gmx_port_reg(i, p, 0x220)
#define OPIP_GMX_TX_BURST(i, p)		opip_gmx_port_reg(i, p, 0x228)
#define OPIP_GMX_SMAC(i, p)		opip_gmx_port_reg(i, p, 0x230)
#define OPIP_GMX_RX_PRTS(i)		opip_gmx_reg(i, 0x410)
#define OPIP_GMX_TX_PRTS(i)		opip_gmx_reg(i, 0x480)
#define OPIP_GMX_INF_MODE(i)		opip_gmx_reg(i, 0x7F8)

/* GMX_PRTn_CFG */
#define OPIP_GMX_PRT_CFG_PKND		GENMASK_ULL(21, 16)
#define OPIP_GMX_PRT_CFG_TX_IDLE	BIT_ULL(13)
#define OPIP_GMX_PRT_CFG_RX_IDLE	BIT_ULL(12)
#define OPIP_GMX_PRT_CFG_SPEED_MSB	BIT_ULL(8)
#define OPIP_GMX_PRT_CFG_SLOTTIME	BIT_ULL(3)
#define OPIP_GMX_PRT_CFG_DUPLEX		BIT_ULL(2)
#define OPIP_GMX_PRT_CFG_SPEED		BIT_ULL(1)
#define OPIP_GMX_PRT_CFG_EN		BIT_ULL(0)

/* GMX_RXn_FRM_CTL; PTP_MODE is CN63XX and later, reserved (0) before */
#define OPIP_GMX_RX_FRM_CTL_PTP_MODE	BIT_ULL(12)
#define OPIP_GMX_RX_FRM_CTL_PRE_FREE	BIT_ULL(6)
#define OPIP_GMX_RX_FRM_CTL_CTL_BCK	BIT_ULL(3)
#define OPIP_GMX_RX_FRM_CTL_CTL_DRP	BIT_ULL(2)
#define OPIP_GMX_RX_FRM_CTL_PRE_CHK	BIT_ULL(0)

/* GMX_RXn_ADR_CTL */
#define OPIP_GMX_RX_ADR_CTL_CAM_MODE	BIT_ULL(3)
#define OPIP_GMX_RX_ADR_CTL_MCST	GENMASK_ULL(2, 1)
#define OPIP_GMX_RX_ADR_CTL_MCST_REJECT	1
#define OPIP_GMX_RX_ADR_CTL_MCST_ACCEPT	2
#define OPIP_GMX_RX_ADR_CTL_BCST	BIT_ULL(0)

/* GMX_TXn_THRESH */
#define OPIP_GMX_TX_THRESH_CNT		GENMASK_ULL(9, 0)

/* GMX_TXn_SLOT and GMX_TXn_BURST, in bit times: 10/100 and 1000 Mbit/s */
#define OPIP_GMX_TX_SLOT_10_100		64
#define OPIP_GMX_TX_SLOT_1000		512
#define OPIP_GMX_TX_BURST_10_100	0
#define OPIP_GMX_TX_BURST_1000		8192

/* GMX_TXn_APPEND */
#define OPIP_GMX_TX_APPEND_FCS		BIT_ULL(2)
#define OPIP_GMX_TX_APPEND_PAD		BIT_ULL(1)

/* GMX_TX_PRTS, GMX_RX_PRTS */
#define OPIP_GMX_TX_PRTS_PRTS		GENMASK_ULL(4, 0)
#define OPIP_GMX_RX_PRTS_PRTS		GENMASK_ULL(2, 0)

/* GMX_INF_MODE */
#define OPIP_GMX_INF_MODE_MODE		GENMASK_ULL(6, 4)
#define OPIP_GMX_INF_MODE_MODE_SGMII	1
#define OPIP_GMX_INF_MODE_MODE_QSGMII	2
#define OPIP_GMX_INF_MODE_P0MII		BIT_ULL(2)
#define OPIP_GMX_INF_MODE_EN		BIT_ULL(1)
#define OPIP_GMX_INF_MODE_TYPE		BIT_ULL(0)

/* ------------------------------------------------------------------ */
/* PCS: the SGMII/1000BASE-X PCS of a port */

#define OPIP_PCS_BASE		0x00011800B0001000ull

static inline u64 opip_pcs_reg(int iface, int index, u64 off)
{
	return OPIP_CSR(OPIP_PCS_BASE + off + (u64)index * 1024 +
			(u64)iface * opip_iface_stride());
}

#define OPIP_PCS_MR_CONTROL(i, p)	opip_pcs_reg(i, p, 0x000)
#define OPIP_PCS_MR_STATUS(i, p)	opip_pcs_reg(i, p, 0x008)
#define OPIP_PCS_AN_ADV(i, p)		opip_pcs_reg(i, p, 0x010)
#define OPIP_PCS_AN_LP_ABIL(i, p)	opip_pcs_reg(i, p, 0x018)
#define OPIP_PCS_AN_RESULTS(i, p)	opip_pcs_reg(i, p, 0x020)
#define OPIP_PCS_LINK_TIMER_COUNT(i, p)	opip_pcs_reg(i, p, 0x040)
#define OPIP_PCS_SGM_AN_ADV(i, p)	opip_pcs_reg(i, p, 0x068)
#define OPIP_PCS_MISC_CTL(i, p)		opip_pcs_reg(i, p, 0x078)
#define OPIP_PCS_INT_REG(i, p)		opip_pcs_reg(i, p, 0x080)

/* PCS_MRn_CONTROL: the clause 22 BMCR */
#define OPIP_PCS_MR_CONTROL_RESET	BMCR_RESET
#define OPIP_PCS_MR_CONTROL_LOOPBCK1	BMCR_LOOPBACK
#define OPIP_PCS_MR_CONTROL_AN_EN	BMCR_ANENABLE
#define OPIP_PCS_MR_CONTROL_PWR_DN	BMCR_PDOWN
#define OPIP_PCS_MR_CONTROL_RST_AN	BMCR_ANRESTART
/* Speed and duplex, mii_bmcr_encode_fixed() */
#define OPIP_PCS_MR_CONTROL_FIXED \
	(BMCR_SPEED1000 | BMCR_SPEED100 | BMCR_FULLDPLX)

/* PCS_MRn_STATUS */
#define OPIP_PCS_MR_STATUS_AN_CPT	BIT_ULL(5)
#define OPIP_PCS_MR_STATUS_LNK_ST	BIT_ULL(2)

/* PCS_ANn_ADV (1000BASE-X) */
#define OPIP_PCS_AN_ADV_REM_FLT		GENMASK_ULL(13, 12)
#define OPIP_PCS_AN_ADV_PAUSE		GENMASK_ULL(8, 7)
#define OPIP_PCS_AN_ADV_HFD		BIT_ULL(6)
#define OPIP_PCS_AN_ADV_FD		BIT_ULL(5)

/* PCS_ANn_RESULTS */
#define OPIP_PCS_AN_RESULTS_SPD		GENMASK_ULL(4, 3)
#define OPIP_PCS_AN_RESULTS_AN_CPT	BIT_ULL(2)
#define OPIP_PCS_AN_RESULTS_DUP		BIT_ULL(1)
#define OPIP_PCS_AN_RESULTS_LINK_OK	BIT_ULL(0)

/* PCS_LINKn_TIMER_COUNT */
#define OPIP_PCS_LINK_TIMER_COUNT_COUNT	GENMASK_ULL(15, 0)

/* PCS_SGMn_AN_ADV (SGMII PHY mode) */
#define OPIP_PCS_SGM_AN_ADV_LINK	BIT_ULL(15)
#define OPIP_PCS_SGM_AN_ADV_DUP		BIT_ULL(12)
#define OPIP_PCS_SGM_AN_ADV_SPEED	GENMASK_ULL(11, 10)

/* PCS_MISCn_CTL */
#define OPIP_PCS_MISC_CTL_GMXENO	BIT_ULL(11)
#define OPIP_PCS_MISC_CTL_MAC_PHY	BIT_ULL(9)
#define OPIP_PCS_MISC_CTL_MODE		BIT_ULL(8)
#define OPIP_PCS_MISC_CTL_SAMP_PT	GENMASK_ULL(6, 0)

/* PCS_INTn_REG */
#define OPIP_PCS_INT_TXFIFO		BIT_ULL(4)
#define OPIP_PCS_INT_TXFIFU		BIT_ULL(3)
#define OPIP_PCS_INT_AN_ERR		BIT_ULL(2)

/* ------------------------------------------------------------------ */
/* PIP: packet input parsing, per input port */

#define OPIP_PIP_BASE		0x00011800A0000000ull

#define OPIP_PIP_REG(off)		OPIP_CSR(OPIP_PIP_BASE + (off))
#define OPIP_PIP_SFT_RST		OPIP_PIP_REG(0x030)
#define OPIP_PIP_FRM_LEN_CHK(n)		OPIP_PIP_REG(0x180 + (n) * 8)
#define OPIP_PIP_PRT_CFG(p)		OPIP_PIP_REG(0x200 + (p) * 8)
#define OPIP_PIP_PRT_TAG(p)		OPIP_PIP_REG(0x400 + (p) * 8)

#define OPIP_PIP_SFT_RST_RST		BIT_ULL(0)

/* PIP_FRM_LEN_CHKn */
#define OPIP_PIP_FRM_LEN_CHK_MAXLEN	GENMASK_ULL(31, 16)
#define OPIP_PIP_FRM_LEN_CHK_MINLEN	GENMASK_ULL(15, 0)

/* PIP_PRT_CFGn */
#define OPIP_PIP_PRT_CFG_LEN_CHK_SEL	BIT_ULL(53)
#define OPIP_PIP_PRT_CFG_MAXERR_EN	BIT_ULL(49)
#define OPIP_PIP_PRT_CFG_MINERR_EN	BIT_ULL(48)
#define OPIP_PIP_PRT_CFG_DYN_RS		BIT_ULL(33)
#define OPIP_PIP_PRT_CFG_INST_HDR	BIT_ULL(32)
#define OPIP_PIP_PRT_CFG_QOS		GENMASK_ULL(26, 24)
#define OPIP_PIP_PRT_CFG_HIGIG_EN	BIT_ULL(11)
#define OPIP_PIP_PRT_CFG_DSA_EN		BIT_ULL(10)
#define OPIP_PIP_PRT_CFG_MODE		GENMASK_ULL(9, 8)
#define OPIP_PIP_PRT_CFG_MODE_SKIP_TO_L2	1
#define OPIP_PIP_PRT_CFG_SKIP		GENMASK_ULL(6, 0)

/* PIP_PRT_TAGn */
#define OPIP_PIP_PRT_TAG_GRPTAGMASK_MSB	GENMASK_ULL(45, 44)
#define OPIP_PIP_PRT_TAG_GRPTAGMASK	GENMASK_ULL(35, 32)
#define OPIP_PIP_PRT_TAG_GRPTAG		BIT_ULL(31)
#define OPIP_PIP_PRT_TAG_INC_PRT_FLAG	BIT_ULL(24)
#define OPIP_PIP_PRT_TAG_IP6_DPRT_FLAG	BIT_ULL(23)
#define OPIP_PIP_PRT_TAG_IP4_DPRT_FLAG	BIT_ULL(22)
#define OPIP_PIP_PRT_TAG_IP6_SPRT_FLAG	BIT_ULL(21)
#define OPIP_PIP_PRT_TAG_IP4_SPRT_FLAG	BIT_ULL(20)
#define OPIP_PIP_PRT_TAG_IP6_DST_FLAG	BIT_ULL(17)
#define OPIP_PIP_PRT_TAG_IP4_DST_FLAG	BIT_ULL(16)
#define OPIP_PIP_PRT_TAG_IP6_SRC_FLAG	BIT_ULL(15)
#define OPIP_PIP_PRT_TAG_IP4_SRC_FLAG	BIT_ULL(14)
#define OPIP_PIP_PRT_TAG_TCP6_TAG_TYPE	GENMASK_ULL(13, 12)
#define OPIP_PIP_PRT_TAG_TCP4_TAG_TYPE	GENMASK_ULL(11, 10)
#define OPIP_PIP_PRT_TAG_IP6_TAG_TYPE	GENMASK_ULL(9, 8)
#define OPIP_PIP_PRT_TAG_IP4_TAG_TYPE	GENMASK_ULL(7, 6)
#define OPIP_PIP_PRT_TAG_NON_TAG_TYPE	GENMASK_ULL(5, 4)
#define OPIP_PIP_PRT_TAG_GRP		GENMASK_ULL(3, 0)

/* ------------------------------------------------------------------ */
/* IPD: packet input data, the buffers and work queue entries */

#define OPIP_IPD_BASE		0x00014F0000000000ull

#define OPIP_IPD_REG(off)		OPIP_CSR(OPIP_IPD_BASE + (off))
#define OPIP_IPD_1ST_MBUFF_SKIP		OPIP_IPD_REG(0x000)
#define OPIP_IPD_NOT_1ST_MBUFF_SKIP	OPIP_IPD_REG(0x008)
#define OPIP_IPD_PACKET_MBUFF_SIZE	OPIP_IPD_REG(0x010)
#define OPIP_IPD_CTL_STATUS		OPIP_IPD_REG(0x018)
#define OPIP_IPD_WQE_FPA_QUEUE		OPIP_IPD_REG(0x020)
#define OPIP_IPD_PORT_BP_PAGE_CNT(p)	OPIP_IPD_REG(0x028 + (p) * 8)
#define OPIP_IPD_SUB_PORT_BP_PAGE_CNT	OPIP_IPD_REG(0x148)
#define OPIP_IPD_1ST_NEXT_PTR_BACK	OPIP_IPD_REG(0x150)
#define OPIP_IPD_2ND_NEXT_PTR_BACK	OPIP_IPD_REG(0x158)
#define OPIP_IPD_SUB_PORT_FCS		OPIP_IPD_REG(0x170)
#define OPIP_IPD_QOS_RED_MARKS(q)	OPIP_IPD_REG(0x178 + (q) * 8)
#define OPIP_IPD_PORT_BP_COUNTERS_PAIR(p) OPIP_IPD_REG(0x1B8 + (p) * 8)
#define OPIP_IPD_RED_PORT_ENABLE	OPIP_IPD_REG(0x2D8)
#define OPIP_IPD_RED_QUE_PARAM(q)	OPIP_IPD_REG(0x2E0 + (q) * 8)
#define OPIP_IPD_PTR_COUNT		OPIP_IPD_REG(0x320)
#define OPIP_IPD_BP_PRT_RED_END		OPIP_IPD_REG(0x328)
#define OPIP_IPD_PWP_PTR_FIFO_CTL	OPIP_IPD_REG(0x340)
#define OPIP_IPD_PRC_HOLD_PTR_FIFO_CTL	OPIP_IPD_REG(0x348)
#define OPIP_IPD_PRC_PORT_PTR_FIFO_CTL	OPIP_IPD_REG(0x350)
#define OPIP_IPD_PKT_PTR_VALID		OPIP_IPD_REG(0x358)
#define OPIP_IPD_WQE_PTR_VALID		OPIP_IPD_REG(0x360)
/* CN68XX: the pointer FIFOs and the prefetched pointers moved */
#define OPIP_IPD_FREE_PTR_FIFO_CTL	OPIP_IPD_REG(0x780)
#define OPIP_IPD_FREE_PTR_VALUE		OPIP_IPD_REG(0x788)
#define OPIP_IPD_HOLD_PTR_FIFO_CTL	OPIP_IPD_REG(0x790)
#define OPIP_IPD_PORT_PTR_FIFO_CTL	OPIP_IPD_REG(0x798)
#define OPIP_IPD_NEXT_PKT_PTR		OPIP_IPD_REG(0x7A0)
#define OPIP_IPD_NEXT_WQE_PTR		OPIP_IPD_REG(0x7A8)

#define OPIP_IPD_MBUFF_SKIP_SZ		GENMASK_ULL(5, 0)
#define OPIP_IPD_PACKET_MBUFF_SIZE_MB_SIZE	GENMASK_ULL(11, 0)
#define OPIP_IPD_NEXT_PTR_BACK_BACK	GENMASK_ULL(3, 0)
#define OPIP_IPD_WQE_FPA_QUEUE_POOL	GENMASK_ULL(2, 0)

/* IPD_CTL_STATUS */
#define OPIP_IPD_CTL_STATUS_NO_WPTR	BIT_ULL(14)
#define OPIP_IPD_CTL_STATUS_LEN_M8	BIT_ULL(9)
#define OPIP_IPD_CTL_STATUS_RESET	BIT_ULL(8)
#define OPIP_IPD_CTL_STATUS_ADDPKT	BIT_ULL(7)
#define OPIP_IPD_CTL_STATUS_NADDBUF	BIT_ULL(6)
#define OPIP_IPD_CTL_STATUS_PKT_LEND	BIT_ULL(5)
#define OPIP_IPD_CTL_STATUS_WQE_LEND	BIT_ULL(4)
#define OPIP_IPD_CTL_STATUS_PBP_EN	BIT_ULL(3)
#define OPIP_IPD_CTL_STATUS_OPC_MODE	GENMASK_ULL(2, 1)
#define OPIP_IPD_CTL_STATUS_IPD_EN	BIT_ULL(0)

/*
 * IPD_PORTn_BP_PAGE_CNT, IPD_SUB_PORT_BP_PAGE_CNT, IPD_PORT_BP_COUNTERS_PAIRn
 */
#define OPIP_IPD_PORT_BP_PAGE_CNT_BP_ENB	BIT_ULL(17)
#define OPIP_IPD_PORT_BP_PAGE_CNT_PAGE_CNT	GENMASK_ULL(16, 0)
#define OPIP_IPD_SUB_PORT_BP_PAGE_CNT_PORT	GENMASK_ULL(30, 25)
#define OPIP_IPD_SUB_PORT_BP_PAGE_CNT_PAGE_CNT	GENMASK_ULL(24, 0)
#define OPIP_IPD_PORT_BP_COUNTERS_CNT_VAL	GENMASK_ULL(24, 0)
/* The executive's threshold before a port's link sets its own (pages) */
#define OPIP_IPD_PORT_BP_PAGES_INIT		100

/* IPD_SUB_PORT_FCS: bit n for port n, ports 36-39 in PORT_BIT2 */
#define OPIP_IPD_SUB_PORT_FCS_PORT_BIT2	GENMASK_ULL(39, 36)
#define OPIP_IPD_SUB_PORT_FCS_PORT_BIT	GENMASK_ULL(31, 0)

/*
 * IPD_QOSn_RED_MARKS, IPD_RED_QUEn_PARAM, IPD_RED_PORT_ENABLE,
 * IPD_BP_PRT_RED_END
 */
#define OPIP_IPD_RED_MARKS_DROP		GENMASK_ULL(63, 32)
#define OPIP_IPD_RED_MARKS_PASS		GENMASK_ULL(31, 0)
#define OPIP_IPD_RED_PARAM_USE_PCNT	BIT_ULL(48)
#define OPIP_IPD_RED_PARAM_NEW_CON	GENMASK_ULL(47, 40)
#define OPIP_IPD_RED_PARAM_AVG_CON	GENMASK_ULL(39, 32)
#define OPIP_IPD_RED_PARAM_PRB_CON	GENMASK_ULL(31, 0)
#define OPIP_IPD_RED_PORT_ENABLE_PRB_DLY	GENMASK_ULL(63, 50)
#define OPIP_IPD_RED_PORT_ENABLE_AVG_DLY	GENMASK_ULL(49, 36)
#define OPIP_IPD_RED_PORT_ENABLE_PRT_ENB	GENMASK_ULL(35, 0)
/* The executive's RED average and probability update delays */
#define OPIP_IPD_RED_PORT_DLY			10000

/* IPD_PTR_COUNT and the pointer FIFOs, read at shutdown */
#define OPIP_IPD_PTR_COUNT_PKTV_CNT	BIT_ULL(18)
#define OPIP_IPD_PTR_COUNT_WQEV_CNT	BIT_ULL(17)
#define OPIP_IPD_PTR_COUNT_PFIF_CNT	GENMASK_ULL(16, 14)
#define OPIP_IPD_PTR_COUNT_PKT_PCNT	GENMASK_ULL(13, 7)
#define OPIP_IPD_PTR_COUNT_WQE_PCNT	GENMASK_ULL(6, 0)
#define OPIP_IPD_PTR_VALID_PTR		GENMASK_ULL(28, 0)
#define OPIP_IPD_PWP_FIFO_MAX_CNTS	GENMASK_ULL(60, 54)
#define OPIP_IPD_PWP_FIFO_WRADDR	GENMASK_ULL(53, 46)
#define OPIP_IPD_PWP_FIFO_PRADDR	GENMASK_ULL(45, 38)
#define OPIP_IPD_PWP_FIFO_PTR		GENMASK_ULL(37, 9)
#define OPIP_IPD_PWP_FIFO_CENA		BIT_ULL(8)
#define OPIP_IPD_PWP_FIFO_RADDR		GENMASK_ULL(7, 0)
#define OPIP_IPD_PRC_HOLD_FIFO_MAX_PKT	GENMASK_ULL(38, 36)
#define OPIP_IPD_PRC_HOLD_FIFO_PRADDR	GENMASK_ULL(35, 33)
#define OPIP_IPD_PRC_HOLD_FIFO_PTR	GENMASK_ULL(32, 4)
#define OPIP_IPD_PRC_HOLD_FIFO_CENA	BIT_ULL(3)
#define OPIP_IPD_PRC_HOLD_FIFO_RADDR	GENMASK_ULL(2, 0)
#define OPIP_IPD_PRC_PORT_FIFO_MAX_PKT	GENMASK_ULL(43, 37)
#define OPIP_IPD_PRC_PORT_FIFO_PTR	GENMASK_ULL(36, 8)
#define OPIP_IPD_PRC_PORT_FIFO_CENA	BIT_ULL(7)
#define OPIP_IPD_PRC_PORT_FIFO_RADDR	GENMASK_ULL(6, 0)
/* CN68XX; the free FIFO's entry is read from IPD_FREE_PTR_VALUE */
#define OPIP_IPD_PTR_VALUE_PTR		GENMASK_ULL(32, 0)
#define OPIP_IPD_FREE_FIFO_MAX_CNTS	GENMASK_ULL(31, 25)
#define OPIP_IPD_FREE_FIFO_WRADDR	GENMASK_ULL(24, 17)
#define OPIP_IPD_FREE_FIFO_PRADDR	GENMASK_ULL(16, 9)
#define OPIP_IPD_FREE_FIFO_CENA		BIT_ULL(8)
#define OPIP_IPD_FREE_FIFO_RADDR	GENMASK_ULL(7, 0)
#define OPIP_IPD_HOLD_FIFO_PTR		GENMASK_ULL(42, 10)
#define OPIP_IPD_HOLD_FIFO_MAX_PKT	GENMASK_ULL(9, 7)
#define OPIP_IPD_HOLD_FIFO_PRADDR	GENMASK_ULL(6, 4)
#define OPIP_IPD_HOLD_FIFO_CENA		BIT_ULL(3)
#define OPIP_IPD_HOLD_FIFO_RADDR	GENMASK_ULL(2, 0)
#define OPIP_IPD_PORT_FIFO_PTR		GENMASK_ULL(47, 15)
#define OPIP_IPD_PORT_FIFO_MAX_PKT	GENMASK_ULL(14, 8)
#define OPIP_IPD_PORT_FIFO_CENA		BIT_ULL(7)
#define OPIP_IPD_PORT_FIFO_RADDR	GENMASK_ULL(6, 0)

/* ------------------------------------------------------------------ */
/* PKO: packet output */

#define OPIP_PKO_BASE		0x0001180050000000ull

#define OPIP_PKO_REG(off)		OPIP_CSR(OPIP_PKO_BASE + (off))
#define OPIP_PKO_REG_FLAGS		OPIP_PKO_REG(0x0000)
#define OPIP_PKO_REG_CMD_BUF		OPIP_PKO_REG(0x0010)
#define OPIP_PKO_REG_GMX_PORT_MODE	OPIP_PKO_REG(0x0018)
#define OPIP_PKO_REG_QUEUE_MODE		OPIP_PKO_REG(0x0048)
#define OPIP_PKO_REG_MIN_PKT		OPIP_PKO_REG(0x0070)
#define OPIP_PKO_REG_QUEUE_PTRS1	OPIP_PKO_REG(0x0100)
#define OPIP_PKO_MEM_QUEUE_PTRS		OPIP_PKO_REG(0x1000)
#define OPIP_PKO_MEM_IPORT_PTRS		OPIP_PKO_REG(0x1030)

/* PKO_REG_FLAGS */
#define OPIP_PKO_REG_FLAGS_RESET	BIT_ULL(3)
#define OPIP_PKO_REG_FLAGS_STORE_BE	BIT_ULL(2)
#define OPIP_PKO_REG_FLAGS_ENA_DWB	BIT_ULL(1)
#define OPIP_PKO_REG_FLAGS_ENA_PKO	BIT_ULL(0)

/*
 * PKO_REG_READ_IDX, PKO_REG_CMD_BUF, PKO_REG_GMX_PORT_MODE, PKO_REG_QUEUE_MODE
 */
#define OPIP_PKO_REG_CMD_BUF_POOL	GENMASK_ULL(22, 20)
#define OPIP_PKO_REG_CMD_BUF_SIZE	GENMASK_ULL(12, 0)
#define OPIP_PKO_REG_GMX_PORT_MODE_MODE1	GENMASK_ULL(5, 3)
#define OPIP_PKO_REG_GMX_PORT_MODE_MODE0	GENMASK_ULL(2, 0)
#define OPIP_PKO_REG_QUEUE_MODE_MODE	GENMASK_ULL(1, 0)

/* PKO_REG_QUEUE_PTRS1, the high bits of PKO_MEM_QUEUE_PTRS on CN5XXX+ */
#define OPIP_PKO_REG_QUEUE_PTRS1_IDX3	BIT_ULL(1)
#define OPIP_PKO_REG_QUEUE_PTRS1_QID7	BIT_ULL(0)

/* PKO_MEM_QUEUE_PTRS */
#define OPIP_PKO_MEM_QUEUE_PTRS_QOS_MASK	GENMASK_ULL(60, 53)
#define OPIP_PKO_MEM_QUEUE_PTRS_BUF_PTR		GENMASK_ULL(52, 17)
#define OPIP_PKO_MEM_QUEUE_PTRS_TAIL		BIT_ULL(16)
#define OPIP_PKO_MEM_QUEUE_PTRS_INDEX		GENMASK_ULL(15, 13)
#define OPIP_PKO_MEM_QUEUE_PTRS_PORT		GENMASK_ULL(12, 7)
#define OPIP_PKO_MEM_QUEUE_PTRS_QUEUE		GENMASK_ULL(6, 0)

/* PKO_MEM_IPORT_PTRS, PKO_MEM_IQUEUE_PTRS (CN68XX) */
#define OPIP_PKO_MEM_IPORT_PTRS_CRC		BIT_ULL(62)
#define OPIP_PKO_MEM_IPORT_PTRS_QOS_MASK	GENMASK_ULL(60, 53)
#define OPIP_PKO_MEM_IPORT_PTRS_MIN_PKT		GENMASK_ULL(52, 50)
#define OPIP_PKO_MEM_IPORT_PTRS_PIPE		GENMASK_ULL(30, 24)
#define OPIP_PKO_MEM_IPORT_PTRS_INTR		GENMASK_ULL(20, 16)
#define OPIP_PKO_MEM_IPORT_PTRS_EID		GENMASK_ULL(12, 8)
#define OPIP_PKO_MEM_IPORT_PTRS_IPID		GENMASK_ULL(6, 0)
/* PKO_REG_MIN_PKT: eight minimum sizes, a byte each */

/* ------------------------------------------------------------------ */
/* POW (SSO on CN68XX): the work scheduler */

#define OPIP_POW_BASE		0x0001670000000000ull

#define OPIP_POW_REG(off)		OPIP_CSR(OPIP_POW_BASE + (off))
#define OPIP_POW_PP_GRP_MSK(core)	OPIP_POW_REG(0x0000 + (core) * 8)
#define OPIP_POW_WQ_INT_THR(grp)	OPIP_POW_REG(0x0080 + (grp) * 8)
#define OPIP_POW_WQ_INT			OPIP_POW_REG(0x0200)
#define OPIP_POW_WQ_INT_PC		OPIP_POW_REG(0x0208)
#define OPIP_SSO_WQ_INT			OPIP_POW_REG(0x1000)
#define OPIP_SSO_WQ_IQ_DIS		OPIP_POW_REG(0x1010)
#define OPIP_SSO_WQ_INT_PC		OPIP_POW_REG(0x1020)
#define OPIP_SSO_PP_GRP_MSK(core)	OPIP_POW_REG(0x6000 + (core) * 8)
#define OPIP_SSO_WQ_INT_THR(grp)	OPIP_POW_REG(0x7000 + (grp) * 8)

#define OPIP_POW_PP_GRP_MSK_GRP_MSK	GENMASK_ULL(15, 0)
#define OPIP_POW_WQ_INT_THR_TC_EN	BIT_ULL(28)
#define OPIP_POW_WQ_INT_THR_TC_THR	GENMASK_ULL(27, 24)
#define OPIP_POW_WQ_INT_IQ_DIS		GENMASK_ULL(31, 16)
#define OPIP_POW_WQ_INT_WQ_INT		GENMASK_ULL(15, 0)
#define OPIP_POW_WQ_INT_PC_PC_THR	GENMASK_ULL(27, 8)
#define OPIP_SSO_WQ_INT_THR_TC_EN	BIT_ULL(32)
#define OPIP_SSO_WQ_INT_THR_TC_THR	GENMASK_ULL(31, 28)

/* The work request I/O address and the IOBDMA command word */
#define OPIP_POW_LOAD_IS_IO		BIT_ULL(48)
#define OPIP_POW_LOAD_DID		GENMASK_ULL(47, 40)
#define OPIP_POW_LOAD_WAIT		BIT_ULL(3)
#define OPIP_POW_DID_SWTAG		0x60	/* CVMX_FULL_DID(TAG=12, 0) */
#define OPIP_POW_RESP_NO_WORK		BIT_ULL(63)
#define OPIP_POW_RESP_ADDR		GENMASK_ULL(39, 0)
#define OPIP_POW_IOBDMA_SCRADDR		GENMASK_ULL(63, 56)
#define OPIP_POW_IOBDMA_LEN		GENMASK_ULL(55, 48)
#define OPIP_POW_IOBDMA_DID		GENMASK_ULL(47, 40)
#define OPIP_POW_IOBDMA_WAIT		BIT_ULL(3)

/* ------------------------------------------------------------------ */
/* FPA: the buffer pools */

#define OPIP_FPA_BASE		0x0001180028000000ull
#define OPIP_FPA_CTL_STATUS		OPIP_CSR(OPIP_FPA_BASE + 0x50)
#define OPIP_FPA_QUE_AVAILABLE(pool)	\
	OPIP_CSR(OPIP_FPA_BASE + 0x98 + (pool) * 8)
#define OPIP_FPA_CTL_STATUS_ENB		BIT_ULL(14)
#define OPIP_FPA_QUE_AVAILABLE_QUE_SIZ	GENMASK_ULL(31, 0)

/* ------------------------------------------------------------------ */
/* Odd ones: the FAU timeout, the L2 arbitration, the debug port, the RML */

#define OPIP_IOB_FAU_TIMEOUT		OPIP_CSR(0x00011800F0000000ull)
#define OPIP_IOB_FAU_TIMEOUT_TOUT_VAL	GENMASK_ULL(11, 0)

#define OPIP_L2C_CFG			OPIP_CSR(0x0001180080000000ull)
#define OPIP_L2C_CFG_RFB_ARB_MODE	BIT_ULL(1)
#define OPIP_L2C_CFG_LRF_ARB_MODE	BIT_ULL(0)

/* ------------------------------------------------------------------ */
/*
 * Register field helper: read, modify and write back a field whose mask
 * is selected at run time (u64_replace_bits() wants a constant one)
 */
static inline u64 opip_field_set_var(u64 reg, u64 mask, u64 val)
{
	return (reg & ~mask) | ((val << __ffs64(mask)) & mask);
}

#endif /* OCTEON_PIP_REGS_H */
