/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The CPU side of the OCTEON packet I/O hardware: register access, the
 * buffer pools, work requests, the FAU counters, the work queue entry and
 * buffer pointer layouts. On OCTEON these are the arch's primitives;
 * under COMPILE_TEST they are the stubs of octeon_pip_hw_stub.h.
 *
 * Derived in part from the work queue entry and buffer pointer layouts of
 * arch/mips/include/asm/octeon/cvmx-wqe.h and cvmx-packet.h,
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_HW_H
#define OCTEON_PIP_HW_H

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/compiler.h>
#include <linux/types.h>

/* The interface modes the hardware can be strapped to */
enum opip_mode {
	OPIP_MODE_DISABLED,
	OPIP_MODE_RGMII,
	OPIP_MODE_GMII,
	OPIP_MODE_SPI,
	OPIP_MODE_PCIE,
	OPIP_MODE_XAUI,
	OPIP_MODE_SGMII,
	OPIP_MODE_PICMG,
	OPIP_MODE_NPI,
	OPIP_MODE_LOOP,
};

/* A link state as the board or a PCS reports it */
struct opip_link {
	bool up;
	bool full_duplex;
	unsigned int speed;	/* Mbit/s */
};

#ifdef CONFIG_CAVIUM_OCTEON_SOC

#include <asm/octeon/octeon.h>
#include <asm/octeon/cvmx-bootinfo.h>
#include <asm/octeon/cvmx-fpa.h>
#include <asm/octeon/cvmx-helper.h>
#include <asm/octeon/cvmx-helper-errata.h>
#include <asm/octeon/cvmx-scratch.h>
#include <asm/octeon/cvmx-sysinfo.h>

static inline u64 opip_rd(u64 addr)
{
	return cvmx_read_csr(addr);
}

static inline void opip_wr(u64 addr, u64 val)
{
	cvmx_write_csr(addr, val);
}

static inline void *opip_fpa_alloc(int pool)
{
	return cvmx_fpa_alloc(pool);
}

/* Free with the write barrier the pool needs, see opip_fpa_free_nosync() */
static inline void opip_fpa_free(void *buf, int pool, unsigned int lines)
{
	cvmx_fpa_free(buf, pool, lines);
}

/*
 * Free without a barrier: for a buffer the CPU has not written, or when
 * one barrier covers a batch of frees. @lines: cache lines of the buffer
 * the hardware need not write back.
 */
static inline void opip_fpa_free_nosync(void *buf, int pool,
					unsigned int lines)
{
	cvmx_fpa_free_nosync(buf, pool, lines);
}

/*
 * A 32 bit FAU register: the PKO's packet counter of a queue, see
 * opip_fau_addr()
 */
static inline u32 opip_fau_rd32(u64 addr)
{
	return cvmx_read64_int32(addr);
}

static inline void opip_fau_wr32(u64 addr, u32 val)
{
	cvmx_write64_int32(addr, val);
}

static inline u64 opip_phys(const void *ptr)
{
	return cvmx_ptr_to_phys((void *)ptr);
}

static inline void *opip_virt(u64 phys)
{
	return cvmx_phys_to_ptr(phys);
}

/*
 * The physical address of a pointer into XKPHYS, where the packet buffers
 * and the skb data are: a mask, without the segment tests of opip_phys()
 */
static inline u64 opip_xkphys_to_phys(const void *ptr)
{
	return XKPHYS_TO_PHYS((uintptr_t)ptr);
}

/* An IOBDMA command: the hardware answers into the core's scratch memory */
static inline void opip_iobdma(u64 cmd)
{
	cvmx_send_single(cmd);
}

/*
 * The CVMSEG scratch memory of the core, where the answers to IOBDMA
 * commands land: the hardware writes a word asynchronously, so both
 * accessors are volatile.
 */
static inline u64 opip_scratch_rd(unsigned int addr)
{
	return cvmx_scratch_read64(addr);
}

static inline void opip_scratch_wr(unsigned int addr, u64 val)
{
	cvmx_scratch_write64(addr, val);
}

#define opip_iobdma_wait()	CVMX_SYNCIOBDMA

static inline unsigned int opip_core(void)
{
	return cvmx_get_core_num();
}

static inline bool opip_on_simulator(void)
{
	return octeon_is_simulation();
}

static inline unsigned int opip_clock_hz(void)
{
	return cvmx_sysinfo_get()->cpu_clock_hz;
}

/* The interfaces as the bootloader strapped them, from the arch */
static inline int opip_num_interfaces(void)
{
	return cvmx_helper_get_number_of_interfaces();
}

/* The enumerations match, the arch's is the hardware's */
static_assert((int)OPIP_MODE_DISABLED ==
	      (int)CVMX_HELPER_INTERFACE_MODE_DISABLED);
static_assert((int)OPIP_MODE_RGMII == (int)CVMX_HELPER_INTERFACE_MODE_RGMII);
static_assert((int)OPIP_MODE_GMII == (int)CVMX_HELPER_INTERFACE_MODE_GMII);
static_assert((int)OPIP_MODE_SPI == (int)CVMX_HELPER_INTERFACE_MODE_SPI);
static_assert((int)OPIP_MODE_PCIE == (int)CVMX_HELPER_INTERFACE_MODE_PCIE);
static_assert((int)OPIP_MODE_XAUI == (int)CVMX_HELPER_INTERFACE_MODE_XAUI);
static_assert((int)OPIP_MODE_SGMII == (int)CVMX_HELPER_INTERFACE_MODE_SGMII);
static_assert((int)OPIP_MODE_PICMG == (int)CVMX_HELPER_INTERFACE_MODE_PICMG);
static_assert((int)OPIP_MODE_NPI == (int)CVMX_HELPER_INTERFACE_MODE_NPI);
static_assert((int)OPIP_MODE_LOOP == (int)CVMX_HELPER_INTERFACE_MODE_LOOP);

static inline enum opip_mode opip_iface_mode(int iface)
{
	return (enum opip_mode)cvmx_helper_interface_get_mode(iface);
}

static inline void opip_errata_qlm_cdr(void)
{
	__cvmx_helper_errata_qlm_disable_2nd_order_cdr(1);
}

#else /* CONFIG_CAVIUM_OCTEON_SOC */

#include "octeon_pip_hw_stub.h"

#endif

/* ------------------------------------------------------------------ */
/*
 * What the driver does differently per chip: one constant description per
 * model family, chosen at probe by the processor ID (opip_chip_find() in
 * octeon_pip_main.c). The errata of single chip passes stay with the
 * one-off setup code they belong to. The complex exists once in a system,
 * at fixed addresses, and so does the pointer to its description.
 */
struct opip_chip {
	const char *name;
	/*
	 * CN68XX: port kinds in place of the input port (the WQE's word 0,
	 * GMX_PRT_CFG[PKND], the PIP's statistics per port kind), the SSO
	 * in place of the POW, the PKO's internal port and queue tables
	 * with one queue per port, the PKO's padding and FCS, no per-port
	 * backpressure count, nine interfaces packed closer together
	 */
	bool cn68xx;
	/*
	 * CN3XXX: no PKO_REG_QUEUE_PTRS1 (eight queues per port at most),
	 * PKO_MEM_DEBUG8/9 where the later chips have 7/8, the driver pads
	 * short frames (errata GMX-401)
	 */
	bool cn3xxx;
	/*
	 * CN30XX, CN31XX, CN50XX: 32 PKO queues, no PKO_REG_GMX_PORT_MODE,
	 * the GMX transmit threshold fixed
	 */
	bool small;
	/*
	 * The GMX checks the frame length per port in GMX_RX_FRM_MAX and the
	 * PIP has no check per interface (CN3XXX, CN58XX)
	 */
	bool gmx_frm_max;
	/* An interface may run QSGMII (CN70XX) */
	bool qsgmii;
	/* GMX_INF_MODE[TYPE] marks an SPI-4.2 interface (CN38XX, CN58XX) */
	bool spi;
	/* The driver sets the L2C arbitration in L2C_CFG (CN3XXX, CN5XXX) */
	bool l2c_cfg;
	/* RGMII: the MII ports of CN30XX and CN50XX run with TX_CLK 1 */
	bool rgmii_mii_clk;
	/* RGMII: the clock delay of the pads */
	u8 rgmii_delay;
	/*
	 * RGMII: the ports of an RGMII interface (CN30XX, CN31XX, CN50XX: 3,
	 * two in GMII mode; CN38XX, CN58XX: 4), 0 for a chip without RGMII
	 */
	u8 rgmii_ports;
	/* The NPI interface's ports, 0 without packet engines */
	u8 npi_ports;
	/* PKO output queues the chip has */
	unsigned int pko_queues;
	/* The queues the layout of opip_pko_layout() stays within */
	unsigned int pko_layout_queues;
	/*
	 * The queues PKO_REG_QUEUE_MODE 2 gives the whole command storage
	 * to, twice as many in mode 1; 0 for a chip without the register
	 */
	unsigned int pko_mode_queues;
	/* The spacing of the GMX/ASX/PCS and of the PCSX register sets */
	u64 iface_stride;
	u64 pcsx_stride;
};

extern const struct opip_chip *opip_chip;

/* ------------------------------------------------------------------ */
/* The work queue entry: what the hardware delivers for a packet */

#define OPIP_WQE_DATA_BYTES	96

struct opip_wqe {
	u64 word0;
	u64 word1;
	u64 word2;
	u64 packet_ptr;
	/*
	 * The packet's first bytes, or the driver's own data of a sent packet
	 */
	u8 packet_data[OPIP_WQE_DATA_BYTES];
};

/* word0 on the chips with port kinds (CN68XX) */
#define OPIP_WQE0_PKND		GENMASK_ULL(5, 0)

/* word1: length, tag, and on the older chips the input port */
#define OPIP_WQE1_LEN		GENMASK_ULL(63, 48)
#define OPIP_WQE1_IPPRT		GENMASK_ULL(47, 42)
#define OPIP_WQE1_TAG		GENMASK_ULL(31, 0)

/* word2: how the packet was parsed and stored */
#define OPIP_WQE2_BUFS		GENMASK_ULL(63, 56)
#define OPIP_WQE2_TCP_OR_UDP	BIT_ULL(18)
#define OPIP_WQE2_IS_V6		BIT_ULL(16)
#define OPIP_WQE2_L4_ERROR	BIT_ULL(14)
#define OPIP_WQE2_IS_FRAG	BIT_ULL(13)
#define OPIP_WQE2_IP_EXC	BIT_ULL(12)
#define OPIP_WQE2_NOT_IP	BIT_ULL(9)
#define OPIP_WQE2_RCV_ERROR	BIT_ULL(8)
#define OPIP_WQE2_ERR_CODE	GENMASK_ULL(7, 0)

/*
 * The receive error codes the driver acts on. The PIP's counters have the
 * codes from OVER_FCS to UNDER (3-8); the others are counted in software.
 */
#define OPIP_RCV_ERR_JABBER	2
#define OPIP_RCV_ERR_OVER_FCS	3
#define OPIP_RCV_ERR_ALIGN	5
#define OPIP_RCV_ERR_FCS	7
#define OPIP_RCV_ERR_UNDER	8
#define OPIP_RCV_ERR_LENGTH	10

/* The buffer pointer: a packet buffer, or a gather list */
#define OPIP_BUF_I		BIT_ULL(63)
#define OPIP_BUF_BACK		GENMASK_ULL(62, 59)
#define OPIP_BUF_POOL		GENMASK_ULL(58, 56)
#define OPIP_BUF_SIZE		GENMASK_ULL(55, 40)
#define OPIP_BUF_ADDR		GENMASK_ULL(39, 0)

/* POW tag types */
#define OPIP_TAG_ORDERED	0

static inline unsigned int opip_wqe_len(const struct opip_wqe *wqe)
{
	return FIELD_GET(OPIP_WQE1_LEN, wqe->word1);
}

static inline u32 opip_wqe_tag(const struct opip_wqe *wqe)
{
	return FIELD_GET(OPIP_WQE1_TAG, wqe->word1);
}

static inline unsigned int opip_wqe_bufs(const struct opip_wqe *wqe)
{
	return FIELD_GET(OPIP_WQE2_BUFS, wqe->word2);
}

static inline unsigned int opip_wqe_err_code(const struct opip_wqe *wqe)
{
	return FIELD_GET(OPIP_WQE2_ERR_CODE, wqe->word2);
}

static inline u64 opip_buf_addr(u64 buf)
{
	return FIELD_GET(OPIP_BUF_ADDR, buf);
}

/* The start of the buffer a pointer points into */
static inline u64 opip_buf_start(u64 buf)
{
	return ((opip_buf_addr(buf) >> 7) - FIELD_GET(OPIP_BUF_BACK, buf)) << 7;
}

static inline u64 opip_buf_build(u64 addr, unsigned int size, int pool,
				 unsigned int back, bool i)
{
	return FIELD_PREP(OPIP_BUF_ADDR, addr) |
	       FIELD_PREP(OPIP_BUF_SIZE, size) |
	       FIELD_PREP(OPIP_BUF_POOL, pool) |
	       FIELD_PREP(OPIP_BUF_BACK, back) |
	       (i ? OPIP_BUF_I : 0);
}

/*
 * The link pointer to a packet's next buffer sits in the eight bytes
 * before a buffer's data; a buffer holds that much less data
 */
#define OPIP_BUF_LINK_SIZE	8

/* The next segment's pointer, before a segment */
static inline u64 opip_buf_next(u64 buf)
{
	return *(u64 *)opip_virt(opip_buf_addr(buf) - OPIP_BUF_LINK_SIZE);
}

/* The first word of a PKO command */
#define OPIP_PKO_CMD_SIZE0	GENMASK_ULL(61, 60)
#define OPIP_PKO_CMD_SUBONE0	BIT_ULL(47)
#define OPIP_PKO_CMD_REG0	GENMASK_ULL(46, 36)
#define OPIP_PKO_CMD_LE		BIT_ULL(35)
#define OPIP_PKO_CMD_N2		BIT_ULL(34)
#define OPIP_PKO_CMD_GATHER	BIT_ULL(31)
#define OPIP_PKO_CMD_IPOFFP1	GENMASK_ULL(30, 24)
#define OPIP_PKO_CMD_DONTFREE	BIT_ULL(22)
#define OPIP_PKO_CMD_SEGS	GENMASK_ULL(21, 16)
#define OPIP_PKO_CMD_TOTAL_BYTES	GENMASK_ULL(15, 0)

#endif /* OCTEON_PIP_HW_H */
