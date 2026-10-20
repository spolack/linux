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

#ifdef CONFIG_CAVIUM_OCTEON_SOC

#include <asm/octeon/octeon.h>
#include <asm/octeon/cvmx-bootinfo.h>
#include <asm/octeon/cvmx-helper.h>

static inline u64 opip_rd(u64 addr)
{
	return cvmx_read_csr(addr);
}

static inline void opip_wr(u64 addr, u64 val)
{
	cvmx_write_csr(addr, val);
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

/* POW tag types */
#define OPIP_TAG_ORDERED	0

#endif /* OCTEON_PIP_HW_H */
