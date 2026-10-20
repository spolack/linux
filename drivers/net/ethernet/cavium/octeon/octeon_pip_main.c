// SPDX-License-Identifier: GPL-2.0
/*
 * octeon_pip: the Ethernet driver of the OCTEON packet I/O blocks.
 *
 * The probe brings the blocks up in the order they depend on each other:
 * the pools, the interfaces and their ports, the output queues, the POW
 * groups that deliver the packets, then packet input, and the network
 * devices last.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper.c, cvmx-helper-errata.c) and the staging octeon
 * driver (ethernet.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_ipd.h"
#include "octeon_pip_pko.h"
#include "octeon_pip_pow.h"
#include "octeon_pip_regs.h"

/* PKO command chunks put in the chunk pool */
#define OPIP_NUM_CMD_CHUNKS	1024

static const char * const opip_mode_names[] = {
	"DISABLED", "RGMII", "GMII", "SPI", "PCIE", "XAUI", "SGMII", "PICMG",
	"NPI", "LOOP",
};

/*
 * The ports of an interface, counted as the executive's enumeration does
 * but without its board quirks: the platform code removes the device tree
 * nodes of the ports a board lacks, see opip_alloc_ports().
 */
static int opip_iface_ports(int iface, enum opip_mode mode)
{
	switch (mode) {
	case OPIP_MODE_RGMII:
		return opip_chip->rgmii_ports;
	case OPIP_MODE_GMII:
		/* GMII/MII mode of CN30XX, CN31XX and CN50XX */
		return 2;
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		if (!opip_chip->qsgmii)
			return 4;
		/* CN70XX: one SGMII port, or four QSGMII ports */
		switch (FIELD_GET(OPIP_GMX_INF_MODE_MODE,
				  opip_rd(OPIP_GMX_INF_MODE(iface)))) {
		case OPIP_GMX_INF_MODE_MODE_SGMII:
			return 1;
		case OPIP_GMX_INF_MODE_MODE_QSGMII:
			return 4;
		default:
			return 0;
		}
	case OPIP_MODE_XAUI:
		return 1;
	case OPIP_MODE_NPI:
		/* The packet engines came with pass 2 of CN52XX and CN56XX */
		if (OCTEON_IS_MODEL(OCTEON_CN52XX_PASS1_X) ||
		    OCTEON_IS_MODEL(OCTEON_CN56XX_PASS1_X))
			return 0;
		return opip_chip->npi_ports;
	case OPIP_MODE_LOOP:
		return 4;
	default:
		return 0;
	}
}

/*
 * Enumerate the interfaces as the bootloader strapped them, and do what
 * each mode needs before input runs: the interface enable of the GMX
 * modes (errata GMX-700), the length checks and FCS handling of the
 * loopback ports. The SPI-4.2 interfaces of CN38XX and CN58XX are not
 * driven: their bring-up and training could not be tested on any board.
 */
static void opip_probe_ifaces(struct opip *opip)
{
	int i, index;

	opip->num_ifaces = min(opip_num_interfaces(), OPIP_MAX_IFACES);
	dev_dbg(&opip->pdev->dev, "%s, %d interfaces\n", opip_chip->name,
		opip->num_ifaces);
	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];

		iface->num = i;
		iface->first_port = opip_port_number(i, 0);
		iface->mode = opip_iface_mode(i);
		/* CN68XX's interfaces 6 to 8 have no port numbers here */
		if (iface->first_port < 0) {
			iface->num_ports = 0;
			continue;
		}
		iface->num_ports = opip_iface_ports(i, iface->mode);

		switch (iface->mode) {
		case OPIP_MODE_SPI:
			dev_dbg(&opip->pdev->dev,
				"interface %d is SPI-4.2, not supported\n", i);
			iface->num_ports = 0;
			break;
		case OPIP_MODE_LOOP:
			for (index = 0; index < 4; index++) {
				int port = opip_port_number(i, index);

				opip_pip_length_checks(port, false);
				opip_ipd_fcs_strip(port, false);
			}
			break;
		default:
			break;
		}
		if (iface->num_ports)
			dev_dbg(&opip->pdev->dev, "interface %d has %d ports (%s)\n",
				i, iface->num_ports,
				opip_mode_names[iface->mode]);
	}
}

/* ------------------------------------------------------------------ */
/* Probe and remove */

/*
 * A kernel that crashed (kdump) or a driver without a shutdown may have
 * left packet I/O running, and its buffers in the pools and the groups.
 * Stop it and take them out without freeing them: they are not this
 * kernel's. After the bootloader there is nothing to take. The IPD's
 * FIFOs are not walked: the reset that follows drops what the IPD holds,
 * and its port FIFO, walked whole, holds what any earlier user left.
 */
static void opip_hw_takeover(struct opip *opip)
{
	unsigned int core, n = 0;
	int pool, group;
	u64 old_mask;

	opip_ipd_disable();
	if (opip_rd(OPIP_PKO_REG_FLAGS) & OPIP_PKO_REG_FLAGS_ENA_PKO) {
		/* It finishes the packets it is sending, see opip_pko_stop() */
		opip_pko_disable();
		msleep(20);
	}
	opip_pko_reset();
	opip_fpa_enable();
	for (pool = OPIP_POOL_PACKET; pool <= OPIP_POOL_CMD; pool++)
		while (opip_fpa_alloc(pool))
			n++;

	/* The group mask is the core's own */
	preempt_disable();
	core = opip_core();
	for (group = 0; group < OPIP_MAX_GROUPS; group++) {
		old_mask = opip_pow_select_group(core, group);
		while (opip_pow_request_sync(false))
			n++;
		opip_pow_restore_groups(core, old_mask);
	}
	preempt_enable();

	if (n)
		dev_warn(&opip->pdev->dev, "dropped %u buffers and work entries of an earlier owner\n",
			 n);
}

const struct opip_chip *opip_chip __read_mostly;

/*
 * The chips, one entry per model family the arch tells apart. A field
 * left out is false or 0: no QSGMII, no SPI, no L2C_CFG, every queue in
 * the layout, no PKO_REG_QUEUE_MODE.
 */
static const struct opip_chip opip_chip_cn30xx = {
	.name = "CN30XX",
	.cn3xxx = true,
	.small = true,
	.gmx_frm_max = true,
	.l2c_cfg = true,
	.rgmii_mii_clk = true,
	.rgmii_delay = 24,
	.rgmii_ports = 3,
	.pko_queues = 32,
	.pko_layout_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn31xx = {
	.name = "CN31XX",
	.cn3xxx = true,
	.small = true,
	.gmx_frm_max = true,
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.rgmii_ports = 3,
	.pko_queues = 32,
	.pko_layout_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn38xx = {
	.name = "CN38XX",
	.cn3xxx = true,
	.gmx_frm_max = true,
	.spi = true,
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.rgmii_ports = 4,
	.npi_ports = 4,
	.pko_queues = 128,
	.pko_layout_queues = 128,
	.pko_mode_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn50xx = {
	.name = "CN50XX",
	.small = true,
	.l2c_cfg = true,
	.rgmii_mii_clk = true,
	.rgmii_delay = 16,
	.rgmii_ports = 3,
	.pko_queues = 32,
	.pko_layout_queues = 32,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn52xx = {
	.name = "CN52XX",
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.npi_ports = 4,
	.pko_queues = 128,
	.pko_layout_queues = 128,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn56xx = {
	.name = "CN56XX",
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.npi_ports = 4,
	.pko_queues = 256,
	.pko_layout_queues = 256,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn58xx = {
	.name = "CN58XX",
	.gmx_frm_max = true,
	.spi = true,
	.l2c_cfg = true,
	.rgmii_delay = 24,
	.rgmii_ports = 4,
	.npi_ports = 4,
	.pko_queues = 256,
	.pko_layout_queues = 256,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

/*
 * OCTEON II and CN70XX: the layout stays within 64 queues, where
 * PKO_REG_QUEUE_MODE 2 gives each queue 32 words of command storage
 */
static const struct opip_chip opip_chip_cn61xx = {
	.name = "CN61XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn63xx = {
	.name = "CN63XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn66xx = {
	.name = "CN66XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x8000000ull,
};

static const struct opip_chip opip_chip_cn68xx = {
	.name = "CN68XX",
	.cn68xx = true,
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x1000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cnf71xx = {
	.name = "CNF71XX",
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

static const struct opip_chip opip_chip_cn70xx = {
	.name = "CN70XX",
	.qsgmii = true,
	.rgmii_delay = 24,
	.pko_queues = 128,
	.pko_layout_queues = 64,
	.pko_mode_queues = 64,
	.iface_stride = 0x8000000ull,
	.pcsx_stride = 0x1000000ull,
};

/* The arch's model constants; each covers the variants of its family */
static const struct {
	u32 model;
	const struct opip_chip *chip;
} opip_chips[] = {
	{ OCTEON_CN30XX, &opip_chip_cn30xx },
	{ OCTEON_CN31XX, &opip_chip_cn31xx },
	{ OCTEON_CN38XX, &opip_chip_cn38xx },
	{ OCTEON_CN50XX, &opip_chip_cn50xx },
	{ OCTEON_CN52XX, &opip_chip_cn52xx },
	{ OCTEON_CN56XX, &opip_chip_cn56xx },
	{ OCTEON_CN58XX, &opip_chip_cn58xx },
	{ OCTEON_CN61XX, &opip_chip_cn61xx },
	{ OCTEON_CN63XX, &opip_chip_cn63xx },
	{ OCTEON_CN66XX, &opip_chip_cn66xx },
	{ OCTEON_CN68XX, &opip_chip_cn68xx },
	{ OCTEON_CNF71XX, &opip_chip_cnf71xx },
	{ OCTEON_CN70XX, &opip_chip_cn70xx },
};

/* The chip the driver runs on, from the processor ID */
static const struct opip_chip *opip_chip_find(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(opip_chips); i++)
		if (OCTEON_IS_MODEL(opip_chips[i].model))
			return opip_chips[i].chip;
	return NULL;
}

/*
 * The packet I/O hardware the ports share, up to the receive groups.
 * Return: 0, or -ENOMEM with the hardware idle and the pools empty
 */
static int opip_hw_init(struct opip *opip)
{
	int i, order, ports = 0;

	/* CN52XX pass 1: a bug in the second order CDR, disable it */
	if (OCTEON_IS_MODEL(OCTEON_CN52XX_PASS1_0))
		opip_errata_qlm_cdr();

	/*
	 * The I/O blocks get priority over the cores at the L2: no starving.
	 * L2C_CFG is CN3XXX/CN5XXX only: OCTEON II/III arbitrate in L2C_CTL,
	 * which the bootloader sets.
	 */
	if (opip_chip->l2c_cfg)
		opip_wr(OPIP_L2C_CFG, opip_rd(OPIP_L2C_CFG) &
			~(OPIP_L2C_CFG_LRF_ARB_MODE |
			  OPIP_L2C_CFG_RFB_ARB_MODE));

	/*
	 * Whatever the bootloader left: the IPD's count of the packet pool
	 * starts at zero while the pool is empty, see opip_ipd_reset().
	 */
	opip_hw_takeover(opip);
	opip_ipd_reset();
	opip_fpa_enable();
	opip_probe_ifaces(opip);

	/* The packet pool by the MAC ports, see OPIP_RX_RING_PER_PORT */
	for (i = 0; i < opip->num_ifaces; i++)
		if (opip_iface_is_gmx(&opip->ifaces[i]))
			ports += opip->ifaces[i].num_ports;
	opip->num_packet_buffers = clamp(ports * OPIP_RX_RING_PER_PORT,
					 OPIP_DEFAULT_PACKET_BUFFERS,
					 OPIP_MAX_PACKET_BUFFERS);

	/* Every received packet takes an entry; the ports add their share */
	opip->wqe_bufs = opip_fpa_fill(opip, OPIP_POOL_WQE, OPIP_POOL_WQE_SIZE,
				       opip->num_packet_buffers);
	opip->cmd_bufs = opip_fpa_fill(opip, OPIP_POOL_CMD, OPIP_POOL_CMD_SIZE,
				       OPIP_NUM_CMD_CHUNKS);
	if (opip->wqe_bufs < opip->num_packet_buffers ||
	    opip->cmd_bufs < OPIP_NUM_CMD_CHUNKS) {
		opip_fpa_empty(opip, OPIP_POOL_WQE, opip->wqe_bufs);
		opip_fpa_empty(opip, OPIP_POOL_CMD, opip->cmd_bufs);
		return -ENOMEM;
	}

	/* One output queue per CPU and port, as far as the chip has them */
	opip_pko_layout(opip, min_t(int, num_possible_cpus(), OPIP_MAX_TXQS));
	opip_pko_init(opip);

	/*
	 * One receive group per CPU, as far as a power of two goes: the PIP
	 * spreads the flows over them by a mask of the tag.
	 */
	order = min(ilog2(num_online_cpus()), 3);
	opip->groups_mask = GENMASK((1 << order) - 1, 0);

	opip_ipd_config();
	opip_ipd_red_setup(opip->num_packet_buffers / 4,
			   opip->num_packet_buffers / 8);
	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];
		int index;

		for (index = 0; index < iface->num_ports; index++) {
			int port = opip_port_number(i, index);

			opip_pip_port_config(i, port, order);
			/*
			 * The bootloader leaves the backpressure counts
			 * nonzero. Zero them now: with the packet pool empty
			 * the IPD counts nothing and no poll subtracts.
			 */
			if ((iface->mode == OPIP_MODE_SGMII ||
			     iface->mode == OPIP_MODE_PICMG) &&
			    !opip_chip->cn68xx)
				opip_ipd_bp_sub(port, opip_ipd_bp_count(port));
		}
	}
	return 0;
}

/* Undo opip_hw_init() */
static void opip_free_hw(struct opip *opip)
{
	opip_pko_reset();
	/* The IPD's count of the packet pool is zero again with the pool */
	opip_ipd_reset();
	opip_fpa_empty(opip, OPIP_POOL_WQE, opip->wqe_bufs);
	opip_fpa_empty(opip, OPIP_POOL_CMD, opip->cmd_bufs);
}

/*
 * Disable the PKO. It stops taking new commands but finishes the packets
 * it is sending: a 9 KB frame takes 8 ms at 10 Mbit/s. Give it the time,
 * then reset it: a port held by PAUSE frames would keep it reading the
 * buffers its queues still hold, which are taken away next.
 */
static void opip_pko_stop(void)
{
	opip_pko_disable();
	msleep(20);
	opip_pko_reset();
}

/* Take the driver down in the order its parts depend on each other */
static void opip_teardown(struct opip *opip)
{
	opip_pko_stop();
	opip_free_hw(opip);
}

static void opip_remove(struct platform_device *pdev)
{
	struct opip *opip = platform_get_drvdata(pdev);

	if (opip->released)
		return;
	opip->released = true;
	opip_teardown(opip);
}

static int opip_probe(struct platform_device *pdev)
{
	struct opip *opip;
	int ret;

	opip_chip = opip_chip_find();
	if (!opip_chip) {
		dev_err(&pdev->dev, "unknown chip\n");
		return -ENODEV;
	}
	/*
	 * Engineering samples: their IPD cannot give back the buffers it
	 * holds, so the driver could never take its pools down again
	 */
	if (OCTEON_IS_MODEL(OCTEON_CN38XX_PASS1) ||
	    OCTEON_IS_MODEL(OCTEON_CN38XX_PASS2)) {
		dev_err(&pdev->dev, "CN38XX pass 1 and 2 are not supported\n");
		return -ENODEV;
	}

	opip = devm_kzalloc(&pdev->dev, sizeof(*opip), GFP_KERNEL);
	if (!opip)
		return -ENOMEM;
	opip->pdev = pdev;
	platform_set_drvdata(pdev, opip);

	ret = opip_hw_init(opip);
	if (ret)
		return ret;

	return 0;
}

/*
 * Leave the packet hardware idle and its pools empty for whatever runs
 * next, a kernel started by kexec or the bootloader: packet input would
 * go on writing to buffers that belong to it, and the pools would hand it
 * buffers of this kernel.
 */
static void opip_shutdown(struct platform_device *pdev)
{
	opip_remove(pdev);
}

static const struct of_device_id opip_match[] = {
	{ .compatible = "cavium,octeon-3860-pip" },
	{ }
};
MODULE_DEVICE_TABLE(of, opip_match);

static struct platform_driver opip_driver = {
	.probe		= opip_probe,
	.remove		= opip_remove,
	.shutdown	= opip_shutdown,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = opip_match,
	},
};

module_platform_driver(opip_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Polack <spolack+git@mailbox.org>");
MODULE_DESCRIPTION("Cavium OCTEON PIP/PKO Ethernet driver");
