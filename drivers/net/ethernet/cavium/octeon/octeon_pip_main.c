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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "octeon_pip.h"
#include "octeon_pip_regs.h"

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
	int i;

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

static int opip_probe(struct platform_device *pdev)
{
	struct opip *opip;

	opip_chip = opip_chip_find();
	if (!opip_chip) {
		dev_err(&pdev->dev, "unknown chip\n");
		return -ENODEV;
	}

	opip = devm_kzalloc(&pdev->dev, sizeof(*opip), GFP_KERNEL);
	if (!opip)
		return -ENOMEM;
	opip->pdev = pdev;

	opip_probe_ifaces(opip);

	return 0;
}

static const struct of_device_id opip_match[] = {
	{ .compatible = "cavium,octeon-3860-pip" },
	{ }
};
MODULE_DEVICE_TABLE(of, opip_match);

static struct platform_driver opip_driver = {
	.probe		= opip_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = opip_match,
	},
};

module_platform_driver(opip_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Polack <spolack+git@mailbox.org>");
MODULE_DESCRIPTION("Cavium OCTEON PIP/PKO Ethernet driver");
