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
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_ipd.h"
#include "octeon_pip_pcs.h"
#include "octeon_pip_pko.h"
#include "octeon_pip_pow.h"
#include "octeon_pip_regs.h"
#include "octeon_pip_rx.h"
#include "octeon_pip_tx.h"

/* PKO command chunks put in the chunk pool */
#define OPIP_NUM_CMD_CHUNKS	1024

/* ------------------------------------------------------------------ */
/* The link of a port, from the blocks that make it */

/**
 * opip_link_set - set up a port for a new link state
 * @p:    Port
 * @link: The new link state
 *
 * Return: zero on success
 */
int opip_link_set(struct opip_port *p, const struct opip_link *link)
{
	int iface = p->iface->num;
	int ret = 0;

	mutex_lock(&p->opip->hw_lock);
	switch (p->iface->mode) {
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		ret = opip_pcs_link_set(iface, p->index, link);
		break;
	default:
		break;
	}
	mutex_unlock(&p->opip->hw_lock);
	return ret;
}

/**
 * opip_pcs_setup - set the PCS of an SGMII, QSGMII or 1000BASE-X port up
 * @p:      Port
 * @inband: In-band autonegotiation, or the PCS forced out of band
 *
 * Set the PCS of an SGMII, QSGMII or 1000BASE-X port up for in-band or
 * out-of-band operation, under the hardware lock. Ports of other modes
 * are left alone.
 *
 * Return: 0, or the error of opip_pcs_config()
 */
int opip_pcs_setup(struct opip_port *p, bool inband)
{
	int ret = 0;

	mutex_lock(&p->opip->hw_lock);
	if (opip_port_has_pcs(p))
		ret = opip_pcs_config(p->iface->num, p->index, inband);
	mutex_unlock(&p->opip->hw_lock);
	return ret;
}

/**
 * opip_pcs_restart_an - restart the PCS autonegotiation of a PCS port
 * @p: Port
 *
 * Under the hardware lock. Ports of other modes are left alone.
 */
void opip_pcs_restart_an(struct opip_port *p)
{
	mutex_lock(&p->opip->hw_lock);
	if (opip_port_has_pcs(p))
		opip_pcs_an_restart(p->iface->num, p->index);
	mutex_unlock(&p->opip->hw_lock);
}

/**
 * opip_port_enable - enable or disable the MAC of a port that has one
 * @p:  Port
 * @en: Enable
 *
 * Only ports of a GMX interface have one; the port number is the port
 * kind on the chips that have them.
 */
void opip_port_enable(struct opip_port *p, bool en)
{
	if (!opip_iface_is_gmx(p->iface))
		return;
	mutex_lock(&p->opip->hw_lock);
	opip_gmx_port_enable(p->iface->num, p->index, en, p->port);
	mutex_unlock(&p->opip->hw_lock);
}

static void opip_refill_worker(struct work_struct *work)
{
	struct opip *opip = container_of(work, struct opip, refill_work.work);

	opip_rx_refill_check(opip);
	if (!READ_ONCE(opip->stopping))
		schedule_delayed_work(&opip->refill_work, HZ);
}

/* ------------------------------------------------------------------ */
/* The network device operations */

/*
 * The receive frame limits of a port for an MTU. The jabber limit is per
 * port. The PIP's length check of the later chips has two checkers: CN68XX
 * picks one per port kind, the interface's (opip_pip_port_config()); on
 * the other chips the vendor's code takes checker 0 to apply to all
 * incoming traffic and leaves checker 1 unused. Both get the largest MTU
 * of the ports they may check: the interface's on CN68XX, all GMX ports
 * otherwise.
 */
static void opip_set_hw_mtu(struct net_device *dev, int new_mtu)
{
	unsigned int max_frame = new_mtu + OPIP_MTU_OVERHEAD;
	struct opip_port *p = opip_port(dev);

	if (!opip_port_has_gmx(p))
		return;

	opip_gmx_set_frame_max(p->iface->num, p->index, max_frame);

	if (!opip_chip->gmx_frm_max) {
		unsigned int max_len = max_frame;
		int port;

		for (port = 0; port < OPIP_MAX_PORTS; port++) {
			struct net_device *other = p->opip->ports[port];

			if (!other || other == dev ||
			    !opip_port_has_gmx(opip_port(other)) ||
			    (opip_chip->cn68xx &&
			     opip_port(other)->iface != p->iface))
				continue;
			max_len = max(max_len, READ_ONCE(other->mtu) +
				      OPIP_MTU_OVERHEAD);
		}
		if (opip_chip->cn68xx) {
			opip_pip_frame_length(p->iface->num, VLAN_ETH_ZLEN,
					      max_len);
		} else {
			opip_pip_frame_length(0, VLAN_ETH_ZLEN, max_len);
			opip_pip_frame_length(1, VLAN_ETH_ZLEN, max_len);
		}
	}
}

static int opip_change_mtu(struct net_device *dev, int new_mtu)
{
	WRITE_ONCE(dev->mtu, new_mtu);
	opip_set_hw_mtu(dev, new_mtu);
	return 0;
}

static void opip_rx_filter_flags(struct net_device *dev, bool *promisc,
				 bool *allmulti)
{
	netif_addr_lock_bh(dev);
	*promisc = dev->flags & IFF_PROMISC;
	*allmulti = (dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev);
	netif_addr_unlock_bh(dev);
}

static void opip_rx_mode_worker(struct work_struct *work)
{
	struct opip_port *p = container_of(work, struct opip_port,
					   rx_mode_work);
	bool promisc, allmulti;

	mutex_lock(&p->opip->hw_lock);
	opip_rx_filter_flags(p->netdev, &promisc, &allmulti);
	opip_gmx_set_rx_filter(p->iface->num, p->index, promisc, allmulti);
	mutex_unlock(&p->opip->hw_lock);
}

/* Called in atomic context: a work item programs the registers */
static void opip_set_rx_mode(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);

	if (opip_port_has_gmx(p))
		schedule_work(&p->rx_mode_work);
}

static void opip_set_mac_filter(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	bool promisc, allmulti;

	if (!opip_port_has_gmx(p))
		return;
	/*
	 * The flags are read under hw_lock, here and in the worker: an
	 * address list change between reading and writing them is then
	 * written by whichever of the two comes second
	 */
	mutex_lock(&p->opip->hw_lock);
	opip_rx_filter_flags(dev, &promisc, &allmulti);
	opip_gmx_set_mac(p->iface->num, p->index, dev->dev_addr);
	opip_gmx_set_rx_filter(p->iface->num, p->index, promisc, allmulti);
	mutex_unlock(&p->opip->hw_lock);
}

static int opip_set_mac_address(struct net_device *dev, void *addr)
{
	int ret = eth_mac_addr(dev, addr);

	if (ret)
		return ret;
	opip_set_mac_filter(dev);
	return 0;
}

/*
 * The port's hardware is set up for the device; it stays disabled until opened
 */
static int opip_ndo_init(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned int i;

	/*
	 * Receive queue i is group i, whose NAPI is on opip->napi_dev and
	 * outlives the registration.
	 */
	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		if (p->opip->groups_mask & BIT(i))
			netif_queue_set_napi(dev, i, NETDEV_QUEUE_TYPE_RX,
					     &p->opip->groups[i].napi);

	opip_set_mac_filter(dev);
	opip_set_hw_mtu(dev, dev->mtu);

	opip_port_enable(p, false);
	return 0;
}

static int opip_ndo_open(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	int ret;

	/* NPI and LOOP: no MAC or link to bring up */
	if (!opip_iface_is_gmx(p->iface))
		return 0;

	/* The simulator has no link: the port keeps the carrier on */
	if (!p->phylink) {
		opip_port_enable(p, true);
		return 0;
	}

	ret = phylink_of_phy_connect(p->phylink, p->of_node, 0);
	if (ret) {
		netdev_err(dev, "could not attach the PHY: %d\n", ret);
		return ret;
	}
	opip_port_enable(p, true);
	phylink_start(p->phylink);
	return 0;
}

static int opip_ndo_stop(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);

	if (!opip_iface_is_gmx(p->iface))
		return 0;

	/*
	 * With the MAC still on: the link goes down through mac_link_down(),
	 * which also stops the PAUSE frames that hold the partner off
	 */
	if (p->phylink) {
		phylink_stop(p->phylink);
		phylink_disconnect_phy(p->phylink);
	}
	opip_port_enable(p, false);
	return 0;
}

static int opip_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct opip_port *p = opip_port(dev);

	if (!netif_running(dev) || !p->phylink)
		return -ENODEV;
	return phylink_mii_ioctl(p->phylink, rq, cmd);
}

static const struct net_device_ops opip_netdev_ops = {
	.ndo_init		= opip_ndo_init,
	.ndo_open		= opip_ndo_open,
	.ndo_stop		= opip_ndo_stop,
	.ndo_start_xmit		= opip_xmit,
	.ndo_set_rx_mode	= opip_set_rx_mode,
	.ndo_set_mac_address	= opip_set_mac_address,
	.ndo_eth_ioctl		= opip_ioctl,
	.ndo_change_mtu		= opip_change_mtu,
};

/* ------------------------------------------------------------------ */
/* The interfaces and their ports */

static struct device_node *opip_of_child(const struct device_node *parent,
					 int reg)
{
	struct device_node *node;

	for_each_child_of_node(parent, node) {
		u32 val;

		if (!of_property_read_u32(node, "reg", &val) && val == reg)
			return node;
	}
	return NULL;
}

/* The device tree node of a port: pip -> interface -> port */
static struct device_node *opip_port_of_node(struct device_node *pip, int iface,
					     int index)
{
	struct device_node *ni = opip_of_child(pip, iface), *np;

	if (!ni)
		return NULL;
	np = opip_of_child(ni, index);
	of_node_put(ni);
	return np;
}

/*
 * The SGMII interface mode covers QSGMII and 1000BASE-X as well. The PHY
 * driver needs to know which: it configures the MAC side of the PHY
 * accordingly. The hardware knows (the bootloader set it up), the device
 * tree is only checked against it.
 */
static void opip_sgmii_phy_mode(struct opip_port *p)
{
	phy_interface_t dt_mode;

	if (opip_pcs_basex(p->iface->num, p->index))
		p->phy_mode = PHY_INTERFACE_MODE_1000BASEX;
	else if (p->iface->qsgmii)
		p->phy_mode = PHY_INTERFACE_MODE_QSGMII;
	else
		p->phy_mode = PHY_INTERFACE_MODE_SGMII;
	if (p->of_node && !of_get_phy_mode(p->of_node, &dt_mode) &&
	    dt_mode != p->phy_mode)
		dev_warn(&p->opip->pdev->dev, "%pOF: phy-mode %s, but interface %d runs %s\n",
			 p->of_node, phy_modes(dt_mode), p->iface->num,
			 phy_modes(p->phy_mode));
}

/* Release what opip_alloc_port() took */
static void opip_put_port(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);

	if (p->phylink)
		phylink_destroy(p->phylink);
	of_node_put(p->of_node);
	free_netdev(dev);
}

/*
 * A net_device for a port, with phylink created. Failures only cost the
 * port.
 */
static struct net_device *opip_alloc_port(struct opip *opip,
					  struct device_node *pip,
					  struct opip_iface *iface, int index)
{
	int port = opip_port_number(iface->num, index);
	const char *name = "eth%d";
	struct net_device *dev;
	struct device_node *np;
	int num_queues;
	struct opip_port *p;

	/* Sending on a port without queues corrupts memory */
	if (port < 0 || !opip_pko_port_valid(opip, port)) {
		dev_warn(&opip->pdev->dev, "interface %d port %d has no output queues, not used\n",
			 iface->num, index);
		return NULL;
	}

	/* A port the device tree disables has nothing connected */
	np = opip_port_of_node(pip, iface->num, index);
	if (np && !of_device_is_available(np)) {
		of_node_put(np);
		return NULL;
	}

	/* A transmit queue per PKO queue, a receive queue per group */
	num_queues = clamp(opip_pko_num_queues(opip, port), 1, OPIP_MAX_TXQS);
	dev = alloc_etherdev_mqs(sizeof(*p), num_queues,
				 hweight32(opip->groups_mask));
	if (!dev) {
		of_node_put(np);
		return NULL;
	}
	SET_NETDEV_DEV(dev, &opip->pdev->dev);

	p = opip_port(dev);
	p->opip = opip;
	p->iface = iface;
	p->netdev = dev;
	p->port = port;
	p->index = index;
	p->of_node = np;
	dev->dev.of_node = np;
	p->phy_mode = PHY_INTERFACE_MODE_NA;
	INIT_WORK(&p->rx_mode_work, opip_rx_mode_worker);
	netif_set_real_num_tx_queues(dev, min_t(int, num_queues,
						num_possible_cpus()));
	dev->min_mtu = VLAN_ETH_ZLEN - OPIP_MTU_OVERHEAD;
	dev->max_mtu = OPIP_MAX_FRAME - OPIP_MTU_OVERHEAD;
	dev->netdev_ops = &opip_netdev_ops;
	dev->hw_features = NETIF_F_RXCSUM | NETIF_F_RXHASH;
	dev->features |= dev->hw_features;
	dev->vlan_features |= dev->hw_features;

	/*
	 * opip_probe() has waited for the nvmem cells: a failure here is final
	 */
	if (of_get_ethdev_address(np, dev)) {
		eth_hw_addr_random(dev);
		if (np)
			dev_warn(&opip->pdev->dev, "%pOF: no MAC address, using a random one\n",
				 np);
	}

	switch (iface->mode) {
	case OPIP_MODE_NPI:
		name = "npi%d";
		break;
	case OPIP_MODE_LOOP:
		name = "loop%d";
		break;
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		opip_sgmii_phy_mode(p);
		break;
	default:
		goto err_put;
	}
	/*
	 * The name templates of the staging driver, which its users have
	 * had since 2009: npi%d, xaui%d and loop%d, eth%d for the others
	 */
	strscpy(dev->name, name, sizeof(dev->name));
	/* The port's node and its IPD/PKO port number, for user space */
	device_set_node(&dev->dev, of_fwnode_handle(np));
	dev->dev_port = port;

	if (opip_iface_is_gmx(iface) && !opip_on_simulator() &&
	    opip_phylink_create(p)) {
		dev_err(&opip->pdev->dev, "failed to set up the link of interface %d port %d\n",
			iface->num, index);
		goto err_put;
	}

	return dev;

err_put:
	opip_put_port(dev);
	return NULL;
}

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
		case OPIP_MODE_SGMII:
		case OPIP_MODE_PICMG:
			opip_gmx_iface_enable(i);
			iface->qsgmii = opip_gmx_iface_is_qsgmii(i);
			break;
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

/*
 * Bring the ports of an interface up, with the IPD enabled, the PKO not.
 * A failure is reported, not fatal: the ports exist, their links stay
 * down.
 */
static void opip_iface_enable(struct opip *opip, struct opip_iface *iface)
{
	int index, ret = 0;

	switch (iface->mode) {
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		ret = opip_sgmii_enable(iface->num, iface->num_ports);
		break;
	case OPIP_MODE_NPI:
		/* Frames of any length; the host enables the ports */
		if (opip_chip->gmx_frm_max)
			break;
		for (index = 0; index < iface->num_ports; index++) {
			int port = opip_port_number(iface->num, index);

			opip_pip_length_checks(port, false);
		}
		break;
	default:
		break;
	}
	if (ret)
		dev_err(&opip->pdev->dev, "interface %d (%s) did not come up: %d\n",
			iface->num, opip_mode_names[iface->mode], ret);
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

/*
 * The interface bring-up enables the MAC of every SGMII and PICMG
 * port. A port without a device has nobody to take its
 * packets: off again.
 */
static void opip_iface_disable_unused(struct opip *opip,
				      struct opip_iface *iface)
{
	int index;

	switch (iface->mode) {
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		break;
	default:
		return;
	}
	for (index = 0; index < iface->num_ports; index++)
		if (!opip->ports[opip_port_number(iface->num, index)])
			opip_gmx_port_enable(iface->num, index, false, 0);
}

/* Packet input on: the IPD, the interfaces */
static void opip_input_enable(struct opip *opip)
{
	int i;

	opip_ipd_enable();
	for (i = 0; i < opip->num_ifaces; i++) {
		if (!opip->ifaces[i].num_ports)
			continue;
		opip_iface_enable(opip, &opip->ifaces[i]);
		opip_iface_disable_unused(opip, &opip->ifaces[i]);
	}
}

/*
 * Undo opip_hw_init() and what the ports put into the pools. @drain once
 * input ran: before, the IPD's FIFOs hold nothing of this driver, and the
 * port FIFO, walked whole, holds what an earlier user left there.
 */
static void opip_free_hw(struct opip *opip, bool drain)
{
	unsigned int owed = 0;
	int i;

	/* The polls are down: none can queue the refill worker again */
	cancel_delayed_work_sync(&opip->refill_work);
	/* What the polls still owe the packet pool is not in it */
	for (i = 0; i < OPIP_MAX_GROUPS; i++)
		owed += opip->groups[i].to_refill;
	opip_pko_reset();
	if (drain)
		opip_ipd_drain();
	opip_fpa_empty_packet_pool(opip, opip->num_packet_buffers - owed);
	/* The IPD's count of the packet pool is zero again with the pool */
	opip_ipd_reset();
	opip_rx_free_pools(opip);
	opip_fpa_empty(opip, OPIP_POOL_WQE, opip->wqe_bufs);
	opip_fpa_empty(opip, OPIP_POOL_CMD, opip->cmd_bufs);
}

/* A device for every port of every interface; returns how many there are */
static int opip_alloc_ports(struct opip *opip, struct device_node *pip)
{
	int i, index, num = 0;

	for (i = 0; i < opip->num_ifaces; i++) {
		struct opip_iface *iface = &opip->ifaces[i];
		bool available, described, missing;
		struct device_node *ni, *np;

		if (iface->mode == OPIP_MODE_DISABLED ||
		    iface->mode == OPIP_MODE_PCIE)
			continue;
		/* An interface the device tree disables has no ports */
		ni = opip_of_child(pip, iface->num);
		available = !ni || of_device_is_available(ni);
		described = !!ni;
		of_node_put(ni);
		if (!available)
			continue;
		for (index = 0; index < iface->num_ports; index++) {
			struct net_device *dev;

			/*
			 * Under an interface node, a port without a node of its
			 * own is not on the board: the platform code removes
			 * the nodes of the ports the executive's board quirks
			 * take away (built-in device tree only)
			 */
			np = opip_port_of_node(pip, iface->num, index);
			missing = described && !np;
			of_node_put(np);
			if (missing)
				continue;
			dev = opip_alloc_port(opip, pip, iface, index);
			if (dev) {
				opip->ports[opip_port(dev)->port] = dev;
				num++;
			}
		}
	}
	return num;
}

/* Register the devices; a port that fails is freed, the others go on */
static int opip_register_ports(struct opip *opip)
{
	int port, num = 0;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		struct net_device *dev = opip->ports[port];
		struct opip_port *p;

		if (!dev)
			continue;
		p = opip_port(dev);
		if (register_netdev(dev)) {
			dev_err(&opip->pdev->dev, "failed to register the device of interface %d port %d\n",
				p->iface->num, p->index);
			/*
			 * No poll or rtnl user of an earlier port may still
			 * hold it when freed
			 */
			rtnl_lock();
			WRITE_ONCE(opip->ports[port], NULL);
			rtnl_unlock();
			synchronize_net();
			opip_port_enable(p, false);
			opip_put_port(dev);
			continue;
		}
		num++;
	}
	return num;
}

/* Unregistering closes the device and waits for its packets in flight */
static void opip_unregister_ports(struct opip *opip)
{
	int port;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		struct net_device *dev = opip->ports[port];
		struct opip_port *p;

		if (!dev || dev->reg_state != NETREG_REGISTERED)
			continue;
		p = opip_port(dev);
		unregister_netdev(dev);
		cancel_work_sync(&p->rx_mode_work);
	}
}

static void opip_free_ports(struct opip *opip)
{
	int port;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		if (!opip->ports[port])
			continue;
		opip_put_port(opip->ports[port]);
		opip->ports[port] = NULL;
	}
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

/*
 * Take the driver down in the order its parts depend on each other: input
 * off, the devices unregistered (each drains its queues), the PKO off, the
 * groups down, and only then the devices and the hardware freed.
 */
static void opip_teardown(struct opip *opip)
{
	opip_ipd_disable();
	WRITE_ONCE(opip->stopping, true);
	cancel_delayed_work_sync(&opip->refill_work);

	opip_unregister_ports(opip);
	opip_pko_stop();
	opip_rx_shutdown(opip);
	opip_free_ports(opip);
	opip_free_hw(opip, true);
	free_netdev(opip->napi_dev);
}

static void opip_remove(struct platform_device *pdev)
{
	struct opip *opip = platform_get_drvdata(pdev);

	if (opip->released)
		return;
	opip->released = true;
	opip_teardown(opip);
}

/*
 * A port's MAC address may sit in an nvmem cell whose provider is not up
 * yet. Find out before the hardware is touched: the probe is deferred as
 * a whole, and a deferred probe that had already set the blocks up would
 * have to take them down again, pools and all.
 *
 * Return: -EPROBE_DEFER if an address is not available yet, else 0
 */
static int opip_wait_for_mac_addresses(struct device_node *pip)
{
	struct device_node *iface, *port;

	for_each_available_child_of_node(pip, iface) {
		for_each_available_child_of_node(iface, port) {
			u8 addr[ETH_ALEN];

			if (of_get_mac_address(port, addr) == -EPROBE_DEFER) {
				of_node_put(port);
				of_node_put(iface);
				return -EPROBE_DEFER;
			}
		}
	}
	return 0;
}

static void opip_free(void *opip)
{
	kfree(opip);
}

static int opip_probe(struct platform_device *pdev)
{
	struct device_node *pip = pdev->dev.of_node;
	struct opip *opip;
	int ret;

	ret = opip_wait_for_mac_addresses(pip);
	if (ret)
		return ret;

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

	/*
	 * Not devm_kzalloc(): the devres header in front of it would leave
	 * the groups off the cache line alignment of struct opip_group
	 */
	opip = kzalloc_obj(*opip);
	if (!opip)
		return -ENOMEM;
	ret = devm_add_action_or_reset(&pdev->dev, opip_free, opip);
	if (ret)
		return ret;
	opip->pdev = pdev;
	mutex_init(&opip->hw_lock);
	INIT_DELAYED_WORK(&opip->refill_work, opip_refill_worker);
	platform_set_drvdata(pdev, opip);

	/*
	 * The receive NAPIs hang off a dummy device: a POW group delivers
	 * the packets of every port (the PIP's tag picks the group), so its
	 * NAPI is none of the ports'. Nobody can set the dummy's
	 * gro_flush_timeout and napi_defer_hard_irqs from user space, so the
	 * driver sets the software interrupt coalescing r8169, bcmgenet,
	 * bcmasp, macb and ravb default to: a poll that ends with work done
	 * leaves the interrupt off once (napi_defer_hard_irqs 1) and is
	 * polled again from a 20 us timer (gro_flush_timeout), which saves
	 * the interrupt, its disable/enable and the group's ack under a
	 * steady load and lets GRO batches grow over polls.
	 */
	opip->napi_dev = alloc_netdev_dummy(0);
	if (!opip->napi_dev)
		return -ENOMEM;
	netdev_sw_irq_coalesce_default_on(opip->napi_dev);

	ret = opip_hw_init(opip);
	if (ret) {
		free_netdev(opip->napi_dev);
		return ret;
	}

	ret = opip_rx_init(opip);
	if (ret)
		goto err_hw;

	if (!opip_alloc_ports(opip, pip)) {
		dev_err(&pdev->dev, "no usable ports\n");
		ret = -ENODEV;
		goto err_ports;
	}

	opip_input_enable(opip);

	if (!opip_register_ports(opip)) {
		dev_err(&pdev->dev, "no device could be registered\n");
		ret = -ENODEV;
		goto err_input;
	}

	schedule_delayed_work(&opip->refill_work, HZ);
	return 0;

err_input:
	opip_teardown(opip);
	return ret;
err_ports:
	opip_rx_shutdown(opip);
	opip_free_ports(opip);
err_hw:
	opip_free_hw(opip, false);
	free_netdev(opip->napi_dev);
	return ret;
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

/* phylink connects the PHY at open: with both modules, the bus must exist */
MODULE_SOFTDEP("pre: mdio-octeon");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Polack <spolack+git@mailbox.org>");
MODULE_DESCRIPTION("Cavium OCTEON PIP/PKO Ethernet driver");
