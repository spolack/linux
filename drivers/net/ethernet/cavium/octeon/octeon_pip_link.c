// SPDX-License-Identifier: GPL-2.0
/*
 * The links of octeon_pip's ports: phylink's MAC operations.
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>

#include "octeon_pip.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_pcs.h"

static struct opip_port *opip_phylink_port(struct phylink_config *config)
{
	return netdev_priv(to_net_dev(config->dev));
}

static struct phylink_pcs *opip_mac_select_pcs(struct phylink_config *config,
					       phy_interface_t interface)
{
	struct opip_port *p = opip_phylink_port(config);

	return p->pcs.ops ? &p->pcs : NULL;
}

static void opip_mac_config(struct phylink_config *config, unsigned int mode,
			    const struct phylink_link_state *state)
{
	/* The port was set up for its interface mode at probe */
}

static void opip_link_change(struct opip_port *p, const struct opip_link *link)
{
	if (opip_link_set(p, link))
		netdev_err(p->netdev, "setting up the port for link %s failed\n",
			   link->up ? "up" : "down");
}

static void opip_mac_link_down(struct phylink_config *config, unsigned int mode,
			       phy_interface_t interface)
{
	struct opip_port *p = opip_phylink_port(config);
	struct opip_link down = { };

	opip_link_change(p, &down);
}

static void opip_mac_link_up(struct phylink_config *config,
			     struct phy_device *phy, unsigned int mode,
			     phy_interface_t interface, int speed, int duplex,
			     bool tx_pause, bool rx_pause)
{
	struct opip_port *p = opip_phylink_port(config);
	struct opip_link link = {
		.up = true,
		.full_duplex = duplex == DUPLEX_FULL,
		.speed = speed,
	};

	opip_link_change(p, &link);
}

static const struct phylink_mac_ops opip_phylink_mac_ops = {
	.mac_select_pcs = opip_mac_select_pcs,
	.mac_config = opip_mac_config,
	.mac_link_down = opip_mac_link_down,
	.mac_link_up = opip_mac_link_up,
};

/**
 * opip_phylink_create - set up phylink for a port with a MAC
 * @p: Port, its interface mode and phy_mode known
 *
 * SGMII and PICMG ports. Their PCS runs in band, with a PHY too, unless
 * phylink resolves out of band: a fixed link, or a PHY that cannot run in
 * band.
 *
 * Return: zero on success, a negative errno otherwise
 */
int opip_phylink_create(struct opip_port *p)
{
	struct phylink_config *config = &p->phylink_config;
	struct phylink *pl;

	config->dev = &p->netdev->dev;
	config->type = PHYLINK_NETDEV;
	__set_bit(p->phy_mode, config->supported_interfaces);
	/* A GMII port runs MII at 10 and 100 Mbit/s */
	if (p->phy_mode == PHY_INTERFACE_MODE_GMII)
		__set_bit(PHY_INTERFACE_MODE_MII, config->supported_interfaces);
	config->mac_capabilities = MAC_10 | MAC_100 | MAC_1000;
	if (p->phy_mode == PHY_INTERFACE_MODE_MII)
		config->mac_capabilities = MAC_10 | MAC_100;

	if (opip_port_has_pcs(p)) {
		/*
		 * The PCS and the other end must agree on the in-band
		 * autonegotiation. In band is the default, as in the SDK:
		 * phylink then resolves the mode from the PCS's and the PHY
		 * driver's capabilities, and runs a PHY that cannot do in
		 * band, and a fixed link, out of band.
		 */
		config->default_an_inband = true;
		p->pcs.ops = &opip_pcs_ops;
		p->pcs.poll = true;
		__set_bit(p->phy_mode, p->pcs.supported_interfaces);
	}

	pl = phylink_create(config, of_fwnode_handle(p->of_node), p->phy_mode,
			    &opip_phylink_mac_ops);
	if (IS_ERR(pl))
		return PTR_ERR(pl);

	p->phylink = pl;
	return 0;
}
