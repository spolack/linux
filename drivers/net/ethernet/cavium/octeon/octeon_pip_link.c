// SPDX-License-Identifier: GPL-2.0
/*
 * The links of octeon_pip's ports: phylink's MAC operations,
 * flow control, and the RGMII preamble workaround.
 */

#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/spinlock.h>

#include "octeon_pip.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_ipd.h"
#include "octeon_pip_pcs.h"

/*
 * Flow control, for the SGMII, QSGMII and PICMG ports.
 *
 * Receive: the GMX holds the transmitter back for a PAUSE frame it
 * receives. Transmit: the GMX sends PAUSE frames while the IPD asserts
 * backpressure for the port, which it does when more than a threshold of
 * packet buffers hold packets of the port that no poll has taken yet, see
 * opip_ipd_bp_sub(). The threshold is a quarter of the packet buffers in
 * units of 256, the register's, and at least one unit: PAUSE frames come
 * before RED drops (below a quarter of the pool free) for every pool of
 * 342 buffers and more; on smaller ones the 256 buffers are above the
 * RED marks and RED drops first. Which of the two is on is
 * phylink's to say (negotiated by default, ethtool -A), applied at link
 * up, see opip_mac_link_up().
 */

static void opip_pause_apply(struct opip_port *p, bool rx, bool tx)
{
	int iface = p->iface->num;

	lockdep_assert_held(&p->opip->hw_lock);
	p->pause_rx = rx;
	p->pause_tx = tx;
	opip_gmx_set_pause(iface, p->index, rx);
	opip_gmx_clear_bp_override(iface, p->index);
	opip_ipd_bp_config(p->port, tx,
			   max_t(unsigned int, 1,
				 READ_ONCE(p->opip->num_packet_buffers) /
				 (4 * 256)));
}

/**
 * opip_pause_refresh - apply a port's flow control state again
 * @p: Port
 *
 * The thresholds follow the packet pool's size: set the state again, with
 * the receive and transmit settings last applied. Under hw_lock, against
 * phylink's link changes.
 */
void opip_pause_refresh(struct opip_port *p)
{
	if (!opip_port_pause_capable(p))
		return;
	mutex_lock(&p->opip->hw_lock);
	opip_pause_apply(p, p->pause_rx, p->pause_tx);
	mutex_unlock(&p->opip->hw_lock);
}

/**
 * opip_port_link - the link state of a port
 * @p: Port
 *
 * From its PCS, the board or the interface, by the interface mode. GMII
 * and MII have no in-band status: down.
 *
 * Return: the link state, down for an interface mode without a link
 */
struct opip_link opip_port_link(struct opip_port *p)
{
	struct opip_link down = { };
	int iface = p->iface->num;

	switch (p->iface->mode) {
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
		return opip_pcs_link(iface, p->index);
	case OPIP_MODE_RGMII:
		if (p->phy_mode == PHY_INTERFACE_MODE_MII)
			return down;
		return opip_gmx_rgmii_link(iface, p->index);
	case OPIP_MODE_GMII:
		if (p->index == 0 && p->phy_mode != PHY_INTERFACE_MODE_MII)
			return opip_gmx_rgmii_link(iface, p->index);
		return down;
	case OPIP_MODE_XAUI:
		return opip_xaui_link(iface);
	default:
		return down;
	}
}

static void opip_link_state(struct opip_port *p,
			    struct phylink_link_state *state)
{
	struct opip_link link = opip_port_link(p);

	state->link = link.up;
	state->an_complete = link.up;
	if (!link.up)
		return;
	state->speed = link.speed;
	state->duplex = link.full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
}

static struct opip_port *opip_phylink_port(struct phylink_config *config)
{
	return netdev_priv(to_net_dev(config->dev));
}

/*
 * The ports without a PCS (RGMII, GMII, XAUI) and without a PHY have a
 * fixed link whose state the hardware reports. phylink polls it. The
 * PCS of the SGMII ports is in octeon_pip_pcs.c.
 */
static void opip_get_fixed_state(struct phylink_config *config,
				 struct phylink_link_state *state)
{
	opip_link_state(opip_phylink_port(config), state);
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

/*
 * IPD_SUB_PORT_FCS and the port's registers, touched by every port, and
 * the link the preamble check of the periodic worker reads
 */
static DEFINE_SPINLOCK(opip_preamble_lock);

static void opip_link_change(struct opip_port *p, const struct opip_link *link)
{
	bool pad = opip_chip->cn3xxx && p->iface->num < 2 &&
		   !(link->up && link->full_duplex);
	unsigned long flags;

	/*
	 * Errata GMX-401: pad from before half duplex until after it, and
	 * while the link is down, so that no runt queued then goes out on a
	 * half duplex link coming up. A transmit that read the flag clear has
	 * posted its packet before the duplex changes.
	 */
	if (pad && !p->pad_runts) {
		WRITE_ONCE(p->pad_runts, true);
		synchronize_net();
	}
	if (opip_link_set(p, link))
		netdev_err(p->netdev, "setting up the port for link %s failed\n",
			   link->up ? "up" : "down");
	if (!pad)
		WRITE_ONCE(p->pad_runts, false);
	spin_lock_irqsave(&opip_preamble_lock, flags);
	p->link = *link;
	spin_unlock_irqrestore(&opip_preamble_lock, flags);
	if (opip_port_rgmii_preamble(p))
		opip_check_preamble_errors(p->netdev);
}

static void opip_mac_link_down(struct phylink_config *config, unsigned int mode,
			       phy_interface_t interface)
{
	struct opip_port *p = opip_phylink_port(config);
	struct opip_link down = { };

	opip_link_change(p, &down);
	/*
	 * Do not leave the link partner paused, also when the port is
	 * stopped: phylink_stop() takes the link down here.
	 */
	if (opip_port_pause_capable(p)) {
		mutex_lock(&p->opip->hw_lock);
		opip_pause_apply(p, p->pause_rx, false);
		mutex_unlock(&p->opip->hw_lock);
	}
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
	/* No PAUSE frames on a half duplex link */
	if (opip_port_pause_capable(p)) {
		mutex_lock(&p->opip->hw_lock);
		opip_pause_apply(p, rx_pause && link.full_duplex,
				 tx_pause && link.full_duplex);
		mutex_unlock(&p->opip->hw_lock);
	}
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
 * RGMII, GMII, SGMII, PICMG and XAUI ports. A port has a PHY or a fixed
 * link in the device tree, which phylink finds there, or neither. SGMII
 * and PICMG ports run their PCS in band, with a PHY too, unless phylink
 * resolves out of band: a fixed link, or a PHY that cannot run in band.
 * The others refuse in-band status (they have no PCS), and without a PHY
 * or a fixed link take the state the hardware reports: the RGMII in-band
 * status, the XAUI PCS.
 *
 * Return: zero on success, a negative errno otherwise
 */
int opip_phylink_create(struct opip_port *p)
{
	struct phylink_config *config = &p->phylink_config;
	struct device_node *np = p->of_node;
	struct phylink_link_state fixed = {
		.speed = SPEED_1000,
		.duplex = DUPLEX_FULL,
	};
	struct phylink *pl;
	bool phy_less;
	int ret;

	config->dev = &p->netdev->dev;
	config->type = PHYLINK_NETDEV;
	/* Which side adds the RGMII delays is the PHY's and the board's */
	if (phy_interface_mode_is_rgmii(p->phy_mode))
		phy_interface_set_rgmii(config->supported_interfaces);
	else
		__set_bit(p->phy_mode, config->supported_interfaces);
	/* A GMII port runs MII at 10 and 100 Mbit/s */
	if (p->phy_mode == PHY_INTERFACE_MODE_GMII)
		__set_bit(PHY_INTERFACE_MODE_MII, config->supported_interfaces);
	config->mac_capabilities = MAC_10 | MAC_100 | MAC_1000;
	if (p->phy_mode == PHY_INTERFACE_MODE_MII)
		config->mac_capabilities = MAC_10 | MAC_100;
	if (p->iface->mode == OPIP_MODE_XAUI) {
		config->mac_capabilities = MAC_10000FD;
		fixed.speed = SPEED_10000;
	}
	if (opip_port_pause_capable(p))
		config->mac_capabilities |= MAC_SYM_PAUSE | MAC_ASYM_PAUSE;

	phy_less = true;
	if (np) {
		struct fwnode_handle *phy;

		/* The PHY as phylink finds it: phy-handle, phy, phy-device */
		phy = fwnode_get_phy_node(of_fwnode_handle(np));
		phy_less = IS_ERR(phy) && !of_phy_is_fixed_link(np);
		fwnode_handle_put(phy);
	}
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
	} else if (np && of_property_match_string(np, "managed",
						  "in-band-status") >= 0) {
		/*
		 * The RGMII in-band status is what a port without a PHY or
		 * a fixed link falls back to, not a mode to ask for
		 */
		dev_err(&p->opip->pdev->dev,
			"%pOF: managed = \"in-band-status\" needs a PCS, the port has none\n",
			np);
		return -EINVAL;
	} else if (phy_less) {
		/*
		 * The device trees of the platform code and the bootloaders
		 * have ports with neither, which the staging driver ran
		 * from the board's link state, the RGMII in-band status
		 */
		config->get_fixed_state = opip_get_fixed_state;
		config->poll_fixed_state = true;
		if (p->phy_mode == PHY_INTERFACE_MODE_MII ||
		    (p->iface->mode == OPIP_MODE_GMII && p->index != 0))
			dev_warn(&p->opip->pdev->dev,
				 "port %d: no PHY or fixed-link, and %s has no in-band status: the link stays down; update the device tree\n",
				 p->port, phy_modes(p->phy_mode));
		else if (p->iface->mode == OPIP_MODE_RGMII ||
			 p->iface->mode == OPIP_MODE_GMII)
			dev_warn(&p->opip->pdev->dev,
				 "port %d: no PHY or fixed-link, using the RGMII in-band status; update the device tree\n",
				 p->port);
	}

	pl = phylink_create(config, of_fwnode_handle(np), p->phy_mode,
			    &opip_phylink_mac_ops);
	if (IS_ERR(pl))
		return PTR_ERR(pl);

	if (config->get_fixed_state) {
		ret = phylink_set_fixed_link(pl, &fixed);
		if (ret) {
			phylink_destroy(pl);
			return ret;
		}
	}
	p->phylink = pl;
	return 0;
}

/* ------------------------------------------------------------------ */
/* The 10 Mbit/s preamble workaround of RGMII ports */

/*
 * With the preamble check on, the GMX strips the preamble and the IPD the
 * FCS. Some PHYs deliver misaligned preambles at 10 Mbit/s; then the check
 * goes off and the receive path does it, see opip_rx_fix_preamble().
 */
static void opip_set_hw_preamble(struct opip_port *p, bool enable)
{
	opip_gmx_set_preamble_check(p->iface->num, p->index, enable);
	opip_ipd_fcs_strip(p->port, enable);
}

/**
 * opip_check_preamble_errors - the 10 Mbit/s preamble workaround of a port
 * @dev: The port's network device
 *
 * On every change of the link, and once a second while the port is open.
 * At 10 Mbit/s, preamble errors turn the hardware preamble check off; a
 * change of the speed turns it on again.
 */
void opip_check_preamble_errors(struct net_device *dev)
{
	struct opip_port *p = opip_port(dev);
	unsigned long flags;

	spin_lock_irqsave(&opip_preamble_lock, flags);
	if (p->link.speed == 10 && p->last_speed == 10) {
		if (opip_gmx_preamble_errors(p->iface->num, p->index)) {
			opip_set_hw_preamble(p, false);
			net_dbg_ratelimited("%s: using 10 Mbit/s with software preamble removal\n",
					    netdev_name(dev));
		}
	} else {
		/* Every speed change starts with the check on */
		if (p->last_speed != p->link.speed)
			opip_set_hw_preamble(p, true);
		p->last_speed = p->link.speed;
	}
	spin_unlock_irqrestore(&opip_preamble_lock, flags);
}
