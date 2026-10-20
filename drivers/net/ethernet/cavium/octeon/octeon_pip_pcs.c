// SPDX-License-Identifier: GPL-2.0
/*
 * The PCS blocks: the SGMII/QSGMII PCS of each port of a GMX, and the
 * XAUI PCS of an interface.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-helper-sgmii.c, cvmx-helper-xaui.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/mutex.h>
#include <linux/phylink.h>

#include "octeon_pip.h"
#include "octeon_pip_gmx.h"
#include "octeon_pip_pcs.h"
#include "octeon_pip_pko.h"
#include "octeon_pip_regs.h"

/* ------------------------------------------------------------------ */
/* SGMII, QSGMII and 1000BASE-X */

/**
 * opip_pcs_basex - whether a port's PCS runs 1000BASE-X
 * @iface: Interface
 * @index: Port on it
 *
 * The bootloader's PCS_MISC_CTL[MODE], as the executive takes it
 * (cvmx-helper-sgmii.c), on PICMG interfaces too: a PICMG 3.1 backplane
 * board sets the mode. The driver does not switch it: no board has an
 * SFP cage directly on a PCS.
 *
 * Return: true for 1000BASE-X, false for SGMII
 */
bool opip_pcs_basex(int iface, int index)
{
	return opip_rd(OPIP_PCS_MISC_CTL(iface, index)) &
	       OPIP_PCS_MISC_CTL_MODE;
}

/*
 * The one-time setup of a port's PCS: the link timer (1.6 ms for SGMII,
 * 10 ms for 1000BASE-X) and the advertisement. In SGMII MAC mode the
 * advertisement is the fixed 0x4001, nothing to write.
 */
static void opip_pcs_init_port(int iface, int index)
{
	u64 misc = opip_rd(OPIP_PCS_MISC_CTL(iface, index));
	u64 clock_mhz = opip_clock_hz() / 1000000;
	bool basex = opip_pcs_basex(iface, index);
	u64 count;

	opip_gmx_port_enable(iface, index, false, 0);

	if (basex)
		count = (10000ull * clock_mhz) >> 10;
	else
		count = (1600ull * clock_mhz) >> 10;
	opip_wr(OPIP_PCS_LINK_TIMER_COUNT(iface, index),
		u64_replace_bits(opip_rd(OPIP_PCS_LINK_TIMER_COUNT(iface,
								   index)),
				 count, OPIP_PCS_LINK_TIMER_COUNT_COUNT));

	if (basex) {
		/* 1000BASE-X: full and half duplex, both pause bits */
		u64 adv = opip_rd(OPIP_PCS_AN_ADV(iface, index));

		adv = u64_replace_bits(adv, 0, OPIP_PCS_AN_ADV_REM_FLT);
		adv = u64_replace_bits(adv, 3, OPIP_PCS_AN_ADV_PAUSE);
		adv |= OPIP_PCS_AN_ADV_HFD | OPIP_PCS_AN_ADV_FD;
		opip_wr(OPIP_PCS_AN_ADV(iface, index), adv);
	} else if (misc & OPIP_PCS_MISC_CTL_MAC_PHY) {
		/* SGMII PHY mode: the link, 1 Gbit/s full duplex */
		u64 adv = opip_rd(OPIP_PCS_SGM_AN_ADV(iface, index));

		adv |= OPIP_PCS_SGM_AN_ADV_LINK | OPIP_PCS_SGM_AN_ADV_DUP;
		adv = u64_replace_bits(adv, 2, OPIP_PCS_SGM_AN_ADV_SPEED);
		opip_wr(OPIP_PCS_SGM_AN_ADV(iface, index), adv);
	}
}

/*
 * Power the PCS of each port down and up again, as the SDK does at every
 * link setup (cvmx-helper-sgmii.c): QCA8033 PHYs and QCA833x switches
 * pass no traffic after a warm reset until the PCS at the other end of
 * the link has been powered down; a reset of the PCS or of the Qualcomm
 * device does not bring them back. The bootloader may have the PCS set
 * up already, which opip_pcs_config() then keeps, so this is done here,
 * once, with the ports disabled. They wait together: 25 ms, 10 ms is too
 * short (SDK). A PCS with autonegotiation on starts a new one.
 */
static void opip_pcs_power_cycle(int iface, int num_ports)
{
	int index;
	u64 ctl;

	for (index = 0; index < num_ports; index++) {
		ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));
		opip_wr(OPIP_PCS_MR_CONTROL(iface, index),
			ctl | OPIP_PCS_MR_CONTROL_PWR_DN);
	}
	msleep(25);
	for (index = 0; index < num_ports; index++) {
		ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));
		ctl &= ~OPIP_PCS_MR_CONTROL_PWR_DN;
		if (ctl & OPIP_PCS_MR_CONTROL_AN_EN)
			ctl |= OPIP_PCS_MR_CONTROL_RST_AN;
		opip_wr(OPIP_PCS_MR_CONTROL(iface, index), ctl);
	}
}

/**
 * opip_sgmii_enable - bring up the SGMII ports of an interface
 * @iface:     Interface
 * @num_ports: Ports on it
 *
 * The GMX port counts, the PCS of each port and its power cycle, then the
 * ports enabled. On the simulator, which has no link polling, the link
 * is set up here instead of the power cycle. The PCS of a port is set up
 * for the link later, from phylink, see opip_pcs_config().
 *
 * Return: 0, or -EINVAL for more ports than a GMX has
 */
int opip_sgmii_enable(int iface, int num_ports)
{
	int index, ret;

	ret = opip_gmx_setup(iface, num_ports);
	if (ret)
		return ret;
	for (index = 0; index < num_ports; index++) {
		opip_pcs_init_port(iface, index);
		if (opip_on_simulator()) {
			struct opip_link link = opip_pcs_link(iface, index);

			opip_pcs_link_up(iface, index, &link, true);
			opip_pcs_link_set(iface, index, &link);
		}
	}
	if (!opip_on_simulator())
		opip_pcs_power_cycle(iface, num_ports);
	for (index = 0; index < num_ports; index++)
		opip_gmx_port_enable(iface, index, true,
				     opip_port_number(iface, index));
	return 0;
}

/*
 * Errata G-15618: no soft reset of the PCS on CN63XX, CN66XX pass 1 and
 * CN68XX; the SDK only restarts the autonegotiation there
 * (cvmx-helper-sgmii.c, __cvmx_helper_need_g15618(); its comment names
 * CN63XX up to pass 2.1, its code, followed here, every CN63XX).
 */
static bool opip_pcs_no_reset(void)
{
	return OCTEON_IS_MODEL(OCTEON_CN63XX) ||
	       OCTEON_IS_MODEL(OCTEON_CN66XX_PASS1_X) ||
	       OCTEON_IS_MODEL(OCTEON_CN68XX);
}

/**
 * opip_pcs_reset - take a port's PCS through a reset and start a new AN
 * @iface: Interface
 * @index: Port on it
 *
 * No reset on the simulator and on the chips of errata G-15618, see
 * opip_pcs_no_reset(): only the new autonegotiation. Does not wait for
 * it: it completes once the PHY's in-band signalling is there, which
 * opip_pcs_link() reports. Speed and duplex bits a previous out-of-band
 * link left in PCS_MR_CONTROL stay: with AN_EN set the PCS takes both
 * from the autonegotiation.
 *
 * Return: 0, or -ETIMEDOUT if the PCS does not come out of the reset
 */
static int opip_pcs_reset(int iface, int index)
{
	u64 ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));

	if (!opip_on_simulator() && !opip_pcs_no_reset()) {
		opip_wr(OPIP_PCS_MR_CONTROL(iface, index),
			ctl | OPIP_PCS_MR_CONTROL_RESET);
		if (read_poll_timeout(opip_rd, ctl,
				      !(ctl & OPIP_PCS_MR_CONTROL_RESET),
				      10, 10000, false,
				      OPIP_PCS_MR_CONTROL(iface, index)))
			return -ETIMEDOUT;
	}
	ctl |= OPIP_PCS_MR_CONTROL_RST_AN | OPIP_PCS_MR_CONTROL_AN_EN;
	ctl &= ~(OPIP_PCS_MR_CONTROL_PWR_DN | OPIP_PCS_MR_CONTROL_RESET);
	opip_wr(OPIP_PCS_MR_CONTROL(iface, index), ctl);
	return 0;
}

/*
 * Out of band: the autonegotiation off, the PCS powered up, 1000 Mbit/s
 * full duplex (the SDK's forced setting) until opip_pcs_link_up() sets
 * the speed and duplex of the link. A PCS already out of band keeps its
 * link: nothing is written.
 */
static void opip_pcs_outband(int iface, int index)
{
	const u64 off = OPIP_PCS_MR_CONTROL_RESET | OPIP_PCS_MR_CONTROL_AN_EN |
			OPIP_PCS_MR_CONTROL_PWR_DN;
	u64 ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));

	if (!(ctl & off))
		return;
	ctl &= ~(off | OPIP_PCS_MR_CONTROL_RST_AN | OPIP_PCS_MR_CONTROL_FIXED);
	ctl |= mii_bmcr_encode_fixed(SPEED_1000, DUPLEX_FULL);
	opip_wr(OPIP_PCS_MR_CONTROL(iface, index), ctl);
}

/**
 * opip_pcs_config - set a port's PCS up for in-band or out-of-band operation
 * @iface:  Interface
 * @index:  Port on it
 * @inband: In-band autonegotiation, as phylink resolved it
 *
 * In band: a reset and a new autonegotiation, unless the PCS is set up
 * for it already (PCS_MR_CONTROL[AN_EN], powered up, out of reset), when
 * a reset would only take a link that is up down again. Decided by what
 * is programmed, not by the link status: PCS_MR_STATUS[LNK_ST] latches
 * low, and reading it here would hide a lost link from pcs_get_state.
 * Out of band: see opip_pcs_outband(). The PCS and the PHY must agree:
 * a PCS waiting for an autonegotiation the PHY does not send, or the
 * other way round, receives nothing. The mode (PCS_MISC_CTL[MODE]) is
 * the bootloader's and stays.
 *
 * Return: 0, or the error of opip_pcs_reset()
 */
int opip_pcs_config(int iface, int index, bool inband)
{
	u64 ctl;

	if (!inband) {
		opip_pcs_outband(iface, index);
		return 0;
	}
	ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));
	if ((ctl & OPIP_PCS_MR_CONTROL_AN_EN) &&
	    !(ctl & (OPIP_PCS_MR_CONTROL_PWR_DN | OPIP_PCS_MR_CONTROL_RESET)))
		return 0;
	return opip_pcs_reset(iface, index);
}

/**
 * opip_pcs_an_restart - restart a port's PCS autonegotiation
 * @iface: Interface
 * @index: Port on it
 *
 * Sets PCS_MR_CONTROL[RST_AN], without a reset of the PCS. phylink asks
 * for it in band only (1000BASE-X).
 */
void opip_pcs_an_restart(int iface, int index)
{
	opip_wr(OPIP_PCS_MR_CONTROL(iface, index),
		opip_rd(OPIP_PCS_MR_CONTROL(iface, index)) |
		OPIP_PCS_MR_CONTROL_RST_AN);
}

/* The simulator and the PCS's internal loopback: up, 1 Gbit/s full duplex */
static bool opip_pcs_forced_up(int iface, int index)
{
	return opip_on_simulator() ||
	       (opip_rd(OPIP_PCS_MR_CONTROL(iface, index)) &
		OPIP_PCS_MR_CONTROL_LOOPBCK1);
}

/**
 * opip_pcs_link - the link state an SGMII port's PCS received in band
 * @iface: Interface
 * @index: Port on it
 *
 * The PHY, or whatever else is at the other end, sends its link state in
 * the SGMII autonegotiation, and the PCS keeps what it got in
 * PCS_AN_RESULTS, in MAC mode (from the PHY) as in PHY mode. That stays
 * when the link is lost, so the link status of the PCS has to be up too.
 * The executive only read it in PHY mode and took the board's word for
 * the link in MAC mode, which the arch no longer gives for the newer
 * chips. 1000BASE-X is decoded by phylink, see opip_pcs_get_state().
 *
 * Return: the link state, down unless both agree
 */
struct opip_link opip_pcs_link(int iface, int index)
{
	struct opip_link link = { };
	u64 status, results;

	if (opip_pcs_forced_up(iface, index))
		return (struct opip_link){ .up = true, .full_duplex = true,
					   .speed = 1000 };

	status = opip_rd(OPIP_PCS_MR_STATUS(iface, index));
	results = opip_rd(OPIP_PCS_AN_RESULTS(iface, index));
	if (!(status & OPIP_PCS_MR_STATUS_LNK_ST) ||
	    !(results & OPIP_PCS_AN_RESULTS_AN_CPT) ||
	    !(results & OPIP_PCS_AN_RESULTS_LINK_OK))
		return link;

	switch (FIELD_GET(OPIP_PCS_AN_RESULTS_SPD, results)) {
	case 0:
		link.speed = 10;
		break;
	case 1:
		link.speed = 100;
		break;
	case 2:
		link.speed = 1000;
		break;
	default:
		return link;
	}
	link.up = true;
	link.full_duplex = results & OPIP_PCS_AN_RESULTS_DUP;
	return link;
}

/*
 * A port's GMX disabled and polled idle around a change of the link
 * setup, its enable state restored afterwards: opip_pcs_gmx_hold() returns
 * whether it was enabled, or a negative errno if the GMX did not go idle
 * (then the port keeps its old setup, and is enabled again as it was).
 */
static int opip_pcs_gmx_hold(int iface, int index)
{
	bool enabled = opip_gmx_port_enabled(iface, index);
	int ret;

	opip_gmx_port_enable(iface, index, false, 0);
	ret = opip_gmx_wait_idle(iface, index);
	if (ret) {
		if (enabled)
			opip_gmx_port_enable(iface, index, true,
					     opip_port_number(iface, index));
		return ret;
	}
	return enabled;
}

static void opip_pcs_gmx_release(int iface, int index, int enabled)
{
	if (enabled)
		opip_gmx_port_enable(iface, index, true,
				     opip_port_number(iface, index));
}

/**
 * opip_pcs_link_up - set an SGMII port's PCS up for the speed of a link
 * @iface:  Interface
 * @index:  Port on it
 * @link:   The link's speed and duplex
 * @inband: The PCS runs the in-band autonegotiation
 *
 * With the GMX disabled and idle: the sample point of the speed in
 * PCS_MISC_CTL[SAMP_PT] (the line runs at 1.25 Gbaud at every speed, the
 * PHY repeats the symbols at 10 and 100 Mbit/s), and out of band the speed
 * and duplex in PCS_MR_CONTROL (a clause 22 BMCR), which the PCS takes
 * from the autonegotiation otherwise: at a speed other than the PCS's no
 * frame passes.
 *
 * Return: 0, or -ETIMEDOUT if the GMX does not go idle
 */
int opip_pcs_link_up(int iface, int index, const struct opip_link *link,
		     bool inband)
{
	int enabled = opip_pcs_gmx_hold(iface, index);
	u64 misc, ctl;

	if (enabled < 0)
		return enabled;
	misc = opip_rd(OPIP_PCS_MISC_CTL(iface, index));
	switch (link->speed) {
	case SPEED_10:
		misc = u64_replace_bits(misc, 25, OPIP_PCS_MISC_CTL_SAMP_PT);
		break;
	case SPEED_100:
		misc = u64_replace_bits(misc, 5, OPIP_PCS_MISC_CTL_SAMP_PT);
		break;
	case SPEED_1000:
		misc = u64_replace_bits(misc, 1, OPIP_PCS_MISC_CTL_SAMP_PT);
		break;
	default:
		break;
	}
	opip_wr(OPIP_PCS_MISC_CTL(iface, index), misc);
	if (!inband) {
		ctl = opip_rd(OPIP_PCS_MR_CONTROL(iface, index));
		ctl &= ~(OPIP_PCS_MR_CONTROL_RST_AN | OPIP_PCS_MR_CONTROL_FIXED);
		ctl |= mii_bmcr_encode_fixed(link->speed, link->full_duplex ?
					     DUPLEX_FULL : DUPLEX_HALF);
		opip_wr(OPIP_PCS_MR_CONTROL(iface, index), ctl);
	}
	opip_pcs_gmx_release(iface, index, enabled);
	return 0;
}

/**
 * opip_pcs_link_set - set an SGMII port's MAC up for a link
 * @iface: Interface
 * @index: Port on it
 * @link:  The link state
 *
 * With the GMX disabled and idle: the GMX programmed for the speed and
 * duplex, and held off by the PCS (PCS_MISC_CTL[GMXENO]) for a down link,
 * released for an up one, after the speed is set: the GMX never runs at
 * the old speed on the new link. phylink has no PCS operation for a
 * link going down, so the hold-off is done here, for the MAC.
 *
 * Return: 0, or -ETIMEDOUT if the GMX does not go idle
 */
int opip_pcs_link_set(int iface, int index, const struct opip_link *link)
{
	int enabled = opip_pcs_gmx_hold(iface, index);
	u64 misc;

	if (enabled < 0)
		return enabled;
	opip_gmx_set_speed(iface, index, link, false);
	misc = opip_rd(OPIP_PCS_MISC_CTL(iface, index));
	if (link->up)
		misc &= ~OPIP_PCS_MISC_CTL_GMXENO;
	else
		misc |= OPIP_PCS_MISC_CTL_GMXENO;
	opip_wr(OPIP_PCS_MISC_CTL(iface, index), misc);
	opip_pcs_gmx_release(iface, index, enabled);
	return 0;
}

/* ------------------------------------------------------------------ */
/* The phylink PCS of an SGMII, QSGMII or 1000BASE-X port */

/*
 * In band, phylink polls the PCS once a second, and with a PHY takes the
 * link as up when both say so. Out of band, with a PHY, the link is the
 * PHY's and phylink does not ask the PCS. The PCS is set up from here,
 * the MAC operations set up the GMX.
 */
static struct opip_port *opip_pcs_port(struct phylink_pcs *pcs)
{
	return container_of(pcs, struct opip_port, pcs);
}

static void opip_pcs_get_state(struct phylink_pcs *pcs, unsigned int neg_mode,
			       struct phylink_link_state *state)
{
	struct opip_port *p = opip_pcs_port(pcs);
	int iface = p->iface->num;
	struct opip_link link;
	u16 status, lpa;

	/*
	 * phylink asks in band only: SGMII and QSGMII with the
	 * autonegotiation, 1000BASE-X with it or without (INBAND_DISABLED).
	 * 1000BASE-X: a clause 37 advertisement, phylink decodes it.
	 */
	if (state->interface == PHY_INTERFACE_MODE_1000BASEX &&
	    !opip_pcs_forced_up(iface, p->index)) {
		status = opip_rd(OPIP_PCS_MR_STATUS(iface, p->index));
		lpa = opip_rd(OPIP_PCS_AN_LP_ABIL(iface, p->index));
		phylink_mii_c22_pcs_decode_state(state, neg_mode, status, lpa);
		return;
	}

	/* SGMII carries no pause */
	link = opip_pcs_link(iface, p->index);
	state->link = link.up;
	state->an_complete = link.up;
	if (!link.up)
		return;
	state->speed = link.speed;
	state->duplex = link.full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
}

static int opip_pcs_ops_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			       phy_interface_t interface,
			       const unsigned long *advertising,
			       bool permit_pause_to_mac)
{
	struct opip_port *p = opip_pcs_port(pcs);
	int ret;

	/*
	 * Out of band and INBAND_DISABLED (1000BASE-X without the
	 * autonegotiation) are the same to the PCS: AN_EN off, the speed and
	 * duplex forced
	 */
	ret = opip_pcs_setup(p, neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED);
	if (ret) {
		netdev_err(p->netdev, "setting up the PCS failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static void opip_pcs_ops_an_restart(struct phylink_pcs *pcs)
{
	opip_pcs_restart_an(opip_pcs_port(pcs));
}

static void opip_pcs_ops_link_up(struct phylink_pcs *pcs, unsigned int neg_mode,
				 phy_interface_t interface, int speed,
				 int duplex)
{
	struct opip_port *p = opip_pcs_port(pcs);
	struct opip_link link = {
		.up = true,
		.full_duplex = duplex == DUPLEX_FULL,
		.speed = speed,
	};
	int ret;

	mutex_lock(&p->opip->hw_lock);
	ret = opip_pcs_link_up(p->iface->num, p->index, &link,
			       neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED);
	mutex_unlock(&p->opip->hw_lock);
	if (ret)
		netdev_err(p->netdev, "setting up the PCS for the link failed: %d\n",
			   ret);
}

/*
 * The PCS runs with the in-band autonegotiation or without it, as phylink
 * resolves it: in band by default, out of band for a fixed link or a PHY
 * that cannot run in band, and on 1000BASE-X also with the
 * autonegotiation turned off by ethtool. A PCS supports one interface
 * mode, the port's.
 */
static unsigned int opip_pcs_inband_caps(struct phylink_pcs *pcs,
					 phy_interface_t interface)
{
	return LINK_INBAND_DISABLE | LINK_INBAND_ENABLE;
}

const struct phylink_pcs_ops opip_pcs_ops = {
	.pcs_inband_caps = opip_pcs_inband_caps,
	.pcs_get_state = opip_pcs_get_state,
	.pcs_config = opip_pcs_ops_config,
	.pcs_an_restart = opip_pcs_ops_an_restart,
	.pcs_link_up = opip_pcs_ops_link_up,
};

/**
 * opip_pcs_poll_errors - count and clear the error bits of a port's PCS
 * @iface: Interface
 * @index: Port on it
 *
 * Only the bits reported are cleared in PCS_INT_REG.
 *
 * Return: the OPIP_PCS_ERR_* bits that were set
 */
unsigned int opip_pcs_poll_errors(int iface, int index)
{
	u64 pcs = opip_rd(OPIP_PCS_INT_REG(iface, index));
	unsigned int errors = 0;
	u64 clr = 0;

	if (pcs & OPIP_PCS_INT_AN_ERR) {
		clr |= OPIP_PCS_INT_AN_ERR;
		errors |= OPIP_PCS_ERR_AN;
	}
	if (pcs & (OPIP_PCS_INT_TXFIFO | OPIP_PCS_INT_TXFIFU)) {
		clr |= pcs & (OPIP_PCS_INT_TXFIFO | OPIP_PCS_INT_TXFIFU);
		errors |= OPIP_PCS_ERR_TX_FIFO;
	}
	if (clr)
		opip_wr(OPIP_PCS_INT_REG(iface, index), clr);
	return errors;
}

/* ------------------------------------------------------------------ */
/* XAUI */

/**
 * opip_xaui_probe - the probe time setup of a XAUI interface
 * @iface: Interface
 *
 * Enable the interface (errata GMX-700), one GMX port, 16 PKO ports.
 */
void opip_xaui_probe(int iface)
{
	opip_gmx_iface_enable(iface);
	opip_gmx_setup(iface, 1);
	opip_pko_xaui_ports(iface);
}

/**
 * opip_xaui_enable - bring up a XAUI interface
 * @iface: Interface
 * @port:  The IPD/PKO port of its one port, the port kind on CN68XX
 *
 * The sequence of the manual: GMX held off, interrupts off, the
 * reconciliation layer, the PCS reset (hangs CN66XX and CN68XX pass 1
 * and 2, skipped there), the GMX configured idle for 10 Gbit/s, the
 * errors cleared, the link waited for, and the port enabled.
 *
 * Return: 0, or -ETIMEDOUT at the step that did not complete
 */
int opip_xaui_enable(int iface, int port)
{
	u64 rx_int_en, tx_int_en, pcs_int_en, ctl, st;

	if (opip_chip->cn68xx)
		opip_wr(OPIP_GMX_PRT_CFG(iface, 0),
			u64_replace_bits(opip_rd(OPIP_GMX_PRT_CFG(iface, 0)),
					 port, OPIP_GMX_PRT_CFG_PKND));

	opip_wr(OPIP_PCSX_MISC_CTL(iface),
		opip_rd(OPIP_PCSX_MISC_CTL(iface)) | OPIP_PCSX_MISC_CTL_GMXENO);

	rx_int_en = opip_rd(OPIP_GMX_RX_INT_EN(iface, 0));
	opip_wr(OPIP_GMX_RX_INT_EN(iface, 0), 0);
	tx_int_en = opip_rd(OPIP_GMX_TX_INT_EN(iface));
	opip_wr(OPIP_GMX_TX_INT_EN(iface), 0);
	pcs_int_en = opip_rd(OPIP_PCSX_INT_EN(iface));
	opip_wr(OPIP_PCSX_INT_EN(iface), 0);

	/* Deficit idle count: better inter-frame gap packing */
	ctl = opip_rd(OPIP_GMX_TX_XAUI_CTL(iface));
	ctl |= OPIP_GMX_TX_XAUI_CTL_DIC_EN;
	ctl &= ~OPIP_GMX_TX_XAUI_CTL_UNI_EN;
	opip_wr(OPIP_GMX_TX_XAUI_CTL(iface), ctl);

	ctl = opip_rd(OPIP_PCSX_CONTROL1(iface)) & ~OPIP_PCSX_CONTROL1_LO_PWR;
	if (!OCTEON_IS_MODEL(OCTEON_CN66XX) &&
	    !OCTEON_IS_MODEL(OCTEON_CN68XX_PASS1_X) &&
	    !OCTEON_IS_MODEL(OCTEON_CN68XX_PASS2_X))
		ctl |= OPIP_PCSX_CONTROL1_RESET;
	opip_wr(OPIP_PCSX_CONTROL1(iface), ctl);

	if (read_poll_timeout(opip_rd, st, !(st & OPIP_PCSX_CONTROL1_RESET),
			      10, 10000, false, OPIP_PCSX_CONTROL1(iface)))
		return -ETIMEDOUT;
	if (read_poll_timeout(opip_rd, st, st & OPIP_PCSX_10GBX_STATUS_ALIGND,
			      10, 10000, false, OPIP_PCSX_10GBX_STATUS(iface)))
		return -ETIMEDOUT;
	if (read_poll_timeout(opip_rd, st,
			      !FIELD_GET(OPIP_GMX_RX_XAUI_CTL_STATUS, st),
			      10, 10000, false, OPIP_GMX_RX_XAUI_CTL(iface)))
		return -ETIMEDOUT;

	opip_gmx_port_enable(iface, 0, false, 0);
	if (opip_gmx_wait_idle(iface, 0))
		return -ETIMEDOUT;

	ctl = opip_rd(OPIP_GMX_PRT_CFG(iface, 0));
	ctl |= OPIP_GMX_PRT_CFG_SPEED | OPIP_GMX_PRT_CFG_SLOTTIME;
	ctl &= ~OPIP_GMX_PRT_CFG_SPEED_MSB;
	opip_wr(OPIP_GMX_TX_PRTS(iface), 1);
	opip_wr(OPIP_GMX_TX_SLOT(iface, 0), OPIP_GMX_TX_SLOT_1000);
	opip_wr(OPIP_GMX_TX_BURST(iface, 0), OPIP_GMX_TX_BURST_1000);
	opip_wr(OPIP_GMX_PRT_CFG(iface, 0), ctl);

	opip_wr(OPIP_GMX_RX_INT_REG(iface, 0),
		opip_rd(OPIP_GMX_RX_INT_REG(iface, 0)));
	opip_wr(OPIP_GMX_TX_INT_REG(iface),
		opip_rd(OPIP_GMX_TX_INT_REG(iface)));
	opip_wr(OPIP_PCSX_INT_REG(iface), opip_rd(OPIP_PCSX_INT_REG(iface)));

	if (read_poll_timeout(opip_rd, st, st & OPIP_PCSX_STATUS1_RCV_LNK,
			      10, 10000, false, OPIP_PCSX_STATUS1(iface)))
		return -ETIMEDOUT;
	if (read_poll_timeout(opip_rd, st, !(st & OPIP_PCSX_STATUS2_XMTFLT),
			      10, 10000, false, OPIP_PCSX_STATUS2(iface)))
		return -ETIMEDOUT;
	if (read_poll_timeout(opip_rd, st, !(st & OPIP_PCSX_STATUS2_RCVFLT),
			      10, 10000, false, OPIP_PCSX_STATUS2(iface)))
		return -ETIMEDOUT;

	opip_wr(OPIP_GMX_RX_INT_EN(iface, 0), rx_int_en);
	opip_wr(OPIP_GMX_TX_INT_EN(iface), tx_int_en);
	opip_wr(OPIP_PCSX_INT_EN(iface), pcs_int_en);

	opip_wr(OPIP_PCSX_MISC_CTL(iface),
		opip_rd(OPIP_PCSX_MISC_CTL(iface)) &
		~OPIP_PCSX_MISC_CTL_GMXENO);
	opip_gmx_port_enable(iface, 0, true, port);
	return 0;
}

/**
 * opip_xaui_link - the link state of a XAUI interface
 * @iface: Interface
 *
 * Up only with both sides of the GMX happy and the PCS receiving. A down
 * link turns the GMX and PCS interrupts off.
 *
 * Return: the link state, 10 Gbit/s full duplex when up
 */
struct opip_link opip_xaui_link(int iface)
{
	struct opip_link link = { };

	if (!FIELD_GET(OPIP_GMX_TX_XAUI_CTL_LS,
		       opip_rd(OPIP_GMX_TX_XAUI_CTL(iface))) &&
	    !FIELD_GET(OPIP_GMX_RX_XAUI_CTL_STATUS,
		       opip_rd(OPIP_GMX_RX_XAUI_CTL(iface))) &&
	    (opip_rd(OPIP_PCSX_STATUS1(iface)) & OPIP_PCSX_STATUS1_RCV_LNK)) {
		link.up = true;
		link.full_duplex = true;
		link.speed = 10000;
	} else {
		opip_wr(OPIP_GMX_RX_INT_EN(iface, 0), 0);
		opip_wr(OPIP_GMX_TX_INT_EN(iface), 0);
		opip_wr(OPIP_PCSX_INT_EN(iface), 0);
	}
	return link;
}

/**
 * opip_xaui_link_set - set a XAUI interface up for a link
 * @iface: Interface
 * @port:  The IPD/PKO port of its one port, for opip_xaui_enable()
 * @link:  The link state
 *
 * A link that should be up and is not, or an interface whose bring-up did
 * not complete (GMXENO still set), gets the interface brought up again.
 *
 * Return: 0, or the error of opip_xaui_enable()
 */
int opip_xaui_link_set(int iface, int port, const struct opip_link *link)
{
	if (!link->up)
		return 0;
	if (!FIELD_GET(OPIP_GMX_TX_XAUI_CTL_LS,
		       opip_rd(OPIP_GMX_TX_XAUI_CTL(iface))) &&
	    !FIELD_GET(OPIP_GMX_RX_XAUI_CTL_STATUS,
		       opip_rd(OPIP_GMX_RX_XAUI_CTL(iface))) &&
	    !(opip_rd(OPIP_PCSX_MISC_CTL(iface)) & OPIP_PCSX_MISC_CTL_GMXENO))
		return 0;
	return opip_xaui_enable(iface, port);
}
