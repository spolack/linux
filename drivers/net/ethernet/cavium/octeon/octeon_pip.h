/* SPDX-License-Identifier: GPL-2.0 */
/*
 * octeon_pip: the Ethernet driver of the OCTEON packet I/O blocks (PIP/IPD,
 * POW, PKO, FPA and the GMX MACs with their PCS).
 *
 * The objects: one struct opip for the complex, one struct opip_iface per
 * interface the bootloader strapped, one struct opip_port (the netdev's
 * private data) per port of an interface, one struct opip_group per POW
 * group that delivers received packets, one struct opip_txq per transmit
 * queue of a port.
 */
#ifndef OCTEON_PIP_H
#define OCTEON_PIP_H

#include <linux/atomic.h>
#include <linux/cache.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "octeon_pip_hw.h"

struct platform_device;

/* ------------------------------------------------------------------ */
/* The hardware layout the driver chooses */

/* The FPA pools: packet buffers, work queue entries, PKO command chunks */
#define OPIP_POOL_PACKET	0
#define OPIP_POOL_PACKET_SIZE	2048
#define OPIP_POOL_WQE		1
#define OPIP_POOL_WQE_SIZE	128
#define OPIP_POOL_CMD		2
#define OPIP_POOL_CMD_SIZE	1024

/*
 * The packet pool is the receive ring of all ports together (ethtool -g):
 * this many buffers for each MAC port by default, at least a thousand,
 * and at most what a page_pool recycles. RED starts to drop below a
 * quarter of it free. 128 a port gives the 1024 of an eight port chip
 * that the forwarding rates were measured with; whether a larger pool
 * helps a board is a matter of one ethtool -G.
 */
#define OPIP_RX_RING_PER_PORT	128
#define OPIP_DEFAULT_PACKET_BUFFERS	1024
#define OPIP_MAX_PACKET_BUFFERS	16384

/*
 * The IPD skips this much of the first buffer of a packet: room for the
 * link pointer the IPD writes just before the data, and headroom for the
 * stack and XDP. Later buffers of a packet start with the link pointer.
 */
#define OPIP_FIRST_SKIP		256
#define OPIP_LATER_SKIP		0

/* IPD/PKO ports: 16 per GMX interface, then the NPI and LOOP ports */
#define OPIP_MAX_PORTS		48
#define OPIP_MAX_IFACES		9
#define OPIP_IFACE_PORTS	16

/* The POW groups the driver can use, one per CPU */
#define OPIP_MAX_GROUPS		16

/* PKO output queues: the most a port can have */
#define OPIP_MAX_TXQS		16

/* Ethernet header, FCS and a VLAN tag on top of the MTU */
#define OPIP_MTU_OVERHEAD	(ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN)
#define OPIP_MAX_FRAME		65392

/* ------------------------------------------------------------------ */

/* An interface: a GMX, NPI or LOOP block with up to 16 ports */
struct opip_iface {
	int num;
	enum opip_mode mode;
	int num_ports;
	/* The IPD/PKO port of index 0 */
	int first_port;
	/* An SGMII interface running QSGMII (CN70XX) */
	bool qsgmii;
};

/* A port, the private data of its net_device */
struct opip_port {
	struct opip *opip;
	struct opip_iface *iface;
	struct net_device *netdev;
	/* IPD/PKO port number, and the index on the interface */
	int port;
	int index;
	phy_interface_t		phy_mode;
	struct work_struct	rx_mode_work;
	struct device_node	*of_node;
	struct phylink		*phylink;
	struct phylink_config	phylink_config;
	/* The PCS of an SGMII port */
	struct phylink_pcs	pcs;
};

/* The packet I/O complex */
struct opip {
	struct platform_device *pdev;
	/* The size of the packet pool */
	unsigned int num_packet_buffers;
	int num_ifaces;
	struct opip_iface ifaces[OPIP_MAX_IFACES];
	/* The devices, by IPD port */
	struct net_device *ports[OPIP_MAX_PORTS];
	/* The POW groups in use */
	unsigned int groups_mask;
	/* PKO output queues per port of the GMX interfaces */
	int queues_per_port;
	/*
	 * Serializes the register sequences of process context: link setup,
	 * filters, statistics. Some wait, so it is a mutex.
	 */
	struct mutex hw_lock;
	/* The hardware was released, by remove or by shutdown */
	bool released;
	/* What was put into the pools, to check what comes back */
	unsigned int wqe_bufs;
	unsigned int cmd_bufs;
};

static inline struct opip_port *opip_port(struct net_device *dev)
{
	return netdev_priv(dev);
}

/* The IPD/PKO port of a port on an interface */
static inline int opip_port_number(int iface, int index)
{
	static const u8 base[] = { 0, 16, 32, 36, 40, 44 };

	if (iface >= ARRAY_SIZE(base))
		return -1;
	return base[iface] + index;
}

/* A port of a GMX block: RGMII, GMII, (Q)SGMII, PICMG or XAUI */
static inline bool opip_iface_is_gmx(const struct opip_iface *iface)
{
	switch (iface->mode) {
	case OPIP_MODE_RGMII:
	case OPIP_MODE_GMII:
	case OPIP_MODE_SGMII:
	case OPIP_MODE_PICMG:
	case OPIP_MODE_XAUI:
		return true;
	default:
		return false;
	}
}

/*
 * The receive filters and frame limits are set up on interfaces 0 and 1:
 * the further GMX blocks of CN68XX are not checked.
 */
static inline bool opip_port_has_gmx(const struct opip_port *p)
{
	return p->iface->num < 2 && opip_iface_is_gmx(p->iface);
}

/* A port with a PCS: SGMII, QSGMII, or PICMG, which is SGMII to a backplane */
static inline bool opip_port_has_pcs(const struct opip_port *p)
{
	return p->iface->mode == OPIP_MODE_SGMII ||
	       p->iface->mode == OPIP_MODE_PICMG;
}

/* octeon_pip_main.c */
int opip_link_set(struct opip_port *p, const struct opip_link *link);
int opip_pcs_setup(struct opip_port *p, bool inband);
void opip_pcs_restart_an(struct opip_port *p);
void opip_port_enable(struct opip_port *p, bool en);

/* octeon_pip_link.c */
int opip_phylink_create(struct opip_port *p);

#endif /* OCTEON_PIP_H */
