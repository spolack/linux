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
#define OPIP_MAX_IFACES		9
#define OPIP_IFACE_PORTS	16

/* The POW groups the driver can use, one per CPU */
#define OPIP_MAX_GROUPS		16

/* ------------------------------------------------------------------ */

/* An interface: a GMX, NPI or LOOP block with up to 16 ports */
struct opip_iface {
	int num;
	enum opip_mode mode;
	int num_ports;
	/* The IPD/PKO port of index 0 */
	int first_port;
};

/* The packet I/O complex */
struct opip {
	struct platform_device *pdev;
	/* The size of the packet pool */
	unsigned int num_packet_buffers;
	int num_ifaces;
	struct opip_iface ifaces[OPIP_MAX_IFACES];
	/* The POW groups in use */
	unsigned int groups_mask;
	/* The hardware was released, by remove or by shutdown */
	bool released;
	/* What was put into the pools, to check what comes back */
	unsigned int wqe_bufs;
};

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

#endif /* OCTEON_PIP_H */
