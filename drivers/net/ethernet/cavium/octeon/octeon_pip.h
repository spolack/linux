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

/* IPD/PKO ports: 16 per GMX interface, then the NPI and LOOP ports */
#define OPIP_MAX_IFACES		9
#define OPIP_IFACE_PORTS	16

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
	int num_ifaces;
	struct opip_iface ifaces[OPIP_MAX_IFACES];
};

/* The IPD/PKO port of a port on an interface */
static inline int opip_port_number(int iface, int index)
{
	static const u8 base[] = { 0, 16, 32, 36, 40, 44 };

	if (iface >= ARRAY_SIZE(base))
		return -1;
	return base[iface] + index;
}

#endif /* OCTEON_PIP_H */
