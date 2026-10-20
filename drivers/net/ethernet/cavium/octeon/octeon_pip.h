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
#include <linux/hrtimer.h>
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

struct page_pool;
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

/*
 * Packets a transmit queue may hand to the PKO before it is stopped, and
 * the ring that holds them (a power of two with room for the stop margin)
 */
#define OPIP_TXQ_DEPTH		256
#define OPIP_TXQ_RING		(2 * OPIP_TXQ_DEPTH)

/* Ethernet header, FCS and a VLAN tag on top of the MTU */
#define OPIP_MTU_OVERHEAD	(ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN)
#define OPIP_MAX_FRAME		65392

/* ------------------------------------------------------------------ */

/* The receive size histogram of the PIP: 64, 65-127, ... 1024-1518, > 1518 */
#define OPIP_PIP_HIST_BINS	7

/* The counters of the PIP and the PKO, accumulated by the periodic worker */
struct opip_hw_stats {
	/* What came in, before the IPD and RED */
	u64 rx_packets;
	u64 rx_octets;
	/* What the PIP parsed and passed on */
	u64 rx_parsed_packets;
	u64 rx_parsed_octets;
	u64 rx_dropped_packets;
	u64 rx_dropped_octets;
	u64 rx_fcs_align_errors;
	u64 rx_runts;
	u64 rx_runts_fcs;
	u64 rx_oversize;
	u64 rx_oversize_fcs;
	u64 rx_broadcast;
	u64 rx_multicast;
	u64 rx_hist[OPIP_PIP_HIST_BINS];
	u64 tx_packets;
	u64 tx_octets;
};

/* The size histogram of the GMX: < 64, 64, 65-127, ... 1024-1518, > 1518 */
#define OPIP_GMX_HIST_BINS	8

/*
 * The counters of a port's MAC, what the GMX saw on the wire. The driver
 * reads and clears them once a second and keeps the sums here.
 */
struct opip_gmx_stats {
	/* PAUSE and other control frames, not passed on */
	u64 rx_ctl_packets;
	/* Rejected by the address filter */
	u64 rx_dmac_packets;
	/* Dropped by the receive FIFO */
	u64 rx_drop_packets;
	u64 tx_packets;
	u64 tx_octets;
	u64 tx_ctl_packets;
	u64 tx_underflow;
	u64 tx_single_collisions;
	u64 tx_multiple_collisions;
	u64 tx_excessive_collisions;
	u64 tx_excessive_deferrals;
	u64 tx_multicast;
	u64 tx_broadcast;
	u64 tx_hist[OPIP_GMX_HIST_BINS];
};

/*
 * A transmit queue: one PKO output queue of a port, with the chunk of
 * command words the driver writes for it. Everything that touches it runs
 * under the netdev queue lock: xmit, the end and the ticks of a receive
 * poll that sent on it, its transmit NAPI, netpoll (a trylock), the
 * watchdog, and the netpoll list's flush at a close; only the teardown
 * after the PKO reset (opip_txq_purge()) runs without, with nothing else
 * left on the queue. The PKO counts the packets it has sent in the
 * queue's FAU register; the completion compares the count with the ring
 * and frees from the head.
 */
struct opip_txq {
	/* The command chunk being written, and the words used in it */
	u64 *chunk;
	unsigned int index;
	/* PKO output queue number */
	int queue;
	/* The address of the FAU register the PKO counts the sent packets in */
	u64 fau;
	/*
	 * The packets the PKO has, in the order of their commands. The
	 * transmit side adds at the tail; the completions take from the head.
	 * The difference is what is in flight.
	 */
	unsigned int tail;
	unsigned int head;
	/* Command words written but not yet announced by a doorbell */
	unsigned int doorbell;
	/*
	 * Packets written since the counter was last read or the queue's
	 * NAPI was scheduled for it, see opip_tx_complete()
	 */
	unsigned int written;
	/* On the list of a receive poll, see opip_tx_poll_end() */
	bool poll_dirty;
	/*
	 * skbs in flight: the polls complete the queue between their builds
	 * while there are any
	 */
	unsigned int skbs;
	/*
	 * Schedules the NAPI for what is left in flight when nothing else
	 * completes it
	 */
	struct hrtimer timer;
	struct net_device *dev;
	unsigned int qid;
	/* Statistics, see octeon_pip_ethtool.c */
	unsigned long linearized;
	unsigned long no_wqe;
	unsigned long no_chunk;
	unsigned long stopped;
	unsigned long ring[OPIP_TXQ_RING];
	/* Page pool skbs netpoll completed with irqs off, for the NAPI */
	struct sk_buff *netpoll_skbs;
	/*
	 * Completes the queue outside of the receive polls, so that every
	 * completion runs in a NAPI poll, see opip_tx_napi_poll()
	 */
	struct napi_struct napi;
} ____cacheline_aligned_in_smp;

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
	/* The first PKO output queue, and how many the port has */
	int base_queue;
	int num_queues;
	phy_interface_t		phy_mode;
	/* The link as phylink last set it up, for the RGMII preamble check */
	struct opip_link	link;
	unsigned int		last_speed;
	/* Open, and the periodic worker checks for preamble errors */
	bool			preamble_polling;
	/* Pad frames under 64 bytes: CN3XXX, GMX port, half duplex (GMX-401) */
	bool			pad_runts;
	/* RGMII: the ASX clock delay steps, negative for the chip's default */
	s8			rgmii_rx_delay;
	s8			rgmii_tx_delay;
	struct delayed_work	periodic_work;
	struct work_struct	rx_mode_work;
	struct device_node	*of_node;
	spinlock_t		stats_lock;	/* hw_stats, gmx_stats */
	struct opip_hw_stats	hw_stats;
	struct opip_gmx_stats	gmx_stats;
	/* The PKO's counters as last read, under hw_lock */
	u32			pko_packets;
	u64			pko_octets;
	/* Error events of the GMX and the PCS, see opip_poll_errors() */
	unsigned long tx_pko_nxa;
	unsigned long pcs_an_error;
	unsigned long pcs_tx_fifo_error;
	struct opip_txq		txq[OPIP_MAX_TXQS];
	struct phylink		*phylink;
	struct phylink_config	phylink_config;
	/* The PCS of an SGMII port */
	struct phylink_pcs	pcs;
	/*
	 * Frames dropped for a receive error the PIP does not count, see
	 * opip_rx_check_error(): cut at the jabber limit, an L2 length field
	 * that does not match, any other. Several receive groups may count.
	 */
	atomic_long_t		rx_jabber;
	atomic_long_t		rx_l2_length;
	atomic_long_t		rx_other_errors;
};

/* opip_group.state */
#define OPIP_GROUP_IRQ_DISABLED	0

/*
 * The queues a receive poll sent on, and the budget the core gave the
 * poll: what the poll completes goes into the per-CPU NAPI skb cache
 * under it
 */
struct opip_tx_poll {
	int budget;
	unsigned int num;
	struct {
		struct net_device *dev;
		unsigned int qid;
	} dirty[8];
};

/*
 * A POW group: delivers received packets to one CPU. The groups sit next
 * to each other in struct opip: a line of their own each, so that one
 * group's poll writing its tail (to_refill, tx_poll) does not share a
 * line with the next group's head, which that group's interrupt and the
 * kicks write (state, csd).
 */
struct opip_group {
	int irq;
	int group;
	/* The CPU the interrupt is bound to: the poll runs there */
	int cpu;
	/*
	 * Schedules the poll on that CPU from another, see
	 * opip_group_schedule()
	 */
	call_single_data_t csd;
	unsigned long state;
	struct napi_struct napi;
	struct opip *opip;
	/*
	 * The pages this group's poll puts into the packet pool. A buffer
	 * may come back through any group: the pool is the hardware's.
	 */
	struct page_pool *pp;
	/* Buffers this group's poll took out of the pool and has to replace */
	unsigned int to_refill;
	/*
	 * The queues the poll sent on: they follow the poll when NAPI
	 * moves it to another CPU, see opip_tx_poll_end()
	 */
	struct opip_tx_poll tx_poll;
} ____cacheline_aligned_in_smp;

/* The packet I/O complex */
struct opip {
	struct platform_device *pdev;
	/*
	 * The size of the packet pool: the buffers free in the FPA, the ones
	 * the IPD, the POW and the PKO hold, and the ones the polls owe
	 * (to_refill: handed to the stack) add up to this
	 */
	unsigned int num_packet_buffers;
	int num_ifaces;
	struct opip_iface ifaces[OPIP_MAX_IFACES];
	/* The devices, by IPD port */
	struct net_device *ports[OPIP_MAX_PORTS];
	/* The POW groups in use */
	unsigned int groups_mask;
	struct opip_group groups[OPIP_MAX_GROUPS];
	/* The NAPI instances hang off a dummy device: the groups are shared */
	struct net_device *napi_dev;
	/* PKO output queues per port of the GMX interfaces */
	int queues_per_port;
	/*
	 * Serializes the register sequences of process context: link setup,
	 * filters, statistics. Some wait, so it is a mutex.
	 */
	struct mutex hw_lock;
	/*
	 * PKO_REG_READ_IDX selects what PKO_MEM_DEBUG* and PKO_MEM_COUNT* read
	 */
	spinlock_t pko_idx_lock;
	struct delayed_work refill_work;
	/* The driver goes away: the periodic works stop */
	bool stopping;
	/* The hardware was released, by remove or by shutdown */
	bool released;
	/* What was put into the pools, to check what comes back */
	unsigned int wqe_bufs;
	unsigned int cmd_bufs;
	struct dentry *debugfs;
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

/* Only true RGMII ports have the preamble errata; port 0 of GMII is one */
static inline bool opip_port_rgmii_preamble(const struct opip_port *p)
{
	return p->iface->mode == OPIP_MODE_RGMII ||
	       (p->iface->mode == OPIP_MODE_GMII && p->index == 0);
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
void opip_update_hw_stats(struct net_device *dev);
void opip_get_hw_stats(struct net_device *dev, struct opip_hw_stats *hw);
void opip_get_gmx_stats(struct net_device *dev, struct opip_gmx_stats *gmx);

/* octeon_pip_link.c */
int opip_phylink_create(struct opip_port *p);
void opip_check_preamble_errors(struct net_device *dev);
struct opip_link opip_port_link(struct opip_port *p);

/* octeon_pip_ethtool.c */
extern const struct ethtool_ops opip_ethtool_ops;

#endif /* OCTEON_PIP_H */
