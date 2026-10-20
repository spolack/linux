// SPDX-License-Identifier: GPL-2.0
/*
 * The transmit side of octeon_pip.
 */

#include <linux/netdevice.h>

#include "octeon_pip_tx.h"

/**
 * opip_xmit - transmit a packet
 * @skb: Packet to send
 * @dev: Port
 *
 * Until the transmit path is in place, every packet is dropped.
 *
 * Return: NETDEV_TX_OK, always
 */
netdev_tx_t opip_xmit(struct sk_buff *skb, struct net_device *dev)
{
	dev_core_stats_tx_dropped_inc(dev);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}
