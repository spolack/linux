/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The transmit side of octeon_pip.
 */
#ifndef OCTEON_PIP_TX_H
#define OCTEON_PIP_TX_H

#include <linux/netdevice.h>

netdev_tx_t opip_xmit(struct sk_buff *skb, struct net_device *dev);

#endif /* OCTEON_PIP_TX_H */
