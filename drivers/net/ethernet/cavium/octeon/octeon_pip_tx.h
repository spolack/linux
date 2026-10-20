/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The transmit side of octeon_pip.
 */
#ifndef OCTEON_PIP_TX_H
#define OCTEON_PIP_TX_H

#include <linux/jump_label.h>
#include <linux/netdevice.h>

#include "octeon_pip.h"

/* Gather list entries a WQE pool buffer holds, for skbs and XDP frames */
#define OPIP_TX_MAX_SEGS	(OPIP_POOL_WQE_SIZE / sizeof(u64))

DECLARE_STATIC_KEY_FALSE(opip_gmx401);

netdev_tx_t opip_xmit(struct sk_buff *skb, struct net_device *dev);
void opip_tx_poll_begin(struct opip_tx_poll *poll, int budget);
void opip_tx_poll_tick(struct opip_tx_poll *poll);
void opip_tx_poll_end(struct opip_tx_poll *poll, bool idle);
void opip_txq_init(struct net_device *dev, unsigned int qid);
void opip_txq_exit(struct opip_txq *q);
void opip_txq_free(struct opip_txq *q);
unsigned int opip_txq_purge(struct opip_txq *q);
bool opip_tx_drain(struct net_device *dev);
void opip_tx_resume(struct net_device *dev);
void opip_tx_stop(struct net_device *dev);
void opip_tx_timeout(struct net_device *dev, unsigned int txqueue);
void opip_tx_netpoll(struct net_device *dev);
netdev_features_t opip_features_check(struct sk_buff *skb,
				      struct net_device *dev,
				      netdev_features_t features);

#endif /* OCTEON_PIP_TX_H */
