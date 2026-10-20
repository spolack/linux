/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The receive side of octeon_pip: the POW groups and their polls.
 */
#ifndef OCTEON_PIP_RX_H
#define OCTEON_PIP_RX_H

#include <linux/netdevice.h>

#include "octeon_pip.h"

int opip_rx_init(struct opip *opip);
void opip_rx_shutdown(struct opip *opip);
void opip_rx_free_pools(struct opip *opip);
void opip_rx_refill_check(struct opip *opip);
void opip_group_schedule(struct opip_group *group);
void opip_rx_free_work(struct opip_wqe *wqe);

#endif /* OCTEON_PIP_RX_H */
