/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The FPA buffer pools of octeon_pip.
 */
#ifndef OCTEON_PIP_FPA_H
#define OCTEON_PIP_FPA_H

#include <linux/types.h>

#include "octeon_pip.h"

void opip_fpa_enable(void);
unsigned int opip_fpa_fill(struct opip *opip, int pool, unsigned int size,
			   unsigned int count);
void opip_fpa_empty(struct opip *opip, int pool, unsigned int count);

#endif /* OCTEON_PIP_FPA_H */
