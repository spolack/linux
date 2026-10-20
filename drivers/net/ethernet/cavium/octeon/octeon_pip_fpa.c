// SPDX-License-Identifier: GPL-2.0
/*
 * The FPA buffer pools of octeon_pip: the packet pool, filled from the
 * page pools of the receive groups, and the pools of work queue entries
 * and PKO command chunks, filled from the slab.
 */

#include <linux/kmemleak.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_regs.h"

/* The pools need 128-byte aligned buffers */
#define OPIP_FPA_ALIGN		128

/**
 * opip_fpa_enable - enable the FPA unit
 *
 * The bootloader leaves the unit enabled but may have turned requests
 * off when it was done with it: write the whole register, as the
 * executive did, so only the enable bit is set.
 */
void opip_fpa_enable(void)
{
	opip_wr(OPIP_FPA_CTL_STATUS, OPIP_FPA_CTL_STATUS_ENB);
}

/**
 * opip_fpa_fill - put slab buffers into a pool
 * @opip:  The complex, for messages
 * @pool:  Pool to fill
 * @size:  Buffer size
 * @count: Buffers to add
 *
 * kmalloc() aligns a power-of-two size to itself, which is all the
 * alignment the pools need.
 *
 * Return: the number of buffers added
 */
unsigned int opip_fpa_fill(struct opip *opip, int pool, unsigned int size,
			   unsigned int count)
{
	unsigned int n;

	BUILD_BUG_ON(!is_power_of_2(OPIP_POOL_WQE_SIZE) ||
		     OPIP_POOL_WQE_SIZE < OPIP_FPA_ALIGN);
	BUILD_BUG_ON(!is_power_of_2(OPIP_POOL_CMD_SIZE) ||
		     OPIP_POOL_CMD_SIZE < OPIP_FPA_ALIGN);

	for (n = 0; n < count; n++) {
		void *buf = kmalloc(size, GFP_KERNEL);

		if (!buf)
			break;
		/* Only the pool holds it, by physical address */
		kmemleak_ignore(buf);
		opip_fpa_free(buf, pool, 0);
	}
	if (n < count)
		dev_warn(&opip->pdev->dev, "pool %d: could add only %u of %u buffers\n",
			 pool, n, count);
	return n;
}

/**
 * opip_fpa_empty - take the slab buffers out of a pool and free them
 * @opip:  The complex, for messages
 * @pool:  Pool to empty
 * @count: Buffers that should be in it
 */
void opip_fpa_empty(struct opip *opip, int pool, unsigned int count)
{
	int left = count;
	void *buf;

	while ((buf = opip_fpa_alloc(pool))) {
		kfree(phys_to_virt(opip_phys(buf)));
		left--;
	}
	if (left < 0)
		dev_warn(&opip->pdev->dev, "pool %d returned %d buffers too many\n",
			 pool, -left);
	else if (left > 0)
		dev_warn(&opip->pdev->dev, "pool %d is missing %d buffers\n",
			 pool, left);
}
