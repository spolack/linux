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
#include <net/page_pool/helpers.h>

#include "octeon_pip.h"
#include "octeon_pip_fpa.h"
#include "octeon_pip_regs.h"

/* The pools need 128-byte aligned buffers */
#define OPIP_FPA_ALIGN		128
/* Pages per barrier in opip_fpa_fill_packet_pool() */
#define OPIP_FPA_FILL_BATCH	32

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
 * opip_fpa_level - buffers in a pool right now
 * @pool: Pool to read
 *
 * Return: the number of buffers in @pool
 */
unsigned int opip_fpa_level(int pool)
{
	return FIELD_GET(OPIP_FPA_QUE_AVAILABLE_QUE_SIZ,
			 opip_rd(OPIP_FPA_QUE_AVAILABLE(pool)));
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

/**
 * opip_fpa_fill_packet_pool - put page_pool pages into the packet pool
 * @pp:    page_pool to take them from
 * @count: Buffers to add
 * @gfp:   Allocation flags for new pages
 *
 * The receive poll builds an skb around a buffer the hardware filled; when
 * that skb is freed the page goes back to @pp and comes out here again.
 * A buffer is a whole page, or a slice of one, see OPIP_RX_FRAG_SIZE: the
 * page goes back once all its slices did.
 * One barrier per batch, after the allocations: what the stack wrote into
 * a page before it came back, and what the page allocator wrote into a new
 * one (init_on_alloc zeroes it), is out before the hardware can fill it.
 * That only orders the stores: the free lets the L2 drop all 16 lines
 * without writing them back, the headroom too, so what is before the
 * link word is undefined, even with init_on_alloc, and XDP can read it.
 *
 * Return: the number of buffers added
 */
unsigned int opip_fpa_fill_packet_pool(struct page_pool *pp,
				       unsigned int count, gfp_t gfp)
{
	struct page *pages[OPIP_FPA_FILL_BATCH] __uninitialized;
	/* The slices' offsets: not even on the stack of a 4 KB kernel */
	unsigned int offsets[OPIP_FPA_FILL_BATCH] __uninitialized;
	unsigned int n = 0, i, got;

	while (n < count) {
		unsigned int want = min(count - n, OPIP_FPA_FILL_BATCH);

		for (got = 0; got < want; got++) {
			if (OPIP_RX_SPLIT)
				pages[got] =
					page_pool_alloc_frag(pp, &offsets[got],
							     OPIP_RX_FRAG_SIZE,
							     gfp);
			else
				pages[got] = page_pool_alloc_pages(pp, gfp);
			if (unlikely(!pages[got]))
				break;
		}
		/* The batch's writes out before the IPD may fill the pages */
		wmb();
		for (i = 0; i < got; i++)
			opip_fpa_free_nosync(page_address(pages[i]) +
					     (OPIP_RX_SPLIT ? offsets[i] : 0),
					     OPIP_POOL_PACKET,
					     OPIP_POOL_PACKET_SIZE / 128);
		n += got;
		if (got < want)
			break;
	}
	return n;
}

/**
 * opip_fpa_put_packet_buf - give a packet buffer back to its page_pool
 * @buf: The buffer, not in the packet pool
 *
 * From process context (a ring resize, the teardown): never into the
 * pool's lockless cache. The page goes back when the last of its slices
 * does. Every packet buffer is a page_pool page; one that is not is a
 * bug, which would leak the page quietly through napi_pp_put_page().
 */
void opip_fpa_put_packet_buf(void *buf)
{
	struct page *page = virt_to_head_page(buf);
	netmem_ref netmem = page_to_netmem(page);

	if (WARN_ON_ONCE(!page_pool_page_is_pp(page)))
		return;
	page_pool_put_full_netmem(netmem_get_pp(netmem), netmem, false);
}

/**
 * opip_fpa_empty_packet_pool - return the packet pool's buffers to their pages
 * @opip:  The complex, for messages
 * @count: Buffers that should be in the pool
 */
void opip_fpa_empty_packet_pool(struct opip *opip, unsigned int count)
{
	int left = count;
	void *buf;

	while ((buf = opip_fpa_alloc(OPIP_POOL_PACKET))) {
		opip_fpa_put_packet_buf(buf);
		left--;
	}
	if (left < 0)
		dev_warn(&opip->pdev->dev, "the packet pool returned %d buffers too many\n",
			 -left);
	else if (left > 0)
		dev_warn(&opip->pdev->dev, "the packet pool is missing %d buffers\n",
			 left);
}
