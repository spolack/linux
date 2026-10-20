/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The FPA buffer pools of octeon_pip.
 */
#ifndef OCTEON_PIP_FPA_H
#define OCTEON_PIP_FPA_H

#include <linux/gfp_types.h>
#include <linux/skbuff.h>
#include <linux/types.h>

#include "octeon_pip.h"

/*
 * A packet buffer of the packet pool is a page_pool page: the
 * OPIP_POOL_PACKET_SIZE bytes the hardware may write at its start, the
 * skb_shared_info of the skb built around it on receive at its end. A
 * whole page, not a fragment: one buffer fits a 4 KB page anyway, and a
 * page with one user is released without an atomic operation. A page
 * that holds two buffers or more (8 KB pages and larger) is cut into
 * slices with the page_pool fragment API instead: the same layout in
 * OPIP_RX_SLICE_SIZE bytes, rounded up to the FPA's 128 byte unit so
 * that every slice starts on one (2432 bytes: 3 to an 8 KB page, 6 to
 * a 16 KB one, 26 to a 64 KB one). The skb and the XDP frame hold the
 * whole buffer, so the whole buffer is their size. OPIP_RX_SPLIT is a
 * constant: a 4 KB kernel has the code of one page per buffer only.
 */
#define OPIP_RX_SLICE_SIZE \
	ALIGN(OPIP_POOL_PACKET_SIZE + \
	      SKB_DATA_ALIGN(sizeof(struct skb_shared_info)), 128)
#define OPIP_RX_SPLIT		(PAGE_SIZE >= 2 * OPIP_RX_SLICE_SIZE)
#define OPIP_RX_FRAG_SIZE	(OPIP_RX_SPLIT ? OPIP_RX_SLICE_SIZE : PAGE_SIZE)
static_assert(SKB_DATA_ALIGN(OPIP_POOL_PACKET_SIZE) +
	      SKB_DATA_ALIGN(sizeof(struct skb_shared_info)) <=
	      OPIP_RX_FRAG_SIZE);
static_assert(IS_ALIGNED(OPIP_RX_FRAG_SIZE, 128));

/*
 * The offset in its page of an address in the packet buffer at @start: a
 * whole page starts at its page, a slice need not
 */
static inline unsigned int opip_rx_page_offset(const void *start,
					       const void *addr)
{
	return OPIP_RX_SPLIT ? offset_in_page(addr) : addr - start;
}

void opip_fpa_enable(void);
unsigned int opip_fpa_level(int pool);
unsigned int opip_fpa_fill(struct opip *opip, int pool, unsigned int size,
			   unsigned int count);
void opip_fpa_empty(struct opip *opip, int pool, unsigned int count);
unsigned int opip_fpa_fill_packet_pool(struct page_pool *pp,
				       unsigned int count, gfp_t gfp);
void *opip_fpa_alloc_packet_buf(struct page_pool *pp, gfp_t gfp);
void opip_fpa_put_packet_buf(void *buf);
void opip_fpa_empty_packet_pool(struct opip *opip, unsigned int count);

#endif /* OCTEON_PIP_FPA_H */
