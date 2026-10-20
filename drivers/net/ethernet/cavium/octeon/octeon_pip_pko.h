/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The PKO (packet output) block of octeon_pip: the output queues, their
 * command chunks and doorbells.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-pko.c, arch/mips/include/asm/octeon/cvmx-pko.h),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */
#ifndef OCTEON_PIP_PKO_H
#define OCTEON_PIP_PKO_H

#include <linux/types.h>

#include "octeon_pip.h"
#include "octeon_pip_regs.h"

/*
 * The command words of a chunk before its link word. PKO_REG_CMD_BUF[SIZE]
 * counts the link: with SIZE = 127 the PKO reads 126 command words and the
 * link at index 126, and the last word of the 1024 byte buffer is unused.
 * An even count also keeps two word commands from straddling chunks.
 */
#define OPIP_PKO_CHUNK_WORDS	(OPIP_POOL_CMD_SIZE / 8 - 2)

/* The PKO's device ID on the I/O bus, for the doorbell address */
#define OPIP_PKO_DID_SEND	0x52

/*
 * The FAU register of a queue: 32 bits each, register 0 is "none" to the
 * PKO. The PKO subtracts one from it per packet sent, see opip_pko_cmd().
 */
#define OPIP_PKO_FAU_SIZE	4

static inline unsigned int opip_pko_queue_fau(int queue)
{
	return (queue + 1) * OPIP_PKO_FAU_SIZE;
}

/*
 * The I/O address of a FAU register for a 32 bit load that adds @inc or a
 * 32 bit store; the instruction gives the operand size, as in the
 * executive's __cvmx_fau_atomic_address(). A little endian core flips the
 * low address bits of a 32 bit I/O access, so the CPU names the register
 * the PKO sees as @reg by @reg ^ 4.
 */
static inline u64 opip_fau_addr(unsigned int reg, s32 inc)
{
	if (IS_ENABLED(CONFIG_CPU_LITTLE_ENDIAN))
		reg ^= OPIP_PKO_FAU_SIZE;

	return OPIP_CSR(OPIP_POW_LOAD_IS_IO |
			FIELD_PREP(OPIP_POW_LOAD_DID, OPIP_FAU_DID) |
			FIELD_PREP(OPIP_FAU_INEVAL, (u32)inc & 0x3fffff) |
			FIELD_PREP(OPIP_FAU_REG, reg));
}

void opip_pko_layout(struct opip *opip, int queues_per_port);
int opip_pko_base_queue(const struct opip *opip, int port);
int opip_pko_num_queues(const struct opip *opip, int port);
bool opip_pko_port_valid(const struct opip *opip, int port);
void opip_pko_init(struct opip *opip);
int opip_pko_queue_init(struct opip *opip, struct opip_txq *txq, int port,
			int index, int num_queues);
void opip_pko_queue_release(struct opip *opip, struct opip_txq *txq);
void opip_pko_enable(void);
void opip_pko_disable(void);
void opip_pko_reset(void);
unsigned int opip_pko_queue_pending(struct opip *opip, int queue);
u64 opip_pko_error(void);
void opip_pko_read_stats(struct opip *opip, int port, u32 *packets,
			 u64 *octets);
void opip_pko_clear_stats(struct opip *opip, int port);
void opip_pko_gmx_port_mode(int iface, int num_ports);
bool opip_pko_write_next(struct opip_txq *txq, u64 w0, u64 w1);

/**
 * opip_pko_doorbell - tell the PKO about command words written to a queue
 * @port:  PKO port of the queue
 * @queue: PKO output queue
 * @words: Command words written since the last doorbell
 *
 * The words are in DRAM before the doorbell: a write barrier.
 */
static inline void opip_pko_doorbell(int port, int queue, unsigned int words)
{
	u64 addr = OPIP_CSR(OPIP_POW_LOAD_IS_IO |
			    FIELD_PREP(OPIP_POW_LOAD_DID, OPIP_PKO_DID_SEND) |
			    FIELD_PREP(OPIP_PKO_DOORBELL_PORT, port) |
			    FIELD_PREP(OPIP_PKO_DOORBELL_QUEUE, queue));

	wmb();	/* the command words are in memory before the PKO reads them */
	opip_wr(addr, words);
}

/**
 * opip_pko_write - write a two word command for a queue
 * @txq: The transmit queue, under its lock
 * @w0:  Command word
 * @w1:  Buffer pointer, or the pointer to a gather list
 *
 * Normally the command fits the chunk being written. When the chunk is
 * full, the link to a new chunk is written and the command at the start
 * of the new one. The new chunk is taken from the pool only then, so it
 * always holds a command: the PKO follows the link only to read a
 * command, and a chunk that is linked to but empty would leave the old
 * one with the PKO until the next command.
 *
 * Return: false if no chunk could be had, nothing written
 */
static __always_inline bool opip_pko_write(struct opip_txq *txq, u64 w0,
					   u64 w1)
{
	u64 *p = txq->chunk + txq->index;

	if (likely(txq->index < OPIP_PKO_CHUNK_WORDS)) {
		p[0] = w0;
		p[1] = w1;
		txq->index += 2;
		return true;
	}
	return opip_pko_write_next(txq, w0, w1);
}

#endif /* OCTEON_PIP_PKO_H */
