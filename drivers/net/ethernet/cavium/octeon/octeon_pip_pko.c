// SPDX-License-Identifier: GPL-2.0
/*
 * The PKO block: packet output. The queues of a port read commands from
 * chains of chunks the driver writes, the ports feed the MACs.
 *
 * Derived in part from the Cavium executive (arch/mips/cavium-octeon/
 * executive/cvmx-pko.c),
 * Copyright (c) 2003-2018 Cavium, Inc. / Cavium Networks.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "octeon_pip.h"
#include "octeon_pip_pko.h"

/* The port number that marks a queue as unused */
#define OPIP_PKO_ILLEGAL_PORT	63

/**
 * opip_pko_layout - lay out the output queues over the ports
 * @opip:            The complex, its interfaces 0 and 1 enumerated
 * @queues_per_port: Queues wanted for each port of interfaces 0 and 1
 *
 * The queues are assigned statically: the ports of interface 0, then
 * those of interface 1, each with the same number of queues, then the
 * NPI and LOOP ports with one queue each. As many queues per port as fit
 * the chip's layout limit (opip_chip->pko_layout_queues), at most
 * @queues_per_port and at most OPIP_MAX_TXQS. CN3XXX has no
 * PKO_REG_QUEUE_PTRS1[IDX3], so at most 8 queues per port. CN68XX has
 * one queue per port by its port tables.
 */
void opip_pko_layout(struct opip *opip, int queues_per_port)
{
	int limit = opip_chip->pko_layout_queues;
	int ports, fixed, max = OPIP_MAX_TXQS;

	if (opip_chip->cn68xx) {
		opip->queues_per_port = 1;
		return;
	}

	ports = opip->ifaces[0].num_ports + opip->ifaces[1].num_ports;
	fixed = 4 + 4;	/* NPI and LOOP, one queue each */
	if (ports)
		max = (limit - fixed) / ports;
	if (opip_chip->cn3xxx)
		max = min(max, 8);
	opip->queues_per_port = clamp(queues_per_port, 1,
				      min(max, OPIP_MAX_TXQS));
}

/**
 * opip_pko_num_queues - the output queues of a port
 * @opip: The complex, its queues laid out
 * @port: PKO port
 *
 * Return: the queues per port of opip_pko_layout() for a port of
 * interfaces 0 and 1, one for the NPI and LOOP ports (32 to 39) and for
 * every port of CN68XX, 0 for any other port
 */
int opip_pko_num_queues(const struct opip *opip, int port)
{
	if (opip_chip->cn68xx)
		return 1;
	if (port < 2 * OPIP_IFACE_PORTS)
		return opip->queues_per_port;
	if (port < 40)
		return 1;
	return 0;
}

/**
 * opip_pko_base_queue - the first output queue of a port
 * @opip: The complex, its queues laid out
 * @port: PKO port
 *
 * In the order of opip_pko_layout(): the ports of interface 0, those of
 * interface 1, then the NPI and LOOP ports. On CN68XX the queue is the
 * port number.
 *
 * Return: the queue number, or -1 for a port without queues
 */
int opip_pko_base_queue(const struct opip *opip, int port)
{
	int p0 = opip->ifaces[0].num_ports, p1 = opip->ifaces[1].num_ports;
	int qpp = opip->queues_per_port;

	if (opip_chip->cn68xx)
		return port;
	if (port < p0)
		return port * qpp;
	if (port >= OPIP_IFACE_PORTS && port < OPIP_IFACE_PORTS + p1)
		return (p0 + port - OPIP_IFACE_PORTS) * qpp;
	if (port >= 32 && port < 40)
		return (p0 + p1) * qpp + port - 32;
	return -1;
}

/**
 * opip_pko_port_valid - whether a port has queues that fit the chip
 * @opip: The complex, its queues laid out
 * @port: PKO port
 *
 * Nothing may be sent on a port that is not valid.
 *
 * Return: true if @port has queues and they are within the chip's
 */
bool opip_pko_port_valid(const struct opip *opip, int port)
{
	int base = opip_pko_base_queue(opip, port);
	int num = opip_pko_num_queues(opip, port);

	if (opip_chip->cn68xx)
		return port < OPIP_MAX_PORTS;
	return port < 40 && num > 0 && base >= 0 &&
	       base + num <= opip_chip->pko_queues;
}

/* PKO_MEM_QUEUE_PTRS and the high bits that moved to PKO_REG_QUEUE_PTRS1 */
static void opip_pko_write_queue_ptrs(int queue, int index, u64 ptrs)
{
	if (!opip_chip->cn3xxx)
		opip_wr(OPIP_PKO_REG_QUEUE_PTRS1,
			(index >> 3 ? OPIP_PKO_REG_QUEUE_PTRS1_IDX3 : 0) |
			(queue >> 7 ? OPIP_PKO_REG_QUEUE_PTRS1_QID7 : 0));
	opip_wr(OPIP_PKO_MEM_QUEUE_PTRS, ptrs |
		FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_INDEX, index & 7) |
		FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_QUEUE, queue & 0x7f));
}

/* A queue nobody uses */
static void opip_pko_queue_unused(int queue)
{
	opip_pko_write_queue_ptrs(queue, 0,
				  OPIP_PKO_MEM_QUEUE_PTRS_TAIL |
				  FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_QOS_MASK,
					     0xff) |
				  FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_PORT,
					     OPIP_PKO_ILLEGAL_PORT));
}

/* The PKO's interrupt/engine number of a CN68XX port */
static int opip_pko_cn68_engine(int iface, int index)
{
	static const u8 base[] = { 0, 4, 8, 12, 16, 0x1c, 0x1d, 0x1e, 0x1f };

	if (iface >= ARRAY_SIZE(base))
		return 31;
	return iface == 1 || iface >= 5 ? base[iface] : base[iface] + index;
}

/*
 * CN68XX: every internal port maps to an engine, the invalid ones to 31.
 * The PKO adds padding and FCS.
 */
static void opip_pko_cn68_port_map(struct opip *opip)
{
	int port;

	for (port = 0; port < 128; port++)
		opip_wr(OPIP_PKO_MEM_IPORT_PTRS,
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_EID, 31) |
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_IPID, port));

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		int iface = port < 32 ? port >> 4 : 2 + (port - 32) / 4;
		int index = port < 32 ? port & 0xf : (port - 32) & 3;
		struct opip_iface *ifc;
		int engine;

		if (iface >= opip->num_ifaces)
			continue;
		ifc = &opip->ifaces[iface];
		if (ifc->mode == OPIP_MODE_DISABLED)
			continue;
		engine = opip_pko_cn68_engine(iface, index);
		opip_wr(OPIP_PKO_MEM_IPORT_PTRS,
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_IPID, port) |
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_QOS_MASK, 0xff) |
			OPIP_PKO_MEM_IPORT_PTRS_CRC |
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_MIN_PKT, 1) |
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_INTR, engine) |
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_EID, engine) |
			FIELD_PREP(OPIP_PKO_MEM_IPORT_PTRS_PIPE,
				   ifc->mode == OPIP_MODE_LOOP ? index : port));
	}
}

/*
 * The PKO holds the command words it has fetched for a queue in a local
 * storage that PKO_REG_QUEUE_MODE sizes. At the reset mode every queue
 * gets 8 words, and a three word command then has to be taken out of a
 * 64 byte fetch unit in two pieces, which the PKO was seen to get wrong
 * (CN7030: PKO_REG_ERROR[CURRZERO], the port dead). The queues in use get
 * the most storage their count allows.
 */
static void opip_pko_set_queue_mode(struct opip *opip)
{
	int max_queues = 0, n = opip_chip->pko_mode_queues;
	int port, mode;

	if (!n)
		return;

	for (port = 0; port < OPIP_MAX_PORTS; port++) {
		int base = opip_pko_base_queue(opip, port);
		int end;

		end = base + opip_pko_num_queues(opip, port);
		if (base >= 0 && opip_pko_port_valid(opip, port) &&
		    end > max_queues)
			max_queues = end;
	}

	mode = max_queues <= n ? 2 : max_queues <= 2 * n ? 1 : 0;
	opip_wr(OPIP_PKO_REG_QUEUE_MODE,
		FIELD_PREP(OPIP_PKO_REG_QUEUE_MODE_MODE, mode));
}

/**
 * opip_pko_init - the global PKO setup, with the PKO disabled
 * @opip: The complex, its interfaces enumerated and queues laid out
 *
 * The command chunk pool and size (an odd number of words, so that two
 * word commands never straddle chunks), every queue unused until a port
 * takes it, the queue storage mode, no FAU timeout, and the minimum
 * packet size of CN68XX.
 */
void opip_pko_init(struct opip *opip)
{
	int queue;

	opip_wr(OPIP_PKO_REG_CMD_BUF,
		FIELD_PREP(OPIP_PKO_REG_CMD_BUF_POOL, OPIP_POOL_CMD) |
		FIELD_PREP(OPIP_PKO_REG_CMD_BUF_SIZE,
			   OPIP_POOL_CMD_SIZE / 8 - 1));

	if (opip_chip->cn68xx) {
		opip_pko_cn68_port_map(opip);
	} else {
		for (queue = 0; queue < opip_chip->pko_queues; queue++)
			opip_pko_queue_unused(queue);
		opip_pko_set_queue_mode(opip);
	}

	opip_wr(OPIP_IOB_FAU_TIMEOUT,
		FIELD_PREP(OPIP_IOB_FAU_TIMEOUT_TOUT_VAL, 0xfff));

	if (opip_chip->cn68xx) {
		u64 min_pkt = 0;
		int i;

		/* Sizes 1 to 7: 59 bytes, the minimum frame less the FCS */
		for (i = 1; i < 8; i++)
			min_pkt |= 59ULL << (i * 8);
		opip_wr(OPIP_PKO_REG_MIN_PKT, min_pkt);
	}
}

/**
 * opip_pko_queue_init - give a port's queue its first chunk and tell the PKO
 * @opip:       The complex
 * @txq:        The transmit queue, its queue number set
 * @port:       PKO port
 * @index:      Index of the queue among the port's
 * @num_queues: Queues of the port
 *
 * Every queue gets the same share of the port (priority 8 of 8), none is
 * static but the single queue of a CN68XX port, as the executive sets it,
 * and its FAU counter is reset. Called with the PKO disabled.
 *
 * Return: 0, or -ENOMEM without a chunk
 */
int opip_pko_queue_init(struct opip *opip, struct opip_txq *txq, int port,
			int index, int num_queues)
{
	u64 *chunk = opip_fpa_alloc(OPIP_POOL_CMD);

	if (!chunk)
		return -ENOMEM;
	txq->chunk = chunk;
	txq->index = 0;
	/* The sent packet count of the queue starts at zero */
	txq->fau = opip_fau_addr(opip_pko_queue_fau(txq->queue), 0);
	opip_fau_wr32(txq->fau | OPIP_FAU_NOADD, 0);
	wmb();	/* the chunk is in memory before the PKO is pointed at it */

	if (opip_chip->cn68xx) {
		opip_wr(OPIP_PKO_MEM_IQUEUE_PTRS,
			FIELD_PREP(OPIP_PKO_MEM_IQUEUE_PTRS_INDEX, index) |
			FIELD_PREP(OPIP_PKO_MEM_IQUEUE_PTRS_QID, txq->queue) |
			FIELD_PREP(OPIP_PKO_MEM_IQUEUE_PTRS_IPID, port) |
			(index == num_queues - 1 ?
			 OPIP_PKO_MEM_IQUEUE_PTRS_TAIL : 0) |
			OPIP_PKO_MEM_IQUEUE_PTRS_STATIC_P |
			OPIP_PKO_MEM_IQUEUE_PTRS_STATIC_Q |
			FIELD_PREP(OPIP_PKO_MEM_IQUEUE_PTRS_QOS_MASK, 0xff) |
			FIELD_PREP(OPIP_PKO_MEM_IQUEUE_PTRS_BUF_PTR,
				   opip_phys(chunk) >> 7));
	} else {
		u64 ptrs = (index == num_queues - 1 ?
			    OPIP_PKO_MEM_QUEUE_PTRS_TAIL : 0) |
			   FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_PORT, port) |
			   FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_QOS_MASK, 0xff) |
			   FIELD_PREP(OPIP_PKO_MEM_QUEUE_PTRS_BUF_PTR,
				      opip_phys(chunk));

		opip_pko_write_queue_ptrs(txq->queue, index, ptrs);
	}
	return 0;
}

/**
 * opip_pko_queue_release - take a queue away from the PKO and free its chunk
 * @opip: The complex
 * @txq:  The transmit queue, the PKO done with it
 *
 * The PKO is stopped, or runs with no command of the queue left: a
 * device that failed to register. A queue without a chunk was never
 * given to the PKO: nothing to do.
 */
void opip_pko_queue_release(struct opip *opip, struct opip_txq *txq)
{
	if (!txq->chunk)
		return;
	if (!opip_chip->cn68xx)
		opip_pko_queue_unused(txq->queue);
	opip_fpa_free(txq->chunk, OPIP_POOL_CMD, 0);
	txq->chunk = NULL;
}

/**
 * opip_pko_write_next - the slow path of opip_pko_write(): the chunk is full
 * @txq: The transmit queue, under its lock
 * @w0:  Command word
 * @w1:  Buffer pointer, or the pointer to a gather list
 *
 * Return: false if no chunk could be had, nothing written
 */
bool opip_pko_write_next(struct opip_txq *txq, u64 w0, u64 w1)
{
	u64 *chunk = opip_fpa_alloc(OPIP_POOL_CMD);

	/* Commands start at even words: a full chunk ends at a command */
	BUILD_BUG_ON(OPIP_PKO_CHUNK_WORDS % 2);
	if (unlikely(!chunk))
		return false;
	/* The word after the commands links to the next chunk */
	txq->chunk[OPIP_PKO_CHUNK_WORDS] = opip_phys(chunk);
	txq->chunk = chunk;
	chunk[0] = w0;
	chunk[1] = w1;
	txq->index = 2;
	return true;
}

/**
 * opip_pko_enable - start the PKO
 *
 * Sets ENA_PKO in PKO_REG_FLAGS, with ENA_DWB and STORE_BE. Nothing to do
 * if the PKO is enabled already.
 */
void opip_pko_enable(void)
{
	u64 flags = opip_rd(OPIP_PKO_REG_FLAGS);

	if (flags & OPIP_PKO_REG_FLAGS_ENA_PKO)
		return;
	/* Big endian storage of the commands, don't write back */
	opip_wr(OPIP_PKO_REG_FLAGS, flags | OPIP_PKO_REG_FLAGS_ENA_PKO |
		OPIP_PKO_REG_FLAGS_ENA_DWB | OPIP_PKO_REG_FLAGS_STORE_BE);
}

/**
 * opip_pko_disable - stop the PKO
 *
 * Clears ENA_PKO in PKO_REG_FLAGS. The PKO stops taking new commands but
 * finishes the packets it is sending.
 */
void opip_pko_disable(void)
{
	opip_wr(OPIP_PKO_REG_FLAGS,
		opip_rd(OPIP_PKO_REG_FLAGS) & ~OPIP_PKO_REG_FLAGS_ENA_PKO);
}

/**
 * opip_pko_reset - reset the PKO
 *
 * Sets RESET in PKO_REG_FLAGS, with the PKO disabled, before the pools
 * are emptied.
 */
void opip_pko_reset(void)
{
	opip_wr(OPIP_PKO_REG_FLAGS,
		opip_rd(OPIP_PKO_REG_FLAGS) | OPIP_PKO_REG_FLAGS_RESET);
}

/**
 * opip_pko_gmx_port_mode - tell the PKO how many ports a GMX interface has
 * @iface:     Interface, 0 or 1
 * @num_ports: Ports on it
 *
 * PKO_REG_GMX_PORT_MODE, which CN30XX, CN31XX and CN50XX do not have.
 */
void opip_pko_gmx_port_mode(int iface, int num_ports)
{
	u64 mask = iface ? OPIP_PKO_REG_GMX_PORT_MODE_MODE1 :
			   OPIP_PKO_REG_GMX_PORT_MODE_MODE0;
	int mode;

	if (opip_chip->small)
		return;

	if (num_ports == 1)
		mode = 4;
	else if (num_ports == 2)
		mode = 3;
	else if (num_ports <= 4)
		mode = 2;
	else if (num_ports <= 8)
		mode = 1;
	else
		mode = 0;
	opip_wr(OPIP_PKO_REG_GMX_PORT_MODE,
		opip_field_set_var(opip_rd(OPIP_PKO_REG_GMX_PORT_MODE), mask,
				   mode));
}
