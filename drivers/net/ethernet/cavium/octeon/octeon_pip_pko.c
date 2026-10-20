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
