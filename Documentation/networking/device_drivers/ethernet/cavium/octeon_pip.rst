.. SPDX-License-Identifier: GPL-2.0

==================================
Cavium OCTEON PIP Ethernet driver
==================================

``octeon_pip`` (``CONFIG_OCTEON_PIP_ETHERNET``) drives the Ethernet
ports of the packet input and output units (PIP/IPD, POW, PKO, FPA and
the GMX MACs) of the Cavium OCTEON CN3XXX, CN5XXX, CN6XXX, CNF71XX,
CN70XX and CN71XX SoCs. The device tree binding is
``Documentation/devicetree/bindings/net/cavium,octeon-3860-pip.yaml``.
It binds the same nodes as the staging ``octeon-ethernet`` driver; on
OCTEON hardware only one of the two can be built.

Hardware and test status
========================

================================= ===========================================
Tested                            CN7030 (Ubiquiti EdgeRouter 4): QSGMII
                                  through a VSC8504 PHY, the loopback ports;
                                  big endian, and little endian with an
                                  earlier version of the driver
Compile-tested only               the other chips and modes: RGMII and GMII
                                  (CN3XXX, CN5XXX), SGMII and 1000BASE-X
                                  (CN52XX/CN56XX, OCTEON II), XAUI, the NPI
                                  ports, CN68XX
Not supported                     the SPI-4.2 interfaces of CN38XX/CN58XX
                                  (left alone), HiGig2 (an interface the
                                  boot firmware left in HiGig2 mode gets no
                                  ports), the ``pow%d`` device of the
                                  staging driver
================================= ===========================================

The interface modes are what the chip's straps and serdes configuration
set; the driver reads them and does not change them. Reports from other
boards are welcome.

Ports and their names
=====================

Every port of an interface gets a network device. Under an interface
node of the device tree only the ports with a node of their own (and
not ``status = "disabled"``) get one; interfaces without a node (the
NPI and loopback ports of most device trees) get all their ports.

The names are the kernel's templates, as the staging driver has used
them since 2009: ``npi%d`` for the PCI packet interface, ``xaui%d`` for
a XAUI port, ``loop%d`` for the loopback ports and ``eth%d`` for all
other MAC ports. A ``label`` in the device tree does not name the port.
For stable names, match in udev on the port's device tree node (the
network device's ``of_node`` link in sysfs) or on ``dev_port``, the
port's IPD/PKO port number, which is unique under the PIP device (0 to
3 on the first interface, 16 to 19 on the second, 32 to 35 for the NPI
ports, 36 to 39 for the loopback ports).

Receive buffers: ``ethtool -g`` / ``-G``
========================================

All ports share one receive ring: the hardware's packet buffer pool,
from which the input unit takes a buffer for every packet of every
port. ``ethtool -g`` shows the same value on every port and
``ethtool -G <dev> rx N`` resizes it for all of them:

- The default is 128 buffers per MAC port of the chip, at least 1024;
  the range is 256 to 16384. The transmit ring size is fixed (256
  packets per queue).
- The input unit starts dropping (RED) when less than a quarter of the
  pool is free, and drops everything below an eighth.
- A grow is all or nothing: on ``-ENOMEM`` the size stays as it was.
- A shrink does not fail and does not wait. The free buffers go back at
  once, down to the new RED pass mark; the buffers in flight under
  load are returned as the receive polls replace fewer of the buffers
  the stack consumes. ``ethtool -g`` shows the configured size at once,
  the memory follows over the next polls.

With 4 KB pages every received packet is charged a whole page
(``truesize`` 4096), as on every driver with one buffer per page: a
UDP socket at the default ``net.core.rmem_default`` queues about 50
packets. Kernels with larger pages (8, 16, 32 or 64 KB) cut the pages
into slices of 2432 bytes (3, 6, 13 or 26 per page), and a packet is
charged its slice.

Flow control: ``ethtool -A``
============================

The driver uses phylink's default: pause is advertised symmetric and
asymmetric and used as negotiated, so against a partner that
negotiates pause the port both obeys and sends PAUSE frames (the
staging driver never sent any). The port sends PAUSE frames when the
input unit holds more than a quarter of the pool for it, counted in
units of 256 buffers and at least 256: with a pool of fewer than 342
buffers RED drops before a PAUSE frame is sent.

For a router, turn transmit pause off, so that an overload is dropped
by RED instead of pausing the upstream link and every flow on it::

  ethtool -A <dev> autoneg off rx on tx off

``tx_pause_frames`` in ``ethtool -I -a`` then stays 0. Pause is offered
on the SGMII, 1000BASE-X and QSGMII ports of the chips other than
CN68XX; not on RGMII, GMII and XAUI ports.

Statistics
==========

The hardware counters are read once a second; the values in
``ip -s link`` move at that rate. The standard ``ethtool -S`` groups
read the hardware on the spot; the driver's own counters are software
counts.

========================= ==================================================
``ip -s -s link``         ``rx_packets``/``rx_bytes``: what the input unit
                          passed on; ``rx_missed_errors``: RED and input
                          drops; ``rx_crc_errors``: FCS and alignment
                          errors; ``rx_length_errors``: runts, oversize
                          frames, frames cut at the jabber limit and L2
                          length mismatches; ``rx_errors`` also counts
                          the other receive errors the driver drops;
                          ``rx_fifo_errors``: frames the MAC's
                          receive FIFO dropped; ``rx_otherhost_dropped``:
                          frames the MAC's address filter rejected;
                          ``tx_packets``/``tx_bytes``: the output unit's
                          counters; ``tx_fifo_errors``: MAC underflows;
                          ``collisions``, ``tx_aborted_errors``: half
                          duplex
``ethtool -S``            ``tx_linearized``, ``tx_no_wqe`` and
                          ``tx_no_chunk`` (no gather list or command
                          buffer; 0 unless memory ran out), and
                          ``tx_pko_nxa``, ``pcs_an_error``,
                          ``pcs_tx_fifo_error``: error events, the
                          times the once-a-second check found the
                          error bit set, not frames;
                          ``rx_other_errors``: frames dropped for a
                          receive error no other counter has
``ethtool -S --groups``   ``eth-mac``, ``eth-ctrl``, ``rmon`` (the
                          receive and transmit size histograms) and
                          ``pause`` (``ethtool -I -a``; only
                          ``tx_pause_frames``: the MAC counts every
                          control frame it receives, which is
                          ``MACControlFramesReceived`` in ``eth-ctrl``)
queue stats (netlink)     how often each transmit queue was stopped
========================= ==================================================

On CN68XX the input counters are read from the per-port-kind
registers. The PHY's counters are in ``ethtool --phy-statistics``.

The status file
===============

``/sys/kernel/debug/octeon_pip/<device>/status``, ``<device>`` being
the name of the PIP's platform device (for example
``11800a0000000.pip``), shows levels, not counters, for a bug report:

=============================== ============================================
``fpa packet buffers``          free receive buffers; the rest are in flight
``fpa wqe buffers``             free work queue entries
``fpa output buffers``          free transmit command buffers: a fixed value
                                when idle, lower means a buffer the output
                                unit did not return
``ipd packet buffers``          the input unit's own count of the pool and
                                its difference to the pool, which must be 0
``pow queued``                  received packets no CPU has taken yet, per
                                POW group and per QOS level, and the
                                pending group interrupts
``pko error``                   ``PKO_REG_ERROR``; must be 0
``gmxN tx_ovr_bp ...``          the MAC's backpressure overrides and receive
                                FIFO state
``<dev> port N inflight``       packets handed to the output unit, per queue
``<dev> port N pko words``      command words the output unit has not
                                fetched or not executed, per queue
``<dev> port N ipd buffers``    buffers holding the port's received packets,
                                and whether the port asserts backpressure
``<dev> port N pcs``            SGMII, QSGMII and 1000BASE-X ports: the
                                interface mode, the PCS in band or out of
                                band, its control word, out of band on
                                SGMII and QSGMII its link status, and its
                                ``PCS_MISC_CTL`` (MAC or PHY side, the MAC
                                held off, the sample point)
``<dev> port N pcs an``         in band only: the last autonegotiation's
                                result and the word the partner sent
=============================== ============================================

A healthy idle system has ``difference 0``, ``pow queued 0``,
``pko error 0x0`` and zeros in the port lines.

XDP
===

The driver supports ``XDP_PASS``, ``XDP_DROP``, ``XDP_TX``,
``XDP_REDIRECT`` and ``ndo_xdp_xmit`` (``NETDEV_XDP_ACT_BASIC``,
``REDIRECT``, ``NDO_XMIT``) and multi-buffer programs
(``NETDEV_XDP_ACT_RX_SG``):

- A packet in one receive buffer fits an MTU of up to 1762. A program
  that handles multi-buffer packets (``xdp.frags``) sees the larger
  ones as an ``xdp_buff`` with frags, up to an MTU of 32362: 16
  buffers, what ``XDP_TX`` sends, so that every verdict works for
  every packet. The MTU is checked when a program is attached and when
  the MTU changes.
- A packet the program cannot see (over several buffers for a program
  without frags, or in more buffers than an ``xdp_buff`` has frags) is
  dropped while a program is attached.
- ``XDP_TX`` sends packets of up to 16 buffers, every packet within
  the MTU limit. ``ndo_xdp_xmit`` does not take frames with frags
  (``NETDEV_XDP_ACT_NDO_XMIT_SG`` is not set).
- The headroom an XDP program can move into holds undefined data.

Limitations
===========

- No hardware timestamping; software timestamps only.
- No TSO. Offloaded are the receive checksum and hash
  (``rx-checksum``, ``rxhash``, both on), scatter-gather and frag-list
  transmit, and the TCP/UDP transmit checksum of IPv4 without options
  and IPv6 without extension headers.
- A transmit queue the stack's watchdog finds stopped is reported, with
  the output unit's state for the queue, and is not recovered: the
  output unit may still hold its last packet, and freeing it would let
  the completion count run ahead of the packets sent. The queue goes on
  once the output unit sends (``tx_timeout`` of the queue in sysfs
  counts the reports).
- A port runs on the POW groups of all CPUs: received packets are
  spread over up to eight groups (a power of two not above the number
  of online CPUs) by the hardware's flow hash, which is not keyed.
- The receive polls run on group NAPIs that serve all ports, on a
  device of their own with software interrupt coalescing
  (``gro_flush_timeout`` 20 us, ``napi_defer_hard_irqs`` 1) that user
  space cannot change.

Hardware notes
==============

Facts about the hardware that explain behaviour visible from user space:

- The input unit counts the receive pool itself
  (``IPD_QUE0_FREE_PAGE_CNT``) and RED decides on that count. A reset
  of the input unit zeroes the count whatever the pool holds, so the
  driver resets it only with the pool empty; the ``difference`` in the
  status file shows that the two agree.
- The output unit fetches commands in 64 byte units, and a command that
  crosses a unit can be assembled from stale words, which stalls the
  port with ``PKO_REG_ERROR[CURRZERO]``. The driver writes only two
  word commands from the start of a 128 byte aligned buffer, which
  never cross a unit, and counts sent packets in a FAU register instead
  of asking for completion work.
- The output unit raises no event for a sent packet: transmit
  completions come from the receive polls, from a per-queue timer that
  schedules the queue's NAPI, and from the transmit itself once a queue
  is half full (the NAPI waits for any other NAPI on its CPU, such as
  the RPS backlog).
- The PCS of an SGMII or QSGMII port and the PHY must agree on the
  in-band autonegotiation: a PCS that waits for an autonegotiation the
  PHY does not send receives nothing, and the other way round. The PCS
  runs in band by default, with a PHY too, as the vendor SDK does; phylink
  resolves the mode from the capabilities the PCS and the PHY driver
  report, and runs a PHY that cannot do in band, and a fixed link, out of
  band. A PHY driver that reports nothing leaves the PHY at its own
  setting, which must then be in band. On 1000BASE-X the autonegotiation
  can also be turned off with ethtool.
