# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the Cavium network device drivers.
#

obj-$(CONFIG_OCTEON_MGMT_ETHERNET)	+= octeon_mgmt.o

obj-$(CONFIG_OCTEON_PIP_ETHERNET)	+= octeon_pip.o

octeon_pip-y := octeon_pip_main.o
octeon_pip-y += octeon_pip_ethtool.o
octeon_pip-y += octeon_pip_fpa.o
octeon_pip-y += octeon_pip_gmx.o
octeon_pip-y += octeon_pip_ipd.o
octeon_pip-y += octeon_pip_link.o
octeon_pip-y += octeon_pip_pcs.o
octeon_pip-y += octeon_pip_pko.o
octeon_pip-y += octeon_pip_pow.o
octeon_pip-y += octeon_pip_rx.o
octeon_pip-y += octeon_pip_tx.o
