# SPDX-License-Identifier: MIT
# Zephyr build of the standalone Ascon implementation in this directory (see
# README.md). Only built with CONFIG_ZEPHCORE_ZCPFS, its one user here.

zephyr_library_named(ascon)
zephyr_library_sources(ascon.c)
zephyr_library_include_directories(.)
