# SPDX-License-Identifier: MIT
# Suppress unique_unit_address warnings for the overlapping nRF52840
# peripherals (power/clock/bprot @ 0x40000000, acl/flash-controller @
# 0x4001e000) — same as the upstream rak4631 board.
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
