# Copyright (c) 2026 ZephCore
# SPDX-License-Identifier: MIT
#
# Same flashing story as seeed_lr2021_evk: the XIAO's USB-C goes to a SAMD11
# that presents CMSIS-DAP plus a USB CDC for the uart20 console. The nRF54LM20A
# does have a USB peripheral, but this XIAO leaves D+/D- unconnected, so there
# is no UF2 and no DFU.
#
# The openocd config is upstream Zephyr's for the XIAO nRF54LM20A, used in place
# rather than copied: it sources boards/seeed/common/openocd_nrf54l.cfg by a
# path relative to itself.

board_runner_args(openocd "--config=${ZEPHYR_BASE}/boards/seeed/xiao_nrf54lm20a/support/openocd.cfg"
                  "--cmd-load=nrf54l_load" "--cmd-erase=nrf54l_mass_erase"
                  "--cmd-verify=verify_image" -c "targets nrf54lm20a.cpu")
board_runner_args(jlink "--device=nRF54LM20A_M33" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
