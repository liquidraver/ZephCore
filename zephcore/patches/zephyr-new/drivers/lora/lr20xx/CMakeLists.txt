# Copyright (c) 2025 ZephCore
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_compile_definitions(LR20XX_DISABLE_WARNINGS)
# The two LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_* defines that used to live
# here are gone: SDK v2.0.2 deleted lr20xx_workarounds_dcdc_reset/_configure and
# the macros that injected them after set_pkt_type / set_rx_path /
# set_modulation_params. That workaround now lives in the Patch RAM, which this
# driver loads on every reset. Nothing references the defines any more.

zephyr_library_sources(lr20xx_lora.c lr20xx_hal_zephyr.c)

# Semtech lr20xx_driver, vendored unmodified in lr20xx_driver/ (with its
# LICENSE). Not taken from LoRa Basics Modem: that snapshot is older and has no
# PRAM loader (lr20xx_patch.c), which the LR2021 needs.
set(LR20XX_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lr20xx_driver)
zephyr_library_sources(
  ${LR20XX_SDK_DIR}/lr20xx_radio_common.c
  ${LR20XX_SDK_DIR}/lr20xx_radio_lora.c
  ${LR20XX_SDK_DIR}/lr20xx_radio_fifo.c
  ${LR20XX_SDK_DIR}/lr20xx_system.c
  ${LR20XX_SDK_DIR}/lr20xx_regmem.c
  ${LR20XX_SDK_DIR}/lr20xx_workarounds.c
  ${LR20XX_SDK_DIR}/lr20xx_driver_version.c
  ${LR20XX_SDK_DIR}/lr20xx_patch.c
)
zephyr_library_include_directories(${LR20XX_SDK_DIR})

zephyr_library_add_dependencies(offsets_h)
