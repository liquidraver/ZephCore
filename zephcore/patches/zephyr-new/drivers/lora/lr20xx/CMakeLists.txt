# Copyright (c) 2025 ZephCore
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_compile_definitions(LR20XX_DISABLE_WARNINGS)
# We run in LDO mode (not DCDC) — disable automatic DCDC workarounds
# that the SDK injects after set_pkt_type, set_rx_path, set_modulation_params.
# Per lr20xx_workarounds.h, these are only needed when REG_MODE_DCDC is used.
zephyr_library_compile_definitions(LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_RESET)
zephyr_library_compile_definitions(LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_CONFIGURE)

zephyr_library_sources(lr20xx_lora.c)

set(LR20XX_SDK_DIR ${APPLICATION_SOURCE_DIR}/adapters/radio/lr20xx)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_hal_zephyr.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_radio_common.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_radio_lora.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_radio_fifo.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_system.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_regmem.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_workarounds.c)
zephyr_library_sources(${LR20XX_SDK_DIR}/lr20xx_driver_version.c)

zephyr_library_include_directories(${LR20XX_SDK_DIR})
zephyr_library_add_dependencies(offsets_h)
