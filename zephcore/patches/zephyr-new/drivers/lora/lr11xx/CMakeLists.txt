# Copyright (c) 2025 ZephCore
# SPDX-License-Identifier: MIT

zephyr_library()

# Suppress deprecated sync-word warning from Semtech SDK
zephyr_library_compile_definitions(LR11XX_DISABLE_WARNINGS)

# LR11xx Zephyr LoRa driver
zephyr_library_sources(lr11xx_lora.c)

# Semtech lr11xx_driver SDK (unchanged, lives in app tree)
# APPLICATION_SOURCE_DIR points to the zephcore/ app directory
set(LR11XX_SDK_DIR ${APPLICATION_SOURCE_DIR}/adapters/radio/lr11xx)
zephyr_library_sources(${LR11XX_SDK_DIR}/lr11xx_hal_zephyr.c)
zephyr_library_sources(${LR11XX_SDK_DIR}/lr11xx_radio.c)
zephyr_library_sources(${LR11XX_SDK_DIR}/lr11xx_radio_timings.c)
zephyr_library_sources(${LR11XX_SDK_DIR}/lr11xx_system.c)
zephyr_library_sources(${LR11XX_SDK_DIR}/lr11xx_regmem.c)

zephyr_library_include_directories(${LR11XX_SDK_DIR})

# Ensure offsets.h is generated before we compile (needed for SPI headers)
zephyr_library_add_dependencies(offsets_h)
