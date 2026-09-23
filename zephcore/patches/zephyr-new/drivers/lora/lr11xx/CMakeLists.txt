# Copyright (c) 2025 ZephCore
# SPDX-License-Identifier: MIT

zephyr_library()

# Suppress deprecated sync-word warning from Semtech SDK
zephyr_library_compile_definitions(LR11XX_DISABLE_WARNINGS)

# LR11xx Zephyr LoRa driver and its Zephyr HAL for the Semtech SDK
zephyr_library_sources(lr11xx_lora.c lr11xx_hal_zephyr.c)

# Semtech lr11xx_driver command layer, compiled unmodified from the LoRa Basics
# Modem module that west already fetches. Only the driver sub-library is used:
# LBM's modem stack and its Kconfig stay off.
set(LR11XX_SDK_DIR
    ${ZEPHYR_LORA_BASICS_MODEM_MODULE_DIR}/lbm_lib/smtc_modem_core/radio_drivers/lr11xx_driver/src)
if(NOT EXISTS ${LR11XX_SDK_DIR}/lr11xx_radio.c)
  message(FATAL_ERROR "LR11xx: Semtech lr11xx_driver not found at ${LR11XX_SDK_DIR} "
                      "(the lora-basics-modem module is missing; run west update)")
endif()
zephyr_library_sources(
  ${LR11XX_SDK_DIR}/lr11xx_radio.c
  ${LR11XX_SDK_DIR}/lr11xx_radio_timings.c
  ${LR11XX_SDK_DIR}/lr11xx_system.c
  ${LR11XX_SDK_DIR}/lr11xx_regmem.c
)
zephyr_library_include_directories(${LR11XX_SDK_DIR})

# Ensure offsets.h is generated before we compile (needed for SPI headers)
zephyr_library_add_dependencies(offsets_h)
