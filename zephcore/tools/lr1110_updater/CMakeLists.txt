# SPDX-License-Identifier: MIT
# ZephCore LR1110 Firmware Updater — updates LR1110 radio firmware to 0x0402.
#
# Board-specific: only builds for boards with an LR1110 radio
# (t1000_e, thinknode_m9).
#
# Build:  west build -b t1000_e/nrf52840 zephcore/tools/lr1110_updater --pristine
#         west build -b thinknode_m9/esp32s3/procpu zephcore/tools/lr1110_updater --pristine
# Flash:  nRF52: drag-drop build/zephyr/zephyr.uf2 onto UF2 drive
#         (or: nrfjprog --program build/zephyr/zephyr.hex --sectorerase -r)
#         ESP32: west flash --esp-device COMX
#
# After the updater finishes, flash ZephCore main firmware.

cmake_minimum_required(VERSION 3.20.0)

# Reuse the main project's custom board definitions
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Extract base board name (strip qualifier like /nrf52840)
string(REPLACE "/" ";" BOARD_PARTS ${BOARD})
list(GET BOARD_PARTS 0 BOARD_BASE)

# Find board overlay — prefer updater-specific overlay, fall back to main board
file(GLOB_RECURSE UPD_OVERLAY_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.overlay"
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.overlay")
if(UPD_OVERLAY_CANDIDATES)
    list(GET UPD_OVERLAY_CANDIDATES 0 UPDATER_BOARD_OVERLAY)
else()
    # Fall back to main board overlay
    file(GLOB_RECURSE BOARD_OVERLAY_CANDIDATES
         "${CMAKE_CURRENT_SOURCE_DIR}/../../boards/*/${BOARD_BASE}/board.overlay")
    if(BOARD_OVERLAY_CANDIDATES)
        list(GET BOARD_OVERLAY_CANDIDATES 0 UPDATER_BOARD_OVERLAY)
    endif()
endif()
if(UPDATER_BOARD_OVERLAY)
    if(EXTRA_DTC_OVERLAY_FILE)
        set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${UPDATER_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_DTC_OVERLAY_FILE "${UPDATER_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    endif()
endif()

# Find updater-specific board.conf
file(GLOB_RECURSE UPD_BOARD_CONF_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.conf"
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.conf")
if(UPD_BOARD_CONF_CANDIDATES)
    list(GET UPD_BOARD_CONF_CANDIDATES 0 UPD_BOARD_CONF)
    if(EXTRA_CONF_FILE)
        set(EXTRA_CONF_FILE "${EXTRA_CONF_FILE};${UPD_BOARD_CONF}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_CONF_FILE "${UPD_BOARD_CONF}" CACHE STRING "" FORCE)
    endif()
endif()

# Optional: route SPI through DMA so a 262-byte WriteFlashEncrypted frame is
# ONE hardware transaction instead of five 64-byte ones. Diagnostic — see the
# header of the board's spi_dma.overlay.
#   west build ... zephcore/tools/lr1110_updater -- -DUPDATER_SPI_DMA=1
if(UPDATER_SPI_DMA)
    set(UPD_DMA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/esp32/${BOARD_BASE}")
    if(NOT EXISTS "${UPD_DMA_DIR}/spi_dma.overlay")
        message(FATAL_ERROR
            "UPDATER_SPI_DMA=1 requested but no spi_dma.overlay exists for "
            "board '${BOARD_BASE}'. This option is ESP32-only — the nRF52 "
            "SPIM driver has no 64-byte transaction limit to work around.")
    endif()
    set(EXTRA_DTC_OVERLAY_FILE
        "${EXTRA_DTC_OVERLAY_FILE};${UPD_DMA_DIR}/spi_dma.overlay"
        CACHE STRING "" FORCE)
    set(EXTRA_CONF_FILE
        "${EXTRA_CONF_FILE};${UPD_DMA_DIR}/spi_dma.conf"
        CACHE STRING "" FORCE)
    message(STATUS "LR1110 updater: SPI DMA ENABLED (single-transaction writes)")
endif()

message(STATUS "LR1110 updater board overlay: ${UPDATER_BOARD_OVERLAY}")
message(STATUS "LR1110 updater board conf: ${UPD_BOARD_CONF}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore_lr1110_updater)

target_sources(app PRIVATE
    src/main.c
    src/lr11xx_hal_updater.c
    src/lr1110_bootloader.c
    src/lr1110_bl_updater.c
)

# Optional: flash transceiver 0x0401 instead of the default 0x0402.
#   west build ... zephcore/tools/lr1110_updater -- -DUPDATER_TARGET_FW=0x0401
if(DEFINED UPDATER_TARGET_FW)
    target_compile_definitions(app PRIVATE UPDATER_TARGET_FW=${UPDATER_TARGET_FW})
endif()

# Opt in to the ONE-WAY chip-bootloader update (0x6500 -> 0x1001). Off by
# default: chips that took it have since refused to boot any transceiver
# image. See the note in src/main.c.
if(DEFINED UPDATER_ALLOW_BOOTLOADER_UPDATE)
    target_compile_definitions(app PRIVATE
        UPDATER_ALLOW_BOOTLOADER_UPDATE=${UPDATER_ALLOW_BOOTLOADER_UPDATE})
endif()

# Idle gap after every flash page program, in ms. Diagnostic for a marginal
# supply — see the note in src/main.c.
#   west build ... -- -DUPDATER_CHUNK_DELAY_MS=10
if(DEFINED UPDATER_CHUNK_DELAY_MS)
    target_compile_definitions(app PRIVATE
        UPDATER_CHUNK_DELAY_MS=${UPDATER_CHUNK_DELAY_MS})
endif()

# Flash even with an SD card in the slot (shared SPI bus — see src/main.c).
#   west build ... -- -DUPDATER_IGNORE_SDCARD=1
if(DEFINED UPDATER_IGNORE_SDCARD)
    target_compile_definitions(app PRIVATE
        UPDATER_IGNORE_SDCARD=${UPDATER_IGNORE_SDCARD})
endif()

# Let a pasted log identify which build produced it.
if(UPDATER_SPI_DMA)
    target_compile_definitions(app PRIVATE UPDATER_SPI_DMA=1)
endif()

# Include paths for Semtech driver headers and firmware image
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../adapters/radio/lr11xx
)
