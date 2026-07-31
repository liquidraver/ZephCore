# SPDX-License-Identifier: MIT
# ZephCore Flash Formatter — erases all filesystem partitions, reboots to UF2 DFU.
#
# Build:  west build -b rak4631/nrf52840 zephcore/tools/formatter --pristine
# Flash:  drag-drop build/zephyr/zephyr.uf2 onto UF2 drive

cmake_minimum_required(VERSION 3.20.0)

# Reuse the main project's custom board definitions (overlays for partitions)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Extract base board name (strip qualifier like /nrf52840)
string(REPLACE "/" ";" BOARD_PARTS ${BOARD})
list(GET BOARD_PARTS 0 BOARD_BASE)

# Find board.overlay — prefer formatter-specific overlay (minimal, no sensors/GPS),
# fall back to main board overlay (full hardware definitions).
# Formatter overlays live in tools/formatter/boards/<board>/board.overlay
file(GLOB_RECURSE FMT_OVERLAY_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.overlay"
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.overlay")
if(FMT_OVERLAY_CANDIDATES)
    list(GET FMT_OVERLAY_CANDIDATES 0 FORMATTER_BOARD_OVERLAY)
else()
    # Fall back to main board overlay (partitions + full hardware)
    file(GLOB_RECURSE BOARD_OVERLAY_CANDIDATES
         "${CMAKE_CURRENT_SOURCE_DIR}/../../boards/*/${BOARD_BASE}/board.overlay")
    if(BOARD_OVERLAY_CANDIDATES)
        list(GET BOARD_OVERLAY_CANDIDATES 0 FORMATTER_BOARD_OVERLAY)
    endif()
endif()
if(FORMATTER_BOARD_OVERLAY)
    if(EXTRA_DTC_OVERLAY_FILE)
        set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${FORMATTER_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_DTC_OVERLAY_FILE "${FORMATTER_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    endif()
endif()

# Find formatter-specific board.conf (e.g., QSPI enablement)
# These live in tools/formatter/boards/<board_base>/board.conf
file(GLOB_RECURSE FMT_BOARD_CONF_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.conf"
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.conf")
if(FMT_BOARD_CONF_CANDIDATES)
    list(GET FMT_BOARD_CONF_CANDIDATES 0 FMT_BOARD_CONF)
    if(EXTRA_CONF_FILE)
        set(EXTRA_CONF_FILE "${EXTRA_CONF_FILE};${FMT_BOARD_CONF}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_CONF_FILE "${FMT_BOARD_CONF}" CACHE STRING "" FORCE)
    endif()
endif()

message(STATUS "Formatter board overlay: ${FORMATTER_BOARD_OVERLAY}")
message(STATUS "Formatter board conf: ${FMT_BOARD_CONF}")

# Read the SoftDevice firmware ID from the MAIN board.conf for this board.
# The Adafruit bootloader rejects a DFU package whose --sd-req doesn't match the
# SoftDevice it's running, which is exactly the guard we want: the SDv6 formatter
# must never apply to an SDv7 node (wrong partition map = corrupted node).
#   SoftDevice v6 -> 0x00B6      SoftDevice v7 -> 0x0123
file(GLOB_RECURSE MAIN_BOARD_CONF_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/../../boards/*/${BOARD_BASE}/board.conf")
if(MAIN_BOARD_CONF_CANDIDATES)
    list(GET MAIN_BOARD_CONF_CANDIDATES 0 MAIN_BOARD_CONF)
    file(STRINGS "${MAIN_BOARD_CONF}" _sd_fwid_line REGEX "^CONFIG_ZEPHCORE_SD_FWID=")
    if(_sd_fwid_line)
        string(REGEX REPLACE "^CONFIG_ZEPHCORE_SD_FWID=" "" FORMATTER_SD_REQ "${_sd_fwid_line}")
    endif()
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore_formatter)

target_sources(app PRIVATE src/main.c src/qspi_probe.c)

# Generate an Adafruit DFU package (.zip) next to the .uf2. The .zip is what the
# Mesh America configurator uses for its automated `erase` flow (spec §4a); the
# .uf2 remains the manual drag-and-drop fallback. See PROVIDER_CATALOG.md.
find_program(ADAFRUIT_NRFUTIL adafruit-nrfutil)
if(ADAFRUIT_NRFUTIL AND FORMATTER_SD_REQ)
  add_custom_target(dfu_zip ALL
    COMMAND ${ADAFRUIT_NRFUTIL} dfu genpkg
      --dev-type 0x0052
      --sd-req ${FORMATTER_SD_REQ}
      --application ${CMAKE_BINARY_DIR}/zephyr/zephyr.hex
      ${CMAKE_BINARY_DIR}/zephyr/zephyr.zip
    DEPENDS ${CMAKE_BINARY_DIR}/zephyr/zephyr.hex
    COMMENT "Generating formatter DFU package: zephyr.zip (sd-req=${FORMATTER_SD_REQ})"
    VERBATIM
  )
  add_dependencies(dfu_zip zephyr_final)
  message(STATUS "Formatter DFU zip: ENABLED (sd-req=${FORMATTER_SD_REQ})")
else()
  message(STATUS "Formatter DFU zip: DISABLED (adafruit-nrfutil or SD_FWID missing)")
endif()
