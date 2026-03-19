# SPDX-License-Identifier: Apache-2.0
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

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore_formatter)

target_sources(app PRIVATE src/main.c src/qspi_probe.c)
