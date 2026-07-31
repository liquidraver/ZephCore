# SPDX-License-Identifier: MIT
# ZephCore RNG selftest — entropy diversity test for identity-seed generation.
#
# Runs ZephyrRNG::mixIdentitySeed N times and checks the results for exact
# duplicates and pairwise correlation. Boots, runs, prints, halts. It is a
# separate image ON PURPOSE: each seed costs ~250 ms of jitter sampling, so
# N=1024 pins the CPU for ~4 minutes — unacceptable inside a live node, where
# it would starve the mesh loop, BLE and LoRa RX.
#
# Build:
#   west build -b rak4631 zephcore/tools/rng_selftest --pristine
#   west build -b xiao_esp32s3/esp32s3/procpu zephcore/tools/rng_selftest --pristine
#
# Sample count (default 1024, ~4 min):
#   west build ... zephcore/tools/rng_selftest -- -DRNG_SELFTEST_N=5000
#
# Flash: nRF52 — drag build/zephyr/zephyr.uf2 onto the UF2 drive
#        ESP32 — west flash --esp-device COMx
#
# The tool never reads or writes stored identity: a device under test keeps
# whatever key it already had. Reflash normal firmware afterwards.

cmake_minimum_required(VERSION 3.20.0)

# Reuse the main project's custom board definitions
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Strip the board qualifier (e.g. "/esp32s3/procpu") to get the base name
string(REPLACE "/" ";" BOARD_PARTS ${BOARD})
list(GET BOARD_PARTS 0 BOARD_BASE)

# Tool-local board overlay ONLY — deliberately not the node's board.overlay.
#
# The node overlay declares /fstab, radio, sensor and partition nodes that this
# tool does not enable (CONFIG_FLASH=n, SPI=n, I2C=n), so reusing it fails on
# an undefined lfs_partition label. Boards needing console plumbing get a
# minimal overlay under boards/<board>/ here; everything else just uses the
# upstream board default, which already provides a console.
set(_SELFTEST_OVERLAY
    "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.overlay")
if(EXISTS "${_SELFTEST_OVERLAY}")
    set(EXTRA_DTC_OVERLAY_FILE "${_SELFTEST_OVERLAY}" CACHE STRING "" FORCE)
    message(STATUS "RNG selftest overlay: ${_SELFTEST_OVERLAY}")
else()
    message(STATUS "RNG selftest: using upstream board console (no overlay)")
endif()

# Per-board conf (UF2 output, code partition) — nRF52 needs it, ESP32 must not
# have it. See boards/rak4631/board.conf.
set(_SELFTEST_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.conf")
if(EXISTS "${_SELFTEST_CONF}")
    set(EXTRA_CONF_FILE "${_SELFTEST_CONF}" CACHE STRING "" FORCE)
    message(STATUS "RNG selftest conf: ${_SELFTEST_CONF}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore_rng_selftest)

# The point of this tool is to exercise the SHIPPING code, not a copy of it.
# ZephyrRNG.cpp and Utils.cpp are compiled straight from the main tree, so the
# algorithm under test is byte-for-byte the one the node runs.
target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../adapters/rng/ZephyrRNG.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Identity.cpp
)

# ZephyrRNG.cpp also carries generateFirstBootIdentity(), which derives the
# Ed25519 keypair via LocalIdentity -> monocypher. We only call
# mixIdentitySeed() here, but linking the real dependency is more honest than
# leaning on --gc-sections to make the undefined symbols disappear — and it
# keeps the door open to testing derived keypairs, not just seeds.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/monocypher monocypher)
target_link_libraries(app PRIVATE monocypher)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../adapters/rng
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/monocypher
)

# ESP32 only: the HWRNG needs the SAR-ADC entropy source enabled before RF is
# up. Same source file the main build pulls in — see zephcore/CMakeLists.txt.
if(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
    set(_ESP_BOOTLOADER_RANDOM
        "${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/bootloader_support/src/bootloader_random_${CONFIG_SOC_SERIES}.c")
    if(EXISTS "${_ESP_BOOTLOADER_RANDOM}")
        message(STATUS "RNG selftest: ESP32 pre-RF entropy (${CONFIG_SOC_SERIES})")
        target_sources(app PRIVATE "${_ESP_BOOTLOADER_RANDOM}")
    else()
        message(WARNING "RNG selftest: no bootloader_random source for ${CONFIG_SOC_SERIES}")
    endif()
endif()

# Sample count. 1024 (~4 min) detects an effective-entropy floor up to ~20 bits
# by the birthday bound — comfortably past the ~15-bit Debian OpenSSL failure.
if(NOT DEFINED RNG_SELFTEST_N)
    set(RNG_SELFTEST_N 1024)
endif()
target_compile_definitions(app PRIVATE RNG_SELFTEST_N=${RNG_SELFTEST_N})
message(STATUS "RNG selftest: N=${RNG_SELFTEST_N} (~${RNG_SELFTEST_N} x 250ms)")

# "Dead HWRNG" mode: zero the hardware-RNG contribution so the diversity run
# measures the two-clock beat ALONE — the definitive answer to "if the HWRNG
# returns nothing, does ZephyrRNG still produce diverse keys?". Needs the
# ZephyrRNG test hook compiled in.
#   west build ... zephcore/tools/rng_selftest -- -DRNG_KILL_HWRNG=1
if(RNG_KILL_HWRNG)
    # RNG_KILL_HWRNG -> main.cpp calls the setter; ZEPHCORE_RNG_TEST_HOOKS ->
    # compiles the setter into ZephyrRNG (both are part of this app target, so
    # the define reaches ZephyrRNG.cpp). Node builds define neither -> the hook
    # is entirely absent from production.
    target_compile_definitions(app PRIVATE RNG_KILL_HWRNG=1 ZEPHCORE_RNG_TEST_HOOKS=1)
    message(STATUS "RNG selftest: KILL_HWRNG — beat-alone measurement")
endif()
