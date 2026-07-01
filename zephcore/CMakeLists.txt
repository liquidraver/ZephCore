# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

# ============================================================================
# Zephyr Patch Auto-Apply
# ============================================================================
# patches/zephyr/*.patch    — unified diffs applied via `git apply` at configure time
# patches/zephyr-new/       — new files copied into the Zephyr tree (no upstream)
#
# Idempotent: stamp file tracks (patch hashes + target HEAD). Conflicts are fatal.
#
function(zephcore_apply_patches PATCH_DIR TARGET_DIR LABEL)
    file(GLOB PATCH_FILES "${PATCH_DIR}/*.patch")
    list(SORT PATCH_FILES)
    if(NOT PATCH_FILES)
        return()
    endif()
    # Stamp = hash(patch contents + target HEAD). Skip if unchanged.
    execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY "${TARGET_DIR}"
        OUTPUT_VARIABLE _target_head
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    set(_hash_input "${_target_head}")
    foreach(_pf ${PATCH_FILES})
        file(MD5 "${_pf}" _h)
        string(APPEND _hash_input "${_h}")
    endforeach()
    string(MD5 _stamp_hash "${_hash_input}")
    set(_stamp "${TARGET_DIR}/.zephcore_${LABEL}.stamp")
    if(EXISTS "${_stamp}")
        file(READ "${_stamp}" _existing_hash)
        string(STRIP "${_existing_hash}" _existing_hash)
        if(_existing_hash STREQUAL _stamp_hash)
            message(STATUS "  [${LABEL}] Patches already applied, skipping.")
            return()
        endif()
        message(STATUS "  [${LABEL}] Patch set changed, re-applying...")
    endif()
    foreach(PATCH_FILE ${PATCH_FILES})
        get_filename_component(PATCH_NAME ${PATCH_FILE} NAME)
        set(PATCH_FILE_TO_APPLY "${PATCH_FILE}")
        # Prepare LF/CRLF patch variants for robust matching across clones.
        file(READ "${PATCH_FILE}" _patch_content)
        string(REPLACE "\r\n" "\n" _patch_lf "${_patch_content}")
        set(_patch_crlf "${_patch_lf}")
        string(REPLACE "\n" "\r\n" _patch_crlf "${_patch_crlf}")
        set(_tmp_patch_dir "${CMAKE_BINARY_DIR}/zephcore_patch_tmp/${LABEL}")
        file(MAKE_DIRECTORY "${_tmp_patch_dir}")
        set(_tmp_patch_lf "${_tmp_patch_dir}/${PATCH_NAME}.lf")
        set(_tmp_patch_crlf "${_tmp_patch_dir}/${PATCH_NAME}.crlf")
        file(WRITE "${_tmp_patch_lf}" "${_patch_lf}")
        file(WRITE "${_tmp_patch_crlf}" "${_patch_crlf}")
        # Dry-run check
        execute_process(
            COMMAND git apply --check "${PATCH_FILE_TO_APPLY}"
            WORKING_DIRECTORY "${TARGET_DIR}"
            RESULT_VARIABLE PATCH_CHECK
            ERROR_VARIABLE PATCH_ERR
        )
        # Retry check with explicit LF/CRLF variants to avoid EOL drift issues.
        if(NOT PATCH_CHECK EQUAL 0)
            execute_process(
                COMMAND git apply --check "${_tmp_patch_lf}"
                WORKING_DIRECTORY "${TARGET_DIR}"
                RESULT_VARIABLE PATCH_CHECK_LF
                ERROR_VARIABLE PATCH_ERR_LF
            )
            if(PATCH_CHECK_LF EQUAL 0)
                set(PATCH_FILE_TO_APPLY "${_tmp_patch_lf}")
                set(PATCH_CHECK 0)
            else()
                execute_process(
                    COMMAND git apply --check "${_tmp_patch_crlf}"
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    RESULT_VARIABLE PATCH_CHECK_CRLF
                    ERROR_VARIABLE PATCH_ERR_CRLF
                )
                if(PATCH_CHECK_CRLF EQUAL 0)
                    set(PATCH_FILE_TO_APPLY "${_tmp_patch_crlf}")
                    set(PATCH_CHECK 0)
                else()
                    # Keep the most relevant error from the last attempted variant.
                    set(PATCH_ERR "${PATCH_ERR_CRLF}")
                endif()
            endif()
        endif()
        # Stale patches from previous build: reset affected files, retry
        if(NOT PATCH_CHECK EQUAL 0)
            # Extract affected file paths from numstat
            execute_process(
                COMMAND git apply --numstat "${PATCH_FILE_TO_APPLY}"
                WORKING_DIRECTORY "${TARGET_DIR}"
                OUTPUT_VARIABLE PATCH_NUMSTAT
                ERROR_QUIET
            )
            # numstat format: "adds\tdels\tpath"
            string(REGEX MATCHALL "[^\t\n]+\t[^\t\n]+\t[^\t\n]+" NUMSTAT_LINES "${PATCH_NUMSTAT}")
            set(PATCH_PATHS "")
            foreach(_line ${NUMSTAT_LINES})
                string(REGEX REPLACE "^[^\t]+\t[^\t]+\t" "" _path "${_line}")
                list(APPEND PATCH_PATHS "${_path}")
            endforeach()
            if(PATCH_PATHS)
                message(STATUS "  [${LABEL}] Resetting stale files for: ${PATCH_NAME}")
                execute_process(
                    COMMAND git checkout -- ${PATCH_PATHS}
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    ERROR_QUIET
                )
                # Retry dry-run
                execute_process(
                    COMMAND git apply --check "${PATCH_FILE_TO_APPLY}"
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    RESULT_VARIABLE PATCH_CHECK
                    ERROR_VARIABLE PATCH_ERR
                )
            endif()
        endif()
        if(NOT PATCH_CHECK EQUAL 0)
            message(FATAL_ERROR
                "ZephCore patch FAILED to apply: ${PATCH_NAME}\n"
                "Target tree: ${TARGET_DIR}\n"
                "Error:\n${PATCH_ERR}\n"
                "Upstream likely changed — rebase the patch:\n"
                "  cd ${TARGET_DIR}\n"
                "  git diff -- <file>   # inspect current upstream\n"
                "  # Regenerate: git diff -- <file> > ${PATCH_FILE}\n"
            )
        endif()
        # Apply
        execute_process(
            COMMAND git apply "${PATCH_FILE_TO_APPLY}"
            WORKING_DIRECTORY "${TARGET_DIR}"
            RESULT_VARIABLE APPLY_RESULT
            ERROR_VARIABLE APPLY_ERR
        )
        if(NOT APPLY_RESULT EQUAL 0)
            message(FATAL_ERROR "git apply failed unexpectedly: ${PATCH_NAME}\n${APPLY_ERR}")
        endif()
        message(STATUS "  [${LABEL}] Applied: ${PATCH_NAME}")
    endforeach()
    file(WRITE "${_stamp}" "${_stamp_hash}")
endfunction()

# Resolve target directories before find_package(Zephyr)
get_filename_component(ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr ABSOLUTE)
get_filename_component(MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../modules ABSOLUTE)

# Apply patches
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/patches/zephyr)
    message(STATUS "Applying ZephCore patches to Zephyr...")
    zephcore_apply_patches(
        "${CMAKE_CURRENT_SOURCE_DIR}/patches/zephyr"
        "${ZEPHYR_DIR}"
        "zephyr"
    )
endif()

# Apply patches to loramac-node module
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/patches/modules/loramac-node)
    message(STATUS "Applying ZephCore patches to loramac-node...")
    zephcore_apply_patches(
        "${CMAKE_CURRENT_SOURCE_DIR}/patches/modules/loramac-node"
        "${MODULES_DIR}/lib/loramac-node"
        "loramac-node"
    )
endif()

# Apply patches to hal_espressif module (ESP32 BLE controller glue)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/patches/modules/hal-espressif)
    message(STATUS "Applying ZephCore patches to hal_espressif...")
    zephcore_apply_patches(
        "${CMAKE_CURRENT_SOURCE_DIR}/patches/modules/hal-espressif"
        "${MODULES_DIR}/hal/espressif"
        "hal-espressif"
    )
endif()

# Copy new files into Zephyr tree
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/patches/zephyr-new)
    file(GLOB_RECURSE ZEPHCORE_NEW_FILES
         RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/patches/zephyr-new
         ${CMAKE_CURRENT_SOURCE_DIR}/patches/zephyr-new/*)
    foreach(REL_PATH ${ZEPHCORE_NEW_FILES})
        set(SRC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/patches/zephyr-new/${REL_PATH})
        set(DST_FILE ${ZEPHYR_DIR}/${REL_PATH})
        configure_file(${SRC_FILE} ${DST_FILE} COPYONLY)
        message(STATUS "  [zephyr-new] ${REL_PATH}")
    endforeach()
endif()

# Custom board root: boards/<mcu>/<board>/ discovered via BOARD_ROOT
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# ========== Auto-Pairing Helper ==========
# For each .conf file, check if a same-named .overlay exists in the same directory.
# If it does, auto-include it in EXTRA_DTC_OVERLAY_FILE. This enables variants to
# pack both Kconfig and device tree changes together (e.g., no_display.conf + no_display.overlay).
#
# Usage: zephcore_auto_pair_overlay("path/to/config.conf")
#
function(zephcore_auto_pair_overlay CONF_FILE)
    if(NOT CONF_FILE)
        return()
    endif()
    # User-supplied EXTRA_CONF_FILE entries are relative to the application
    # source dir (e.g. "boards/linux_native/femtofox.conf"). CMake's EXISTS does
    # not resolve relative paths against it, so resolve to absolute first — else
    # the paired .overlay is silently skipped (which dropped the SX1262 node and
    # broke ZEPHCORE_LORA when device wiring moved out of linux_common.overlay).
    if(NOT IS_ABSOLUTE "${CONF_FILE}")
        set(CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${CONF_FILE}")
    endif()
    if(NOT EXISTS "${CONF_FILE}")
        return()
    endif()
    string(REPLACE ".conf" ".overlay" OVERLAY_FILE "${CONF_FILE}")
    if(EXISTS "${OVERLAY_FILE}")
        if(EXTRA_DTC_OVERLAY_FILE)
            set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${OVERLAY_FILE}" PARENT_SCOPE)
        else()
            set(EXTRA_DTC_OVERLAY_FILE "${OVERLAY_FILE}" PARENT_SCOPE)
        endif()
        message(STATUS "  [auto-pair] ${CONF_FILE} → ${OVERLAY_FILE}")
    endif()
endfunction()

# ========== Board Configuration Hierarchy ==========
# prj.conf → zephcore_common.conf → <platform>_common.conf → <board>/board.conf

# Recover BOARD and EXTRA_CONF_FILE from sysbuild cache (needed before find_package)
if(NOT BOARD AND DEFINED SYSBUILD_CACHE AND EXISTS "${SYSBUILD_CACHE}")
    file(STRINGS "${SYSBUILD_CACHE}" _sysbuild_strings ENCODING UTF-8)
    foreach(_str ${_sysbuild_strings})
        if(_str MATCHES "^BOARD:STRING=(.+)")
            set(BOARD "${CMAKE_MATCH_1}")
        endif()
        # User-specified EXTRA_CONF_FILE (e.g., repeater.conf) is passed to sysbuild,
        # not the app. Recover it so our auto-include logic can detect it.
        if(_str MATCHES "^EXTRA_CONF_FILE:UNINITIALIZED=(.+)")
            set(_SYSBUILD_USER_CONF "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    if(BOARD)
        message(STATUS "Sysbuild: BOARD=${BOARD}")
    endif()
    if(_SYSBUILD_USER_CONF)
        # Merge user extras into EXTRA_CONF_FILE so downstream checks see them
        if(EXTRA_CONF_FILE)
            set(EXTRA_CONF_FILE "${EXTRA_CONF_FILE};${_SYSBUILD_USER_CONF}")
        else()
            set(EXTRA_CONF_FILE "${_SYSBUILD_USER_CONF}")
        endif()
        message(STATUS "Sysbuild: user EXTRA_CONF_FILE=${_SYSBUILD_USER_CONF}")
    endif()
endif()

# Build common config file list based on detected platform
set(ZEPHCORE_COMMON_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/zephcore_common.conf")

# Detect platform from board name and add platform-specific common config
# For boards with qualifiers like "wio_tracker_l1/nrf52840", check both the full BOARD
# string and the BOARD_QUALIFIERS which contains just the qualifier (e.g., "nrf52840")
if(BOARD MATCHES ".*nrf52.*" OR BOARD MATCHES "rak4631" OR BOARD MATCHES "rak_wismesh_tag" OR BOARD MATCHES "rak3401_1watt" OR BOARD MATCHES "wio_tracker" OR BOARD MATCHES "ikoka_nano" OR BOARD MATCHES "t1000_e" OR BOARD MATCHES "thinknode_m1" OR BOARD MATCHES "thinknode_m3" OR BOARD MATCHES "thinknode_m6" OR BOARD MATCHES "promicro_lr2021" OR BOARD MATCHES "promicro_sx1262" OR BOARD MATCHES "sensecap_solar" OR BOARD MATCHES "xiao_nrf52840" OR BOARD MATCHES "lilygo_techo" OR BOARD MATCHES "heltec_t114" OR BOARD MATCHES "heltec_t096" OR BOARD MATCHES "gat562")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf52_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*nrf52.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf52_common.conf")
elseif(BOARD MATCHES ".*esp32.*" OR BOARD MATCHES "station_g2" OR BOARD MATCHES "lilygo_tlora_c6" OR BOARD MATCHES "heltec_wifi_lora32_v3" OR BOARD MATCHES "heltec_wifi_lora32_v4")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*esp32.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32_common.conf")
elseif(BOARD MATCHES ".*nrf54l.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf54l_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*nrf54l.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf54l_common.conf")
elseif(BOARD MATCHES ".*mg24.*" OR BOARD MATCHES ".*efr32.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/mg24_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*mg24.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/mg24_common.conf")
elseif(BOARD MATCHES "native_sim" OR BOARD MATCHES "native_posix")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/linux_native/linux_common.conf")
else()
    set(ZEPHCORE_PLATFORM_CONF "")
endif()

# Check for board-specific config in new folder structure
# Priority: boards/<board>/board.conf > boards/<board>_<variant>.conf (legacy)
#
# For custom boards (e.g., wio_tracker_l1/nrf52840), BOARD contains the base name
# For Zephyr boards with qualifiers (e.g., rak4631/nrf52840), BOARD contains base/qualifier
# We extract just the base board name for folder lookup
if(BOARD)
    string(REPLACE "/" ";" BOARD_PARTS "${BOARD}")
    list(GET BOARD_PARTS 0 BOARD_BASE)
else()
    set(BOARD_BASE "")
endif()

# Search for board.conf in possible locations
set(ZEPHCORE_BOARD_CONF "")
file(GLOB_RECURSE BOARD_CONF_CANDIDATES "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.conf")
if(BOARD_CONF_CANDIDATES)
    list(GET BOARD_CONF_CANDIDATES 0 ZEPHCORE_BOARD_CONF)
endif()
# Fallback: direct path without vendor subdirectory
if(NOT ZEPHCORE_BOARD_CONF AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.conf")
    set(ZEPHCORE_BOARD_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.conf")
endif()

# Build the EXTRA_CONF_FILE list (append to any user-provided extras)
# Auto-pair each .conf with a same-named .overlay if it exists.
set(ZEPHCORE_CONF_FILES "")
if(EXISTS ${ZEPHCORE_COMMON_CONF})
    list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_COMMON_CONF})
    zephcore_auto_pair_overlay("${ZEPHCORE_COMMON_CONF}")
endif()
if(ZEPHCORE_PLATFORM_CONF AND EXISTS ${ZEPHCORE_PLATFORM_CONF})
    list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_PLATFORM_CONF})
    zephcore_auto_pair_overlay("${ZEPHCORE_PLATFORM_CONF}")
endif()
if(ZEPHCORE_BOARD_CONF AND EXISTS ${ZEPHCORE_BOARD_CONF})
    list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_BOARD_CONF})
    zephcore_auto_pair_overlay("${ZEPHCORE_BOARD_CONF}")
endif()

# Production is the default. Debug logging is opt-in via debug.conf.

# Auto-include WiFi OTA for ESP32 repeater builds.
# Enables WiFi AP + HTTP server + MCUboot image management.
# Requires --sysbuild to build MCUboot alongside the app.
# Non-ESP32 builds and companion builds are unaffected.
#
# Excluded on the classic ESP32 (e.g. T-Beam, board qualifier ".../esp32/...").
# It builds, but only because the WiFi DT node would be left disabled; with WiFi
# actually functional, the driver buffers + 64KB OTA heap overflow the classic
# ESP32's DRAM by ~10KB. Rather than trim WiFi/heap to an unverifiable margin,
# the classic ESP32 repeater stays CLI-only on simple boot (handled in build.sh).
# The ESP32-S3/C-series have the DRAM headroom and keep WiFi OTA.
if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common" AND EXTRA_CONF_FILE MATCHES "repeater"
   AND NOT BOARD MATCHES "/esp32/")
    set(ZEPHCORE_WIFI_OTA_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/wifi_ota.conf")
    if(EXISTS ${ZEPHCORE_WIFI_OTA_CONF})
        list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_WIFI_OTA_CONF})
        zephcore_auto_pair_overlay("${ZEPHCORE_WIFI_OTA_CONF}")
        message(STATUS "  WiFi OTA: auto-enabled (ESP32 repeater)")
    endif()
endif()

# Append to EXTRA_CONF_FILE — auto-generated configs first, user extras last.
# User-specified extras (repeater.conf, prod.conf) MUST come after the auto chain
# so they can override settings (e.g. CONFIG_BT=n in repeater.conf).
if(ZEPHCORE_CONF_FILES)
    if(EXTRA_CONF_FILE)
        set(EXTRA_CONF_FILE "${ZEPHCORE_CONF_FILES};${EXTRA_CONF_FILE}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_CONF_FILE "${ZEPHCORE_CONF_FILES}" CACHE STRING "" FORCE)
    endif()
endif()

# ESP32 debug builds: debug.conf is nRF/SEGGER-RTT shaped — on ESP32 RTT yields
# no output (no J-Link; logs go out the UART console) and the deferred-log DRAM
# overflows the RAM-tight classic ESP32. Append debug_esp32.conf LAST so it wins
# over debug.conf (console backend, smaller buffers, trimmed companion arrays).
if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common"
   AND EXTRA_CONF_FILE MATCHES "debug" AND NOT EXTRA_CONF_FILE MATCHES "debug_esp32")
    set(ZEPHCORE_DEBUG_ESP32_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/debug_esp32.conf")
    if(EXISTS ${ZEPHCORE_DEBUG_ESP32_CONF})
        set(EXTRA_CONF_FILE "${EXTRA_CONF_FILE};${ZEPHCORE_DEBUG_ESP32_CONF}" CACHE STRING "" FORCE)
        message(STATUS "  ESP32 debug: appended debug_esp32.conf (no RTT, smaller buffers)")
    endif()
endif()

# Auto-pair user-supplied EXTRA_CONF_FILE entries with same-named .overlay files.
# This allows repeater.conf + repeater.overlay, prod.conf + prod.overlay, etc.
if(EXTRA_CONF_FILE)
    foreach(_conf IN LISTS EXTRA_CONF_FILE)
        zephcore_auto_pair_overlay("${_conf}")
    endforeach()
endif()

# ========== DTC Overlay Hierarchy ==========
# Search for board.overlay in possible locations (same as board.conf)
set(ZEPHCORE_BOARD_OVERLAY "")
file(GLOB_RECURSE BOARD_OVERLAY_CANDIDATES "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.overlay")
if(BOARD_OVERLAY_CANDIDATES)
    list(GET BOARD_OVERLAY_CANDIDATES 0 ZEPHCORE_BOARD_OVERLAY)
endif()
# Fallback: direct path without vendor subdirectory
if(NOT ZEPHCORE_BOARD_OVERLAY AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.overlay")
    set(ZEPHCORE_BOARD_OVERLAY "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.overlay")
endif()

# Append to EXTRA_DTC_OVERLAY_FILE (preserves any user-specified extras)
if(ZEPHCORE_BOARD_OVERLAY AND EXISTS ${ZEPHCORE_BOARD_OVERLAY})
    if(EXTRA_DTC_OVERLAY_FILE)
        set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${ZEPHCORE_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_DTC_OVERLAY_FILE "${ZEPHCORE_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    endif()
endif()


# Debug: Show what configs are being used
message(STATUS "ZephCore config hierarchy:")
message(STATUS "  Common: ${ZEPHCORE_COMMON_CONF}")
message(STATUS "  Platform: ${ZEPHCORE_PLATFORM_CONF}")
message(STATUS "  Board conf: ${ZEPHCORE_BOARD_CONF}")
message(STATUS "  Board overlay: ${ZEPHCORE_BOARD_OVERLAY}")
message(STATUS "  EXTRA_CONF_FILE: ${EXTRA_CONF_FILE}")
message(STATUS "  EXTRA_DTC_OVERLAY_FILE: ${EXTRA_DTC_OVERLAY_FILE}")

# Upstream mbedTLS 4.x has unused-parameter warnings in ssl_misc.h / md.c that
# fail with -Werror. Disable fatal warnings for the vendored mbedtls build.
set(MBEDTLS_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)
set(TF_PSA_CRYPTO_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore)

# Dynamic build date - always current, even without --pristine
string(TIMESTAMP ZEPHCORE_BUILD_DATE "%Y %b %d" UTC)
add_definitions(-DFIRMWARE_BUILD_DATE="${ZEPHCORE_BUILD_DATE}")

# Single source of truth for the firmware version string (vMAJOR.MINOR.PATCH-zephyr).
# Injected globally like FIRMWARE_BUILD_DATE above, so every app TU sees the same
# value; the per-app `#ifndef FIRMWARE_VERSION` fallbacks only apply to builds that
# bypass this injection. NOTE: the BLE DIS value in boards/common/zephcore_common.conf
# (CONFIG_BT_DIS_FW_REV_STR) is Kconfig, not C, so it must be bumped here AND there.
set(ZEPHCORE_FIRMWARE_VERSION "v1.16.3-zephyr")
add_definitions(-DFIRMWARE_VERSION="${ZEPHCORE_FIRMWARE_VERSION}")

add_subdirectory(lib/monocypher)
target_link_libraries(app PRIVATE monocypher)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/helpers
  ${CMAKE_CURRENT_SOURCE_DIR}/app
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/monocypher
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/board
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/clock
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/datastore
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/radio
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/rng
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/gps
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/sensors
  ${CMAKE_CURRENT_SOURCE_DIR}/adapters/ble
)

# ========== Core Sources (both roles) ==========
target_sources(app PRIVATE
  src/ContentionTracker.cpp
  src/Dispatcher.cpp
  src/Identity.cpp
  $<$<BOOL:${CONFIG_ZEPHCORE_APC}>:src/PowerController.cpp>
  src/Mesh.cpp
  src/Packet.cpp
  src/StaticPoolPacketManager.cpp
  src/Utils.cpp
  adapters/board/ZephyrBoard.cpp
  adapters/radio/LoRaRadioBase.cpp
  adapters/clock/ZephyrMillisecondClock.cpp
  adapters/clock/ZephyrRTCClock.cpp
  adapters/rng/RNG.cpp
  adapters/rng/ZephyrRNG.cpp
  adapters/gps/ZephyrGPSManager.cpp
  adapters/sensors/ZephyrEnvSensors.cpp
  helpers/AdvertDataHelpers.cpp
  helpers/oled_power.c
  helpers/fatal_reboot.c
)

# Boot-time hardware-RTC auto-discovery (compact raw-I2C). Always compiled so
# the zephcore_rtc_* symbols exist on every board; it self-stubs internally
# when CONFIG_ZEPHCORE_RTC_AUTODISCOVER is off or no zephcore,rtc-i2c node is
# present in DT.
target_sources(app PRIVATE adapters/clock/ZephyrRTCDiscover.c)

# ========== Battery Curve Selection ==========
# helpers/battery_curve.c is always compiled — it provides battery_curve_lookup()
# and the weak battery_curve_default. A board-specific battery_curve.c is compiled
# alongside it to override the weak symbol with measured cell data.
target_sources(app PRIVATE helpers/battery_curve.c)
file(GLOB_RECURSE _BOARD_BATTERY_CURVE
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/battery_curve.c")
if(_BOARD_BATTERY_CURVE)
  list(GET _BOARD_BATTERY_CURVE 0 _BOARD_BATTERY_CURVE)
  message(STATUS "ZephCore Battery: board-specific curve (${_BOARD_BATTERY_CURVE})")
  target_sources(app PRIVATE "${_BOARD_BATTERY_CURVE}")
else()
  message(STATUS "ZephCore Battery: generic LiPo curve")
endif()

# Native-Linux (native_sim) runtime setup: force real-time clock mode so the
# simulated clock tracks wall time (required for the real SX126x radio's
# BUSY/DIO1 timing).  Bakes in the equivalent of the --rt command-line flag.
if(CONFIG_ARCH_POSIX)
  target_sources(app PRIVATE adapters/transport/linux_native_setup.c)
endif()

# ========== Radio Driver Selection ==========
if(CONFIG_ZEPHCORE_RADIO_LR1110)
  message(STATUS "ZephCore Radio: LR1110 (Zephyr LoRa driver)")
  # Driver + Semtech SDK compiled by Zephyr via patched drivers/lora/lr11xx/
  # (patches/zephyr/ auto-applied at configure time — see patch block above)
  # Only the ZephCore adapter is compiled here.
  target_sources(app PRIVATE
    adapters/radio/LR1110Radio.cpp
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/radio/lr11xx
  )
  # Driver header (lr11xx_lora.h) is in the Zephyr tree after patching
  get_filename_component(ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr ABSOLUTE)
  target_include_directories(app PRIVATE
    ${ZEPHYR_DIR}/drivers/lora/lr11xx
  )
elseif(CONFIG_ZEPHCORE_RADIO_LR2021)
  message(STATUS "ZephCore Radio: LR2021 (Zephyr LoRa driver)")
  target_sources(app PRIVATE
    adapters/radio/LR2021Radio.cpp
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/radio/lr20xx
  )
  get_filename_component(ZEPHYR_DIR_LR20 ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr ABSOLUTE)
  target_include_directories(app PRIVATE
    ${ZEPHYR_DIR_LR20}/drivers/lora/lr20xx
  )
elseif(CONFIG_ZEPHCORE_RADIO_SX127X)
  message(STATUS "ZephCore Radio: SX127x (Zephyr loramac-node driver)")
  target_sources(app PRIVATE
    adapters/radio/SX127xRadio.cpp
  )
else()
  # Default: SX126x via native Zephyr LoRa driver
  message(STATUS "ZephCore Radio: SX126x (native Zephyr driver)")
  target_sources(app PRIVATE
    adapters/radio/SX126xRadio.cpp
  )
  # Extension API header (sx126x_ext.h) is in the Zephyr driver tree
  get_filename_component(ZEPHYR_DIR_SX ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr ABSOLUTE)
  target_include_directories(app PRIVATE
    ${ZEPHYR_DIR_SX}/drivers/lora/native/sx126x
  )
endif()

# ========== Role-Specific Sources ==========
if(CONFIG_ZEPHCORE_ROLE_REPEATER)
  message(STATUS "ZephCore Role: REPEATER")
  target_sources(app PRIVATE
    src/main_repeater.cpp
    app/RepeaterMesh.cpp
    app/RepeaterRegionCLI.cpp
    app/RepeaterDataStore.cpp
    helpers/ClientACL.cpp
    helpers/RegionMap.cpp
    helpers/TransportKeyStore.cpp
    helpers/CommonCLI.cpp
  )
  # Headless repeater (e.g. native-Linux SBC): no display/buttons/buzzer, so the
  # real ui_* implementation (helpers/ui-button or helpers/ui-joystick) isn't
  # compiled. main_repeater.cpp calls ui_* unconditionally — pull in the weak
  # no-op stubs to satisfy the link. Mirrors the companion TCP branch above.
  if(NOT CONFIG_ZEPHCORE_UI_DESIGN_BUTTON AND NOT CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
    target_sources(app PRIVATE helpers/ui/ui_headless_stubs.c)
    target_include_directories(app PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/helpers/ui
    )
  endif()
  # Shared USBD CDC ACM init + 1200-baud DFU + DTR event/callback module.
  # ESP32 boards use usb_serial (no UDC/CDC ACM), so skip when not present.
  if(NOT CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT AND (CONFIG_USB_CDC_ACM OR CONFIG_USBD_CDC_ACM_CLASS))
    target_sources(app PRIVATE
      adapters/usb/ZephyrUSBCDC.cpp
    )
  endif()
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/usb
  )
  if(CONFIG_ZEPHCORE_REPEATER_UPLINK AND CONFIG_MQTT_LIB)
    target_sources(app PRIVATE
      app/RepeaterUplink.cpp
      app/observer_creds.cpp
      adapters/wifi/ZephyrWiFiStation.c
      adapters/mqtt/ZephyrMQTTPublisher.c
    )
    target_include_directories(app PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/adapters/wifi
      ${CMAKE_CURRENT_SOURCE_DIR}/adapters/mqtt
      ${CMAKE_CURRENT_SOURCE_DIR}/app
    )
  elseif(CONFIG_ZEPHCORE_REPEATER_UPLINK)
    message(WARNING "ZephCore repeater uplink requested, but MQTT is disabled (likely prod.conf). Build will exclude uplink runtime.")
  endif()
  target_compile_definitions(app PRIVATE ZEPHCORE_REPEATER=1)
elseif(CONFIG_ZEPHCORE_ROLE_ROOM_SERVER)
  message(STATUS "ZephCore Role: ROOM SERVER")
  target_sources(app PRIVATE
    src/main_room_server.cpp
    app/RoomServerMesh.cpp
    app/RoomServerRegionCLI.cpp
    app/RepeaterDataStore.cpp
    helpers/ClientACL.cpp
    helpers/RegionMap.cpp
    helpers/TransportKeyStore.cpp
    helpers/CommonCLI.cpp
  )
  # Headless build (e.g. native-Linux SBC): no display/buttons/buzzer, so the
  # real ui_* implementation (helpers/ui-button or helpers/ui-joystick) isn't
  # compiled. main_room_server.cpp calls ui_* unconditionally — pull in the weak
  # no-op stubs to satisfy the link. Mirrors the repeater/observer branches.
  if(NOT CONFIG_ZEPHCORE_UI_DESIGN_BUTTON AND NOT CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
    target_sources(app PRIVATE helpers/ui/ui_headless_stubs.c)
    target_include_directories(app PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/helpers/ui
    )
  endif()
  # Shared USBD CDC ACM init + 1200-baud DFU + DTR event/callback module.
  if(NOT CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT AND (CONFIG_USB_CDC_ACM OR CONFIG_USBD_CDC_ACM_CLASS))
    target_sources(app PRIVATE
      adapters/usb/ZephyrUSBCDC.cpp
    )
  endif()
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/usb
  )
  target_compile_definitions(app PRIVATE ZEPHCORE_ROOM_SERVER=1)
elseif(CONFIG_ZEPHCORE_ROLE_OBSERVER)
  message(STATUS "ZephCore Role: OBSERVER")
  target_sources(app PRIVATE
    app/main_observer.cpp
    app/ObserverMesh.cpp
    app/observer_creds.cpp
    app/RepeaterDataStore.cpp
    adapters/wifi/ZephyrWiFiStation.c
    adapters/mqtt/ZephyrMQTTPublisher.c
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/wifi
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/mqtt
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/usb
    ${CMAKE_CURRENT_SOURCE_DIR}/app
  )
  target_compile_definitions(app PRIVATE ZEPHCORE_OBSERVER=1)
else()
  message(STATUS "ZephCore Role: COMPANION")
  target_sources(app PRIVATE
    src/main_companion.cpp
    adapters/datastore/ZephyrDataStore.cpp
    helpers/BaseChatMesh.cpp
    helpers/TransportKeyStore.cpp
    helpers/ui/ui_mesh_actions.cpp
    app/CompanionMesh.cpp
  )
  # Companion transport: TCP socket on native Linux, BLE NUS on MCU builds.
  if(CONFIG_ZEPHCORE_TRANSPORT_TCP)
    message(STATUS "ZephCore Companion Transport: TCP (Linux)")
    target_sources(app PRIVATE adapters/transport/LinuxTCPTransport.c)
    # Headless Linux build: no display/buttons/buzzer source files are
    # compiled, but companion code calls ui_* unconditionally. Weak no-op
    # stubs satisfy the link.
    target_sources(app PRIVATE helpers/ui/ui_headless_stubs.c)
    target_include_directories(app PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/helpers/ui
    )
  else()
    message(STATUS "ZephCore Companion Transport: BLE NUS")
    target_sources(app PRIVATE adapters/ble/ZephyrBLE.cpp)
  endif()
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/transport
  )
  # USB CDC companion transport.  Compiled whenever logging is on (USB can
  # double as the log backend) OR the standalone companion-over-USB feature is
  # enabled — this MUST match the ZEPHCORE_USB_STACK guard in main_companion.cpp
  # (CONFIG_LOG || CONFIG_ZEPHCORE_COMPANION_USB), otherwise the call sites
  # compile but the implementation is never linked (undefined references).
  # Pulls in the shared ZephyrUSBCDC module too (USBD lifecycle + DTR callback).
  if(CONFIG_LOG OR CONFIG_ZEPHCORE_COMPANION_USB)
    target_sources(app PRIVATE
      adapters/usb/ZephyrCompanionUSB.cpp
      helpers/CommonCLI.cpp  # backs the USB-only text CLI dispatch
    )
    if(NOT CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT AND (CONFIG_USB_CDC_ACM OR CONFIG_USBD_CDC_ACM_CLASS))
      target_sources(app PRIVATE
        adapters/usb/ZephyrUSBCDC.cpp
      )
    endif()
  endif()
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/usb
  )
  target_compile_definitions(app PRIVATE ZEPHCORE_COMPANION=1)
endif()

# ========== UI Helpers (conditional on Kconfig) ==========
if(CONFIG_ZEPHCORE_UI_MULTI_TAP)
  target_sources(app PRIVATE
    helpers/ui/input_multi_tap.c
  )
endif()

if(CONFIG_ZEPHCORE_UI_BUZZER)
  target_sources(app PRIVATE
    helpers/ui/buzzer.c
  )
endif()

if(CONFIG_ZEPHCORE_UI_DISPLAY)
  target_sources(app PRIVATE
    helpers/ui/display.c
    helpers/ui/cfb_font_0608.c
  )
endif()

if(CONFIG_ZEPHCORE_DISPLAY_MONO_TFT)
  target_sources(app PRIVATE
    helpers/ui/display_mono_tft.c
  )
endif()

if(CONFIG_ZEPHCORE_UI_BUTTONS OR CONFIG_ZEPHCORE_UI_BUZZER OR CONFIG_ZEPHCORE_UI_DISPLAY)
  target_sources(app PRIVATE
    helpers/ui/ui_common.c
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/ui
  )
  # Repeater and Observer builds need weak stubs for companion-only UI mesh actions
  if(CONFIG_ZEPHCORE_ROLE_REPEATER OR CONFIG_ZEPHCORE_ROLE_OBSERVER OR CONFIG_ZEPHCORE_ROLE_ROOM_SERVER)
    target_sources(app PRIVATE
      helpers/ui/ui_mesh_actions_stubs.c
    )
  endif()
endif()

if(CONFIG_ZEPHCORE_UI_DESIGN_BUTTON)
  target_sources(app PRIVATE
    helpers/ui-button/ui_task.c
    helpers/ui-button/time_sync.c
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/ui-button
  )
endif()

if(CONFIG_ZEPHCORE_UI_DISPLAY AND CONFIG_ZEPHCORE_UI_DESIGN_BUTTON)
  target_sources(app PRIVATE
    helpers/ui-button/ui_pages.c
  )
endif()

if(CONFIG_ZEPHCORE_UI_DISPLAY AND CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
  target_sources(app PRIVATE
    helpers/ui-joystick/time_sync.c
    helpers/ui-joystick/joystick_ui_task.cpp
    helpers/ui-joystick/joystick_ui_hooks.cpp
    helpers/ui-joystick/screens/home.cpp
    helpers/ui-joystick/screens/contacts.cpp
    helpers/ui-joystick/screens/messaging.cpp
    helpers/ui-joystick/screens/tools.cpp
    helpers/ui-joystick/screens/system.cpp
    helpers/ui-joystick/screens/input.cpp
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/ui-joystick
  )
endif()

if(CONFIG_ZEPHCORE_EASTER_EGG_DOOM)
  target_sources(app PRIVATE
    helpers/ui/doom_game.c
  )
endif()

# ========== WiFi OTA (ESP32 repeater builds) ==========
if(CONFIG_ZEPHCORE_WIFI_OTA)
  message(STATUS "ZephCore WiFi OTA: ENABLED")
  target_sources(app PRIVATE
    adapters/ota/wifi_ota.c
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/ota
  )
  # Register per-service HTTP resource iterable section (required by Zephyr HTTP server).
  # Without this, the linker can't find _http_resource_desc_ota_service_list_start/end.
  zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_SOURCE_DIR}/adapters/ota/sections-rom.ld)
endif()

# ========== Post-build: ESP32 flash instructions ==========
if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common")
  if(CONFIG_ZEPHCORE_WIFI_OTA)
    # MCUboot build (via --sysbuild): merged image or west flash
    message(STATUS "")
    message(STATUS "ESP32 Flash Commands (MCUboot + WiFi OTA):")
    message(STATUS "  Flash all (MCUboot + app):  west flash --build-dir build_dir")
    message(STATUS "  OTA update binary:          build_dir/zephcore/zephyr/zephyr.signed.bin")
    message(STATUS "  Full erase first:           esptool --port COMx erase_flash")
    message(STATUS "")
  else()
    # Simple boot: zephyr.bin is a self-contained image at 0x0.
    message(STATUS "")
    message(STATUS "ESP32 Flash Commands:")
    message(STATUS "  Normal update (preserves NVS + user data):")
    message(STATUS "    esptool --port COMx --baud 921600 write_flash 0x0 build_dir/zephyr/zephyr.bin")
    message(STATUS "  Full flash (after erase_flash):")
    message(STATUS "    esptool --port COMx --baud 921600 erase_flash")
    message(STATUS "    esptool --port COMx --baud 921600 write_flash 0x0 build_dir/zephyr/zephyr.bin")
    message(STATUS "  Or use: west flash --build-dir build_dir")
    message(STATUS "")
  endif()
endif()

# ========== Post-build: Generate DFU zip for OTA updates (nRF52 only) ==========
# This runs after ALL build steps (linking, objcopy) via a custom target
# Only applicable on nRF52 boards with Adafruit bootloader
find_program(ADAFRUIT_NRFUTIL adafruit-nrfutil)
if(ADAFRUIT_NRFUTIL AND ZEPHCORE_PLATFORM_CONF MATCHES "nrf52_common")
  set(DFU_ZIP "${CMAKE_BINARY_DIR}/zephyr/zephyr.zip")
  set(DFU_HEX "${CMAKE_BINARY_DIR}/zephyr/zephyr.hex")
  add_custom_target(dfu_zip ALL
    COMMAND ${ADAFRUIT_NRFUTIL} dfu genpkg
      --dev-type 0x0052
      --sd-req ${CONFIG_ZEPHCORE_SD_FWID}
      --application ${DFU_HEX}
      ${DFU_ZIP}
    DEPENDS ${DFU_HEX}
    COMMENT "Generating DFU package: zephyr.zip (sd-req=${CONFIG_ZEPHCORE_SD_FWID})"
    VERBATIM
  )
  # Ensure dfu_zip runs after Zephyr's final target generates the hex
  add_dependencies(dfu_zip zephyr_final)
  message(STATUS "DFU zip generation: ENABLED (adafruit-nrfutil found)")
else()
  message(STATUS "DFU zip generation: DISABLED (adafruit-nrfutil not found)")
endif()
