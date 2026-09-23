# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

# ============================================================================
# Zephyr Patch Auto-Apply
# ============================================================================
# patches/zephyr/*.patch    — unified diffs applied via `git apply` at configure time
# patches/modules/*/*.patch — the same, for west modules (loramac-node, hal_espressif)
# patches/zephyr-new/       — new files copied into the Zephyr tree (no upstream)
#
# Implemented in cmake/zephyr_patches.cmake and cmake/zephyr_new_files.cmake,
# included below and from sysbuild/CMakeLists.txt.

# Resolve target directories before find_package(Zephyr)
get_filename_component(ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr ABSOLUTE)
get_filename_component(MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../modules ABSOLUTE)

# Apply patches, then copy new files into the Zephyr tree.
# Both are shared with sysbuild/CMakeLists.txt, which must do the same before
# MCUboot's devicetree pass — see the modules for why.
set(ZEPHCORE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(ZEPHCORE_ZEPHYR_DIR ${ZEPHYR_DIR})
set(ZEPHCORE_MODULES_DIR ${MODULES_DIR})
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/zephyr_patches.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/zephyr_new_files.cmake)

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
        # Append once, keeping the FIRST position. The user-extras pairing loop
        # below walks the whole combined EXTRA_CONF_FILE, so it re-pairs every
        # conf the auto chain already paired -- board.overlay used to land in
        # the list three times that way. Position is precedence in devicetree,
        # so silently re-appending an overlay also moved it later and let it
        # override things it was never meant to.
        if(EXTRA_DTC_OVERLAY_FILE)
            if(NOT "${OVERLAY_FILE}" IN_LIST EXTRA_DTC_OVERLAY_FILE)
                set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${OVERLAY_FILE}" PARENT_SCOPE)
                message(STATUS "  [auto-pair] ${CONF_FILE} → ${OVERLAY_FILE}")
            endif()
        else()
            set(EXTRA_DTC_OVERLAY_FILE "${OVERLAY_FILE}" PARENT_SCOPE)
            message(STATUS "  [auto-pair] ${CONF_FILE} → ${OVERLAY_FILE}")
        endif()
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
if(BOARD MATCHES ".*nrf52.*" OR BOARD MATCHES "rak4631" OR BOARD MATCHES "rak_wismesh_tag" OR BOARD MATCHES "rak3401_1watt" OR BOARD MATCHES "wio_tracker" OR BOARD MATCHES "ikoka_nano" OR BOARD MATCHES "t1000_e" OR BOARD MATCHES "meshtracker_x1" OR BOARD MATCHES "thinknode_m1" OR BOARD MATCHES "thinknode_m3" OR BOARD MATCHES "thinknode_m6" OR BOARD MATCHES "promicro_lr2021" OR BOARD MATCHES "promicro_sx1262" OR BOARD MATCHES "sensecap_solar" OR BOARD MATCHES "xiao_nrf52840" OR BOARD MATCHES "lilygo_techo" OR BOARD MATCHES "lilygo_timpulse_plus" OR BOARD MATCHES "heltec_t114" OR BOARD MATCHES "heltec_t096" OR BOARD MATCHES "heltec_t1" OR BOARD MATCHES "gat562" OR BOARD MATCHES "muziworks_r1neo")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf52_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*nrf52.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf52_common.conf")
elseif(BOARD MATCHES ".*esp32.*" OR BOARD MATCHES "station_g2" OR BOARD MATCHES "thinknode_m9" OR BOARD MATCHES "lilygo_tlora_c6" OR BOARD MATCHES "heltec_wifi_lora32_v3" OR BOARD MATCHES "heltec_wifi_lora32_v4" OR BOARD MATCHES "meshnology_w12")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*esp32.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32_common.conf")
elseif(BOARD MATCHES ".*nrf54l.*" OR BOARD MATCHES "me25ls02")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf54l_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*nrf54l.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/nrf54l_common.conf")
elseif(BOARD MATCHES ".*mg24.*" OR BOARD MATCHES ".*efr32.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/mg24_common.conf")
elseif(DEFINED BOARD_QUALIFIERS AND BOARD_QUALIFIERS MATCHES ".*mg24.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/mg24_common.conf")
elseif(BOARD MATCHES "lora_e5" OR BOARD MATCHES ".*stm32wl.*")
    set(ZEPHCORE_PLATFORM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/stm32wl_common.conf")
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

# ========== DTC Overlay Precedence ==========
# board.overlay goes in FIRST, so everything applied after it can override it.
#
# Devicetree is last-writer-wins per property, and this list is that order. The
# board overlay is the board's *baseline*; the overlays paired with the confs
# that follow (repeater/pm_esp32/esp32s3_usb/no_display/...) are deliberate
# per-role deviations from it, and the conf chain already documents that same
# precedence ("user extras last so they can override"). Until 2026-08-28 the
# overlay order was the exact opposite of the conf order, with two live
# consequences:
#   - esp32s3_usb.overlay could not move the console off USB Serial/JTAG on the
#     boards that pick a console in board.overlay (heltec V4/V43, wireless
#     tracker v2) -- the reroute was overridden and the console pointed at a
#     peripheral that no longer owned the pins.
#   - pm_esp32.overlay could not hold the SX1262 NSS across light sleep on
#     heltec_wireless_tracker, whose board.overlay re-declares spi2 cs-gpios
#     without ESP32_GPIO_SLEEP_HOLD_EN -- i.e. the exact floating-NSS failure
#     that overlay exists to prevent.
# partitions.overlay is appended after everything (see below) and stays there:
# it must also be fed verbatim to the MCUboot image.
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
# ESP32-S3 and ESP32-C3 (shared BLE controller): overrides esp32_common.conf,
# stays below board.conf.  Every S3 board target carries the /esp32s3/
# qualifier; the C3 boards are single-SoC and are named without one
# (xiao_esp32c3), so match the chip in the name (see esp32c3s3_common.conf).
if(BOARD MATCHES "/esp32s3/" OR BOARD MATCHES "esp32c3")
    list(APPEND ZEPHCORE_CONF_FILES "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32c3s3_common.conf")
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
# ========== ESP32 CPU clock ==========
# Espressif parts idle in the tens of mA (WAITI gates the core clock but leaves
# PLL/peripherals/RAM powered), so CPU frequency is a first-order term in a
# battery node's draw — unlike nRF52, which already idles at microamps.  Pull
# every ESP32 build down to 80 MHz, matching what Arduino MeshCore ships.
# hal_espressif derives ESP_DEFAULT_CPU_FREQ_MHZ from cpu@0 directly, so 80
# needs no module patch (our 0002 carried that until the Zephyr-main bump).
#
# Observers are the exception and keep the SoC default (240 MHz on S3/classic,
# 160 MHz on C3/C6): they run a full WiFi + TLS + MQTT stack and are mains- or
# solar-with-a-big-panel powered, so throughput matters more than current.
# Implemented as "don't apply the overlay" rather than an override, so the
# per-SoC maximum is inherited instead of hardcoded here.
if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common" AND NOT EXTRA_CONF_FILE MATCHES "observer")
    set(ZEPHCORE_ESP32_CPU_OVERLAY
        "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32_cpu_80mhz.overlay")
    if(EXISTS ${ZEPHCORE_ESP32_CPU_OVERLAY})
        if(EXTRA_DTC_OVERLAY_FILE)
            set(EXTRA_DTC_OVERLAY_FILE
                "${EXTRA_DTC_OVERLAY_FILE};${ZEPHCORE_ESP32_CPU_OVERLAY}" CACHE STRING "" FORCE)
        else()
            set(EXTRA_DTC_OVERLAY_FILE
                "${ZEPHCORE_ESP32_CPU_OVERLAY}" CACHE STRING "" FORCE)
        endif()
        message(STATUS "  ESP32 CPU: 80 MHz (observers keep the SoC maximum)")
    endif()
endif()

# ========== ESP32 light sleep (allowlisted repeater boards) ==========
# 80 MHz above cuts the idle draw; light sleep is what removes it. See
# boards/common/pm_esp32.conf for the mechanism and its requirements — this
# block is only the policy that decides who gets it automatically.
#
# REPEATERS ONLY, and the gate is deliberately the same one WiFi OTA uses
# below: the Espressif HCI driver takes no pm_policy locks, so a companion
# build would light-sleep mid-advertising or mid-connection. repeater.conf
# sets CONFIG_BT=n, which removes the problem instead of racing it.
#
# BOARD ALLOWLIST, not a platform check. The hard requirement is DIO1 on an
# RTC-capable pin (GPIO 0-21 on the S3) so a received packet can wake the SoC.
# Every board below wires DIO1 to GPIO14. Deliberately ABSENT and must stay
# absent unless their wiring changes:
#   xiao_esp32s3 (DIO1=GPIO39), station_g2 (GPIO48), thinknode_m9 (GPIO42),
#   lilygo_tlora_c6 (GPIO23, outside the C6's LP range 0-7),
#   lilygo_t3s3 (GPIO33)
#     -> cannot wake on a received packet; the node would go deaf while
#        looking excellent on a current meter.
#   ttgo_lora32 -> SX127x; the DIO1 wake flag lives in the sx126x half of
#        patches/zephyr/0003, so nothing marks the pin as a wake source.
#   heltec_wifi_lora32_v4_r8 -> absent for a different reason: its wiring does
#        qualify (DIO1=GPIO14, NSS=GPIO8, RESET=GPIO12 on &spi2, button on
#        gpio0 0), so every criterion stated here and in pm_esp32.conf is met.
#        It is out because CONFIG_PM has never been validated on the board.
#        Without the anchoring below it would be swept in by substring match
#        rather than by decision. Add it once someone has run pm_esp32.conf's
#        three checks on hardware.
#
# CONSOLE, and why it does NOT gate this list. A native USB Serial/JTAG console
# cannot survive light sleep: the peripheral drops off the bus and the HAL has
# no USJ wake source (only UART0/1/2 and LP-UART). Of the boards below, only
# Heltec V3 consoles on a physical uart0 (CP2102). V4, V4.3 and Wireless
# Tracker V2 choose usb_serial in their board.overlay, and the uart0 reroute in
# boards/common/esp32s3_usb.overlay is COMPANION-ONLY (the auto-include below
# excludes repeater/observer/room_server), so a repeater build of those boards
# keeps a USJ console.
#
# The consequence is accepted, not overlooked: on V4/V4.3/WT-V2 the USB CLI
# stops answering once ZEPHCORE_PM_BOOT_AWAKE_MS expires, and the console flush
# in helpers/pm_esp32_console.c targets UART0, which is not their console, so it
# is inert there. Remote admin over LoRa is unaffected, and connecting a
# terminal normally resets the board (the bridge drives EN from DTR) which
# re-arms the window. These boards are in the field deliberately to get light
# sleep exercised; drop them from the regex to take it away.
#
# Anchored, and every member spelled out. CMake MATCHES is a substring search,
# so the bare alternation matched any BOARD merely *containing* one of these
# names at any offset: "heltec_wifi_lora32_v4" also matches
# "heltec_wifi_lora32_v4_r8", so that board would get CONFIG_PM=y without
# anyone deciding it should. The leading "^" is what stops mid-string matches
# and is not made redundant by the tail.
#
# BOARD carries its qualifiers (heltec_wifi_lora32_v4/esp32s3/procpu), and may
# carry a revision as heltec_wifi_lora32_v3@1/esp32s3/procpu, so the tail
# accepts "@", "/" or end-of-string rather than "$" alone. No board here
# declares a revision today; without "@" one that later did would drop out of
# this list silently.
if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common" AND EXTRA_CONF_FILE MATCHES "repeater"
   AND BOARD MATCHES "^(heltec_wifi_lora32_v3|heltec_wifi_lora32_v4|heltec_wifi_lora32_v43|heltec_wireless_tracker|heltec_wireless_tracker_v2)(@|/|$)")
    set(ZEPHCORE_PM_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/pm_esp32.conf")
    if(EXISTS ${ZEPHCORE_PM_CONF})
        list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_PM_CONF})
        zephcore_auto_pair_overlay("${ZEPHCORE_PM_CONF}")
        message(STATUS "  ESP32 light sleep: auto-enabled (DIO1 wake-capable repeater board)")
    endif()
endif()

if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common" AND EXTRA_CONF_FILE MATCHES "repeater"
   AND NOT BOARD MATCHES "/esp32/")
    set(ZEPHCORE_WIFI_OTA_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/wifi_ota.conf")
    if(EXISTS ${ZEPHCORE_WIFI_OTA_CONF})
        list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_WIFI_OTA_CONF})
        zephcore_auto_pair_overlay("${ZEPHCORE_WIFI_OTA_CONF}")
        message(STATUS "  WiFi OTA: auto-enabled (ESP32 repeater)")
    endif()
endif()

# Auto-include the ESP32-S3 USB CDC-ACM companion transport.
#
# The policy lives here rather than in build.sh so that a locally built
# companion is the same firmware as the published release.  The ESP32 release
# flow already diverges from a plain build in its *layout* (sysbuild/MCUboot);
# letting it diverge in which *transports* the app speaks as well would make
# user bug reports unreproducible from source.
#
# Detection is the board's own devicetree, not a name list: a board that
# includes esp32s3_usb_otg.dtsi has declared the cdc_acm_uart node, which is
# exactly the precondition the USB companion call sites are compiled behind
# (see ZEPHCORE_USB_STACK in src/main_companion.cpp).  Boards without it are
# skipped automatically -- Heltec V3 (USB-C via CP2102, uses
# serial_companion.conf), Wireless Tracker V1, and every C3/C6/classic-ESP32
# part, which has no DWC2 controller at all.
#
# COMPANION ROLE ONLY.  esp32s3_usb.conf sets CONFIG_ZEPHCORE_COMPANION_USB=y,
# which carries no role dependency of its own, and its paired overlay moves the
# console to uart0.  In a repeater/observer/room-server build that would both
# compile the companion USB stack and strand the serial CLI on GPIO43/44.
# serial_companion.conf is excluded for a different reason: COMPANION_SERIAL
# `depends on !COMPANION_USB`, so adding this would silently disable it.
#
# DEBUG BUILDS ARE EXCLUDED.  ESP32 has no RTT (debug_esp32.conf routes logs to
# the console backend) and this conf hands that USB port to the binary
# companion protocol, so an auto-enabled debug build would have nowhere to log.
# Pass both confs explicitly to debug the USB transport itself; logs then need
# a UART adapter on GPIO43/44.
if(ZEPHCORE_PLATFORM_CONF MATCHES "esp32_common" AND ZEPHCORE_BOARD_CONF
   AND NOT EXTRA_CONF_FILE MATCHES "repeater|observer|room_server|serial_companion|esp32s3_usb|debug")
    get_filename_component(ZEPHCORE_BOARD_DIR "${ZEPHCORE_BOARD_CONF}" DIRECTORY)
    if(EXISTS "${ZEPHCORE_BOARD_DIR}/board.overlay")
        file(READ "${ZEPHCORE_BOARD_DIR}/board.overlay" ZEPHCORE_BOARD_OVERLAY_TEXT)
        if(ZEPHCORE_BOARD_OVERLAY_TEXT MATCHES "esp32s3_usb_otg\\.dtsi")
            set(ZEPHCORE_USB_CONF "${CMAKE_CURRENT_SOURCE_DIR}/boards/common/esp32s3_usb.conf")
            if(EXISTS ${ZEPHCORE_USB_CONF})
                list(APPEND ZEPHCORE_CONF_FILES ${ZEPHCORE_USB_CONF})
                zephcore_auto_pair_overlay("${ZEPHCORE_USB_CONF}")
                message(STATUS "  USB CDC companion: auto-enabled (board declares USB OTG CDC-ACM)")
            endif()
        endif()
        unset(ZEPHCORE_BOARD_OVERLAY_TEXT)
    endif()
endif()

# Append to EXTRA_CONF_FILE — auto-generated configs first, user extras last.
# User-specified extras (repeater.conf, debug.conf) MUST come after the auto chain
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
# This allows repeater.conf + repeater.overlay, wifi_ota.conf + wifi_ota.overlay, etc.
if(EXTRA_CONF_FILE)
    foreach(_conf IN LISTS EXTRA_CONF_FILE)
        zephcore_auto_pair_overlay("${_conf}")
    endforeach()
endif()

# ========== DTC Overlay Hierarchy ==========
# Flash partition layout lives in a separate boards/<...>/<board>/partitions.overlay
# so the SAME file can be fed to the MCUboot image too (see sysbuild/CMakeLists.txt).
# The app and MCUboot MUST share one flash map — otherwise MCUboot stages/looks for
# the OTA image at a different address than the app writes it to, and OTA silently
# reverts on reboot. Keeping it in its own file is what makes that sharing possible.
set(ZEPHCORE_BOARD_PARTITIONS "")
file(GLOB_RECURSE BOARD_PARTITIONS_CANDIDATES "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/partitions.overlay")
if(BOARD_PARTITIONS_CANDIDATES)
    list(GET BOARD_PARTITIONS_CANDIDATES 0 ZEPHCORE_BOARD_PARTITIONS)
endif()
if(NOT ZEPHCORE_BOARD_PARTITIONS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/partitions.overlay")
    set(ZEPHCORE_BOARD_PARTITIONS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/partitions.overlay")
endif()
if(ZEPHCORE_BOARD_PARTITIONS AND EXISTS ${ZEPHCORE_BOARD_PARTITIONS})
    if(EXTRA_DTC_OVERLAY_FILE)
        set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${ZEPHCORE_BOARD_PARTITIONS}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_DTC_OVERLAY_FILE "${ZEPHCORE_BOARD_PARTITIONS}" CACHE STRING "" FORCE)
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

# Build-time UNIX epoch: mesh time sync's "provably dead clock" floor (a local
# time below this is impossible for a running build). string(TIMESTAMP) honors
# SOURCE_DATE_EPOCH for reproducible builds.
string(TIMESTAMP ZEPHCORE_BUILD_EPOCH "%s" UTC)
add_definitions(-DFIRMWARE_BUILD_EPOCH=${ZEPHCORE_BUILD_EPOCH}u)

# Single source of truth for the firmware version string (vMAJOR.MINOR.PATCH-zephyr).
# Injected globally like FIRMWARE_BUILD_DATE above, so every app TU sees the same
# value; the per-app `#ifndef FIRMWARE_VERSION` fallbacks only apply to builds that
# bypass this injection. NOTE: the BLE DIS value in boards/common/zephcore_common.conf
# (CONFIG_BT_DIS_FW_REV_STR) is Kconfig, not C, so it must be bumped here AND there.
# The single source of truth for the firmware version. This exact string is what
# the device reports, what CONFIG_BT_DIS_FW_REV_STR advertises, what the GitHub
# release is tagged/named, and what the Mesh America catalog uses as its version
# key — so the configurator can match a running device against the catalog. Keep
# all four identical; the release workflow reads this value directly.
set(ZEPHCORE_FIRMWARE_VERSION "1.17.5c-zephcore")
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
  helpers/MeshTimeSync.cpp
  # Captures the reset cause before anything consumes it; every role needs it
  # at boot, so it is never conditional.
  helpers/boot_info.c
  helpers/oled_power.c
  helpers/fatal_reboot.c
  # LED master gate ("set leds on|off"). Always compiled: helpers/ui/ui_common.c
  # only exists in UI builds, but a headless repeater still drives lora-tx-led.
  helpers/led_gate.c
  # Notification-mode gate. Same reason as led_gate: the CLI needs these
  # symbols on boards that compile no buzzer at all.
  helpers/buzzer_gate.c
  # QSPI bring-up probe. Self-stubs unless CONFIG_ZEPHCORE_QSPI_RDID_PROBE.
  helpers/qspi_probe.c
  # Factory format of every storage region. Compiled for all roles: the
  # companion builds ZephyrDataStore, the other three build
  # RepeaterDataStore, and both call into this.
  adapters/datastore/ZephyrFsFormat.c
)

# Boot-time hardware-RTC auto-discovery (compact raw-I2C). Always compiled so
# the zephcore_rtc_* symbols exist on every board; it self-stubs internally
# when CONFIG_ZEPHCORE_RTC_AUTODISCOVER is off or no zephcore,rtc-i2c node is
# present in DT.
target_sources(app PRIVATE adapters/clock/ZephyrRTCDiscover.c)

# SPA06-003 barometer. No upstream Zephyr driver; compiles to nothing when no
# goertek,spa06 node is present.
target_sources_ifdef(CONFIG_SENSOR app PRIVATE adapters/sensors/spa06.c)

# T1000-E onboard NTC + photocell on the SAADC. Same deal: compiles to nothing
# when no seeed,t1000e-analog node is present.
target_sources_ifdef(CONFIG_SENSOR app PRIVATE adapters/sensors/t1000e_analog.c)

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

# ========== I2C keypad (boards whose only input is a keyboard) ==========
if(CONFIG_INPUT AND CONFIG_DT_HAS_ZEPHCORE_STC8H_KEYPAD_ENABLED)
  target_sources_ifdef(CONFIG_ZEPHCORE_UI_DISPLAY app PRIVATE
    helpers/input/stc8h_keypad.c)
endif()

# ========== Board-specific startup quirks ==========
# Optional per-board sd_quiesce.c — boards whose SD slot shares the radio's SPI
# bus put any inserted card into SPI idle before the radio driver initialises,
# so it cannot disturb the bus. Registers itself with SYS_INIT; nothing calls
# into it, so it is simply compiled in when the board provides one.
file(GLOB_RECURSE _BOARD_SD_QUIESCE
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/sd_quiesce.c")
if(_BOARD_SD_QUIESCE)
  list(GET _BOARD_SD_QUIESCE 0 _BOARD_SD_QUIESCE)
  message(STATUS "ZephCore SD: shared-bus quiesce (${_BOARD_SD_QUIESCE})")
  target_sources(app PRIVATE "${_BOARD_SD_QUIESCE}")
endif()

# ESP32 pre-RF entropy source (bootloader_random).
#
# The ESP32 hardware TRNG (WDEV_RANDOM) is a PRNG that only receives real
# entropy while WiFi or BT is enabled — see drivers/entropy/entropy_esp32.c.
# Identity keygen runs long before any of that (and repeaters/room servers
# never enable RF at all), so sys_csrand_get() contributes nothing there.
#
# Espressif's documented remedy is bootloader_random_enable(), which puts the
# SAR ADC into continuous sampling and mixes its noise into the HWRNG. Its own
# header endorses this exact use: "Can also be called from app code, if true
# random numbers are required without initialized RF subsystem."
# ZephyrRNG::mixIdentitySeed brackets its entropy collection with it.
#
# hal/espressif compiles only part of bootloader_support into app builds (the
# include path is already set up, but not these sources), so pull in the
# per-SoC implementation explicitly. The generic bootloader_random.c is NOT
# needed — it only defines these symbols for the bring-up-bypass stub.
if(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
  set(_ESP_BOOTLOADER_RANDOM
      "${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/bootloader_support/src/bootloader_random_${CONFIG_SOC_SERIES}.c")
  if(EXISTS "${_ESP_BOOTLOADER_RANDOM}")
    message(STATUS "ZephCore RNG: ESP32 pre-RF entropy (${CONFIG_SOC_SERIES})")
    target_sources(app PRIVATE "${_ESP_BOOTLOADER_RANDOM}")
  else()
    message(WARNING
      "ZephCore RNG: no bootloader_random source for ${CONFIG_SOC_SERIES} — "
      "identity keygen will fall back to CPU jitter alone. Check "
      "hal/espressif components/bootloader_support/src/.")
  endif()
endif()

# ESP32 light-sleep console guards — TX flush before sleep + post-boot awake
# window. Only meaningful when CONFIG_PM is on (boards/common/pm_esp32.conf).
if(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32 AND CONFIG_PM)
  target_sources(app PRIVATE helpers/pm_esp32_console.c)
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
    message(WARNING "ZephCore repeater uplink requested, but MQTT is disabled in this configuration. Build will exclude uplink runtime.")
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
    # CommonCLI backs the v-contact loopback admin chat (companion_cli_exec in
    # main_companion.cpp), which is NOT gated on the wired-USB stack — so it
    # must be compiled unconditionally. Builds without CONFIG_LOG/COMPANION_USB/
    # COMPANION_SERIAL (e.g. plain ESP32/S3 companions) failed to link otherwise.
    helpers/CommonCLI.cpp
  )
  # Companion transport: TCP socket on native Linux, serial (UART) on boards
  # with no Bluetooth controller (e.g. STM32WL), BLE NUS everywhere else.
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
  elseif(NOT CONFIG_BT)
    # No BLE controller (STM32WL) — companion speaks MeshCore serial framing
    # over the chosen console UART. SerialCompanionTransport.c is a full
    # drop-in zephcore_ble_* provider, just like LinuxTCPTransport.c.
    message(STATUS "ZephCore Companion Transport: Serial (UART)")
    target_sources(app PRIVATE adapters/transport/SerialCompanionTransport.c)
  else()
    message(STATUS "ZephCore Companion Transport: BLE NUS")
    target_sources(app PRIVATE adapters/ble/ZephyrBLE.cpp)
  endif()
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/adapters/transport
  )
  # Wired-companion transport.  Compiled whenever logging is on (USB can
  # double as the log backend), the native-USB companion is enabled, OR the
  # plain-UART companion is enabled — this MUST match the ZEPHCORE_USB_STACK
  # guard in main_companion.cpp (CONFIG_LOG || CONFIG_ZEPHCORE_COMPANION_USB ||
  # CONFIG_ZEPHCORE_COMPANION_SERIAL), otherwise the call sites compile but the
  # implementation is never linked (undefined references).  The shared
  # ZephyrUSBCDC module (USBD lifecycle + DTR) is pulled in only for the native
  # USB CDC backend — the plain-UART backend has no USBD stack.
  if(CONFIG_LOG OR CONFIG_ZEPHCORE_COMPANION_USB OR CONFIG_ZEPHCORE_COMPANION_SERIAL)
    target_sources(app PRIVATE
      adapters/usb/ZephyrCompanionUSB.cpp
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
    helpers/ui/haptic.c
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
