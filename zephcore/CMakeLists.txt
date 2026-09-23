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

# Single source of truth for the firmware version string (vMAJOR.MINOR.PATCH-zephyr).
# This exact string is what the device reports, what the BLE Device Information
# Service advertises (CONFIG_BT_DIS_FW_REV_STR, generated from it below), what the
# GitHub release is tagged/named, and what the Mesh America catalog uses as its
# version key. The release workflow reads this line directly — keep its form.
set(ZEPHCORE_FIRMWARE_VERSION "1.17.5c-zephcore")

# ========== Helpers ==========
# Append FILE to the overlay list once, keeping its FIRST position. Position is
# precedence in devicetree, so a re-append would silently move an overlay later
# and let it override things it was never meant to.
function(zephcore_add_overlay FILE)
    if(EXISTS "${FILE}" AND NOT "${FILE}" IN_LIST ZEPHCORE_OVERLAYS)
        set(ZEPHCORE_OVERLAYS ${ZEPHCORE_OVERLAYS} "${FILE}" PARENT_SCOPE)
    endif()
endfunction()

# Append a conf to the chain as given (a missing file stays Zephyr's error to
# report), plus its same-named .overlay if one exists, so a variant can ship
# Kconfig and devicetree changes together (no_display.conf + no_display.overlay).
# The overlay lookup resolves relative paths against the app source dir: CMake's
# EXISTS does not, and a skipped pairing once dropped the SX1262 node from a
# native-Linux preset.
function(zephcore_add_conf FILE)
    set(ZEPHCORE_CONFS ${ZEPHCORE_CONFS} "${FILE}" PARENT_SCOPE)
    if(NOT IS_ABSOLUTE "${FILE}")
        set(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${FILE}")
    endif()
    string(REGEX REPLACE "\\.conf$" ".overlay" _ov "${FILE}")
    if(EXISTS "${_ov}" AND NOT "${_ov}" IN_LIST ZEPHCORE_OVERLAYS)
        set(ZEPHCORE_OVERLAYS ${ZEPHCORE_OVERLAYS} "${_ov}" PARENT_SCOPE)
        message(STATUS "  [auto-pair] ${FILE} → ${_ov}")
    endif()
endfunction()

# ========== User input ==========
# The confs/overlays the user asked for, kept apart from what we derive so that
# a reconfigure without --pristine starts from the same input instead of from
# our own previous output (which used to get the auto chain prepended again).
# A value different from the one we last wrote is new user input.
foreach(_kind CONF_FILE DTC_OVERLAY_FILE)
    if(NOT DEFINED ZEPHCORE_WRITTEN_EXTRA_${_kind}
       OR NOT "${EXTRA_${_kind}}" STREQUAL "${ZEPHCORE_WRITTEN_EXTRA_${_kind}}")
        set(ZEPHCORE_USER_EXTRA_${_kind} "${EXTRA_${_kind}}" CACHE INTERNAL "")
    endif()
endforeach()
set(ZEPHCORE_USER_CONFS "${ZEPHCORE_USER_EXTRA_CONF_FILE}")

# Under sysbuild the user's EXTRA_CONF_FILE goes to sysbuild, not the app, and
# BOARD is not set yet: recover both from the sysbuild cache.
if(NOT BOARD AND DEFINED SYSBUILD_CACHE AND EXISTS "${SYSBUILD_CACHE}")
    file(STRINGS "${SYSBUILD_CACHE}" _sysbuild_strings ENCODING UTF-8)
    foreach(_str ${_sysbuild_strings})
        if(_str MATCHES "^BOARD:STRING=(.+)")
            set(BOARD "${CMAKE_MATCH_1}")
        endif()
        if(_str MATCHES "^EXTRA_CONF_FILE:UNINITIALIZED=(.+)")
            list(APPEND ZEPHCORE_USER_CONFS "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    message(STATUS "Sysbuild: BOARD=${BOARD} user confs=${ZEPHCORE_USER_CONFS}")
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/zephcore_board.cmake)

# ========== Build class ==========
# Role and debug class decide the auto-includes below, which have to be chosen
# before Kconfig exists. They are derived from the user's conf *names* (exact
# match, not substring) or from a -DCONFIG_ZEPHCORE_ROLE_* on the command line,
# and checked against the resolved Kconfig after find_package().
set(_user_conf_names "")
foreach(_c IN LISTS ZEPHCORE_USER_CONFS)
    get_filename_component(_n "${_c}" NAME_WE)
    list(APPEND _user_conf_names "${_n}")
endforeach()
if("observer" IN_LIST _user_conf_names OR CONFIG_ZEPHCORE_ROLE_OBSERVER)
    set(ZEPHCORE_BUILD_ROLE observer)
elseif("room_server" IN_LIST _user_conf_names OR CONFIG_ZEPHCORE_ROLE_ROOM_SERVER)
    set(ZEPHCORE_BUILD_ROLE room_server)
elseif("repeater" IN_LIST _user_conf_names OR CONFIG_ZEPHCORE_ROLE_REPEATER)
    set(ZEPHCORE_BUILD_ROLE repeater)
else()
    set(ZEPHCORE_BUILD_ROLE companion)
endif()
# Diagnostic overlays count as debug builds: on ESP32 they keep the USB port for
# the console and get debug_esp32.conf's console log backend.
set(ZEPHCORE_BUILD_DEBUG FALSE)
foreach(_n debug debug_esp32 hangdebug gps_debug)
    if(_n IN_LIST _user_conf_names)
        set(ZEPHCORE_BUILD_DEBUG TRUE)
    endif()
endforeach()

# ========== Config chain ==========
# Kconfig and devicetree share one precedence order, later wins:
#   zephcore_common → <platform>_common → esp32c3s3_common → version → board
#   → auto-included feature confs → user extras → (debug_esp32) → partitions.overlay
#
# board.overlay goes FIRST among overlays: it is the board's baseline, and the
# overlays paired with later confs (repeater/pm_esp32/esp32s3_usb/no_display/...)
# are deliberate per-role deviations from it. Until 2026-08-28 the overlay order
# was the opposite of the conf order, which broke esp32s3_usb.overlay's console
# reroute on boards that pick a console in board.overlay (heltec V4/V43, wireless
# tracker v2) and pm_esp32.overlay's SX1262 NSS sleep-hold on
# heltec_wireless_tracker.
set(ZEPHCORE_CONFS "")
set(ZEPHCORE_OVERLAYS "")
zephcore_board_file(ZEPHCORE_BOARD_OVERLAY board.overlay)
zephcore_add_overlay("${ZEPHCORE_BOARD_OVERLAY}")

set(_zc "${CMAKE_CURRENT_SOURCE_DIR}/boards/common")
set(_platform_conf_of_nrf52   ${_zc}/nrf52_common.conf)
set(_platform_conf_of_nrf54l  ${_zc}/nrf54l_common.conf)
set(_platform_conf_of_esp32   ${_zc}/esp32_common.conf)
set(_platform_conf_of_mg24    ${_zc}/mg24_common.conf)
set(_platform_conf_of_stm32wl ${_zc}/stm32wl_common.conf)
set(_platform_conf_of_linux   ${CMAKE_CURRENT_SOURCE_DIR}/boards/linux_native/linux_common.conf)
zephcore_add_conf(${_zc}/zephcore_common.conf)
if(ZEPHCORE_PLATFORM)
    zephcore_add_conf(${_platform_conf_of_${ZEPHCORE_PLATFORM}})
endif()
# ESP32-S3 and ESP32-C3 (shared BLE controller): overrides esp32_common.conf,
# stays below board.conf. Every S3 target carries the /esp32s3/ qualifier; the
# C3 boards are single-SoC and named without one (xiao_esp32c3).
if(BOARD MATCHES "/esp32s3/" OR BOARD MATCHES "esp32c3")
    zephcore_add_conf(${_zc}/esp32c3s3_common.conf)
endif()
# The BLE DIS firmware revision is Kconfig, so it is generated from the one
# version string above rather than kept by hand in a second place.
set(_version_conf "${CMAKE_BINARY_DIR}/zephcore_version.conf")
file(WRITE "${_version_conf}" "CONFIG_BT_DIS_FW_REV_STR=\"${ZEPHCORE_FIRMWARE_VERSION}\"\n")
list(APPEND ZEPHCORE_CONFS "${_version_conf}")
zephcore_board_file(ZEPHCORE_BOARD_CONF board.conf)
if(ZEPHCORE_BOARD_CONF)
    zephcore_add_conf("${ZEPHCORE_BOARD_CONF}")
endif()

# ---- Auto-included per-role features (ESP32 only today) ----
if(ZEPHCORE_PLATFORM STREQUAL "esp32")
    # CPU clock. Espressif parts idle in the tens of mA (WAITI gates the core
    # clock but leaves PLL/peripherals/RAM powered), so CPU frequency is a
    # first-order term in a battery node's draw — unlike nRF52, which already
    # idles at microamps. Every ESP32 build runs at 80 MHz, matching Arduino
    # MeshCore; hal_espressif derives ESP_DEFAULT_CPU_FREQ_MHZ from cpu@0, so no
    # module patch is needed. Observers keep the SoC maximum (240 MHz S3/classic,
    # 160 MHz C3/C6): full WiFi + TLS + MQTT on mains or big-panel solar, so
    # throughput matters more than current. "Don't apply the overlay" rather than
    # an override, so the per-SoC maximum is inherited, not hardcoded.
    if(NOT ZEPHCORE_BUILD_ROLE STREQUAL "observer")
        zephcore_add_overlay(${_zc}/esp32_cpu_80mhz.overlay)
        message(STATUS "  ESP32 CPU: 80 MHz (observers keep the SoC maximum)")
    endif()

    # Light sleep: REPEATERS on boards that declare it. 80 MHz cuts the idle draw;
    # light sleep is what removes it (mechanism and requirements:
    # boards/common/pm_esp32.conf). Repeaters only: the Espressif HCI driver takes
    # no pm_policy locks, so a companion would light-sleep mid-advertising or
    # mid-connection; repeater.conf sets CONFIG_BT=n. Which boards qualify is a
    # hardware fact (DIO1 on an RTC-wake-capable GPIO, and validated on hardware),
    # so the board declares it: `capabilities: light_sleep: true` in its
    # zephcore.yml. pm_esp32.conf lists which boards fail the wiring test and why;
    # a board that passes it but was never validated (v4_r8) stays undeclared.
    if(ZEPHCORE_BUILD_ROLE STREQUAL "repeater" AND ZEPHCORE_BOARD_LIGHT_SLEEP)
        zephcore_add_conf(${_zc}/pm_esp32.conf)
        message(STATUS "  ESP32 light sleep: auto-enabled (DIO1 wake-capable repeater board)")
    endif()

    # WiFi OTA for S3/C-series repeaters: WiFi AP + HTTP server + MCUboot image
    # management, so these builds need --sysbuild. Not on the classic ESP32
    # (".../esp32/..."): with WiFi functional, the driver buffers + 64 KB OTA heap
    # overflow its DRAM by ~10 KB, so the classic repeater stays CLI-only on simple
    # boot.
    if(ZEPHCORE_BUILD_ROLE STREQUAL "repeater" AND NOT BOARD MATCHES "/esp32/")
        zephcore_add_conf(${_zc}/wifi_ota.conf)
        message(STATUS "  WiFi OTA: auto-enabled (ESP32 repeater)")
    endif()

    # USB CDC-ACM companion transport on S3 boards whose board.overlay includes
    # esp32s3_usb_otg.dtsi (that include declares the cdc_acm_uart node the USB
    # companion call sites are compiled behind — see ZEPHCORE_USB_STACK in
    # src/main_companion.cpp). Lives here, not in build.sh, so a locally built
    # companion is the same firmware as the release. Companion only:
    # esp32s3_usb.conf enables the companion USB stack and its overlay moves the
    # console to uart0, which would strand a server role's CLI on GPIO43/44. Not
    # with serial_companion (COMPANION_SERIAL depends on !COMPANION_USB) and not in
    # debug builds (ESP32 has no RTT; the port must stay the log console).
    if(ZEPHCORE_BUILD_ROLE STREQUAL "companion" AND NOT ZEPHCORE_BUILD_DEBUG
       AND NOT "serial_companion" IN_LIST _user_conf_names
       AND NOT "esp32s3_usb" IN_LIST _user_conf_names
       AND ZEPHCORE_BOARD_OVERLAY)
        file(READ "${ZEPHCORE_BOARD_OVERLAY}" _board_overlay_text)
        if(_board_overlay_text MATCHES "esp32s3_usb_otg\\.dtsi")
            zephcore_add_conf(${_zc}/esp32s3_usb.conf)
            message(STATUS "  USB CDC companion: auto-enabled (board declares USB OTG CDC-ACM)")
        endif()
    endif()
endif()

# ---- User extras last, so they override everything above ----
foreach(_c IN LISTS ZEPHCORE_USER_CONFS)
    zephcore_add_conf("${_c}")
endforeach()

# ESP32 debug builds: debug.conf is nRF/SEGGER-RTT shaped — on ESP32 RTT yields
# no output (no J-Link; logs go out the UART console) and the deferred-log DRAM
# overflows the classic ESP32. debug_esp32.conf goes LAST so it wins over
# debug.conf (console backend, smaller buffers, trimmed companion arrays).
if(ZEPHCORE_PLATFORM STREQUAL "esp32" AND ZEPHCORE_BUILD_DEBUG
   AND NOT "debug_esp32" IN_LIST _user_conf_names)
    zephcore_add_conf(${_zc}/debug_esp32.conf)
    message(STATUS "  ESP32 debug: appended debug_esp32.conf (no RTT, smaller buffers)")
endif()

# User-supplied overlays (-DEXTRA_DTC_OVERLAY_FILE) win over everything derived
# above, like user confs do. Passed through as given — Zephyr resolves relative
# paths and reports a missing file — and moved to the end if already listed.
# (Before 2026-09-23 they went in FIRST, i.e. lowest precedence.)
foreach(_ov IN LISTS ZEPHCORE_USER_EXTRA_DTC_OVERLAY_FILE)
    list(REMOVE_ITEM ZEPHCORE_OVERLAYS "${_ov}")
    list(APPEND ZEPHCORE_OVERLAYS "${_ov}")
endforeach()

# Flash partitions last, even after user overlays: the SAME file is fed to the
# MCUboot image
# (sysbuild/CMakeLists.txt). App and MCUboot must share one flash map, otherwise
# MCUboot looks for the OTA image at a different address than the app writes it
# to and OTA silently reverts on reboot.
zephcore_board_file(ZEPHCORE_BOARD_PARTITIONS partitions.overlay)
zephcore_add_overlay("${ZEPHCORE_BOARD_PARTITIONS}")

set(EXTRA_CONF_FILE "${ZEPHCORE_CONFS}" CACHE STRING "" FORCE)
set(EXTRA_DTC_OVERLAY_FILE "${ZEPHCORE_OVERLAYS}" CACHE STRING "" FORCE)
set(ZEPHCORE_WRITTEN_EXTRA_CONF_FILE "${EXTRA_CONF_FILE}" CACHE INTERNAL "")
set(ZEPHCORE_WRITTEN_EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE}" CACHE INTERNAL "")

message(STATUS "ZephCore config hierarchy:")
message(STATUS "  Board dir: ${ZEPHCORE_BOARD_DIR}")
message(STATUS "  Platform: ${ZEPHCORE_PLATFORM}  Role: ${ZEPHCORE_BUILD_ROLE}  Debug: ${ZEPHCORE_BUILD_DEBUG}")
message(STATUS "  EXTRA_CONF_FILE: ${EXTRA_CONF_FILE}")
message(STATUS "  EXTRA_DTC_OVERLAY_FILE: ${EXTRA_DTC_OVERLAY_FILE}")

# Upstream mbedTLS 4.x has unused-parameter warnings in ssl_misc.h / md.c that
# fail with -Werror. Disable fatal warnings for the vendored mbedtls build.
set(MBEDTLS_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)
set(TF_PSA_CRYPTO_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore)

# The build-class guess made before Kconfig existed must match the role Kconfig
# actually resolved; otherwise the auto-included confs above were chosen for the
# wrong role (e.g. the ESP32 USB companion transport in a server build, which
# strands the serial CLI). Pass the role via boards/common/<role>.conf.
set(_kconfig_role companion)
foreach(_r repeater room_server observer)
    string(TOUPPER "${_r}" _R)
    if(CONFIG_ZEPHCORE_ROLE_${_R})
        set(_kconfig_role ${_r})
    endif()
endforeach()
if(NOT _kconfig_role STREQUAL ZEPHCORE_BUILD_ROLE)
    message(FATAL_ERROR
        "ZephCore: Kconfig resolved role '${_kconfig_role}' but the build was "
        "classified as '${ZEPHCORE_BUILD_ROLE}' from EXTRA_CONF_FILE. Select the "
        "role with boards/common/${_kconfig_role}.conf (or -DCONFIG_ZEPHCORE_ROLE_*).")
endif()

# Build date and epoch, injected into every app TU. Both are day-granular so a
# reconfigure on the same day leaves every compile command unchanged and ccache's
# direct mode keeps hitting. The epoch is mesh time sync's "provably dead clock"
# floor (a local time below it is impossible for a running build); midnight UTC
# of the build day keeps that true. string(TIMESTAMP) honors SOURCE_DATE_EPOCH
# for reproducible builds.
string(TIMESTAMP ZEPHCORE_BUILD_DATE "%Y %b %d" UTC)
add_definitions(-DFIRMWARE_BUILD_DATE="${ZEPHCORE_BUILD_DATE}")
string(TIMESTAMP _now "%s" UTC)
math(EXPR ZEPHCORE_BUILD_EPOCH "${_now} - (${_now} % 86400)")
add_definitions(-DFIRMWARE_BUILD_EPOCH=${ZEPHCORE_BUILD_EPOCH}u)

# Version string: set once near the top of this file (the release workflow reads
# that line). The per-app `#ifndef FIRMWARE_VERSION` fallbacks only apply to
# builds that bypass this injection.
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
zephcore_board_file(_BOARD_BATTERY_CURVE battery_curve.c)
if(_BOARD_BATTERY_CURVE)
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
zephcore_board_file(_BOARD_SD_QUIESCE sd_quiesce.c)
if(_BOARD_SD_QUIESCE)
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
  target_include_directories(app PRIVATE
    ${ZEPHYR_DIR}/drivers/lora/lr20xx
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
  target_include_directories(app PRIVATE
    ${ZEPHYR_DIR}/drivers/lora/native/sx126x
  )
endif()

# ========== Role-Specific Sources ==========
# Source order is link order and so affects the image; keep it stable.
if(CONFIG_ZEPHCORE_ROLE_REPEATER OR CONFIG_ZEPHCORE_ROLE_ROOM_SERVER)
  if(CONFIG_ZEPHCORE_ROLE_REPEATER)
    message(STATUS "ZephCore Role: REPEATER")
    target_sources(app PRIVATE
      src/main_repeater.cpp
      app/RepeaterMesh.cpp
      app/RepeaterRegionCLI.cpp
    )
  else()
    message(STATUS "ZephCore Role: ROOM SERVER")
    target_sources(app PRIVATE
      src/main_room_server.cpp
      app/RoomServerMesh.cpp
      app/RoomServerRegionCLI.cpp
    )
  endif()
  # Shared by both server roles: persistence under /lfs/repeater/, client ACL,
  # region filtering, transport keys, and the text CLI.
  target_sources(app PRIVATE
    app/RepeaterDataStore.cpp
    helpers/ClientACL.cpp
    helpers/RegionMap.cpp
    helpers/TransportKeyStore.cpp
    helpers/CommonCLI.cpp
  )
  # Headless build (e.g. native-Linux SBC): no display/buttons/buzzer, so the
  # real ui_* implementation (helpers/ui-button or helpers/ui-joystick) isn't
  # compiled, but the server mains call ui_* unconditionally — the weak no-op
  # stubs satisfy the link.
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
  if(CONFIG_ZEPHCORE_ROLE_REPEATER)
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
  else()
    target_compile_definitions(app PRIVATE ZEPHCORE_ROOM_SERVER=1)
  endif()
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

# ========== Post-build: Generate DFU zip for OTA updates (nRF52 only) ==========
# This runs after ALL build steps (linking, objcopy) via a custom target
# Only applicable on nRF52 boards with Adafruit bootloader
find_program(ADAFRUIT_NRFUTIL adafruit-nrfutil)
if(ADAFRUIT_NRFUTIL AND ZEPHCORE_PLATFORM STREQUAL "nrf52")
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
