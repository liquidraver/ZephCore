/*
 * SPDX-License-Identifier: MIT
 * Weak stubs for companion-only UI mesh actions.
 *
 * Repeater builds with a display compile ui_task.c which references these
 * functions. The real implementations live in ui_mesh_actions.cpp and are
 * linked only for companion builds. These weak stubs allow repeater+display
 * builds to link cleanly — button actions that require BLE/GPS/companion
 * features simply do nothing.
 */

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

__attribute__((weak)) void mesh_send_flood_advert(void) {}
__attribute__((weak)) void mesh_send_zerohop_advert(void) {}
__attribute__((weak)) void mesh_gps_set_enabled(bool enable) { ARG_UNUSED(enable); }
__attribute__((weak)) void mesh_ble_set_enabled(bool enable) { ARG_UNUSED(enable); }
__attribute__((weak)) void mesh_set_buzzer_quiet(bool quiet) { ARG_UNUSED(quiet); }
__attribute__((weak)) void mesh_set_offgrid_mode(bool enable) { ARG_UNUSED(enable); }
__attribute__((weak)) void mesh_set_leds_disabled(bool disabled) { ARG_UNUSED(disabled); }
__attribute__((weak)) void mesh_disable_power_regulators(void) {}
__attribute__((weak)) void mesh_reboot_to_ota_dfu(void) {}
__attribute__((weak)) void mesh_handle_ui_actions(void) {}
__attribute__((weak)) void mesh_housekeeping_ui_refresh(void) {}
