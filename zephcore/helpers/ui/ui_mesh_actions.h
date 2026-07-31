/*
 * SPDX-License-Identifier: MIT
 * ZephCore UI ↔ Mesh Action Helpers
 *
 * Deferred UI button actions and periodic housekeeping display refresh.
 * Called from the mesh event loop thread (safe for LoRa TX, flash writes).
 *
 * The extern "C" functions (mesh_send_flood_advert, etc.) are called from
 * ui_task.c button handlers in the input thread context.
 */
#ifndef ZEPHCORE_UI_MESH_ACTIONS_H
#define ZEPHCORE_UI_MESH_ACTIONS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the UI mesh actions module.
 * Must be called after companion_mesh is constructed and before the event loop.
 *
 * @param mesh_events  Pointer to the mesh k_event
 * @param mesh_event_ui_action  Bitmask for MESH_EVENT_UI_ACTION
 * @param companion_mesh  Opaque pointer to CompanionMesh (cast internally)
 * @param data_store      Opaque pointer to ZephyrDataStore
 * @param lora_radio      Opaque pointer to LoRa radio adapter
 * @param zephyr_board    Opaque pointer to ZephyrBoard
 * @param rtc_clock       Opaque pointer to ZephyrRTCClock
 */
void ui_mesh_actions_init(struct k_event *mesh_events, uint32_t mesh_event_ui_action,
			  void *companion_mesh, void *data_store,
			  void *lora_radio, void *zephyr_board, void *rtc_clock);

/**
 * Process deferred UI actions — called from mesh event loop on MESH_EVENT_UI_ACTION.
 * Safe to access LoRa radio, flash, and CompanionMesh from here.
 */
void mesh_handle_ui_actions(void);

/**
 * Periodic UI display refresh — called from mesh event loop on MESH_EVENT_HOUSEKEEPING.
 * Updates battery, noise floor, GPS, recently heard, and sensors.
 */
void mesh_housekeeping_ui_refresh(void);

/*
 * Button action wrappers — called from ui_task.c (C code).
 * These run in the input thread. Heavy work is deferred to mesh thread
 * via atomic flags + MESH_EVENT_UI_ACTION.
 */
void mesh_send_flood_advert(void);
void mesh_send_zerohop_advert(void);
void mesh_gps_set_enabled(bool enable);
void mesh_ble_set_enabled(bool enable);
void mesh_set_buzzer_quiet(bool quiet);
void mesh_set_offgrid_mode(bool enable);
void mesh_set_leds_disabled(bool disabled);
void mesh_disable_power_regulators(void);
void mesh_reboot_to_ota_dfu(void);
void mesh_save_brightness(uint8_t brightness);
void mesh_save_and_restart(void);
void mesh_set_wake_on_msg(bool enabled);
void mesh_save_screen_off_secs(uint16_t secs);
/* path_hash_mode: 0/1/2 → 1/2/3 bytes per hop appended to outbound flood path */
void mesh_save_path_hash_mode(uint8_t mode);
/* GPS duty-cycle interval in seconds (0 = always on). Applied live immediately
 * (lightweight); the flash write is deferred to the mesh thread. */
void mesh_save_gps_duty_sec(uint32_t sec);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_UI_MESH_ACTIONS_H */
