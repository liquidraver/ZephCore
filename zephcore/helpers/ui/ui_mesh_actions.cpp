/*
 * SPDX-License-Identifier: MIT
 * ZephCore UI ↔ Mesh Action Helpers
 *
 * Deferred UI button actions and periodic housekeeping display refresh.
 * Extracted from main_companion.cpp.
 *
 * This is a .cpp file because it accesses C++ mesh objects (CompanionMesh,
 * ZephyrDataStore, LoRaRadioBase, ZephyrBoard, ZephyrRTCClock).
 * The extern "C" wrappers are called from ui_task.c (C code).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_ui_actions, CONFIG_ZEPHCORE_UI_ACTIONS_LOG_LEVEL);

#include <app/CompanionMesh.h>
#include <ZephyrDataStore.h>
#include <adapters/radio/LoRaRadioBase.h>
#include <adapters/board/ZephyrBoard.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <ZephyrBLE.h>
#include <ZephyrSensorManager.h>
#include "ui_task.h"
#include <joystick_ui_hooks.h>
#include "ui_mesh_actions.h"

/* UI action bit flags — set from input thread, consumed by mesh event loop */
#define UI_ACTION_FLOOD_ADVERT      BIT(0)
#define UI_ACTION_GPS_TOGGLE        BIT(1)
#define UI_ACTION_BUZZER_TOGGLE     BIT(2)
#define UI_ACTION_ZEROHOP_ADVERT    BIT(3)
#define UI_ACTION_OFFGRID_TOGGLE    BIT(4)
#define UI_ACTION_LEDS_TOGGLE       BIT(5)
#define UI_ACTION_BLE_TOGGLE        BIT(6)
#define UI_ACTION_BRIGHTNESS_SAVE   BIT(7)
#define UI_ACTION_SAVE_RESTART      BIT(8)
#define UI_ACTION_WAKE_ON_MSG_SAVE  BIT(9)
#define UI_ACTION_SCREEN_OFF_SAVE   BIT(10)
#define UI_ACTION_PATH_HASH_MODE_SAVE BIT(11)

/* Module-local pointers, set by init */
static CompanionMesh *s_mesh;
static ZephyrDataStore *s_data_store;
static mesh::LoRaRadioBase *s_lora_radio;
static mesh::ZephyrBoard *s_board;
static mesh::ZephyrRTCClock *s_rtc_clock;
static struct k_event *s_mesh_events;
static uint32_t s_mesh_event_ui_action;

/* Atomic UI action flags — set from input thread, consumed in mesh event loop */
static atomic_t pending_ui_actions;

/* Pending state for toggle actions (pass the value to mesh thread).
 * Written before atomic_or on pending_ui_actions, read after atomic_clear,
 * so the atomic provides ordering. Using atomic_t for portability. */
static atomic_t pending_gps_enabled;
static atomic_t pending_buzzer_quiet;
static atomic_t pending_offgrid_enabled;
static atomic_t pending_leds_disabled;
static atomic_t pending_ble_disabled;
static atomic_t pending_brightness;
static atomic_t pending_wake_on_msg;
static atomic_t pending_screen_off_secs;
static atomic_t pending_path_hash_mode;

extern "C" void ui_mesh_actions_init(struct k_event *mesh_events,
				     uint32_t mesh_event_ui_action,
				     void *companion_mesh, void *data_store,
				     void *lora_radio, void *zephyr_board,
				     void *rtc_clock)
{
	s_mesh_events = mesh_events;
	s_mesh_event_ui_action = mesh_event_ui_action;
	s_mesh = static_cast<CompanionMesh *>(companion_mesh);
	s_data_store = static_cast<ZephyrDataStore *>(data_store);
	s_lora_radio = static_cast<mesh::LoRaRadioBase *>(lora_radio);
	s_board = static_cast<mesh::ZephyrBoard *>(zephyr_board);
	s_rtc_clock = static_cast<mesh::ZephyrRTCClock *>(rtc_clock);
}

/* ========== Button action wrappers (called from input thread) ========== */

extern "C" void mesh_send_flood_advert(void)
{
	atomic_or(&pending_ui_actions, UI_ACTION_FLOOD_ADVERT);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_send_zerohop_advert(void)
{
	atomic_or(&pending_ui_actions, UI_ACTION_ZEROHOP_ADVERT);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_gps_set_enabled(bool enable)
{
	/* Toggle GPS hardware immediately (lightweight, no flash) */
	gps_enable(enable);

	/* Defer the flash write (savePrefs) to mesh thread */
	atomic_set(&pending_gps_enabled, enable ? 1 : 0);
	atomic_or(&pending_ui_actions, UI_ACTION_GPS_TOGGLE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_ble_set_enabled(bool enable)
{
	zephcore_ble_set_enabled(enable);
	atomic_set(&pending_ble_disabled, enable ? 0 : 1);
	atomic_or(&pending_ui_actions, UI_ACTION_BLE_TOGGLE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_save_brightness(uint8_t brightness)
{
	atomic_set(&pending_brightness, (atomic_val_t)brightness);
	atomic_or(&pending_ui_actions, UI_ACTION_BRIGHTNESS_SAVE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_save_and_restart(void)
{
	atomic_or(&pending_ui_actions, UI_ACTION_SAVE_RESTART);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_set_wake_on_msg(bool enabled)
{
	atomic_set(&pending_wake_on_msg, enabled ? 1 : 0);
	atomic_or(&pending_ui_actions, UI_ACTION_WAKE_ON_MSG_SAVE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_save_screen_off_secs(uint16_t secs)
{
	atomic_set(&pending_screen_off_secs, (atomic_val_t)secs);
	atomic_or(&pending_ui_actions, UI_ACTION_SCREEN_OFF_SAVE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_save_path_hash_mode(uint8_t mode)
{
	atomic_set(&pending_path_hash_mode, (atomic_val_t)mode);
	atomic_or(&pending_ui_actions, UI_ACTION_PATH_HASH_MODE_SAVE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_set_buzzer_quiet(bool quiet)
{
	/* Defer the flash write (savePrefs) to mesh thread */
	atomic_set(&pending_buzzer_quiet, quiet ? 1 : 0);
	atomic_or(&pending_ui_actions, UI_ACTION_BUZZER_TOGGLE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_set_offgrid_mode(bool enable)
{
	/* Defer the flash write (savePrefs) to mesh thread */
	atomic_set(&pending_offgrid_enabled, enable ? 1 : 0);
	atomic_or(&pending_ui_actions, UI_ACTION_OFFGRID_TOGGLE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

extern "C" void mesh_set_leds_disabled(bool disabled)
{
	/* Defer the flash write (savePrefs) to mesh thread */
	atomic_set(&pending_leds_disabled, disabled ? 1 : 0);
	atomic_or(&pending_ui_actions, UI_ACTION_LEDS_TOGGLE);
	k_event_post(s_mesh_events, s_mesh_event_ui_action);
}

/* Disable power regulators for System OFF.
 * Only touches sensor power and buzzer power-gate regulators.
 * GPS is handled separately by gps_power_off_for_shutdown().
 * DO NOT touch BLE here — that corrupts controller state across reset. */
extern "C" void mesh_disable_power_regulators(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(sensor_power))
	const struct device *sensor_reg = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sensor_power));
	if (sensor_reg && device_is_ready(sensor_reg)) {
		regulator_disable(sensor_reg);
	}
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(buzzer_enable))
	const struct device *buzz_reg = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(buzzer_enable));
	if (buzz_reg && device_is_ready(buzz_reg)) {
		regulator_disable(buzz_reg);
	}
#endif
}

extern "C" void mesh_reboot_to_ota_dfu(void)
{
	char reply[64];
	s_board->startOTAUpdate(nullptr, reply);
	/* Never returns */
}

/* ========== Deferred action processing (runs on mesh thread) ========== */

extern "C" void mesh_handle_ui_actions(void)
{
	uint32_t actions = atomic_clear(&pending_ui_actions);

	if (!s_mesh || actions == 0) {
		return;
	}

	if (actions & (UI_ACTION_FLOOD_ADVERT | UI_ACTION_ZEROHOP_ADVERT)) {
		bool flood = !!(actions & UI_ACTION_FLOOD_ADVERT);
		/* Route through the shared canonical path so flood adverts honor
		 * prefs.path_hash_mode and the default transport scope, matching
		 * the mobile-app CMD_SEND_SELF_ADVERT behavior. */
		if (s_mesh->sendSelfAdvert(flood)) {
			LOG_INF("%s advert sent (button)",
				flood ? "flood" : "zero-hop");
		}
	}

	/* Save prefs if any toggle action changed them */
	bool need_save = false;

	if (actions & UI_ACTION_GPS_TOGGLE) {
		bool gps_en = atomic_get(&pending_gps_enabled) != 0;
		s_mesh->prefs.gps_enabled = gps_en ? 1 : 0;
		LOG_INF("GPS %s (button)", gps_en ? "on" : "off");
		need_save = true;
	}

	if (actions & UI_ACTION_BUZZER_TOGGLE) {
		bool bq = atomic_get(&pending_buzzer_quiet) != 0;
		s_mesh->prefs.buzzer_quiet = bq ? 1 : 0;
		LOG_INF("buzzer_quiet=%d (button)", bq);
		need_save = true;
	}

	if (actions & UI_ACTION_OFFGRID_TOGGLE) {
		bool og = atomic_get(&pending_offgrid_enabled) != 0;
		s_mesh->prefs.client_repeat = og ? 1 : 0;
		LOG_INF("client_repeat=%d (button)", og);
		need_save = true;
	}

	if (actions & UI_ACTION_LEDS_TOGGLE) {
		bool ld = atomic_get(&pending_leds_disabled) != 0;
		s_mesh->prefs.leds_disabled = ld ? 1 : 0;
		LOG_INF("leds_disabled=%d (button)", ld);
		need_save = true;
	}

	if (actions & UI_ACTION_BLE_TOGGLE) {
		bool bd = atomic_get(&pending_ble_disabled) != 0;
		s_mesh->prefs.ble_disabled = bd ? 1 : 0;
		LOG_INF("ble_disabled=%d (button)", bd);
		need_save = true;
	}

	if (actions & UI_ACTION_BRIGHTNESS_SAVE) {
		uint8_t br = (uint8_t)atomic_get(&pending_brightness);
		s_mesh->prefs.display_brightness = br;
		LOG_INF("display_brightness=%d (button)", br);
		need_save = true;
	}

	if (actions & UI_ACTION_WAKE_ON_MSG_SAVE) {
		s_mesh->prefs.wake_on_msg = atomic_get(&pending_wake_on_msg) ? 1 : 0;
		LOG_INF("wake_on_msg=%d (button)", s_mesh->prefs.wake_on_msg);
		need_save = true;
	}

	if (actions & UI_ACTION_SCREEN_OFF_SAVE) {
		s_mesh->prefs.screen_off_secs = (uint16_t)atomic_get(&pending_screen_off_secs);
		LOG_INF("screen_off_secs=%d (button)", s_mesh->prefs.screen_off_secs);
		need_save = true;
	}

	if (actions & UI_ACTION_PATH_HASH_MODE_SAVE) {
		uint8_t mode = (uint8_t)atomic_get(&pending_path_hash_mode);
		if (mode > 2) mode = 2;  /* clamp to valid range (0-2 → 1-3 bytes) */
		s_mesh->prefs.path_hash_mode = mode;
		LOG_INF("path_hash_mode=%d (%d bytes per hop)", mode, mode + 1);
		need_save = true;
	}

	if (need_save || (actions & UI_ACTION_SAVE_RESTART)) {
		s_data_store->savePrefs(s_mesh->prefs);
		LOG_INF("prefs saved (button action)");
	}

	if (actions & UI_ACTION_SAVE_RESTART) {
		LOG_INF("rebooting (save+restart action)");
		sys_reboot(SYS_REBOOT_COLD);
	}
}

/* ========== Periodic housekeeping UI refresh ========== */

extern "C" void mesh_housekeeping_ui_refresh(void)
{
	if (!s_mesh) {
		return;
	}

	/* Battery is now refreshed lazily from ui_pages_render() with a 30 s
	 * freshness guard — no periodic ADC fire here. */

	/* Update top bar clock from RTC */
	ui_set_clock(s_rtc_clock->getCurrentTime());

	ui_set_radio_params(
		s_lora_radio->getActiveFrequencyHz(),
		s_lora_radio->getActiveSpreadingFactor(),
		s_lora_radio->getActiveBandwidthKHzX10(),
		s_lora_radio->getActiveCodingRate(),
		s_lora_radio->getConfiguredTxPower(),
		s_lora_radio->getNoiseFloor());
	{
		bool apc_enabled = false;
		int8_t apc_reduction = 0;
		int16_t apc_margin_x10 = 0;
		uint8_t apc_target = s_mesh->prefs.apc_margin;

#ifdef CONFIG_ZEPHCORE_APC
		apc_enabled = s_mesh->isAPCEnabled();
		apc_reduction = s_mesh->getAPCReduction();
		apc_margin_x10 = (int16_t)(s_mesh->getAPCMargin() * 10.0f);
		apc_target = s_mesh->getAPCTargetMargin();
#endif
		ui_set_radio_runtime(
			s_lora_radio->getEffectiveTxPower(),
			apc_enabled,
			apc_reduction,
			apc_margin_x10,
			apc_target,
			s_lora_radio->getActiveSyncWord(),
			s_lora_radio->getActivePreambleLength(),
			s_lora_radio->isRxDutyCycleEnabled(),
			s_lora_radio->isRadioReady(),
			s_lora_radio->isInRecvMode(),
			s_lora_radio->isTxActive());
	}
	ui_set_radio_stats(
		s_lora_radio->getPacketsRecv(),
		s_lora_radio->getPacketsSent(),
		s_lora_radio->getPacketsRecvErrors());

	/* Update GPS satellite count even without fix */
	if (gps_is_enabled()) {
		struct gps_position gpos;

		gps_get_position(&gpos);
		if (!gpos.valid) {
			ui_set_gps_data(false, (uint8_t)gpos.satellites,
					0, 0, 0);
		}
	}

	/* Update GPS state machine info (state, last fix age, next search) */
	{
		struct gps_state_info gsi;

		gps_get_state_info(&gsi);
		ui_set_gps_state(gsi.state, gsi.last_fix_age_s, gsi.next_search_s);
	}

	/* Update recently heard contacts from mesh advert path table.
	 * getRecentlyHeard() returns the N most recent entries sorted
	 * descending by recv_timestamp (newest first).
	 * We compute age_s from RTC timestamps and pass to UI. */
	{
		AdvertPath recent_adverts[4];
		int num = s_mesh->getRecentlyHeard(recent_adverts, 4);
		uint32_t now_rtc = s_rtc_clock->getCurrentTime();

		ui_clear_recent();
		for (int i = num - 1; i >= 0; i--) {
			if (recent_adverts[i].name[0]) {
				/* Guard: if recv_timestamp > now_rtc (clock went
				 * backwards after time sync change), clamp to 0. */
				uint32_t age_s = 0;
				if (now_rtc >= recent_adverts[i].recv_timestamp) {
					age_s = now_rtc - recent_adverts[i].recv_timestamp;
				}
				ui_add_recent(recent_adverts[i].name, 0, age_s);
			}
		}
	}

	/* Update offline queue message count */
	ui_set_msg_count(s_mesh->getOfflineQueueCount());
}
