/*
 * SPDX-License-Identifier: Apache-2.0
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
#include "ui_mesh_actions.h"

/* UI action bit flags — set from input thread, consumed by mesh event loop */
#define UI_ACTION_FLOOD_ADVERT   BIT(0)
#define UI_ACTION_GPS_TOGGLE     BIT(1)
#define UI_ACTION_BUZZER_TOGGLE  BIT(2)
#define UI_ACTION_ZEROHOP_ADVERT BIT(3)
#define UI_ACTION_OFFGRID_TOGGLE BIT(4)

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
		mesh::Packet *adv;
		if (s_mesh->prefs.advert_loc_policy == 0) {
			adv = s_mesh->createSelfAdvert(
				s_mesh->prefs.node_name);
		} else {
			adv = s_mesh->createSelfAdvert(
				s_mesh->prefs.node_name,
				s_mesh->prefs.node_lat,
				s_mesh->prefs.node_lon);
		}
		if (adv) {
			if (flood) {
				s_mesh->sendFlood(adv);
				LOG_INF("flood advert sent (button)");
			} else {
				s_mesh->sendZeroHop(adv);
				LOG_INF("zero-hop advert sent (button)");
			}
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

	if (need_save) {
		s_data_store->savePrefs(s_mesh->prefs);
		LOG_INF("prefs saved (button action)");
	}
}

/* ========== Periodic housekeeping UI refresh ========== */

extern "C" void mesh_housekeeping_ui_refresh(void)
{
	if (!s_mesh) {
		return;
	}

	/* Battery voltage changes slowly — read every ~60s (12 × 5s housekeeping)
	 * instead of every 5s. Saves power (regulator toggle + ADC) and log noise. */
	static uint8_t batt_counter;
	if (++batt_counter >= 12) {
		batt_counter = 0;
		ui_set_battery(s_board->getBattMilliVolts(), 0);
	}

	/* Update top bar clock from RTC */
	ui_set_clock(s_rtc_clock->getCurrentTime());

	ui_set_radio_params(
		(uint32_t)(s_mesh->prefs.freq * 1000000.0f + 0.5f),
		s_mesh->prefs.sf,
		(uint16_t)(s_mesh->prefs.bw * 10.0f + 0.5f),
		s_mesh->prefs.cr,
		s_mesh->prefs.tx_power_dbm,
		s_lora_radio->getNoiseFloor());

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

	/* Read environment sensors if available */
	if (env_sensors_available()) {
		struct env_data edata;

		if (env_sensors_read(&edata) == 0) {
			ui_set_sensor_data(
				edata.has_temperature ? (int16_t)(edata.temperature_c * 10) : 0,
				edata.has_pressure ? (uint32_t)(edata.pressure_hpa * 100) : 0,
				edata.has_humidity ? (uint16_t)(edata.humidity_pct * 10) : 0,
				0);
		}
	}

	/* Trigger display refresh */
	ui_refresh_display();
}
