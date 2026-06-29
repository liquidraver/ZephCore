/*
 * ZephCore - Joystick UI Hooks
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * JoystickUIHooks: ui_task.h implementation for the joystick companion UI.
 *
 * Replaces helpers/ui/ui_task.c when CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK=y.
 * CompanionMesh and ui_mesh_actions call the same C API unchanged.
 * This file forwards relevant calls to the registered JoystickUITask instance.
 *
 * Hardware initialization (display, buzzer, LED heartbeat) still happens in
 * ui_init() so that main_companion.cpp requires no extra #ifdefs.
 * Input handling is owned by JoystickUITask's INPUT_CALLBACK_DEFINE,
 * so no second input callback is registered here.
 */

#include "joystick_ui_hooks.h"
#include "joystick_ui_task.h"
#include <helpers/ui/ui_task.h>

#ifdef CONFIG_ZEPHCORE_UI_BUZZER
#include "buzzer.h"
#endif

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
#include "display.h"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(joystick_ui_hooks, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ===== Registered task and signal callbacks ===== */

static JoystickUITask *s_task;
static ui_signal_fn s_signal_refresh;
static ui_signal_fn s_signal_tx;

static void joystick_render_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	if (s_signal_refresh) s_signal_refresh();
}

K_TIMER_DEFINE(s_joystick_render_timer, joystick_render_timer_fn, NULL);

static void joystick_schedule_render(uint32_t delay_ms)
{
	k_timer_start(&s_joystick_render_timer, K_MSEC(delay_ms), K_NO_WAIT);
}

extern "C" void joystick_ui_hooks_register(
	void *task,
	ui_signal_fn signal_refresh,
	ui_signal_fn signal_tx
){
	s_task = static_cast<JoystickUITask *>(task);
	s_signal_refresh = signal_refresh;
	s_signal_tx = signal_tx;
	JoystickUITask::setSignalFn(signal_refresh);
	JoystickUITask::setScheduleRenderFn(joystick_schedule_render);
}

extern "C" void ui_signal_refresh(void)
{
	if (s_signal_refresh) {
		s_signal_refresh();
	}
}

extern "C" void ui_signal_tx(void)
{
	if (s_signal_tx) {
		s_signal_tx();
	}
}

/* ===== ui_task.h implementation ===== */

extern "C" int ui_init(void)
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	int ret = buzzer_init();
	if (ret == 0) {
		LOG_INF("buzzer ready");
	} else if (ret == -ENODEV) {
		LOG_INF("no buzzer hardware");
	} else {
		LOG_WRN("buzzer init failed: %d", ret);
	}
#endif

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	int dret = mc_display_init();
	if (dret == 0) {
		LOG_INF("display ready");
		mc_display_reset_auto_off();
	} else if (dret == -ENODEV) {
		LOG_INF("no display hardware");
	} else {
		LOG_WRN("display init failed: %d", dret);
	}
#endif

	ui_led_heartbeat_init();
	LOG_INF("joystick UI hooks initialized");
	return 0;
}

extern "C" void ui_notify(enum ui_event event)
{
	if (!s_task) {
		return;
	}

	switch (event) {
	case UI_EVENT_BLE_CONNECTED:
		s_task->setBLEConnected(true);
		s_task->notify();
		break;

	case UI_EVENT_BLE_DISCONNECTED:
		s_task->setBLEConnected(false);
		s_task->notify();
		break;

	case UI_EVENT_ACK:
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
		if (!buzzer_is_quiet()) {
			buzzer_play("ack:d=32,o=6,b=200:e,g");
		}
#endif
		s_task->notify();
		break;

	default:
		break;
	}
}

extern "C" void ui_notify_contact_msg(uint8_t path_len, const char *from_name,
	const char *text, uint16_t msg_count)
{
	if (s_task) {
		s_task->newMsg(path_len, from_name, text, (int)msg_count);
	}
}

extern "C" void ui_notify_channel_msg(const char *channel_name, const char *text,
	uint32_t ts, uint8_t path_len, uint16_t msg_count)
{
	(void)msg_count;
	if (s_task) {
		s_task->newChannelMsg(channel_name, text, ts, path_len);
	}
}

extern "C" void ui_notify_joystick_event(
	enum ui_joystick_event_type type,
	const uint8_t *pub_key_prefix,
	const void *payload
){
	if (!s_task) {
		return;
	}
	switch (type) {
	case UI_JOYSTICK_LOGIN_RESULT: {
		const auto *d = static_cast<const struct ui_joystick_login_data *>(payload);
		s_task->onRepeaterAdminLoginResult(pub_key_prefix, d->success, d->permissions, d->server_time);
		break;
	}
	case UI_JOYSTICK_CLI_RESPONSE: {
		const auto *d = static_cast<const struct ui_joystick_cli_data *>(payload);
		s_task->onRepeaterAdminCliResponse(pub_key_prefix, d->text);
		break;
	}
	case UI_JOYSTICK_REQ_RESPONSE: {
		const auto *d = static_cast<const struct ui_joystick_req_response_data *>(payload);
		s_task->onRepeaterReqResponse(pub_key_prefix, d->snr_local, d->snr_remote, d->data, d->data_len);
		break;
	}
	case UI_JOYSTICK_DISCOVER_RESP: {
		const auto *d = static_cast<const struct ui_joystick_discover_data *>(payload);
		s_task->onRepeaterDiscoverResp(pub_key_prefix, d->snr, d->snr_remote, d->path_len);
		break;
	}
	}
}

extern "C" void ui_notify_packet_sent(void)
{
	if (s_task) {
		s_task->notifyPacketSent();
	}
}

extern "C" void ui_set_ble_status(bool connected, const char *name)
{
	(void)name;
	if (s_task) {
		s_task->setBLEConnected(connected);
		s_task->notify();
	}
}

extern "C" void ui_set_radio_params(
	uint32_t freq_hz,
	uint8_t sf,
	uint16_t bw_khz_x10,
	uint8_t cr, int8_t tx_power, int16_t noise_floor)
{
	(void)freq_hz; (void)sf; (void)bw_khz_x10; (void)cr; (void)tx_power;
	if (s_task) {
		s_task->setNoiseFloor(noise_floor);
	}
}

extern "C" void ui_set_radio_runtime(int8_t effective_tx_power, bool apc_enabled,
	int8_t apc_reduction, int16_t apc_margin_x10,
	uint8_t apc_target_margin, uint8_t sync_word,
	uint16_t preamble_len, bool rx_duty_cycle,
	bool radio_ready, bool in_rx, bool tx_active)
{
	(void)effective_tx_power; (void)apc_enabled; (void)apc_reduction;
	(void)apc_margin_x10; (void)apc_target_margin; (void)sync_word;
	(void)preamble_len; (void)rx_duty_cycle; (void)radio_ready;
	(void)in_rx; (void)tx_active;
}

extern "C" void ui_set_radio_stats(uint32_t pkt_recv, uint32_t pkt_sent,
	uint32_t pkt_errors)
{
	ui_notify_radio_stats(pkt_recv, pkt_sent, pkt_errors);
}

extern "C" void ui_notify_radio_stats(uint32_t pkt_recv, uint32_t pkt_sent, uint32_t pkt_errors)
{
	if (s_task) {
		s_task->setPacketStats(pkt_recv, pkt_sent, pkt_errors);
	}
}

extern "C" bool ui_joystick_try_match_ack(uint32_t ack, uint8_t out_pubkey[6])
{
	if (!s_task) return false;
	ContactInfo *c = s_task->tryMatchPendingAck(ack);
	if (!c) return false;
	memcpy(out_pubkey, c->id.pub_key, 6);
	return true;
}

extern "C" void ui_set_battery(uint16_t mv, uint8_t /*pct*/)
{
	if (s_task) {
		s_task->setCachedBattMilliVolts(mv);
	}
}

extern "C" void ui_set_ble_enabled(bool enabled)
{
	if (s_task) {
		s_task->setBLEEnabled(enabled);
	}
}

extern "C" void ui_refresh_display(void)
{
	if (s_task) {
		s_task->forceRefresh();
	}
}


/* ===== Pull-model no-ops =====
 * The joystick UI reads all hardware state on demand at render time, so these
 * push-model hooks from the old button UI have no work to do. */
extern "C" void ui_set_gps_data(bool, uint8_t, int32_t, int32_t, int32_t) {}
extern "C" void ui_set_clock(uint32_t) {}
extern "C" void ui_add_recent(const char *, int16_t, uint32_t) {}
extern "C" void ui_set_node_name(const char *) {}
extern "C" void ui_clear_recent(void) {}
extern "C" void ui_set_sensor_data(int16_t, uint32_t, uint16_t, uint16_t) {}
extern "C" void ui_set_gps_available(bool) {}
extern "C" void ui_set_gps_enabled(bool) {}
extern "C" void ui_set_gps_state(uint8_t, uint32_t, uint32_t) {}
extern "C" void ui_set_buzzer_quiet(bool) {}
extern "C" void ui_set_offgrid_mode(bool) {}
extern "C" void ui_set_msg_count(uint16_t count) {}
