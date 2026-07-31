/*
 * ZephCore - Joystick UI Hooks (public API)
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Mesh layer → joystick UI bridge. Real implementations live in
 * helpers/ui-joystick/joystick_ui_hooks.cpp when CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK=y.
 * Inline no-ops below keep call sites free of #ifdef when joystick UI is off.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ui_signal_fn)(void);

enum ui_joystick_event_type {
	UI_JOYSTICK_LOGIN_RESULT   = 0,
	UI_JOYSTICK_CLI_RESPONSE   = 1,
	UI_JOYSTICK_REQ_RESPONSE   = 2,
	UI_JOYSTICK_DISCOVER_RESP  = 3,
};

struct ui_joystick_login_data {
	bool success;
	uint8_t permissions;
	uint32_t server_time;
};

struct ui_joystick_cli_data {
	const char *text;
};

struct ui_joystick_req_response_data {
	int8_t snr_local;
	int8_t snr_remote;
	const uint8_t *data;
	uint8_t data_len;
};

struct ui_joystick_discover_data {
	int8_t snr;
	int8_t snr_remote;
	uint8_t path_len;
};

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)

void joystick_ui_hooks_register(void *task,
	ui_signal_fn signal_refresh,
	ui_signal_fn signal_tx);

void ui_notify_joystick_event(enum ui_joystick_event_type type,
	const uint8_t *pub_key_prefix,
	const void *payload);

void ui_notify_radio_stats(uint32_t pkt_recv, uint32_t pkt_sent, uint32_t pkt_errors);

void ui_signal_refresh(void);
void ui_signal_tx(void);

bool ui_joystick_try_match_ack(uint32_t ack, uint8_t out_pubkey[6]);

#else

static inline void joystick_ui_hooks_register(void *task,
	ui_signal_fn signal_refresh,
	ui_signal_fn signal_tx)
{
	ARG_UNUSED(task);
	ARG_UNUSED(signal_refresh);
	ARG_UNUSED(signal_tx);
}

static inline void ui_notify_joystick_event(enum ui_joystick_event_type type,
	const uint8_t *pub_key_prefix,
	const void *payload)
{
	ARG_UNUSED(type);
	ARG_UNUSED(pub_key_prefix);
	ARG_UNUSED(payload);
}

static inline void ui_notify_radio_stats(uint32_t pkt_recv, uint32_t pkt_sent,
	uint32_t pkt_errors)
{
	ARG_UNUSED(pkt_recv);
	ARG_UNUSED(pkt_sent);
	ARG_UNUSED(pkt_errors);
}

static inline void ui_signal_refresh(void) {}
static inline void ui_signal_tx(void) {}

static inline bool ui_joystick_try_match_ack(uint32_t ack, uint8_t out_pubkey[6])
{
	ARG_UNUSED(ack);
	ARG_UNUSED(out_pubkey);
	return false;
}

#endif /* CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK */

#ifdef __cplusplus
}
#endif
