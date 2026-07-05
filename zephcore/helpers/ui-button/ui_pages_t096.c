/*
 * ZephCore - Heltec T096 Compact Color UI Renderer
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * This renderer is intentionally scoped to CONFIG_ZEPHCORE_UI_RENDERER_T096.
 * Generic color-capable boards continue through ui_pages.c unless they opt in
 * to their own compatible renderer.
 */

#include "ui_pages_t096.h"
#include "display.h"
#include "ui_task.h"

#include <zephyr/kernel.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define T096_FONT_W 6
#define T096_FONT_H 8
#define T096_CONTENT_Y 18
#define T096_LINE_H 10
#define T096_ACTIVITY_SAMPLES 16
#define T096_OFF_NOTICE_MS 320

#define T096_BG        MC_COLOR_BLACK
#define T096_PANEL     0x1082
#define T096_PANEL_2   0x2104
#define T096_TEXT      0xffde
#define T096_MUTED     0x8410
#define T096_DIM       0x4208
#define T096_GREEN     0x07e0
#define T096_AMBER     0xfdc0
#define T096_ORANGE    MC_COLOR_ORANGE
#define T096_RED       0xf800

static uint8_t activity_rx[T096_ACTIVITY_SAMPLES];
static uint8_t activity_tx[T096_ACTIVITY_SAMPLES];
static uint8_t activity_head;
static bool activity_initialized;
static uint32_t activity_last_rx;
static uint32_t activity_last_tx;
static uint32_t activity_last_sample_ms;

static struct k_work_delayable off_notice_work;
static bool off_notice_work_ready;
static bool off_notice_pending;

#ifdef ZEPHCORE_REPEATER
static const enum ui_page t096_active_pages[] = {
	UI_PAGE_STATUS,
	UI_PAGE_RADIO,
	UI_PAGE_SHUTDOWN,
};
#else
static const enum ui_page t096_active_pages[] = {
	UI_PAGE_MESSAGES,
	UI_PAGE_RECENT,
	UI_PAGE_RADIO,
	UI_PAGE_TRAFFIC,
	UI_PAGE_BLUETOOTH,
	UI_PAGE_ADVERT,
	UI_PAGE_GPS,
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	UI_PAGE_BUZZER,
#endif
	UI_PAGE_LEDS,
	UI_PAGE_SENSORS,
	UI_PAGE_OFFGRID,
	UI_PAGE_DFU,
	UI_PAGE_SHUTDOWN,
};
#endif

#define T096_ACTIVE_PAGE_COUNT \
	((int)(sizeof(t096_active_pages) / sizeof(t096_active_pages[0])))

void __real_ui_pages_render(void);
void __real_ui_prepare_for_system_off(void);

static int active_page_index(enum ui_page page)
{
	for (int i = 0; i < T096_ACTIVE_PAGE_COUNT; i++) {
		if (t096_active_pages[i] == page) {
			return i;
		}
	}

	return -1;
}

bool ui_pages_t096_renderer_available(void)
{
	return IS_ENABLED(CONFIG_ZEPHCORE_UI_RENDERER_T096) &&
	       mc_display_has_color() &&
	       mc_display_width() >= 120 && mc_display_width() <= 180 &&
	       mc_display_height() >= 64 && mc_display_height() <= 100;
}

static int text_w(const char *text)
{
	return (int)strlen(text) * T096_FONT_W;
}

static uint8_t battery_pct(const struct ui_state *st)
{
	if (st->battery_pct > 0) {
		return st->battery_pct;
	}
	if (st->battery_mv >= 4200) {
		return 100;
	}
	if (st->battery_mv <= 3000) {
		return 0;
	}
	return (uint8_t)((st->battery_mv - 3000) * 100 / 1200);
}

static void text_right(int y, const char *text, uint16_t color)
{
	int x = (int)mc_display_width() - text_w(text) - 2;

	if (x < 0) {
		x = 0;
	}
	mc_display_color_text(x, y, text, color);
}

static void text_centered(int y, const char *text, uint16_t color)
{
	int x = ((int)mc_display_width() - text_w(text)) / 2;

	if (x < 0) {
		x = 0;
	}
	mc_display_color_text(x, y, text, color);
}

static void fmt_count(char *buf, size_t len, uint32_t value)
{
	if (value >= 1000000U) {
		snprintf(buf, len, "%luM", (unsigned long)(value / 1000000U));
	} else if (value >= 1000U) {
		snprintf(buf, len, "%luk", (unsigned long)(value / 1000U));
	} else {
		snprintf(buf, len, "%lu", (unsigned long)value);
	}
}

static const char *radio_label(const struct ui_state *st)
{
	return st->lora_tx_active ? "TX" :
	       st->lora_in_rx ? "RX" :
	       st->lora_radio_ready ? "RDY" : "WAIT";
}

static uint16_t radio_color(const struct ui_state *st)
{
	return st->lora_tx_active ? T096_ORANGE :
	       st->lora_in_rx ? T096_GREEN :
	       st->lora_radio_ready ? T096_GREEN : T096_AMBER;
}

static int badge(int x, int y, const char *text, uint16_t color)
{
	int w = text_w(text) + 7;
	int max_x = (int)mc_display_width() - w;

	if (x > max_x) {
		x = max_x;
	}
	if (x < 0) {
		x = 0;
	}

	mc_display_color_fill_rect(x, y, w, T096_FONT_H + 2, T096_PANEL_2);
	mc_display_color_fill_rect(x, y, 2, T096_FONT_H + 2, color);
	mc_display_color_text(x + 4, y + 1, text, color);
	return x + w + 3;
}

static void metric_bar(int x, int y, int w, int h, int value, int max_value,
		       uint16_t color)
{
	if (w <= 0 || h <= 0) {
		return;
	}
	if (max_value <= 0) {
		max_value = 1;
	}
	if (value < 0) {
		value = 0;
	}
	if (value > max_value) {
		value = max_value;
	}

	int filled = (w * value) / max_value;

	mc_display_color_fill_rect(x, y, w, h, T096_DIM);
	if (filled > 0) {
		mc_display_color_fill_rect(x, y, filled, h, color);
	}
}

static void header(const char *title, const struct ui_state *st,
		   int page_index, int page_count)
{
	int w = mc_display_width();

	mc_display_color_fill_rect(0, 0, w, 13, T096_PANEL);
	mc_display_color_text(2, 2, title, T096_AMBER);

	if (st->battery_mv > 0) {
		char buf[8];
		uint8_t pct = battery_pct(st);
		uint16_t color = (pct <= 15) ? T096_RED :
				 (pct <= 30) ? T096_AMBER : T096_GREEN;

		snprintf(buf, sizeof(buf), "%u%%", pct);
		text_right(2, buf, color);
	}

	mc_display_color_fill_rect(0, 13, w, 1, T096_DIM);
	mc_display_color_fill_rect(0, 14, w, 1, T096_PANEL_2);
	if (page_count > 0) {
		int filled = (w * (page_index + 1)) / page_count;

		if (filled < 1) {
			filled = 1;
		}
		mc_display_color_fill_rect(0, 14, filled, 1, T096_AMBER);
	}
}

static void row_lr(int y, const char *label, const char *value,
		   uint16_t value_color)
{
	mc_display_color_text(2, y, label, T096_MUTED);
	text_right(y, value, value_color);
}

static void render_messages(const struct ui_state *st, int page_index,
			    int page_count)
{
	char buf[24];
	int y = T096_CONTENT_Y;
	int x = 2;
	const char *ble = !st->ble_enabled ? "BLE OFF" :
			  st->ble_connected ? "BLE OK" : "BLE ADV";
	uint16_t ble_color = !st->ble_enabled ? T096_MUTED :
			     st->ble_connected ? T096_GREEN : T096_AMBER;

	header("HOME", st, page_index, page_count);

	x = badge(x, y, ble, ble_color);
	x = badge(x, y, radio_label(st), radio_color(st));
	badge((int)mc_display_width() -
	      text_w(st->offgrid_enabled ? "GRID" : "LOCAL") - 7,
	      y, st->offgrid_enabled ? "GRID" : "LOCAL",
	      st->offgrid_enabled ? T096_GREEN : T096_MUTED);
	y += 13;

	mc_display_color_text(2, y, "MESSAGES", T096_MUTED);
	snprintf(buf, sizeof(buf), "MSG %u", st->msg_count);
	text_right(y, buf, st->msg_count ? T096_AMBER : T096_TEXT);
	y += T096_LINE_H + 1;

	text_centered(y, st->ble_connected ? "APP LINKED" : "WAITING APP",
		      st->ble_connected ? T096_GREEN : T096_AMBER);
	y += T096_LINE_H + 1;

	snprintf(buf, sizeof(buf), "OFFGRID %s", st->offgrid_enabled ? "ON" : "OFF");
	text_centered(y, buf, st->offgrid_enabled ? T096_GREEN : T096_MUTED);

	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);

	snprintf(buf, sizeof(buf), "UP %ud %uh %um",
		 up_s / 86400, (up_s % 86400) / 3600, (up_s % 3600) / 60);
	mc_display_color_text(2, (int)mc_display_height() - 10, buf, T096_MUTED);
}

static void render_radio(const struct ui_state *st, int page_index,
			 int page_count)
{
	char buf[28];
	char bw_buf[10];
	int y = T096_CONTENT_Y;
	uint32_t freq_mhz = st->lora_freq_hz / 1000000;
	uint32_t freq_frac = (st->lora_freq_hz % 1000000 + 500) / 1000;
	uint16_t bw_int = st->lora_bw_khz_x10 / 10;
	uint16_t bw_frac = st->lora_bw_khz_x10 % 10;
	int max_tx = st->lora_tx_power > 0 ? st->lora_tx_power : 22;
	int eff_tx = st->lora_effective_tx_power > 0 ?
		     st->lora_effective_tx_power : st->lora_tx_power;
	uint16_t tx_color = (st->lora_apc_enabled && st->lora_apc_reduction > 0)
			    ? T096_AMBER : T096_GREEN;

	header("RADIO", st, page_index, page_count);

	if (bw_frac) {
		snprintf(bw_buf, sizeof(bw_buf), "BW%u.%u", bw_int, bw_frac);
	} else {
		snprintf(bw_buf, sizeof(bw_buf), "BW%u", bw_int);
	}
	mc_display_color_text(2, y, "RF", T096_MUTED);
	snprintf(buf, sizeof(buf), "%u.%03u", freq_mhz, freq_frac);
	mc_display_color_text(22, y, buf, T096_TEXT);
	text_right(y, bw_buf, T096_TEXT);
	y += T096_LINE_H;

	snprintf(buf, sizeof(buf), "SF%u CR%u", st->lora_sf, st->lora_cr);
	mc_display_color_text(2, y, buf, T096_TEXT);
	snprintf(buf, sizeof(buf), "P%u", st->lora_preamble_len);
	text_right(y, buf, T096_MUTED);
	y += T096_LINE_H;

	badge(2, y, radio_label(st), radio_color(st));
	text_right(y + 1, st->lora_rx_duty_cycle ? "DUTY RX" : "CONT RX",
		   st->lora_rx_duty_cycle ? T096_AMBER : T096_GREEN);
	y += T096_LINE_H + 2;

	mc_display_color_text(2, y, "TX", T096_MUTED);
	if (st->lora_apc_enabled) {
		snprintf(buf, sizeof(buf), "%d/%ddBm", eff_tx, st->lora_tx_power);
	} else {
		snprintf(buf, sizeof(buf), "%ddBm", st->lora_tx_power);
	}
	mc_display_color_text(22, y, buf, tx_color);
	metric_bar(86, y + 2, (int)mc_display_width() - 88, 5,
		   eff_tx, max_tx, tx_color);
	y += T096_LINE_H;

	if (st->lora_apc_enabled) {
		snprintf(buf, sizeof(buf), "ON R%d M%d.%d",
			 st->lora_apc_reduction,
			 st->lora_apc_margin_x10 / 10,
			 abs(st->lora_apc_margin_x10 % 10));
	} else {
		snprintf(buf, sizeof(buf), "OFF");
	}
	row_lr(y, "APC", buf,
	       !st->lora_apc_enabled ? T096_MUTED :
	       st->lora_apc_reduction > 0 ? T096_AMBER : T096_GREEN);
	y += T096_LINE_H;

	char rx_count[6];
	char tx_count[6];
	char err_count[6];

	fmt_count(rx_count, sizeof(rx_count), st->lora_packets_rx);
	fmt_count(tx_count, sizeof(tx_count), st->lora_packets_tx);
	fmt_count(err_count, sizeof(err_count), st->lora_packets_err);
	snprintf(buf, sizeof(buf), "NF%d", st->lora_noise_floor);
	mc_display_color_text(2, y, buf, T096_MUTED);
	snprintf(buf, sizeof(buf), "R%s T%s E%s", rx_count, tx_count, err_count);
	text_right(y, buf, st->lora_packets_err ? T096_RED : T096_TEXT);
}

static uint8_t clamp_delta(uint32_t delta)
{
	return (delta > 15U) ? 15U : (uint8_t)delta;
}

static void sample_activity(const struct ui_state *st)
{
	uint32_t now = k_uptime_get_32();
	uint32_t rx_delta = 0;
	uint32_t tx_delta = 0;

	if (!activity_initialized) {
		activity_last_rx = st->lora_packets_rx;
		activity_last_tx = st->lora_packets_tx;
		activity_last_sample_ms = now;
		activity_initialized = true;
		return;
	}

	if (st->lora_packets_rx >= activity_last_rx) {
		rx_delta = st->lora_packets_rx - activity_last_rx;
	}
	if (st->lora_packets_tx >= activity_last_tx) {
		tx_delta = st->lora_packets_tx - activity_last_tx;
	}
	if (rx_delta == 0 && tx_delta == 0 &&
	    (now - activity_last_sample_ms) < 5000U) {
		return;
	}

	activity_last_rx = st->lora_packets_rx;
	activity_last_tx = st->lora_packets_tx;
	activity_last_sample_ms = now;
	activity_head = (activity_head + 1U) % T096_ACTIVITY_SAMPLES;
	activity_rx[activity_head] = clamp_delta(rx_delta);
	activity_tx[activity_head] = clamp_delta(tx_delta);
}

static void draw_activity_graph(int x, int y, int w, int h,
				const struct ui_state *st)
{
	uint8_t max_v = 0;
	int mid = y + h / 2;
	int col_w = w / T096_ACTIVITY_SAMPLES;

	if (col_w < 1) {
		col_w = 1;
	}

	sample_activity(st);

	mc_display_color_fill_rect(x, y, w, h, T096_BG);
	mc_display_color_fill_rect(x, mid, w, 1, T096_DIM);

	for (int i = 0; i < T096_ACTIVITY_SAMPLES; i++) {
		uint8_t idx = (uint8_t)((activity_head + 1U + i) %
					T096_ACTIVITY_SAMPLES);

		if (activity_rx[idx] > max_v) {
			max_v = activity_rx[idx];
		}
		if (activity_tx[idx] > max_v) {
			max_v = activity_tx[idx];
		}
	}

	if (max_v == 0) {
		mc_display_color_text(x + w - 30, y + 1, "RX", T096_GREEN);
		mc_display_color_text(x + w - 14, y + 1, "TX", T096_ORANGE);
		return;
	}

	for (int i = 0; i < T096_ACTIVITY_SAMPLES; i++) {
		uint8_t idx = (uint8_t)((activity_head + 1U + i) %
					T096_ACTIVITY_SAMPLES);
		int bx = x + i * col_w;
		int draw_w = (col_w > 1) ? col_w - 1 : 1;
		int tx_h = ((h / 2 - 1) * activity_tx[idx]) / max_v;
		int rx_h = ((h / 2 - 1) * activity_rx[idx]) / max_v;

		if (tx_h > 0) {
			mc_display_color_fill_rect(bx, mid - tx_h, draw_w, tx_h,
						   T096_ORANGE);
		}
		if (rx_h > 0) {
			mc_display_color_fill_rect(bx, mid + 1, draw_w, rx_h,
						   T096_GREEN);
		}
	}

	mc_display_color_text(x + w - 30, y + 1, "RX", T096_GREEN);
	mc_display_color_text(x + w - 14, y + 1, "TX", T096_ORANGE);
}

static void render_traffic(const struct ui_state *st, int page_index,
			   int page_count)
{
	char rx_count[6];
	char tx_count[6];
	char err_count[6];
	char buf[12];
	int y = T096_CONTENT_Y;
	int x = 2;
	int graph_y;
	int graph_h;

	header("TRAFFIC", st, page_index, page_count);

	fmt_count(rx_count, sizeof(rx_count), st->lora_packets_rx);
	fmt_count(tx_count, sizeof(tx_count), st->lora_packets_tx);
	fmt_count(err_count, sizeof(err_count), st->lora_packets_err);

	snprintf(buf, sizeof(buf), "RX %s", rx_count);
	x = badge(x, y, buf, T096_GREEN);
	snprintf(buf, sizeof(buf), "TX %s", tx_count);
	x = badge(x, y, buf, T096_ORANGE);
	if (st->lora_packets_err > 0) {
		snprintf(buf, sizeof(buf), "E %s", err_count);
		badge(x, y, buf, T096_RED);
	}

	graph_y = y + 14;
	graph_h = (int)mc_display_height() - graph_y - 3;
	if (graph_h < 22) {
		graph_h = 22;
	}
	draw_activity_graph(2, graph_y, (int)mc_display_width() - 4,
			    graph_h, st);
}

static bool shutdown_confirm_active(const struct ui_state *st)
{
	return st->shutdown_confirm_time != 0 &&
	       (k_uptime_get_32() - st->shutdown_confirm_time) <=
	       CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS;
}

static void draw_sad_face(int cx, int y, bool waving)
{
	mc_display_color_text(cx - 9, y, ":-(", T096_AMBER);
	if (!waving) {
		return;
	}

	int hand_x = cx + 18;

	mc_display_color_fill_rect(cx + 11, y + 9, 12, 3, T096_AMBER);
	mc_display_color_fill_rect(hand_x, y + 3, 3, 10, T096_TEXT);
	mc_display_color_fill_rect(hand_x + 5, y, 3, 8, T096_TEXT);
	mc_display_color_fill_rect(hand_x + 10, y + 3, 3, 7, T096_TEXT);
	mc_display_color_fill_rect(hand_x + 16, y + 1, 2, 2, T096_ORANGE);
	mc_display_color_fill_rect(hand_x + 19, y + 7, 2, 2, T096_ORANGE);
}

static void render_shutdown(const struct ui_state *st, int page_index,
			    int page_count)
{
	bool confirm = shutdown_confirm_active(st);
	int y = T096_CONTENT_Y;

	header("SYSTEM OFF", st, page_index, page_count);

	if (confirm) {
		text_centered(y, "CONFIRM OFF", T096_AMBER);
		draw_sad_face((int)mc_display_width() / 2, y + 18, false);
		text_centered((int)mc_display_height() - 11, "press again",
			      T096_MUTED);
		return;
	}

	badge(2, y, "PWR", T096_RED);
	mc_display_color_text(38, y + 1, "system off", T096_TEXT);
	text_centered(y + 24, "press to select", T096_MUTED);
}

bool ui_pages_t096_render(enum ui_page page, const struct ui_state *st,
			  int page_index, int page_count)
{
	if (!ui_pages_t096_renderer_available() || !st) {
		return false;
	}

	switch (page) {
	case UI_PAGE_MESSAGES:
		render_messages(st, page_index, page_count);
		return true;
	case UI_PAGE_RADIO:
		render_radio(st, page_index, page_count);
		return true;
	case UI_PAGE_TRAFFIC:
		render_traffic(st, page_index, page_count);
		return true;
	case UI_PAGE_SHUTDOWN:
		render_shutdown(st, page_index, page_count);
		return true;
	default:
		return false;
	}
}

bool ui_pages_t096_render_system_off_notice(void)
{
	int w;
	int h;

	if (!ui_pages_t096_renderer_available()) {
		return false;
	}

	w = mc_display_width();
	h = mc_display_height();
	mc_display_clear();
	mc_display_color_fill_rect(0, 0, w, h, T096_BG);
	text_centered(14, "SYSTEM OFF", T096_AMBER);
	draw_sad_face(w / 2, h / 2 - 4, true);
	text_centered(h - 11, "73", T096_MUTED);
	mc_display_finalize();
	return true;
}

void __wrap_ui_pages_render(void)
{
	enum ui_page page = ui_pages_current();
	struct ui_state *st = ui_pages_get_state();
	int page_index = active_page_index(page);

	if (st && page_index >= 0 && ui_pages_t096_renderer_available()) {
		ui_refresh_battery();
		mc_display_clear();
		if (ui_pages_t096_render(page, st, page_index,
					 T096_ACTIVE_PAGE_COUNT)) {
			mc_display_finalize();
			return;
		}
	}

	__real_ui_pages_render();
}

void __wrap_ui_prepare_for_system_off(void)
{
	if (ui_pages_t096_render_system_off_notice()) {
		k_sleep(K_MSEC(220));
	}

	__real_ui_prepare_for_system_off();
}

static void render_off_notice(void)
{
	int w = mc_display_width();
	int h = mc_display_height();
	int cx = w / 2;
	int y = h / 2 - 12;

	mc_display_clear();
	mc_display_color_fill_rect(0, 0, w, h, T096_BG);
	mc_display_color_text(cx - 9, y, "73", T096_AMBER);

	/* Tiny hand built from rectangles: no bitmap asset, no heap, no delay. */
	y += 14;
	mc_display_color_fill_rect(cx - 18, y + 8, 26, 8, T096_AMBER);
	mc_display_color_fill_rect(cx - 12, y + 3, 4, 8, T096_TEXT);
	mc_display_color_fill_rect(cx - 6, y, 4, 11, T096_TEXT);
	mc_display_color_fill_rect(cx, y + 2, 4, 9, T096_TEXT);
	mc_display_color_fill_rect(cx + 6, y + 6, 4, 8, T096_TEXT);
	mc_display_color_fill_rect(cx - 23, y + 4, 3, 3, T096_ORANGE);
	mc_display_color_fill_rect(cx + 16, y + 2, 3, 3, T096_ORANGE);

	mc_display_color_text(cx - 9, h - 11, "bye", T096_MUTED);
	mc_display_finalize();
}

static void off_notice_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	off_notice_pending = false;
	mc_display_off();
}

void ui_t096_display_cancel_auto_off_notice(void)
{
	if (!off_notice_work_ready || !off_notice_pending) {
		return;
	}

	k_work_cancel_delayable(&off_notice_work);
	off_notice_pending = false;
}

bool ui_t096_display_auto_off_notice(void)
{
	if (!ui_pages_t096_renderer_available()) {
		return false;
	}

	if (!off_notice_work_ready) {
		k_work_init_delayable(&off_notice_work, off_notice_work_handler);
		off_notice_work_ready = true;
	}

	if (off_notice_pending) {
		return true;
	}

	render_off_notice();
	off_notice_pending = true;
	k_work_reschedule(&off_notice_work, K_MSEC(T096_OFF_NOTICE_MS));
	return true;
}
