/*
 * ZephCore - UI Page Renderers
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Multi-page UI system matching Arduino's ui-new HomeScreen implementation.
 * Each page renders via the display abstraction (any resolution/display type).
 *
 * Layout is resolution-independent — all positions derived from actual
 * display dimensions and font size queried at runtime:
 *   Top bar:    Node name left, battery right
 *   Dots row:   Page indicator dots (centered)
 *   Separator:  Horizontal line
 *   Content:    Page-specific rendering
 */

#include "ui_pages.h"
#include "ui_task.h"
#include "display.h"

#include <time_sync.h>
#include <ZephyrSensorManager.h>

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ui_pages, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ZephCore logo bitmap is defined in helpers/ui/ui_common.c and shared with
 * the joystick UI's splash. See display.h for declaration. */

/* ========== Layout ========== */
/* All layout is derived from the actual display and font dimensions
 * queried at runtime, so the UI adapts to any resolution/font size.
 *
 * Naming convention: UPPER_CASE macros call display getters (cheap —
 * they just return a cached static).  This keeps page renderers
 * readable while being resolution-independent.
 *
 * Layout (top to bottom):
 *   Row 0..FONT_H-1:  Top bar (node name left, battery right)
 *   DOTS_Y..SEP_Y:    Page indicator dots + separator line
 *   CONTENT_Y..end:    Page-specific content
 */
#define FONT_W       mc_display_font_width()
#define FONT_H       mc_display_font_height()
#define DISP_W       mc_display_width()
#define DISP_H       mc_display_height()
#define LINE_H       (FONT_H + 2)           /* font height + 2px gap */
#define TOP_BAR_Y    0
#define DOTS_Y       (FONT_H + 2)           /* just below top bar */
#define SEP_Y        (DOTS_Y + 4)           /* below dots */
#define CONTENT_Y    (SEP_Y + 4)            /* below separator */
#define MAX_CHARS    (DISP_W / (FONT_W ? FONT_W : 1))

/* Compact RGB565 palette for small color TFTs.  These are used only through
 * mc_display_has_color(); monochrome displays keep the existing CFB path. */
#define UI_COLOR_HEADER_BG  0x018c  /* deep teal/blue */
#define UI_COLOR_TITLE      0x5dff  /* readable cyan */
#define UI_COLOR_LABEL      0x8c71  /* muted blue-gray */
#define UI_COLOR_VALUE      MC_COLOR_WHITE
#define UI_COLOR_OK         0x07e0
#define UI_COLOR_ACTIVE     0x05ff
#define UI_COLOR_WARN       0xffa0
#define UI_COLOR_ERROR      MC_COLOR_RED
#define UI_COLOR_DIM        0x4208
#define UI_COLOR_DISABLED   MC_COLOR_GRAY
#define UI_COLOR_TX         MC_COLOR_ORANGE
#define UI_COLOR_RX         UI_COLOR_ACTIVE

#define COLOR_FONT_W 6
#define COLOR_FONT_H 8
#define ACTIVITY_GRAPH_SAMPLES 16

/* Vertically center `total` rows of text within the content area and
 * return the y pixel position for row `idx` (0-based).  This replaces
 * the old hand-picked offsets (CONTENT_Y+16, +28, +34, ...) that were
 * calibrated for the 6x8 font and broke when switching to 10x16.
 * Works for any font size / display resolution. */
static inline int centered_row(int idx, int total)
{
	int content_h = (int)DISP_H - CONTENT_Y;
	int used_h = total * LINE_H;
	int top = (content_h - used_h) / 2;

	if (top < 0) {
		top = 0;
	}
	return CONTENT_Y + top + idx * LINE_H;
}

/* ========== Global State ========== */
static struct ui_state state;

static uint8_t activity_rx[ACTIVITY_GRAPH_SAMPLES];
static uint8_t activity_tx[ACTIVITY_GRAPH_SAMPLES];
static uint8_t activity_head;
static bool activity_initialized;
static uint32_t activity_last_rx;
static uint32_t activity_last_tx;
static uint32_t activity_last_sample_ms;

/* ========== Active Pages (role-dependent) ========== */
/* Repeater: minimal pages — status, radio, shutdown.
 * Companion: full page set matching Arduino UI.
 * Navigation walks this array instead of the raw enum. */

#ifdef ZEPHCORE_REPEATER
static const enum ui_page active_pages[] = {
	UI_PAGE_STATUS,
	UI_PAGE_RADIO,
	UI_PAGE_SHUTDOWN,
};
#else
static const enum ui_page active_pages[] = {
	UI_PAGE_MESSAGES,
	UI_PAGE_RECENT,
	UI_PAGE_RADIO,
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

#define ACTIVE_PAGE_COUNT ((int)(sizeof(active_pages) / sizeof(active_pages[0])))

/* Current index into active_pages[] */
static int current_page_idx;

/* ========== Helper: Battery Percentage from mV ========== */

static uint8_t calc_battery_pct(uint16_t mv)
{
	if (mv >= 4200) {
		return 100;
	}
	if (mv <= 3000) {
		return 0;
	}
	return (uint8_t)((mv - 3000) * 100 / 1200);
}

/* ========== Helper: Top Bar (Node Name + Battery) ========== */

/* One-char tag for where the displayed clock was last freshly synced from
 * (see time_sync_get_source() for the freshness window). Always returns a
 * letter — falls back to 'L' (local) when no recent external sync. */
static char time_source_tag(enum time_sync_source src)
{
	switch (src) {
	case TIME_SYNC_GPS:  return 'G';  /* GPS fix */
	case TIME_SYNC_APP:  return 'A';  /* phone/companion app */
	case TIME_SYNC_WIFI: return 'N';  /* network (SNTP) */
	default:             return 'L';  /* local: manual/CLI or stale/none */
	}
}

static void render_top_bar(void)
{
	bool color = mc_display_has_color();
	/* Right side: "HH:MM<S> XX%" — clock (with 1-char source tag) then
	 * battery, right-aligned.  Built first so the node name can be clipped
	 * to the space that remains on the left, preventing overlap. */
	char right[16] = "";
	int pos = 0;

	/* 24h clock from RTC epoch (only if time has been synced).
	 * Before sync, getCurrentTime() returns bare uptime (~seconds),
	 * so check for a sane epoch (after Jan 1 2025 = 1735689600). */
	if (state.rtc_epoch > 1735689600) {
		uint32_t day_sec = state.rtc_epoch % 86400;
		uint8_t hh = day_sec / 3600;
		uint8_t mm = (day_sec % 3600) / 60;

		/* Source tag: G=GPS, A=app, N=network, L=local (always set). */
		char src = time_source_tag(time_sync_get_source());

		pos = snprintf(right, sizeof(right), "%02u:%02u%c ", hh, mm, src);
	}

	/* Battery percentage */
	if (state.battery_mv > 0) {
		uint8_t pct = state.battery_pct;

		if (pct == 0) {
			pct = calc_battery_pct(state.battery_mv);
		}
		snprintf(right + pos, sizeof(right) - pos, "%u%%", pct);
	}

	int right_x = DISP_W;
	if (right[0]) {
		int fw = color ? COLOR_FONT_W : FONT_W;
		uint16_t batt_color = UI_COLOR_VALUE;

		if (state.battery_mv > 0) {
			uint8_t pct = state.battery_pct;

			if (pct == 0) {
				pct = calc_battery_pct(state.battery_mv);
			}
			batt_color = (pct <= 15) ? UI_COLOR_ERROR :
				     (pct <= 30) ? UI_COLOR_WARN : UI_COLOR_OK;
		}

		right_x = DISP_W - ((int)strlen(right) * fw);
		if (color) {
			mc_display_color_text(right_x, TOP_BAR_Y, right, batt_color);
		} else {
			mc_display_text(right_x, TOP_BAR_Y, right, false);
		}
	}

	/* Node name on the left, clipped to the gap before the right block
	 * (one char-width of padding so it never touches the clock). */
	if (state.node_name[0]) {
		char name[16];
		int fw = color ? COLOR_FONT_W : FONT_W;
		int max_chars = (right_x - fw) / fw;

		if (max_chars > (int)sizeof(name) - 1) {
			max_chars = sizeof(name) - 1;
		}
		if (max_chars < 0) {
			max_chars = 0;
		}
		strncpy(name, state.node_name, max_chars);
		name[max_chars] = '\0';
		if (color) {
			mc_display_color_text(0, TOP_BAR_Y, name, UI_COLOR_TITLE);
		} else {
			mc_display_text(0, TOP_BAR_Y, name, false);
		}
	}
}

/* ========== Helper: Page Indicator Dots ========== */

static void render_page_indicator(void)
{
	/* Centered dots matching Arduino layout:
	 * Each dot spaced 10px apart, centered on screen.
	 * Current page = filled 3x3 block, others = single pixel. */
	bool color = mc_display_has_color();
	int total = ACTIVE_PAGE_COUNT;
	int dot_spacing = 10;
	int total_width = (total - 1) * dot_spacing;
	int start_x = (DISP_W - total_width) / 2;

	for (int i = 0; i < total; i++) {
		int x = start_x + i * dot_spacing;

		if (i == current_page_idx) {
			/* Current page: filled 3x3 block */
			if (color) {
				mc_display_color_fill_rect(x - 2, DOTS_Y - 1, 5, 5,
							   UI_COLOR_ACTIVE);
			} else {
				mc_display_fill_rect(x - 1, DOTS_Y, 3, 3);
			}
		} else {
			/* Other pages: single pixel */
			if (color) {
				mc_display_color_fill_rect(x, DOTS_Y + 1, 1, 1,
							   UI_COLOR_DIM);
			} else {
				mc_display_fill_rect(x, DOTS_Y + 1, 1, 1);
			}
		}
	}

	/* Separator line below dots */
	if (color) {
		mc_display_color_fill_rect(0, SEP_Y, DISP_W, 1, UI_COLOR_DIM);
	} else {
		mc_display_hline(0, SEP_Y, DISP_W);
	}
}

/* ========== Helper: Center text ========== */

static void draw_centered(int y, const char *text)
{
	int len = (int)strlen(text);
	int x = (DISP_W - (len * FONT_W)) / 2;

	if (x < 0) {
		x = 0;
	}
	mc_display_text(x, y, text, false);
}

static void draw_centered_color(int y, const char *text, uint16_t color)
{
	int len = (int)strlen(text);
	int x = (DISP_W - (len * COLOR_FONT_W)) / 2;

	if (x < 0) {
		x = 0;
	}
	mc_display_color_text(x, y, text, color);
}

static void draw_color_segments_at(int x, int y, const char *label,
				   const char *value, uint16_t value_color)
{
	mc_display_color_text(x, y, label, UI_COLOR_LABEL);
	x += (int)strlen(label) * 6;
	mc_display_color_text(x, y, value, value_color);
}

static void draw_color_segments(int y, const char *label,
				const char *value, uint16_t value_color)
{
	draw_color_segments_at(0, y, label, value, value_color);
}

static int color_text_width(const char *text)
{
	return (int)strlen(text) * COLOR_FONT_W;
}

static void draw_badge(int x, int y, const char *text, uint16_t color)
{
	int w = color_text_width(text) + 4;

	mc_display_color_fill_rect(x, y - 1, w, COLOR_FONT_H + 2, UI_COLOR_DIM);
	mc_display_color_text(x + 2, y, text, color);
}

static void draw_metric_bar(int x, int y, int w, int h, int value,
			    int max_value, uint16_t color)
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

	mc_display_color_fill_rect(x, y, w, h, UI_COLOR_DIM);
	if (filled > 0) {
		mc_display_color_fill_rect(x, y, filled, h, color);
	}
}

static void fmt_compact_count(char *buf, size_t len, uint32_t value)
{
	if (value >= 1000000U) {
		snprintf(buf, len, "%luM", (unsigned long)(value / 1000000U));
	} else if (value >= 1000U) {
		snprintf(buf, len, "%luk", (unsigned long)(value / 1000U));
	} else {
		snprintf(buf, len, "%lu", (unsigned long)value);
	}
}

static bool use_compact_color_home(void)
{
	return mc_display_has_color() && DISP_W <= 180 && DISP_H <= 100;
}

static uint8_t clamp_activity_delta(uint32_t delta)
{
	return (delta > 15U) ? 15U : (uint8_t)delta;
}

static void sample_activity_graph(void)
{
	uint32_t now = k_uptime_get_32();
	uint32_t rx = state.lora_packets_rx;
	uint32_t tx = state.lora_packets_tx;
	uint32_t rx_delta = 0;
	uint32_t tx_delta = 0;

	if (!activity_initialized) {
		activity_last_rx = rx;
		activity_last_tx = tx;
		activity_last_sample_ms = now;
		activity_initialized = true;
		return;
	}

	if (rx >= activity_last_rx) {
		rx_delta = rx - activity_last_rx;
	}
	if (tx >= activity_last_tx) {
		tx_delta = tx - activity_last_tx;
	}

	/* Age the graph slowly during quiet periods, but do not shift it on
	 * every incidental repaint. Counter wrap/reset simply records zero. */
	if (rx_delta == 0 && tx_delta == 0 &&
	    (now - activity_last_sample_ms) < 5000U) {
		return;
	}

	activity_last_rx = rx;
	activity_last_tx = tx;
	activity_last_sample_ms = now;
	activity_head = (activity_head + 1U) % ACTIVITY_GRAPH_SAMPLES;
	activity_rx[activity_head] = clamp_activity_delta(rx_delta);
	activity_tx[activity_head] = clamp_activity_delta(tx_delta);
}

static void draw_activity_graph(int x, int y, int w, int h)
{
	uint8_t max_v = 0;
	int mid = y + h / 2;
	int col_w = w / ACTIVITY_GRAPH_SAMPLES;

	if (col_w < 1) {
		col_w = 1;
	}

	sample_activity_graph();

	mc_display_color_fill_rect(x, y, w, h, MC_COLOR_BLACK);
	mc_display_color_fill_rect(x, mid, w, 1, UI_COLOR_DIM);

	for (int i = 0; i < ACTIVITY_GRAPH_SAMPLES; i++) {
		uint8_t idx = (uint8_t)((activity_head + 1U + i) %
					ACTIVITY_GRAPH_SAMPLES);

		if (activity_rx[idx] > max_v) {
			max_v = activity_rx[idx];
		}
		if (activity_tx[idx] > max_v) {
			max_v = activity_tx[idx];
		}
	}
	if (max_v == 0) {
		mc_display_color_text(x + w - 30, y + 1, "RX", UI_COLOR_RX);
		mc_display_color_text(x + w - 14, y + 1, "TX", UI_COLOR_TX);
		return;
	}

	for (int i = 0; i < ACTIVITY_GRAPH_SAMPLES; i++) {
		uint8_t idx = (uint8_t)((activity_head + 1U + i) %
					ACTIVITY_GRAPH_SAMPLES);
		int bx = x + i * col_w;
		int draw_w = (col_w > 1) ? col_w - 1 : 1;
		int tx_h = ((h / 2 - 1) * activity_tx[idx]) / max_v;
		int rx_h = ((h / 2 - 1) * activity_rx[idx]) / max_v;

		if (tx_h > 0) {
			mc_display_color_fill_rect(bx, mid - tx_h, draw_w, tx_h,
						   UI_COLOR_TX);
		}
		if (rx_h > 0) {
			mc_display_color_fill_rect(bx, mid + 1, draw_w, rx_h,
						   UI_COLOR_RX);
		}
	}

	mc_display_color_text(x + w - 30, y + 1, "RX", UI_COLOR_RX);
	mc_display_color_text(x + w - 14, y + 1, "TX", UI_COLOR_TX);
}

/* ========== Page Renderers ========== */

static void render_messages(void)
{
	/* 3 centered rows: msg count, BLE status, offgrid status */
	char buf[24];

	if (use_compact_color_home()) {
		int y = CONTENT_Y;
		uint16_t ble_color = state.ble_connected ? UI_COLOR_OK : UI_COLOR_WARN;
		const char *ble = state.ble_connected ? "BLE OK" : "BLE ADV";
		int graph_h = 16;
		int graph_y = DISP_H - graph_h - 2;

		draw_badge(0, y, ble, ble_color);
		draw_badge(DISP_W - color_text_width(state.offgrid_enabled ? "GRID" : "LOCAL") - 4,
			   y, state.offgrid_enabled ? "GRID" : "LOCAL",
			   state.offgrid_enabled ? UI_COLOR_ACTIVE : UI_COLOR_DISABLED);
		y += LINE_H + 2;

		mc_display_color_text(0, y, "MESSAGES", UI_COLOR_LABEL);
		snprintf(buf, sizeof(buf), "%u", state.msg_count);
		mc_display_color_text(DISP_W - color_text_width(buf), y, buf,
				      state.msg_count ? UI_COLOR_WARN : UI_COLOR_VALUE);
		y += LINE_H;

		if (state.ble_connected) {
			draw_centered_color(y, "Connected", UI_COLOR_OK);
		} else {
			mc_display_color_text(34, y, "Waiting for app", UI_COLOR_WARN);
		}
		y += LINE_H;

		snprintf(buf, sizeof(buf), "Offgrid %s",
			 state.offgrid_enabled ? "on" : "off");
		draw_centered_color(y, buf,
				    state.offgrid_enabled ? UI_COLOR_ACTIVE
							  : UI_COLOR_DISABLED);
		draw_activity_graph(2, graph_y, DISP_W - 4, graph_h);
		return;
	}

	if (mc_display_has_color()) {
		int y = CONTENT_Y;
		uint16_t ble_color = state.ble_connected ? UI_COLOR_OK : UI_COLOR_WARN;
		const char *ble = state.ble_connected ? "BLE OK" : "BLE ADV";

		draw_badge(0, y, ble, ble_color);
		draw_badge(DISP_W - color_text_width(state.offgrid_enabled ? "GRID" : "LOCAL") - 4,
			   y, state.offgrid_enabled ? "GRID" : "LOCAL",
			   state.offgrid_enabled ? UI_COLOR_ACTIVE : UI_COLOR_DISABLED);
		y += LINE_H + 2;

		mc_display_color_text(0, y, "MESSAGES", UI_COLOR_LABEL);
		snprintf(buf, sizeof(buf), "%u", state.msg_count);
		mc_display_color_text(DISP_W - color_text_width(buf), y, buf,
				      state.msg_count ? UI_COLOR_WARN : UI_COLOR_VALUE);
		y += LINE_H;

		if (state.ble_connected) {
			draw_centered_color(y, "Connected", UI_COLOR_OK);
		} else {
			draw_centered_color(y, "Waiting for app", UI_COLOR_WARN);
		}
		y += LINE_H;

		snprintf(buf, sizeof(buf), "Offgrid %s",
			 state.offgrid_enabled ? "on" : "off");
		draw_centered_color(y, buf,
				    state.offgrid_enabled ? UI_COLOR_ACTIVE
							  : UI_COLOR_DISABLED);
		return;
	}

	snprintf(buf, sizeof(buf), "MSG: %u", state.msg_count);
	draw_centered(centered_row(0, 3), buf);

	if (state.ble_connected) {
		draw_centered(centered_row(1, 3), "< Connected >");
	} else {
		draw_centered(centered_row(1, 3), "Waiting for app...");
	}

	snprintf(buf, sizeof(buf), "Offgrid: %s",
		 state.offgrid_enabled ? "ON" : "OFF");
	draw_centered(centered_row(2, 3), buf);
}

static void render_recent(void)
{
	if (state.recent_count == 0) {
		if (mc_display_has_color()) {
			draw_centered_color(centered_row(0, 1), "no contacts heard",
					    UI_COLOR_DISABLED);
		} else {
			draw_centered(centered_row(0, 1), "(no contacts heard)");
		}
		return;
	}

	int y = CONTENT_Y;

	for (int i = 0; i < state.recent_count && i < 4; i++) {
		/* age_s is recomputed from RTC timestamps each housekeeping
		 * cycle (~5s), so it's always fresh and monotonic. */
		uint32_t age_s = state.recent[i].age_s;

		/* Format elapsed time */
		char time_str[8];

		if (age_s < 60) {
			snprintf(time_str, sizeof(time_str), "%us", age_s);
		} else if (age_s < 3600) {
			snprintf(time_str, sizeof(time_str), "%um", age_s / 60);
		} else if (age_s < 86400) {
			snprintf(time_str, sizeof(time_str), "%uh", age_s / 3600);
		} else {
			snprintf(time_str, sizeof(time_str), "%ud", age_s / 86400);
		}

		/* Name left-aligned, time right-aligned */
		char buf[24];

		snprintf(buf, sizeof(buf), "%-13s %s",
			 state.recent[i].name, time_str);
		if (mc_display_has_color()) {
			uint16_t age_color = (age_s < 300) ? UI_COLOR_OK :
					     (age_s < 3600) ? UI_COLOR_WARN
							    : UI_COLOR_DISABLED;

			mc_display_color_text(0, y, state.recent[i].name,
					      UI_COLOR_VALUE);
			mc_display_color_text(DISP_W - color_text_width(time_str),
					      y, time_str, age_color);
		} else {
			mc_display_text(0, y, buf, false);
		}
		y += LINE_H;
	}
}

static void render_radio(void)
{
	char buf[32];
	int y = CONTENT_Y;
	uint32_t freq_mhz = state.lora_freq_hz / 1000000;
	uint32_t freq_frac = (state.lora_freq_hz % 1000000 + 500) / 1000;
	uint16_t bw_int = state.lora_bw_khz_x10 / 10;
	uint16_t bw_frac = state.lora_bw_khz_x10 % 10;
	const char *packet_state = state.lora_tx_active ? "TX" :
				   (state.lora_in_rx ? "RX" :
				    (state.lora_radio_ready ? "RDY" : "WAIT"));
	const char *rx_mode = state.lora_rx_duty_cycle ? "DC" : "CONT";
	uint16_t warn_color = (state.lora_apc_enabled && state.lora_apc_reduction > 0)
			      ? UI_COLOR_WARN : UI_COLOR_OK;

	if (mc_display_has_color()) {
		uint16_t state_color = state.lora_tx_active ? UI_COLOR_WARN :
				       state.lora_in_rx ? UI_COLOR_ACTIVE :
				       state.lora_radio_ready ? UI_COLOR_OK
							      : UI_COLOR_DISABLED;
		uint16_t tx_color = (state.lora_apc_enabled &&
				     state.lora_apc_reduction > 0)
				    ? UI_COLOR_WARN : UI_COLOR_OK;
		int max_tx = state.lora_tx_power > 0 ? state.lora_tx_power : 22;
		int eff_tx = state.lora_effective_tx_power > 0
			     ? state.lora_effective_tx_power : state.lora_tx_power;
		int badge_x;

		mc_display_color_fill_rect(0, y - 1, DISP_W, COLOR_FONT_H + 2,
					   UI_COLOR_HEADER_BG);
		mc_display_color_text(2, y, "RADIO", UI_COLOR_TITLE);
		badge_x = DISP_W - color_text_width(rx_mode) - 6;
		draw_badge(badge_x, y, rx_mode,
			   state.lora_rx_duty_cycle ? UI_COLOR_WARN : UI_COLOR_ACTIVE);
		badge_x -= color_text_width(packet_state) + 8;
		draw_badge(badge_x, y, packet_state, state_color);
		y += LINE_H;

		if (bw_frac) {
			snprintf(buf, sizeof(buf), "%u.%03u BW%u.%u",
				 freq_mhz, freq_frac, bw_int, bw_frac);
		} else {
			snprintf(buf, sizeof(buf), "%u.%03u BW%u",
				 freq_mhz, freq_frac, bw_int);
		}
		draw_color_segments(y, "RF ", buf, UI_COLOR_VALUE);
		y += LINE_H;

		snprintf(buf, sizeof(buf), "SF%u CR%u SW%02X P%u",
			 state.lora_sf, state.lora_cr, state.lora_sync_word,
			 state.lora_preamble_len);
		draw_color_segments(y, "LoRa ", buf, UI_COLOR_VALUE);
		y += LINE_H;

		if (state.lora_apc_enabled) {
			snprintf(buf, sizeof(buf), "%d/%ddBm",
				 state.lora_effective_tx_power, state.lora_tx_power);
		} else {
			snprintf(buf, sizeof(buf), "%ddBm", state.lora_tx_power);
		}
		draw_color_segments(y, "TX ", buf, tx_color);
		draw_metric_bar(DISP_W - 50, y + 2, 48, 5, eff_tx, max_tx, tx_color);
		y += LINE_H;

		snprintf(buf, sizeof(buf), "red %d M%d.%d T%u",
			 state.lora_apc_reduction,
			 state.lora_apc_margin_x10 / 10,
			 abs(state.lora_apc_margin_x10 % 10),
			 state.lora_apc_target_margin);
		draw_color_segments(y, "APC ", buf, warn_color);
		draw_badge(DISP_W - color_text_width("APC") - 4, y, "APC",
			   !state.lora_apc_enabled ? UI_COLOR_DISABLED :
			   state.lora_apc_reduction > 0 ? UI_COLOR_WARN : UI_COLOR_OK);
		y += LINE_H;

		char rx_count[6];
		char tx_count[6];
		char err_count[6];

		fmt_compact_count(rx_count, sizeof(rx_count), state.lora_packets_rx);
		fmt_compact_count(tx_count, sizeof(tx_count), state.lora_packets_tx);
		fmt_compact_count(err_count, sizeof(err_count), state.lora_packets_err);
		snprintf(buf, sizeof(buf), "NF%d R%s T%s E%s",
			 state.lora_noise_floor, rx_count, tx_count, err_count);
		draw_color_segments(y, "PKT ", buf,
				    state.lora_packets_err ? UI_COLOR_ERROR
							   : UI_COLOR_VALUE);
		return;
	}

	if (bw_frac) {
		snprintf(buf, sizeof(buf), "%u.%03u BW%u.%u",
			 freq_mhz, freq_frac, bw_int, bw_frac);
	} else {
		snprintf(buf, sizeof(buf), "%u.%03u BW%u",
			 freq_mhz, freq_frac, bw_int);
	}
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	snprintf(buf, sizeof(buf), "SF%u CR%u SW%02X P%u",
		 state.lora_sf, state.lora_cr, state.lora_sync_word,
		 state.lora_preamble_len);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	if (state.lora_apc_enabled) {
		snprintf(buf, sizeof(buf), "TX:%d/%ddBm APC:on",
			 state.lora_effective_tx_power, state.lora_tx_power);
	} else {
		snprintf(buf, sizeof(buf), "TX:%ddBm APC:off", state.lora_tx_power);
	}
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	snprintf(buf, sizeof(buf), "R%d M%d.%d T%u %s/%s",
		 state.lora_apc_reduction,
		 state.lora_apc_margin_x10 / 10,
		 abs(state.lora_apc_margin_x10 % 10),
		 state.lora_apc_target_margin, packet_state, rx_mode);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	snprintf(buf, sizeof(buf), "NF%d R%lu T%lu E%lu",
		 state.lora_noise_floor,
		 (unsigned long)state.lora_packets_rx,
		 (unsigned long)state.lora_packets_tx,
		 (unsigned long)state.lora_packets_err);
	mc_display_text(0, y, buf, false);
}

static void render_bluetooth(void)
{
	if (mc_display_has_color()) {
		int y = CONTENT_Y;
		uint16_t color = !state.ble_enabled ? UI_COLOR_DISABLED :
				 state.ble_connected ? UI_COLOR_OK : UI_COLOR_WARN;

		draw_badge(0, y, "BLE", color);
		mc_display_color_text(32, y,
				      !state.ble_enabled ? "disabled" :
				      state.ble_connected ? "connected" : "advertising",
				      color);
		y += LINE_H + 4;
		draw_centered_color(y,
				    state.ble_enabled ? "Press to disable"
						      : "Press to enable",
				    UI_COLOR_VALUE);
		return;
	}

	if (!state.ble_enabled) {
		draw_centered(centered_row(0, 2), "BLE: OFF");
		draw_centered(centered_row(1, 2), "Press to Enable");
		return;
	}

	if (state.ble_connected) {
		draw_centered(centered_row(0, 2), "BLE: Connected");
		draw_centered(centered_row(1, 2), "Press to Disable");
	} else {
		draw_centered(centered_row(0, 2), "BLE: Advertising");
		draw_centered(centered_row(1, 2), "Press to Disable");
	}
}

static void render_advert(void)
{
	/* Check for recent "Sent!" feedback (show for 2 seconds) */
	uint32_t now = k_uptime_get_32();
	bool just_sent = (state.advert_sent_time > 0 &&
			  (now - state.advert_sent_time) < 2000);

	if (mc_display_has_color()) {
		int y = CONTENT_Y;

		draw_badge(0, y, "ADV", just_sent ? UI_COLOR_OK : UI_COLOR_ACTIVE);
		mc_display_color_text(32, y, "Zero-hop advert", UI_COLOR_VALUE);
		y += LINE_H + 2;

		if (just_sent) {
			draw_centered_color(y,
					    state.advert_was_flood ? "Flood sent" : "Sent",
					    UI_COLOR_OK);
		} else {
			draw_centered_color(y, "Press to send", UI_COLOR_VALUE);
		}
		y += LINE_H;
		draw_centered_color(y, "2x press: flood", UI_COLOR_LABEL);
		return;
	}

	draw_centered(centered_row(0, 3), "Send Zero-Hop Advert");

	if (just_sent) {
		if (state.advert_was_flood) {
			draw_centered(centered_row(1, 3), ">> Flood Sent! <<");
		} else {
			draw_centered(centered_row(1, 3), ">>> Sent! <<<");
		}
	} else {
		draw_centered(centered_row(1, 3), "Press to Send");
	}

	draw_centered(centered_row(2, 3), "(2x Press: Flood)");
}

/* Format seconds into compact time string: "3m20s", "1h05m", "12s" */
static void fmt_duration(char *buf, size_t len, uint32_t secs)
{
	if (secs >= 3600) {
		snprintf(buf, len, "%uh%02um", secs / 3600, (secs % 3600) / 60);
	} else if (secs >= 60) {
		snprintf(buf, len, "%um%02us", secs / 60, secs % 60);
	} else {
		snprintf(buf, len, "%us", secs);
	}
}

static void render_gps(void)
{
	char buf[32];
	int y = CONTENT_Y;
	bool color = mc_display_has_color();

	/* No GPS hardware on this board */
	if (!state.gps_available) {
		if (color) {
			draw_badge(0, y, "GPS", UI_COLOR_DISABLED);
			mc_display_color_text(32, y, "not detected", UI_COLOR_DISABLED);
		} else {
			mc_display_text(0, y, "GPS: not detected", false);
		}
		return;
	}

	/* GPS enabled/disabled state */
	snprintf(buf, sizeof(buf), "GPS: %s",
		 state.gps_enabled ? "on" : "off");
	if (color) {
		draw_badge(0, y, "GPS", state.gps_enabled ? UI_COLOR_OK
							    : UI_COLOR_DISABLED);
		mc_display_color_text(32, y, state.gps_enabled ? "on" : "off",
				      state.gps_enabled ? UI_COLOR_OK
							: UI_COLOR_DISABLED);
	} else {
		mc_display_text(0, y, buf, false);
	}
	y += LINE_H;

	if (!state.gps_enabled) {
		if (color) {
			draw_centered_color(y + 8, "Press to enable", UI_COLOR_VALUE);
		} else {
			draw_centered(y + 8, "Press to Enable");
		}
		return;
	}

	/* State-dependent display */
	if (state.gps_state == 2) {
		/* ACQUIRING — actively searching for satellites */
		snprintf(buf, sizeof(buf), "Searching... sat:%u",
			 state.gps_satellites);
		if (color) {
			draw_color_segments(y, "SAT ", buf, UI_COLOR_WARN);
		} else {
			mc_display_text(0, y, buf, false);
		}
		y += LINE_H;

		/* Show last fix age if we have one */
		if (state.gps_last_fix_age_s != UINT32_MAX) {
			char tbuf[12];

			fmt_duration(tbuf, sizeof(tbuf), state.gps_last_fix_age_s);
			snprintf(buf, sizeof(buf), "Last fix: %s ago", tbuf);
			if (color) {
				draw_color_segments(y, "FIX ", buf, UI_COLOR_VALUE);
			} else {
				mc_display_text(0, y, buf, false);
			}
		} else {
			if (color) {
				mc_display_color_text(0, y, "No fix yet", UI_COLOR_WARN);
			} else {
				mc_display_text(0, y, "No fix yet", false);
			}
		}
	} else if (state.gps_state == 1) {
		/* STANDBY — sleeping between fix cycles */
		if (state.gps_last_fix_age_s != UINT32_MAX) {
			char tbuf[12];

			fmt_duration(tbuf, sizeof(tbuf), state.gps_last_fix_age_s);
			snprintf(buf, sizeof(buf), "Last fix: %s ago", tbuf);
			if (color) {
				draw_color_segments(y, "FIX ", buf, UI_COLOR_VALUE);
			} else {
				mc_display_text(0, y, buf, false);
			}
			y += LINE_H;
		} else {
			if (color) {
				mc_display_color_text(0, y, "No fix yet", UI_COLOR_WARN);
			} else {
				mc_display_text(0, y, "No fix yet", false);
			}
			y += LINE_H;
		}

		if (state.gps_next_search_s > 0) {
			char tbuf[12];

			fmt_duration(tbuf, sizeof(tbuf), state.gps_next_search_s);
			snprintf(buf, sizeof(buf), "Next search: %s", tbuf);
			if (color) {
				draw_color_segments(y, "NEXT ", buf, UI_COLOR_LABEL);
			} else {
				mc_display_text(0, y, buf, false);
			}
		}
	} else {
		/* OFF — shouldn't reach here if gps_enabled is true */
		if (color) {
			mc_display_color_text(0, y, "GPS off", UI_COLOR_DISABLED);
		} else {
			mc_display_text(0, y, "GPS off", false);
		}
	}
}

static void render_buzzer(void)
{
	char buf[24];
	int y = CONTENT_Y;

	snprintf(buf, sizeof(buf), "Buzzer: %s",
		 state.buzzer_quiet ? "off" : "on");
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	draw_centered(y + 8,
			  state.buzzer_quiet ? "Press to Enable" : "Press to Disable");
}

static void render_leds(void)
{
	char buf[24];
	int y = CONTENT_Y;

	if (mc_display_has_color()) {
		draw_badge(0, y, "LED", state.leds_disabled ? UI_COLOR_DISABLED
							      : UI_COLOR_OK);
		mc_display_color_text(32, y,
				      state.leds_disabled ? "off" : "on",
				      state.leds_disabled ? UI_COLOR_DISABLED
							  : UI_COLOR_OK);
		y += LINE_H + 4;
		draw_centered_color(y,
				    state.leds_disabled ? "Press to enable"
							: "Press to disable",
				    UI_COLOR_VALUE);
		return;
	}

	snprintf(buf, sizeof(buf), "LEDs: %s",
		 state.leds_disabled ? "off" : "on");
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	draw_centered(y + 8,
			  state.leds_disabled ? "Press to Enable" : "Press to Disable");
}

static void render_sensors(void)
{
	char buf[24];
	int y = CONTENT_Y;
	bool color = mc_display_has_color();

	/* Lazy read: only fetch sensors when the user is actually looking at
	 * this page.  Render is event-driven (schedule_render only on real UI
	 * events), so this never fires periodically when idle. */
	struct env_data edata;
	bool have_env = env_sensors_available() && env_sensors_read(&edata) == 0;

	if (have_env && edata.has_temperature) {
		int16_t t10 = (int16_t)(edata.temperature_c * 10);
		snprintf(buf, sizeof(buf), "Temp: %d.%d C",
			 t10 / 10, abs(t10 % 10));
		if (color) {
			draw_color_segments(y, "TMP ", buf, UI_COLOR_VALUE);
		} else {
			mc_display_text(0, y, buf, false);
		}
		y += LINE_H;
	}

	if (have_env && edata.has_pressure) {
		snprintf(buf, sizeof(buf), "Press: %u hPa",
			 (unsigned)edata.pressure_hpa);
		if (color) {
			draw_color_segments(y, "BAR ", buf, UI_COLOR_VALUE);
		} else {
			mc_display_text(0, y, buf, false);
		}
		y += LINE_H;
	}

	if (have_env && edata.has_humidity) {
		uint16_t h10 = (uint16_t)(edata.humidity_pct * 10);
		snprintf(buf, sizeof(buf), "Humid: %u.%u%%",
			 h10 / 10, h10 % 10);
		if (color) {
			draw_color_segments(y, "HUM ", buf, UI_COLOR_VALUE);
		} else {
			mc_display_text(0, y, buf, false);
		}
		y += LINE_H;
	}

	/* Battery at bottom */
	snprintf(buf, sizeof(buf), "Batt: %u%% (%umV)",
		 state.battery_pct > 0 ? state.battery_pct
					   : calc_battery_pct(state.battery_mv),
		 state.battery_mv);
	if (color) {
		uint8_t pct = state.battery_pct > 0 ? state.battery_pct
						    : calc_battery_pct(state.battery_mv);
		uint16_t batt_color = (pct <= 15) ? UI_COLOR_ERROR :
				      (pct <= 30) ? UI_COLOR_WARN : UI_COLOR_OK;

		snprintf(buf, sizeof(buf), "%u%% %umV", pct, state.battery_mv);
		draw_color_segments(y, "BAT ", buf, batt_color);
		draw_metric_bar(DISP_W - 44, y + 2, 42, 5, pct, 100, batt_color);
	} else {
		mc_display_text(0, y, buf, false);
	}
}

static void render_offgrid(void)
{
	/* Check if confirmation has expired */
	if (state.offgrid_confirm_time != 0 &&
		(k_uptime_get_32() - state.offgrid_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.offgrid_confirm_time = 0;
	}

	char buf[24];

	if (mc_display_has_color()) {
		int y = CONTENT_Y;

		draw_badge(0, y, "GRID", state.offgrid_enabled ? UI_COLOR_ACTIVE
								: UI_COLOR_DISABLED);
		mc_display_color_text(38, y, "client repeat",
				      state.offgrid_enabled ? UI_COLOR_ACTIVE
							    : UI_COLOR_LABEL);
		y += LINE_H + 4;
		snprintf(buf, sizeof(buf), "Status %s",
			 state.offgrid_enabled ? "on" : "off");
		draw_centered_color(y, buf,
				    state.offgrid_enabled ? UI_COLOR_ACTIVE
							  : UI_COLOR_DISABLED);
		y += LINE_H;
		draw_centered_color(y,
				    state.offgrid_confirm_time != 0 ?
					    "Press to confirm" :
					    (state.offgrid_enabled ? "Press to disable"
								   : "Press to enable"),
				    state.offgrid_confirm_time != 0 ? UI_COLOR_WARN
								    : UI_COLOR_VALUE);
		return;
	}

	draw_centered(centered_row(0, 3), "Offgrid Mode");

	snprintf(buf, sizeof(buf), "Status: %s",
		 state.offgrid_enabled ? "ON" : "OFF");
	draw_centered(centered_row(1, 3), buf);

	if (state.offgrid_confirm_time != 0) {
		draw_centered(centered_row(2, 3), "Press to confirm");
	} else {
		draw_centered(centered_row(2, 3),
				  state.offgrid_enabled ? "Press to Disable"
							: "Press to Enable");
	}
}

static void render_dfu(void)
{
	/* Check if confirmation has expired */
	if (state.dfu_confirm_time != 0 &&
		(k_uptime_get_32() - state.dfu_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.dfu_confirm_time = 0;
	}

	if (mc_display_has_color()) {
		int y = CONTENT_Y;

		draw_badge(0, y, "DFU", state.dfu_confirm_time != 0
						? UI_COLOR_WARN : UI_COLOR_ACTIVE);
		mc_display_color_text(32, y, "BLE update", UI_COLOR_VALUE);
		y += LINE_H + 4;
		draw_centered_color(y,
				    state.dfu_confirm_time != 0 ?
					    "Press to confirm" : "Press to run",
				    state.dfu_confirm_time != 0 ? UI_COLOR_WARN
								: UI_COLOR_VALUE);
		return;
	}

	draw_centered(centered_row(0, 2), "BLE DFU Update");
	if (state.dfu_confirm_time != 0) {
		draw_centered(centered_row(1, 2), "Press to confirm");
	} else {
		draw_centered(centered_row(1, 2), "Press to Run");
	}
}

static void render_shutdown(void)
{
	/* Check if confirmation has expired */
	if (state.shutdown_confirm_time != 0 &&
		(k_uptime_get_32() - state.shutdown_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.shutdown_confirm_time = 0;
	}

	if (mc_display_has_color()) {
		int y = CONTENT_Y;

		draw_badge(0, y, "PWR", state.shutdown_confirm_time != 0
						? UI_COLOR_WARN : UI_COLOR_ERROR);
		mc_display_color_text(32, y, "power off", UI_COLOR_VALUE);
		y += LINE_H + 4;
		draw_centered_color(y,
				    state.shutdown_confirm_time != 0 ?
					    "Press to confirm" : "Press to run",
				    state.shutdown_confirm_time != 0 ? UI_COLOR_WARN
								    : UI_COLOR_VALUE);
		return;
	}

	draw_centered(centered_row(0, 2), "Power Off");
	if (state.shutdown_confirm_time != 0) {
		draw_centered(centered_row(1, 2), "Press to confirm");
	} else {
		draw_centered(centered_row(1, 2), "Press to Run");
	}
}

static void render_status(void)
{
	char buf[28];
	int y = CONTENT_Y;
	bool color = mc_display_has_color();

	/* Role label */
	if (color) {
		draw_badge(0, y, "MODE", UI_COLOR_ACTIVE);
		mc_display_color_text(38, y, "repeater", UI_COLOR_VALUE);
	} else {
		draw_centered(y, "REPEATER");
	}
	y += LINE_H;

	/* Uptime */
	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);
	uint32_t days = up_s / 86400;
	uint32_t hours = (up_s % 86400) / 3600;
	uint32_t mins = (up_s % 3600) / 60;

	snprintf(buf, sizeof(buf), "Up: %ud %uh %um", days, hours, mins);
	if (color) {
		draw_color_segments(y, "UP ", buf, UI_COLOR_VALUE);
	} else {
		mc_display_text(0, y, buf, false);
	}
	y += LINE_H;

	/* Clock — only if RTC has been synced (after Jan 1 2025) */
	if (state.rtc_epoch > 1735689600) {
		uint32_t day_sec = state.rtc_epoch % 86400;
		uint8_t hh = day_sec / 3600;
		uint8_t mm = (day_sec % 3600) / 60;
		uint8_t ss = day_sec % 60;

		snprintf(buf, sizeof(buf), "Time: %02u:%02u:%02u UTC", hh, mm, ss);
		if (color) {
			draw_color_segments(y, "CLK ", buf, UI_COLOR_OK);
		} else {
			mc_display_text(0, y, buf, false);
		}
	} else {
		if (color) {
			draw_color_segments(y, "CLK ", "not synced", UI_COLOR_WARN);
		} else {
			mc_display_text(0, y, "Time: not synced", false);
		}
	}
	y += LINE_H;

	/* Battery */
	if (state.battery_mv > 0) {
		uint8_t pct = state.battery_pct;

		if (pct == 0) {
			pct = calc_battery_pct(state.battery_mv);
		}
		snprintf(buf, sizeof(buf), "Batt: %u%% (%umV)", pct, state.battery_mv);
		if (color) {
			uint16_t batt_color = (pct <= 15) ? UI_COLOR_ERROR :
					      (pct <= 30) ? UI_COLOR_WARN : UI_COLOR_OK;

			snprintf(buf, sizeof(buf), "%u%% %umV", pct, state.battery_mv);
			draw_color_segments(y, "BAT ", buf, batt_color);
			draw_metric_bar(DISP_W - 44, y + 2, 42, 5, pct, 100,
					batt_color);
		} else {
			mc_display_text(0, y, buf, false);
		}
	}
}

/* ========== Page render dispatch ========== */

typedef void (*page_render_fn)(void);

static const page_render_fn renderers[] = {
	[UI_PAGE_MESSAGES]  = render_messages,
	[UI_PAGE_RECENT]    = render_recent,
	[UI_PAGE_RADIO]     = render_radio,
	[UI_PAGE_BLUETOOTH] = render_bluetooth,
	[UI_PAGE_ADVERT]    = render_advert,
	[UI_PAGE_GPS]       = render_gps,
	[UI_PAGE_BUZZER]    = render_buzzer,
	[UI_PAGE_LEDS]      = render_leds,
	[UI_PAGE_SENSORS]   = render_sensors,
	[UI_PAGE_OFFGRID]   = render_offgrid,
	[UI_PAGE_DFU]       = render_dfu,
	[UI_PAGE_SHUTDOWN]  = render_shutdown,
	[UI_PAGE_STATUS]    = render_status,
};

/* ========== Public API ========== */

struct ui_state *ui_pages_get_state(void)
{
	return &state;
}

void ui_pages_render(void)
{
	/* Refresh battery lazily — ADC only fires when the cached reading is
	 * stale (≥30 s).  Render is event-driven, so during idle this never
	 * runs.  Telemetry / stats paths bypass the cache entirely. */
	ui_refresh_battery();

	mc_display_clear();
	render_top_bar();
	render_page_indicator();

	enum ui_page page = active_pages[current_page_idx];

	if (page < UI_PAGE_COUNT && renderers[page]) {
		renderers[page]();
	}

	mc_display_finalize();
}

void ui_pages_render_splash(void)
{
	mc_display_clear();

	/* ZephCore logo bitmap (128x13px) — center horizontally */
	int logo_w = 128;
	int logo_h = 13;
	int logo_x = (DISP_W >= logo_w) ? (DISP_W - logo_w) / 2 : 0;
	int y = 3;

	mc_display_xbm(logo_x, y, zephcore_logo, logo_w, logo_h);
	y += logo_h + LINE_H;

	/* "MeshCore on Zephyr" centered below logo */
	draw_centered(y, "MeshCore on Zephyr");
	y += LINE_H * 2;

	/* Build date centered below (format: "2026 Feb 15") */
#ifdef FIRMWARE_BUILD_DATE
	draw_centered(y, FIRMWARE_BUILD_DATE);
#endif

	mc_display_finalize();
}

void ui_pages_next(void)
{
	state.shutdown_confirm_time = 0;  /* Reset confirmation on navigate */
	current_page_idx++;
	if (current_page_idx >= ACTIVE_PAGE_COUNT) {
		current_page_idx = 0;
	}
	state.current_page = active_pages[current_page_idx];
}

void ui_pages_prev(void)
{
	state.shutdown_confirm_time = 0;  /* Reset confirmation on navigate */
	current_page_idx--;
	if (current_page_idx < 0) {
		current_page_idx = ACTIVE_PAGE_COUNT - 1;
	}
	state.current_page = active_pages[current_page_idx];
}

void ui_pages_set(enum ui_page page)
{
	/* Find page in active list, fall back to first active page */
	for (int i = 0; i < ACTIVE_PAGE_COUNT; i++) {
		if (active_pages[i] == page) {
			current_page_idx = i;
			state.current_page = page;
			return;
		}
	}
	/* Page not in active list — go to first active page */
	current_page_idx = 0;
	state.current_page = active_pages[0];
}

enum ui_page ui_pages_current(void)
{
	return active_pages[current_page_idx];
}

void ui_pages_set_node_name(const char *name)
{
	if (name) {
		/* Sanitize UTF-8 to Latin-1 (strips emojis, keeps accents) */
		utf8_to_latin1(state.node_name, name, sizeof(state.node_name));
	}
}

void ui_pages_advert_sent(bool flood)
{
	state.advert_sent_time = k_uptime_get_32();
	state.advert_was_flood = flood;
}
