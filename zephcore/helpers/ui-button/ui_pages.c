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
/* Full-scale end of the radio page's TX power bar.  The board's configured
 * ceiling when it has one (same symbol CommonCLI clamps "set tx" against),
 * else the common SX126x/LR11xx maximum. */
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
#define TX_POWER_BAR_MAX_DBM  CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
#else
#define TX_POWER_BAR_MAX_DBM  22
#endif

#define FONT_W       mc_display_font_width()
#define FONT_H       mc_display_font_height()
#define DISP_W       mc_display_width()
#define DISP_H       mc_display_height()
#define TOP_BAR_Y    0
#define DOTS_Y       (FONT_H + 2)           /* just below top bar */
#define SEP_Y        (DOTS_Y + 4)           /* below dots */
#define MAX_CHARS    (DISP_W / (FONT_W ? FONT_W : 1))

/* ---- Tiny-panel mode (e.g. 64x32 wristband OLED) ----
 * When the panel is too short for the top bar + page dots + separator, drop
 * that chrome and hand the whole panel to page content, tightening the line
 * spacing so four rows of the 6x8 font fit in 32px.  Auto-detected from the
 * panel height, so 128x64 boards are unaffected.  There is no persistent title
 * bar on tiny panels — the page name is flashed briefly on a page change (see
 * tiny_flash_kick / render_tiny_title). */
static inline bool ui_tiny(void)
{
	return DISP_H < 48;
}

static inline int ui_line_h(void)
{
	return ui_tiny() ? FONT_H : (FONT_H + 2);   /* drop the inter-line gap */
}

static inline int ui_content_y(void)
{
	return ui_tiny() ? 0 : (SEP_Y + 4);         /* full height when tiny */
}

#define LINE_H       (ui_line_h())
#define CONTENT_Y    (ui_content_y())

/* Compact RGB565 palette for small color TFTs.  These are used only through
 * mc_display_has_color(); monochrome displays keep the existing CFB path. */
#define UI_COLOR_BG         MC_COLOR_BLACK
#define UI_COLOR_HEADER_BG  0x2104  /* dark neutral panel */
#define UI_COLOR_TITLE      MC_COLOR_LIGHT_GRAY  /* soft white, slightly recessed vs values */
#define UI_COLOR_LABEL      MC_COLOR_WHITE  /* small-TFT gamma renders mid-gray near-black */
#define UI_COLOR_VALUE      MC_COLOR_WHITE
#define UI_COLOR_OK         MC_COLOR_GREEN
#define UI_COLOR_ACTIVE     UI_COLOR_OK
#define UI_COLOR_WARN       0xffa0
#define UI_COLOR_ERROR      MC_COLOR_RED
#define UI_COLOR_DIM        MC_COLOR_LIGHT_GRAY  /* small-TFT gamma (T114) crushes anything
						  * darker toward black — in person this reads
						  * as a proper gray, not white (PR #62) */
#define UI_COLOR_DISABLED   UI_COLOR_DIM  /* off/disabled states dimmer than values */
#define UI_COLOR_FAINT      0x4208  /* dark structural fills (badge bg, separators, bar tracks) —
				     * decoration, must stay subtle so content pops against it */
#define UI_COLOR_TX         MC_COLOR_ORANGE
#define UI_COLOR_RX         UI_COLOR_OK

/* Glyph cell of the color renderer (display.c upscales the 6x8 font under
 * LARGE_FONT) — NOT the CFB font metrics, which can differ (10x16 vs 9x12). */
#define COLOR_FONT_W  mc_display_color_font_width()
#define COLOR_FONT_H  mc_display_color_font_height()
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

/* Tiny-mode page-title flash: after a page change we show the page name for
 * TINY_FLASH_MS, then re-render into content (there is no persistent bar on
 * tiny panels).  The follow-up render is scheduled by the UI task via
 * ui_pages_flash_remaining_ms(). */
#define TINY_FLASH_MS 1000U
static uint32_t tiny_flash_until;

static void tiny_flash_kick(void)
{
	tiny_flash_until = k_uptime_get_32() + TINY_FLASH_MS;
}

#if MC_DISPLAY_COLOR_PANEL
static uint8_t activity_rx[ACTIVITY_GRAPH_SAMPLES];
static uint8_t activity_tx[ACTIVITY_GRAPH_SAMPLES];
static uint8_t activity_head;
static bool activity_initialized;
static uint32_t activity_last_rx;
static uint32_t activity_last_tx;
static uint32_t activity_last_sample_ms;
#endif

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
	case TIME_SYNC_MESH: return 'M';  /* mesh time-sync consensus */
	default:             return 'L';  /* local: manual/CLI or stale/none */
	}
}

static void render_top_bar(void)
{
	if (ui_tiny()) {
		return;   /* no room for a bar; page title is flashed on change */
	}

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
	if (ui_tiny()) {
		return;   /* no room for dots/separator on a tiny panel */
	}

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
		mc_display_color_fill_rect(0, SEP_Y, DISP_W, 1, UI_COLOR_FAINT);
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
	x += (int)strlen(label) * COLOR_FONT_W;
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

	mc_display_color_fill_rect(x, y - 1, w, COLOR_FONT_H + 2, UI_COLOR_FAINT);
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

	mc_display_color_fill_rect(x, y, w, h, UI_COLOR_FAINT);
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

#if MC_DISPLAY_COLOR_PANEL
static bool use_compact_color_home(void)
{
	return mc_display_has_color() && DISP_W <= 180 && DISP_H <= 100;
}
#endif

/* Short label for the current LoRa radio state, shared across pages. */
static const char *radio_state_label(void)
{
	return state.lora_tx_active ? "TX" :
	       state.lora_in_rx ? "RX" :
	       state.lora_radio_ready ? "RDY" : "WAIT";
}

#if MC_DISPLAY_COLOR_PANEL
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

	mc_display_color_fill_rect(x, y, w, h, UI_COLOR_BG);
	mc_display_color_fill_rect(x, mid, w, 1, UI_COLOR_FAINT);

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
#endif /* MC_DISPLAY_COLOR_PANEL */

/* ========== Page Renderers ========== */

/* ========== Tiny-panel page title (flashed on page change) ========== */

static const char *tiny_page_title(enum ui_page p)
{
	switch (p) {
	case UI_PAGE_MESSAGES:  return "MESSAGES";
	case UI_PAGE_RECENT:    return "RECENT";
	case UI_PAGE_RADIO:     return "RADIO";
	case UI_PAGE_TRAFFIC:   return "TRAFFIC";
	case UI_PAGE_BLUETOOTH: return "BLUETOOTH";
	case UI_PAGE_ADVERT:    return "ADVERT";
	case UI_PAGE_GPS:       return "GPS";
	case UI_PAGE_BUZZER:    return "BUZZER";
	case UI_PAGE_LEDS:      return "LEDS";
	case UI_PAGE_SENSORS:   return "SENSORS";
	case UI_PAGE_OFFGRID:   return "OFFGRID";
	case UI_PAGE_DFU:       return "DFU";
	case UI_PAGE_SHUTDOWN:  return "SHUTDOWN";
	case UI_PAGE_STATUS:    return "STATUS";
	default:                return "";
	}
}

static void render_tiny_title(void)
{
	const char *title = tiny_page_title(active_pages[current_page_idx]);
	int rows = (state.battery_mv > 0) ? 2 : 1;

	draw_centered(centered_row(0, rows), title);

	if (state.battery_mv > 0) {
		char buf[8];
		uint8_t pct = state.battery_pct ? state.battery_pct
					        : calc_battery_pct(state.battery_mv);

		snprintf(buf, sizeof(buf), "%u%%", pct);
		draw_centered(centered_row(1, rows), buf);
	}
}

/*
 * Per-page renderer split (UI profile pattern)
 * --------------------------------------------
 * Each page's capability-divergent bodies live in dedicated _mono / _color
 * functions.  render_<page>() is a thin dispatcher that picks one:
 *   - Mono / tiny / e-ink panels use render_<page>_mono().
 *   - RGB565 color panels use render_<page>_color(), which is compiled ONLY
 *     when a `tft` node exists in devicetree (MC_DISPLAY_COLOR_PANEL).  On a
 *     mono board the color body — and every color-only helper it references —
 *     drops out entirely, so no color RAM/flash is spent.
 * The bodies are moved verbatim from the old combined function, so on-screen
 * output is byte-identical to before.
 */
static void render_messages_mono(void)
{
	/* 3 centered rows: msg count, BLE status, offgrid status */
	char buf[24];

	if (ui_tiny()) {
		snprintf(buf, sizeof(buf), "MSG:%u", state.msg_count);
		draw_centered(centered_row(0, 3), buf);
		draw_centered(centered_row(1, 3),
			      state.ble_connected ? "BLE:conn" : "BLE:adv");

		uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);

		snprintf(buf, sizeof(buf), "%uh%02um",
			 (up_s % 86400) / 3600, (up_s % 3600) / 60);
		draw_centered(centered_row(2, 3), buf);
		return;
	}

	snprintf(buf, sizeof(buf), "MSG: %u", state.msg_count);
	draw_centered(centered_row(0, 4), buf);

	if (state.ble_connected) {
		draw_centered(centered_row(1, 4), "< Connected >");
	} else {
		draw_centered(centered_row(1, 4), "Waiting for app...");
	}

	snprintf(buf, sizeof(buf), "Offgrid: %s",
		 state.offgrid_enabled ? "ON" : "OFF");
	draw_centered(centered_row(2, 4), buf);

	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);
	snprintf(buf, sizeof(buf), "Up: %ud %uh %um",
		 up_s / 86400, (up_s % 86400) / 3600, (up_s % 3600) / 60);
	draw_centered(centered_row(3, 4), buf);
}

#if MC_DISPLAY_COLOR_PANEL
static void render_messages_color(void)
{
	char buf[24];

	if (use_compact_color_home()) {
		int y = CONTENT_Y;
		uint16_t ble_color = state.ble_connected ? UI_COLOR_OK : UI_COLOR_WARN;
		const char *ble = state.ble_connected ? "BLE OK" : "BLE ADV";
		const char *radio = radio_state_label();
		uint16_t radio_color = state.lora_tx_active ? UI_COLOR_TX :
				       state.lora_in_rx ? UI_COLOR_RX :
				       state.lora_radio_ready ? UI_COLOR_OK
							      : UI_COLOR_WARN;

		draw_badge(0, y, ble, ble_color);
		draw_badge(DISP_W - color_text_width(radio) - 4, y, radio,
			   radio_color);
		y += LINE_H + 2;

		mc_display_color_text(0, y, "COMPANION", UI_COLOR_LABEL);
		snprintf(buf, sizeof(buf), "MSG %u", state.msg_count);
		mc_display_color_text(DISP_W - color_text_width(buf), y, buf,
				      state.msg_count ? UI_COLOR_WARN : UI_COLOR_VALUE);
		y += LINE_H;

		if (state.ble_connected) {
			draw_centered_color(y, "Connected", UI_COLOR_OK);
		} else {
			mc_display_color_text(30, y, "Waiting for app", UI_COLOR_WARN);
		}
		y += LINE_H;

		snprintf(buf, sizeof(buf), "Offgrid %s",
			 state.offgrid_enabled ? "on" : "off");
		draw_centered_color(y, buf,
				    state.offgrid_enabled ? UI_COLOR_ACTIVE
							  : UI_COLOR_DISABLED);
		y += LINE_H;

		uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);
		snprintf(buf, sizeof(buf), "Up: %ud %uh %um",
			 up_s / 86400, (up_s % 86400) / 3600, (up_s % 3600) / 60);
		draw_centered_color(y, buf, UI_COLOR_LABEL);
		return;
	}

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
	y += LINE_H;

	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);
	snprintf(buf, sizeof(buf), "Up: %ud %uh %um",
		 up_s / 86400, (up_s % 86400) / 3600, (up_s % 3600) / 60);
	draw_centered_color(y, buf, UI_COLOR_LABEL);
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_messages(void)
{
#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_messages_color();
		return;
	}
#endif
	render_messages_mono();
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

static void render_radio_mono(void)
{
	char buf[32];
	int y = CONTENT_Y;
	uint32_t freq_mhz = state.lora_freq_hz / 1000000;
	uint32_t freq_frac = (state.lora_freq_hz % 1000000 + 500) / 1000;
	uint16_t bw_int = state.lora_bw_khz_x10 / 10;
	uint16_t bw_frac = state.lora_bw_khz_x10 % 10;
	const char *packet_state = radio_state_label();
	const char *rx_mode = state.lora_rx_duty_cycle ? "DC" : "CONT";

	if (ui_tiny()) {
		snprintf(buf, sizeof(buf), "%u.%uM", freq_mhz, freq_frac / 100);
		draw_centered(centered_row(0, 3), buf);
		snprintf(buf, sizeof(buf), "SF%u BW%u", state.lora_sf, bw_int);
		draw_centered(centered_row(1, 3), buf);
		snprintf(buf, sizeof(buf), "P%d %s", state.lora_tx_power, rx_mode);
		draw_centered(centered_row(2, 3), buf);
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

	snprintf(buf, sizeof(buf), "TX:%ddBm %s/%s",
		 state.lora_tx_power, packet_state, rx_mode);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	snprintf(buf, sizeof(buf), "NF%d R%lu T%lu E%lu",
		 state.lora_noise_floor,
		 (unsigned long)state.lora_packets_rx,
		 (unsigned long)state.lora_packets_tx,
		 (unsigned long)state.lora_packets_err);
	mc_display_text(0, y, buf, false);
}

#if MC_DISPLAY_COLOR_PANEL
static void render_radio_color(void)
{
	char buf[32];
	int y = CONTENT_Y;
	uint32_t freq_mhz = state.lora_freq_hz / 1000000;
	uint32_t freq_frac = (state.lora_freq_hz % 1000000 + 500) / 1000;
	uint16_t bw_int = state.lora_bw_khz_x10 / 10;
	uint16_t bw_frac = state.lora_bw_khz_x10 % 10;
	const char *packet_state = radio_state_label();
	const char *rx_mode = state.lora_rx_duty_cycle ? "DC" : "CONT";

	{
		uint16_t state_color = state.lora_tx_active ? UI_COLOR_WARN :
				       state.lora_in_rx ? UI_COLOR_ACTIVE :
				       state.lora_radio_ready ? UI_COLOR_OK
							      : UI_COLOR_DISABLED;
		uint16_t tx_color = UI_COLOR_OK;
		/* Scale the TX bar against the board's hardware ceiling, not
		 * against the current setting — the old max came from APC
		 * (effective vs. configured power) and with APC gone both ends
		 * would be lora_tx_power, pinning the bar permanently full. */
		int max_tx = TX_POWER_BAR_MAX_DBM;
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

		snprintf(buf, sizeof(buf), "%ddBm", state.lora_tx_power);
		draw_color_segments(y, "TX ", buf, tx_color);
		draw_metric_bar(DISP_W - 50, y + 2, 48, 5, state.lora_tx_power,
				max_tx, tx_color);
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
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_radio(void)
{
#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_radio_color();
		return;
	}
#endif
	render_radio_mono();
}

static void render_traffic_mono(void)
{
	char rx_count[6];
	char tx_count[6];
	char err_count[6];
	char buf[32];
	int y = CONTENT_Y;

	fmt_compact_count(rx_count, sizeof(rx_count), state.lora_packets_rx);
	fmt_compact_count(tx_count, sizeof(tx_count), state.lora_packets_tx);
	fmt_compact_count(err_count, sizeof(err_count), state.lora_packets_err);

	draw_centered(y, "Traffic");
	y += LINE_H;

	snprintf(buf, sizeof(buf), "RX:%s TX:%s", rx_count, tx_count);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	if (state.lora_packets_err > 0) {
		snprintf(buf, sizeof(buf), "ERR:%s", err_count);
		mc_display_text(0, y, buf, false);
	}
}

#if MC_DISPLAY_COLOR_PANEL
static void render_traffic_color(void)
{
	char rx_count[6];
	char tx_count[6];
	char err_count[6];
	char buf[32];
	int y = CONTENT_Y;
	int graph_y;
	int graph_h;
	int x;

	fmt_compact_count(rx_count, sizeof(rx_count), state.lora_packets_rx);
	fmt_compact_count(tx_count, sizeof(tx_count), state.lora_packets_tx);
	fmt_compact_count(err_count, sizeof(err_count), state.lora_packets_err);

	mc_display_color_fill_rect(0, y - 1, DISP_W, COLOR_FONT_H + 2,
				   UI_COLOR_HEADER_BG);
	mc_display_color_text(2, y, "TRAFFIC", UI_COLOR_TITLE);
	if (state.lora_packets_err > 0) {
		snprintf(buf, sizeof(buf), "E %s", err_count);
		mc_display_color_text(DISP_W - color_text_width(buf), y, buf,
				      UI_COLOR_ERROR);
	}
	y += LINE_H;

	x = 0;
	mc_display_color_text(x, y, "RX ", UI_COLOR_LABEL);
	x += 3 * COLOR_FONT_W;
	mc_display_color_text(x, y, rx_count, UI_COLOR_RX);
	x += color_text_width(rx_count) + 10;
	mc_display_color_text(x, y, "TX ", UI_COLOR_LABEL);
	x += 3 * COLOR_FONT_W;
	mc_display_color_text(x, y, tx_count, UI_COLOR_TX);

	graph_y = y + LINE_H + 2;
	graph_h = (int)DISP_H - graph_y - 2;
	if (graph_h < 16) {
		graph_h = 16;
	}
	draw_activity_graph(2, graph_y, DISP_W - 4, graph_h);
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_traffic(void)
{
#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color() && DISP_W >= 120 && DISP_H >= 64) {
		render_traffic_color();
		return;
	}
#endif
	render_traffic_mono();
}

static void render_bluetooth_mono(void)
{
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

#if MC_DISPLAY_COLOR_PANEL
static void render_bluetooth_color(void)
{
	int y = CONTENT_Y;
	uint16_t color = !state.ble_enabled ? UI_COLOR_DISABLED :
			 state.ble_connected ? UI_COLOR_OK : UI_COLOR_WARN;

	draw_badge(0, y, "BLE", color);
	mc_display_color_text(32, y,
			      !state.ble_enabled ? "disabled" :
			      state.ble_connected ? "connected" : "advertising",
			      color);
	y += LINE_H + (FONT_H / 4);
	draw_centered_color(y,
			    state.ble_enabled ? "Press to disable"
					      : "Press to enable",
			    UI_COLOR_VALUE);
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_bluetooth(void)
{
#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_bluetooth_color();
		return;
	}
#endif
	render_bluetooth_mono();
}

static void render_advert_mono(void)
{
	/* Check for recent "Sent!" feedback (show for 2 seconds) */
	uint32_t now = k_uptime_get_32();
	bool just_sent = (state.advert_sent_time > 0 &&
			  (now - state.advert_sent_time) < 2000);

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

#if MC_DISPLAY_COLOR_PANEL
static void render_advert_color(void)
{
	uint32_t now = k_uptime_get_32();
	bool just_sent = (state.advert_sent_time > 0 &&
			  (now - state.advert_sent_time) < 2000);
	int y = CONTENT_Y;

	draw_badge(0, y, "ADV", just_sent ? UI_COLOR_OK : UI_COLOR_ACTIVE);
	mc_display_color_text(32, y, "Zero-hop advert", UI_COLOR_VALUE);
	y += LINE_H + (FONT_H / 2);

	if (just_sent) {
		draw_centered_color(y,
				    state.advert_was_flood ? "Flood sent" : "Sent",
				    UI_COLOR_OK);
	} else {
		draw_centered_color(y, "Press to send", UI_COLOR_VALUE);
	}
	y += LINE_H;
	draw_centered_color(y, "2x press: flood", UI_COLOR_LABEL);
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_advert(void)
{
#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_advert_color();
		return;
	}
#endif
	render_advert_mono();
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

	if (ui_tiny()) {
		if (!state.gps_available) {
			draw_centered(centered_row(0, 1), "No GPS");
		} else if (!state.gps_enabled) {
			draw_centered(centered_row(0, 1), "GPS off");
		} else if (state.gps_state == 2) {
			snprintf(buf, sizeof(buf), "Sats:%u", state.gps_satellites);
			draw_centered(centered_row(0, 2), "Searching");
			draw_centered(centered_row(1, 2), buf);
		} else if (state.gps_state == 1) {
			draw_centered(centered_row(0, 2), "Standby");
			if (state.gps_last_fix_age_s != UINT32_MAX) {
				char tb[12];

				fmt_duration(tb, sizeof(tb), state.gps_last_fix_age_s);
				snprintf(buf, sizeof(buf), "fix %s", tb);
				draw_centered(centered_row(1, 2), buf);
			}
		} else {
			draw_centered(centered_row(0, 1), "GPS off");
		}
		return;
	}

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

static void render_leds_mono(void)
{
	char buf[24];
	int y = CONTENT_Y;

	snprintf(buf, sizeof(buf), "LEDs: %s",
		 state.leds_disabled ? "off" : "on");
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	draw_centered(y + 8,
			  state.leds_disabled ? "Press to Enable" : "Press to Disable");
}

#if MC_DISPLAY_COLOR_PANEL
static void render_leds_color(void)
{
	int y = CONTENT_Y;

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
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_leds(void)
{
#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_leds_color();
		return;
	}
#endif
	render_leds_mono();
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

static void render_offgrid_mono(void)
{
	char buf[24];

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

#if MC_DISPLAY_COLOR_PANEL
static void render_offgrid_color(void)
{
	char buf[24];
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
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_offgrid(void)
{
	/* Check if confirmation has expired */
	if (state.offgrid_confirm_time != 0 &&
		(k_uptime_get_32() - state.offgrid_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.offgrid_confirm_time = 0;
	}

#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_offgrid_color();
		return;
	}
#endif
	render_offgrid_mono();
}

static void render_dfu_mono(void)
{
	draw_centered(centered_row(0, 2), "BLE DFU Update");
	if (state.dfu_confirm_time != 0) {
		draw_centered(centered_row(1, 2), "Press to confirm");
	} else {
		draw_centered(centered_row(1, 2), "Press to Run");
	}
}

#if MC_DISPLAY_COLOR_PANEL
static void render_dfu_color(void)
{
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
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_dfu(void)
{
	/* Check if confirmation has expired */
	if (state.dfu_confirm_time != 0 &&
		(k_uptime_get_32() - state.dfu_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.dfu_confirm_time = 0;
	}

#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_dfu_color();
		return;
	}
#endif
	render_dfu_mono();
}

static void render_shutdown_mono(void)
{
	draw_centered(centered_row(0, 2), "Power Off");
	if (state.shutdown_confirm_time != 0) {
		draw_centered(centered_row(1, 2), "Press to confirm");
	} else {
		draw_centered(centered_row(1, 2), "Press to Run");
	}
}

#if MC_DISPLAY_COLOR_PANEL
static void render_shutdown_color(void)
{
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
}
#endif /* MC_DISPLAY_COLOR_PANEL */

static void render_shutdown(void)
{
	/* Check if confirmation has expired */
	if (state.shutdown_confirm_time != 0 &&
		(k_uptime_get_32() - state.shutdown_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.shutdown_confirm_time = 0;
	}

#if MC_DISPLAY_COLOR_PANEL
	if (mc_display_has_color()) {
		render_shutdown_color();
		return;
	}
#endif
	render_shutdown_mono();
}

static void render_status(void)
{
	char buf[28];
	int y = CONTENT_Y;
	bool color = mc_display_has_color();

	if (ui_tiny()) {
		uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);

		snprintf(buf, sizeof(buf), "Up %ud%02uh",
			 up_s / 86400, (up_s % 86400) / 3600);
		draw_centered(centered_row(0, 3), buf);

		if (state.rtc_epoch > 1735689600) {
			uint32_t ds = state.rtc_epoch % 86400;

			snprintf(buf, sizeof(buf), "%02u:%02u UTC",
				 (unsigned)(ds / 3600), (unsigned)((ds % 3600) / 60));
		} else {
			snprintf(buf, sizeof(buf), "no time");
		}
		draw_centered(centered_row(1, 3), buf);

		if (state.battery_mv > 0) {
			uint8_t pct = state.battery_pct ? state.battery_pct
						        : calc_battery_pct(state.battery_mv);

			snprintf(buf, sizeof(buf), "Batt %u%%", pct);
			draw_centered(centered_row(2, 3), buf);
		}
		return;
	}

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
	[UI_PAGE_TRAFFIC]   = render_traffic,
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

	/* Tiny panels flash the page title briefly after a change, then fall
	 * through to content on the follow-up render (scheduled by the UI task
	 * via ui_pages_flash_remaining_ms()). */
	if (ui_tiny() && k_uptime_get_32() < tiny_flash_until) {
		render_tiny_title();
		mc_display_finalize();
		return;
	}

	render_top_bar();
	render_page_indicator();

	enum ui_page page = active_pages[current_page_idx];

	if (page < UI_PAGE_COUNT && renderers[page]) {
		renderers[page]();
	}

	mc_display_finalize();
}

/* Milliseconds until the tiny-panel title flash expires (0 when not flashing).
 * The UI task uses this to schedule the one follow-up render that swaps the
 * flashed title for page content. */
uint32_t ui_pages_flash_remaining_ms(void)
{
	if (!ui_tiny()) {
		return 0;
	}

	uint32_t now = k_uptime_get_32();

	return (now < tiny_flash_until) ? (tiny_flash_until - now) : 0;
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
	tiny_flash_kick();
	current_page_idx++;
	if (current_page_idx >= ACTIVE_PAGE_COUNT) {
		current_page_idx = 0;
	}
	state.current_page = active_pages[current_page_idx];
}

void ui_pages_prev(void)
{
	state.shutdown_confirm_time = 0;  /* Reset confirmation on navigate */
	tiny_flash_kick();
	current_page_idx--;
	if (current_page_idx < 0) {
		current_page_idx = ACTIVE_PAGE_COUNT - 1;
	}
	state.current_page = active_pages[current_page_idx];
}

void ui_pages_set(enum ui_page page)
{
	tiny_flash_kick();
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
