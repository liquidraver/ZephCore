/*
 * ZephCore - UI Page Renderers
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
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
#include "display.h"

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ui_pages, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ZephCore logo bitmap - 128x13px, MSB-first (Adafruit drawBitmap format) */
static const uint8_t zephcore_logo[] = {
	0x00, 0x01, 0xff, 0x7f, 0xe7, 0xf8, 0x70, 0x70,
	0x3c, 0x01, 0xe0, 0x7f, 0xc3, 0xff, 0x00, 0x00,
	0x00, 0x01, 0xff, 0x7f, 0xe7, 0xfc, 0x70, 0x70,
	0xff, 0x07, 0xf8, 0x7f, 0xe3, 0xff, 0x00, 0x00,
	0x00, 0x01, 0xff, 0x7f, 0xe7, 0xfe, 0x70, 0x71,
	0xff, 0x0f, 0xfc, 0x7f, 0xf3, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x0e, 0x70, 0x07, 0x0e, 0x70, 0x71,
	0xc7, 0x8e, 0x1c, 0x70, 0x73, 0x80, 0x00, 0x00,
	0x00, 0x00, 0x1c, 0x70, 0x07, 0x0e, 0x70, 0x73,
	0x83, 0x1c, 0x0e, 0x70, 0x73, 0x80, 0x00, 0x00,
	0x00, 0x00, 0x38, 0x7f, 0xe7, 0xfe, 0x7f, 0xf3,
	0x80, 0x1c, 0x0e, 0x7f, 0xf3, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x78, 0x7f, 0xe7, 0xfc, 0x7f, 0xf3,
	0x80, 0x1c, 0x0e, 0x7f, 0xe3, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x70, 0x7f, 0xe7, 0xf8, 0x7f, 0xf3,
	0x80, 0x1c, 0x0e, 0x7f, 0x83, 0xff, 0x00, 0x00,
	0x00, 0x00, 0xe0, 0x70, 0x07, 0x00, 0x70, 0x73,
	0x83, 0x1c, 0x0e, 0x73, 0xc3, 0x80, 0x00, 0x00,
	0x00, 0x01, 0xc0, 0x70, 0x07, 0x00, 0x70, 0x71,
	0xc7, 0x8e, 0x1c, 0x71, 0xe3, 0x80, 0x00, 0x00,
	0x00, 0x03, 0xff, 0x7f, 0xe7, 0x00, 0x70, 0x71,
	0xff, 0x0f, 0xfc, 0x70, 0xe3, 0xff, 0x00, 0x00,
	0x00, 0x03, 0xff, 0x7f, 0xe7, 0x00, 0x70, 0x70,
	0xff, 0x07, 0xf8, 0x70, 0xf3, 0xff, 0x00, 0x00,
	0x00, 0x03, 0xff, 0x7f, 0xe7, 0x00, 0x70, 0x70,
	0x3c, 0x01, 0xe0, 0x70, 0x7b, 0xff, 0x00, 0x00,
};

/* ========== UTF-8 to Latin-1 Sanitizer ========== */
/* Node names may contain UTF-8 emojis or accented chars.
 * Our font covers 32-255 (ASCII + Latin-1 Supplement).
 * This function:
 *   - Passes ASCII (0x00-0x7F) unchanged
 *   - Converts 2-byte Latin-1 sequences (U+00A0-U+00FF) to raw bytes
 *   - Strips all other multi-byte sequences (emojis, CJK, etc.)
 *   - Strips leading/trailing whitespace from removed chars
 */
void utf8_to_latin1(char *dst, const char *src, size_t dst_size)
{
	size_t di = 0;
	size_t si = 0;

	while (src[si] && di < dst_size - 1) {
		uint8_t c = (uint8_t)src[si];

		if (c < 0x80) {
			/* ASCII byte — pass through */
			dst[di++] = (char)c;
			si++;
		} else if ((c & 0xE0) == 0xC0) {
			/* 2-byte UTF-8: 110xxxxx 10xxxxxx */
			uint8_t c2 = (uint8_t)src[si + 1];

			if ((c2 & 0xC0) != 0x80) {
				/* Invalid continuation — skip lead byte */
				si++;
				continue;
			}

			uint16_t cp = ((c & 0x1F) << 6) | (c2 & 0x3F);

			if (cp >= 0xA0 && cp <= 0xFF) {
				/* Latin-1 Supplement — fits in our font */
				dst[di++] = (char)(uint8_t)cp;
			}
			/* else: Latin Extended etc — skip */
			si += 2;
		} else if ((c & 0xF0) == 0xE0) {
			/* 3-byte UTF-8 — skip (BMP: CJK, symbols) */
			si += 3;
		} else if ((c & 0xF8) == 0xF0) {
			/* 4-byte UTF-8 — skip (emojis, supplementary) */
			si += 4;
		} else {
			/* Invalid/continuation byte — skip */
			si++;
		}
	}
	dst[di] = '\0';

	/* Strip leading spaces (from removed emoji prefix) */
	size_t start = 0;

	while (dst[start] == ' ') {
		start++;
	}
	if (start > 0) {
		memmove(dst, dst + start, di - start + 1);
	}
}

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
	UI_PAGE_BUZZER,
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

static void render_top_bar(void)
{
	/* Node name on the left */
	if (state.node_name[0]) {
		/* Truncate to fit left side (leave room for battery) */
		char name[16];

		strncpy(name, state.node_name, sizeof(name) - 1);
		name[sizeof(name) - 1] = '\0';
		mc_display_text(0, TOP_BAR_Y, name, false);
	}

	/* Right side: "HH:MM XX%" — clock then battery, right-aligned */
	{
		char right[16] = "";
		int pos = 0;

		/* 24h clock from RTC epoch (only if time has been synced).
		 * Before sync, getCurrentTime() returns bare uptime (~seconds),
		 * so check for a sane epoch (after Jan 1 2025 = 1735689600). */
		if (state.rtc_epoch > 1735689600) {
			uint32_t day_sec = state.rtc_epoch % 86400;
			uint8_t hh = day_sec / 3600;
			uint8_t mm = (day_sec % 3600) / 60;

			pos = snprintf(right, sizeof(right), "%02u:%02u ", hh, mm);
		}

		/* Battery percentage */
		if (state.battery_mv > 0) {
			uint8_t pct = state.battery_pct;

			if (pct == 0) {
				pct = calc_battery_pct(state.battery_mv);
			}
			snprintf(right + pos, sizeof(right) - pos, "%u%%", pct);
		}

		if (right[0]) {
			int x = DISP_W - ((int)strlen(right) * FONT_W);

			mc_display_text(x, TOP_BAR_Y, right, false);
		}
	}
}

/* ========== Helper: Page Indicator Dots ========== */

static void render_page_indicator(void)
{
	/* Centered dots matching Arduino layout:
	 * Each dot spaced 10px apart, centered on screen.
	 * Current page = filled 3x3 block, others = single pixel. */
	int total = ACTIVE_PAGE_COUNT;
	int dot_spacing = 10;
	int total_width = (total - 1) * dot_spacing;
	int start_x = (DISP_W - total_width) / 2;

	for (int i = 0; i < total; i++) {
		int x = start_x + i * dot_spacing;

		if (i == current_page_idx) {
			/* Current page: filled 3x3 block */
			mc_display_fill_rect(x - 1, DOTS_Y, 3, 3);
		} else {
			/* Other pages: single pixel */
			mc_display_fill_rect(x, DOTS_Y + 1, 1, 1);
		}
	}

	/* Separator line below dots */
	mc_display_hline(0, SEP_Y, DISP_W);
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

/* ========== Page Renderers ========== */

static void render_messages(void)
{
	/* 3 centered rows: msg count, BLE status, offgrid status */
	char buf[24];

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
		draw_centered(centered_row(0, 1), "(no contacts heard)");
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
		mc_display_text(0, y, buf, false);
		y += LINE_H;
	}
}

static void render_radio(void)
{
	char buf[28];
	int y = CONTENT_Y;

	/* Line 1: FQ, SF, BW */
	uint32_t freq_mhz = state.lora_freq_hz / 1000000;
	uint32_t freq_frac = (state.lora_freq_hz % 1000000 + 500) / 1000;
	uint16_t bw_int = state.lora_bw_khz_x10 / 10;
	uint16_t bw_frac = state.lora_bw_khz_x10 % 10;

	if (bw_frac) {
		snprintf(buf, sizeof(buf), "%u.%03u SF%u BW%u.%u",
			 freq_mhz, freq_frac, state.lora_sf, bw_int, bw_frac);
	} else {
		snprintf(buf, sizeof(buf), "%u.%03u SF%u BW%u",
			 freq_mhz, freq_frac, state.lora_sf, bw_int);
	}
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	/* Line 2: CR, TX Power */
	snprintf(buf, sizeof(buf), "CR:%u  TX:%ddBm", state.lora_cr, state.lora_tx_power);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	/* Line 3: Noise floor */
	snprintf(buf, sizeof(buf), "Noise: %ddBm", state.lora_noise_floor);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	/* Uptime */
	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);
	uint32_t days = up_s / 86400;
	uint32_t hours = (up_s % 86400) / 3600;
	uint32_t mins = (up_s % 3600) / 60;

	snprintf(buf, sizeof(buf), "Up: %ud %uh %um", days, hours, mins);
	mc_display_text(0, y, buf, false);
}

static void render_bluetooth(void)
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

static void render_advert(void)
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

	/* No GPS hardware on this board */
	if (!state.gps_available) {
		mc_display_text(0, y, "GPS: not detected", false);
		return;
	}

	/* GPS enabled/disabled state */
	snprintf(buf, sizeof(buf), "GPS: %s",
		 state.gps_enabled ? "on" : "off");
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	if (!state.gps_enabled) {
		draw_centered(y + 8, "Press to Enable");
		return;
	}

	/* State-dependent display */
	if (state.gps_state == 2) {
		/* ACQUIRING — actively searching for satellites */
		snprintf(buf, sizeof(buf), "Searching... sat:%u",
			 state.gps_satellites);
		mc_display_text(0, y, buf, false);
		y += LINE_H;

		/* Show last fix age if we have one */
		if (state.gps_last_fix_age_s != UINT32_MAX) {
			char tbuf[12];

			fmt_duration(tbuf, sizeof(tbuf), state.gps_last_fix_age_s);
			snprintf(buf, sizeof(buf), "Last fix: %s ago", tbuf);
			mc_display_text(0, y, buf, false);
		} else {
			mc_display_text(0, y, "No fix yet", false);
		}
	} else if (state.gps_state == 1) {
		/* STANDBY — sleeping between fix cycles */
		if (state.gps_last_fix_age_s != UINT32_MAX) {
			char tbuf[12];

			fmt_duration(tbuf, sizeof(tbuf), state.gps_last_fix_age_s);
			snprintf(buf, sizeof(buf), "Last fix: %s ago", tbuf);
			mc_display_text(0, y, buf, false);
			y += LINE_H;
		} else {
			mc_display_text(0, y, "No fix yet", false);
			y += LINE_H;
		}

		if (state.gps_next_search_s > 0) {
			char tbuf[12];

			fmt_duration(tbuf, sizeof(tbuf), state.gps_next_search_s);
			snprintf(buf, sizeof(buf), "Next search: %s", tbuf);
			mc_display_text(0, y, buf, false);
		}
	} else {
		/* OFF — shouldn't reach here if gps_enabled is true */
		mc_display_text(0, y, "GPS off", false);
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

	/* Temperature */
	if (state.temperature_c10 != 0) {
		snprintf(buf, sizeof(buf), "Temp: %d.%d C",
			 state.temperature_c10 / 10,
			 abs(state.temperature_c10 % 10));
		mc_display_text(0, y, buf, false);
		y += LINE_H;
	}

	/* Pressure */
	if (state.pressure_pa != 0) {
		snprintf(buf, sizeof(buf), "Press: %u hPa",
			 (unsigned)(state.pressure_pa / 100));
		mc_display_text(0, y, buf, false);
		y += LINE_H;
	}

	/* Humidity */
	if (state.humidity_rh10 != 0) {
		snprintf(buf, sizeof(buf), "Humid: %u.%u%%",
			 state.humidity_rh10 / 10, state.humidity_rh10 % 10);
		mc_display_text(0, y, buf, false);
		y += LINE_H;
	}

	/* Light */
	if (state.light_lux != 0) {
		snprintf(buf, sizeof(buf), "Light: %u lux", state.light_lux);
		mc_display_text(0, y, buf, false);
		y += LINE_H;
	}

	/* Battery at bottom */
	snprintf(buf, sizeof(buf), "Batt: %u%% (%umV)",
		 state.battery_pct > 0 ? state.battery_pct
				       : calc_battery_pct(state.battery_mv),
		 state.battery_mv);
	mc_display_text(0, y, buf, false);
}

static void render_offgrid(void)
{
	/* Check if confirmation has expired */
	if (state.offgrid_confirm_time != 0 &&
	    (k_uptime_get_32() - state.offgrid_confirm_time) > CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
		state.offgrid_confirm_time = 0;
	}

	draw_centered(centered_row(0, 3), "Offgrid Mode");

	char buf[24];

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

	/* Role label */
	draw_centered(y, "REPEATER");
	y += LINE_H;

	/* Uptime */
	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);
	uint32_t days = up_s / 86400;
	uint32_t hours = (up_s % 86400) / 3600;
	uint32_t mins = (up_s % 3600) / 60;

	snprintf(buf, sizeof(buf), "Up: %ud %uh %um", days, hours, mins);
	mc_display_text(0, y, buf, false);
	y += LINE_H;

	/* Clock — only if RTC has been synced (after Jan 1 2025) */
	if (state.rtc_epoch > 1735689600) {
		uint32_t day_sec = state.rtc_epoch % 86400;
		uint8_t hh = day_sec / 3600;
		uint8_t mm = (day_sec % 3600) / 60;
		uint8_t ss = day_sec % 60;

		snprintf(buf, sizeof(buf), "Time: %02u:%02u:%02u UTC", hh, mm, ss);
		mc_display_text(0, y, buf, false);
	} else {
		mc_display_text(0, y, "Time: not synced", false);
	}
	y += LINE_H;

	/* Battery */
	if (state.battery_mv > 0) {
		uint8_t pct = state.battery_pct;

		if (pct == 0) {
			pct = calc_battery_pct(state.battery_mv);
		}
		snprintf(buf, sizeof(buf), "Batt: %u%% (%umV)", pct, state.battery_mv);
		mc_display_text(0, y, buf, false);
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
