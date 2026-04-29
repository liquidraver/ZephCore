/*
 * ZephCore - Display Abstraction (CFB)
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wraps Zephyr's Character Framebuffer (CFB) subsystem.
 * Auto-detects any Zephyr-supported display from devicetree:
 *   1. "zephyr,display" chosen node (standard — works for any display)
 *   2. Legacy nodelabels: sh1106, ssd1306 (backwards compat)
 *
 * Resolution is queried from the driver at runtime — no hardcoded
 * dimensions.  Layout code should use mc_display_width/height().
 *
 * Auto-off timer turns display off after CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS.
 */

#include "display.h"
#include "doom_game.h"

#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_display, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ========== State ========== */

static const struct device *disp_dev;
static bool disp_on;
static bool disp_initialized;

/* Runtime display geometry (queried from driver) */
static uint16_t disp_width;
static uint16_t disp_height;
static uint8_t  font_w;
static uint8_t  font_h;
static bool     is_epd;       /* true for e-paper displays */

/* Optional symmetric inset (pixels).  Shrinks reported width/height and
 * offsets all draw primitives so panels with edge artefacts can hide them
 * behind a clean background margin. */
#define DISP_INSET ((int)CONFIG_ZEPHCORE_DISPLAY_INSET)

/* Optional display backlight regulator (e.g. e-paper frontlight).
 * Boards define a "disp_pwr_enable" regulator-fixed node to gate the
 * backlight circuit.  When present, backlight follows display on/off. */
#if DT_NODE_EXISTS(DT_NODELABEL(disp_pwr_enable))
static const struct device *backlight_reg =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(disp_pwr_enable));
#else
static const struct device *backlight_reg;
#endif

static bool backlight_on;

/* EPD frame change detection (Arduino-style):
 * hash draw calls across a frame and skip hardware flush if unchanged. */
static uint32_t epd_frame_hash;
static uint32_t epd_last_frame_hash = UINT_MAX;

static inline void epd_hash_bytes(const void *data, size_t len)
{
	if (!is_epd || !data || len == 0) {
		return;
	}

	const uint8_t *p = (const uint8_t *)data;

	for (size_t i = 0; i < len; i++) {
		/* FNV-1a */
		epd_frame_hash ^= p[i];
		epd_frame_hash *= 16777619u;
	}
}

static inline void epd_hash_u32(uint32_t v)
{
	epd_hash_bytes(&v, sizeof(v));
}

static inline void backlight_set(bool on)
{
	if (backlight_reg && device_is_ready(backlight_reg) && on != backlight_on) {
		if (on) {
			regulator_enable(backlight_reg);
		} else {
			regulator_disable(backlight_reg);
		}
		backlight_on = on;
	}
}

/* Auto-off work */
static struct k_work_delayable auto_off_work;

static void auto_off_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	/* Don't blank display while Doom easter egg is playing */
	if (doom_game_is_running()) {
		return;
	}
	/* E-paper content persists without power — blanking wastes a full
	 * refresh cycle (~2s) for no benefit.  Just turn off the backlight
	 * and mark display "off" so the next button press triggers
	 * mc_display_on() → backlight restore. */
	if (is_epd) {
		backlight_set(false);
		disp_on = false;
		return;
	}
	if (disp_on) {
		mc_display_off();
	}
}

/* ========== Early blanking ==========
 * OLED controllers (SSD1306, SH1106) turn the display ON during driver init,
 * showing stale VRAM from before reset.  Our mc_display_init() runs much later
 * (after BLE, LoRa, etc.), so there's a visible garbage flash.
 *
 * Fix: SYS_INIT hook runs right after the driver, sending "Display OFF" before
 * main() starts.  This is harmless for non-OLED displays (blanking is a no-op
 * or already blanked). */
static int display_early_blank(void)
{
	const struct device *dev = NULL;

	/* Try standard chosen node first */
#if DT_HAS_CHOSEN(zephyr_display)
	dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
#endif
	/* Legacy nodelabel fallback */
	if (!dev) {
		dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sh1106));
	}
	if (!dev) {
		dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ssd1306));
	}
	if (dev && device_is_ready(dev)) {
		/* EPD displays are bistable and already show clean white after
		 * driver init's full refresh.  Calling blanking_on here would
		 * leave blanking_on=true so the subsequent blanking_off in
		 * mc_display_init() triggers an extra unnecessary full refresh.
		 * Skip blanking for EPD; OLED still needs it to hide stale VRAM. */
		struct display_capabilities caps;

		display_get_capabilities(dev, &caps);
		if (!(caps.screen_info & SCREEN_INFO_EPD)) {
			display_blanking_on(dev);
		}
	}
	return 0;
}
SYS_INIT(display_early_blank, APPLICATION, 99);

/* ========== Public API ========== */

int mc_display_init(void)
{
	/* Find display device from devicetree.
	 * Priority: zephyr,display chosen > sh1106 nodelabel > ssd1306 nodelabel.
	 * This supports any Zephyr display driver (SSD1306, SH1106, ST7735,
	 * ILI9341, SSD1681 e-ink, etc.) via the standard chosen mechanism. */
#if DT_HAS_CHOSEN(zephyr_display)
	disp_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
#endif
	if (!disp_dev) {
		disp_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sh1106));
	}
	if (!disp_dev) {
		disp_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ssd1306));
	}

	if (!disp_dev || !device_is_ready(disp_dev)) {
		LOG_INF("no display found - display disabled");
		return -ENODEV;
	}

	/* Query actual resolution from display driver */
	struct display_capabilities caps;

	display_get_capabilities(disp_dev, &caps);
	disp_width = caps.x_resolution;
	disp_height = caps.y_resolution;
	is_epd = (caps.screen_info & SCREEN_INFO_EPD) != 0;

	LOG_INF("display: %ux%u%s", disp_width, disp_height,
		is_epd ? " (e-paper)" : "");

	/* OLED: blank before CFB init so stale VRAM isn't visible while we
	 * build the first frame.  EPD: driver init already performed a clean
	 * full refresh — the panel shows white.  Skip blanking to avoid the
	 * extra full refresh that blanking_off would trigger. */
	if (!is_epd) {
		display_blanking_on(disp_dev);
	}

	/* Initialize CFB */
	int ret = cfb_framebuffer_init(disp_dev);

	if (ret) {
		LOG_ERR("CFB init failed: %d", ret);
		return ret;
	}

	/* Font selection.
	 * Default: smallest height for best text density — our custom 6x8
	 * Latin-1 font typically wins on OLEDs.
	 * LARGE_FONT: smallest font whose height is >= 16 — picks Zephyr's
	 * built-in 10x16 (cfb_fonts.c) for larger e-paper panels where 6x8
	 * is too small to read.  Falls back to smallest-overall if no tall
	 * font is compiled in. */
	const bool want_large = IS_ENABLED(CONFIG_ZEPHCORE_DISPLAY_LARGE_FONT);
	int num_fonts = cfb_get_numof_fonts(disp_dev);

	LOG_DBG("display: %d fonts available", num_fonts);

	int best_idx = -1;
	uint8_t best_h = 255;

	for (int i = 0; i < num_fonts; i++) {
		uint8_t fw = 0, fh = 0;

		cfb_get_font_size(disp_dev, i, &fw, &fh);
		LOG_DBG("  font[%d]: %ux%u", i, fw, fh);
		if (want_large) {
			if (fh >= 16 && fh < best_h) {
				best_h = fh;
				best_idx = i;
			}
		} else {
			if (fh < best_h) {
				best_h = fh;
				best_idx = i;
			}
		}
	}
	if (best_idx < 0) {
		/* No font satisfied the LARGE_FONT threshold — fall back to
		 * the smallest so we still render something. */
		best_idx = 0;
		for (int i = 0; i < num_fonts; i++) {
			uint8_t fw = 0, fh = 0;

			cfb_get_font_size(disp_dev, i, &fw, &fh);
			if (fh < best_h) {
				best_h = fh;
				best_idx = i;
			}
		}
	}

	cfb_framebuffer_set_font(disp_dev, best_idx);
	cfb_get_font_size(disp_dev, best_idx, &font_w, &font_h);
	LOG_INF("display: selected font[%d] (%ux%u)", best_idx, font_w, font_h);

	/* CFB inversion no longer needed — Zephyr commit 2374ef62f97 fixed
	 * the MONO10/MONO01 polarity logic in cfb_framebuffer_finalize().
	 * SSD1306 OLED reports MONO01 by default, which CFB now handles
	 * correctly (white pixels on black background) without manual invert. */

	/* Clear CPU-side framebuffer (zeroes the RAM buffer — no SPI transfer). */
	cfb_framebuffer_clear(disp_dev, false);

	/* Unblank the display so the driver uses partial refresh for all
	 * subsequent renders (ssd16xx: partial_refresh = !blanking_on).
	 * OLED: also push a blank frame first to clear stale VRAM.
	 * EPD: skip the frame push — partial refresh will write real content. */
	if (!is_epd) {
		cfb_framebuffer_finalize(disp_dev);
	}
	display_blanking_off(disp_dev);
	backlight_set(true);
	disp_on = true;
	disp_initialized = true;

	/* Set up auto-off timer and schedule initial timeout */
	k_work_init_delayable(&auto_off_work, auto_off_handler);
	mc_display_reset_auto_off();

	LOG_INF("display initialized (%ux%u, font %ux%u)",
		disp_width, disp_height, font_w, font_h);
	return 0;
}

uint16_t mc_display_width(void)
{
	int w = (int)disp_width - 2 * DISP_INSET;

	return (w > 0) ? (uint16_t)w : 0;
}

uint16_t mc_display_height(void)
{
	int h = (int)disp_height - 2 * DISP_INSET;

	return (h > 0) ? (uint16_t)h : 0;
}

uint8_t mc_display_font_width(void)
{
	return font_w;
}

uint8_t mc_display_font_height(void)
{
	return font_h;
}

void mc_display_on(void)
{
	if (!disp_initialized) {
		return;
	}

	if (!disp_on) {
		/* EPD: content persists (bistable) — no need to unblank,
		 * just restore backlight.  OLED: actually unblank. */
		if (!is_epd) {
			display_blanking_off(disp_dev);
		}
		disp_on = true;
	}
	backlight_set(true);

	mc_display_reset_auto_off();
}

void mc_display_off(void)
{
	if (!disp_initialized) {
		return;
	}

	if (disp_on) {
		display_blanking_on(disp_dev);
		backlight_set(false);
		disp_on = false;
	}
}

bool mc_display_is_on(void)
{
	return disp_on;
}

bool mc_display_is_epd(void)
{
	return is_epd;
}

void mc_display_clear(void)
{
	if (!disp_initialized) {
		return;
	}

	if (is_epd) {
		epd_frame_hash = 2166136261u;
	}
	cfb_framebuffer_clear(disp_dev, false);
}

void mc_display_text(int x, int y, const char *text, bool invert)
{
	if (!disp_initialized || !text) {
		return;
	}

	if (invert) {
		cfb_framebuffer_invert(disp_dev);
	}

	if (is_epd) {
		epd_hash_u32((uint32_t)x);
		epd_hash_u32((uint32_t)y);
		epd_hash_u32(invert ? 1u : 0u);
		epd_hash_bytes(text, strlen(text));
	}
	cfb_print(disp_dev, text, x + DISP_INSET, y + DISP_INSET);

	if (invert) {
		cfb_framebuffer_invert(disp_dev);
	}
}

void mc_display_fill_rect(int x, int y, int w, int h)
{
	if (!disp_initialized) {
		return;
	}

	/* CFB doesn't have a native fill_rect, so we draw line by line */
	const int row_clamp = (int)disp_height - DISP_INSET;

	if (is_epd) {
		epd_hash_u32((uint32_t)x);
		epd_hash_u32((uint32_t)y);
		epd_hash_u32((uint32_t)w);
		epd_hash_u32((uint32_t)h);
	}
	for (int row = y + DISP_INSET; row < y + h + DISP_INSET && row < row_clamp; row++) {
		struct cfb_position start = { .x = x + DISP_INSET, .y = row };
		struct cfb_position end = { .x = x + w - 1 + DISP_INSET, .y = row };
		cfb_draw_line(disp_dev, &start, &end);
	}
}

void mc_display_hline(int x, int y, int w)
{
	if (!disp_initialized) {
		return;
	}

	struct cfb_position start = { .x = x + DISP_INSET, .y = y + DISP_INSET };
	struct cfb_position end = { .x = x + w - 1 + DISP_INSET, .y = y + DISP_INSET };

	if (is_epd) {
		epd_hash_u32((uint32_t)x);
		epd_hash_u32((uint32_t)y);
		epd_hash_u32((uint32_t)w);
	}
	cfb_draw_line(disp_dev, &start, &end);
}

void mc_display_xbm(int x, int y, const uint8_t *data, int w, int h)
{
	if (!disp_initialized || !data) {
		return;
	}

	/* Adafruit drawBitmap format (MSB first): row-major, bit 7 = leftmost.
	 * This matches the Arduino MeshCore logo data from icons.h.
	 * Each row is padded to byte boundary: bytes_per_row = (w+7)/8 */
	int bytes_per_row = (w + 7) / 8;
	size_t bitmap_len = (size_t)bytes_per_row * (size_t)h;

	if (is_epd) {
		epd_hash_u32((uint32_t)x);
		epd_hash_u32((uint32_t)y);
		epd_hash_u32((uint32_t)w);
		epd_hash_u32((uint32_t)h);
		epd_hash_bytes(data, bitmap_len);
	}

	for (int row = 0; row < h; row++) {
		for (int col = 0; col < w; col++) {
			int byte_idx = row * bytes_per_row + col / 8;
			int bit_idx = 7 - (col % 8);  /* MSB first */

			if (data[byte_idx] & (1 << bit_idx)) {
				struct cfb_position pos = {
					.x = (int16_t)(x + col + DISP_INSET),
					.y = (int16_t)(y + row + DISP_INSET)
				};
				cfb_draw_point(disp_dev, &pos);
			}
		}
	}
}

void mc_display_finalize(void)
{
	if (!disp_initialized) {
		return;
	}

	/* Don't let CFB overwrite display while Doom is rendering directly */
	if (doom_game_is_running()) {
		return;
	}

	if (is_epd && epd_frame_hash == epd_last_frame_hash) {
		return;
	}
	cfb_framebuffer_finalize(disp_dev);
	if (is_epd) {
		epd_last_frame_hash = epd_frame_hash;
	}
}

void mc_display_reset_auto_off(void)
{
	if (!disp_initialized) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS
	uint32_t timeout = CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS;

	if (timeout > 0) {
		k_work_reschedule(&auto_off_work, K_MSEC(timeout));
	}
#endif
}

void mc_display_epd_full_reset(void)
{
	if (!disp_initialized || !is_epd) {
		return;
	}

	/* Force the SSD16xx path back through a full-refresh cycle before
	 * entering steady-state partial updates for page rendering. */
	display_blanking_on(disp_dev);
	display_blanking_off(disp_dev);
	epd_last_frame_hash = UINT_MAX;

	/* After splash handoff, keep frontlight off; next user interaction
	 * wakes it via mc_display_on(). */
	backlight_set(false);
	disp_on = false;
}

const struct device *mc_display_get_device(void)
{
	return disp_initialized ? disp_dev : NULL;
}
