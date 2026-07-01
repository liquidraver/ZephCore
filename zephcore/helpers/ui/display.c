/*
 * ZephCore - Display Abstraction (CFB)
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
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
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>

/* The SSD16xx EPD driver exposes ssd16xx_clear_red_ram() to reset the
 * partial-refresh old-frame reference after a periodic full refresh.  It is
 * only compiled when an SSD16xx-family panel is present in devicetree. */
#define ZEPHCORE_DISPLAY_HAS_SSD16XX \
	(DT_HAS_COMPAT_STATUS_OKAY(solomon_ssd1608) || \
	 DT_HAS_COMPAT_STATUS_OKAY(solomon_ssd1673) || \
	 DT_HAS_COMPAT_STATUS_OKAY(solomon_ssd1675a) || \
	 DT_HAS_COMPAT_STATUS_OKAY(solomon_ssd1680) || \
	 DT_HAS_COMPAT_STATUS_OKAY(solomon_ssd1681))
#if ZEPHCORE_DISPLAY_HAS_SSD16XX
#include <zephyr/display/ssd16xx.h>
#endif

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
static bool     has_color;    /* true when a raw RGB565 TFT is available */

const uint8_t *zephcore_font_6x8_glyph(uint8_t c);

#define COLOR_TEXT_MAX_CHARS 32
#define COLOR_MAX_OPS        72
#define COLOR_MAX_WIDTH      320

enum color_op_type {
	COLOR_OP_TEXT,
	COLOR_OP_RECT,
};

struct color_op {
	enum color_op_type type;
	int16_t x;
	int16_t y;
	int16_t w;
	int16_t h;
	uint16_t color;
	char text[COLOR_TEXT_MAX_CHARS];
};

static struct color_op color_ops[COLOR_MAX_OPS];
static uint8_t color_op_count;
static uint16_t color_line[COLOR_MAX_WIDTH];

#if DT_NODE_EXISTS(DT_NODELABEL(tft))
static const struct device *color_dev =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(tft));
#else
static const struct device *color_dev;
#endif

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

/* Optional TFT panel VDD regulator (e.g. T114 P0.03).
 * Enabled once before CFB init and never disabled => panel VDD must stay on.
 * Screenless builds omit display.c entirely so this is never called. */
#if DT_NODE_EXISTS(DT_NODELABEL(tft_pwr_enable))
static const struct device *panel_vdd_reg =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(tft_pwr_enable));
#else
static const struct device *panel_vdd_reg;
#endif

/* EPD frame change detection (Arduino-style):
 * hash draw calls across a frame and skip hardware flush if unchanged. */
static uint32_t epd_frame_hash;
static uint32_t epd_last_frame_hash = UINT_MAX;

/* Periodic full refresh: count partial refreshes and force a full refresh
 * every CONFIG_ZEPHCORE_DISPLAY_EPD_FULL_REFRESH_INTERVAL frames to clear
 * accumulated e-paper ghosting.  0 = disabled (partial-only). */
static uint32_t epd_partial_count;

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

static inline void panel_vdd_enable(void)
{
	if (panel_vdd_reg && device_is_ready(panel_vdd_reg)) {
		regulator_enable(panel_vdd_reg);
	}
}

static void color_overlay_probe(void)
{
	has_color = false;

	if (!color_dev || color_dev == disp_dev || !device_is_ready(color_dev)) {
		return;
	}

	struct display_capabilities caps;

	display_get_capabilities(color_dev, &caps);
	if ((caps.current_pixel_format == PIXEL_FORMAT_RGB_565 ||
	     caps.current_pixel_format == PIXEL_FORMAT_RGB_565X ||
	     (caps.supported_pixel_formats & (PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_565X))) &&
	    !(caps.screen_info & SCREEN_INFO_EPD)) {
		has_color = true;
		LOG_INF("display: color overlay enabled");
	}
}

static bool color_queue(enum color_op_type type, int x, int y, int w, int h,
			const char *text, uint16_t color)
{
	if (!has_color || color_op_count >= COLOR_MAX_OPS) {
		return false;
	}

	struct color_op *op = &color_ops[color_op_count++];

	op->type = type;
	op->x = (int16_t)x;
	op->y = (int16_t)y;
	op->w = (int16_t)w;
	op->h = (int16_t)h;
	op->color = color;
	op->text[0] = '\0';
	if (text) {
		strncpy(op->text, text, sizeof(op->text) - 1);
		op->text[sizeof(op->text) - 1] = '\0';
	}
	return true;
}

static void color_write_rect_now(int x, int y, int w, int h, uint16_t color)
{
	if (!has_color || w <= 0 || h <= 0) {
		return;
	}

	x += DISP_INSET;
	y += DISP_INSET;
	if (x < 0) {
		w += x;
		x = 0;
	}
	if (y < 0) {
		h += y;
		y = 0;
	}
	if (x + w > (int)disp_width) {
		w = (int)disp_width - x;
	}
	if (y + h > (int)disp_height) {
		h = (int)disp_height - y;
	}
	if (w <= 0 || h <= 0) {
		return;
	}
	if (w > COLOR_MAX_WIDTH) {
		w = COLOR_MAX_WIDTH;
	}

	uint16_t be = sys_cpu_to_be16(color);
	for (int i = 0; i < w; i++) {
		color_line[i] = be;
	}

	const struct display_buffer_descriptor desc = {
		.buf_size = (uint32_t)w * 2U,
		.width = (uint16_t)w,
		.height = 1U,
		.pitch = (uint16_t)w,
	};

	for (int row = 0; row < h; row++) {
		display_write(color_dev, (uint16_t)x, (uint16_t)(y + row),
			      &desc, color_line);
	}
}

static void color_write_char_now(int x, int y, uint8_t c, uint16_t color)
{
	uint16_t glyph_buf[6 * 8];
	const uint8_t *glyph = zephcore_font_6x8_glyph(c);
	uint16_t fg = sys_cpu_to_be16(color);
	uint16_t bg = sys_cpu_to_be16(MC_COLOR_BLACK);

	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 6; col++) {
			bool on = (glyph[col] >> row) & 0x01;
			glyph_buf[row * 6 + col] = on ? fg : bg;
		}
	}

	const struct display_buffer_descriptor desc = {
		.buf_size = sizeof(glyph_buf),
		.width = 6,
		.height = 8,
		.pitch = 6,
	};

	display_write(color_dev, (uint16_t)(x + DISP_INSET),
		      (uint16_t)(y + DISP_INSET), &desc, glyph_buf);
}

static void color_write_text_now(int x, int y, const char *text, uint16_t color)
{
	if (!has_color || !text) {
		return;
	}

	for (const char *p = text; *p; p++, x += 6) {
		uint8_t c = (uint8_t)*p;

		if (c < 32) {
			c = '?';
		}
		if (x + 6 > (int)mc_display_width() || y + 8 > (int)mc_display_height()) {
			break;
		}
		color_write_char_now(x, y, c, color);
	}
}

static void color_flush_ops(void)
{
	if (!has_color) {
		color_op_count = 0;
		return;
	}

	for (uint8_t i = 0; i < color_op_count; i++) {
		struct color_op *op = &color_ops[i];

		if (op->type == COLOR_OP_RECT) {
			color_write_rect_now(op->x, op->y, op->w, op->h, op->color);
		} else {
			color_write_text_now(op->x, op->y, op->text, op->color);
		}
	}
	color_op_count = 0;
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
	color_overlay_probe();

	/* OLED: blank before CFB init so stale VRAM isn't visible while we
	 * build the first frame.  EPD: driver init already performed a clean
	 * full refresh — the panel shows white.  Skip blanking to avoid the
	 * extra full refresh that blanking_off would trigger. */
	if (!is_epd) {
		display_blanking_on(disp_dev);
	}

	/* Enable panel VDD before CFB init so the controller is powered
	 * when the init sequence is sent over SPI. */
	panel_vdd_enable();

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

bool mc_display_has_color(void)
{
	return has_color;
}

void mc_display_clear(void)
{
	if (!disp_initialized) {
		return;
	}

	color_op_count = 0;
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

void mc_display_color_text(int x, int y, const char *text, uint16_t color)
{
	if (!disp_initialized || !text) {
		return;
	}

	if (!color_queue(COLOR_OP_TEXT, x, y, 0, 0, text, color)) {
		mc_display_text(x, y, text, false);
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

void mc_display_color_fill_rect(int x, int y, int w, int h, uint16_t color)
{
	if (!disp_initialized) {
		return;
	}

	if (!color_queue(COLOR_OP_RECT, x, y, w, h, NULL, color)) {
		mc_display_fill_rect(x, y, w, h);
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

void mc_display_invert_rect(int x, int y, int w, int h)
{
	if (!disp_initialized) {
		return;
	}
	cfb_invert_area(disp_dev,
		(uint16_t)(x + DISP_INSET), (uint16_t)(y + DISP_INSET),
		(uint16_t)w, (uint16_t)h
	);
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

	const int full_interval = CONFIG_ZEPHCORE_DISPLAY_EPD_FULL_REFRESH_INTERVAL;

	if (is_epd && full_interval > 0 && ++epd_partial_count >= (uint32_t)full_interval) {
		/* Periodic full refresh to clear accumulated ghosting.
		 *
		 * Replicate the clean post-boot sequence: full-refresh the panel
		 * to WHITE, then redraw the current page as a partial.  blanking_on
		 * selects the FULL profile; ssd16xx_fill_ram_white clears BOTH RAM
		 * banks to white; blanking_off runs the full-refresh waveform,
		 * leaving the panel (and both RAM banks) white.  The trailing
		 * cfb_framebuffer_finalize then redraws the current frame — still
		 * held in the CFB buffer — as a partial against the all-white
		 * reference: every content pixel is actively driven and there is
		 * nothing stale to erase.
		 *
		 * Going through white is required.  A partial after a full refresh
		 * of *content* leaves unchanged regions (e.g. the top bar) on the
		 * neutral waveform → they fade; and forcing the old-frame reference
		 * white while the panel still shows content fails to erase pixels
		 * that should clear → letters overlap.  The brief white flash is
		 * the intended clean between frames. */
		display_blanking_on(disp_dev);
#if ZEPHCORE_DISPLAY_HAS_SSD16XX
		ssd16xx_fill_ram_white(disp_dev);
#endif
		display_blanking_off(disp_dev);
		cfb_framebuffer_finalize(disp_dev);
		epd_partial_count = 0;
		epd_last_frame_hash = epd_frame_hash;
		return;
	}

	cfb_framebuffer_finalize(disp_dev);
	color_flush_ops();

	if (is_epd) {
		epd_last_frame_hash = epd_frame_hash;
	}
}

static uint32_t auto_off_ms_override;

void mc_display_set_auto_off_ms(uint32_t ms)
{
	auto_off_ms_override = ms;
}

void mc_display_reset_auto_off(void)
{
	if (!disp_initialized) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS
	uint32_t timeout = auto_off_ms_override
		? auto_off_ms_override
		: CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS;

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
	epd_partial_count = 0;

	/* After splash handoff, keep frontlight off; next user interaction
	 * wakes it via mc_display_on(). */
	backlight_set(false);
	disp_on = false;
}

const struct device *mc_display_get_device(void)
{
	return disp_initialized ? disp_dev : NULL;
}

/* ========== UTF-8 to display-charset sanitizer ==========
 *
 * The 6x8 font covers Latin-1 at its native code points (0xA0-0xFF) and
 * hosts 32 Latin Extended-A letters (Hungarian, Czech, Slovak, Polish, ...)
 * in the otherwise-unused C1 range 128-159 (see cfb_font_0608.c). */

/* Latin Extended-A code points with a real glyph -> font slot 128-159 */
static const struct {
	uint16_t cp;
	uint8_t slot;
} latin2_slots[] = {
	{ 0x0104, 132 }, { 0x0105, 133 },	/* A/a ogonek */
	{ 0x0106, 134 }, { 0x0107, 135 },	/* C/c acute */
	{ 0x010C, 136 }, { 0x010D, 137 },	/* C/c caron */
	{ 0x0118, 140 }, { 0x0119, 141 },	/* E/e ogonek */
	{ 0x011A, 138 }, { 0x011B, 139 },	/* E/e caron */
	{ 0x0141, 142 }, { 0x0142, 143 },	/* L/l stroke */
	{ 0x0143, 144 }, { 0x0144, 145 },	/* N/n acute */
	{ 0x0150, 128 }, { 0x0151, 129 },	/* O/o double acute */
	{ 0x0158, 146 }, { 0x0159, 147 },	/* R/r caron */
	{ 0x015A, 148 }, { 0x015B, 149 },	/* S/s acute */
	{ 0x0160, 150 }, { 0x0161, 151 },	/* S/s caron */
	{ 0x016E, 152 }, { 0x016F, 153 },	/* U/u ring */
	{ 0x0170, 130 }, { 0x0171, 131 },	/* U/u double acute */
	{ 0x0179, 154 }, { 0x017A, 155 },	/* Z/z acute */
	{ 0x017B, 156 }, { 0x017C, 157 },	/* Z/z dot above */
	{ 0x017D, 158 }, { 0x017E, 159 },	/* Z/z caron */
};

/* Base-letter fold for all of Latin Extended-A (U+0100..U+017F), indexed by
 * cp - 0x100. Used for code points without a glyph slot above. */
static const char latin_ext_a_fold[] =
	"AaAaAa" "CcCcCcCc" "DdDd" "EeEeEeEeEe" "GgGgGgGg" "HhHh"
	"IiIiIiIiIi" "Ii" "Jj" "Kkk" "LlLlLlLlLl" "NnNnNnnNn"
	"OoOoOoOo" "RrRrRr" "SsSsSsSs" "TtTtTt" "UuUuUuUuUuUu"
	"Ww" "YyY" "ZzZzZz" "s";

static uint8_t display_charset_map(uint32_t cp)
{
	if (cp >= 0xA0 && cp <= 0xFF) {
		return (uint8_t)cp;	/* native Latin-1 glyph */
	}
	if (cp >= 0x100 && cp <= 0x17F) {
		for (size_t i = 0; i < ARRAY_SIZE(latin2_slots); i++) {
			if (latin2_slots[i].cp == cp) {
				return latin2_slots[i].slot;
			}
		}
		return (uint8_t)latin_ext_a_fold[cp - 0x100];
	}
	switch (cp) {	/* Romanian uses comma-below variants */
	case 0x218: return 'S';
	case 0x219: return 's';
	case 0x21A: return 'T';
	case 0x21B: return 't';
	default:    return 0;	/* drop: emoji, other scripts, ... */
	}
}

void utf8_to_display(char *dst, const char *src, size_t dst_size)
{
	size_t di = 0;
	size_t si = 0;

	if (dst_size == 0) {
		return;
	}
	while (src[si] && di < dst_size - 1) {
		uint8_t c = (uint8_t)src[si];
		uint32_t cp;
		int len;

		if (c < 0x80) {
			dst[di++] = (char)c;
			si++;
			continue;
		} else if ((c & 0xE0) == 0xC0) {
			cp = c & 0x1F;
			len = 2;
		} else if ((c & 0xF0) == 0xE0) {
			cp = c & 0x0F;
			len = 3;
		} else if ((c & 0xF8) == 0xF0) {
			cp = c & 0x07;
			len = 4;
		} else {
			/* Not a UTF-8 lead byte: already display-encoded text
			 * (or junk) — pass through so a second sanitizing pass
			 * is harmless. */
			dst[di++] = (char)c;
			si++;
			continue;
		}

		bool valid = true;
		for (int k = 1; k < len; k++) {
			if (((uint8_t)src[si + k] & 0xC0) != 0x80) {
				valid = false;
				break;
			}
			cp = (cp << 6) | ((uint8_t)src[si + k] & 0x3F);
		}
		if (!valid) {
			dst[di++] = (char)c;	/* lone lead byte: pass through */
			si++;
			continue;
		}
		si += len;

		uint8_t out = display_charset_map(cp);
		if (out) {
			dst[di++] = (char)out;
		}
	}
	dst[di] = '\0';
}

/* Legacy entry point: transcode + trim leading spaces (names are sometimes
 * space-padded to game sort order). */
void utf8_to_latin1(char *dst, const char *src, size_t dst_size)
{
	utf8_to_display(dst, src, dst_size);

	size_t start = 0;
	while (dst[start] == ' ') {
		start++;
	}
	if (start > 0) {
		memmove(dst, dst + start, strlen(dst + start) + 1);
	}
}
