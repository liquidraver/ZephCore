/*
 * ZephCore - Display Abstraction (CFB)
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Wraps Zephyr's Character Framebuffer (CFB) subsystem with:
 * - Auto-detection from devicetree (any Zephyr-supported display)
 * - Runtime resolution query (supports any size, not just 128x64)
 * - Auto-off timer via k_work_delayable
 * - Simple text/rect drawing API for UI pages
 *
 * All functions prefixed mc_display_ to avoid collision with
 * Zephyr's display_* namespace in <zephyr/drivers/display.h>.
 */

#ifndef ZEPHCORE_DISPLAY_H
#define ZEPHCORE_DISPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the display from devicetree.
 * Detects any Zephyr-supported display via:
 *   1. "zephyr,display" chosen node (standard)
 *   2. Legacy nodelabels: sh1106, ssd1306 (backwards compat)
 *
 * Queries actual resolution from driver — no hardcoded dimensions.
 *
 * @return 0 on success, negative errno on failure, -ENODEV if no display
 */
int mc_display_init(void);

/**
 * Get display width in pixels (queried from hardware at init).
 * Returns 0 if display not initialized.
 */
uint16_t mc_display_width(void);

/**
 * Get display height in pixels (queried from hardware at init).
 * Returns 0 if display not initialized.
 */
uint16_t mc_display_height(void);

/**
 * Get active font width in pixels.
 * Returns 0 if display not initialized.
 */
uint8_t mc_display_font_width(void);

/**
 * Get active font height in pixels.
 * Returns 0 if display not initialized.
 */
uint8_t mc_display_font_height(void);

/**
 * Turn display on (wake from blanking).
 * Resets the auto-off timer.
 */
void mc_display_on(void);

/**
 * Turn display off (blanking).
 */
void mc_display_off(void);

/**
 * @return true if the display is currently on
 */
bool mc_display_is_on(void);

/**
 * @return true if the display is an e-paper (EPD) type.
 * EPD displays have slow refresh (~2s) and use zero power when static,
 * so callers should use longer update intervals and skip blanking.
 */
bool mc_display_is_epd(void);

/**
 * @return true when a raw RGB565-capable color panel is available for
 * optional color overlays. Monochrome displays return false.
 */
bool mc_display_has_color(void);

/**
 * Clear the framebuffer (fill with black).
 * Call before rendering a new frame.
 */
void mc_display_clear(void);

/**
 * Draw text at position.
 *
 * @param x      X position in pixels
 * @param y      Y position in pixels
 * @param text   Null-terminated string
 * @param invert If true, draw black text on white background
 */
void mc_display_text(int x, int y, const char *text, bool invert);

/* Common RGB565 colors for optional color-capable pages. */
#define MC_COLOR_BLACK    0x0000
#define MC_COLOR_WHITE    0xffff
#define MC_COLOR_GREEN    0x07e0
#define MC_COLOR_CYAN     0x07ff
#define MC_COLOR_YELLOW   0xffe0
#define MC_COLOR_ORANGE   0xfd20
#define MC_COLOR_RED      0xf800
#define MC_COLOR_BLUE     0x001f
#define MC_COLOR_GRAY     0x8410

/**
 * Draw text using RGB565 color when supported. On non-color displays this
 * falls back to mc_display_text(..., invert=false).
 *
 * Color overlays are flushed after the normal CFB frame in mc_display_finalize().
 */
void mc_display_color_text(int x, int y, const char *text, uint16_t color);

/**
 * Draw a filled rectangle.
 *
 * @param x  Top-left X
 * @param y  Top-left Y
 * @param w  Width
 * @param h  Height
 */
void mc_display_fill_rect(int x, int y, int w, int h);

/**
 * Draw a filled rectangle using RGB565 color when supported. On non-color
 * displays this falls back to mc_display_fill_rect().
 */
void mc_display_color_fill_rect(int x, int y, int w, int h, uint16_t color);

/**
 * Draw a horizontal line.
 */
void mc_display_hline(int x, int y, int w);

/**
 * Invert a rectangular region of the framebuffer.
 * Pixels that are on (white) become off (black) and vice versa.
 * Used to create clean dark-background modal overlays.
 */
void mc_display_invert_rect(int x, int y, int w, int h);

/**
 * Draw a monochrome bitmap (Adafruit/Arduino format).
 * MSB first, row-major, 1=foreground.
 * Compatible with Arduino's drawBitmap() and MeshCore icons.h data.
 *
 * @param x      Top-left X position
 * @param y      Top-left Y position
 * @param data   Bitmap data (MSB first, row-major)
 * @param w      Width in pixels
 * @param h      Height in pixels
 */
void mc_display_xbm(int x, int y, const uint8_t *data, int w, int h);

/**
 * ZephCore logo bitmap (128 × 13 px, MSB-first, row-major).
 * Shared by both UI variants' splash screens. Defined in ui_common.c.
 */
#define ZEPHCORE_LOGO_W  128
#define ZEPHCORE_LOGO_H  13
extern const uint8_t zephcore_logo[];

/**
 * Flush the framebuffer to the display hardware.
 * Call after all drawing operations for a frame are complete.
 */
void mc_display_finalize(void);

/**
 * Reset the auto-off timer (called on user interaction).
 */
void mc_display_reset_auto_off(void);

/**
 * Override the auto-off timeout (0 = revert to Kconfig default).
 * Call from the UI layer when the user changes the screen-off duration
 * so the Kconfig-driven timer and the UI timer stay in sync.
 */
void mc_display_set_auto_off_ms(uint32_t ms);

/**
 * EPD-only: force a full panel reset cycle before normal page rendering.
 * No-op on non-EPD displays or when display is not initialized.
 */
void mc_display_epd_full_reset(void);

/**
 * Get the raw display device pointer.
 * Used by easter egg (Doom) to bypass CFB and write directly.
 * Returns NULL if display not initialized.
 */
const struct device *mc_display_get_device(void);

/**
 * Convert UTF-8 text to the display charset for rendering.
 * Passes ASCII unchanged, converts Latin-1 (U+00A0-U+00FF) to its native
 * code points, maps 32 Latin-2 letters (Hungarian/Czech/Slovak/Polish/...)
 * into font slots 128-159, folds the rest of Latin Extended-A to base ASCII
 * letters, and strips everything else (emojis, CJK). Bytes that are not
 * valid UTF-8 pass through unchanged, so already-converted text survives a
 * second pass.
 */
void utf8_to_display(char *dst, const char *src, size_t dst_size);

/**
 * utf8_to_display() + leading-space trim (names are sometimes space-padded
 * to game sort order).
 */
void utf8_to_latin1(char *dst, const char *src, size_t dst_size);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_DISPLAY_H */
