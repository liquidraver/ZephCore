/*
 * ZephCore - Display Abstraction (CFB)
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
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
 * Draw a horizontal line.
 */
void mc_display_hline(int x, int y, int w);

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
 * Flush the framebuffer to the display hardware.
 * Call after all drawing operations for a frame are complete.
 */
void mc_display_finalize(void);

/**
 * Reset the auto-off timer (called on user interaction).
 */
void mc_display_reset_auto_off(void);

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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_DISPLAY_H */
