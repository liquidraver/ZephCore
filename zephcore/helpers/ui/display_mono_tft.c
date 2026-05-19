/*
 * ZephCore - Monochrome wrapper for RGB TFT displays
 * Copyright (c) 2026 ZephCore
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reports PIXEL_FORMAT_MONO01 to CFB so the framebuffer is ~4 KB
 * (240×135/8 = 4050 bytes) instead of the ~64 KB RGB565 buffer that
 * CFB would otherwise allocate for an ST7789V.
 *
 * On display_write(), converts row major 1bpp MONO01 (LSB first) to
 * RGB565 one row at a time and forwards each row to the underlying TFT.
 */

#define DT_DRV_COMPAT zephcore_mono_tft

#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_mono_tft);

struct mono_tft_config {
    const struct device *tft;
    uint16_t            width;
    uint16_t            height;
};

/* Hardcoded to instance 0. Only one display wrapper per board is supported. */
#define MONO_TFT_MAX_WIDTH DT_INST_PROP(0, width)

struct mono_tft_data {
    /* Static allocation avoids ~480 bytes of stack pressure per display_write() call. */
    uint16_t line_buf[MONO_TFT_MAX_WIDTH];
};

static int mono_tft_blanking_on(const struct device *dev)
{
    const struct mono_tft_config *cfg = dev->config;

    return display_blanking_on(cfg->tft);
}

static int mono_tft_blanking_off(const struct device *dev)
{
    const struct mono_tft_config *cfg = dev->config;

    return display_blanking_off(cfg->tft);
}

static int mono_tft_write(
    const struct device *dev,
    const uint16_t x_start, const uint16_t y_start,
    const struct display_buffer_descriptor *desc,
    const void *buf
){
    const struct mono_tft_config *cfg = dev->config;
    struct mono_tft_data *data = dev->data;
    const uint8_t *mono_buf = buf;
    /* Pitch is in pixels. Convert to bytes by rounding up to the nearest boundary. */
    uint16_t pitch_bytes = (desc->pitch + 7U) / 8U;

    const struct display_buffer_descriptor line_desc = {
        .buf_size = (uint32_t)desc->width * 2U,
        .width    = desc->width,
        .height   = 1U,
        .pitch    = desc->width,
    };

    int err = 0;

    for (uint16_t row = 0; row < desc->height; row++) {
        const uint8_t *mono_row = mono_buf + (size_t)row * pitch_bytes;

        for (uint16_t col = 0; col < desc->width; col++) {
            /* CFB MONO01 is row major and LSB first. Pixel at col uses bit (col%8) of byte (col/8). */
            bool pixel_on = (mono_row[col / 8U] >> (col & 7U)) & 1U;
            /* ST7789V expects RGB565 in big endian byte order over SPI. */
            data->line_buf[col] = pixel_on ? sys_cpu_to_be16(0xFFFFU) : 0x0000U;
        }

        err = display_write(cfg->tft, x_start, y_start + row, &line_desc, data->line_buf);
        if (err) {
            return err;
        }
    }
    return 0;
}

static void mono_tft_get_capabilities(
    const struct device *dev,
    struct display_capabilities *caps
){
    const struct mono_tft_config *cfg = dev->config;

    memset(caps, 0, sizeof(*caps));
    caps->x_resolution            = cfg->width;
    caps->y_resolution            = cfg->height;
    caps->supported_pixel_formats = PIXEL_FORMAT_MONO01;
    caps->current_pixel_format    = PIXEL_FORMAT_MONO01;
}

static int mono_tft_set_pixel_format(
    const struct device *dev,
    const enum display_pixel_format pixel_format
){
    ARG_UNUSED(dev);
    return (pixel_format == PIXEL_FORMAT_MONO01) ? 0 : -ENOTSUP;
}

static int mono_tft_init(const struct device *dev)
{
    const struct mono_tft_config *cfg = dev->config;

    if (!device_is_ready(cfg->tft)) {
        LOG_ERR("underlying TFT device not ready");
        return -ENODEV;
    }
    return 0;
}

static const struct display_driver_api mono_tft_api = {
    .blanking_on      = mono_tft_blanking_on,
    .blanking_off     = mono_tft_blanking_off,
    .write            = mono_tft_write,
    .get_capabilities = mono_tft_get_capabilities,
    .set_pixel_format = mono_tft_set_pixel_format,
};

static struct mono_tft_data mono_tft_data;
static const struct mono_tft_config mono_tft_config = {
    .tft    = DEVICE_DT_GET(DT_INST_PHANDLE(0, tft_dev)),
    .width  = DT_INST_PROP(0, width),
    .height = DT_INST_PROP(0, height),
};

DEVICE_DT_INST_DEFINE(
    0,
    mono_tft_init,
    NULL,
    &mono_tft_data,
    &mono_tft_config,
    POST_KERNEL,
    CONFIG_DISPLAY_INIT_PRIORITY,
    &mono_tft_api
);
