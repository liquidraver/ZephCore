/*
 * SPDX-License-Identifier: MIT
 * LR11xx HAL implementation for Zephyr — ZephCore
 */

#include "lr11xx_hal_zephyr.h"
#include "lr11xx_hal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr11xx_hal, CONFIG_LORA_LOG_LEVEL);

/* Datasheet: firmware boot ≤273ms after reset; 3000ms gives margin for slow startup */
#define LR11XX_BUSY_TIMEOUT_MS  3000

static struct gpio_callback dio1_gpio_cb;
static lr11xx_dio1_callback_t dio1_user_cb = NULL;
static void *dio1_user_data = NULL;
static struct lr11xx_hal_context *current_ctx = NULL;

/* BUSY falling-edge interrupt wakes wait_on_busy() via semaphore */
static struct gpio_callback busy_gpio_cb;
static K_SEM_DEFINE(busy_sem, 0, 1);

static void busy_isr_callback(const struct device *dev, struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&busy_sem);
}

/* Last SPI opcode and timestamp — reported on BUSY timeout for diagnosis */
static uint16_t last_opcode;
static int64_t last_cmd_time;

/**
 * @brief Block until BUSY low or timeout; uses GPIO interrupt + semaphore (not polling).
 *
 * Double-checked locking: sample BUSY before and after enabling the interrupt
 * to avoid missing the falling edge between the two checks.
 */
static lr11xx_hal_status_t wait_on_busy(struct lr11xx_hal_context *ctx)
{
    if (!gpio_pin_get_dt(&ctx->busy)) {
        return LR11XX_HAL_STATUS_OK;
    }

    k_sem_reset(&busy_sem);
    gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_EDGE_TO_INACTIVE);

    if (!gpio_pin_get_dt(&ctx->busy)) {
        gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_DISABLE);
        return LR11XX_HAL_STATUS_OK;
    }

    int ret = k_sem_take(&busy_sem, K_MSEC(LR11XX_BUSY_TIMEOUT_MS));
    gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_DISABLE);

    if (ret == -EAGAIN) {
        LOG_ERR("BUSY timeout! last_op=0x%04x sent_at=%lld (%lld ms ago) DIO1=%d",
                last_opcode, last_cmd_time,
                k_uptime_get() - last_cmd_time,
                gpio_pin_get_dt(&ctx->dio1));
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief Ensure chip is ready; wake from sleep via NSS pulse if needed.
 *
 * Wakeup procedure per datasheet: assert NSS, hold ≥10µs, deassert, then wait BUSY low.
 */
static lr11xx_hal_status_t check_device_ready(struct lr11xx_hal_context *ctx)
{
    if (!ctx->radio_is_sleeping) {
        return wait_on_busy(ctx);
    }

    gpio_pin_set_dt(&ctx->nss, 1);
    k_busy_wait(10);  /* ≥10µs NSS hold for wakeup (datasheet §5.1) */
    gpio_pin_set_dt(&ctx->nss, 0);

    ctx->radio_is_sleeping = false;
    return wait_on_busy(ctx);
}

/* DIO1 rising-edge ISR; forwards to user callback (must be ISR-safe) */
static void dio1_isr_callback(const struct device *dev, struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (dio1_user_cb) {
        dio1_user_cb(dio1_user_data);
    }
}

int lr11xx_hal_init(struct lr11xx_hal_context *ctx)
{
    int ret;

    current_ctx = ctx;
    ctx->radio_is_sleeping = false;

    /* NSS idle = deselected; ACTIVE_LOW polarity handled by GPIO_OUTPUT_INACTIVE */
    ret = gpio_pin_configure_dt(&ctx->nss, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure NSS: %d", ret);
        return ret;
    }

    /* RESET idle = released; ACTIVE_LOW polarity handled by GPIO_OUTPUT_INACTIVE */
    ret = gpio_pin_configure_dt(&ctx->reset, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure RESET: %d", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&ctx->busy, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure BUSY: %d", ret);
        return ret;
    }

    /* BUSY interrupt enabled on-demand inside wait_on_busy() */
    gpio_init_callback(&busy_gpio_cb, busy_isr_callback, BIT(ctx->busy.pin));
    ret = gpio_add_callback(ctx->busy.port, &busy_gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add BUSY callback: %d", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&ctx->dio1, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure DIO1: %d", ret);
        return ret;
    }

    gpio_init_callback(&dio1_gpio_cb, dio1_isr_callback, BIT(ctx->dio1.pin));
    ret = gpio_add_callback(ctx->dio1.port, &dio1_gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add DIO1 callback: %d", ret);
        return ret;
    }

    LOG_INF("LR11xx HAL initialized");
    return 0;
}

void lr11xx_hal_set_dio1_callback(struct lr11xx_hal_context *ctx,
                                   lr11xx_dio1_callback_t cb, void *user_data)
{
    ARG_UNUSED(ctx);
    dio1_user_cb = cb;
    dio1_user_data = user_data;
}

void lr11xx_hal_enable_dio1_irq(struct lr11xx_hal_context *ctx)
{
    gpio_pin_interrupt_configure_dt(&ctx->dio1, GPIO_INT_EDGE_RISING);
}

void lr11xx_hal_disable_dio1_irq(struct lr11xx_hal_context *ctx)
{
    gpio_pin_interrupt_configure_dt(&ctx->dio1, GPIO_INT_DISABLE);
}

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
                                      const uint16_t command_length,
                                      const uint8_t *data, const uint16_t data_length)
{
    struct lr11xx_hal_context *ctx = (struct lr11xx_hal_context *)context;
    int ret;

    /* Track opcode for BUSY timeout diagnostics */
    if (command_length >= 2) {
        last_opcode = ((uint16_t)command[0] << 8) | command[1];
    }
    last_cmd_time = k_uptime_get();

    if (check_device_ready(ctx) != LR11XX_HAL_STATUS_OK) {
        LOG_ERR("hal_write: device not ready, op=0x%04x", last_opcode);
        return LR11XX_HAL_STATUS_ERROR;
    }

    const struct spi_buf tx_bufs[] = {
        { .buf = (uint8_t *)command, .len = command_length },
        { .buf = (uint8_t *)data, .len = data_length },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = (data_length > 0) ? 2 : 1,
    };

    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);
    gpio_pin_set_dt(&ctx->nss, 0);

    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
        return LR11XX_HAL_STATUS_ERROR;
    }

    /* SetSleep opcode 0x011B — chip won't drive BUSY; track state and wait for entry */
    if (command_length >= 2 && command[0] == 0x01 && command[1] == 0x1B) {
        ctx->radio_is_sleeping = true;
        k_busy_wait(1000);  /* ≥500µs for sleep entry per datasheet; 1ms for margin */
        return LR11XX_HAL_STATUS_OK;
    }

    return wait_on_busy(ctx);
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
                                     const uint16_t command_length,
                                     uint8_t *data, const uint16_t data_length)
{
    struct lr11xx_hal_context *ctx = (struct lr11xx_hal_context *)context;
    int ret;

    /* Track opcode for BUSY timeout diagnostics */
    if (command_length >= 2) {
        last_opcode = ((uint16_t)command[0] << 8) | command[1];
    }
    last_cmd_time = k_uptime_get();

    /* Crypto engine restore (opcode 0x050B) requires delay before issuing command */
    if (command_length >= 2 && command[0] == 0x05 && command[1] == 0x0B) {
        k_busy_wait(1000);  /* TODO: document required delay from datasheet */
    }

    if (check_device_ready(ctx) != LR11XX_HAL_STATUS_OK) {
        LOG_ERR("hal_read: device not ready, op=0x%04x", last_opcode);
        return LR11XX_HAL_STATUS_ERROR;
    }

    /* Phase 1: send command opcode */
    const struct spi_buf tx_buf = { .buf = (uint8_t *)command, .len = command_length };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);
    gpio_pin_set_dt(&ctx->nss, 0);

    if (ret < 0) {
        LOG_ERR("SPI write (cmd) failed: %d", ret);
        return LR11XX_HAL_STATUS_ERROR;
    }

    if (data_length == 0) {
        return wait_on_busy(ctx);
    }

    /* Phase 2: wait BUSY, then read response — LR11XX prepends one dummy status byte */
    if (check_device_ready(ctx) != LR11XX_HAL_STATUS_OK) {
        return LR11XX_HAL_STATUS_ERROR;
    }

    uint8_t dummy;
    const struct spi_buf rx_bufs[] = {
        { .buf = &dummy, .len = 1 },  /* discard leading status byte */
        { .buf = data, .len = data_length },
    };
    const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_read(ctx->spi_dev, &ctx->spi_cfg, &rx);
    gpio_pin_set_dt(&ctx->nss, 0);

    if (ret < 0) {
        LOG_ERR("SPI read failed: %d", ret);
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
                                            const uint16_t data_length)
{
    struct lr11xx_hal_context *ctx = (struct lr11xx_hal_context *)context;
    int ret;

    if (check_device_ready(ctx) != LR11XX_HAL_STATUS_OK) {
        return LR11XX_HAL_STATUS_ERROR;
    }

    const struct spi_buf rx_buf = { .buf = data, .len = data_length };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_read(ctx->spi_dev, &ctx->spi_cfg, &rx);
    gpio_pin_set_dt(&ctx->nss, 0);

    if (ret < 0) {
        LOG_ERR("SPI direct read failed: %d", ret);
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{
    struct lr11xx_hal_context *ctx = (struct lr11xx_hal_context *)context;

    LOG_INF("LR11xx reset");

    gpio_pin_set_dt(&ctx->reset, 1);
    k_msleep(10);   /* ≥100µs reset pulse required (datasheet §5.1); 10ms for margin */

    gpio_pin_set_dt(&ctx->reset, 0);
    k_msleep(300);  /* Firmware boot ≤273ms (datasheet); 300ms for margin */

    LOG_INF("LR11xx reset complete, BUSY=%d", gpio_pin_get_dt(&ctx->busy));

    ctx->radio_is_sleeping = false;
    return wait_on_busy(ctx);
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
    struct lr11xx_hal_context *ctx = (struct lr11xx_hal_context *)context;
    return check_device_ready(ctx);
}
