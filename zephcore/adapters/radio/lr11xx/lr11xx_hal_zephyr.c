/*
 * SPDX-License-Identifier: Apache-2.0
 * LR11xx HAL implementation for Zephyr - ZephCore
 *
 * Based on Semtech SWDR001 lr11xx_driver and IRNAS/SWDR001-Zephyr port.
 */

#include "lr11xx_hal_zephyr.h"
#include "lr11xx_hal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr11xx_hal, CONFIG_LORA_LOG_LEVEL);

/* Timeout for busy wait in milliseconds.
 * LR1110 should respond within a few ms after most commands.
 * After reset, firmware boot can take up to 273ms (datasheet).
 * Use 3000ms to match RadioLib's timeout. */
#define LR11XX_BUSY_TIMEOUT_MS  3000

/* Static state for DIO1 interrupt handling */
static struct gpio_callback dio1_gpio_cb;
static lr11xx_dio1_callback_t dio1_user_cb = NULL;
static void *dio1_user_data = NULL;
static struct lr11xx_hal_context *current_ctx = NULL;

/* Work queue for deferred DIO1 processing
 * SPI operations CANNOT be done from ISR context on nRF52!
 * The GPIO interrupt triggers this work item which runs in thread context.
 */
static struct k_work dio1_work;

/* BUSY pin interrupt — wakes wait_on_busy() via semaphore instead of polling */
static struct gpio_callback busy_gpio_cb;
static K_SEM_DEFINE(busy_sem, 0, 1);

static void dio1_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (dio1_user_cb) {
        dio1_user_cb(dio1_user_data);
    }
}

static void busy_isr_callback(const struct device *dev, struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&busy_sem);
}

/* Track the last SPI opcode for debugging BUSY stuck */
static uint16_t last_opcode;
static int64_t last_cmd_time;

/**
 * @brief Wait until BUSY pin goes low or timeout.
 *
 * Uses GPIO interrupt + semaphore instead of polling.  The CPU sleeps
 * while waiting, saving power during long BUSY periods (reset, sleep
 * wake, firmware commands).  For sub-microsecond waits the fast-path
 * check returns immediately without touching the interrupt at all.
 */
static lr11xx_hal_status_t wait_on_busy(struct lr11xx_hal_context *ctx)
{
    /* Fast path: already ready */
    if (!gpio_pin_get_dt(&ctx->busy)) {
        return LR11XX_HAL_STATUS_OK;
    }

    k_sem_reset(&busy_sem);
    gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_EDGE_TO_INACTIVE);

    /* Re-check after enabling interrupt to close the race window where
     * BUSY dropped between our first check and the interrupt enable. */
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
 * @brief Check device ready, wake from sleep if needed
 */
static lr11xx_hal_status_t check_device_ready(struct lr11xx_hal_context *ctx)
{
    if (!ctx->radio_is_sleeping) {
        return wait_on_busy(ctx);
    }

    /* Radio is sleeping - wake it with NSS pulse
     * NSS is ACTIVE_LOW: logical 1 = physical LOW = asserted
     */
    gpio_pin_set_dt(&ctx->nss, 1);  /* Assert NSS (pull LOW) */
    k_busy_wait(10);
    gpio_pin_set_dt(&ctx->nss, 0);  /* Deassert NSS (release HIGH) */

    ctx->radio_is_sleeping = false;
    return wait_on_busy(ctx);
}

/**
 * @brief DIO1 GPIO interrupt callback (ISR context)
 *
 * CRITICAL: This runs in ISR context! Cannot do SPI operations here.
 * Instead, we submit work to the system work queue which runs in thread context.
 */
static void dio1_isr_callback(const struct device *dev, struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* Defer to work queue - SPI ops not allowed in ISR */
    k_work_submit(&dio1_work);
}

/* Public HAL API - called by Semtech driver */

int lr11xx_hal_init(struct lr11xx_hal_context *ctx)
{
    int ret;

    current_ctx = ctx;
    ctx->radio_is_sleeping = false;

    /* Configure NSS as output, inactive (deselected).
     * GPIO_OUTPUT_INACTIVE respects GPIO_ACTIVE_LOW from DTS:
     * inactive = logical 0 = physical HIGH = chip deselected. */
    ret = gpio_pin_configure_dt(&ctx->nss, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure NSS: %d", ret);
        return ret;
    }

    /* Configure RESET as output, inactive (not in reset).
     * GPIO_OUTPUT_INACTIVE with GPIO_ACTIVE_LOW:
     * inactive = logical 0 = physical HIGH = reset released. */
    ret = gpio_pin_configure_dt(&ctx->reset, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure RESET: %d", ret);
        return ret;
    }

    /* Configure BUSY as input with interrupt support */
    ret = gpio_pin_configure_dt(&ctx->busy, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure BUSY: %d", ret);
        return ret;
    }

    /* Set up BUSY interrupt callback (interrupt enabled on-demand by wait_on_busy) */
    gpio_init_callback(&busy_gpio_cb, busy_isr_callback, BIT(ctx->busy.pin));
    ret = gpio_add_callback(ctx->busy.port, &busy_gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add BUSY callback: %d", ret);
        return ret;
    }

    /* Configure DIO1 as input with interrupt */
    ret = gpio_pin_configure_dt(&ctx->dio1, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure DIO1: %d", ret);
        return ret;
    }

    /* Initialize DIO1 work queue handler */
    k_work_init(&dio1_work, dio1_work_handler);

    /* Set up DIO1 interrupt callback */
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

/* Semtech HAL interface implementation */

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

    /* Build SPI transaction */
    const struct spi_buf tx_bufs[] = {
        { .buf = (uint8_t *)command, .len = command_length },
        { .buf = (uint8_t *)data, .len = data_length },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = (data_length > 0) ? 2 : 1,
    };

    /* Assert NSS (active LOW in DTS, so logical 1 = physical LOW = chip selected) */
    gpio_pin_set_dt(&ctx->nss, 1);

    ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);

    /* Deassert NSS (logical 0 = physical HIGH = chip deselected) */
    gpio_pin_set_dt(&ctx->nss, 0);

    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
        return LR11XX_HAL_STATUS_ERROR;
    }

    /* Check for sleep command: opcode 0x011B */
    if (command_length >= 2 && command[0] == 0x01 && command[1] == 0x1B) {
        ctx->radio_is_sleeping = true;
        k_busy_wait(1000);  /* 1ms for sleep transition */
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

    /* Special case: crypto restore command needs delay */
    if (command_length >= 2 && command[0] == 0x05 && command[1] == 0x0B) {
        k_busy_wait(1000);
    }

    if (check_device_ready(ctx) != LR11XX_HAL_STATUS_OK) {
        LOG_ERR("hal_read: device not ready, op=0x%04x", last_opcode);
        return LR11XX_HAL_STATUS_ERROR;
    }

    /* Step 1: Write command */
    const struct spi_buf tx_buf = { .buf = (uint8_t *)command, .len = command_length };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    /* Assert NSS (active LOW in DTS, so logical 1 = physical LOW = chip selected) */
    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);
    /* Deassert NSS */
    gpio_pin_set_dt(&ctx->nss, 0);

    if (ret < 0) {
        LOG_ERR("SPI write (cmd) failed: %d", ret);
        return LR11XX_HAL_STATUS_ERROR;
    }

    if (data_length == 0) {
        return wait_on_busy(ctx);
    }

    /* Step 2: Wait for device ready, then read response */
    if (check_device_ready(ctx) != LR11XX_HAL_STATUS_OK) {
        return LR11XX_HAL_STATUS_ERROR;
    }

    /* LR11xx returns 1 dummy byte + data */
    uint8_t dummy;
    const struct spi_buf rx_bufs[] = {
        { .buf = &dummy, .len = 1 },
        { .buf = data, .len = data_length },
    };
    const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

    /* Assert NSS */
    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_read(ctx->spi_dev, &ctx->spi_cfg, &rx);
    /* Deassert NSS */
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

    /* Assert NSS (active LOW in DTS, so logical 1 = physical LOW = chip selected) */
    gpio_pin_set_dt(&ctx->nss, 1);
    ret = spi_read(ctx->spi_dev, &ctx->spi_cfg, &rx);
    /* Deassert NSS */
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

    LOG_INF("LR11xx reset: assert reset, hold 10ms");

    /* Reset pin is ACTIVE_LOW in DTS:
     * gpio_pin_set_dt(..., 1) = logical assert = physical LOW = reset active
     * gpio_pin_set_dt(..., 0) = logical deassert = physical HIGH = reset released
     *
     * RadioLib sequence (LR_common.cpp):
     * 1. digitalWrite(rst, LOW)  - assert reset
     * 2. delay(10)               - 10ms hold
     * 3. digitalWrite(rst, HIGH) - release reset
     * 4. delay(300)              - wait for firmware (datasheet: 273ms typical)
     * 5. wait for BUSY low
     */
    gpio_pin_set_dt(&ctx->reset, 1);  /* Assert reset (pull LOW) */
    k_msleep(10);  /* 10ms hold in reset (matches RadioLib) */

    gpio_pin_set_dt(&ctx->reset, 0);  /* Deassert reset (release to HIGH) */

    /* Wait 300ms for internal LR11xx firmware (RadioLib uses 300ms, datasheet 273ms) */
    k_msleep(300);

    LOG_INF("LR11xx reset complete, BUSY=%d", gpio_pin_get_dt(&ctx->busy));

    ctx->radio_is_sleeping = false;

    /* Wait for BUSY to go low - chip is ready */
    return wait_on_busy(ctx);
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
    struct lr11xx_hal_context *ctx = (struct lr11xx_hal_context *)context;
    return check_device_ready(ctx);
}
