/*
 * SPDX-License-Identifier: MIT
 * LR11xx HAL for Zephyr — ZephCore
 */

#ifndef LR11XX_HAL_ZEPHYR_H
#define LR11XX_HAL_ZEPHYR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "lr11xx_hal.h"

/**
 * @brief HAL context passed as 'context' to all lr11xx_hal_* functions.
 *
 * NSS is software-controlled (not via SPI CS) to allow the two-phase read
 * protocol (write command / deassert / reassert / read response).
 *
 * The LR1110 is accessed from two threads (main mesh loop and DIO1 work queue);
 * callers must serialize access externally. Concurrent SPI access corrupts the
 * command/response protocol and stalls BUSY HIGH permanently.
 */
struct lr11xx_hal_context {
    const struct device *spi_dev;
    struct spi_config spi_cfg;

    struct gpio_dt_spec nss;    /* chip select, software-controlled, ACTIVE_LOW in DTS */
    struct gpio_dt_spec reset;  /* hardware reset, ACTIVE_LOW in DTS */
    struct gpio_dt_spec busy;   /* busy indicator, high = chip processing */
    struct gpio_dt_spec dio1;   /* IRQ output from chip */

    struct gpio_dt_spec dio2;   /* optional: RF switch or second IRQ */
    struct gpio_dt_spec rxen;   /* optional: external LNA enable */
    struct gpio_dt_spec txen;   /* optional: external PA enable */

    uint16_t tcxo_voltage_mv;   /* 0 = XTAL, non-zero = TCXO supply in mV */
    uint32_t tcxo_startup_us;   /* TCXO startup time; passed to SetTcxoMode timeout */

    volatile bool radio_is_sleeping;
};

/**
 * @brief Configure GPIOs and register interrupt callbacks. Call before any HAL function.
 *
 * @param ctx HAL context with gpio specs filled in
 * @return 0 on success, negative errno on failure
 */
int lr11xx_hal_init(struct lr11xx_hal_context *ctx);

/** @brief DIO1 interrupt callback type; invoked directly from GPIO ISR — must be ISR-safe */
typedef void (*lr11xx_dio1_callback_t)(void *user_data);

/**
 * @brief Register DIO1 rising-edge callback.
 * @param cb Must be ISR-safe (e.g., submit to a k_work_q, not block)
 */
void lr11xx_hal_set_dio1_callback(struct lr11xx_hal_context *ctx,
                                   lr11xx_dio1_callback_t cb, void *user_data);

void lr11xx_hal_enable_dio1_irq(struct lr11xx_hal_context *ctx);
void lr11xx_hal_disable_dio1_irq(struct lr11xx_hal_context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LR11XX_HAL_ZEPHYR_H */
