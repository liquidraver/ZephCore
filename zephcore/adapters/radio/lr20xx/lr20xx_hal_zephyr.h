/*
 * SPDX-License-Identifier: MIT
 * LR20xx HAL implementation for Zephyr - ZephCore
 *
 * Zephyr hardware context for the Semtech lr20xx_driver HAL interface.
 */

#ifndef LR20XX_HAL_ZEPHYR_H
#define LR20XX_HAL_ZEPHYR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "lr20xx_hal.h"

/**
 * @brief LR20xx HAL context — passed as 'context' to all lr20xx_hal_* functions.
 *
 * WARNING: All SPI operations must be serialized. The LR2021 is accessed from
 * two threads (main event loop + DIO1 work queue). Concurrent SPI access will
 * corrupt the command/response protocol and permanently stick BUSY HIGH.
 */
struct lr20xx_hal_context {
	const struct device *spi_dev;
	struct spi_config spi_cfg;

	struct gpio_dt_spec nss;    /* Chip select (direct GPIO, not SPI controller CS) */
	struct gpio_dt_spec reset;  /* Reset (active-low) */
	struct gpio_dt_spec busy;   /* BUSY: high = chip processing command */
	struct gpio_dt_spec dio1;   /* DIO1 interrupt */

	volatile bool radio_is_sleeping;
};

/**
 * @brief Initialize HAL context GPIOs. Must be called before any other HAL function.
 *
 * @param ctx HAL context with gpio specs filled in
 * @return 0 on success, negative errno on failure
 */
int lr20xx_hal_init(struct lr20xx_hal_context *ctx);

/**
 * @brief GPIO callback type for DIO1 interrupt
 */
typedef void (*lr20xx_dio1_callback_t)(void *user_data);

/**
 * @brief Set DIO1 interrupt callback (invoked directly from GPIO ISR — must be ISR-safe).
 */
void lr20xx_hal_set_dio1_callback(struct lr20xx_hal_context *ctx,
				   lr20xx_dio1_callback_t cb, void *user_data);

/** @brief Enable DIO1 edge interrupt */
void lr20xx_hal_enable_dio1_irq(struct lr20xx_hal_context *ctx);

/** @brief Disable DIO1 interrupt */
void lr20xx_hal_disable_dio1_irq(struct lr20xx_hal_context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LR20XX_HAL_ZEPHYR_H */
