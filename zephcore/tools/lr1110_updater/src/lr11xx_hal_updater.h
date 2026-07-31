/*
 * SPDX-License-Identifier: MIT
 * Minimal LR11xx HAL for firmware updater — no IRQ, no DIO1, just SPI+GPIO.
 *
 * Provides the lr11xx_hal_write/read/direct_read/reset interface that the
 * Semtech bootloader driver (lr1110_bootloader.c) needs.
 */

#ifndef LR11XX_HAL_UPDATER_H
#define LR11XX_HAL_UPDATER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/**
 * @brief Initialize SPI and GPIOs for LR1110 communication.
 *
 * Reads pin configuration from the devicetree "semtech,lr1110" node on SPI1.
 *
 * @return 0 on success, negative errno on failure
 */
int lr1110_updater_hal_init(void);

/* TCXO configuration from devicetree (0 mV = crystal board, no TCXO) */
uint16_t lr1110_updater_tcxo_voltage_mv(void);
uint32_t lr1110_updater_tcxo_startup_delay_ms(void);

/**
 * @brief Hardware reset the LR1110 (pulse RESET, wait for BUSY low).
 *
 * After reset the LR1110 boots into firmware (if flash valid) or bootloader.
 *
 * @return 0 on success, negative errno on failure
 */
int lr1110_updater_hw_reset(void);

/**
 * @brief Force LR1110 into bootloader mode via hardware reset.
 *
 * Holds BUSY LOW as output during RESET pulse — this forces the LR1110
 * into bootloader mode regardless of flash content. This is the official
 * Semtech approach (lr1110_updater_tool).
 *
 * @return 0 on success, negative errno on failure
 */
int lr1110_updater_reset_to_bootloader(void);

/**
 * @brief Get the opaque HAL context pointer for Semtech driver calls.
 *
 * This pointer is passed as 'context' to lr1110_bootloader_*() functions.
 */
void *lr1110_updater_get_context(void);

/**
 * @brief Instrumentation for the most recent command.
 *
 * WriteFlashEncrypted has no read-back and no per-command acknowledgement, so
 * BUSY timing is the only direct evidence that the chip actually performed the
 * flash program cycle. These expose it for per-chunk tracing:
 *
 *  - rise_us: how long after NSS deassert the chip asserted BUSY.
 *  - hold_us: how long BUSY stayed high — the real program time (~3.8 ms for a
 *             256-byte page). A near-zero hold means nothing was programmed.
 *  - busy_seen: false if BUSY never rose within the bounded watch window.
 *  - spi_ret: return code of the last spi_write().
 */
/**
 * @brief Probe the shared SPI bus for an inserted SD card.
 *
 * Boards where the SD slot shares the radio's bus cannot flash the LR1110
 * reliably with a card inserted. No card-detect pin is available, so presence
 * is established with an SPI-mode CMD0 exchange.
 *
 * @return 1 if a card responded, 0 if the slot is empty, -ENOTSUP on boards
 *         with no shared SD slot, or a negative errno on bus failure.
 */
int lr1110_updater_probe_sdcard(void);

uint32_t lr1110_updater_last_busy_rise_us(void);
uint32_t lr1110_updater_last_busy_hold_us(void);
bool     lr1110_updater_last_busy_seen(void);
int      lr1110_updater_last_spi_ret(void);

#endif /* LR11XX_HAL_UPDATER_H */
