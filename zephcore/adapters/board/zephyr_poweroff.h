/*
 * SPDX-License-Identifier: MIT
 *
 * Powering the node off, and why the last run did. One path for every
 * trigger: `poweroff` / `shutdown` (MainBoard::powerOff()), the UI's power-off,
 * low-battery auto-shutdown.
 */

#pragma once

#include <stdint.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Shutdown reasons: upstream's codes (NRF52Board.h). */
#define ZC_SHUTDOWN_NONE         0x00
#define ZC_SHUTDOWN_LOW_VOLTAGE  0x4C  /* 'L' */
#define ZC_SHUTDOWN_USER         0x55  /* 'U' */

/* Record why the node is powering off, for the next boot (a /lfs marker; best
 * effort, it may run at a critically low battery). */
void zephcore_shutdown_reason_save(uint8_t reason);

/* Why the previous run powered off, ZC_SHUTDOWN_NONE if it did not say. The
 * first call reads and clears the marker; later calls return the same value.
 * Call from main() after /lfs is mounted. */
uint8_t zephcore_shutdown_reason(void);

const char *zephcore_shutdown_reason_str(uint8_t reason);

/* Every load down, LoRa held in reset, the wake button armed, the power latch
 * released last, then System OFF. Does not return. */
FUNC_NORETURN void zephcore_power_off(void);

/* The UI's part (heartbeat LED, display), run first. Weak no-op here; the UI
 * builds provide it. */
void ui_before_power_off(void);

#ifdef __cplusplus
}
#endif
