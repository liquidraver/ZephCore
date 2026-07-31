/*
 * SPDX-License-Identifier: MIT
 * ZephCore - OLED display power management
 *
 * Sends I2C display-off command to SH1106/SSD1306 OLED to save power.
 * No-op if I2C or OLED hardware is not present.
 */

#ifndef ZEPHCORE_OLED_POWER_H
#define ZEPHCORE_OLED_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Put OLED display to sleep (Display OFF command via I2C).
 * Safe to call on any board — no-op when I2C or OLED is absent.
 */
void oled_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_OLED_POWER_H */
