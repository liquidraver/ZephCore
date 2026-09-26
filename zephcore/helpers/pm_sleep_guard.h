/*
 * SPDX-License-Identifier: MIT
 * Sleep guard — block SoC light sleep across a critical stretch.
 *
 * Zephyr's policy locks are reference-counted, so every zc_pm_block_sleep()
 * needs exactly one matching zc_pm_unblock_sleep(); nested holders compose.
 * While at least one lock is held the idle path skips PM_STATE_STANDBY and the
 * SoC idles normally instead — device power management is untouched either way.
 *
 * Both calls compile to nothing without CONFIG_PM, so callers do not need to
 * guard their own use sites.
 *
 * The second half is the ESP32 light-sleep policy (helpers/pm_esp32_wake.c and
 * helpers/pm_esp32_console.c, built only with CONFIG_PM on an ESP32): the
 * `powersaving` switch, the console window and the `get pm` counters. Every
 * other build gets the stubs, so callers need no #if either.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(CONFIG_PM)

#include <zephyr/pm/policy.h>

static inline void zc_pm_block_sleep(void)
{
	pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
}

static inline void zc_pm_unblock_sleep(void)
{
	pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
}

#else /* !CONFIG_PM */

static inline void zc_pm_block_sleep(void) { }
static inline void zc_pm_unblock_sleep(void) { }

#endif /* CONFIG_PM */

#if defined(CONFIG_PM) && defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)

#ifdef __cplusplus
extern "C" {
#endif

/* The `powersaving` pref: off holds a sleep lock for as long as it stays off. */
void zc_pm_set_powersaving(bool on);

/* (Re)open the console window: no light sleep for ZEPHCORE_PM_BOOT_AWAKE_MS
 * from now. Taken at boot and on every user-button press; ISR-safe. */
void zc_pm_console_window_open(void);
uint32_t zc_pm_console_window_remaining_ms(void);

/* One `get pm` line into buf; returns its length. */
int zc_pm_format_stats(char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#define ZC_PM_LIGHT_SLEEP 1

#else

static inline void zc_pm_set_powersaving(bool on) { (void)on; }
static inline void zc_pm_console_window_open(void) { }
static inline uint32_t zc_pm_console_window_remaining_ms(void) { return 0; }
static inline int zc_pm_format_stats(char *buf, size_t len) { (void)buf; (void)len; return 0; }

#define ZC_PM_LIGHT_SLEEP 0

#endif /* CONFIG_PM && CONFIG_SOC_FAMILY_ESPRESSIF_ESP32 */
