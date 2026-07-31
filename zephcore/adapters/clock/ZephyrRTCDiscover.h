/*
 * SPDX-License-Identifier: MIT
 *
 * Boot-time hardware-RTC auto-discovery (compact raw-I2C reader).
 *
 * Probes every I2C RTC chip declared with the "zephcore,rtc-i2c" binding
 * (boards/common/rtc-i2c.dtsi, opt-in per board). Chips that aren't physically
 * present fail the probe and are skipped — like the environment sensors.
 *
 * If a present chip holds a valid time, zephcore_rtc_restore() returns it so
 * the soft clock can be seeded at boot (shown tagged "L" — local). On every
 * authoritative sync (GPS/app/CLI) the caller writes it back via
 * zephcore_rtc_save() so time survives the next power-off.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Probe all declared RTC chips. If one is present and holds a sane time
 * (year >= 2025 and its power-loss flag is clear), store the Unix epoch in
 * *epoch_out and return true. The present chip (valid time or not) is
 * remembered as the write-back target. Returns false if none present or no
 * trustworthy time is held.
 */
bool zephcore_rtc_restore(uint32_t *epoch_out);

/*
 * Persist an authoritative epoch to the discovered RTC chip and clear its
 * power-loss flag. No-op if no RTC was discovered. Safe to call often, but
 * intended only for real syncs (GPS/app/CLI), not per-packet clock nudges.
 */
void zephcore_rtc_save(uint32_t epoch);

#ifdef __cplusplus
}
#endif
