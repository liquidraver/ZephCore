/*
 * SPDX-License-Identifier: MIT
 *
 * Boot-time hardware-RTC auto-discovery (compact raw-I2C reader).
 *
 * Probes every I2C RTC chip declared with the "zephcore,rtc-i2c" binding
 * (boards/common/rtc-i2c.dtsi, or a board's own overlay). Chips that aren't
 * physically present fail the probe and are skipped — like the environment
 * sensors.
 *
 * If a present chip holds a valid time, zephcore_rtc_restore() returns it so
 * the soft clock can be seeded at boot (shown tagged "L" — local). Every
 * ZephyrRTCClock::setCurrentTime() writes it back via zephcore_rtc_save(), so
 * time survives the next power-off.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Probe the declared RTC chips in order, stopping at the first that holds a
 * sane time (year >= 2025 and its power-loss flag clear): store its Unix
 * epoch in *epoch_out and return true. The first chip found (valid time or
 * not) is remembered as the write-back target. No time is taken while the
 * descriptor's twelve-hour-bit reads set or cannot be read cleanly. Returns
 * false if none is present or none holds a trustworthy time.
 *
 * A descriptor with rv3028-eeprom-config also has that config stored in the
 * chip's EEPROM, retried on the system work queue if it fails.
 */
bool zephcore_rtc_restore(uint32_t *epoch_out);

/*
 * Persist an authoritative epoch to the discovered RTC chip, selecting
 * 24-hour mode first, and clear its power-loss flag. No-op if no RTC was
 * discovered. Intended for real syncs (GPS/app/CLI), not per-packet clock
 * nudges. Call only from the system work queue: an unconfirmed write is
 * repeated there and shares state with this call.
 */
void zephcore_rtc_save(uint32_t epoch);

#ifdef __cplusplus
}
#endif
