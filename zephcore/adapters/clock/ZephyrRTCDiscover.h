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
 * not) is remembered as the write-back target. A failed first read means
 * nothing is there, and a first read of all 0xFF (an erased EEPROM) is
 * skipped. Otherwise a device is passed over only if two reads each show it
 * is not that RTC: a bit set that the data sheet shows as 0 (the
 * descriptor's zero-mask), seconds, minutes, date or month out of range
 * while the power-loss flag does not read as set (an unreadable flag counts
 * as not set), or, on the second read, all 0xFF. A failed second read never
 * rules a device out. A time is taken only from a clean read whose fields,
 * hours and year are each in range (12-hour mode is not decoded). Returns
 * false if none present or no trustworthy time is held.
 */
bool zephcore_rtc_restore(uint32_t *epoch_out);

/*
 * Persist an authoritative epoch to the discovered RTC chip and clear its
 * power-loss flag. No-op if no RTC was discovered. Safe to call often, but
 * intended only for real syncs (GPS/app/CLI), not per-packet clock nudges.
 * If restore adopted nothing but skipped a candidate whose first read was all
 * 0xFF, the first save probes again, once per boot, in the caller's context.
 * A device ruled out on its second read is not probed again for that reason.
 */
void zephcore_rtc_save(uint32_t epoch);

#ifdef __cplusplus
}
#endif
