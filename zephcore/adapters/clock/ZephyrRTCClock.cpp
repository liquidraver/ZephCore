/*
 * SPDX-License-Identifier: MIT
 */

#include "ZephyrRTCClock.h"
#include "ZephyrRTCDiscover.h"
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

namespace mesh {

#if DT_HAS_COMPAT_STATUS_OKAY(zephcore_rtc_i2c)
/* One clock per build. The work reads the time when it runs, so any number
 * of sets before it runs are one write, of the latest time. */
static ZephyrRTCClock *s_saved_clock;

static void rtc_save_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	zephcore_rtc_save(s_saved_clock->getCurrentTime());
}

static K_WORK_DEFINE(s_rtc_save_work, rtc_save_work_fn);
#endif

/* The last time read, kept across a reset that keeps RAM (reboot, crash,
 * watchdog), as upstream's ESP32RTCClock does. ESP32 keeps it in RTC slow
 * memory: the bootloader may reuse ordinary noinit RAM. Power-on leaves
 * garbage, which the magic rejects. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#define RTC_BACKUP_ATTR Z_GENERIC_SECTION(.rtc_noinit)
#else
#define RTC_BACKUP_ATTR __noinit
#endif
static RTC_BACKUP_ATTR uint32_t _rtc_backup_time;
static RTC_BACKUP_ATTR uint32_t _rtc_backup_magic;
#define RTC_BACKUP_MAGIC  0xAA55CC33
#define RTC_TIME_MIN      1772323200  // 1 Mar 2026

ZephyrRTCClock::ZephyrRTCClock()
{
	if (_rtc_backup_magic == RTC_BACKUP_MAGIC && _rtc_backup_time > RTC_TIME_MIN) {
		epoch_offset = _rtc_backup_time;
	}
}

uint32_t ZephyrRTCClock::getCurrentTime()
{
	uint32_t uptime_sec = (uint32_t)(k_uptime_get() / 1000);
	uint32_t now = epoch_offset + uptime_sec;

	/* upstream does this in tick(), which nothing calls here */
	if (now > RTC_TIME_MIN && now != _rtc_backup_time) {
		_rtc_backup_time = now;
		_rtc_backup_magic = RTC_BACKUP_MAGIC;
	}
	return now;
}

void ZephyrRTCClock::seedCurrentTime(uint32_t time)
{
	uint32_t uptime_sec = (uint32_t)(k_uptime_get() / 1000);
	epoch_offset = time - uptime_sec;
	/* As upstream's setCurrentTime(): unconditional, so a set below
	 * RTC_TIME_MIN (clkreboot) replaces the backup and is not restored. */
	_rtc_backup_time = time;
	_rtc_backup_magic = RTC_BACKUP_MAGIC;
}

void ZephyrRTCClock::setCurrentTime(uint32_t time)
{
	seedCurrentTime(time);
#if DT_HAS_COMPAT_STATUS_OKAY(zephcore_rtc_i2c)
	s_saved_clock = this;
	k_work_submit(&s_rtc_save_work);
#endif
}

} /* namespace mesh */
