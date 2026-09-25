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

uint32_t ZephyrRTCClock::getCurrentTime()
{
	uint32_t uptime_sec = (uint32_t)(k_uptime_get() / 1000);
	return epoch_offset + uptime_sec;
}

void ZephyrRTCClock::seedCurrentTime(uint32_t time)
{
	uint32_t uptime_sec = (uint32_t)(k_uptime_get() / 1000);
	epoch_offset = time - uptime_sec;
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
