/*
 * SPDX-License-Identifier: MIT
 */

#include "ZephyrRTCClock.h"
#include <zephyr/kernel.h>

namespace mesh {

uint32_t ZephyrRTCClock::getCurrentTime()
{
	uint32_t uptime_sec = (uint32_t)(k_uptime_get() / 1000);
	return epoch_offset + uptime_sec;
}

void ZephyrRTCClock::setCurrentTime(uint32_t time)
{
	uint32_t uptime_sec = (uint32_t)(k_uptime_get() / 1000);
	epoch_offset = time - uptime_sec;
}

} /* namespace mesh */
