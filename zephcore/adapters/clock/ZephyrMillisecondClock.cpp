/*
 * SPDX-License-Identifier: MIT
 */

#include "ZephyrMillisecondClock.h"
#include <zephyr/kernel.h>

namespace mesh {

unsigned long ZephyrMillisecondClock::getMillis()
{
	return (unsigned long)k_uptime_get_32();
}

} /* namespace mesh */
