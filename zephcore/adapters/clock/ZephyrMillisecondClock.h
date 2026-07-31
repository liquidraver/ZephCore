/*
 * SPDX-License-Identifier: MIT
 * Zephyr implementation of MillisecondClock
 */

#pragma once

#include <mesh/Clock.h>

namespace mesh {

class ZephyrMillisecondClock : public MillisecondClock {
public:
	unsigned long getMillis() override;
};

} /* namespace mesh */
