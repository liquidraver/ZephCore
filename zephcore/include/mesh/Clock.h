/*
 * SPDX-License-Identifier: MIT
 * ZephCore clock interfaces
 */

#pragma once

#include <stdint.h>

namespace mesh {

class MillisecondClock {
public:
	virtual unsigned long getMillis() = 0;
};

} /* namespace mesh */
