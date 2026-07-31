/*
 * SPDX-License-Identifier: MIT
 * ZephCore RTC interface - matches MeshCore.h
 */

#pragma once

#include <stdint.h>

namespace mesh {

class RTCClock {
	uint32_t last_unique;

protected:
	RTCClock() : last_unique(0) {}

public:
	virtual uint32_t getCurrentTime() = 0;
	virtual void setCurrentTime(uint32_t time) = 0;
	virtual void tick() {}

	uint32_t getCurrentTimeUnique() {
		uint32_t t = getCurrentTime();
		if (t <= last_unique) {
			return ++last_unique;
		}
		return last_unique = t;
	}
};

} /* namespace mesh */
