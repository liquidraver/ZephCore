/*
 * SPDX-License-Identifier: MIT
 * Zephyr software RTC - boot-relative time with settable offset
 */

#pragma once

#include <mesh/RTC.h>

namespace mesh {

class ZephyrRTCClock : public RTCClock {
public:
	uint32_t getCurrentTime() override;
	void setCurrentTime(uint32_t time) override;

private:
	uint32_t epoch_offset = 0;
};

} /* namespace mesh */
