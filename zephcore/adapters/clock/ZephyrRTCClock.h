/*
 * SPDX-License-Identifier: MIT
 * Zephyr software RTC - boot-relative time with settable offset
 */

#pragma once

#include <mesh/MeshCore.h>

namespace mesh {

class ZephyrRTCClock : public RTCClock {
public:
	uint32_t getCurrentTime() override;
	/* Also written to the board's hardware RTC, if it has one, as upstream's
	 * AutoDiscoverRTCClock does: coalesced on the system work queue, so
	 * callers on any thread pay no I2C. */
	void setCurrentTime(uint32_t time) override;
	/* Boot only: the time just read from the hardware RTC, not written back. */
	void seedCurrentTime(uint32_t time);

private:
	uint32_t epoch_offset = 0;
};

} /* namespace mesh */
