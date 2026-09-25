/*
 * SPDX-License-Identifier: MIT
 *
 * Upstream's SensorManager over Zephyr's sensor drivers (ZephyrEnvSensors) and
 * the GPS manager (ZephyrGPSManager). Telemetry as upstream's
 * EnvironmentSensorManager: the GPS on the node's own channel, every sensor
 * found at boot on its own channel from 2 up, in probe order; board-local
 * analog sensors (T1000-E) on the node's channel, as upstream's T1000 variant.
 * Main thread only.
 */

#pragma once

#include <helpers/SensorManager.h>
#include "ZephyrGPSManager.h"
#include "ZephyrEnvSensors.h"

class ZephyrSensorManager : public SensorManager {
public:
	bool begin() override;
	bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
	int getNumSettings() const override;
	const char* getSettingName(int i) const override;
	const char* getSettingValue(int i) const override;
	/* "gps" 0|1: on or off; "gps_interval" <sec>: the GPS duty interval
	 * (upstream: its position refresh interval). Not persisted, as upstream;
	 * `gps on|off` and the app's custom vars persist. */
	bool setSettingValue(const char* name, const char* value) override;
	bool isGPSDetected() const override;
};

/* The one instance, as upstream's per-variant `sensors` (target.cpp). */
extern ZephyrSensorManager sensors;
