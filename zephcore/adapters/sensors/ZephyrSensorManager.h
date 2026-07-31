/*
 * SPDX-License-Identifier: MIT
 * Backward-compatibility shim — includes GPS manager + env sensors.
 * All callers can continue using #include "ZephyrSensorManager.h".
 */

#pragma once

#include "ZephyrGPSManager.h"
#include "ZephyrEnvSensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Legacy combined init — calls all subsystem inits */
static inline int sensor_manager_init(void)
{
	gps_manager_init();
	env_sensors_init();
	power_sensors_init();
	return 0;
}

#ifdef __cplusplus
}
#endif
