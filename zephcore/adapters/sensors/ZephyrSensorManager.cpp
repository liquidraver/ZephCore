/*
 * SPDX-License-Identifier: MIT
 * See ZephyrSensorManager.h.
 */

#include "ZephyrSensorManager.h"

#include <stdlib.h>
#include <string.h>

ZephyrSensorManager sensors;

bool ZephyrSensorManager::begin()
{
	gps_manager_init();
	env_sensors_init();
	return true;
}

bool ZephyrSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry)
{
	/* Upstream sends the position while the GPS is on; here only a real fix,
	 * never the (0,0) of a GPS that has not found one yet. */
	if ((requester_permissions & TELEM_PERM_LOCATION) && gps_is_enabled()) {
		struct gps_position pos;

		if (gps_get_last_known_position(&pos)) {
			telemetry.addGPS(TELEM_CHANNEL_SELF, pos.latitude_ndeg / 1e9,
					 pos.longitude_ndeg / 1e9, pos.altitude_mm / 1000.0f);
		}
	}

	if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
		uint8_t next_available_channel = TELEM_CHANNEL_SELF + 1;

		for (int i = 0; i < env_sensor_count(); i++) {
			struct env_sensor_reading r;
			bool board_local = env_sensor_is_board_local(i);
			uint8_t ch = board_local ? TELEM_CHANNEL_SELF : next_available_channel++;

			if (env_sensor_read(i, &r) != 0) {
				continue;  /* the channel stays taken: numbering stays stable */
			}
			if (r.fields & ENV_F_LUMINOSITY) {
				float lum = r.luminosity < 0.0f ? 0.0f : r.luminosity;

				telemetry.addLuminosity(ch, (uint32_t)(lum > 65535.0f ? 65535.0f : lum));
			}
			if (r.fields & ENV_F_TEMPERATURE) {
				telemetry.addTemperature(ch, r.temperature_c);
			}
			if (r.fields & ENV_F_HUMIDITY) {
				telemetry.addRelativeHumidity(ch, r.humidity_pct);
			}
			if (r.fields & ENV_F_PRESSURE) {
				telemetry.addBarometricPressure(ch, r.pressure_hpa);
			}
			if (r.fields & ENV_F_ALTITUDE) {
				telemetry.addAltitude(ch, r.altitude_m);
			}
			if (r.fields & ENV_F_VOLTAGE) {
				telemetry.addVoltage(ch, r.voltage_v);
			}
			if (r.fields & ENV_F_CURRENT) {
				telemetry.addCurrent(ch, r.current_a);
			}
			if (r.fields & ENV_F_POWER) {
				telemetry.addPower(ch, r.power_w);
			}
		}
	}
	return true;
}

/* As upstream: the one setting, "gps", listed only when a GPS is fitted. */
int ZephyrSensorManager::getNumSettings() const
{
	return gps_is_available() ? 1 : 0;
}

const char* ZephyrSensorManager::getSettingName(int i) const
{
	return (gps_is_available() && i == 0) ? "gps" : NULL;
}

const char* ZephyrSensorManager::getSettingValue(int i) const
{
	if (gps_is_available() && i == 0) {
		return gps_is_enabled() ? "1" : "0";
	}
	return NULL;
}

bool ZephyrSensorManager::setSettingValue(const char* name, const char* value)
{
	if (!gps_is_available()) {
		return false;
	}
	if (strcmp(name, "gps") == 0) {
		gps_enable(strcmp(value, "0") != 0);
		return true;
	}
	if (strcmp(name, "gps_interval") == 0) {
		gps_set_poll_interval_sec((uint32_t)atoi(value));
		return true;
	}
	return false;
}

bool ZephyrSensorManager::isGPSDetected() const
{
	return gps_is_available();
}
