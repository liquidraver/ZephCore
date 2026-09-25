/*
 * SPDX-License-Identifier: MIT
 * Zephyr Environment & Power Sensors
 *
 * Environment: temperature, humidity, pressure, light
 *   Supports: SHTC3, AHT20/DHT20/AM2301B, SHT4x, SHT3x, BME280, BME680, BMP280, BMP388, LPS22HB, SPA06
 *   Board-local analog sensors: T1000-E NTC thermistor + photocell
 *
 * Power monitors: voltage, current, power
 *   Supports: INA219, INA3221 (each of its 3 channels), INA226, INA228, INA230, INA232, INA236, INA237
 *
 * Every part found at boot is one entry, in probe order; telemetry gives each
 * its own LPP channel (adapters/sensors/ZephyrSensorManager.cpp).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Probe every declared sensor (call once at boot). */
int env_sensors_init(void);

/* ========== Per-device ========== */

/* Fields present in a reading */
#define ENV_F_TEMPERATURE  (1u << 0)
#define ENV_F_HUMIDITY     (1u << 1)
#define ENV_F_PRESSURE     (1u << 2)
#define ENV_F_ALTITUDE     (1u << 3)   /* from pressure, standard atmosphere */
#define ENV_F_LUMINOSITY   (1u << 4)
#define ENV_F_VOLTAGE      (1u << 5)
#define ENV_F_CURRENT      (1u << 6)
#define ENV_F_POWER        (1u << 7)

struct env_sensor_reading {
	uint16_t fields;         /* ENV_F_* */
	float temperature_c;
	float humidity_pct;
	float pressure_hpa;
	float altitude_m;
	float luminosity;        /* see the note below */
	float voltage_v;
	float current_a;         /* negative = reverse flow (bidirectional monitors) */
	float power_w;
};

/* Note on luminosity: the unit is whatever the board's light sensor reports on
 * SENSOR_CHAN_LIGHT, and it is forwarded to CayenneLPP luminosity unscaled.
 * A true ambient-light part gives lux; the T1000-E's photocell gives Seeed's
 * 0-100 scale, which Arduino MeshCore also reports verbatim. Keeping it
 * unscaled is what makes a ZephCore node read the same as the stock firmware
 * it replaced. */

/* Number of sensor entries found by env_sensors_init(). */
int env_sensor_count(void);

/* True for a board-local sensor (the T1000-E's analog pair), which reports on
 * the node's own channel, as upstream's T1000 variant does. */
bool env_sensor_is_board_local(int index);

/* Read one entry. 0 on success (r->fields says what it had). */
int env_sensor_read(int index, struct env_sensor_reading *r);

/* ========== Merged view (UI) ========== */

/* The first temperature, humidity, pressure and light any sensor gives. */
struct env_data {
	float temperature_c;
	float humidity_pct;
	float pressure_hpa;
	float luminosity;
	bool has_temperature;
	bool has_humidity;
	bool has_pressure;
	bool has_luminosity;
};

bool env_sensors_available(void);
int env_sensors_read(struct env_data *data);

#ifdef __cplusplus
}
#endif
