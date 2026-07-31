/*
 * SPDX-License-Identifier: MIT
 * Zephyr Environment & Power Sensors
 *
 * Environment: temperature, humidity, pressure
 *   Supports: SHTC3, AHT20/DHT20/AM2301B, SHT4x, SHT3x, BME280, BME680, BMP280, BMP388, LPS22HB
 *   MCU die temperature as fallback (nordic,nrf-temp)
 *
 * Power monitors: voltage, current, power
 *   Supports: INA219, INA3221, INA226, INA228, INA230, INA232, INA236, INA237
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Environment Sensors ========== */

/* Environment sensor data */
struct env_data {
	float temperature_c;     /* Temperature in Celsius (external sensor) */
	float humidity_pct;      /* Relative humidity in percent */
	float pressure_hpa;      /* Barometric pressure in hPa */
	float mcu_temperature_c; /* MCU die temperature in Celsius */
	bool has_temperature;
	bool has_humidity;
	bool has_pressure;
	bool has_mcu_temperature;
};

/* Initialize environment sensors (call once at boot) */
int env_sensors_init(void);

/* Environment sensor functions */
bool env_sensors_available(void);
int env_sensors_read(struct env_data *data);

/* ========== Power Monitor Sensors ========== */

#define POWER_MAX_CHANNELS 4  /* INA3221 has 3 channels + 1 spare */

/* Single power monitor channel */
struct power_channel {
	float voltage_v;   /* Bus voltage in volts */
	float current_a;   /* Current in amperes */
	float power_w;     /* Power in watts */
	bool valid;        /* True if this channel was read successfully */
};

/* Power monitor data */
struct power_data {
	struct power_channel channels[POWER_MAX_CHANNELS];
	uint8_t num_channels;  /* Number of valid channels */
};

/* Initialize power monitor sensors (call once at boot) */
int power_sensors_init(void);

/* Power monitor functions */
bool power_sensors_available(void);
int power_sensors_read(struct power_data *data);

#ifdef __cplusplus
}
#endif
