/*
 * SPDX-License-Identifier: MIT
 * Zephyr Environment & Power Sensors
 *
 * Auto-detects available sensors via Zephyr devicetree nodelabels. See
 * ZephyrEnvSensors.h for the list.
 */

#include "ZephyrEnvSensors.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(zephcore_sensors, CONFIG_ZEPHCORE_SENSORS_LOG_LEVEL);

/* ========== Sensor Support ========== */
#if IS_ENABLED(CONFIG_SENSOR)
#define HAS_ENV_SENSORS 1
#include <zephyr/drivers/sensor.h>
#else
#define HAS_ENV_SENSORS 0
#endif

#if HAS_ENV_SENSORS

/* INA3221 channel selection attribute — defined in driver's private header
 * (zephyr/drivers/sensor/ti/ina3221/ina3221.h), replicated here to avoid
 * including private driver internals */
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START + 1)

/* What a part measures, which decides the channels read. */
enum sensor_kind {
	KIND_TEMP_HUMIDITY,  /* SHTC3, AHT20, SHT4x, SHT3xD */
	KIND_TEMP_HUM_PRESS, /* BME280, BME680 (+ altitude) */
	KIND_PRESSURE,       /* LPS22HB, BMP388, SPA06: pressure + die temperature */
	KIND_PRESSURE_ALT,   /* BMP280: pressure + temperature + altitude, as upstream */
	KIND_BOARD_ANALOG,   /* T1000-E NTC + photocell */
	KIND_POWER,          /* INA219, ina2xx */
	KIND_POWER_3221,     /* one INA3221 channel */
};

struct sensor_entry {
	const struct device *dev;
	uint8_t kind;
	uint8_t sub;         /* INA3221 channel, 1-3 */
};

/* Worst case: every part present, INA3221 as three. */
#define MAX_SENSORS 16
static struct sensor_entry s_sensors[MAX_SENSORS];
static uint8_t s_count;

/* Is this sensor usable — bringing it up first if its node deferred init?
 *
 * A part behind a switched rail cannot be probed at POST_KERNEL. Regulators
 * come up at priority 75 and sensors at 90, typically microseconds later, and a
 * regulator-boot-on rail never applies its startup-delay-us (regulator_common_init
 * takes the refcount-only branch, so regulator_delay() never runs). Such a node
 * is marked zephyr,deferred-init and initialised from here instead, where the
 * rail has had the whole boot to settle. See the i2c0 comment in the
 * MeshTracker X1 DTS for the failure this prevents.
 *
 * Safe to call for every candidate on every board: do_device_init() marks a
 * device initialized even when its init function failed, so device_init()
 * answers -EALREADY for anything that already ran at POST_KERNEL and this
 * reduces to a plain device_is_ready() check. */
static bool sensor_ready(const struct device *dev)
{
	if (dev == NULL) {
		return false;
	}
	if (!device_is_ready(dev)) {
		(void)device_init(dev);
	}
	return device_is_ready(dev);
}

static void add(const struct device *dev, enum sensor_kind kind, uint8_t sub, const char *part)
{
	if (s_count >= MAX_SENSORS) {
		return;
	}
	s_sensors[s_count++] = (struct sensor_entry){ dev, (uint8_t)kind, sub };
	LOG_INF("Found sensor: %s (%s)", dev->name, part);
}

/* A part that may sit at either of two nodes (two addresses, or several
 * compatibles of one chip family): the first that answers. */
static void probe(const struct device *dev, enum sensor_kind kind, const char *part)
{
	if (sensor_ready(dev)) {
		add(dev, kind, 0, part);
	}
}

#define NODE_DEV(label) DEVICE_DT_GET_OR_NULL(DT_NODELABEL(label))

static const struct device *first_ready(const struct device *a, const struct device *b,
					const struct device *c)
{
	if (sensor_ready(a)) {
		return a;
	}
	if (sensor_ready(b)) {
		return b;
	}
	return sensor_ready(c) ? c : NULL;
}

/* Standard atmosphere, as upstream's BME280/BMP280/BME680 queries. */
static float pressure_altitude_m(float hpa)
{
	return 44330.0f * (1.0f - powf(hpa / 1013.25f, 0.1903f));
}

static bool channel_float(const struct device *dev, enum sensor_channel chan, float *out)
{
	struct sensor_value val;

	if (sensor_channel_get(dev, chan, &val) != 0) {
		return false;
	}
	*out = sensor_value_to_float(&val);
	return true;
}

#endif /* HAS_ENV_SENSORS */

int env_sensors_init(void)
{
#if HAS_ENV_SENSORS
	/* Probe order is channel order: temperature/humidity parts, the Bosch
	 * combos, barometers, board-local analog, power monitors. */
	probe(NODE_DEV(shtc3), KIND_TEMP_HUMIDITY, "SHTC3");
	probe(first_ready(NODE_DEV(aht20), NODE_DEV(dht20), NODE_DEV(am2301b)),
	      KIND_TEMP_HUMIDITY, "AHT20/DHT20");
	probe(NODE_DEV(sht4x), KIND_TEMP_HUMIDITY, "SHT4x");
	probe(NODE_DEV(sht3xd), KIND_TEMP_HUMIDITY, "SHT3xD");
	/* bosch,bme280 also drives the BMP280 (no humidity channel). */
	probe(NODE_DEV(bme280), KIND_TEMP_HUM_PRESS, "BME280/BMP280");
	probe(NODE_DEV(bme280_alt), KIND_TEMP_HUM_PRESS, "BME280/BMP280 at 0x77");
	probe(NODE_DEV(bme680), KIND_TEMP_HUM_PRESS, "BME680");
	probe(NODE_DEV(lps22hb), KIND_PRESSURE, "LPS22HB");
	probe(NODE_DEV(bmp280), KIND_PRESSURE_ALT, "BMP280");
	probe(NODE_DEV(bmp388), KIND_PRESSURE, "BMP388");
	/* SPA06: the same part at its two possible addresses; the one that
	 * isn't there fails its ID check. */
	probe(first_ready(NODE_DEV(spa06), NODE_DEV(spa06_alt), NULL), KIND_PRESSURE, "SPA06");

	/* Not on any bus — a thermistor and a photocell wired straight to the
	 * SoC's ADC, so the node's presence in DT is the whole detection. */
	probe(NODE_DEV(t1000e_sensors), KIND_BOARD_ANALOG, "T1000-E NTC + photocell");

	const struct device *ina3221 = NODE_DEV(ina3221);

	if (sensor_ready(ina3221)) {
		for (uint8_t ch = 1; ch <= 3; ch++) {
			add(ina3221, KIND_POWER_3221, ch, "INA3221");
		}
	}
	probe(NODE_DEV(ina219), KIND_POWER, "INA219");
	const struct device *ina2xx = first_ready(NODE_DEV(ina226), NODE_DEV(ina228),
						  NODE_DEV(ina230));

	if (ina2xx == NULL) {
		ina2xx = first_ready(NODE_DEV(ina232), NODE_DEV(ina236), NODE_DEV(ina237));
	}
	probe(ina2xx, KIND_POWER, "ina2xx");

	if (s_count == 0) {
		LOG_INF("No environment sensors found");
	}
#endif
	return 0;
}

int env_sensor_count(void)
{
#if HAS_ENV_SENSORS
	return s_count;
#else
	return 0;
#endif
}

bool env_sensor_is_board_local(int index)
{
#if HAS_ENV_SENSORS
	return index >= 0 && index < s_count && s_sensors[index].kind == KIND_BOARD_ANALOG;
#else
	ARG_UNUSED(index);
	return false;
#endif
}

int env_sensor_read(int index, struct env_sensor_reading *r)
{
	memset(r, 0, sizeof(*r));
#if HAS_ENV_SENSORS
	if (index < 0 || index >= s_count) {
		return -EINVAL;
	}
	const struct sensor_entry *e = &s_sensors[index];

	if (e->kind == KIND_POWER_3221) {
		struct sensor_value sel = { .val1 = e->sub, .val2 = 0 };

		if (sensor_attr_set(e->dev, SENSOR_CHAN_ALL,
				    (enum sensor_attribute)SENSOR_ATTR_INA3221_SELECTED_CHANNEL,
				    &sel) != 0) {
			return -EIO;
		}
	}
	/* One fetch per part: on the T1000-E it switches the sensor rail. */
	if (sensor_sample_fetch(e->dev) != 0) {
		return -EIO;
	}

	switch (e->kind) {
	case KIND_TEMP_HUMIDITY:
	case KIND_TEMP_HUM_PRESS:
	case KIND_PRESSURE:
	case KIND_PRESSURE_ALT:
		if (channel_float(e->dev, SENSOR_CHAN_AMBIENT_TEMP, &r->temperature_c)) {
			r->fields |= ENV_F_TEMPERATURE;
		}
		if (e->kind <= KIND_TEMP_HUM_PRESS &&
		    channel_float(e->dev, SENSOR_CHAN_HUMIDITY, &r->humidity_pct)) {
			r->fields |= ENV_F_HUMIDITY;
		}
		if (e->kind != KIND_TEMP_HUMIDITY &&
		    channel_float(e->dev, SENSOR_CHAN_PRESS, &r->pressure_hpa)) {
			r->pressure_hpa *= 10.0f;  /* kPa */
			r->fields |= ENV_F_PRESSURE;
			if (e->kind == KIND_TEMP_HUM_PRESS || e->kind == KIND_PRESSURE_ALT) {
				r->altitude_m = pressure_altitude_m(r->pressure_hpa);
				r->fields |= ENV_F_ALTITUDE;
			}
		}
		break;
	case KIND_BOARD_ANALOG:
		if (channel_float(e->dev, SENSOR_CHAN_LIGHT, &r->luminosity)) {
			r->fields |= ENV_F_LUMINOSITY;
		}
		if (channel_float(e->dev, SENSOR_CHAN_AMBIENT_TEMP, &r->temperature_c)) {
			r->fields |= ENV_F_TEMPERATURE;
		}
		break;
	case KIND_POWER:
	case KIND_POWER_3221:
		if (channel_float(e->dev, SENSOR_CHAN_VOLTAGE, &r->voltage_v)) {
			r->fields |= ENV_F_VOLTAGE;
		}
		if (channel_float(e->dev, SENSOR_CHAN_CURRENT, &r->current_a)) {
			r->fields |= ENV_F_CURRENT;
		}
		if (channel_float(e->dev, SENSOR_CHAN_POWER, &r->power_w)) {
			r->fields |= ENV_F_POWER;
		}
		break;
	}
	return 0;
#else
	ARG_UNUSED(index);
	return -ENOTSUP;
#endif
}

bool env_sensors_available(void)
{
	return env_sensor_count() > 0;
}

int env_sensors_read(struct env_data *data)
{
	memset(data, 0, sizeof(*data));

	for (int i = 0; i < env_sensor_count(); i++) {
		struct env_sensor_reading r;

#if HAS_ENV_SENSORS
		if (s_sensors[i].kind >= KIND_POWER) {
			continue;  /* no environment fields; skip the I2C */
		}
#endif
		if (env_sensor_read(i, &r) != 0) {
			continue;
		}
		if ((r.fields & ENV_F_TEMPERATURE) && !data->has_temperature) {
			data->temperature_c = r.temperature_c;
			data->has_temperature = true;
		}
		if ((r.fields & ENV_F_HUMIDITY) && !data->has_humidity) {
			data->humidity_pct = r.humidity_pct;
			data->has_humidity = true;
		}
		if ((r.fields & ENV_F_PRESSURE) && !data->has_pressure) {
			data->pressure_hpa = r.pressure_hpa;
			data->has_pressure = true;
		}
		if ((r.fields & ENV_F_LUMINOSITY) && !data->has_luminosity) {
			data->luminosity = r.luminosity;
			data->has_luminosity = true;
		}
	}
	return (data->has_temperature || data->has_humidity || data->has_pressure ||
		data->has_luminosity) ? 0 : -ENODATA;
}
