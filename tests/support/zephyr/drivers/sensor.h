#pragma once
#include <cstdint>
// Host-only sensor API boundary; the test file defines the functions.
struct device;
struct sensor_value { int32_t val1; int32_t val2; };
enum sensor_channel {
	SENSOR_CHAN_ALL, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_HUMIDITY, SENSOR_CHAN_PRESS,
	SENSOR_CHAN_LIGHT, SENSOR_CHAN_VOLTAGE, SENSOR_CHAN_CURRENT, SENSOR_CHAN_POWER,
};
enum sensor_attribute { SENSOR_ATTR_PRIV_START = 0x8000 };
extern "C" int sensor_sample_fetch(const struct device *dev);
extern "C" int sensor_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val);
extern "C" int sensor_attr_set(const struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr, const struct sensor_value *val);
inline float sensor_value_to_float(const struct sensor_value *val)
{
	return (float)val->val1 + (float)val->val2 / 1000000.0f;
}
