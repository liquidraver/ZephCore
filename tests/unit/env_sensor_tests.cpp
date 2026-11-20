#include "test.h"
#include "ZephyrEnvSensors.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <algorithm>
#include <cerrno>
#include <cstring>

// Each case runs in its own process (tests/run.py), so env_sensors_init()
// runs once per case against the fakes set up by start().
namespace {
const struct device ina219 = { "ina219" };
const struct device bme280 = { "bme280" };
bool bme280_present;
int resume_result;
std::vector<std::string> calls;

void start(bool with_bme280, int resume)
{
	bme280_present = with_bme280;
	resume_result = resume;
	calls.clear();
	CHECK(env_sensors_init() == 0);
}

bool fetched()
{
	return std::any_of(calls.begin(), calls.end(),
			   [](const std::string& c) { return c.find(":fetch") != std::string::npos; });
}
}

extern "C" const struct device *test_device_by_label(const char *label)
{
	if (std::strcmp(label, "ina219") == 0) {
		return &ina219;
	}
	if (bme280_present && std::strcmp(label, "bme280") == 0) {
		return &bme280;
	}
	return nullptr;
}
extern "C" bool device_is_ready(const struct device *dev) { return dev != nullptr; }
extern "C" int device_init(const struct device *) { return 0; }
extern "C" int sensor_attr_set(const struct device *, enum sensor_channel, enum sensor_attribute,
			       const struct sensor_value *) { return 0; }
extern "C" int sensor_sample_fetch(const struct device *dev)
{
	calls.push_back(std::string(dev->name) + ":fetch");
	return 0;
}
extern "C" int sensor_channel_get(const struct device *, enum sensor_channel, struct sensor_value *val)
{
	*val = { 1, 0 };
	return 0;
}
extern "C" int pm_device_action_run(const struct device *dev, enum pm_device_action action)
{
	if (action == PM_DEVICE_ACTION_RESUME) {
		calls.push_back(std::string(dev->name) + ":resume");
		return resume_result;
	}
	calls.push_back(std::string(dev->name) + ":suspend");
	return 0;
}

TEST(env_power_monitor_pm, "UNIT-SENSOR-001",
     "A power monitor is powered down after probe and woken only around each read") {
	start(false, 0);
	CHECK(env_sensor_count() == 1);
	CHECK((calls == std::vector<std::string>{ "ina219:suspend" }));
	calls.clear();

	struct env_sensor_reading r;
	CHECK(env_sensor_read(0, &r) == 0);
	CHECK((calls == std::vector<std::string>{ "ina219:resume", "ina219:fetch", "ina219:suspend" }));
	CHECK(r.fields & ENV_F_CURRENT);
}

TEST(env_power_monitor_no_pm, "UNIT-SENSOR-002",
     "A power monitor whose driver has no PM is still read") {
	start(false, -ENOSYS);
	struct env_sensor_reading r;
	CHECK(env_sensor_read(0, &r) == 0);
	CHECK(r.fields & ENV_F_CURRENT);
}

TEST(env_power_monitor_wake_failure, "UNIT-SENSOR-003",
     "A power monitor that fails to wake is not read and reports an error") {
	start(false, -EIO);
	calls.clear();
	struct env_sensor_reading r;
	CHECK(env_sensor_read(0, &r) != 0);
	CHECK(!fetched());
	CHECK(r.fields == 0);
}

TEST(env_power_monitor_already_awake, "UNIT-SENSOR-004",
     "A power monitor that is already awake is read") {
	start(false, -EALREADY);
	struct env_sensor_reading r;
	CHECK(env_sensor_read(0, &r) == 0);
	CHECK(r.fields & ENV_F_CURRENT);
}

TEST(env_sensor_left_alone, "UNIT-SENSOR-005",
     "An environment sensor gets no PM calls, at probe or at read") {
	start(true, 0);
	CHECK(env_sensor_count() == 2);
	CHECK((calls == std::vector<std::string>{ "ina219:suspend" }));
	calls.clear();

	struct env_sensor_reading r;
	CHECK(env_sensor_read(0, &r) == 0);
	CHECK((calls == std::vector<std::string>{ "bme280:fetch" }));
	CHECK(r.fields & ENV_F_TEMPERATURE);
}
