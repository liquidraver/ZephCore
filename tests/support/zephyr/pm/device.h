#pragma once
// Host-only device PM boundary; the test file defines pm_device_action_run().
struct device;
enum pm_device_action { PM_DEVICE_ACTION_SUSPEND, PM_DEVICE_ACTION_RESUME };
extern "C" int pm_device_action_run(const struct device *dev, enum pm_device_action action);
