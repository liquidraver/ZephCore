#pragma once
// Host-only device boundary: each devicetree node label resolves to whatever
// the test file's test_device_by_label() returns (a fake, or nullptr).
struct device { const char *name; };
#define DT_NODELABEL(label) #label
#define DEVICE_DT_GET_OR_NULL(node) test_device_by_label(node)
extern "C" const struct device *test_device_by_label(const char *label);
extern "C" bool device_is_ready(const struct device *dev);
extern "C" int device_init(const struct device *dev);
