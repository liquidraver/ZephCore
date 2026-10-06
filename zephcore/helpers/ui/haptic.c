/*
 * ZephCore - Haptic feedback (DRV2605)
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "haptic.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_haptic, CONFIG_ZEPHCORE_SENSORS_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(ti_drv2605)

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/drivers/haptics/drv2605.h>

#define HAPTIC_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_drv2605)

#if DT_PROP(HAPTIC_NODE, zephyr_deferred_init)
/* Wake a deferred-init DRV2605 and run its init here. The enable pin is
 * asserted, with a wait, before device_init(): the driver reads the status
 * register right after raising EN. */
static void haptic_deferred_init(const struct device *dev)
{
	static const struct gpio_dt_spec en =
		GPIO_DT_SPEC_GET_OR(HAPTIC_NODE, en_gpios, {0});
	int rc;

	if (en.port != NULL && gpio_is_ready_dt(&en)) {
		if (gpio_pin_configure_dt(&en, GPIO_OUTPUT_ACTIVE) == 0) {
			/* 250 us is the datasheet figure; 1 ms costs nothing on a
			 * one-shot boot path and covers the rail as well. */
			k_msleep(1);
		}
	}

	rc = device_init(dev);
	LOG_DBG("haptic deferred init: rc=%d ready=%d", rc,
		(int)device_is_ready(dev));
}
#else
static inline void haptic_deferred_init(const struct device *dev) { ARG_UNUSED(dev); }
#endif

static const struct device *haptic_dev;
static bool haptic_enabled = true;

/* Library effect 1 = "Strong Click - 100%", short enough to sit under a
 * notification melody without outlasting it. */
static struct drv2605_rom_data rom_data = {
	.trigger = DRV2605_MODE_INTERNAL_TRIGGER,
	.library = DRV2605_LIBRARY_LRA,
	.seq_regs = { 1 },
};

int haptic_init(void)
{
	const union drv2605_config_data cfg = { .rom_data = &rom_data };
	int rc;

	haptic_dev = DEVICE_DT_GET(HAPTIC_NODE);
	if (!device_is_ready(haptic_dev)) {
		haptic_deferred_init(haptic_dev);
	}
	if (!device_is_ready(haptic_dev)) {
		/* The DRV2605 driver logs its own reason at debug level, so say
		 * enough here to tell "chip absent" from "bus down". */
		const struct device *bus = DEVICE_DT_GET(DT_BUS(HAPTIC_NODE));

		LOG_WRN("haptic %s not ready (addr 0x%02x, bus %s %s) — "
			"build with CONFIG_HAPTICS_LOG_LEVEL_DBG for the cause",
			haptic_dev->name, (unsigned int)DT_REG_ADDR(HAPTIC_NODE),
			bus->name, device_is_ready(bus) ? "up" : "DOWN");
		haptic_dev = NULL;
		return -ENODEV;
	}

	rc = drv2605_haptic_config(haptic_dev, DRV2605_HAPTICS_SOURCE_ROM, &cfg);
	if (rc < 0) {
		LOG_WRN("haptic config failed: %d", rc);
		haptic_dev = NULL;
		return rc;
	}

	LOG_INF("haptic driver ready");
	return 0;
}

void haptic_pulse(void)
{
	if (!haptic_dev || !haptic_enabled) {
		return;
	}

	(void)haptics_start_output(haptic_dev);
}

void haptic_set_enabled(bool enabled)
{
	haptic_enabled = enabled;

	if (!enabled && haptic_dev) {
		(void)haptics_stop_output(haptic_dev);
	}
}

bool haptic_available(void)
{
	return haptic_dev != NULL;
}

#else /* no ti,drv2605 node — link-compatible stubs */

int haptic_init(void) { return -ENODEV; }
void haptic_pulse(void) { }
void haptic_set_enabled(bool enabled) { ARG_UNUSED(enabled); }
bool haptic_available(void) { return false; }

#endif
