/*
 * SPDX-License-Identifier: MIT
 * ZephCore - OLED display power management
 *
 * Sends I2C display-off command to SH1106/SSD1306 OLED to save power.
 * The Wio Tracker L1's SH1106 has no hardware power gate, so this is
 * the only way to prevent it from drawing ~20mA in standby.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_oled_power, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#include "oled_power.h"

#if IS_ENABLED(CONFIG_I2C) && DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)

#include <zephyr/drivers/i2c.h>

#define OLED_I2C_ADDR 0x3D  /* Wio Tracker L1 SH1106 address */

void oled_sleep(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		return;
	}

	uint8_t cmd[] = {0x00, 0xAE};  /* Display OFF */
	int ret = i2c_write(i2c_dev, cmd, sizeof(cmd), OLED_I2C_ADDR);
	if (ret == 0) {
		LOG_INF("OLED display put to sleep (power saving)");
	} else {
		LOG_DBG("OLED sleep failed (ret=%d) — no OLED present", ret);
	}
}

#else

void oled_sleep(void) { }

#endif
