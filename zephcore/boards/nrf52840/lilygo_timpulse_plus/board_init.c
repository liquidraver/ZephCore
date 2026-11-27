/*
 * LilyGo T-Impulse Plus — SGM41562 charger setup.
 * SPDX-License-Identifier: MIT
 *
 * The SGM41562 comes out of power-on reset with charging disabled (REG01
 * CEB=1) and its I2C watchdog armed at 160 s whenever input power is present.
 * If nothing programs it, the battery never charges and, on USB, the chip
 * power-cycles the whole system every time the watchdog expires.
 *
 * At boot this writes LilyGo's charger configuration (cpp_bus_driver
 * Sgm41562xx, kInitSequenceAb): watchdog off, NTC off, PCB OTP off, input
 * current limit released, charging on. Every CHARGER_TICK it writes REG05
 * again: on USB insertion and on each nINT pulse the chip re-arms its
 * watchdog at 40 s until the next I2C write. If REG01 shows charging disabled
 * (the chip lost power or reset), the full configuration is applied again.
 *
 * Best effort: a missing or silent charger is logged and boot continues.
 * Rationale and register decoding: devdocs/lld/10-peripherals-power.md.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(timpulse_charger, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#define SGM_ADDR        0x03

#define SGM_REG01       0x01    /* power-on configuration */
#define SGM_REG05       0x05    /* termination / timers / watchdog */
#define SGM_REG06       0x06    /* NTC, shipping FET, INT masks */
#define SGM_REG07       0x07    /* system regulation, PCB OTP */
#define SGM_REG08       0x08    /* status, input current limit release */
#define SGM_REG09       0x09    /* faults */
#define SGM_REG0B       0x0B    /* device id */

#define SGM_ID          0x04
#define SGM_REG01_CEB   0x08    /* 1 = charge disabled */

/* Below the 40 s watchdog the chip re-arms on input or nINT events. */
#define CHARGER_TICK    K_SECONDS(20)

static const struct device *const bus = DEVICE_DT_GET(DT_NODELABEL(charger_i2c));

/* LilyGo order: configure first, enable charging last. REG05 leads so the
 * watchdog is off before anything else. */
static const uint8_t charger_cfg[][2] = {
	{ SGM_REG05, 0x1A },    /* watchdog off, termination on, 5 h safety timer */
	{ SGM_REG07, 0xB7 },    /* PCB OTP off, VSYS 4.55 V, TJ_REG 120 C */
	{ SGM_REG06, 0x40 },    /* NTC off, battery FET on, INTs unmasked */
	{ SGM_REG08, 0x40 },    /* release input current limit */
	{ SGM_REG01, 0xA4 },    /* CEB=0: charging on; HIZ off, UVLO 2.76 V */
};

static struct k_work_delayable charger_work;

static int charger_apply(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(charger_cfg); i++) {
		int rc = i2c_reg_write_byte(bus, SGM_ADDR, charger_cfg[i][0],
					    charger_cfg[i][1]);
		if (rc < 0) {
			return rc;
		}
	}
	return 0;
}

static void charger_tick(struct k_work *work)
{
	uint8_t reg01;

	ARG_UNUSED(work);

	if (i2c_reg_read_byte(bus, SGM_ADDR, SGM_REG01, &reg01) < 0) {
		LOG_WRN("SGM41562 not responding");
	} else if (reg01 & SGM_REG01_CEB) {
		LOG_WRN("SGM41562 back at defaults (REG01=0x%02X), reapplying", reg01);
		if (charger_apply() < 0) {
			LOG_WRN("SGM41562 configuration failed");
		}
	} else {
		/* Any write restores the programmed watchdog setting (off). */
		(void)i2c_reg_write_byte(bus, SGM_ADDR, SGM_REG05, charger_cfg[0][1]);
	}

	k_work_reschedule(&charger_work, CHARGER_TICK);
}

static int timpulse_charger_init(void)
{
	uint8_t id, stat, fault;

	if (!device_is_ready(bus)) {
		LOG_WRN("charger I2C bus not ready");
		return 0;
	}
	if (i2c_reg_read_byte(bus, SGM_ADDR, SGM_REG0B, &id) < 0) {
		LOG_WRN("SGM41562 not found at 0x%02X", SGM_ADDR);
		return 0;
	}
	if (id != SGM_ID) {
		LOG_WRN("unexpected charger id 0x%02X, leaving it alone", id);
		return 0;
	}
	if (charger_apply() < 0) {
		LOG_WRN("SGM41562 configuration failed");
	}

	if (i2c_reg_read_byte(bus, SGM_ADDR, SGM_REG08, &stat) == 0 &&
	    i2c_reg_read_byte(bus, SGM_ADDR, SGM_REG09, &fault) == 0) {
		LOG_INF("SGM41562 charging enabled (status 0x%02X, faults 0x%02X)",
			stat, fault);
	}

	k_work_init_delayable(&charger_work, charger_tick);
	k_work_schedule(&charger_work, CHARGER_TICK);
	return 0;
}

/* After the I2C drivers (KERNEL_INIT_PRIORITY_DEVICE = 50). */
SYS_INIT(timpulse_charger_init, POST_KERNEL, 60);
