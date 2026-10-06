/*
 * SPDX-License-Identifier: MIT
 * GPS manager: module power (see gps_internal.h).
 */

#include "gps_internal.h"
#include "ZephyrGPSManager.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>

LOG_MODULE_DECLARE(zephcore_gps, CONFIG_ZEPHCORE_GPS_LOG_LEVEL);

/* ========== GPS Power Strategy ==========
 * The module is switched by GPIO or regulator, not by GNSS driver PM. The two
 * PM calls made here, both on the main thread: a one-time RESUME of the GNSS
 * device at boot, and suspend/resume of the GNSS UARTE around standby/off. */

/* ========== GPS Power GPIO Control ==========
 * Not gated by HAS_GNSS: gps_power_off_for_shutdown() must exist for System
 * OFF on every board. No GPIO is touched during init (the GNSS driver needs
 * the module powered); the lines are configured on the first power-off. */
#if HAS_GPS_POWER_CONTROL
static const struct gpio_dt_spec gps_enable_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_enable), gpios);
#endif

/* GPS powered by a PMU regulator rail instead of a discrete enable GPIO (e.g.
 * LilyGo T-Beam: GPS is on the AXP2101 ALDO3 rail). Selected via the chosen
 * `zephcore,gps-power` node pointing at the regulator. It is the power switch
 * on every path, duty-cycle standby included; the AXP2101 VBACKUP charger
 * (below) keeps the receiver's backup domain alive across each cut, so a wake
 * is a warm start. */
#if HAS_GPS_POWER_REGULATOR
static const struct device *const gps_power_reg =
	DEVICE_DT_GET(DT_CHOSEN(zephcore_gps_power));
/* Tracks our intended rail state so enable/disable stay balanced (idempotent).
 * Starts true: the rail is `regulator-boot-on`, so it is already up at boot. */
static bool gps_reg_enabled = true;
#endif

/* AXP2101 backup (button-battery) charger — feeds the GPS receiver's V_BCKP
 * domain so ephemeris/RTC survive main-rail (ALDO3) power cuts, giving a
 * warm/hot re-fix instead of a cold start each duty cycle. The Zephyr regulator
 * driver doesn't expose VBACKUP, so enable it with raw I2C at boot (mirrors
 * Arduino enablePowerOutput(XPOWERS_VBACKUP) + setPowerChannelVoltage 3.3V).
 * Selected via chosen `zephcore,gps-backup-pmu` pointing at the AXP2101 node. */
#if DT_NODE_EXISTS(DT_CHOSEN(zephcore_gps_backup_pmu))
#define AXP2101_REG_CHG_GAUGE_WDT_CTRL  0x18U  /* bit 2 = button-battery charge enable */
#define AXP2101_BTN_CHARGE_ENABLE       BIT(2)
#define AXP2101_REG_BTN_BAT_CHG_VOL_SET 0x6AU  /* low 3 bits: (mV - 2600) / 100 */
#define AXP2101_BTN_VOL_3V3             0x07U  /* (3300 - 2600) / 100 */
static int gps_backup_charger_init(void)
{
	static const struct i2c_dt_spec axp = I2C_DT_SPEC_GET(DT_CHOSEN(zephcore_gps_backup_pmu));

	if (!device_is_ready(axp.bus)) {
		LOG_WRN("GPS backup: AXP2101 I2C bus not ready");
		return 0;
	}
	/* Set the backup-charge target to 3.3V (low 3 bits), then enable the
	 * charger. Read-modify-write so the fuel-gauge enable (bit 3 of 0x18) and
	 * the other 0x6A bits are preserved. */
	i2c_reg_update_byte_dt(&axp, AXP2101_REG_BTN_BAT_CHG_VOL_SET, 0x07U, AXP2101_BTN_VOL_3V3);
	i2c_reg_update_byte_dt(&axp, AXP2101_REG_CHG_GAUGE_WDT_CTRL,
			       AXP2101_BTN_CHARGE_ENABLE, AXP2101_BTN_CHARGE_ENABLE);
	LOG_INF("GPS backup: AXP2101 VBACKUP charger enabled (3.3V)");
	return 0;
}
/* After the MFD/I2C is up (POST_KERNEL ~86); APPLICATION is safely later. */
SYS_INIT(gps_backup_charger_init, APPLICATION, 50);
#endif

/* T1000-E specific GPS control pins */
#if DT_NODE_EXISTS(DT_ALIAS(gps_vrtc_enable))
static const struct gpio_dt_spec gps_vrtc_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_vrtc_enable), gpios);
#define HAS_GPS_VRTC 1
#else
#define HAS_GPS_VRTC 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(gps_reset))
static const struct gpio_dt_spec gps_reset_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_reset), gpios);
#define HAS_GPS_RESET 1
#else
#define HAS_GPS_RESET 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(gps_sleep_int))
static const struct gpio_dt_spec gps_sleep_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_sleep_int), gpios);
#define HAS_GPS_SLEEP 1
#else
#define HAS_GPS_SLEEP 0
#endif

/* GPS RTC interrupt pin — held de-asserted during normal operation */
#if DT_NODE_EXISTS(DT_ALIAS(gps_rtc_int))
static const struct gpio_dt_spec gps_rtcint_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_rtc_int), gpios);
#define HAS_GPS_RTCINT 1
#else
#define HAS_GPS_RTCINT 0
#endif

/* GPS RESETB (active-LOW reset, declared GPIO_ACTIVE_LOW) — must be
 * INPUT_PULLUP for normal operation; the pull-up is a physical setting.
 * Without the pull-up, this pin floats LOW and holds the AG3335 in permanent
 * reset, preventing any UART output. */
#if DT_NODE_EXISTS(DT_ALIAS(gps_resetb))
static const struct gpio_dt_spec gps_resetb_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_resetb), gpios);
#define HAS_GPS_RESETB 1
#else
#define HAS_GPS_RESETB 0
#endif

/* Module standby pin (asserted = run) on a board that also has a supply
 * switch: standby uses the pin, a full off cuts the supply. A board whose only
 * control is the standby pin declares it as gps-enable instead. */
#if HAS_GPS_WAKEUP
static const struct gpio_dt_spec gps_wakeup_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_wakeup), gpios);
#endif

/* T1000-E has extra GPS control pins that require a specific init sequence */
#define HAS_T1000_GPS_CONTROL (HAS_GPS_VRTC || HAS_GPS_RESET || HAS_GPS_SLEEP)

/* AG3335 RTC backup sleep (upstream AirohaSleep.h; T1000-E, MeshTracker X1):
 * the module is told to save its state and sleep before GPS_EN is cut, VRTC
 * keeps its RTC domain powered, and a GPS_RTC_INT pulse wakes it. */
#define HAS_GPS_BACKUP_SLEEP \
	(HAS_GPS_POWER_CONTROL && HAS_GPS_VRTC && HAS_GPS_RTCINT && HAS_GPS_UART)

#if HAS_GPS_POWER_CONTROL
static bool gps_gpio_configured = false;
/* T1000-E sequence only: the module was powered on by us / is in backup sleep. */
static bool gps_module_on = false;
static bool gps_backup_sleeping = false;
#endif

#if HAS_GPS_BACKUP_SLEEP
/* Upstream stop_gps(). The ack ($PAIR001,650,0) cannot be read here, the GNSS
 * driver owns the UART RX, so the command goes out blind and upstream's 50 ms
 * ack window plus 50 ms grace is waited out. A missed command is the old
 * behaviour: a power cut with VRTC kept. */
static void gps_enter_backup_sleep(void)
{
	static const char pair_sleep[] = "$PAIR650,0*25\r\n";

	gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_INACTIVE);
	gps_uart_send((const uint8_t *)pair_sleep, sizeof(pair_sleep) - 1);
	k_msleep(100);
	gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_INACTIVE);
	gps_module_on = false;
	gps_backup_sleeping = true;
	LOG_INF("GPS power OFF (RTC backup sleep)");
}
#endif

/* The off levels, shared by gps_power_control(false) and System OFF: reset
 * asserted, enable and rtcint de-asserted, VRTC de-asserted unless keep_vrtc.
 * The sleep line stays asserted at runtime and is dropped for System OFF.
 * Every line is configured here: a boot with the GPS off never ran power-on. */
static void gps_drive_off_levels(bool keep_vrtc, bool system_off)
{
#if HAS_GPS_WAKEUP
	/* Low before the supply goes: no pin driven into an unpowered module. */
	if (gpio_is_ready_dt(&gps_wakeup_gpio)) {
		gpio_pin_configure_dt(&gps_wakeup_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if HAS_GPS_RESET
	if (gpio_is_ready_dt(&gps_reset_gpio)) {
		gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_ACTIVE);
	}
#endif
#if HAS_GPS_VRTC
	if (!keep_vrtc && gpio_is_ready_dt(&gps_vrtc_gpio)) {
		gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_INACTIVE);
	}
#else
	ARG_UNUSED(keep_vrtc);
#endif
#if HAS_GPS_POWER_CONTROL
	/* INACTIVE, not LOW: physical LOW leaves an active-low enable asserted. */
	if (gpio_is_ready_dt(&gps_enable_gpio)) {
		gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_INACTIVE);
		gps_gpio_configured = true;
	}
#endif
#if HAS_GPS_SLEEP
	if (system_off && gpio_is_ready_dt(&gps_sleep_gpio)) {
		gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_INACTIVE);
	}
#else
	ARG_UNUSED(system_off);
#endif
#if HAS_GPS_RTCINT
	if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
		gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if HAS_GPS_RESETB
	/* Declared GPIO_ACTIVE_LOW, so ACTIVE drives it LOW. */
	if (gpio_is_ready_dt(&gps_resetb_gpio)) {
		gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_OUTPUT_ACTIVE);
	}
#endif
}

/* GPS power control.
 * @param on        true = power on, false = power off
 * @param keep_vrtc when powering off: true = the state-keeping off this board
 *                  has, false = full power-off. A board with one off state
 *                  lands in it either way. */
void gps_power_control(bool on, bool keep_vrtc)
{
#if HAS_GPS_POWER_REGULATOR
	/* Master power rail (PMU regulator). Idempotent enable/disable so the
	 * refcount stays balanced regardless of how often this is called. */
	if (on != gps_reg_enabled && device_is_ready(gps_power_reg)) {
		int ret = on ? regulator_enable(gps_power_reg)
			     : regulator_disable(gps_power_reg);
		if (ret == 0) {
			gps_reg_enabled = on;
			LOG_INF("GPS power %s (regulator)", on ? "ON" : "OFF");
		} else {
			LOG_WRN("GPS regulator %s failed: %d", on ? "enable" : "disable", ret);
		}
	}
#endif
#if HAS_GPS_POWER_CONTROL
	/* We switch the module ourselves; the GNSS driver's modem pipe stays open. */
	if (on) {
#if HAS_GPS_WAKEUP
		if (gpio_is_ready_dt(&gps_wakeup_gpio)) {
			gpio_pin_configure_dt(&gps_wakeup_gpio, GPIO_OUTPUT_ACTIVE);
		}
#endif
#if HAS_T1000_GPS_CONTROL
		/* Power-on sequence, in this order with delays (Arduino start_gps()):
		 * enable, VRTC, reset pulse, sleep line. Levels are asserted/de-asserted, not
		 * physical; a bare gps-reset alias is enough to route a board here. */
		if (gpio_is_ready_dt(&gps_enable_gpio)) {
			gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_ACTIVE);
		}
		k_msleep(10);

#if HAS_GPS_VRTC
		if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
			gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_ACTIVE);
		}
		k_msleep(10);
#endif

#if HAS_GPS_RESET
		/* Not when waking from backup sleep: upstream resets the module
		 * once, in begin(), and a reset here would make the wake a restart. */
		if (!gps_backup_sleeping && gpio_is_ready_dt(&gps_reset_gpio)) {
			gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_ACTIVE);
			k_msleep(10);
			gpio_pin_set_dt(&gps_reset_gpio, 0);  /* Release reset (logical) */
		}
#endif

#if HAS_GPS_SLEEP
		if (gpio_is_ready_dt(&gps_sleep_gpio)) {
			gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_ACTIVE);
		}
#endif

#if HAS_GPS_RTCINT
		/* GPS_RTC_INT: a 5 ms pulse wakes the AG3335 from RTC backup sleep
		 * (upstream start_gps(), on every power-on); otherwise de-asserted. */
		if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
			gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_ACTIVE);
			k_msleep(5);
			gpio_pin_set_dt(&gps_rtcint_gpio, 0);
		}
#endif

#if HAS_GPS_RESETB
		/* GPS_RESETB (P1.14) — active-LOW reset, must be pulled HIGH.
		 * INPUT_PULLUP de-asserts reset so the AG3335 can boot.
		 * Without this the pin floats LOW → chip stuck in reset → no UART. */
		if (gpio_is_ready_dt(&gps_resetb_gpio)) {
			gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_INPUT | GPIO_PULL_UP);
		}
#endif
		gps_gpio_configured = true;
		LOG_INF("GPS power ON (T1000-E sequence%s)",
			gps_backup_sleeping ? ", wake from backup sleep" : "");
		gps_module_on = true;
		gps_backup_sleeping = false;
#else
		/* Simple boards - just GPS_EN */
		if (!gps_gpio_configured) {
			if (gpio_is_ready_dt(&gps_enable_gpio)) {
				/* ACTIVE, not HIGH: gpio_pin_set_dt() below is
				 * logical, so a physical init flag here would assert
				 * the opposite level on an active-low gps-enable. */
				gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_ACTIVE);
				gps_gpio_configured = true;
				LOG_INF("GPS power GPIO configured, set ACTIVE");
			} else {
				LOG_WRN("GPS power GPIO not ready");
				return;
			}
		} else {
			gpio_pin_set_dt(&gps_enable_gpio, 1);
			LOG_INF("GPS power ON");
		}
#endif
	} else {
#if HAS_GPS_BACKUP_SLEEP
		if (keep_vrtc && gps_module_on) {
			gps_enter_backup_sleep();
			return;
		}
		if (keep_vrtc && gps_backup_sleeping) {
			return;  /* already asleep (disabled while in standby) */
		}
#endif
#if HAS_GPS_WAKEUP
		/* Standby by the module's own pin, supply kept: the wake is a hot
		 * start and skips the reset pulse (gps_backup_sleeping). */
		if (keep_vrtc && gpio_is_ready_dt(&gps_wakeup_gpio)) {
			gpio_pin_configure_dt(&gps_wakeup_gpio, GPIO_OUTPUT_INACTIVE);
			gps_module_on = false;
			gps_backup_sleeping = true;
			LOG_INF("GPS standby (WAKEUP pin, supply kept)");
			return;
		}
#endif
		gps_drive_off_levels(keep_vrtc, false);
		gps_module_on = false;
		gps_backup_sleeping = false;

#if HAS_GPS_VRTC
		LOG_INF("GPS power OFF (%s)", keep_vrtc ?
			"standby — VRTC retained" : "full");
#else
		LOG_INF("GPS power OFF");
#endif
	}
#else
	ARG_UNUSED(keep_vrtc);
#endif
}

/* Put every GPS control line this board declares into its System OFF state
 * (see gps_drive_off_levels()), the regulator rail off and VRTC off too.
 * Configures the pins, so it works even if gps_power_control() never ran. */
void gps_power_off_for_shutdown(void)
{
#if HAS_GPS_POWER_REGULATOR
	if (gps_reg_enabled && device_is_ready(gps_power_reg)) {
		regulator_disable(gps_power_reg);
		gps_reg_enabled = false;
	}
#endif
	gps_drive_off_levels(false, true);
}

#if HAS_GNSS

/* ========== Software Sleep/Wake (no GPIO required) ==========
 * Boards without GPS power control: both the MediaTek and the u-blox sleep
 * command are sent, and the absent module ignores the other's bytes. Any byte
 * on the UART wakes either. CASIC parts ignore both. */

/* HAS_GPS_UART, gps_uart_dev and gps_uart_send are defined near the top of
 * this file (see "GPS Feature Detection") — they are needed by the boot-time
 * module configuration, which runs long before this section. */

/* ========== GNSS UART Suspend/Resume (device PM) ==========
 * nRF UARTE only: an armed UARTE RX holds HFCLK even with the module off, so
 * standby/off suspends the UART and every wake resumes it first. Every GPS
 * UART node needs a sleep pinctrl state. Relies on patch 0010. */
#if HAS_GPS_UART && defined(CONFIG_PM_DEVICE) && \
	DT_NODE_HAS_COMPAT(DT_BUS(DT_NODELABEL(gnss)), nordic_nrf_uarte)
#define HAS_GPS_UART_PM 1
#else
#define HAS_GPS_UART_PM 0
#endif

/* Suspend/resume the GNSS UART. Main thread only (like all GPS power
 * paths — pm_device_action_run() calls the driver synchronously).
 * Ordering: resume BEFORE powering the module / sending the wake byte;
 * suspend AFTER the module is off / sleep commands were sent. */
#if HAS_GPS_UART_PM
void gps_uart_set_power(bool on)
{
	if (!device_is_ready(gps_uart_dev)) {
		return;
	}
	if (!on) {
		/* Let a byte in flight finish and the RX ISR re-arm before the suspend's
		 * STOPRX. */
		k_msleep(5);
	}
	int ret = pm_device_action_run(gps_uart_dev,
				       on ? PM_DEVICE_ACTION_RESUME
					  : PM_DEVICE_ACTION_SUSPEND);
	if (ret == 0) {
		LOG_INF("GPS UART %s", on ? "resumed" : "suspended");
	} else if (ret != -EALREADY) {
		LOG_WRN("GPS UART %s failed: %d", on ? "resume" : "suspend", ret);
	}
}
#else
void gps_uart_set_power(bool on) { ARG_UNUSED(on); }
#endif

#if HAS_GPS_UART && !HAS_GPS_POWER_CONTROL && !HAS_GPS_POWER_REGULATOR
/* gps_uart_send() lives near the top of the file — see "GPS Feature
 * Detection". Only the sleep/wake commands below are gated on this board
 * having no hardware GPS power control. */

/* MediaTek parts: $PMTK161,0*28\r\n → enter standby mode
 * Module stops NMEA output and draws ~1mA. Wakes on any UART RX byte.
 * Inert on CASIC parts (L76K and relatives) — they have no such command. */
static const uint8_t pmtk_standby[] = "$PMTK161,0*28\r\n";

/* u-blox ZOE-M8Q: UBX-RXM-PMREQ, enter backup mode until a wake source fires.
 * wakeupSources = 0x28: uartrx (bit 3) | extint0 (bit 5). uartrx must be set:
 * EXTINT is not wired on the RAK12500, and without it the module never wakes. */
static const uint8_t ubx_pmreq_backup[] = {
	0xB5, 0x62,             /* UBX sync chars */
	0x02, 0x41,             /* Class: RXM, ID: PMREQ */
	0x10, 0x00,             /* Length: 16 bytes (little-endian) */
	/* Payload */
	0x00,                   /* version */
	0x00, 0x00, 0x00,       /* reserved1[3] */
	0x00, 0x00, 0x00, 0x00, /* duration: 0 = infinite */
	0x06, 0x00, 0x00, 0x00, /* flags: backup(0x02) | force(0x04) */
	0x28, 0x00, 0x00, 0x00, /* wakeupSources: uartrx(bit3) | extint0(bit5) */
	/* Checksum (Fletcher-8 over class..payload) */
	0x81, 0xEB
};

/* Put GPS module into software sleep (for boards without GPIO power control).
 * Sends both Quectel PMTK and u-blox UBX commands — the wrong one is
 * harmlessly ignored by whichever module is actually connected. */
static void gps_software_sleep(void)
{
	LOG_INF("GPS: Sending software sleep (PMTK + UBX)");

	/* MediaTek standby */
	gps_uart_send(pmtk_standby, sizeof(pmtk_standby) - 1);  /* exclude null terminator */

	/* Small delay between commands — let the first one drain */
	k_msleep(50);

	/* u-blox ZOE-M8Q backup */
	gps_uart_send(ubx_pmreq_backup, sizeof(ubx_pmreq_backup));

	LOG_DBG("GPS: Software sleep commands sent");
}

/* Wake GPS module from software sleep.
 * A single 0xFF byte on UART triggers wake on both Quectel and u-blox.
 * After wake, the module resumes NMEA output within ~100-500ms. */
static void gps_software_wake(void)
{
	LOG_INF("GPS: Sending UART wake byte");
	const uint8_t wake = 0xFF;
	gps_uart_send(&wake, 1);
	/* Give the module time to boot and start NMEA output */
	k_msleep(200);
}
#endif /* HAS_GPS_UART && !HAS_GPS_POWER_CONTROL && !HAS_GPS_POWER_REGULATOR */

/* Module on or off with whatever this board has: the power GPIO or PMU rail,
 * else the UART sleep commands. keep_vrtc: T1000-E warm standby (off only). */
uint32_t gps_power_on_count;
uint32_t gps_power_off_count;

void gps_module_power(bool on, bool keep_vrtc)
{
	if (on) {
		gps_power_on_count++;
	} else {
		gps_power_off_count++;
	}
#if HAS_GPS_POWER_CONTROL || HAS_GPS_POWER_REGULATOR
	gps_power_control(on, keep_vrtc);
#elif HAS_GPS_UART
	ARG_UNUSED(keep_vrtc);
	if (on) {
		gps_software_wake();
	} else {
		gps_software_sleep();
	}
#else
	ARG_UNUSED(on);
	ARG_UNUSED(keep_vrtc);
#endif
}

#if HAS_GPS_POWER_CONTROL
/**
 * Log actual GPIO pin states after power-up sequence.
 * Reads back each configured pin to verify the hardware accepted our config.
 */
void gps_dump_gpio_states(void)
{
	/* Port/pin come from the gpio_dt_spec so the board's real wiring is
	 * printed. These used to be hardcoded T1000-E pin numbers, which read
	 * as plausible nonsense on every other board. */
#define GPS_LOG_PIN(_label, _spec)                                            \
	if (gpio_is_ready_dt(&(_spec))) {                                     \
		LOG_INF("  %-14s %s.%02u: %d", _label, (_spec).port->name,    \
			(_spec).pin, gpio_pin_get_dt(&(_spec)));              \
	}

	LOG_INF("GPS GPIO states after power-up:");
	GPS_LOG_PIN("GPS_EN", gps_enable_gpio);
#if HAS_GPS_VRTC
	GPS_LOG_PIN("GPS_VRTC_EN", gps_vrtc_gpio);
#endif
#if HAS_GPS_RESET
	GPS_LOG_PIN("GPS_RESET", gps_reset_gpio);
#endif
#if HAS_GPS_SLEEP
	GPS_LOG_PIN("GPS_SLEEP_INT", gps_sleep_gpio);
#endif
#if HAS_GPS_RTCINT
	GPS_LOG_PIN("GPS_RTC_INT", gps_rtcint_gpio);
#endif
#if HAS_GPS_RESETB
	GPS_LOG_PIN("GPS_RESETB", gps_resetb_gpio);   /* INPUT_PULLUP, expect 0 (logical, de-asserted) */
#endif

#undef GPS_LOG_PIN
}
#endif /* HAS_GPS_POWER_CONTROL */

#endif /* HAS_GNSS */
