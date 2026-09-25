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
 * Module power is GPIO/regulator controlled — GNSS driver PM is not used
 * for the module itself:
 * - Wio Tracker L1 (L76K): P1.09 is the module's WAKEUP pin, not a supply
 *   switch. Per the L76K hardware design: WAKEUP is a digital input, active
 *   low with an internal pull-up, that "enters or exits Standby mode". In
 *   Standby the RF is powered off but the internal core and I/O power domain
 *   stay active, so VCC is never removed and ephemeris/almanac/RTC survive —
 *   every wake is a warm start, not a cold one. (Backup mode, the deeper
 *   state, requires cutting VCC while V_BCKP holds the RTC domain; this
 *   board has no VCC switch, so Standby is the floor available to us.)
 * - T1000-E (AG3335): GPS_EN de-asserted + VRTC asserted = warm standby (ephemeris
 *   preserved via backup RAM, ~1-2µA VRTC current)
 * - All boards: gps-enable alias → GPIO power control
 *
 * CONFIG_PM_DEVICE is on globally, but nothing suspends automatically —
 * system-managed suspend is compiled only under CONFIG_PM (off everywhere).
 * This manager makes exactly two kinds of PM calls, both main-thread only:
 * - a one-time RESUME of the GNSS device at boot (gnss-nmea-generic inits
 *   suspended under CONFIG_PM_DEVICE and never opens its pipe otherwise);
 * - suspend/resume of the GNSS UARTE around standby/off (an armed UARTE RX
 *   holds HFCLK ≈0.5-1 mA on nRF52840 even with the module powered off).
 * The old "PM broke GPS" deadlock was modem_chat_run_script() being reached
 * from the system workqueue via driver PM hooks — the air530z driver is
 * PM-less now and every PM call here stays on the main thread. */

/* ========== GPS Power GPIO Control ==========
 * These are unconditional (not gated by HAS_GNSS) because
 * gps_power_off_for_shutdown() must be available for System OFF
 * even on boards without a GNSS driver.
 *
 * IMPORTANT: Do NOT touch GPIO during init! The GNSS driver needs the GPS
 * to be powered and outputting NMEA for the modem pipe to work.
 * We only configure GPIO lazily on first power-off request.
 *
 * Board-specific pins (defined in board overlays as gps-enable alias):
 * - T1000-E: P1.11 (GPS_EN), P0.8 (GPS_VRTC_EN), P1.15 (GPS_RESET), P1.12 (GPS_SLEEP_INT)
 * - Wio Tracker L1: P1.09 (GPS power, shared with luatos,air530z on-off-gpios)
 */
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

/* T1000-E has extra GPS control pins that require a specific init sequence */
#define HAS_T1000_GPS_CONTROL (HAS_GPS_VRTC || HAS_GPS_RESET || HAS_GPS_SLEEP)

#if HAS_GPS_POWER_CONTROL
static bool gps_gpio_configured = false;
#endif

/* GPS power control with warm standby support.
 * @param on        true = power on, false = power off
 * @param keep_vrtc When powering off: true = keep VRTC alive (warm standby,
 *                  preserves ephemeris/almanac/RTC for fast re-acquisition),
 *                  false = full power-off (cold start on next wake).
 *                  Only relevant on T1000-E (HAS_GPS_VRTC); ignored on other boards. */
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
	/* Direct GPIO power control — works on all boards.
	 * We toggle the GPS power pin ourselves rather than using driver PM
	 * (driver PM can hang on modem_pipe_close / modem_chat_run_script).
	 * The GNSS driver's modem pipe stays open.
	 *
	 * T1000-E (HAS_GPS_VRTC): Use warm standby (keep VRTC) for app toggle
	 *   so UART/chip state is preserved. Matches Arduino sleep_gps().
	 * Simple boards (Wio etc.): Full power off/on via GPS_EN. */
	if (on) {
#if HAS_T1000_GPS_CONTROL
		/* T1000-E power-on sequence (from Arduino target.cpp start_gps())
		 * Must follow this exact order with delays. Levels are ASSERTED /
		 * DE-ASSERTED, not physical: Arduino states them as HIGH/LOW because
		 * its pins are all active-high, while this block is also reached by
		 * boards whose lines are active-low (see the gate below).
		 * 1. GPS_EN asserted, delay 10ms
		 * 2. GPS_VRTC_EN asserted, delay 10ms (critical - RTC power)
		 * 3. GPS_RESET asserted, delay 10ms, then released
		 * 4. GPS_SLEEP_INT asserted
		 *
		 * Despite the name this is not a T1000-E-only path:
		 * HAS_T1000_GPS_CONTROL is (HAS_GPS_VRTC || HAS_GPS_RESET ||
		 * HAS_GPS_SLEEP), so a bare gps-reset alias is enough to route a
		 * board here. heltec_wifi_lora32_v4, _v43 and thinknode_m9 all arrive
		 * this way, and all three declare gps-enable active-low.
		 */
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
		if (gpio_is_ready_dt(&gps_reset_gpio)) {
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
		/* GPS_RTC_INT (P0.15) — held de-asserted during normal operation */
		if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
			gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_INACTIVE);
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
		LOG_INF("GPS power ON (T1000-E sequence)");
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
		/* Power off sequence */
#if HAS_GPS_RESET
		/* Hold GPS in reset during power-off — matches Arduino sleep_gps()/stop_gps().
		 * Ensures chip sees RESET asserted when GPS_EN is asserted on next
		 * power-on, preventing uncontrolled startup before the reset pulse.
		 * Configure-on-first-use (mirrors the GPS_EN pin below): on a
		 * boot-with-GPS-off the power-on path never ran, so the pin isn't an
		 * output yet — gpio_pin_set_dt() alone would leave it floating instead
		 * of asserting reset. GPIO_OUTPUT_ACTIVE drives the active (asserted)
		 * level directly. */
		if (gpio_is_ready_dt(&gps_reset_gpio)) {
			if (!gps_gpio_configured) {
				gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_ACTIVE);
			} else {
				gpio_pin_set_dt(&gps_reset_gpio, 1);
			}
		}
#endif

#if HAS_GPS_VRTC
		if (!keep_vrtc) {
			/* Full power-off: VRTC off too (cold start on next wake) */
			if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
				if (!gps_gpio_configured) {
					gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_INACTIVE);
				} else {
					gpio_pin_set_dt(&gps_vrtc_gpio, 0);
				}
			}
		}
		/* else: warm standby — VRTC stays asserted, preserving
		 * ephemeris/almanac/RTC for fast re-acquisition (~1-2 µA) */
#endif
		if (gpio_is_ready_dt(&gps_enable_gpio)) {
			if (!gps_gpio_configured) {
				gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_INACTIVE);
				gps_gpio_configured = true;
			} else {
				gpio_pin_set_dt(&gps_enable_gpio, 0);
			}
		}

#if HAS_GPS_RESETB
		/* Assert RESETB when GPS is off (Arduino sleep_gps/stop_gps).
		 * The alias is declared GPIO_ACTIVE_LOW, so this drives it LOW. */
		if (gpio_is_ready_dt(&gps_resetb_gpio)) {
			gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_OUTPUT_ACTIVE);
		}
#endif

#if HAS_GPS_RTCINT
		/* GPS_RTC_INT stays de-asserted during sleep/off (same as normal operation) */
		if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
			gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_INACTIVE);
		}
#endif

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

/* Put every GPS control line this board declares into its System OFF state:
 * the power enable, and where present VRTC, sleep and rtcint, de-asserted;
 * reset and resetb asserted, holding the module in reset while its supply is
 * gated -- the same levels gps_power_control(false) leaves them at.
 * Uses gpio_pin_configure_dt() so pins are properly set even if
 * gps_power_control() was never called (GPIO not yet configured). */
void gps_power_off_for_shutdown(void)
{
#if HAS_GPS_POWER_REGULATOR
	if (gps_reg_enabled && device_is_ready(gps_power_reg)) {
		regulator_disable(gps_power_reg);
		gps_reg_enabled = false;
	}
#endif
#if HAS_GPS_POWER_CONTROL
	/* INACTIVE: physical LOW leaves an active-low enable asserted, i.e. the
	 * module still powered across System OFF. */
	if (gpio_is_ready_dt(&gps_enable_gpio)) {
		gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if HAS_GPS_VRTC
	if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
		gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if HAS_GPS_RESET
	/* ACTIVE, matching the power-off path: reset stays asserted across
	 * System OFF, as Arduino stop_gps() leaves it. */
	if (gpio_is_ready_dt(&gps_reset_gpio)) {
		gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_ACTIVE);
	}
#endif
#if HAS_GPS_SLEEP
	if (gpio_is_ready_dt(&gps_sleep_gpio)) {
		gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if HAS_GPS_RTCINT
	if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
		gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if HAS_GPS_RESETB
	if (gpio_is_ready_dt(&gps_resetb_gpio)) {
		gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_OUTPUT_ACTIVE);
	}
#endif
}

#if HAS_GNSS

/* ========== Software Sleep/Wake (no GPIO required) ==========
 *
 * On boards without dedicated GPS power control (e.g. RAK3401 where the
 * 3V3_S rail is shared with the LoRa FEM), we send vendor-specific UART
 * commands to put the GPS module into low-power mode.
 *
 * Strategy: send BOTH MediaTek and u-blox sleep commands — the module that
 * isn't present simply ignores the bytes it doesn't understand.
 *
 * - MediaTek (e.g. L76B):      $PMTK161,0*28\r\n → standby (~1mA), wake on UART
 * - u-blox ZOE-M8Q (RAK12500): UBX-RXM-PMREQ    → backup  (~7µA), wake on UART
 *
 * Neither reaches a CASIC part (L76K/L76KB/Air530Z): those ignore PMTK, UBX
 * and PCAS12 sleep commands alike — verified on hardware, which is why the
 * boards carrying them duty-cycle with a power GPIO instead. Note also that
 * the RAK1910 is a u-blox MAX-7Q, not an L76K, despite older comments here.
 *
 * Wake: any byte on UART wakes both modules from their low-power modes.
 * After wake, the module resumes outputting NMEA autonomously.
 */

/* HAS_GPS_UART, gps_uart_dev and gps_uart_send are defined near the top of
 * this file (see "GPS Feature Detection") — they are needed by the boot-time
 * module configuration, which runs long before this section. */

/* ========== GNSS UART Suspend/Resume (device PM) ==========
 * nRF UARTE only. An armed UARTE RX holds HFCLK (~0.5-1 mA on nRF52840)
 * even when the GPS module is powered off or silent, so standby/off
 * suspends the UART device and every wake resumes it first.
 *
 * Verified symmetric in uart_nrfx_uarte.c under the still-open modem pipe:
 * suspend saves the RX-interrupt state, STOPRXes, disables the peripheral
 * and applies the sleep pinctrl; resume restores all of it. Other UART
 * drivers (legacy nordic,nrf-uart on RAK4631, ESP32) are deliberately not
 * gated in — their suspend/resume round-trip is unverified and the HFCLK
 * cost is UARTE-specific.
 *
 * Every GPS UART node must carry a sleep pinctrl state (all boards do):
 * without one, suspend fails *after* disabling RX while the PM state stays
 * ACTIVE, so the next resume no-ops with -EALREADY — a dead GPS.
 *
 * REQUIRES GPS POWER CONTROL — do not relax this gate.
 * uarte_pm_suspend() busy-waits for RXTO with no timeout after triggering
 * STOPRX (uart_nrfx_uarte.c, the only unbounded wait in the path). In
 * interrupt-driven mode RX runs on a 1-byte buffer with no ENDRX_STARTRX
 * short, so the receiver stops after every byte until the ISR re-arms it —
 * and suspend disables the ENDRX interrupt *before* STOPRX, removing the
 * re-arm. Land in that window with bytes still arriving and STOPRX hits an
 * already-stopped receiver, no RXTO is generated, and the caller spins
 * forever. It runs on the main thread, so the whole mesh wedges (observed:
 * RAK3401 1W repeater on 1.16.6, CLI answering only "-> busy").
 *
 * Boards with GPIO/regulator power control cut the module before we get
 * here, so the line is genuinely quiet and STOPRX always yields RXTO.
 * Boards without it fall back to gps_software_sleep(), whose PMTK/UBX
 * commands the module may simply ignore (u-blox MAX-7Q on RAK3401 is
 * protocol 14/15; the 16-byte UBX-RXM-PMREQ we send is protocol 23+) —
 * NMEA keeps streaming straight into the suspend. Those boards give up the
 * ~0.5-1 mA HFCLK saving; uptime wins. */
#if HAS_GPS_UART && defined(CONFIG_PM_DEVICE) && \
	(HAS_GPS_POWER_CONTROL || HAS_GPS_POWER_REGULATOR) && \
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
		/* Let the GNSS line go quiet before suspending. Every caller
		 * cuts module power (GPS_EN low / reset asserted / regulator
		 * off) immediately before this, but a byte can still be in
		 * flight. Settle so it finishes and the driver's RX ISR re-arms,
		 * leaving the receiver armed-and-idle when the suspend's STOPRX
		 * fires — that state yields RXTO, whereas a just-stopped,
		 * un-rearmed receiver can produce none and (pre-0010) hung the
		 * main thread. ~5 ms comfortably covers one character time at
		 * GNSS baud plus ISR latency; standby happens at most every few
		 * minutes, so the cost is negligible. Backstop: patch 0010
		 * bounds the driver's RXTO wait so a missed RXTO can never hang
		 * us even if a byte still lands in the race window. */
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

/* u-blox ZOE-M8Q: UBX-RXM-PMREQ → enter backup mode
 * UBX frame: B5 62 | 02 41 | 10 00 | payload(16) | CK_A CK_B
 * Payload (protocol 23+, 16 bytes):
 *   version=0, reserved[3]=0,
 *   duration=0x00000000 (infinite),
 *   flags=0x00000006 (backup + force),
 *   wakeupSources=0x00000028 (uartrx bit3 | extint0 bit5)
 *
 * THE WAKE SOURCE BIT IS LOAD-BEARING. wakeupSources bit 3 is uartrx; bit 5
 * is extint0. This frame previously sent 0x20 — extint0 only — with a comment
 * claiming that was "UART RX (bit 5)". It is not. EXTINT is not routed to the
 * WisBlock connector on the RAK12500 (RAK's datasheet: only UART/I2C, 1PPS,
 * RESET, VDD and GND are connected), so the module was told to sleep forever
 * with a wake source that can never be asserted. duration=0 means infinite,
 * so it never came back: no NMEA at any baud, and no I2C either, because
 * backup mode powers down the DDC interface too. Only a physical power cycle
 * recovered it — a reboot does not, since the 3V3_S rail stays up.
 * Confirmed on hardware: reseat -> module answers -> one fix -> first standby
 * -> gone again.
 *
 * 0x28 sets both, so a board that does wire EXTINT keeps that path as well.
 * Module stops all output and draws ~20µA (ZOE-M8Q at 3V, datasheet Table 13;
 * the 15µA hardware-backup figure needs VCC removed entirely). */
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
void gps_module_power(bool on, bool keep_vrtc)
{
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
