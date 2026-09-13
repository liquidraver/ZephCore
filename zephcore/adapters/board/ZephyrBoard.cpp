/*
 * SPDX-License-Identifier: MIT
 */

#include "ZephyrBoard.h"
#include "battery_curve.h"
#include "led_gate.h"
#include <NodePrefs.h>   /* LEDS_RADIO_* mode values */
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>
#include <string.h>

#if defined(CONFIG_SOC_SERIES_NRF52)
#include <hal/nrf_power.h>
/* Adafruit bootloader GPREGRET magic values */
#define BOOTLOADER_DFU_SERIAL_MAGIC 0x4e  /* Enter serial DFU mode (CDC only) */
#define BOOTLOADER_DFU_UF2_MAGIC    0x57  /* Enter UF2 mass storage mode (CDC + MSC) */
#define BOOTLOADER_DFU_OTA_MAGIC    0xA8  /* Enter BLE OTA DFU mode */
#define NRF52_GPREGRET 1
#endif

/* ESP32 boards whose console is the native USB-Serial-JTAG: sys_reboot() does
 * a digital soft reset but does not detach the USB PHY, so the D+ pull-up stays
 * asserted and the host never sees a disconnect.  On macOS the serial port then
 * wedges after a settings reboot (GH #43).  Drop the pad before rebooting so the
 * host gets a clean disconnect/re-enumerate.  Gated to exactly the boards that
 * both exhibit the defect and expose the PHY knob to fix it. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32) && \
    DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), espressif_esp32_usb_serial)
#include <hal/usb_serial_jtag_ll.h>
#define ESP32_USB_SERIAL_DETACH 1
#endif

/* ESP32-S3: reboot into the ROM download mode rather than back into the app.
 *
 * The USB CDC companion transport (boards/common/esp32s3_usb.conf) hands the
 * shared D+/D- pads to USB OTG and disables USB-Serial-JTAG.  USJ is exactly
 * what esptool drives to put the chip into download mode, so once our CDC-ACM
 * device owns the port esptool's auto-reset is inert: DTR/RTS land on a Zephyr
 * CDC endpoint that is not wired to EN/BOOT, and the only way back into the
 * bootloader is the physical BOOT button.
 *
 * FORCE_DOWNLOAD_BOOT lives in the RTC domain, which a software system reset
 * does not clear (only a power-on reset does), so setting it and rebooting
 * brings the ROM up in download mode.  That gives both `start dfu` and the
 * Arduino-style 1200-baud touch a way to hand the port back to esptool /
 * esptool-js / the web configurator.
 *
 * Setting it is not enough on its own, though.  WHICH USB controller the ROM
 * appears on is a separate mux, and it lives in the same RTC domain: Zephyr's
 * DWC2 quirk layer claims the internal PHY for USB OTG at init
 * (usb_wrap_ll_phy_enable_external(hw, false) -> RTC_CNTL_USB_CONF_REG
 * SW_HW_USB_PHY_SEL=1, SW_USB_PHY_SEL=1), and those bits survive the reboot
 * exactly like FORCE_DOWNLOAD_BOOT does.  Left alone the ROM therefore comes up
 * on USB OTG (303a:0009), not USB-Serial-JTAG (303a:1001).  That state is still
 * flashable -- esptool reports "USB mode: USB-OTG" and even loads the stub --
 * but it breaks everything that expects USJ: esptool's auto-reset does nothing
 * over ROM OTG CDC so it needs --before no-reset, and esptool-js special-cases
 * only PID 0x1001, so browser flashers fall into a classic DTR/RTS reset path
 * that cannot work.  Clearing SW_HW_USB_PHY_SEL hands the mux back to
 * hardware/eFuse control, whose default routes the internal PHY to USJ -- i.e.
 * it reproduces what a power-on reset would have left behind.
 *
 * Same register and sequence Arduino-ESP32 uses in usb_persist_restart()
 * (RESTART_BOOTLOADER).  OPTION1 carries no other bits on this SoC, but set the
 * bit rather than writing the word so it stays correct if that changes.  Scoped
 * to the S3 deliberately: it is the only part we ship whose USB port can be
 * taken away from the ROM this way (C3/C6 use a different LP_AON register, and
 * classic ESP32 has no native USB — it always flashes through its UART bridge).
 */
#if defined(CONFIG_SOC_SERIES_ESP32S3)
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#define ESP32_FORCE_DOWNLOAD_BOOT 1
#endif

/* Detach the USB device stack before a reset so the host sees a real unplug —
 * see zephcore_usbd_detach().  Matters most on the ESP32-S3 USB companion,
 * whose OTG PHY survives a soft reset with D+ still pulled up.
 *
 * The condition below must be EXACTLY the one CMakeLists.txt uses to compile
 * adapters/usb/ZephyrUSBCDC.cpp, or this call site compiles against an
 * implementation that is never linked (findings #22 — that failure mode has
 * already cost this repo four separate undefined-reference bugs).  The
 * repeater/observer/room-server branches use the inner half alone; the
 * companion branch wraps it in (LOG || COMPANION_USB || COMPANION_SERIAL), and
 * ZEPHCORE_COMPANION is the compile definition that identifies that branch. */
#if !defined(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT) && \
    (defined(CONFIG_USB_CDC_ACM) || defined(CONFIG_USBD_CDC_ACM_CLASS))
#if !defined(ZEPHCORE_COMPANION) || defined(CONFIG_LOG) || \
    defined(CONFIG_ZEPHCORE_COMPANION_USB) || defined(CONFIG_ZEPHCORE_COMPANION_SERIAL)
#include <ZephyrUSBCDC.h>
#define ZEPHCORE_USBD_DETACH 1
#endif
#endif

/* LoRa radio activity LED (optional — defined per-board via DT alias). Still
 * called tx_led after "set leds.radio" gave it an RX mode, because the DT
 * alias it comes from is named lora-tx-led on every board that has one.
 *
 * PWM path (DT_ALIAS(lora_tx_pwm_led)) takes priority when it exists — used
 * on boards where the TX LED is the same physical LED as the heartbeat
 * (e.g. T096, see helpers/ui/ui_common.c), so both share the one brightness
 * in helpers/led_gate.h instead of the plain GPIO path fighting over the
 * pin. */
#if DT_NODE_EXISTS(DT_ALIAS(lora_tx_pwm_led))
static const struct pwm_dt_spec tx_led_pwm = PWM_DT_SPEC_GET(DT_ALIAS(lora_tx_pwm_led));
#define HAS_TX_LED_PWM 1
#define HAS_TX_LED 0
/* Init deferred to the combined tx_led_init() below (with the GPIO path),
 * since both now also need to arm s_rx_pulse_off -- see that block. */
#elif DT_NODE_EXISTS(DT_ALIAS(lora_tx_led))
static const struct gpio_dt_spec tx_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(lora_tx_led), gpios);
#define HAS_TX_LED_PWM 0
#define HAS_TX_LED 1
#else
#define HAS_TX_LED_PWM 0
#define HAS_TX_LED 0
#endif

/* True when this board wires the activity LED to the same pin as the heartbeat
 * (8 of the supported boards do, counting only the plain-GPIO ones -- see the
 * PWM branch below for the rest).  Only those pay for the arbitration hold in
 * led_gate.c — everywhere else the two LEDs are independent and the calls
 * compile out.  led1 is checked as well because ui_common.c falls back to it
 * when a board has no led0. */
#if HAS_TX_LED && DT_NODE_EXISTS(DT_ALIAS(led0)) && \
    DT_SAME_NODE(DT_ALIAS(led0), DT_ALIAS(lora_tx_led))
#define ZEPHCORE_LED_PIN_SHARED 1
#elif HAS_TX_LED && !DT_NODE_EXISTS(DT_ALIAS(led0)) && \
      DT_NODE_EXISTS(DT_ALIAS(led1)) && \
      DT_SAME_NODE(DT_ALIAS(led1), DT_ALIAS(lora_tx_led))
#define ZEPHCORE_LED_PIN_SHARED 1
/* PWM equivalent: true when heartbeat and TX activity are aliased to the same
 * PWM LED node (T096, Wireless Tracker V2). Not the case on every PWM board --
 * RAK3401 has two independent PWM LEDs (green/blue), so this stays 0 there and
 * the hold below is correctly never taken. */
#elif HAS_TX_LED_PWM && DT_NODE_EXISTS(DT_ALIAS(heartbeat_pwm_led)) && \
      DT_SAME_NODE(DT_ALIAS(heartbeat_pwm_led), DT_ALIAS(lora_tx_pwm_led))
#define ZEPHCORE_LED_PIN_SHARED 1
#else
#define ZEPHCORE_LED_PIN_SHARED 0
#endif

/* PWM-or-GPIO write for the activity LED, mirroring heartbeat_led_write() in
 * helpers/ui/ui_common.c -- same reasoning: the write itself must not care
 * which path is compiled in, only the callers (onBeforeTransmit/
 * onAfterTransmit/onPacketReceived/rx_pulse_off_handler below) do. */
#if HAS_TX_LED_PWM
static inline void tx_led_write(bool on)
{
	uint32_t pulse = on ? (uint32_t)((uint64_t)tx_led_pwm.period *
					  zephcore_led_brightness_pct() / 100)
			     : 0;
	pwm_set_pulse_dt(&tx_led_pwm, pulse);
}
#elif HAS_TX_LED
static inline void tx_led_write(bool on)
{
	gpio_pin_set_dt(&tx_led, on ? 1 : 0);
}
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_board, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
    DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/regulator.h>

/* Battery ADC channel from devicetree zephyr,user { io-channels } */
static const struct adc_dt_spec vbat_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static bool vbat_adc_configured;

/* Battery ADC enable regulator (optional - saves power when not reading) */
#if DT_NODE_EXISTS(DT_NODELABEL(vbat_enable))
static const struct device *vbat_enable_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(vbat_enable));
#else
static const struct device *vbat_enable_dev = NULL;
#endif

/*
 * Battery voltage multiplier - prefer devicetree, fallback to Kconfig
 * Formula: Battery_mV = (raw * VBAT_MV_MULTIPLIER) / 4096
 *
 * To define in devicetree, add to board's DTS/overlay:
 *   zephyr,user {
 *       vbat-mv-multiplier = <7200>;
 *   };
 */
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, vbat_mv_multiplier)
#define VBAT_MV_MULTIPLIER DT_PROP(ZEPHYR_USER_NODE, vbat_mv_multiplier)
#else
#define VBAT_MV_MULTIPLIER CONFIG_ZEPHCORE_VBAT_MV_MULTIPLIER
#endif
#define VBAT_ADC_SAMPLES   8
#endif

/* Battery fuel gauge (AXP2101 PMU etc.) — preferred over the ADC divider when
 * present. Reports battery voltage and state-of-charge over I2C, so boards with
 * a PMU and no battery ADC (e.g. LilyGo T-Beam) still get battery telemetry. */
#if DT_HAS_COMPAT_STATUS_OKAY(x_powers_axp2101_fuel_gauge)
#include <zephyr/drivers/fuel_gauge.h>
#define HAS_FUEL_GAUGE 1
static const struct device *const fuel_gauge_dev =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(x_powers_axp2101_fuel_gauge));
#else
#define HAS_FUEL_GAUGE 0
#endif

/* Initialize activity LED (PWM or GPIO) at boot */
#if HAS_TX_LED_PWM || HAS_TX_LED
/* Width of the receive blink.  A transmit holds the LED for its whole airtime,
 * but a receive is a single edge — the packet is over by the time the driver
 * hands it up — so RX has to be a fixed one-shot.  30 ms is deliberately longer
 * than the heartbeat's 20 ms tick so the two read differently on the boards
 * that share one pin, and short enough that a busy channel gives a flicker
 * rather than a solid glow. */
#define RX_PULSE_MS 30

/* Set only while a transmit is actually holding the LED lit.  It is the
 * interlock that keeps an RX one-shot from clearing a pin that TX still owns:
 * the two are driven from different threads (radio TX path vs system work
 * queue), so without it a pulse landing mid-transmit would blank the LED for
 * the rest of the packet. Applies equally to the PWM-shared boards (T096,
 * Wireless Tracker V2): same race, same fix, just written through
 * tx_led_write() instead of a bare gpio_pin_set_dt(). */
static atomic_t s_tx_lit;
static struct k_work_delayable s_rx_pulse_off;

static void rx_pulse_off_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	/* Leave the pin alone if a transmit started while this pulse was in
	 * flight — onAfterTransmit() owns clearing it in that case. */
	if (atomic_get(&s_tx_lit)) {
		return;
	}
	tx_led_write(false);
#if ZEPHCORE_LED_PIN_SHARED
	zephcore_led_radio_hold_pin(false);
#endif
}

static int tx_led_init(void)
{
#if HAS_TX_LED_PWM
	if (device_is_ready(tx_led_pwm.dev)) {
		pwm_set_pulse_dt(&tx_led_pwm, 0);  /* start dark, same as GPIO_OUTPUT_INACTIVE */
	}
#else
	if (gpio_is_ready_dt(&tx_led)) {
		gpio_pin_configure_dt(&tx_led, GPIO_OUTPUT_INACTIVE);
	}
#endif
	k_work_init_delayable(&s_rx_pulse_off, rx_pulse_off_handler);
	return 0;
}
SYS_INIT(tx_led_init, APPLICATION, 90);
#endif

namespace mesh {

uint16_t ZephyrBoard::getBattMilliVolts()
{
#if HAS_FUEL_GAUGE
	if (device_is_ready(fuel_gauge_dev)) {
		union fuel_gauge_prop_val val;
		int ret = fuel_gauge_get_prop(fuel_gauge_dev, FUEL_GAUGE_VOLTAGE, &val);
		if (ret == 0) {
			return (uint16_t)(val.voltage / 1000);  /* µV -> mV */
		}
		LOG_WRN("Fuel gauge voltage read failed: %d", ret);
	}
	return 0;
#elif DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
    DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	if (!adc_is_ready_dt(&vbat_adc)) {
		LOG_ERR("ADC not ready");
		return 0;
	}

	if (!vbat_adc_configured) {
		int ret = adc_channel_setup_dt(&vbat_adc);
		if (ret < 0) {
			LOG_ERR("ADC channel setup failed: %d", ret);
			return 0;
		}
		vbat_adc_configured = true;
	}

	/* Enable battery ADC voltage divider (saves power when not reading) */
	if (vbat_enable_dev && device_is_ready(vbat_enable_dev)) {
		regulator_enable(vbat_enable_dev);
		k_msleep(10);  /* 10ms settling time for voltage divider + capacitor (matches Arduino) */
	}

	int32_t raw = 0;
	int valid_samples = 0;
	for (int i = 0; i < VBAT_ADC_SAMPLES; i++) {
		int16_t val = 0;  /* Use int16_t for 12-bit ADC */
		struct adc_sequence seq = {
			.buffer = &val,
			.buffer_size = sizeof(val),
		};
		int ret = adc_sequence_init_dt(&vbat_adc, &seq);
		if (ret < 0) {
			LOG_WRN("ADC sequence init failed: %d", ret);
			continue;
		}
		ret = adc_read_dt(&vbat_adc, &seq);
		if (ret == 0) {
			raw += val;
			valid_samples++;
		} else {
			LOG_WRN("ADC read failed: %d", ret);
		}
	}

	/* Disable battery ADC voltage divider to save power */
	if (vbat_enable_dev && device_is_ready(vbat_enable_dev)) {
		regulator_disable(vbat_enable_dev);
	}

	if (valid_samples == 0) {
		LOG_ERR("No valid ADC samples");
		return 0;
	}
	raw /= valid_samples;
	int64_t mult = (_adc_multiplier_override != 0.0f)
		? (int64_t)_adc_multiplier_override
		: (int64_t)VBAT_MV_MULTIPLIER;
	uint16_t mv = (uint16_t)((mult * (int64_t)raw) / 4096);
	LOG_DBG("Battery: raw=%d multiplier=%lld mv=%u", (int)raw, (long long)mult, mv);
	return mv;
#else
	return 0;
#endif
}

uint8_t ZephyrBoard::getBattPercent()
{
#if HAS_FUEL_GAUGE
	if (device_is_ready(fuel_gauge_dev)) {
		union fuel_gauge_prop_val val;
		int ret = fuel_gauge_get_prop(fuel_gauge_dev,
					      FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &val);
		if (ret == 0) {
			return val.relative_state_of_charge;
		}
		LOG_WRN("Fuel gauge SoC read failed: %d", ret);
	}
	/* Fall through to the voltage-curve estimate if the gauge read fails. */
#endif
	return battery_curve_lookup(&battery_curve_default, getBattMilliVolts());
}

bool ZephyrBoard::setAdcMultiplier(float multiplier)
{
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
    DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	_adc_multiplier_override = multiplier;
	return true;
#else
	(void)multiplier;
	return false;
#endif
}

float ZephyrBoard::getAdcMultiplier() const
{
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
    DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	return (_adc_multiplier_override != 0.0f)
		? _adc_multiplier_override
		: (float)VBAT_MV_MULTIPLIER;
#else
	return 0.0f;
#endif
}

float ZephyrBoard::getMCUTemperature()
{
	/* nRF52840 die temperature sensor - "nordic,nrf-temp" at 0x4000c000
	 * Nodelabel "temp" is defined in nrf52840.dtsi, status="okay" by default */
	const struct device *dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(temp));
	if (!dev || !device_is_ready(dev)) {
		return NAN;
	}
	struct sensor_value val;
	if (sensor_sample_fetch(dev) == 0 &&
	    sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &val) == 0) {
		return sensor_value_to_float(&val);
	}
	return NAN;
}

const char *ZephyrBoard::getManufacturerName() const
{
	return CONFIG_ZEPHCORE_BOARD_NAME;
}

void ZephyrBoard::onBeforeTransmit()
{
	/* Honour the LED master gate ("set leds off") and then the activity mode
	 * ("set leds.radio"). On a headless repeater this is the only LED that ever
	 * lights, so both have to be checked here and not just in the UI layer.
	 * onAfterTransmit() still clears the pin unconditionally, so a gate or mode
	 * flipped mid-transmit can't strand it lit. Same interlock on PWM-shared
	 * boards (T096, Wireless Tracker V2) as on plain-GPIO ones -- only the
	 * final write (tx_led_write()) differs. */
#if HAS_TX_LED_PWM || HAS_TX_LED
	uint8_t mode = zephcore_leds_radio_mode();
	if (!zephcore_leds_disabled() &&
	    (mode == LEDS_RADIO_TX || mode == LEDS_RADIO_ALL)) {
		/* A receive blink may still be in flight; take the pin from it so its
		 * handler doesn't clear the LED partway through this transmit. */
		k_work_cancel_delayable(&s_rx_pulse_off);
		atomic_set(&s_tx_lit, 1);
#if ZEPHCORE_LED_PIN_SHARED
		zephcore_led_radio_hold_pin(true);
#endif
		tx_led_write(true);
	}
#endif
}

void ZephyrBoard::onAfterTransmit()
{
#if HAS_TX_LED_PWM || HAS_TX_LED
	atomic_set(&s_tx_lit, 0);
	tx_led_write(false);
#if ZEPHCORE_LED_PIN_SHARED
	zephcore_led_radio_hold_pin(false);
#endif
#endif
}

void ZephyrBoard::onPacketReceived()
{
#if HAS_TX_LED_PWM || HAS_TX_LED
	uint8_t mode = zephcore_leds_radio_mode();
	if (zephcore_leds_disabled() ||
	    (mode != LEDS_RADIO_RX && mode != LEDS_RADIO_ALL)) {
		return;
	}
	/* Never interrupt a transmit that is holding the LED. Half-duplex makes
	 * this all but impossible in practice, but the two run on different
	 * threads and the cost of being wrong is an LED stuck dark for a whole
	 * packet. */
	if (atomic_get(&s_tx_lit)) {
		return;
	}
#if ZEPHCORE_LED_PIN_SHARED
	zephcore_led_radio_hold_pin(true);
#endif
	tx_led_write(true);
	/* Reschedule rather than schedule: back-to-back packets should extend the
	 * blink, not have the first one's handler cut the second one short. */
	k_work_reschedule(&s_rx_pulse_off, K_MSEC(RX_PULSE_MS));
#endif
}

void ZephyrBoard::reboot()
{
	k_msleep(50);  /* Let UART/USB flush */
#ifdef ZEPHCORE_USBD_DETACH
	/* USB device stack (CDC ACM): detach so the host sees an unplug. */
	zephcore_usbd_detach();
	k_msleep(100);  /* Hold SE0 long enough for host disconnect debounce */
#endif
#ifdef ESP32_USB_SERIAL_DETACH
	/* Drop the D+ pull-up so the host sees a clean USB disconnect before the
	 * soft reset (GH #43 — wedged serial port on macOS). */
	usb_serial_jtag_ll_phy_enable_pad(false);
	k_msleep(100);  /* Hold SE0 long enough for host disconnect debounce */
#endif
	sys_reboot(SYS_REBOOT_COLD);
}

void ZephyrBoard::rebootToBootloader()
{
#ifdef NRF52_GPREGRET
	/* Write magic value to GPREGRET0 - enter UF2 bootloader mode.
	 * UF2 supports both drag-and-drop (.uf2) and serial DFU (nrfutil). */
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
#endif
#ifdef ESP32_FORCE_DOWNLOAD_BOOT
	/* Come back up in the ROM download mode instead of the app.  Which USB
	 * controller it lands on is settled by the PHY mux below, after the
	 * detach. */
	REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
#endif
	k_msleep(50);  /* Let UART/USB flush */
#ifdef ZEPHCORE_USBD_DETACH
	/* Detach the CDC ACM device — same reason as reboot(), and doubly so here:
	 * the ESP32-S3 comes back in ROM download mode, where the port belongs to
	 * USB-Serial-JTAG.  The host must see the old device leave the bus first or
	 * it will not enumerate the new one. */
	zephcore_usbd_detach();
	k_msleep(100);
#endif
#ifdef ESP32_USB_SERIAL_DETACH
	/* Clean USB disconnect before the soft reset — same reason as reboot(). */
	usb_serial_jtag_ll_phy_enable_pad(false);
	k_msleep(100);
#endif
#ifdef ESP32_FORCE_DOWNLOAD_BOOT
	/* Hand the internal USB PHY back to USB-Serial-JTAG, so the ROM download
	 * mode we just armed appears as 303a:1001 rather than ROM USB OTG.  See
	 * the header comment on ESP32_FORCE_DOWNLOAD_BOOT for why this is needed
	 * and why it has to happen HERE -- after zephcore_usbd_detach(), which
	 * has to drop the D+ pull-up while OTG still owns the PHY, or the host
	 * never sees a clean disconnect.
	 *
	 * No-op on builds that never took the PHY (repeater/observer/debug keep
	 * USJ, so both bits are already 0). */
	REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL);
	REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_USB_PHY_SEL);
#endif
	sys_reboot(SYS_REBOOT_COLD);
}

bool ZephyrBoard::startOTAUpdate(const char *id, char reply[])
{
#ifdef NRF52_GPREGRET
	/* Write magic value to GPREGRET0 - enter BLE OTA DFU mode */
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_OTA_MAGIC);
	sprintf(reply, "OK - rebooting to BLE DFU (name: %s)", id ? id : "DfuTarg");
	k_msleep(50);  /* Let UART/USB flush */
	sys_reboot(SYS_REBOOT_COLD);
	return true;  /* Never reached */
#else
	(void)id;
	strcpy(reply, "Error: BLE OTA not supported on this platform");
	return false;
#endif
}

bool ZephyrBoard::getBootloaderVersion(char *out, size_t max_len)
{
#if defined(CONFIG_SOC_SERIES_NRF52)
	/* Scan flash for UF2 bootloader version string.
	 * info.txt lives somewhere in the 0xFB000-0xFE000 range depending
	 * on SoftDevice version and bootloader build. */
	static const char MARKER[] = "UF2 Bootloader ";
	const uint8_t *flash = (const uint8_t *)0x000FB000;

	for (uint32_t i = 0; i < 0x3000 - (sizeof(MARKER) - 1); i++) {
		if (memcmp(&flash[i], MARKER, sizeof(MARKER) - 1) == 0) {
			const char *ver = (const char *)&flash[i + sizeof(MARKER) - 1];
			size_t len = 0;
			while (len < max_len - 1 && ver[len] != '\0' &&
			       ver[len] != ' ' && ver[len] != '\n' && ver[len] != '\r') {
				out[len] = ver[len];
				len++;
			}
			out[len] = '\0';
			return len > 0;
		}
	}
#else
	(void)out;
	(void)max_len;
#endif
	return false;
}

void ZephyrBoard::clearBootloaderMagic()
{
#ifdef NRF52_GPREGRET
	/* Clear any stale GPREGRET values from previous sessions.
	 * GPREGRET0: bootloader DFU mode select (0x57=UF2, 0xA8=OTA)
	 * GPREGRET1: wake gate / deep sleep flag */
	nrf_power_gpregret_set(NRF_POWER, 0, 0x00);
	nrf_power_gpregret_set(NRF_POWER, 1, 0x00);
#endif
}

uint8_t ZephyrBoard::getStartupReason() const
{
	return BD_STARTUP_NORMAL;
}

bool ZephyrBoard::isExternalPowered()
{
#if defined(CONFIG_SOC_SERIES_NRF52)
	/* VBUS detect from the POWER peripheral — true when USB/charger is
	 * attached. Read-only status register; safe alongside the BLE controller
	 * (we already poke NRF_POWER->GPREGRET directly elsewhere). */
	return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
#else
	/* Other platforms have no portable VBUS query — report battery (false)
	 * so low-battery auto-shutdown is never inhibited. */
	return false;
#endif
}

} /* namespace mesh */
