/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ZephyrBoard.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>

#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
#include <hal/nrf_power.h>
/* Adafruit bootloader GPREGRET magic values */
#define BOOTLOADER_DFU_SERIAL_MAGIC 0x4e  /* Enter serial DFU mode (CDC only) */
#define BOOTLOADER_DFU_UF2_MAGIC    0x57  /* Enter UF2 mass storage mode (CDC + MSC) */
#define BOOTLOADER_DFU_OTA_MAGIC    0xA8  /* Enter BLE OTA DFU mode */
#define NRF52_GPREGRET 1
#endif

/* LoRa TX activity LED (optional — defined per-board via DT alias) */
#if DT_NODE_EXISTS(DT_ALIAS(lora_tx_led))
static const struct gpio_dt_spec tx_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(lora_tx_led), gpios);
#define HAS_TX_LED 1
#else
#define HAS_TX_LED 0
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

/* Initialize TX LED GPIO at boot */
#if HAS_TX_LED
static int tx_led_init(void)
{
	if (gpio_is_ready_dt(&tx_led)) {
		gpio_pin_configure_dt(&tx_led, GPIO_OUTPUT_INACTIVE);
	}
	return 0;
}
SYS_INIT(tx_led_init, APPLICATION, 90);
#endif

namespace mesh {

uint16_t ZephyrBoard::getBattMilliVolts()
{
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
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
#if HAS_TX_LED
	gpio_pin_set_dt(&tx_led, 1);
#endif
}

void ZephyrBoard::onAfterTransmit()
{
#if HAS_TX_LED
	gpio_pin_set_dt(&tx_led, 0);
#endif
}

void ZephyrBoard::reboot()
{
	k_msleep(50);  /* Let UART/USB flush */
	sys_reboot(SYS_REBOOT_COLD);
}

void ZephyrBoard::rebootToBootloader()
{
#ifdef NRF52_GPREGRET
	/* Write magic value to GPREGRET0 - enter UF2 bootloader mode.
	 * UF2 supports both drag-and-drop (.uf2) and serial DFU (nrfutil). */
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
#endif
	k_msleep(50);  /* Let UART/USB flush */
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
#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
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

} /* namespace mesh */
