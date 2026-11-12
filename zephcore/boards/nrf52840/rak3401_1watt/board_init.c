/*
 * RAK3401 1W — sleep a RAK4631's own SX1262 when this firmware runs on one.
 * SPDX-License-Identifier: MIT
 *
 * A RAK4631 core paired with a RAK13302 runs the RAK3401 firmware; the core's
 * onboard SX1262 (P1.05-P1.15) is then powered, never initialised, and idles
 * in STDBY_RC. At boot this resets it over its own pins, reads its version
 * string, and puts it in cold sleep, as upstream's sleepOnboardRadio()
 * (variants/rak3401, #3613). On a real RAK3401 those pins are unconnected:
 * BUSY never goes low and nothing is sent. SPI is bit-banged on GPIO, so the
 * RAK13302's SPI1 is not touched. NSS stays driven high afterwards: a falling
 * edge would wake the radio. devdocs/lld/10-peripherals-power.md.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <hal/nrf_gpio.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rak3401_onboard, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#define OB_ANT_SW  NRF_GPIO_PIN_MAP(1, 5)   /* powers the onboard RF switch */
#define OB_RESET   NRF_GPIO_PIN_MAP(1, 6)
#define OB_NSS     NRF_GPIO_PIN_MAP(1, 10)
#define OB_SCK     NRF_GPIO_PIN_MAP(1, 11)
#define OB_MOSI    NRF_GPIO_PIN_MAP(1, 12)
#define OB_MISO    NRF_GPIO_PIN_MAP(1, 13)
#define OB_BUSY    NRF_GPIO_PIN_MAP(1, 14)

#define SX126X_CMD_READ_REGISTER  0x1D
#define SX126X_CMD_SET_SLEEP      0x84
#define SX126X_SLEEP_COLD_RTC_OFF 0x00
#define SX126X_REG_VERSION_STRING 0x0320
#define SX126X_VERSION            "SX1261"  /* the SX1262 reports this prefix too */

#define BUSY_TIMEOUT_MS 20

static bool wait_busy(bool high)
{
	for (int ms = 0; ms < BUSY_TIMEOUT_MS; ms++) {
		if ((nrf_gpio_pin_read(OB_BUSY) != 0) == high) {
			return true;
		}
		k_busy_wait(1000);
	}
	return (nrf_gpio_pin_read(OB_BUSY) != 0) == high;
}

/* SPI mode 0, MSB first, ~100 kHz. */
static uint8_t spi_byte(uint8_t out)
{
	uint8_t in = 0;

	for (int bit = 7; bit >= 0; bit--) {
		nrf_gpio_pin_write(OB_MOSI, (out >> bit) & 1U);
		k_busy_wait(5);
		nrf_gpio_pin_set(OB_SCK);
		in = (uint8_t)((in << 1) | (nrf_gpio_pin_read(OB_MISO) != 0 ? 1U : 0U));
		k_busy_wait(5);
		nrf_gpio_pin_clear(OB_SCK);
	}
	return in;
}

static void read_version(char *version)
{
	nrf_gpio_pin_clear(OB_NSS);
	spi_byte(SX126X_CMD_READ_REGISTER);
	spi_byte(SX126X_REG_VERSION_STRING >> 8);
	spi_byte(SX126X_REG_VERSION_STRING & 0xFF);
	spi_byte(0);  /* status */
	for (int i = 0; i < 6; i++) {
		version[i] = (char)spi_byte(0);
	}
	nrf_gpio_pin_set(OB_NSS);
}

static const char *onboard_sleep(void)
{
	char version[6];

	nrf_gpio_pin_set(OB_NSS);
	nrf_gpio_cfg_output(OB_NSS);
	nrf_gpio_pin_set(OB_RESET);
	nrf_gpio_cfg_output(OB_RESET);
	nrf_gpio_pin_clear(OB_ANT_SW);
	nrf_gpio_cfg_output(OB_ANT_SW);
	nrf_gpio_pin_clear(OB_SCK);
	nrf_gpio_cfg_output(OB_SCK);
	nrf_gpio_pin_clear(OB_MOSI);
	nrf_gpio_cfg_output(OB_MOSI);
	/* Pulled so an absent chip reads BUSY high and MISO low. */
	nrf_gpio_cfg_input(OB_BUSY, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(OB_MISO, NRF_GPIO_PIN_PULLDOWN);

	nrf_gpio_pin_clear(OB_RESET);
	k_busy_wait(1000);
	nrf_gpio_pin_set(OB_RESET);
	k_busy_wait(5000);

	if (!wait_busy(false)) {
		return "absent";  /* BUSY never released: no chip */
	}
	read_version(version);
	if (memcmp(version, SX126X_VERSION, sizeof(version)) != 0) {
		return "absent";
	}
	nrf_gpio_cfg_input(OB_BUSY, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(OB_MISO, NRF_GPIO_PIN_NOPULL);
	if (!wait_busy(false)) {
		return "busy-timeout";
	}

	nrf_gpio_pin_clear(OB_NSS);
	spi_byte(SX126X_CMD_SET_SLEEP);
	spi_byte(SX126X_SLEEP_COLD_RTC_OFF);
	nrf_gpio_pin_set(OB_NSS);
	k_busy_wait(1000);

	/* BUSY is high while asleep; SPI here would wake it. */
	return wait_busy(true) ? "asleep" : "sleep-failed";
}

static int rak3401_onboard_radio_init(void)
{
	const char *outcome = onboard_sleep();

	/* Back to the reset default (input buffer disconnected); NSS, RESET and
	 * ANT_SW stay driven. */
	nrf_gpio_cfg_default(OB_SCK);
	nrf_gpio_cfg_default(OB_MOSI);
	nrf_gpio_cfg_default(OB_MISO);
	nrf_gpio_cfg_default(OB_BUSY);
	if (strcmp(outcome, "absent") == 0) {
		/* A real RAK3401: release the pins too. */
		nrf_gpio_cfg_default(OB_NSS);
		nrf_gpio_cfg_default(OB_RESET);
		nrf_gpio_cfg_default(OB_ANT_SW);
		LOG_DBG("onboard SX1262: absent");
	} else {
		LOG_INF("onboard SX1262 (RAK4631 core): %s", outcome);
	}
	return 0;
}

SYS_INIT(rak3401_onboard_radio_init, POST_KERNEL, 0);
