/*
 * ThinkNode M9 — quiesce an inserted SD card before the radio comes up.
 * SPDX-License-Identifier: MIT
 *
 * The SD slot shares SPI2 with the LR1110 and the ST7789. Its CS is parked
 * HIGH by the `sd-cs-park` gpio-hog in the base DTS, which is enough for a
 * card that is powered and has been initialised: it tri-states its data-out
 * while deselected and ignores the bus.
 *
 * A card that has never been addressed is not in the second of those states: it
 * comes up in native SD mode, not SPI mode, so it has not yet been told to
 * ignore traffic it was not addressed for. Until something runs the SPI-mode
 * entry sequence, its behaviour on a busy shared bus is simply not defined by
 * the spec.
 *
 * The card is powered normally whenever the firmware runs, so this is NOT the
 * parasitic-supply trap that VEXT once was for the LR1110. Confirmed on the
 * V1.0 schematic: SD_3V3 comes off VDD_PERIPH through series R86, with no
 * enable of its own, and VDD_PERIPH is the MOSFET-gated rail that GPIO18
 * (VDD_PERIPH_EN) switches — asserted at boot by the tft_pwr_enable regulator.
 *
 * Worth knowing when debugging this board: VDD_PERIPH fans out through one
 * series resistor per consumer — R82 to VDD_RADIO, R86 to SD_3V3, R88 to
 * DISPLAY_VDD, R85 to RTC_3V3. The radio and the SD slot therefore share a
 * single gated rail. If an inserted card ever turns out to disturb the radio
 * in a way this quiesce does not fix, supply sag under combined load is the
 * next suspect, and no amount of bus discipline will address it.
 *
 * An inserted card was field-confirmed to break LR1110 flash programming on
 * this board: every write reported OK, the chip programmed for real, and the
 * image still failed its integrity check at boot with no error reported
 * anywhere. This runs the standard SPI-mode entry sequence once at startup so
 * any card present lands in SPI idle — a defined, quiet state where it ignores
 * everything until selected — rather than whatever it powered up into.
 *
 * Runs at POST_KERNEL priority 70: after the SPI bus (KERNEL_INIT_PRIORITY_DEVICE
 * = 50) and GPIO, before the LoRa driver (CONFIG_LORA_INIT_PRIORITY = 90), so
 * the card is quiet before any radio traffic exists.
 *
 * Best-effort by design. Every failure path leaves CS parked HIGH exactly as
 * the hog had it and returns success — a card we could not talk to is no worse
 * off than one we never tried, and refusing to boot over it would be far worse
 * than the problem.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m9_sd_quiesce, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* SD CS is GPIO48 = gpio1 pin 16. No DT node describes the slot — the base
 * board DTS only parks this pin with a gpio-hog. */
#define SD_CS_PORT DT_NODELABEL(gpio1)
#define SD_CS_PIN  16

/* The radio's bus. Taken from the lora node so this cannot drift apart from
 * the devicetree if SPI2 is ever renumbered. */
#define SD_SPI_BUS DT_BUS(DT_NODELABEL(lora))

static int m9_sd_quiesce(void)
{
	const struct device *spi = DEVICE_DT_GET(SD_SPI_BUS);
	const struct device *cs_port = DEVICE_DT_GET(SD_CS_PORT);
	/* SD initialisation is specified at 100-400 kHz; the radio keeps its
	 * own clock, this config is local to these few transfers. */
	struct spi_config cfg = {
		.frequency = 400000,
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	};
	/* CMD0 GO_IDLE_STATE, arg 0, CRC7 0x95 — CRC is still checked in the
	 * card's power-up state, so it must be correct. */
	static const uint8_t cmd0[6] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x95 };
	uint8_t tx[8], rx[8];

	if (!device_is_ready(spi) || !device_is_ready(cs_port)) {
		return 0;
	}
	if (gpio_pin_configure(cs_port, SD_CS_PIN, GPIO_OUTPUT_HIGH) < 0) {
		return 0;
	}

	/* >=74 clocks with CS HIGH — the SD power-up requirement. */
	memset(tx, 0xFF, sizeof(tx));
	const struct spi_buf wake_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set wake = { .buffers = &wake_buf, .count = 1 };

	if (spi_write(spi, &cfg, &wake) < 0 || spi_write(spi, &cfg, &wake) < 0) {
		goto park;
	}

	/* Select, send CMD0, read the reply. Response bytes are clocked with
	 * MOSI held high as the spec requires — spi_read() would drive zeros. */
	gpio_pin_set(cs_port, SD_CS_PIN, 0);

	const struct spi_buf cmd_buf = { .buf = (uint8_t *)cmd0, .len = sizeof(cmd0) };
	const struct spi_buf_set cmd = { .buffers = &cmd_buf, .count = 1 };

	if (spi_write(spi, &cfg, &cmd) == 0) {
		memset(tx, 0xFF, sizeof(tx));
		const struct spi_buf rtx_buf = { .buf = tx, .len = sizeof(rx) };
		const struct spi_buf_set rtx = { .buffers = &rtx_buf, .count = 1 };
		const struct spi_buf rrx_buf = { .buf = rx, .len = sizeof(rx) };
		const struct spi_buf_set rrx = { .buffers = &rrx_buf, .count = 1 };

		if (spi_transceive(spi, &cfg, &rtx, &rrx) == 0) {
			for (size_t i = 0; i < sizeof(rx); i++) {
				if ((rx[i] & 0x80) == 0) { /* valid R1 token */
					LOG_INF("SD card present (R1=0x%02X) — "
						"put into SPI idle", rx[i]);
					goto release;
				}
			}
		}
	}
	LOG_DBG("No SD card detected");

release:
	/* Deselect, then one more byte so the card releases the bus. */
	gpio_pin_set(cs_port, SD_CS_PIN, 1);
	memset(tx, 0xFF, sizeof(tx));
	const struct spi_buf rel_buf = { .buf = tx, .len = 1 };
	const struct spi_buf_set rel = { .buffers = &rel_buf, .count = 1 };

	spi_write(spi, &cfg, &rel);

park:
	/* Restore the hog's parked state. */
	gpio_pin_configure(cs_port, SD_CS_PIN, GPIO_OUTPUT_HIGH);
	return 0;
}

SYS_INIT(m9_sd_quiesce, POST_KERNEL, 70);
