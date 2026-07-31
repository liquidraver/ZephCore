/*
 * SPDX-License-Identifier: MIT
 * Minimal LR11xx HAL for firmware updater.
 *
 * Implements the Semtech lr11xx_hal_* interface used by lr1110_bootloader.c.
 * Stripped down: no DIO1 interrupt, no work queue, no sleep tracking.
 * Just SPI + GPIO for bootloader commands.
 */

#include "lr11xx_hal_updater.h"
#include "lr11xx_hal.h"

#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr1110_hal, LOG_LEVEL_INF);

/* ── Hardware from devicetree ──────────────────────────────── */

#define LR1110_NODE DT_NODELABEL(lora)

#if !DT_NODE_EXISTS(LR1110_NODE)
#error "No 'lora' node found in devicetree — is this an LR1110 board?"
#endif

/* SPI bus device */
static const struct device *spi_dev = DEVICE_DT_GET(DT_BUS(LR1110_NODE));

/* Flashing SPI clock cap.
 *
 * In bootloader mode the chip runs off its internal RC oscillator (the XOSC
 * needs either a crystal or a powered TCXO), so it has far less timing
 * margin than during normal operation. Flashing pushes ~1000 back-to-back
 * 256-byte writes, and the images are encrypted+signed: a SINGLE corrupted
 * byte anywhere makes the whole image fail its integrity check at boot,
 * with every write still reporting OK (the bootloader never reads back).
 * Cap the operational 8-16 MHz down to a conservative rate — the entire
 * 239 KB image still takes ~1 s of SPI time at 2 MHz. */
#define UPDATER_SPI_MAX_HZ 2000000

/* SPI config — manual CS (we toggle NSS via GPIO) */
static struct spi_config spi_cfg = {
	.frequency = MIN(DT_PROP(LR1110_NODE, spi_max_frequency), UPDATER_SPI_MAX_HZ),
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
};

/* TCXO configuration from devicetree (0 mV = crystal board, no TCXO) */
uint16_t lr1110_updater_tcxo_voltage_mv(void)
{
	return DT_PROP_OR(LR1110_NODE, tcxo_voltage_mv, 0);
}

uint32_t lr1110_updater_tcxo_startup_delay_ms(void)
{
	return DT_PROP_OR(LR1110_NODE, tcxo_startup_delay_ms, 5);
}

/* GPIO pins */
static const struct gpio_dt_spec pin_nss   = GPIO_DT_SPEC_GET(DT_BUS(LR1110_NODE), cs_gpios);
static const struct gpio_dt_spec pin_reset = GPIO_DT_SPEC_GET(LR1110_NODE, reset_gpios);
static const struct gpio_dt_spec pin_busy  = GPIO_DT_SPEC_GET(LR1110_NODE, busy_gpios);

/* BUSY timeout — 3 seconds (flash erase can take ~2.5s) */
#define BUSY_TIMEOUT_MS 3000

/* Extended BUSY timeout for flash erase (0x8000) and for the loader's
 * bootloader rewrite (0x8100) — both keep BUSY high for seconds. */
#define LONG_BUSY_TIMEOUT_MS 10000

/* How long to watch for BUSY to RISE after a command before assuming the
 * chip finished it too quickly to observe. See wait_command_complete(). */
#define BUSY_RISE_TIMEOUT_MS 2

/* Largest command+payload frame we ever put on the wire. */
#define LR11XX_HAL_MAX_FRAME 272

/* ── Context (opaque pointer for Semtech driver) ──────────── */

/* The Semtech driver passes 'context' to every HAL function.
 * We use a dummy static — all state is in file-scope globals. */
static int dummy_context;

void *lr1110_updater_get_context(void)
{
	return &dummy_context;
}

/* ── BUSY wait ────────────────────────────────────────────── */

static int wait_on_busy(uint32_t timeout_ms)
{
	int64_t start = k_uptime_get();

	while (gpio_pin_get_dt(&pin_busy)) {
		if ((k_uptime_get() - start) > timeout_ms) {
			printk("ERROR: BUSY timeout after %u ms\n", timeout_ms);
			return -ETIMEDOUT;
		}
		k_busy_wait(100); /* 100us */
	}
	return 0;
}

/*
 * Wait for a command the chip has just been given to actually COMPLETE.
 *
 * The chip does not raise BUSY the instant NSS deasserts — it needs a few
 * microseconds. Polling only for "BUSY is low" therefore has a race: on a
 * fast host (ESP32-S3 at 240 MHz drives GPIO in nanoseconds) the poll can
 * observe the *stale* pre-command LOW and conclude the command is already
 * finished. The next transaction then starts clocking while the chip is
 * still writing flash, and because WriteFlashEncrypted is fire-and-forget
 * — no read-back, no status check — the resulting corruption is silent.
 * One bad chunk anywhere invalidates the whole signed image, so the odds
 * of a clean flash fall off a cliff as the image grows: a 19 KB loader is
 * 77 transactions, a 239 KB firmware is 959.
 *
 * So: first watch for the rising edge (bounded — a command that finishes
 * faster than we can look is fine and simply never appears busy), then
 * wait for the fall.
 */
/* ── Per-command instrumentation ───────────────────────────
 *
 * Captured for the most recent command so the caller can trace every chunk:
 * how long the chip took to ASSERT busy (rise latency) and how long it then
 * held it (the real flash-program time). A chunk that never asserts busy at
 * all is the signature of the race this HAL exists to avoid — worth seeing
 * per chunk rather than inferring from a summary. */
static uint32_t last_busy_rise_us;
static uint32_t last_busy_hold_us;
static bool     last_busy_seen;
static int      last_spi_ret;

uint32_t lr1110_updater_last_busy_rise_us(void) { return last_busy_rise_us; }
uint32_t lr1110_updater_last_busy_hold_us(void) { return last_busy_hold_us; }
bool     lr1110_updater_last_busy_seen(void)    { return last_busy_seen; }
int      lr1110_updater_last_spi_ret(void)      { return last_spi_ret; }

static int wait_command_complete(uint32_t timeout_ms)
{
	const uint32_t cyc_entry = k_cycle_get_32();
	int64_t start = k_uptime_get();

	last_busy_seen    = false;
	last_busy_rise_us = 0;
	last_busy_hold_us = 0;

	while (!gpio_pin_get_dt(&pin_busy)) {
		if ((k_uptime_get() - start) > BUSY_RISE_TIMEOUT_MS) {
			break; /* never went busy — nothing to wait for */
		}
		k_busy_wait(1);
	}

	if (gpio_pin_get_dt(&pin_busy)) {
		last_busy_seen = true;
		last_busy_rise_us =
			k_cyc_to_us_floor32(k_cycle_get_32() - cyc_entry);
	}

	const uint32_t cyc_high = k_cycle_get_32();
	int ret = wait_on_busy(timeout_ms);
	last_busy_hold_us = k_cyc_to_us_floor32(k_cycle_get_32() - cyc_high);

	return ret;
}

/* ── Public init/reset ────────────────────────────────────── */

int lr1110_updater_hal_init(void)
{
	int ret;

	if (!device_is_ready(spi_dev)) {
		printk("ERROR: SPI device not ready\n");
		return -ENODEV;
	}

	/* NSS — output, inactive (HIGH = deselected) */
	ret = gpio_pin_configure_dt(&pin_nss, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("ERROR: NSS config failed: %d\n", ret);
		return ret;
	}

	/* RESET — output, inactive (HIGH = not in reset) */
	ret = gpio_pin_configure_dt(&pin_reset, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("ERROR: RESET config failed: %d\n", ret);
		return ret;
	}

	/* BUSY — input */
	ret = gpio_pin_configure_dt(&pin_busy, GPIO_INPUT);
	if (ret < 0) {
		printk("ERROR: BUSY config failed: %d\n", ret);
		return ret;
	}

	printk("LR1110 HAL initialized (SPI @ %u Hz)\n", spi_cfg.frequency);
	return 0;
}

int lr1110_updater_hw_reset(void)
{
	printk("Resetting LR1110...\n");

	/* Assert reset (active-low: logical 1 = physical LOW = reset active) */
	gpio_pin_set_dt(&pin_reset, 1);
	k_msleep(10);

	/* Release reset */
	gpio_pin_set_dt(&pin_reset, 0);

	/* After reset, the LR1110 boots into bootloader if flash is empty,
	 * or into firmware if flash has valid content.
	 * Firmware boot takes up to 273ms (datasheet). */
	k_msleep(300);

	int ret = wait_on_busy(BUSY_TIMEOUT_MS);
	if (ret) {
		printk("ERROR: BUSY stuck after reset\n");
	} else {
		printk("LR1110 reset complete, BUSY=low\n");
	}
	return ret;
}

int lr1110_updater_reset_to_bootloader(void)
{
	printk("Resetting LR1110 into bootloader (BUSY held LOW)...\n");

	/* Semtech lr1110_updater_tool pattern:
	 * 1. Drive BUSY LOW as output during reset
	 * 2. Pulse RESET
	 * 3. Wait 500ms
	 * 4. Release BUSY back to input
	 * 5. Wait 100ms + BUSY low
	 *
	 * When BUSY is held LOW by the host during reset, the LR1110
	 * enters bootloader mode instead of executing flash firmware. */

	/* Drive BUSY to physical LOW (pin_busy has GPIO_ACTIVE_HIGH,
	 * so we use raw GPIO to be explicit about physical level) */
	gpio_pin_configure(pin_busy.port, pin_busy.pin,
			   GPIO_OUTPUT_LOW);

	/* Assert reset */
	gpio_pin_set_dt(&pin_reset, 1);
	k_msleep(10);

	/* Release reset — chip starts booting, sees BUSY held LOW → bootloader */
	gpio_pin_set_dt(&pin_reset, 0);
	k_msleep(500);

	/* Release BUSY back to input */
	gpio_pin_configure_dt(&pin_busy, GPIO_INPUT);
	k_msleep(100);

	int ret = wait_on_busy(BUSY_TIMEOUT_MS);
	if (ret) {
		printk("ERROR: BUSY stuck after bootloader reset\n");
	} else {
		printk("LR1110 in bootloader mode, BUSY=low\n");
	}
	return ret;
}

/* ── SD card presence probe ───────────────────────────────────
 *
 * On boards where the SD slot shares the radio's SPI bus, an inserted card
 * breaks LR1110 flashing: every write still reports OK and the programmed
 * image fails its integrity check at boot, which is indistinguishable from a
 * dozen other faults and cost days to track down. There is no card-detect pin
 * wired on the M9, so presence is established over the bus.
 *
 * Standard SPI-mode detection: >=74 dummy clocks with CS high to bring the
 * card up in SPI mode, then CMD0 (GO_IDLE_STATE). A present card answers R1
 * with the MSB clear (0x01 = idle). An empty slot leaves MISO pulled high, so
 * every byte reads 0xFF and nothing else. False "absent" is possible if a card
 * ignores CMD0 — no worse than not probing; false "present" essentially cannot
 * happen, since 0xFF is all an empty slot can produce.
 *
 * Runs at 400 kHz (SD init is specified at 100-400 kHz; the radio path stays
 * at its own clock) and is safe on the shared bus: the LR1110's NSS is parked
 * inactive by hal_init, and the TFT is held in reset and is write-only.
 */
#if defined(CONFIG_BOARD_THINKNODE_M9)
/* No DT node exists for the slot — the base board DTS only parks its CS with
 * a gpio-hog. GPIO48 = gpio1 pin 16. */
#define SDCARD_CS_PORT_NODE DT_NODELABEL(gpio1)
#define SDCARD_CS_PIN       16
#endif

int lr1110_updater_probe_sdcard(void)
{
#ifdef SDCARD_CS_PORT_NODE
	const struct device *cs_port = DEVICE_DT_GET(SDCARD_CS_PORT_NODE);
	struct spi_config slow_cfg = {
		.frequency = 400000,
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	};
	/* CMD0: GO_IDLE_STATE, arg 0, CRC7 0x95 (valid, and required while the
	 * card is still in its CRC-checked power-up state). */
	static const uint8_t cmd0[6] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x95 };
	uint8_t tx[8], rx[8];
	int ret, present = 0;

	if (!device_is_ready(cs_port)) {
		return -ENODEV;
	}
	if (gpio_pin_configure(cs_port, SDCARD_CS_PIN, GPIO_OUTPUT_HIGH) < 0) {
		return -EIO;
	}

	/* Wake-up clocks, CS HIGH (deselected) — 10 bytes = 80 cycles. */
	memset(tx, 0xFF, sizeof(tx));
	const struct spi_buf wake_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set wake = { .buffers = &wake_buf, .count = 1 };

	ret = spi_write(spi_dev, &slow_cfg, &wake);
	if (ret == 0) {
		ret = spi_write(spi_dev, &slow_cfg, &wake); /* >=74 clocks total */
	}
	if (ret < 0) {
		goto out;
	}

	/* Select the card and issue CMD0. */
	gpio_pin_set(cs_port, SDCARD_CS_PIN, 0);

	const struct spi_buf cmd_buf = { .buf = (uint8_t *)cmd0, .len = sizeof(cmd0) };
	const struct spi_buf_set cmd = { .buffers = &cmd_buf, .count = 1 };

	ret = spi_write(spi_dev, &slow_cfg, &cmd);
	if (ret == 0) {
		/* Clock the response out with MOSI held high, as the spec
		 * requires — spi_read() would drive zeros instead. */
		memset(tx, 0xFF, sizeof(tx));
		const struct spi_buf rtx_buf = { .buf = tx, .len = sizeof(rx) };
		const struct spi_buf_set rtx = { .buffers = &rtx_buf, .count = 1 };
		const struct spi_buf rrx_buf = { .buf = rx, .len = sizeof(rx) };
		const struct spi_buf_set rrx = { .buffers = &rrx_buf, .count = 1 };

		ret = spi_transceive(spi_dev, &slow_cfg, &rtx, &rrx);
		if (ret == 0) {
			for (size_t i = 0; i < sizeof(rx); i++) {
				if ((rx[i] & 0x80) == 0) { /* valid R1 token */
					present = 1;
					break;
				}
			}
		}
	}

	/* Deselect, then one more byte so the card releases the bus. */
	gpio_pin_set(cs_port, SDCARD_CS_PIN, 1);
	memset(tx, 0xFF, sizeof(tx));
	const struct spi_buf rel_buf = { .buf = tx, .len = 1 };
	const struct spi_buf_set rel = { .buffers = &rel_buf, .count = 1 };

	spi_write(spi_dev, &slow_cfg, &rel);

out:
	/* Leave CS parked HIGH exactly as the gpio-hog had it. */
	gpio_pin_configure(cs_port, SDCARD_CS_PIN, GPIO_OUTPUT_HIGH);

	return (ret < 0) ? ret : present;
#else
	return -ENOTSUP;
#endif
}

/* ── Semtech HAL interface ────────────────────────────────── */

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
				     const uint16_t command_length,
				     const uint8_t *data, const uint16_t data_length)
{
	(void)context;
	int ret;

	/* Wait for device ready */
	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* Send the command and its payload as ONE contiguous buffer.
	 *
	 * Passing them as two spi_bufs makes Zephyr's ESP32 SPI driver walk
	 * the set buffer-by-buffer (spi_context_max_continuous_chunk() never
	 * spans a buffer boundary), so a 6-byte command and a 256-byte payload
	 * become separate hardware transactions — and each is further split at
	 * SOC_SPI_MAXIMUM_BUFFER_SIZE (64 bytes on the S3, no DMA).
	 *
	 * That matters here because WriteFlashEncrypted is the ONLY command
	 * this tool issues with a payload: every command known to work
	 * (GetVersion, GetStatus, EraseFlash, the EUI reads) is a single
	 * sub-64-byte frame. Keeping the frame contiguous — together with a
	 * flash chunk size chosen so command+payload stays under 64 bytes —
	 * makes the write path look exactly like the paths already proven
	 * good on this hardware. */
	static uint8_t txbuf[LR11XX_HAL_MAX_FRAME];

	if ((size_t)command_length + (size_t)data_length > sizeof(txbuf)) {
		printk("ERROR: SPI frame too large (%u + %u)\n",
		       command_length, data_length);
		return LR11XX_HAL_STATUS_ERROR;
	}

	memcpy(txbuf, command, command_length);
	if (data_length > 0) {
		memcpy(txbuf + command_length, data, data_length);
	}

	const struct spi_buf tx_buf = {
		.buf = txbuf,
		.len = (size_t)command_length + (size_t)data_length,
	};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	gpio_pin_set_dt(&pin_nss, 1); /* Assert NSS (LOW) */
	ret = spi_write(spi_dev, &spi_cfg, &tx);
	gpio_pin_set_dt(&pin_nss, 0); /* Deassert NSS (HIGH) */

	last_spi_ret = ret;

	if (ret < 0) {
		printk("ERROR: SPI write failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* Flash erase (0x8000) holds BUSY for ~2.5s; the loader's bootloader
	 * rewrite (0x8100) also runs for seconds — give both the long timeout. */
	uint16_t opcode = 0;
	if (command_length >= 2) {
		opcode = ((uint16_t)command[0] << 8) | command[1];
	}
	uint32_t timeout = (opcode == 0x8000 || opcode == 0x8100)
		? LONG_BUSY_TIMEOUT_MS : BUSY_TIMEOUT_MS;

	/* Rising-edge aware: the command must be seen through to completion,
	 * not merely observed to be "not busy yet". */
	if (wait_command_complete(timeout)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length,
				    uint8_t *data, const uint16_t data_length)
{
	(void)context;
	int ret;

	/* Wait for device ready */
	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* Step 1: Write command */
	const struct spi_buf tx_buf = { .buf = (uint8_t *)command, .len = command_length };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	gpio_pin_set_dt(&pin_nss, 1);
	ret = spi_write(spi_dev, &spi_cfg, &tx);
	gpio_pin_set_dt(&pin_nss, 0);

	if (ret < 0) {
		printk("ERROR: SPI write (cmd) failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (data_length == 0) {
		return (wait_command_complete(BUSY_TIMEOUT_MS) == 0)
			? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
	}

	/* Step 2: Wait for the command to complete, then read the response */
	if (wait_command_complete(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* LR11xx returns 1 dummy byte + data */
	uint8_t dummy;
	const struct spi_buf rx_bufs[] = {
		{ .buf = &dummy, .len = 1 },
		{ .buf = data, .len = data_length },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

	gpio_pin_set_dt(&pin_nss, 1);
	ret = spi_read(spi_dev, &spi_cfg, &rx);
	gpio_pin_set_dt(&pin_nss, 0);

	if (ret < 0) {
		printk("ERROR: SPI read failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
					   const uint16_t data_length)
{
	(void)context;
	int ret;

	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf rx_buf = { .buf = data, .len = data_length };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	gpio_pin_set_dt(&pin_nss, 1);
	ret = spi_read(spi_dev, &spi_cfg, &rx);
	gpio_pin_set_dt(&pin_nss, 0);

	if (ret < 0) {
		printk("ERROR: SPI direct read failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{
	(void)context;
	return (lr1110_updater_hw_reset() == 0)
		? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
	(void)context;
	return (wait_on_busy(BUSY_TIMEOUT_MS) == 0)
		? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}
