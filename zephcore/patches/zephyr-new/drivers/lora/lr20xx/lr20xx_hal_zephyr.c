/*
 * SPDX-License-Identifier: MIT
 * LR20xx HAL implementation for Zephyr - ZephCore
 *
 * Based on Semtech SWDR001 lr20xx_driver and the ZephCore lr11xx HAL.
 */

#include "lr20xx_hal_zephyr.h"
#include "lr20xx_hal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr20xx_hal, CONFIG_LORA_LOG_LEVEL);

/* BUSY timeout; covers worst-case post-reset firmware boot (~300ms) with margin */
#define LR20XX_BUSY_TIMEOUT_MS  3000

/* This HAL is single-instance: everything below is file-scope, not per-context.
 * The driver instantiates with DT_INST_FOREACH_STATUS_OKAY, so a second
 * semtech,lr2021 node would silently share the DIO1 callback, the BUSY
 * callback and the BUSY semaphore between two radios — the second init would
 * overwrite the first's callback and both would wait on one semaphore.  No
 * board does this today; the assert makes the assumption fail loudly at build
 * time instead of mysteriously at run time.  The fix, if a board ever needs
 * two, is to move these into struct lr20xx_hal_context. */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(semtech_lr2021) <= 1,
	     "lr20xx_hal_zephyr.c holds its DIO1/BUSY state in file-scope "
	     "statics — it supports exactly one LR20xx instance");

/* Static state for DIO1 interrupt handling */
static struct gpio_callback dio1_gpio_cb;
static lr20xx_dio1_callback_t dio1_user_cb = NULL;
static void *dio1_user_data = NULL;

/* BUSY pin interrupt — wakes wait_on_busy() via semaphore instead of polling */
static struct gpio_callback busy_gpio_cb;
static K_SEM_DEFINE(busy_sem, 0, 1);

static void busy_isr_callback(const struct device *dev, struct gpio_callback *cb,
			       uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_sem_give(&busy_sem);
}

/* Track the last SPI opcode for debugging BUSY stuck */
static uint16_t last_opcode;
static int64_t last_cmd_time;

/**
 * @brief Wait until BUSY deasserts, using GPIO IRQ + semaphore (not polling).
 * Fast-path returns immediately if already idle.
 */
static lr20xx_hal_status_t wait_on_busy(struct lr20xx_hal_context *ctx)
{
	if (!gpio_pin_get_dt(&ctx->busy)) {
		return LR20XX_HAL_STATUS_OK;
	}

	k_sem_reset(&busy_sem);
	gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_EDGE_TO_INACTIVE);

	/* Re-check after arming IRQ to close the race: BUSY may have dropped
	 * between the first read and the interrupt enable. */
	if (!gpio_pin_get_dt(&ctx->busy)) {
		gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_DISABLE);
		return LR20XX_HAL_STATUS_OK;
	}

	int ret = k_sem_take(&busy_sem, K_MSEC(LR20XX_BUSY_TIMEOUT_MS));
	gpio_pin_interrupt_configure_dt(&ctx->busy, GPIO_INT_DISABLE);

	if (ret == -EAGAIN) {
		LOG_ERR("BUSY timeout! last_op=0x%04x sent_at=%lld (%lld ms ago) DIO1=%d",
			last_opcode, last_cmd_time,
			k_uptime_get() - last_cmd_time,
			gpio_pin_get_dt(&ctx->dio1));
		return LR20XX_HAL_STATUS_ERROR;
	}

	return LR20XX_HAL_STATUS_OK;
}

/**
 * @brief Assert ready; if sleeping, issue NSS wake pulse first.
 */
static lr20xx_hal_status_t check_device_ready(struct lr20xx_hal_context *ctx)
{
	if (!ctx->radio_is_sleeping) {
		/* Always wait — never second-guess a high BUSY.
		 *
		 * There used to be an "if a duty cycle is armed and BUSY is high
		 * the chip must have self-slept, so wake it" short-circuit here.
		 * It could not tell self-sleep apart from a command still running,
		 * and BUSY is high for both: CAD holds it for the CAD duration,
		 * SetTx until the PA has ramped, SetRxDutyCycle and the FE
		 * calibration for their own spans.  So after every slow command
		 * the next one skipped the wait and fired an NSS pulse instead —
		 * a frame with no clock cycles, which the chip reads as a
		 * malformed command (CMD_PERR, DS Table 6-38) and which latches
		 * CmdError.  That is the rejection storm and reset loop seen on
		 * the MeshTracker X1 whenever rxduty was on, and why rxduty off
		 * always worked.
		 *
		 * Waiting costs nothing worth saving: a duty-cycle sleep window is
		 * bounded by cycle_time - rx_max_time (25 ms at the presets we
		 * use) and the chip drops BUSY by itself on waking into its RX
		 * window.  There is no timeout to burn.  The LR11xx driver has
		 * never had such a branch, for the same reason. */
		return wait_on_busy(ctx);
	}

	/* Wake from sleep: the chip leaves Sleep when NSS is held low for 100us
	 * (datasheet, Sleep mode). Semtech's reference HAL allows 1 ms; keep that
	 * margin — a short pulse leaves the radio asleep and every following
	 * command is answered by a chip that is not listening. */
	gpio_pin_set_dt(&ctx->nss, 1);
	k_busy_wait(1000);
	gpio_pin_set_dt(&ctx->nss, 0);

	ctx->radio_is_sleeping = false;
	return wait_on_busy(ctx);
}

static void dio1_isr_callback(const struct device *dev, struct gpio_callback *cb,
			       uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (dio1_user_cb) {
		dio1_user_cb(dio1_user_data);
	}
}

int lr20xx_hal_init(struct lr20xx_hal_context *ctx)
{
	int ret;

	ctx->radio_is_sleeping = false;

	ret = gpio_pin_configure_dt(&ctx->nss, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure NSS: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&ctx->reset, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure RESET: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&ctx->busy, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure BUSY: %d", ret);
		return ret;
	}

	/* BUSY IRQ enabled on-demand by wait_on_busy() */
	gpio_init_callback(&busy_gpio_cb, busy_isr_callback, BIT(ctx->busy.pin));
	ret = gpio_add_callback(ctx->busy.port, &busy_gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add BUSY callback: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&ctx->dio1, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure DIO1: %d", ret);
		return ret;
	}

	gpio_init_callback(&dio1_gpio_cb, dio1_isr_callback, BIT(ctx->dio1.pin));
	ret = gpio_add_callback(ctx->dio1.port, &dio1_gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add DIO1 callback: %d", ret);
		return ret;
	}

	LOG_INF("LR20xx HAL initialized");
	return 0;
}

void lr20xx_hal_set_dio1_callback(struct lr20xx_hal_context *ctx,
				   lr20xx_dio1_callback_t cb, void *user_data)
{
	ARG_UNUSED(ctx);
	dio1_user_cb = cb;
	dio1_user_data = user_data;
}

void lr20xx_hal_enable_dio1_irq(struct lr20xx_hal_context *ctx)
{
	gpio_pin_interrupt_configure_dt(&ctx->dio1, GPIO_INT_EDGE_RISING);
}

void lr20xx_hal_disable_dio1_irq(struct lr20xx_hal_context *ctx)
{
	gpio_pin_interrupt_configure_dt(&ctx->dio1, GPIO_INT_DISABLE);
}

lr20xx_hal_status_t lr20xx_hal_write(const void *context, const uint8_t *command,
				      const uint16_t command_length,
				      const uint8_t *data, const uint16_t data_length)
{
	struct lr20xx_hal_context *ctx = (struct lr20xx_hal_context *)context;
	int ret;

	/* Track opcode for BUSY timeout diagnostics */
	if (command_length >= 2) {
		last_opcode = ((uint16_t)command[0] << 8) | command[1];
	}
	last_cmd_time = k_uptime_get();

	if (check_device_ready(ctx) != LR20XX_HAL_STATUS_OK) {
		LOG_ERR("hal_write: device not ready, op=0x%04x", last_opcode);
		return LR20XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf tx_bufs[] = {
		{ .buf = (uint8_t *)command, .len = command_length },
		{ .buf = (uint8_t *)data, .len = data_length },
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = (data_length > 0) ? 2 : 1,
	};

	gpio_pin_set_dt(&ctx->nss, 1);
	ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);
	gpio_pin_set_dt(&ctx->nss, 0);

	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	/* Opcode 0x0127 = SetSleep (LR2021 datasheet §5.4.2) */
	if (command_length >= 2 && command[0] == 0x01 && command[1] == 0x27) {
		ctx->radio_is_sleeping = true;
		k_busy_wait(1000);  /* ≥500us sleep entry per datasheet §5.4.2 */
		return LR20XX_HAL_STATUS_OK;
	}

	return wait_on_busy(ctx);
}

/* LR2021 prepends a 2-byte stat header before response data (datasheet §5.4.1.2) */
#define LR20XX_STAT_LEN 2

/**
 * @brief Clock a command and read back its response, as two NSS windows.
 *
 * The command goes out in its own window.  The chip raises BUSY while it
 * prepares the answer, and a second window clocks that answer out:
 *
 *     window 1   MOSI [ command_length ]   MISO [ undefined ]
 *                -- BUSY high while the chip prepares the answer --
 *     window 2   MOSI [ LR20XX_STAT_LEN + data_length ]
 *                MISO [ stat header ][ payload ]
 *
 * This mirrors Semtech's reference HAL (lr20xx_hal_read() in LoRa Basics
 * Modem): command out, NSS released, wait on BUSY, then two dummy bytes and the
 * payload in a fresh window.  The Rx FIFO pop is the one read on this chip that
 * does NOT work this way — single window, no stat header, no BUSY wait; see
 * lr20xx_hal_direct_read_fifo().
 *
 * The BUSY wait between the two windows is load-bearing, and the failure it
 * guards is silent rather than loud: clock the second window too early and the
 * chip answers with its default status / IRQ stream instead of the payload,
 * which the caller then parses as a plausible-looking short integer rather than
 * reporting an SPI error.  The status check at the bottom of this function is
 * the backstop for the residual case where BUSY has not yet risen when the wait
 * samples it.
 *
 * NULL TX buffers clock the SPI controller's over-read character, NOT zero --
 * Nordic defaults it to 0xff, which the LR2021 reads as a bogus opcode and
 * rejects. Boards using this driver must set overrun-character = <0x00> (the
 * LR2021 NOP) on the SPI node.
 */
/* Command status field of the stat1 header (DS Table 6-38, bits [2:1]).
 * 0 FAIL, 1 PERR, 2 CMD_OK, 3 CMD_DATA.  The vendor decoder in
 * lr20xx_system.c shifts without masking; mask here, the field is 2 bits. */
#define LR20XX_STAT1_CMD_STATUS(b)	(((b) >> 1) & 0x03)
#define LR20XX_CMD_STATUS_DATA		0x03

/* Attempts at the two-window read before giving up.  Upstream MeshCore
 * measured recovery on the first retry in every observed case (4/4 on LR11xx,
 * 11/11 on LR2021, with the BUSY wait skipped on purpose). */
#define LR20XX_READ_ATTEMPTS		3

static int lr20xx_spi_read_frame(struct lr20xx_hal_context *ctx, const uint8_t *command,
				  uint16_t command_length, uint8_t *data, uint16_t data_length)
{
	int ret = 0;
	uint8_t stat[LR20XX_STAT_LEN];

	/* Retried as a whole: the chip only streams the answer in the window
	 * that follows its command, so recovering means re-issuing the command.
	 * Safe for every caller that lands here — lr20xx_hal_read() serves
	 * getters and address-based register reads, all idempotent.  The one
	 * read that is NOT idempotent, the Rx FIFO pop, does not come through
	 * this path at all (lr20xx_hal_direct_read_fifo(), single NSS window,
	 * no stat header, no BUSY wait — structurally immune to the race
	 * below). */
	for (int attempt = 0; attempt < LR20XX_READ_ATTEMPTS; attempt++) {
		/* Wait on BUSY before re-issuing, exactly as lr20xx_hal_read() does
		 * before the first pass.  A command clocked into a chip that has not
		 * dropped BUSY is not a retry, it is a malformed frame the chip
		 * answers with CMD_PERR — see check_device_ready() above for what
		 * that cost on the MeshTracker X1. */
		if (attempt > 0 && check_device_ready(ctx) != LR20XX_HAL_STATUS_OK) {
			return -ETIMEDOUT;
		}

		/* Phase 1: the command, in its own NSS window. */
		{
			const struct spi_buf tx_buf = {
				.buf = (uint8_t *)command,
				.len = command_length,
			};
			const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

			gpio_pin_set_dt(&ctx->nss, 1);
			ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);
			gpio_pin_set_dt(&ctx->nss, 0);

			if (ret < 0) {
				return ret;
			}
		}

		/* The answer is not ready until BUSY drops — without this the second
		 * window clocks out whatever the chip had, two bytes early. */
		if (wait_on_busy(ctx) != LR20XX_HAL_STATUS_OK) {
			return -ETIMEDOUT;
		}

		/* Phase 2: two dummy bytes absorb the stat header, then the payload.
		 * The stat header lands in a real buffer rather than being discarded
		 * with a NULL one: spi_nrfx_spim rejects a transfer whose first TX and
		 * RX buffers are both NULL (-EINVAL), and TX is legitimately NULL here. */
		{
			const struct spi_buf tx_buf = {
				.buf = NULL,
				.len = LR20XX_STAT_LEN + data_length,
			};
			const struct spi_buf rx_bufs[] = {
				{ .buf = stat, .len = LR20XX_STAT_LEN },
				{ .buf = data, .len = data_length },
			};
			const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
			const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

			gpio_pin_set_dt(&ctx->nss, 1);
			ret = spi_transceive(ctx->spi_dev, &ctx->spi_cfg, &tx, &rx);
			gpio_pin_set_dt(&ctx->nss, 0);

			if (ret < 0) {
				return ret;
			}
		}

		/* Did the chip actually answer US?
		 *
		 * wait_on_busy() returns immediately on a BUSY that reads low, and
		 * cannot tell "the command finished" from "BUSY has not risen yet".
		 * The gap between releasing NSS above and sampling the pin is two
		 * GPIO calls, so losing that race is entirely possible; RadioLib
		 * loses it with a whole microsecond of margin (MeshCore PR #3261).
		 * When it is lost, phase 2 clocks out the chip's default
		 * [stat 2B][irq 4B] stream instead of the payload, stat[] absorbs
		 * the two status bytes, and the caller parses IRQ bits as its
		 * answer.  Nothing reports an error: for GetRxPacketLength that is
		 * irq[31:16], which with RX_DONE (bit 18) set is exactly 4, so a
		 * real frame of any size is read out of the FIFO as a 4-byte one
		 * and silently dropped upstream as corrupt.
		 *
		 * The status separates the two cleanly.  CMD_DATA means "a read was
		 * processed and data is being transmitted instead of IRQ status" —
		 * the only correct answer for this window.  A reply produced by the
		 * race reports CMD_OK instead, i.e. the chip is streaming status,
		 * not data.  The Rx FIFO is untouched at this point, so re-reading
		 * recovers the frame rather than losing it. */
		if (LR20XX_STAT1_CMD_STATUS(stat[0]) == LR20XX_CMD_STATUS_DATA) {
			return ret;
		}
	}

	/* Out of attempts.  Hand back what came in rather than failing the call:
	 * that is exactly today's behaviour, so no existing caller regresses on a
	 * read whose status is legitimately something else.  Tighten to an error
	 * return only once hardware confirms every read reports CMD_DATA. */
	LOG_WRN("read op=0x%04x: stat1=0x%02x (cmd_status=%u, want CMD_DATA) after %d attempts",
		last_opcode, stat[0], LR20XX_STAT1_CMD_STATUS(stat[0]), LR20XX_READ_ATTEMPTS);

	return ret;
}

lr20xx_hal_status_t lr20xx_hal_read(const void *context, const uint8_t *command,
				     const uint16_t command_length,
				     uint8_t *data, const uint16_t data_length)
{
	struct lr20xx_hal_context *ctx = (struct lr20xx_hal_context *)context;
	int ret;

	/* Track opcode for BUSY timeout diagnostics */
	if (command_length >= 2) {
		last_opcode = ((uint16_t)command[0] << 8) | command[1];
	}
	last_cmd_time = k_uptime_get();

	if (check_device_ready(ctx) != LR20XX_HAL_STATUS_OK) {
		LOG_ERR("hal_read: device not ready, op=0x%04x", last_opcode);
		return LR20XX_HAL_STATUS_ERROR;
	}

	/* No response requested — this is a plain command write. */
	if (data_length == 0) {
		const struct spi_buf tx_buf = {
			.buf = (uint8_t *)command,
			.len = command_length,
		};
		const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

		gpio_pin_set_dt(&ctx->nss, 1);
		ret = spi_write(ctx->spi_dev, &ctx->spi_cfg, &tx);
		gpio_pin_set_dt(&ctx->nss, 0);

		if (ret < 0) {
			LOG_ERR("SPI write (cmd) failed: %d", ret);
			return LR20XX_HAL_STATUS_ERROR;
		}

		return wait_on_busy(ctx);
	}

	ret = lr20xx_spi_read_frame(ctx, command, command_length, data, data_length);
	if (ret < 0) {
		LOG_ERR("SPI read failed: %d", ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read(const void *context, uint8_t *data,
					    const uint16_t data_length)
{
	struct lr20xx_hal_context *ctx = (struct lr20xx_hal_context *)context;
	int ret;

	if (check_device_ready(ctx) != LR20XX_HAL_STATUS_OK) {
		return LR20XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf rx_buf = { .buf = data, .len = data_length };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	gpio_pin_set_dt(&ctx->nss, 1);
	ret = spi_read(ctx->spi_dev, &ctx->spi_cfg, &rx);
	gpio_pin_set_dt(&ctx->nss, 0);

	if (ret < 0) {
		LOG_ERR("SPI direct read failed: %d", ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(const void *context,
						  const uint8_t *command,
						  const uint16_t command_length,
						  uint8_t *data,
						  const uint16_t data_length)
{
	struct lr20xx_hal_context *ctx = (struct lr20xx_hal_context *)context;
	int ret;

	if (check_device_ready(ctx) != LR20XX_HAL_STATUS_OK) {
		return LR20XX_HAL_STATUS_ERROR;
	}

	/* Unlike every other read, the FIFO answers inside the command's own NSS
	 * window with no stat header and no BUSY wait — command bytes out, payload
	 * straight back. Semtech's reference HAL is explicit about this
	 * (lr20xx_hal_direct_read_fifo); treating it like a normal read releases
	 * NSS mid-frame and eats two payload bytes as a header. */
	{
		const struct spi_buf tx_bufs[] = {
			{ .buf = (uint8_t *)command, .len = command_length },
			{ .buf = NULL, .len = data_length },
		};
		const struct spi_buf rx_bufs[] = {
			{ .buf = NULL, .len = command_length },
			{ .buf = data, .len = data_length },
		};
		const struct spi_buf_set tx = { .buffers = tx_bufs, .count = 2 };
		const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

		gpio_pin_set_dt(&ctx->nss, 1);
		ret = spi_transceive(ctx->spi_dev, &ctx->spi_cfg, &tx, &rx);
		gpio_pin_set_dt(&ctx->nss, 0);
	}

	if (ret < 0) {
		LOG_ERR("SPI FIFO read failed: %d", ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_reset(const void *context)
{
	struct lr20xx_hal_context *ctx = (struct lr20xx_hal_context *)context;

	LOG_INF("LR20xx reset: assert reset, hold 1ms");

	/* DS §4.2: "Assert NRESET low for minimum 100 us."  1 ms is 10x the
	 * minimum and matches Semtech's reference HAL; the 10 ms here before
	 * was 100x it. */
	gpio_pin_set_dt(&ctx->reset, 1);
	k_msleep(1);

	gpio_pin_set_dt(&ctx->reset, 0);

	/* No blind boot delay.  DS §4.2/§4.3 make BUSY the completion signal —
	 * "During each of the reset procedures … the BUSY signal remains high …
	 * As the device enters Standby RC mode, the BUSY signal returns to a low
	 * state" — and the datasheet prescribes no post-release wait.  The
	 * wait_on_busy() below is interrupt-driven with a 3 s bound and logs on
	 * timeout, so the 300 ms that used to sit here was 300 ms of sleeping
	 * before asking the question that answers itself.  It cost that on every
	 * one of up to 3 boot attempts, and again inside lr20xx_hardware_reset(),
	 * which runs on the DIO1 work queue holding spi_mutex and now also
	 * carries the PRAM block writes. */
	LOG_INF("LR20xx reset complete, BUSY=%d", gpio_pin_get_dt(&ctx->busy));

	ctx->radio_is_sleeping = false;

	return wait_on_busy(ctx);
}

lr20xx_hal_status_t lr20xx_hal_wakeup(const void *context)
{
	struct lr20xx_hal_context *ctx = (struct lr20xx_hal_context *)context;
	return check_device_ready(ctx);
}
