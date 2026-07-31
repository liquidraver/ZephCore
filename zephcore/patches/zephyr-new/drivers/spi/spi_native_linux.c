/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Zephyr SPI driver that bridges to a Linux /dev/spidev character device.
 * Runs under native_sim on a real Linux host (not in QEMU/simulation).
 *
 * Host-side ioctls live in spi_native_linux_adapt.c (compiled into the
 * native_simulator INTERFACE) to keep host headers out of the Zephyr
 * driver translation unit.
 */

#define DT_DRV_COMPAT zephcore_spi_native_linux

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_native_linux, CONFIG_SPI_LOG_LEVEL);

#include <cmdline.h>
#include <posix_native_task.h>

#include "spi_native_linux_adapt.h"

/* spi_context.h uses LOG_* macros, so it must come AFTER LOG_MODULE_REGISTER. */
#include "spi_context.h"

struct spi_native_linux_config {
	const char *spi_dev_path;
	uint8_t spi_mode;
};

struct spi_native_linux_data {
	struct spi_context ctx;
	int fd;
	uint32_t current_speed_hz;
	uint8_t current_mode;
	uint8_t current_bits;
	bool configured;
};

/* Runtime cmdline override of the SPI device path (applies to instance 0). */
static char *spi_dev_cmd_opt;

static int spi_native_linux_configure(const struct device *dev,
				       const struct spi_config *config)
{
	struct spi_native_linux_data *data = dev->data;
	uint8_t mode_byte = 0;
	uint8_t bits = 8;
	uint32_t speed_hz;
	int ret;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex SPI not supported by spidev");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_TRANSFER_LSB) != 0) {
		LOG_ERR("LSB-first not supported by spidev");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Only 8-bit words supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_MODE_CPOL) {
		mode_byte |= 0x02;
	}
	if (config->operation & SPI_MODE_CPHA) {
		mode_byte |= 0x01;
	}

	speed_hz = config->frequency;
	if (speed_hz == 0U) {
		speed_hz = 2000000U;
	}

	if (mode_byte != data->current_mode) {
		ret = spi_native_linux_set_mode(data->fd, mode_byte);
		if (ret < 0) {
			LOG_ERR("set_mode(%u) failed: %d", mode_byte, ret);
			return -EIO;
		}
		data->current_mode = mode_byte;
	}

	if (bits != data->current_bits) {
		ret = spi_native_linux_set_bits(data->fd, bits);
		if (ret < 0) {
			LOG_ERR("set_bits(%u) failed: %d", bits, ret);
			return -EIO;
		}
		data->current_bits = bits;
	}

	if (speed_hz != data->current_speed_hz) {
		ret = spi_native_linux_set_speed(data->fd, speed_hz);
		if (ret < 0) {
			LOG_ERR("set_speed(%u) failed: %d", speed_hz, ret);
			return -EIO;
		}
		data->current_speed_hz = speed_hz;
	}

	data->ctx.config = config;
	data->configured = true;

	return 0;
}

/* Sum buffer lengths in a buf_set, treating NULL as zero. */
static size_t bufset_total_len(const struct spi_buf_set *set)
{
	size_t total = 0;

	if (set == NULL) {
		return 0;
	}
	for (size_t i = 0; i < set->count; i++) {
		total += set->buffers[i].len;
	}
	return total;
}

/*
 * Flatten scatter-gather buffers into a single contiguous TX/RX pair,
 * issue one SPI_IOC_MESSAGE, then scatter the RX result back.
 *
 * spidev's SPI_IOC_MESSAGE does support multiple transfers in one ioctl,
 * but flattening is dramatically simpler and matches how the SX126x driver
 * actually uses SPI (one address byte + payload, no large gathers).
 */
static int spi_native_linux_transceive(const struct device *dev,
					const struct spi_config *config,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs)
{
	struct spi_native_linux_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = spi_native_linux_configure(dev, config);
	if (ret != 0) {
		spi_context_release(&data->ctx, ret);
		return ret;
	}

	size_t tx_total = bufset_total_len(tx_bufs);
	size_t rx_total = bufset_total_len(rx_bufs);
	size_t xfer_len = tx_total > rx_total ? tx_total : rx_total;

	if (xfer_len == 0U) {
		spi_context_release(&data->ctx, 0);
		return 0;
	}

	uint8_t *tx_flat = k_malloc(xfer_len);
	uint8_t *rx_flat = k_malloc(xfer_len);

	if (tx_flat == NULL || rx_flat == NULL) {
		k_free(tx_flat);
		k_free(rx_flat);
		LOG_ERR("Out of memory for %zu-byte SPI buffer", xfer_len);
		spi_context_release(&data->ctx, -ENOMEM);
		return -ENOMEM;
	}

	memset(tx_flat, 0, xfer_len);
	memset(rx_flat, 0, xfer_len);

	/* Pack TX bytes */
	if (tx_bufs != NULL) {
		size_t off = 0;
		for (size_t i = 0; i < tx_bufs->count; i++) {
			const struct spi_buf *b = &tx_bufs->buffers[i];
			if (b->buf != NULL && b->len > 0U) {
				memcpy(tx_flat + off, b->buf, b->len);
			}
			off += b->len;
		}
	}

	/* Manually drive CS via spi_context (which uses GPIO from DT spi-cs-gpios). */
	spi_context_cs_control(&data->ctx, true);

	ret = spi_native_linux_xfer(data->fd, tx_flat, rx_flat, xfer_len,
				    data->current_speed_hz);

	spi_context_cs_control(&data->ctx, false);

	if (ret < 0) {
		LOG_ERR("SPI_IOC_MESSAGE failed: %d", ret);
		k_free(tx_flat);
		k_free(rx_flat);
		spi_context_release(&data->ctx, -EIO);
		return -EIO;
	}

	/* Scatter RX bytes back */
	if (rx_bufs != NULL) {
		size_t off = 0;
		for (size_t i = 0; i < rx_bufs->count; i++) {
			const struct spi_buf *b = &rx_bufs->buffers[i];
			if (b->buf != NULL && b->len > 0U) {
				memcpy(b->buf, rx_flat + off, b->len);
			}
			off += b->len;
		}
	}

	k_free(tx_flat);
	k_free(rx_flat);

	spi_context_release(&data->ctx, 0);
	return 0;
}

static int spi_native_linux_release(const struct device *dev,
				     const struct spi_config *config)
{
	struct spi_native_linux_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, spi_native_linux_driver_api) = {
	.transceive = spi_native_linux_transceive,
	.release = spi_native_linux_release,
};

static int spi_native_linux_init(const struct device *dev)
{
	const struct spi_native_linux_config *cfg = dev->config;
	struct spi_native_linux_data *data = dev->data;
	const char *path;
	int ret;

	path = (spi_dev_cmd_opt != NULL) ? spi_dev_cmd_opt : cfg->spi_dev_path;

	LOG_INF("Opening SPI host device: %s", path);
	data->fd = spi_native_linux_open(path);
	if (data->fd < 0) {
		LOG_ERR("Failed to open %s: %d", path, data->fd);
		return -ENODEV;
	}

	data->current_speed_hz = 0;
	data->current_mode = 0xFF;
	data->current_bits = 0xFF;
	data->configured = false;

	/* Apply initial mode hint from DT so the line is in a sane state
	 * before the first transceive() configures it from spi_config. */
	if (cfg->spi_mode != 0xFF) {
		ret = spi_native_linux_set_mode(data->fd, cfg->spi_mode);
		if (ret == 0) {
			data->current_mode = cfg->spi_mode;
		}
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		LOG_ERR("CS GPIO configure failed: %d", ret);
		spi_native_linux_close(data->fd);
		data->fd = -1;
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	LOG_INF("SPI native_linux driver ready (fd=%d)", data->fd);
	return 0;
}

#define SPI_NATIVE_LINUX_INIT(inst)						\
	static const struct spi_native_linux_config				\
		spi_native_linux_cfg_##inst = {					\
		.spi_dev_path = DT_INST_PROP(inst, spi_dev),			\
		.spi_mode = DT_INST_PROP_OR(inst, spi_mode, 0),			\
	};									\
										\
	static struct spi_native_linux_data spi_native_linux_data_##inst = {	\
		SPI_CONTEXT_INIT_LOCK(spi_native_linux_data_##inst, ctx),	\
		SPI_CONTEXT_INIT_SYNC(spi_native_linux_data_##inst, ctx),	\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)		\
	};									\
										\
	SPI_DEVICE_DT_INST_DEFINE(inst,						\
				  spi_native_linux_init,			\
				  NULL,						\
				  &spi_native_linux_data_##inst,		\
				  &spi_native_linux_cfg_##inst,			\
				  POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	\
				  &spi_native_linux_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_NATIVE_LINUX_INIT)

/* Command-line arg registration: --lora-spidev=<path> overrides DT spi-dev */
static void spi_native_linux_add_cmdline_opts(void)
{
	static struct args_struct_t spi_native_options[] = {
		{
			.option = "lora-spidev",
			.name = "path",
			.type = 's',
			.dest = (void *)&spi_dev_cmd_opt,
			.descript = "Linux SPI device path (overrides DT spi-dev)",
		},
		ARG_TABLE_ENDMARKER,
	};

	native_add_command_line_opts(spi_native_options);
}

NATIVE_TASK(spi_native_linux_add_cmdline_opts, PRE_BOOT_1, 11);
