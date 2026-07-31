/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Host-side spidev wrapper functions for spi_native_linux.
 *
 * This file is compiled into the native_simulator INTERFACE target, NOT
 * the Zephyr application. That keeps host headers (<linux/spi/spidev.h>,
 * <sys/ioctl.h>) out of the Zephyr translation unit, avoiding type
 * collisions between the Zephyr kernel and the host C library.
 *
 * Only plain C ABI types cross the boundary back into the Zephyr-side
 * driver (see spi_native_linux_adapt.h).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#ifdef __linux
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#else
#error "spi_native_linux only builds on Linux hosts"
#endif

#include "spi_native_linux_adapt.h"

int spi_native_linux_open(const char *path)
{
	int fd = open(path, O_RDWR);

	if (fd < 0) {
		return -errno;
	}
	return fd;
}

void spi_native_linux_close(int fd)
{
	if (fd >= 0) {
		close(fd);
	}
}

int spi_native_linux_set_mode(int fd, uint8_t mode)
{
	uint8_t m = mode;

	if (ioctl(fd, SPI_IOC_WR_MODE, &m) < 0) {
		return -errno;
	}
	return 0;
}

int spi_native_linux_set_bits(int fd, uint8_t bits)
{
	uint8_t b = bits;

	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &b) < 0) {
		return -errno;
	}
	return 0;
}

int spi_native_linux_set_speed(int fd, uint32_t speed_hz)
{
	uint32_t s = speed_hz;

	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &s) < 0) {
		return -errno;
	}
	return 0;
}

int spi_native_linux_xfer(int fd, const uint8_t *tx, uint8_t *rx,
			   size_t len, uint32_t speed_hz)
{
	struct spi_ioc_transfer xfer;

	if (len == 0) {
		return 0;
	}

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (uintptr_t)tx;
	xfer.rx_buf = (uintptr_t)rx;
	xfer.len = (uint32_t)len;
	xfer.speed_hz = speed_hz;
	xfer.bits_per_word = 8;
	xfer.cs_change = 0;
	xfer.delay_usecs = 0;

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);

	if (ret < 0) {
		return -errno;
	}
	return ret;
}
