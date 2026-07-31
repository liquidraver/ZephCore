/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Host-side spidev wrapper functions for spi_native_linux.
 * Compiled into the native_simulator INTERFACE so host headers
 * (<linux/spi/spidev.h>, etc.) never reach the Zephyr translation unit.
 */

#ifndef ZEPHCORE_SPI_NATIVE_LINUX_ADAPT_H
#define ZEPHCORE_SPI_NATIVE_LINUX_ADAPT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Open /dev/spidevX.Y, return fd >= 0 on success or -errno. */
int spi_native_linux_open(const char *path);

/* Close the fd. */
void spi_native_linux_close(int fd);

/* Set SPI mode (0..3) via SPI_IOC_WR_MODE. Returns 0 or -errno. */
int spi_native_linux_set_mode(int fd, uint8_t mode);

/* Set bits-per-word via SPI_IOC_WR_BITS_PER_WORD. Returns 0 or -errno. */
int spi_native_linux_set_bits(int fd, uint8_t bits);

/* Set max clock speed via SPI_IOC_WR_MAX_SPEED_HZ. Returns 0 or -errno. */
int spi_native_linux_set_speed(int fd, uint32_t speed_hz);

/* Full-duplex transfer of `len` bytes via SPI_IOC_MESSAGE(1).
 * tx and rx must each be `len` bytes (rx receives the input, tx is sent).
 * Returns bytes transferred on success, -errno on failure.
 */
int spi_native_linux_xfer(int fd, const uint8_t *tx, uint8_t *rx,
			   size_t len, uint32_t speed_hz);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_SPI_NATIVE_LINUX_ADAPT_H */
