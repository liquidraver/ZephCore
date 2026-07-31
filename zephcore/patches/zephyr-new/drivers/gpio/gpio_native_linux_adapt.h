/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Host-side libgpiod v2 wrapper for gpio_native_linux.
 * Compiled into the native_simulator INTERFACE so libgpiod headers
 * never reach the Zephyr translation unit.
 */

#ifndef ZEPHCORE_GPIO_NATIVE_LINUX_ADAPT_H
#define ZEPHCORE_GPIO_NATIVE_LINUX_ADAPT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handles — actual types live host-side. */
typedef void *gnl_chip_t;
typedef void *gnl_line_t;

/* Edge mode constants (mirror libgpiod v2 enum) */
#define GNL_EDGE_NONE     0
#define GNL_EDGE_RISING   1
#define GNL_EDGE_FALLING  2
#define GNL_EDGE_BOTH     3

/* Open chip. Returns handle or NULL on error. */
gnl_chip_t gnl_chip_open(const char *path);
void gnl_chip_close(gnl_chip_t chip);

/* Request a single line as OUTPUT with initial value `init_val` (0/1).
 * Returns handle or NULL on error. */
gnl_line_t gnl_request_output(gnl_chip_t chip, unsigned int offset,
			       int init_val, bool active_low);

/* Request a single line as INPUT (no edge detection).
 * Returns handle or NULL on error. */
gnl_line_t gnl_request_input(gnl_chip_t chip, unsigned int offset,
			      bool pull_up, bool pull_down,
			      bool active_low);

/* Request a single line as INPUT with edge-event detection.
 * edge: one of GNL_EDGE_*.
 * Returns handle or NULL on error. */
gnl_line_t gnl_request_input_edge(gnl_chip_t chip, unsigned int offset,
				   int edge, bool pull_up, bool pull_down,
				   bool active_low);

/* Release a line request. Safe to pass NULL. */
void gnl_line_release(gnl_line_t line);

/* Read current value of an input line (0/1) or -errno on error. */
int gnl_line_get_value(gnl_line_t line);

/* Set output line value (0/1). Returns 0 or -errno. */
int gnl_line_set_value(gnl_line_t line, int value);

/* Get pollable fd for edge events on this line, or -1 if none. */
int gnl_line_get_fd(gnl_line_t line);

/* Drain pending edge events on this line. Returns number drained, or -errno. */
int gnl_line_drain_events(gnl_line_t line);

/* Block on poll() across an array of fds, up to timeout_ms (-1 = forever).
 * Returns: bitmask of which fds (by index into the input array) are ready,
 * or 0 on timeout, -errno on error. Max 32 fds supported. */
uint32_t gnl_poll_fds(const int *fds, size_t count, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_GPIO_NATIVE_LINUX_ADAPT_H */
