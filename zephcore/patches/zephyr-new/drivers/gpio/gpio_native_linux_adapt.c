/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-side Linux GPIO chardev V2 wrapper for gpio_native_linux.
 *
 * Compiled into the native_simulator INTERFACE target. Keeps host
 * headers (<linux/gpio.h>) out of the Zephyr translation unit.
 *
 * Uses the kernel's GPIO V2 character-device uAPI directly via ioctl —
 * no libgpiod dependency. Available on every Linux ≥ 5.10 (Dec 2020).
 * This is the same ABI libgpiod v2 wraps; we just skip the wrapper to
 * avoid the libgpiod v1-vs-v2 packaging mess on common SBC distros
 * (Ubuntu 24.04 ships v1; Debian 13 ships v2; we work on both).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#ifdef __linux
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#else
#error "gpio_native_linux only builds on Linux hosts"
#endif

#include "gpio_native_linux_adapt.h"

#ifndef GPIO_V2_GET_LINE_IOCTL
#error "Linux kernel headers too old; GPIO V2 chardev uAPI (>= 5.10) required"
#endif

/* Host-side per-line state: line request fd + the offset we requested. */
struct gnl_line_handle {
	int line_fd;
	uint32_t offset;
};

gnl_chip_t gnl_chip_open(const char *path)
{
	int fd = open(path, O_RDWR | O_CLOEXEC);

	if (fd < 0) {
		return NULL;
	}
	/* Encode fd in the pointer so callers see "non-NULL = success".
	 * fd of 0 would alias NULL, but Linux never hands /dev/gpiochipN
	 * out as fd 0 in practice (stdin keeps that). Still, defend: */
	if (fd == 0) {
		int newfd = fcntl(fd, F_DUPFD_CLOEXEC, 1);

		close(fd);
		if (newfd < 0) {
			return NULL;
		}
		fd = newfd;
	}
	return (gnl_chip_t)(intptr_t)fd;
}

void gnl_chip_close(gnl_chip_t chip)
{
	if (chip != NULL) {
		close((int)(intptr_t)chip);
	}
}

static struct gnl_line_handle *do_request(int chip_fd, unsigned int offset,
					   uint64_t flags)
{
	struct gpio_v2_line_request req;

	memset(&req, 0, sizeof(req));
	req.offsets[0] = offset;
	req.num_lines = 1;
	req.config.flags = flags;
	strncpy(req.consumer, "zephcore", sizeof(req.consumer) - 1);

	if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		return NULL;
	}

	struct gnl_line_handle *h = calloc(1, sizeof(*h));

	if (h == NULL) {
		close(req.fd);
		return NULL;
	}
	h->line_fd = req.fd;
	h->offset = offset;
	return h;
}

gnl_line_t gnl_request_output(gnl_chip_t chip, unsigned int offset,
			       int init_val, bool active_low)
{
	int chip_fd = (int)(intptr_t)chip;
	uint64_t flags = GPIO_V2_LINE_FLAG_OUTPUT;

	/* Do NOT apply ACTIVE_LOW at the kernel level: Zephyr's generic GPIO
	 * layer already converts logical<->physical for GPIO_ACTIVE_LOW (and
	 * hands us a physical init value + raw set/get).  Inverting again here
	 * double-inverts active-low pins (e.g. SX126x RESET), holding the chip
	 * in reset.  The driver's port_*_raw contract is physical/raw. */
	(void)active_low;

	struct gnl_line_handle *h = do_request(chip_fd, offset, flags);

	if (h == NULL) {
		return NULL;
	}

	/* Drive the initial value. */
	struct gpio_v2_line_values vals;

	memset(&vals, 0, sizeof(vals));
	vals.mask = 1ULL;
	vals.bits = init_val ? 1ULL : 0ULL;
	(void)ioctl(h->line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);

	return (gnl_line_t)h;
}

static uint64_t input_flags(bool pull_up, bool pull_down, bool active_low)
{
	uint64_t flags = GPIO_V2_LINE_FLAG_INPUT;

	/* See gnl_request_output: Zephyr's generic layer owns ACTIVE_LOW
	 * inversion; applying it here too would double-invert. */
	(void)active_low;
	if (pull_up) {
		flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
	} else if (pull_down) {
		flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
	}
	/* else: leave bias AS-IS (no flag).  Forcing BIAS_DISABLED can turn off
	 * the pin's input path on some SoCs (e.g. Rockchip), making a
	 * push-pull-driven input (SX126x BUSY) read stuck-low.  libgpiod
	 * defaults to AS-IS, which reads these pins correctly. */
	return flags;
}

gnl_line_t gnl_request_input(gnl_chip_t chip, unsigned int offset,
			      bool pull_up, bool pull_down,
			      bool active_low)
{
	int chip_fd = (int)(intptr_t)chip;
	uint64_t flags = input_flags(pull_up, pull_down, active_low);

	return (gnl_line_t)do_request(chip_fd, offset, flags);
}

gnl_line_t gnl_request_input_edge(gnl_chip_t chip, unsigned int offset,
				   int edge, bool pull_up, bool pull_down,
				   bool active_low)
{
	int chip_fd = (int)(intptr_t)chip;
	uint64_t flags = input_flags(pull_up, pull_down, active_low);

	switch (edge) {
	case GNL_EDGE_RISING:
		flags |= GPIO_V2_LINE_FLAG_EDGE_RISING;
		break;
	case GNL_EDGE_FALLING:
		flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING;
		break;
	case GNL_EDGE_BOTH:
		flags |= GPIO_V2_LINE_FLAG_EDGE_RISING |
			 GPIO_V2_LINE_FLAG_EDGE_FALLING;
		break;
	default:
		break;
	}
	return (gnl_line_t)do_request(chip_fd, offset, flags);
}

void gnl_line_release(gnl_line_t line)
{
	struct gnl_line_handle *h = (struct gnl_line_handle *)line;

	if (h == NULL) {
		return;
	}
	if (h->line_fd >= 0) {
		close(h->line_fd);
	}
	free(h);
}

int gnl_line_get_value(gnl_line_t line)
{
	struct gnl_line_handle *h = (struct gnl_line_handle *)line;

	if (h == NULL) {
		return -EINVAL;
	}

	struct gpio_v2_line_values vals;

	memset(&vals, 0, sizeof(vals));
	vals.mask = 1ULL;

	int rc = ioctl(h->line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals);

	if (rc < 0) {
		return -errno;
	}
	return (vals.bits & 1ULL) ? 1 : 0;
}

int gnl_line_set_value(gnl_line_t line, int value)
{
	struct gnl_line_handle *h = (struct gnl_line_handle *)line;

	if (h == NULL) {
		return -EINVAL;
	}

	struct gpio_v2_line_values vals;

	memset(&vals, 0, sizeof(vals));
	vals.mask = 1ULL;
	vals.bits = value ? 1ULL : 0ULL;

	if (ioctl(h->line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals) < 0) {
		return -errno;
	}
	return 0;
}

int gnl_line_get_fd(gnl_line_t line)
{
	struct gnl_line_handle *h = (struct gnl_line_handle *)line;

	if (h == NULL) {
		return -1;
	}
	return h->line_fd;
}

int gnl_line_drain_events(gnl_line_t line)
{
	struct gnl_line_handle *h = (struct gnl_line_handle *)line;

	if (h == NULL) {
		return -EINVAL;
	}

	struct gpio_v2_line_event ev;
	int count = 0;

	/* Drain all currently-buffered events (non-blocking by way of poll). */
	while (true) {
		struct pollfd pfd = { .fd = h->line_fd, .events = POLLIN };
		int pr = poll(&pfd, 1, 0);

		if (pr <= 0) {
			break;
		}
		ssize_t n = read(h->line_fd, &ev, sizeof(ev));

		if (n != (ssize_t)sizeof(ev)) {
			break;
		}
		count++;
	}
	return count;
}

uint32_t gnl_poll_fds(const int *fds, size_t count, int timeout_ms)
{
	if (count > 32) {
		count = 32;
	}

	struct pollfd pfds[32];

	for (size_t i = 0; i < count; i++) {
		pfds[i].fd = fds[i];
		pfds[i].events = POLLIN;
		pfds[i].revents = 0;
	}

	int ret = poll(pfds, count, timeout_ms);

	if (ret <= 0) {
		return 0;
	}

	uint32_t mask = 0;

	for (size_t i = 0; i < count; i++) {
		if (pfds[i].revents & POLLIN) {
			mask |= (1U << i);
		}
	}
	return mask;
}
