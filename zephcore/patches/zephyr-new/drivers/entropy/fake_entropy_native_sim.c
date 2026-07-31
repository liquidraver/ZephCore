/*
 * Copyright (c) 2018 Oticon A/S
 * Copyright (c) 2026 ZephCore
 *
 * SPDX-License-Identifier: MIT
 *
 * ZephCore replacement for Zephyr's fake_entropy_native_sim.c.
 *
 * Reads entropy from /dev/urandom via NSI host trampolines — same DTS
 * compatible (zephyr,native-sim-rng), same Kconfig symbol
 * (FAKE_ENTROPY_NATIVE_SIM), no new DTS plumbing needed.
 *
 * Differences from upstream:
 *  - Reads real entropy from /dev/urandom on every call.
 *  - No "WARNING: Using a test - not safe - entropy source" printed.
 *  - --seed / --seed-random CLI options kept for reproducible test runs:
 *    pass --seed=<N> to force a fixed seed (falls back to libc random()).
 *  - Default (no CLI args): /dev/urandom, proper entropy, no warning.
 */

#define DT_DRV_COMPAT zephyr_native_sim_rng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/arch/posix/posix_trace.h>
#include "cmdline.h"
#include "posix_native_task.h"
#include "nsi_host_trampolines.h"

/* O_RDONLY = 0 on every Linux ABI we target. */
#define NSI_O_RDONLY 0

static unsigned int seed;
static bool use_seed; /* true → fall back to seeded libc random() */
static bool seed_set; /* set by --seed CLI callback */

static int entropy_native_sim_get_entropy(const struct device *dev,
					   uint8_t *buffer, uint16_t length)
{
	ARG_UNUSED(dev);

	if (use_seed) {
		/* Reproducible mode: use libc random() (same as upstream). */
		while (length) {
			long value = nsi_host_random();
			size_t to_copy = MIN(length, 3);

			memcpy(buffer, &value, to_copy);
			buffer += to_copy;
			length -= to_copy;
		}
		return 0;
	}

	/* Default: read from /dev/urandom. */
	int fd = nsi_host_open("/dev/urandom", NSI_O_RDONLY);

	if (fd < 0) {
		posix_print_error_and_exit("entropy: failed to open "
					   "/dev/urandom\n");
		return -EIO; /* unreachable */
	}

	while (length > 0) {
		long n = nsi_host_read(fd, buffer, length);

		if (n <= 0) {
			nsi_host_close(fd);
			posix_print_error_and_exit("entropy: read from "
						   "/dev/urandom failed\n");
			return -EIO;
		}
		buffer += n;
		length -= (uint16_t)n;
	}

	nsi_host_close(fd);
	return 0;
}

static int entropy_native_sim_get_entropy_isr(const struct device *dev,
					       uint8_t *buf, uint16_t len,
					       uint32_t flags)
{
	ARG_UNUSED(flags);
	entropy_native_sim_get_entropy(dev, buf, len);
	return len;
}

static int entropy_native_sim_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (seed_set) {
		/* User passed --seed: reproducible mode. */
		use_seed = true;
		nsi_host_srandom(seed);
	}
	/* No warning: /dev/urandom is a legitimate entropy source. */
	return 0;
}

static DEVICE_API(entropy, entropy_native_sim_api_funcs) = {
	.get_entropy     = entropy_native_sim_get_entropy,
	.get_entropy_isr = entropy_native_sim_get_entropy_isr,
};

DEVICE_DT_INST_DEFINE(0,
		      entropy_native_sim_init, NULL,
		      NULL, NULL,
		      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
		      &entropy_native_sim_api_funcs);

static void seed_was_set(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	seed_set = true;
}

static void add_fake_entropy_option(void)
{
	static struct args_struct_t entropy_options[] = {
		{
			.option = "seed",
			.name = "r_seed",
			.type = 'u',
			.dest = (void *)&seed,
			.call_when_found = seed_was_set,
			.descript = "Fix the entropy seed for reproducible runs "
				    "(disables /dev/urandom). E.g. --seed=97229",
		},
		ARG_TABLE_ENDMARKER,
	};

	native_add_command_line_opts(entropy_options);
}

NATIVE_TASK(add_fake_entropy_option, PRE_BOOT_1, 10);
