/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Native-Linux (native_sim) runtime setup.
 *
 * Forces the native_simulator's simulated clock into real-time mode so it
 * tracks host wall-clock time.  This is REQUIRED for ZephCore on native
 * Linux: unlike a pure simulation, we drive a REAL SX126x radio over
 * SPI/GPIO, and the radio's BUSY / DIO1 (CAD, TX-done, RX-done) signals
 * arrive in real time.  Without real-time mode the simulated clock races
 * ahead of the hardware and every radio timeout fires prematurely (CAD
 * -ETIMEDOUT, TX/RX stalls).  Equivalent to always passing --rt, so the
 * binary no longer needs that flag.
 */

#include <stdbool.h>

#include <posix_native_task.h>

/* Provided by the native_simulator timer model (nsi_timer_model.h); declared
 * here directly to avoid pulling the native_simulator-internal include path
 * into this translation unit. */
extern void hwtimer_set_real_time_mode(bool new_rt);

static void zephcore_native_force_realtime(void)
{
	hwtimer_set_real_time_mode(true);
}

/* PRE_BOOT_2: runs after command-line parsing but before the HW models
 * (incl. the timer model) are initialized, so real-time mode is in effect
 * from the very first tick.  An explicit --rt/--no-rt on the command line
 * still overrides this if the user wants the old fast-as-possible behaviour
 * (those options are parsed before PRE_BOOT_2... so to let --no-rt win we
 * would need PRE_BOOT_1; PRE_BOOT_2 intentionally forces real-time). */
NATIVE_TASK(zephcore_native_force_realtime, PRE_BOOT_2, 1);
