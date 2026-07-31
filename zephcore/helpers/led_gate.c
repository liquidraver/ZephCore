/*
 * ZephCore - LED master gate
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "led_gate.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/* Atomic because the readers are not all on one thread: the heartbeat work
 * handler runs on the system work queue, while the TX LED is driven from the
 * dispatcher's transmit path. Writers are the main thread (boot / CLI) and the
 * UI task. */
static atomic_t s_leds_disabled;

/*
 * Weak: overridden by helpers/ui/ui_common.c in builds that have a UI, so a
 * change made from the CLI also extinguishes a lit heartbeat LED immediately
 * and syncs the UI's LED page. No-op in headless builds, where ui_common.c
 * isn't compiled at all — which is exactly why the gate lives here and not
 * there.
 */
__weak void zephcore_leds_ui_sync(bool disabled) { ARG_UNUSED(disabled); }

bool zephcore_leds_disabled(void)
{
	return atomic_get(&s_leds_disabled) != 0;
}

void zephcore_leds_set_disabled(bool disabled)
{
	atomic_set(&s_leds_disabled, disabled ? 1 : 0);
	zephcore_leds_ui_sync(disabled);
}
