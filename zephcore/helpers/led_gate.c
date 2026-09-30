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

/*
 * Both modes default to 0, which every LEDS_* enum defines as the behaviour the
 * firmware had before these settings existed.  That matters beyond tidiness: a
 * role whose boot path forgets to apply the pref, or a build with no CLI at
 * all, still lands on the historical behaviour rather than something new.
 */
static atomic_t s_radio_mode;   /* LEDS_RADIO_TX */
static atomic_t s_hb_mode;      /* LEDS_HB_ALL */
static atomic_t s_radio_holds_pin;

uint8_t zephcore_leds_radio_mode(void)
{
	return (uint8_t)atomic_get(&s_radio_mode);
}

void zephcore_leds_set_radio_mode(uint8_t mode)
{
	atomic_set(&s_radio_mode, mode);
}

uint8_t zephcore_leds_hb_mode(void)
{
	return (uint8_t)atomic_get(&s_hb_mode);
}

void zephcore_leds_set_hb_mode(uint8_t mode)
{
	atomic_set(&s_hb_mode, mode);
}

bool zephcore_led_radio_holds_pin(void)
{
	return atomic_get(&s_radio_holds_pin) != 0;
}

void zephcore_led_radio_hold_pin(bool held)
{
	atomic_set(&s_radio_holds_pin, held ? 1 : 0);
}

/* Same cross-thread readers/writers as the gate above (heartbeat work queue,
 * TX path, CLI) — atomic for the same reason. RAM-only on purpose: no
 * savePrefs() call anywhere near this, see led_gate.h. */
static atomic_t s_led_brightness_pct = ATOMIC_INIT(ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT);

uint8_t zephcore_led_brightness_pct(void)
{
	return (uint8_t)atomic_get(&s_led_brightness_pct);
}

void zephcore_led_set_brightness_pct(uint8_t pct)
{
	if (pct > 100) {
		pct = 100;
	}
	atomic_set(&s_led_brightness_pct, pct);
}
