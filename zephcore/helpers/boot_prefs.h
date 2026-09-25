/*
 * ZephCore - apply persisted prefs to hardware at boot
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Shared by the repeater, room server and observer mains.  Header-only so it
 * logs under the including file's log module.
 */

#ifndef ZEPHCORE_BOOT_PREFS_H
#define ZEPHCORE_BOOT_PREFS_H

#include <zephyr/logging/log.h>
#include <helpers/NodePrefs.h>
#include "led_gate.h"
#include "ZephyrGPSManager.h"

/* Push the prefs that drive hardware directly (LED gate, GPS duty) into their
 * managers.  Ordering is load-bearing:
 *   - after loadPrefs(): before it the struct still holds the initNodePrefs()
 *     defaults (leds on, gps_interval=300, the companion value).  On
 *     repeater/room server loadPrefs() is also where the 48 h GPS default is
 *     set and persisted.  Calling this too early is the bug class where CLI
 *     readback is correct and the hardware disagrees.
 *   - after ui_init(), so the heartbeat cycle exists to be stopped.
 *   - after the GPS fix/event callbacks are registered, because
 *     gps_enable() starts acquisition immediately.
 * gps_time_sync: put GPS in repeater (time-sync-only) mode and apply
 * prefs.gps_enabled, as upstream's applyGpsPrefs().  Only roles that own the
 * GPS pass true. */
static inline void apply_boot_prefs(const NodePrefs *p, bool gps_time_sync)
{
	bool leds_off = p->leds_disabled != 0;

	zephcore_leds_set_disabled(leds_off);
	zephcore_leds_set_radio_mode(p->leds_radio_mode);
	zephcore_leds_set_hb_mode(p->leds_hb_mode);
	LOG_INF("LEDs: %s (from prefs)", leds_off ? "disabled" : "enabled");

	if (gps_time_sync && gps_is_available()) {
		gps_set_poll_interval_sec(p->gps_interval);
		gps_set_repeater_mode(true);
		if (p->gps_enabled) {
			gps_enable(true);
		} else {
			gps_ensure_power_state(false);
		}
	}
}

#endif /* ZEPHCORE_BOOT_PREFS_H */
