/*
 * SPDX-License-Identifier: MIT
 * LoopWakeStats - how often a role's mesh thread leaves k_event_wait(), and
 * for which event bits (CONFIG_ZEPHCORE_LOOP_WAKE_STATS, `get loop.wakes`).
 */

#pragma once

#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdio.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_LOOP_WAKE_STATS)

struct LoopWakeStats {
	uint32_t wakes;
	uint32_t bits[16];  /* per event bit; one wake may carry several */
};

inline LoopWakeStats loop_wake_stats;

/* Call with the events k_event_wait() returned. */
static inline void loopWakeNote(uint32_t events)
{
	loop_wake_stats.wakes++;
	events &= 0xFFFF;
	while (events) {
		int b = __builtin_ctz(events);
		loop_wake_stats.bits[b]++;
		events &= events - 1;
	}
}

/* "> wakes=N up=Ss b0=.. b3=..": counts since boot, non-zero bits only. The
 * bit numbers are the role's MESH_EVENT_* bits. */
static inline void loopWakeFormat(char *reply, size_t cap)
{
	int n = snprintf(reply, cap, "> wakes=%u up=%us", (unsigned)loop_wake_stats.wakes,
			 (unsigned)(k_uptime_get() / 1000));
	for (int b = 0; b < 16 && n > 0 && (size_t)n < cap; b++) {
		if (loop_wake_stats.bits[b]) {
			n += snprintf(reply + n, cap - n, " b%d=%u", b, (unsigned)loop_wake_stats.bits[b]);
		}
	}
}

#else

static inline void loopWakeNote(uint32_t events) { (void)events; }

#endif
