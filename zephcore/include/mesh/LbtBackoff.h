#pragma once

/* LBT slot draw (Mesh::getCADFailRetryDelay()): a random whole number of
 * slots, counted from the end of a reception. Every forward and every retry
 * after a busy CAD waits one draw. The window grows with the contention
 * estimate (copies heard per forward), which neighbours share, never with this
 * node's own refusals: only the loser would widen, which halves its chance in
 * the next gap. Design and numbers: devdocs/lld/02-core-mesh.md §18-§21.
 * Zephyr-free so the host tests build it. */

#include <stdint.h>

namespace mesh {

/* Smallest window in slots (120 ms at SF7/62.5 kHz): sized to the measured
 * gaps between repeated packets on a busy site, half of which close within
 * 100 ms (LLD 02 §20). Also the window before the estimate has warmed up. */
#define LBT_BACKOFF_SLOTS            10
/* Largest window in slots, for dense sites (LLD 02 §21). */
#define LBT_BACKOFF_SLOTS_MAX        64
/* Slot when the radio cannot say (no symbol time known). */
#define LBT_BACKOFF_SLOT_FALLBACK_MS 10

/* Window in slots for a contention estimate: 2 * est + 2, clamped to
 * [LBT_BACKOFF_SLOTS, LBT_BACKOFF_SLOTS_MAX]. est < 0 = not known yet. */
static inline uint32_t lbtBackoffSlots(float est)
{
	if (!(est > 0.0f)) {
		return LBT_BACKOFF_SLOTS;
	}
	if (est >= (float)LBT_BACKOFF_SLOTS_MAX) {
		return LBT_BACKOFF_SLOTS_MAX;
	}
	uint32_t slots = (uint32_t)(2.0f * est) + 2U;

	if (slots < LBT_BACKOFF_SLOTS) {
		return LBT_BACKOFF_SLOTS;
	}
	return slots > LBT_BACKOFF_SLOTS_MAX ? LBT_BACKOFF_SLOTS_MAX : slots;
}

/* rnd: any uniformly random 32-bit value. Result is in [slot, window]. */
static inline uint32_t lbtBackoffDelayMs(uint32_t window_ms, uint32_t slot_ms, uint32_t rnd)
{
	if (slot_ms == 0) {
		slot_ms = LBT_BACKOFF_SLOT_FALLBACK_MS;
	}
	uint32_t slots = window_ms / slot_ms;

	if (slots < 2) {
		slots = 2;
	}
	return slot_ms * (1U + rnd % slots);
}

} /* namespace mesh */
