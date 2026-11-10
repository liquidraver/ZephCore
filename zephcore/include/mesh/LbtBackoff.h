#pragma once

/* Post-busy retry delay (Mesh::getCADFailRetryDelay()). A random whole number
 * of slots in a fixed window of LBT_BACKOFF_SLOTS, counted from the end of the
 * reception the node waited for. It does not widen with refusals: only the
 * loser would widen, which halves its chance in the next gap. Design and
 * numbers: devdocs/lld/02-core-mesh.md §18-§20. Zephyr-free so the host tests
 * build it. */

#include <stdint.h>

namespace mesh {

/* Window in slots (120 ms at SF7/62.5 kHz): sized to the measured gaps
 * between repeated packets on a busy site, half of which close within
 * 100 ms (LLD 02 §20). */
#define LBT_BACKOFF_SLOTS            10
/* Slot when the radio cannot say (no symbol time known). */
#define LBT_BACKOFF_SLOT_FALLBACK_MS 10

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
