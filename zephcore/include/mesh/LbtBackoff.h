#pragma once

/* Post-busy retry delay (Mesh::getCADFailRetryDelay()). A random whole number
 * of slots in a fixed window: one typical packet's airtime, counted from the
 * end of the reception the node waited for. It does not widen with refusals:
 * only the loser would widen, which halves its chance in the next gap.
 * Design and numbers: devdocs/lld/02-core-mesh.md §18-§19. Zephyr-free so the
 * host tests build it. */

#include <stdint.h>

namespace mesh {

/* Packet length whose airtime sizes the window: about a median mesh packet
 * (398 ms, 33 slots at SF7/62.5 kHz). */
#define LBT_BACKOFF_TYPICAL_LEN      100
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
