#pragma once

/* Post-busy retry delay (Mesh::getCADFailRetryDelay()). A random whole number
 * of slots in a window sized to one typical packet's airtime, doubling with
 * each consecutive LBT refusal. Design and numbers: devdocs/lld/02-core-mesh.md
 * §18. Zephyr-free so the host tests build it. */

#include <stdint.h>

namespace mesh {

/* Packet length whose airtime sizes the base window: about a median mesh
 * packet, so a deferral expects the foreign packet to last about this long. */
#define LBT_BACKOFF_TYPICAL_LEN      100
/* Window doublings: 1x, 2x, 4x the base. */
#define LBT_BACKOFF_MAX_EXP          2
/* Growth stops here, unless one typical packet is already longer. */
#define LBT_BACKOFF_CAP_MS           2000
/* Slot when the radio cannot say (no symbol time known). */
#define LBT_BACKOFF_SLOT_FALLBACK_MS 10

static inline uint32_t lbtBackoffWindowMs(uint32_t typical_airtime_ms, uint8_t refusals)
{
	uint8_t exp = refusals < LBT_BACKOFF_MAX_EXP ? refusals : LBT_BACKOFF_MAX_EXP;
	uint32_t window = typical_airtime_ms << exp;
	uint32_t cap = typical_airtime_ms > LBT_BACKOFF_CAP_MS ? typical_airtime_ms
							       : LBT_BACKOFF_CAP_MS;

	return window < cap ? window : cap;
}

/* rnd: any uniformly random 32-bit value. Result is in [slot, window]. */
static inline uint32_t lbtBackoffDelayMs(uint32_t typical_airtime_ms, uint32_t slot_ms,
					 uint8_t refusals, uint32_t rnd)
{
	if (slot_ms == 0) {
		slot_ms = LBT_BACKOFF_SLOT_FALLBACK_MS;
	}
	uint32_t slots = lbtBackoffWindowMs(typical_airtime_ms, refusals) / slot_ms;

	if (slots < 2) {
		slots = 2;
	}
	return slot_ms * (1U + rnd % slots);
}

} /* namespace mesh */
