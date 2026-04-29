/*
 * SPDX-License-Identifier: Apache-2.0
 * Adaptive Contention Window — EMA-based flood retransmit delay
 *
 * Counts neighbor retransmit dupes within a 10s window per packet.
 * Dupe counts feed a rolling EMA that drives an adaptive delay factor.
 */

#pragma once

#include <stdint.h>

namespace mesh {

class Packet;

class ContentionTracker {
public:
	ContentionTracker();

	/* FNV-1a 32-bit hash for ring buffer correlation (not dedup SHA256). */
	static uint32_t computePacketHash32(const Packet *pkt);

	void trackRetransmit(uint32_t hash32, uint32_t now_ms);

	/* Returns true if packet matched a tracked retransmit (dupe recorded). */
	bool recordDupeIfTracked(uint32_t hash32, uint32_t now_ms);

	/* Returns backoff_multiplier * airtime, clamped by remaining headroom.
	 * Returns 0 when hard cap reached or backoff disabled. */
	uint16_t getReactiveHeadroom(uint32_t hash32, uint32_t airtime_ms) const;

	void addReactiveExtension(uint32_t hash32, uint16_t added_ms);

	/* Finalize expired entries into EMA. */
	void tick(uint32_t now_ms);

	float getContentionEstimate() const;

	/* Saturating curve: MIN + (MAX-MIN) * est/(est+HALFPOINT).
	 * Returns 0.5 during warmup. */
	float getFloodDelayFactor() const;

	bool isWarmedUp() const { return _finalized_count >= WARMUP_PACKETS; }

	void setBackoffMultiplier(float m) { _backoff_multiplier = m; }
	float getBackoffMultiplier() const { return _backoff_multiplier; }

private:
	static constexpr int RING_SIZE = 24;              /* max concurrent tracked retransmits */
	static constexpr uint32_t WINDOW_MS = 10000;      /* dupe observation window; covers SF12 2-hop */
	static constexpr int EMA_SHIFT = 3;               /* alpha = 1/8 */
	static constexpr int WARMUP_PACKETS = 4;          /* min samples before EMA is trusted */
	static constexpr float MIN_FLOOD_FACTOR = 0.40f;  /* sparse mesh baseline */
	static constexpr float MAX_FLOOD_FACTOR = 1.00f;  /* dense mesh ceiling */
	static constexpr float FLOOD_EST_HALFPOINT = 4.0f; /* midpoint: factor=0.60 at est=4 */
	static constexpr float DEFAULT_BACKOFF_MULT = 0.2f; /* airtime*0.2 per dupe heard */
	static constexpr uint32_t REACTIVE_HARD_CAP_MS = 2000; /* max cumulative reactive extension */
	static constexpr uint32_t STALE_MS = 300000;      /* 5 min: reset EMA if no traffic */

	struct Entry {
		uint32_t hash32;
		uint32_t first_seen_ms;
		uint8_t dupe_count;
		uint16_t reactive_added_ms;
		bool active;
	};

	Entry _ring[RING_SIZE];
	int _next_idx;
	uint32_t _ema_x256;
	int _finalized_count;
	uint32_t _last_retransmit_ms;
	float _backoff_multiplier;

	void finalizeEntry(int idx);
	int findEntry(uint32_t hash32) const;
};

} /* namespace mesh */
