/*
 * SPDX-License-Identifier: Apache-2.0
 * Adaptive Contention Window — replaces static txdelay/rxdelay
 *
 * Measures local retransmit contention by counting how many times
 * we hear the same flood packet retransmitted by neighbors within
 * a 10-second window after we decide to retransmit it ourselves.
 * Feeds dupe counts into a rolling EMA to produce an adaptive
 * delay factor for future retransmits.
 */

#pragma once

#include <stdint.h>

namespace mesh {

class Packet;

class ContentionTracker {
public:
	ContentionTracker();

	/* Cheap 32-bit hash for packet correlation (FNV-1a).
	 * NOT the same as the SHA256 used for dedup — this is only
	 * for matching packets in the 16-entry ring buffer. */
	static uint32_t computePacketHash32(const Packet *pkt);

	/* Called when we decide to retransmit a flood packet. */
	void trackRetransmit(uint32_t hash32, uint32_t now_ms);

	/* Called for every received flood packet. Returns true if
	 * the packet matched a tracked retransmit (dupe recorded).
	 * Caller should attempt reactive backoff when true. */
	bool recordDupeIfTracked(uint32_t hash32, uint32_t now_ms);

	/* Check if reactive extension is within cap for this entry.
	 * Returns the max additional ms allowed, 0 if cap reached. */
	uint16_t getReactiveHeadroom(uint32_t hash32, uint32_t airtime_ms) const;

	/* Record that we added reactive extension to this entry. */
	void addReactiveExtension(uint32_t hash32, uint16_t added_ms);

	/* Finalize expired entries into EMA. Call from maintenanceLoop. */
	void tick(uint32_t now_ms);

	/* Current contention estimate (EMA of dupes per retransmitted packet). */
	float getContentionEstimate() const;

	/* Adaptive delay factor for flood retransmits.
	 * sqrt curve: 0.05 + 0.116 * sqrt(est), cap 2.0.
	 * Returns 0.5 during warmup. */
	float getFloodDelayFactor() const;

	bool isWarmedUp() const { return _finalized_count >= WARMUP_PACKETS; }

	void setBackoffMultiplier(float m) { _backoff_multiplier = m; }
	float getBackoffMultiplier() const { return _backoff_multiplier; }

private:
	static constexpr int RING_SIZE = 16;
	static constexpr uint32_t WINDOW_MS = 10000;
	static constexpr int EMA_SHIFT = 3;          /* alpha = 1/8 */
	static constexpr int WARMUP_PACKETS = 4;
	static constexpr float MIN_FLOOD_FACTOR = 0.05f;
	static constexpr float FLOOD_SCALE = 0.116f; /* (0.5 - 0.05) / sqrt(15) */
	static constexpr float MAX_FLOOD_FACTOR = 2.0f;
	static constexpr float DEFAULT_BACKOFF_MULT = 0.5f;
	static constexpr uint32_t STALE_MS = 300000;  /* 5 minutes */

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
