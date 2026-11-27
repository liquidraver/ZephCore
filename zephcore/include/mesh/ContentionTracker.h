/*
 * SPDX-License-Identifier: MIT
 * Contention estimate — EMA of the dupes heard per retransmit
 *
 * Counts neighbor retransmit dupes within a 10s window per packet.
 * The EMA sizes the LBT slot window (mesh/LbtBackoff.h, lbtBackoffSlots()).
 */

#pragma once

#include <stdint.h>
#include <mesh/Maintenance.h>

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

	/* Capture the dupe count for the given hash and remove the entry from
	 * active tracking (also folds the sample into the EMA exactly once,
	 * same as natural finalization would have). Returns the dupe count, or
	 * -1 if the entry isn't found (already finalized, or never tracked).
	 * Used by the joystick UI to ask "did my channel send get repeated?"
	 * a few seconds after sending. */
	int extractDupeCount(uint32_t hash32);

	/* Finalize expired entries into EMA. */
	void tick(uint32_t now_ms);

	/* Milliseconds until tick() next has work to do, or MAINTENANCE_IDLE
	 * when nothing is pending (no tracked entries, EMA already at rest).
	 * Lets the event loop schedule a wake at the deadline instead of
	 * polling: an idle repeater has no tracked retransmits and a decayed
	 * EMA, so this returns MAINTENANCE_IDLE and costs no wakes at all. */
	uint32_t msUntilNextTick(uint32_t now_ms) const;

	float getContentionEstimate() const;

	bool isWarmedUp() const { return _finalized_count >= WARMUP_PACKETS; }

private:
	static constexpr int RING_SIZE = 24;              /* max concurrent tracked retransmits */
	static constexpr uint32_t WINDOW_MS = 10000;      /* dupe observation window; covers SF12 2-hop */
	static constexpr int EMA_SHIFT = 3;               /* alpha = 1/8 */
	static constexpr int WARMUP_PACKETS = 4;          /* min samples before EMA is trusted */
	static constexpr uint32_t STALE_MS = 300000;      /* 5 min: reset EMA if no traffic */
	/* Wall-clock period of one 1/8 stale-decay step.  The decay used to be
	 * "one step per tick()", which silently coupled it to the 5 s
	 * housekeeping cadence; pinning it to 5000 ms keeps exactly that rate
	 * while making it independent of how often tick() is actually called. */
	static constexpr uint32_t DECAY_PERIOD_MS = 5000;
	/* Bound the catch-up loop when tick() is called much later than one
	 * period.  8 steps takes the EMA down by ~66%; anything beyond that is
	 * indistinguishable from rest given the WARMUP/read paths. */
	static constexpr int MAX_DECAY_CATCHUP = 8;

	struct Entry {
		uint32_t hash32;
		uint32_t first_seen_ms;
		uint8_t dupe_count;
		bool active;
	};

	Entry _ring[RING_SIZE];
	int _next_idx;
	uint32_t _ema_x256;
	int _finalized_count;
	uint32_t _last_retransmit_ms;
	uint32_t _last_decay_ms;

	void finalizeEntry(int idx);
	int findEntry(uint32_t hash32) const;
};

} /* namespace mesh */
