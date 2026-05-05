/*
 * SPDX-License-Identifier: Apache-2.0
 * Adaptive Power Control (APC) — echo-based TX power reduction
 *
 * Measures link margin by tracking echo packets (flood dupes of
 * packets we sent or retransmitted, heard back from neighbors).
 * Feeds per-packet margins into a rolling EMA to produce an
 * adaptive TX power reduction in dBm.
 *
 * Rogue filtering: when 2+ distinct neighbors echo the same packet,
 * clusters their SNRs within 6 dB of the best.  An isolated high
 * outlier (rogue, badly placed neighbor) is dropped.
 */

#pragma once

#include <stdint.h>

namespace mesh {

class Packet;

class PowerController {
public:
	PowerController();

	/* Enable/disable APC. When disabled, APC tracking/math is bypassed. */
	void setEnabled(bool en);
	bool isEnabled() const { return _enabled; }

	/* Set current spreading factor (needed for margin calculation). */
	void setSF(uint8_t sf) { _sf = sf; }

	/* Set target link margin in dB.  Higher = more conservative
	 * (better for networks with poor-RX hardware).  Default 16 dB. */
	void setTargetMargin(uint8_t margin_db) { _target_margin_x4 = (int)margin_db * 4; }
	uint8_t getTargetMargin() const { return (uint8_t)(_target_margin_x4 / 4); }

	/* Called when we send or retransmit a flood packet. */
	void trackTransmit(uint32_t hash32, uint32_t now_ms);

	/* Called for every received flood dupe.  Updates per-source best
	 * SNR and source diversity.  Returns true if the dupe matched a
	 * tracked transmit. */
	bool recordEcho(uint32_t hash32, int8_t snr_x4,
			uint8_t first_hop_hash, uint32_t now_ms);

	/* Finalize expired entries, update EMA, adjust power, handle
	 * staleness.  Call from maintenanceLoop (~5 s). */
	void tick(uint32_t now_ms);

	/* Current TX power reduction in dBm (0 to MAX_REDUCTION_DB).
	 * Returns 0 when disabled. */
	int8_t getPowerReduction() const { return _enabled ? _power_reduction_db : 0; }

	/* Current margin estimate in dB (for diagnostics). */
	float getMarginEstimate() const;

	/* Source count from most recently finalized entry (diagnostics). */
	uint8_t getLastSourceCount() const { return _last_source_count; }

	bool isWarmedUp() const { return _finalized_count >= WARMUP_COUNT; }
	bool isStale(uint32_t now_ms) const;

private:
	static constexpr int RING_SIZE = 16;
	static constexpr uint32_t ECHO_WINDOW_MS = 10000;  /* 10s: covers SF12 2-hop echo */
	static constexpr uint32_t STALE_MS = 120000;       /* 2 min */
	static constexpr int EMA_SHIFT = 2;                 /* alpha = 1/4 */
	static constexpr int WARMUP_COUNT = 3;
	static constexpr int MAX_SOURCES = 3;
	static constexpr int8_t STEP_DOWN_DB = 3;
	static constexpr int8_t STEP_UP_DB = 6;
	static constexpr int8_t MAX_REDUCTION_DB = 12;
	static constexpr int8_t MIN_TX_POWER_DBM = -9;      /* SX1262 hw min */
	static constexpr int CLUSTER_WIDTH_X4 = 24;          /* 6 dB in x4 */
	static constexpr int DEFAULT_TARGET_MARGIN_X4 = 64;  /* 16 dB * 4 */
	static constexpr int HYSTERESIS_X4 = 4;              /* 1 dB * 4 */

	struct Source {
		uint8_t hash;
		int8_t snr_x4;
	};

	struct EchoEntry {
		uint32_t hash32;
		uint32_t timestamp_ms;
		uint8_t source_count;
		uint8_t sf_at_track;    /* SF when packet was transmitted */
		Source sources[MAX_SOURCES];
		bool active;
	};

	EchoEntry _ring[RING_SIZE];
	int _next_idx;
	int32_t _margin_ema_x256;       /* fixed-point EMA (x4 * 256) */
	int _finalized_count;
	uint32_t _last_echo_ms;
	int8_t _power_reduction_db;
	bool _enabled;
	uint8_t _sf;
	uint8_t _last_source_count;
	int _target_margin_x4;

	void finalizeEntry(int idx);
	int findEntry(uint32_t hash32) const;
	int8_t computeRobustSNR(const EchoEntry &entry) const;

	/* SNR threshold for a given SF (x4 fixed point). */
	static int8_t sfThresholdX4(uint8_t sf);
};

} /* namespace mesh */
