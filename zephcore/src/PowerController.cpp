/*
 * SPDX-License-Identifier: Apache-2.0
 * Adaptive Power Control — echo-based TX power reduction
 */

#include <mesh/PowerController.h>
#include <mesh/Packet.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_apc, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* SNR thresholds per SF (x4 fixed point, matching radio_common.h) */
static constexpr int8_t snr_threshold_x4[] = {
	-30, /* SF7:  -7.5 dB */
	-40, /* SF8: -10.0 dB */
	-50, /* SF9: -12.5 dB */
	-60, /* SF10: -15.0 dB */
	-70, /* SF11: -17.5 dB */
	-80, /* SF12: -20.0 dB */
};

namespace mesh {

PowerController::PowerController()
	: _next_idx(0), _margin_ema_x256(0), _finalized_count(0),
	  _last_echo_ms(0), _power_reduction_db(0), _enabled(true),
	  _sf(8), _last_source_count(0), _target_margin_x4(DEFAULT_TARGET_MARGIN_X4)
{
	memset(_ring, 0, sizeof(_ring));
}

void PowerController::setEnabled(bool en)
{
	if (_enabled == en) return;
	_enabled = en;
	if (!_enabled) {
		/* Drop APC runtime state while disabled so no tracking work runs. */
		memset(_ring, 0, sizeof(_ring));
		_next_idx = 0;
		_margin_ema_x256 = 0;
		_finalized_count = 0;
		_last_echo_ms = 0;
		_last_source_count = 0;
		_power_reduction_db = 0;
	}
}

int8_t PowerController::sfThresholdX4(uint8_t sf)
{
	int idx = (int)sf - 7;
	if (idx < 0) idx = 0;
	if (idx > 5) idx = 5;
	return snr_threshold_x4[idx];
}

int PowerController::findEntry(uint32_t hash32) const
{
	for (int i = 0; i < RING_SIZE; i++) {
		if (_ring[i].active && _ring[i].hash32 == hash32) {
			return i;
		}
	}
	return -1;
}

void PowerController::trackTransmit(uint32_t hash32, uint32_t now_ms)
{
	if (!_enabled) return;

	/* If ring slot is occupied, finalize it first */
	if (_ring[_next_idx].active) {
		finalizeEntry(_next_idx);
	}

	EchoEntry &e = _ring[_next_idx];
	e.hash32 = hash32;
	e.timestamp_ms = now_ms;
	e.source_count = 0;
	e.sf_at_track = _sf;
	memset(e.sources, 0, sizeof(e.sources));
	e.active = true;

	_next_idx = (_next_idx + 1) % RING_SIZE;
}

bool PowerController::recordEcho(uint32_t hash32, int8_t snr_x4,
				 uint8_t first_hop_hash, uint32_t now_ms)
{
	if (!_enabled) return false;

	int idx = findEntry(hash32);
	if (idx < 0) return false;

	EchoEntry &e = _ring[idx];

	/* Check if entry has expired */
	if (now_ms - e.timestamp_ms > ECHO_WINDOW_MS) {
		finalizeEntry(idx);
		return false;
	}

	/* Update existing source or add new one */
	for (int i = 0; i < e.source_count; i++) {
		if (e.sources[i].hash == first_hop_hash) {
			if (snr_x4 > e.sources[i].snr_x4) {
				e.sources[i].snr_x4 = snr_x4;
			}
			_last_echo_ms = now_ms;
			return true;
		}
	}

	if (e.source_count < MAX_SOURCES) {
		e.sources[e.source_count].hash = first_hop_hash;
		e.sources[e.source_count].snr_x4 = snr_x4;
		e.source_count++;
	}

	_last_echo_ms = now_ms;
	return true;
}

int8_t PowerController::computeRobustSNR(const EchoEntry &entry) const
{
	if (entry.source_count == 0) {
		return sfThresholdX4(entry.sf_at_track); /* no echo = margin 0 */
	}

	if (entry.source_count == 1) {
		return entry.sources[0].snr_x4;
	}

	/* 2-3 sources: sort descending, then cluster + rogue filter */
	int8_t sorted[MAX_SOURCES];
	int n = entry.source_count;
	for (int i = 0; i < n; i++) {
		sorted[i] = entry.sources[i].snr_x4;
	}
	/* Simple insertion sort (max 3 elements) */
	for (int i = 1; i < n; i++) {
		int8_t key = sorted[i];
		int j = i - 1;
		while (j >= 0 && sorted[j] < key) {
			sorted[j + 1] = sorted[j];
			j--;
		}
		sorted[j + 1] = key;
	}

	/* Count how many are within CLUSTER_WIDTH of the best */
	int cluster_count = 1;
	for (int i = 1; i < n; i++) {
		if (sorted[0] - sorted[i] <= CLUSTER_WIDTH_X4) {
			cluster_count++;
		}
	}

	if (cluster_count >= 2) {
		/* 2+ in cluster: median of the cluster values */
		/* For 2 values: average. For 3 values: middle one. */
		if (cluster_count == 2) {
			return (int8_t)(((int)sorted[0] + (int)sorted[1]) / 2);
		}
		/* cluster_count == 3 (all 3 within 6 dB) */
		return sorted[1]; /* median */
	}

	/* Only 1 in top cluster → rogue. Drop it, use next. */
	if (n >= 3 && sorted[1] - sorted[2] <= CLUSTER_WIDTH_X4) {
		/* sources[1] and [2] cluster together — median them */
		return (int8_t)(((int)sorted[1] + (int)sorted[2]) / 2);
	}
	/* Fall back to second-best */
	return sorted[1];
}

void PowerController::finalizeEntry(int idx)
{
	if (!_ring[idx].active) return;

	EchoEntry &e = _ring[idx];
	_last_source_count = e.source_count;

	int8_t robust_snr = computeRobustSNR(e);
	int32_t margin_x4 = (int32_t)robust_snr - (int32_t)sfThresholdX4(e.sf_at_track);
	/* margin_x4 is in x4 units. Convert to x256 for EMA. */
	int32_t sample_x256 = margin_x4 << 6; /* x4 * 64 = x256 */

	int32_t diff = sample_x256 - _margin_ema_x256;

	if (_finalized_count < WARMUP_COUNT) {
		/* During warmup, seed the EMA faster */
		if (_finalized_count == 0) {
			_margin_ema_x256 = sample_x256;
		} else {
			_margin_ema_x256 += diff >> 1;
		}
	} else {
		/* Normal EMA update: ema += (sample - ema) >> shift */
		_margin_ema_x256 += diff >> EMA_SHIFT;
	}

	_finalized_count++;
	e.active = false;

	LOG_DBG("APC finalize: sources=%d robust_snr=%.1f margin=%.1f ema=%.1f",
		(int)_last_source_count,
		(double)(robust_snr / 4.0f),
		(double)(margin_x4 / 4.0f),
		(double)getMarginEstimate());
}

void PowerController::tick(uint32_t now_ms)
{
	if (!_enabled) return;

	/* Finalize expired entries */
	for (int i = 0; i < RING_SIZE; i++) {
		if (_ring[i].active && now_ms - _ring[i].timestamp_ms > ECHO_WINDOW_MS) {
			finalizeEntry(i);
		}
	}

	if (!isWarmedUp()) return;

	int32_t margin_x256 = _margin_ema_x256;
	int32_t target_x256 = _target_margin_x4 << 6;
	int32_t hyst_x256 = HYSTERESIS_X4 << 6;

	int8_t old_reduction = _power_reduction_db;

	/* Staleness takes priority: ramp back to full power if no echoes.
	 * When stale, never increase reduction — old EMA data is unreliable. */
	if (isStale(now_ms)) {
		if (_power_reduction_db > 0) {
			_power_reduction_db -= STEP_DOWN_DB;
			if (_power_reduction_db < 0) {
				_power_reduction_db = 0;
			}
		}
	} else if (margin_x256 > target_x256 + hyst_x256) {
		/* Margin very good — step down */
		if (_power_reduction_db < MAX_REDUCTION_DB) {
			_power_reduction_db += STEP_DOWN_DB;
			if (_power_reduction_db > MAX_REDUCTION_DB) {
				_power_reduction_db = MAX_REDUCTION_DB;
			}
		}
	} else if (margin_x256 < target_x256 - hyst_x256) {
		/* Margin too low — step up (reduce the reduction) */
		if (_power_reduction_db > 0) {
			_power_reduction_db -= STEP_UP_DB;
			if (_power_reduction_db < 0) {
				_power_reduction_db = 0;
			}
		}
	}

	if (_power_reduction_db != old_reduction) {
		LOG_INF("APC: reduction %d -> %d dBm (margin=%.1f)",
			(int)old_reduction, (int)_power_reduction_db,
			(double)getMarginEstimate());
	}
}

float PowerController::getMarginEstimate() const
{
	return (float)_margin_ema_x256 / 256.0f;
}

bool PowerController::isStale(uint32_t now_ms) const
{
	if (_last_echo_ms == 0) return false;
	return now_ms - _last_echo_ms > STALE_MS;
}

} /* namespace mesh */
