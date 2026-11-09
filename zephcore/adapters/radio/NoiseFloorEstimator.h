/*
 * SPDX-License-Identifier: MIT
 * Noise-floor estimator: an EMA over idle-channel RSSI samples.
 *
 * Pure algorithm. LoRaRadio owns the sampler (when to read, the
 * idle-RX guards, the median-of-N burst) and feeds each median in here.
 */

#pragma once

#include <stdint.h>
#include "radio_tuning.h"

namespace mesh {

/* Sorts `s` in place and returns the median (mean of the two middle values
 * for even n). Insertion sort: n is 8. */
inline int16_t sortAndMedian(int16_t *s, int n)
{
	for (int i = 1; i < n; i++) {
		int16_t key = s[i];
		int j = i - 1;
		while (j >= 0 && s[j] > key) {
			s[j + 1] = s[j];
			j--;
		}
		s[j + 1] = key;
	}
	return (int16_t)((s[n / 2 - 1] + s[n / 2]) / 2);
}

class NoiseFloorEstimator {
public:
	enum Result { SEEDED, REJECTED, UPDATED };

	/* DEFAULT_NOISE_FLOOR (0) until the first sample. */
	int floor() const { return _floor; }
	bool seeded() const { return _floor != DEFAULT_NOISE_FLOOR; }
	/* Ticks since seeding (wraps at 255; only the low bits matter). */
	uint8_t tick() const { return _unguarded; }

	/* Is `rssi` at the floor? Judged against the floor before the sample is
	 * folded in; an unseeded estimator calls everything quiet. */
	bool isQuiet(int rssi) const
	{
		return !seeded() || rssi <= _floor + CAD_PROBE_RSSI_GUARD;
	}

	/* Should the sampler read again before feeding `rssi` in? Yes for a
	 * seed and for a reading NOISE_FLOOR_CONFIRM_DB or more below the floor:
	 * the likely cause is the RSSI start-up dip, not a quieter channel. */
	bool wantsConfirm(int rssi) const
	{
		return !seeded() || rssi <= _floor - NOISE_FLOOR_CONFIRM_DB;
	}

	/* Fold one sample in, clamped to NOISE_FLOOR_CEILING_DBM. A sample below
	 * floor_min (thermal noise for the bandwidth, 10*log10(BW)) cannot be a
	 * channel reading and is rejected, never clamped in.
	 *
	 * The first W = 2^EMA_SHIFT ticks after the seed accept every sample and
	 * weigh them equally (running mean, 1/2 .. 1/W), so a bad seed neither
	 * locks the real floor out nor lingers; after that the EMA runs at 1/W,
	 * and samples more than SAMPLING_THRESHOLD above the floor are rejected
	 * as interference, except every Pth tick, so a sustained rise is still
	 * tracked. */
	Result update(int rssi, int floor_min)
	{
		if (rssi < floor_min) {
			return REJECTED;
		}
		if (!seeded()) {
			_floor = clamp(rssi, floor_min);
			_unguarded = 0;
			return SEEDED;
		}

		const int W = (1 << NOISE_FLOOR_EMA_SHIFT);
		const int P = NOISE_FLOOR_UNGUARDED_INTERVAL;
		bool warmup = (_unguarded < W);
		bool periodic = (!warmup && (_unguarded & (P - 1)) == 0);
		int div = (warmup && _unguarded + 2 < W) ? _unguarded + 2 : W;
		_unguarded++;

		if (!warmup && !periodic &&
		    rssi >= _floor + NOISE_FLOOR_SAMPLING_THRESHOLD) {
			return REJECTED;
		}

		/* floor += round_nearest((sample - floor) / div), symmetric in sign:
		 * plain >> biases down and plain / has a +-(div-1) dead zone. */
		int diff = rssi - _floor;
		int half = div / 2;
		_floor = clamp(_floor + (diff + (diff > 0 ? half : -half)) / div, floor_min);
		return UPDATED;
	}

private:
	static int clamp(int v, int floor_min)
	{
		if (v < floor_min) v = floor_min;
		if (v > NOISE_FLOOR_CEILING_DBM) v = NOISE_FLOOR_CEILING_DBM;
		return v;
	}

	int _floor = DEFAULT_NOISE_FLOOR;
	uint8_t _unguarded = 0;
};

} /* namespace mesh */
