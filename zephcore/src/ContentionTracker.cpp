/*
 * SPDX-License-Identifier: MIT
 * Adaptive Contention Window — dupe-counting based delay estimation
 */

#include <mesh/ContentionTracker.h>
#include <mesh/Packet.h>
#include <string.h>

namespace mesh {

ContentionTracker::ContentionTracker()
	: _next_idx(0), _ema_x256(0), _finalized_count(0),
	  _last_retransmit_ms(0), _last_decay_ms(0),
	  _backoff_multiplier(DEFAULT_BACKOFF_MULT)
{
	memset(_ring, 0, sizeof(_ring));
}

/* FNV-1a over payload_type + first 8 payload bytes */
uint32_t ContentionTracker::computePacketHash32(const Packet *pkt)
{
	uint32_t h = 0x811c9dc5u; /* FNV-1a offset basis */
	uint8_t t = pkt->getPayloadType();
	h = (h ^ t) * 0x01000193u;
	int n = pkt->payload_len < 8 ? pkt->payload_len : 8;
	for (int i = 0; i < n; i++) {
		h = (h ^ pkt->payload[i]) * 0x01000193u;
	}
	return h;
}

int ContentionTracker::findEntry(uint32_t hash32) const
{
	for (int i = 0; i < RING_SIZE; i++) {
		if (_ring[i].active && _ring[i].hash32 == hash32) {
			return i;
		}
	}
	return -1;
}

void ContentionTracker::finalizeEntry(int idx)
{
	if (!_ring[idx].active) return;

	uint32_t sample_x256 = (uint32_t)_ring[idx].dupe_count << 8;

	int32_t diff = (int32_t)sample_x256 - (int32_t)_ema_x256;

	if (_finalized_count < WARMUP_PACKETS) {
		/* Warmup: seed EMA with fast convergence */
		if (_finalized_count == 0) {
			_ema_x256 = sample_x256;
		} else {
			_ema_x256 = (uint32_t)((int32_t)_ema_x256 + (diff >> 1));
		}
	} else {
		_ema_x256 = (uint32_t)((int32_t)_ema_x256 + (diff >> EMA_SHIFT));
	}

	_finalized_count++;
	_ring[idx].active = false;
}

void ContentionTracker::trackRetransmit(uint32_t hash32, uint32_t now_ms)
{
	_last_retransmit_ms = now_ms;

	/* Evict oldest if ring slot occupied */
	if (_ring[_next_idx].active) {
		finalizeEntry(_next_idx);
	}

	Entry &e = _ring[_next_idx];
	e.hash32 = hash32;
	e.first_seen_ms = now_ms;
	e.dupe_count = 0;
	e.reactive_added_ms = 0;
	e.active = true;

	_next_idx = (_next_idx + 1) % RING_SIZE;
}

bool ContentionTracker::recordDupeIfTracked(uint32_t hash32, uint32_t now_ms)
{
	int idx = findEntry(hash32);
	if (idx < 0) return false;

	Entry &e = _ring[idx];

	if (now_ms - e.first_seen_ms > WINDOW_MS) {
		finalizeEntry(idx);
		return false;
	}

	if (e.dupe_count < 255) {
		e.dupe_count++;
	}
	return true;
}

int ContentionTracker::extractDupeCount(uint32_t hash32)
{
	int idx = findEntry(hash32);
	if (idx < 0) return -1;
	int count = (int)_ring[idx].dupe_count;
	finalizeEntry(idx);  /* folds into EMA, marks inactive */
	return count;
}

uint16_t ContentionTracker::getReactiveHeadroom(uint32_t hash32, uint32_t airtime_ms) const
{
	int idx = findEntry(hash32);
	if (idx < 0) return 0;

	uint32_t per_dupe = (uint32_t)(_backoff_multiplier * (float)airtime_ms);
	if (per_dupe == 0) return 0;

	/* Effective cap: ~12 relay-slots (airtime-scaled), absolute ceiling REACTIVE_HARD_CAP_MS */
	uint32_t effective_cap = 12 * airtime_ms;
	if (effective_cap > REACTIVE_HARD_CAP_MS) effective_cap = REACTIVE_HARD_CAP_MS;

	if (_ring[idx].reactive_added_ms >= effective_cap) return 0;

	uint32_t remaining = effective_cap - _ring[idx].reactive_added_ms;
	if (per_dupe > remaining) per_dupe = remaining;
	return per_dupe > 0xFFFF ? 0xFFFF : (uint16_t)per_dupe;
}

void ContentionTracker::addReactiveExtension(uint32_t hash32, uint16_t added_ms)
{
	int idx = findEntry(hash32);
	if (idx < 0) return;

	uint32_t total = (uint32_t)_ring[idx].reactive_added_ms + added_ms;
	_ring[idx].reactive_added_ms = total > 0xFFFF ? 0xFFFF : (uint16_t)total;
}

void ContentionTracker::tick(uint32_t now_ms)
{
	for (int i = 0; i < RING_SIZE; i++) {
		if (_ring[i].active && now_ms - _ring[i].first_seen_ms > WINDOW_MS) {
			finalizeEntry(i);
		}
	}

	/* Decay EMA toward 0 if no retransmit in STALE_MS.  Paced by wall clock
	 * (one 1/8 step per DECAY_PERIOD_MS) rather than one step per call, so
	 * the decay rate no longer depends on how often the event loop happens
	 * to call us — see DECAY_PERIOD_MS. */
	if (_last_retransmit_ms != 0 && now_ms - _last_retransmit_ms > STALE_MS &&
	    _ema_x256 > 0) {
		if (_last_decay_ms == 0) {
			/* Arm one full period in the PAST, so the loop below
			 * applies a step on this very call.  The old code decayed
			 * immediately on the first tick that observed staleness;
			 * arming at now_ms instead would make the first check
			 * (0 < DECAY_PERIOD_MS) break without decaying, deferring
			 * onset by a whole period on every stale transition — and
			 * would let the reset branch below starve decay entirely
			 * for traffic that goes stale and un-stale repeatedly.
			 * Unsigned wraparound makes this exact even near zero. */
			_last_decay_ms = now_ms - DECAY_PERIOD_MS;
		}
		for (int n = 0; n < MAX_DECAY_CATCHUP; n++) {
			if (now_ms - _last_decay_ms < DECAY_PERIOD_MS ||
			    _ema_x256 == 0) {
				break;
			}
			_ema_x256 -= _ema_x256 >> EMA_SHIFT;
			_last_decay_ms += DECAY_PERIOD_MS;
		}
	} else {
		/* Not decaying — restart the phase next time we are. */
		_last_decay_ms = 0;
	}
}

uint32_t ContentionTracker::msUntilNextTick(uint32_t now_ms) const
{
	uint32_t next = MAINTENANCE_IDLE;

	/* Soonest ring entry to fall out of the observation window. */
	for (int i = 0; i < RING_SIZE; i++) {
		if (!_ring[i].active) {
			continue;
		}
		next = maintenanceSooner(
			next, maintenanceUntil(now_ms,
					       _ring[i].first_seen_ms + WINDOW_MS + 1));
	}

	/* Stale-decay step, only while there is EMA left to decay. */
	if (_last_retransmit_ms != 0 && _ema_x256 > 0) {
		uint32_t stale_at = _last_retransmit_ms + STALE_MS + 1;

		if (now_ms - _last_retransmit_ms > STALE_MS) {
			/* Already stale: next step is one period after the last
			 * one (or immediately, if we have not started yet). */
			next = maintenanceSooner(
				next, _last_decay_ms == 0
					      ? 0
					      : maintenanceUntil(now_ms,
								 _last_decay_ms +
									 DECAY_PERIOD_MS));
		} else {
			next = maintenanceSooner(next,
						 maintenanceUntil(now_ms, stale_at));
		}
	}

	return next;
}

float ContentionTracker::getContentionEstimate() const
{
	return (float)_ema_x256 / 256.0f;
}

float ContentionTracker::getFloodDelayFactor() const
{
	if (!isWarmedUp()) return 0.5f; /* conservative default before warmup */

	float est = getContentionEstimate();
	if (est <= 0.0f) return MIN_FLOOD_FACTOR;

	/* Arduino-like center near 0.5 in light contention, rising smoothly
	 * toward 0.8 as contention increases. */
	float factor = MIN_FLOOD_FACTOR + (MAX_FLOOD_FACTOR - MIN_FLOOD_FACTOR) *
		       (est / (est + FLOOD_EST_HALFPOINT));
	if (factor > MAX_FLOOD_FACTOR) factor = MAX_FLOOD_FACTOR;
	return factor;
}

} /* namespace mesh */
