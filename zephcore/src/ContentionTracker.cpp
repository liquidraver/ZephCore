/*
 * SPDX-License-Identifier: Apache-2.0
 * Adaptive Contention Window — dupe-counting based delay estimation
 */

#include <mesh/ContentionTracker.h>
#include <mesh/Packet.h>
#include <math.h>
#include <string.h>

namespace mesh {

ContentionTracker::ContentionTracker()
	: _next_idx(0), _ema_x256(0), _finalized_count(0),
	  _last_retransmit_ms(0), _backoff_multiplier(DEFAULT_BACKOFF_MULT)
{
	memset(_ring, 0, sizeof(_ring));
}

/* FNV-1a hash of payload_type + first 8 bytes of payload.
 * Cheap and sufficient for 16-entry correlation. */
uint32_t ContentionTracker::computePacketHash32(const Packet *pkt)
{
	uint32_t h = 0x811c9dc5u; /* FNV offset basis */
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
		/* During warmup, seed the EMA directly */
		if (_finalized_count == 0) {
			_ema_x256 = sample_x256;
		} else {
			_ema_x256 = (uint32_t)((int32_t)_ema_x256 + (diff >> 1));
		}
	} else {
		/* Normal EMA update: ema += (sample - ema) >> shift */
		_ema_x256 = (uint32_t)((int32_t)_ema_x256 + (diff >> EMA_SHIFT));
	}

	_finalized_count++;
	_ring[idx].active = false;
}

void ContentionTracker::trackRetransmit(uint32_t hash32, uint32_t now_ms)
{
	_last_retransmit_ms = now_ms;

	/* If ring is full, finalize the oldest active entry */
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

	/* Check if entry has expired */
	if (now_ms - e.first_seen_ms > WINDOW_MS) {
		finalizeEntry(idx);
		return false;
	}

	if (e.dupe_count < 255) {
		e.dupe_count++;
	}
	return true;
}

uint16_t ContentionTracker::getReactiveHeadroom(uint32_t hash32, uint32_t airtime_ms) const
{
	int idx = findEntry(hash32);
	if (idx < 0) return 0;

	uint32_t cap = (uint32_t)(_backoff_multiplier * (float)airtime_ms);
	if (_ring[idx].reactive_added_ms >= cap) return 0;

	uint32_t remaining = cap - _ring[idx].reactive_added_ms;
	return remaining > 0xFFFF ? 0xFFFF : (uint16_t)remaining;
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
	/* Finalize expired entries */
	for (int i = 0; i < RING_SIZE; i++) {
		if (_ring[i].active && now_ms - _ring[i].first_seen_ms > WINDOW_MS) {
			finalizeEntry(i);
		}
	}

	/* Staleness decay: if no retransmit in 5 minutes, decay toward 0 */
	if (_last_retransmit_ms != 0 && now_ms - _last_retransmit_ms > STALE_MS) {
		if (_ema_x256 > 0) {
			_ema_x256 -= _ema_x256 >> EMA_SHIFT;
		}
	}
}

float ContentionTracker::getContentionEstimate() const
{
	return (float)_ema_x256 / 256.0f;
}

float ContentionTracker::getFloodDelayFactor() const
{
	if (!isWarmedUp()) return 0.5f;

	float est = getContentionEstimate();
	if (est <= 0.0f) return MIN_FLOOD_FACTOR;

	float factor = MIN_FLOOD_FACTOR + FLOOD_SCALE * sqrtf(est);
	if (factor > MAX_FLOOD_FACTOR) factor = MAX_FLOOD_FACTOR;
	return factor;
}

} /* namespace mesh */
