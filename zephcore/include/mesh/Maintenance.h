/*
 * SPDX-License-Identifier: MIT
 * Deadline-driven maintenance scheduling.
 *
 * maintenanceLoop() used to be driven by a fixed periodic tick (the 5 s
 * "housekeeping" timer).  Every item inside it is really either a deadline
 * (AGC reset, CAD probe, advert timers, tempradio revert) or a sampler with
 * its own interval (noise floor) — nothing needs a poll.  The msUntilNext*()
 * queries below let the event loop ask "when does maintenance next have work?"
 * and arm a single one-shot wake for exactly that moment, so an otherwise idle
 * node sleeps until its soonest real deadline instead of waking on a fixed
 * cadence.
 *
 * Contract: every msUntilNext*() returns milliseconds from now, 0 when work is
 * due right now, and MAINTENANCE_IDLE when the item has nothing pending at all.
 * Callers combine them with maintenanceSooner().
 */

#pragma once

#include <stdint.h>

namespace mesh {

/* "No deadline pending." Deliberately not UINT32_MAX so that callers can add
 * a modest slack to a returned deadline without wrapping. */
static const uint32_t MAINTENANCE_IDLE = 0x7FFFFFFFu;

/* Fold one deadline into a running minimum. */
static inline uint32_t maintenanceSooner(uint32_t a, uint32_t b)
{
	return (a < b) ? a : b;
}

/* Milliseconds from now until `deadline` (a millisecond-clock timestamp),
 * saturating at 0 when it has already passed.  Wrap-safe: the subtraction is
 * done in unsigned arithmetic and the sign is taken from the wrapped result,
 * matching Dispatcher::millisHasNowPassed(). */
static inline uint32_t maintenanceUntil(uint32_t now_ms, uint32_t deadline_ms)
{
	int32_t remaining = (int32_t)(deadline_ms - now_ms);

	if (remaining <= 0) {
		return 0;
	}
	return ((uint32_t)remaining < MAINTENANCE_IDLE) ? (uint32_t)remaining
						       : MAINTENANCE_IDLE;
}

} /* namespace mesh */
