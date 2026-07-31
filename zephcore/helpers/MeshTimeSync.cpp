/*
 * SPDX-License-Identifier: MIT
 * MeshTimeSync - mesh clock consensus from Ed25519-signed advert timestamps.
 * See MeshTimeSync.h for the role contract, ARCHITECTURE.md for the design.
 */

#include "MeshTimeSync.h"

#include <adapters/clock/ZephyrRTCDiscover.h>
#include <helpers/time_sync.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(zephcore_timesync, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* Clamp an int64 skew to a printable long (display only). */
static long clampl(int64_t v)
{
	if (v > 2000000000LL) return 2000000000L;
	if (v < -2000000000LL) return -2000000000L;
	return (long)v;
}

void MeshTimeSync::reset(uint32_t build_epoch, bool forward_only)
{
	memset(_slots, 0, sizeof(_slots));
	_build_epoch = build_epoch;
	_forward_only = forward_only;
	_next_eval_uptime = 0;
	_suppress_uptime = 0;
	_suppressed = false;
	_pedigree_uptime = 0;
	_pedigree = false;
	_last_step_uptime = 0;
	_stepped_once = false;
	_evals = _abstains = _steps = _bootstrap_steps = _backward_skips = 0;
	_last_step_delta = 0;
}

MeshTimeSync::Slot *MeshTimeSync::findSlot(const uint8_t *prefix)
{
	for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
		if (_slots[i].used && memcmp(_slots[i].prefix, prefix, sizeof(_slots[i].prefix)) == 0) {
			return &_slots[i];
		}
	}
	return nullptr;
}

bool MeshTimeSync::slotTenured(const Slot &s, uint32_t uptime_secs) const
{
	return (uptime_secs - s.first_uptime) >= TENURE_SECS &&
	       s.count >= TENURE_MIN_ADVERTS;
}

bool MeshTimeSync::slotEligible(const Slot &s, uint32_t uptime_secs, bool bootstrap) const
{
	return s.used && (uptime_secs - s.arrival_uptime) <= MAX_SAMPLE_AGE_SECS &&
	       (bootstrap || slotTenured(s, uptime_secs));
}

int64_t MeshTimeSync::slotSkew(const Slot &s, uint32_t local_time, uint32_t uptime_secs) const
{
	/* Project the sender's clock forward by our elapsed uptime, then compare
	 * with our wall clock. Uptime anchor keeps this valid across own steps. */
	return (int64_t)s.advert_ts + (int64_t)(uptime_secs - s.arrival_uptime) -
	       (int64_t)local_time;
}

bool MeshTimeSync::wouldAccept(const uint8_t *pubkey, uint32_t advert_ts) const
{
	for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
		const Slot &s = _slots[i];
		if (s.used && memcmp(s.prefix, pubkey, sizeof(s.prefix)) == 0) {
			return advert_ts > s.advert_ts;
		}
	}
	return true;
}

void MeshTimeSync::onAdvertHeard(const uint8_t *pubkey, uint32_t advert_ts,
                                 uint8_t hops, uint32_t uptime_secs)
{
	if (hops > HOP_CAP) return;

	Slot *s = findSlot(pubkey);
	if (s) {
		if (advert_ts <= s->advert_ts) return;  /* per-sender monotonic dedup */

		uint32_t d_up = uptime_secs - s->arrival_uptime;
		uint32_t d_ts = advert_ts - s->advert_ts;
		int64_t err = (int64_t)d_ts - (int64_t)d_up;
		int64_t tol = CONSISTENCY_BASE_SECS + ((int64_t)d_up * CONSISTENCY_PPM) / 1000000;
		if (err > tol || err < -tol) {
			/* Sender rebooted, corrected itself, or is lying — re-earn
			 * tenure from scratch. >150 ppm is beyond crystal physics. */
			s->first_uptime = uptime_secs;
			s->count = 1;
		} else if (s->count < 0xFFFF) {
			s->count++;
		}
		s->advert_ts = advert_ts;
		s->arrival_uptime = uptime_secs;
		s->hops = hops;
		return;
	}

	for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
		if (!_slots[i].used) { s = &_slots[i]; break; }
	}

	if (!s) {
		/* Hop-priority admission: a new sender may only displace a young
		 * (not tenure-eligible) entry farther than it; mature entries are
		 * protected unless silent > 24 h. Among candidates evict farthest,
		 * then lowest count, then stalest. (Naive LRU churns hub nodes to
		 * zero eligible voters.) */
		for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
			Slot &c = _slots[i];
			bool young = !slotTenured(c, uptime_secs);
			bool silent = (uptime_secs - c.arrival_uptime) > MATURE_SILENT_EVICT_SECS;
			if (!young && !silent) continue;
			if (c.hops <= hops) continue;
			if (s == nullptr ||
			    c.hops > s->hops ||
			    (c.hops == s->hops && c.count < s->count) ||
			    (c.hops == s->hops && c.count == s->count &&
			     c.arrival_uptime < s->arrival_uptime)) {
				s = &c;
			}
		}
		if (!s) return;  /* table full of closer/protected senders — drop */
	}

	memcpy(s->prefix, pubkey, sizeof(s->prefix));
	s->advert_ts = advert_ts;
	s->arrival_uptime = uptime_secs;
	s->first_uptime = uptime_secs;
	s->count = 1;
	s->hops = hops;
	s->used = 1;
}

MeshTimeSync::Consensus MeshTimeSync::computeConsensus(uint32_t local_time,
                                                       uint32_t uptime_secs,
                                                       bool bootstrap) const
{
	Consensus c;
	memset(&c, 0, sizeof(c));

	/* Collect eligible votes directly as Marzullo interval endpoints.
	 * Bootstrap relaxes tenure: any sender with a fresh sample may vote
	 * (the table is RAM-only, so after the reboot that dead-ended the
	 * clock everything in it is freshly heard anyway). */
	int64_t val[2 * MESHTIMESYNC_TABLE_SIZE];
	int8_t typ[2 * MESHTIMESYNC_TABLE_SIZE];  /* +1 = start, -1 = end */
	int n = 0, m = 0;
	for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
		const Slot &s = _slots[i];
		if (!slotEligible(s, uptime_secs, bootstrap)) continue;
		int64_t skew = slotSkew(s, local_time, uptime_secs);
		int32_t r = RADIUS_BASE_SECS + RADIUS_PER_HOP_SECS * s.hops;
		val[m] = skew - r; typ[m] = 1;  m++;
		val[m] = skew + r; typ[m] = -1; m++;
		n++;
	}
	c.eligible = (uint8_t)n;
	if (n == 0) return c;

	/* Marzullo endpoint sweep — interval intersection with the most votes.
	 * No absolute outlier thresholds against our own clock: clustering does
	 * the rejection, so an epoch-0 local clock still finds the true cluster.
	 * Insertion sort by value; starts before ends at equal values so
	 * touching intervals count as overlapping. */
	for (int i = 1; i < m; i++) {
		int64_t v = val[i];
		int8_t t = typ[i];
		int j = i - 1;
		while (j >= 0 && (val[j] > v || (val[j] == v && typ[j] < t))) {
			val[j + 1] = val[j];
			typ[j + 1] = typ[j];
			j--;
		}
		val[j + 1] = v;
		typ[j + 1] = t;
	}

	int cur = 0, best = 0, best_idx = 0;
	for (int i = 0; i < m; i++) {
		cur += typ[i];
		if (typ[i] > 0 && cur > best) {
			best = cur;
			best_idx = i;
		}
	}
	int64_t lo = val[best_idx];
	int64_t hi = val[best_idx + 1];  /* next endpoint always exists and, at the
	                                  * maximum, is an end — a start would have
	                                  * raised the count past `best` */

	c.valid = true;
	c.votes_for = (uint8_t)best;
	c.votes_against = (uint8_t)(n - best);
	c.mid = (lo + hi) / 2;
	c.radius = (int32_t)((hi - lo) / 2);
	return c;
}

MeshTimeSync::Verdict MeshTimeSync::evaluateNow(uint32_t local_time,
                                                uint32_t uptime_secs) const
{
	Verdict v;
	memset(&v, 0, sizeof(v));

	bool boot = isBootstrap(local_time);
	v.bootstrap = boot;
	v.consensus = computeConsensus(local_time, uptime_secs, boot);

	if (!v.consensus.valid) {
		v.type = VERDICT_ABSTAIN;
		v.reason = REASON_NO_DATA;
		return v;
	}

	if (boot) {
		if (v.consensus.votes_for < BOOTSTRAP_QUORUM) {
			v.type = VERDICT_ABSTAIN;
			v.reason = REASON_NO_QUORUM;
			return v;
		}
		/* Step to the cluster's low edge: from below, all later refinement
		 * is forward steps, which are always monotonicity-safe. */
		int64_t delta = v.consensus.mid - RADIUS_BASE_SECS;
		if (delta < STEP_TRIGGER_SECS) {
			v.type = VERDICT_NONE;
			v.reason = REASON_IN_BAND;
			return v;
		}
		if (isSuppressed(uptime_secs)) {  /* suppression gates bootstrap too */
			v.type = VERDICT_NONE;
			v.reason = REASON_SUPPRESSED;
			return v;
		}
		if (_stepped_once && (uptime_secs - _last_step_uptime) < STEP_INTERVAL_SECS) {
			v.type = VERDICT_NONE;
			v.reason = REASON_RATE_LIMITED;
			return v;
		}
		v.type = VERDICT_STEP;
		v.delta = delta;  /* bootstrap exempt from the ±1 h cap */
		return v;
	}

	if (v.consensus.eligible < MESHTIMESYNC_QUORUM) {
		v.type = VERDICT_ABSTAIN;
		v.reason = REASON_NO_QUORUM;
		return v;
	}
	if ((int)v.consensus.votes_for * 2 <= (int)v.consensus.eligible) {
		v.type = VERDICT_ABSTAIN;
		v.reason = REASON_NO_MAJORITY;
		return v;
	}

	int64_t mag = v.consensus.mid < 0 ? -v.consensus.mid : v.consensus.mid;
	if (mag < STEP_TRIGGER_SECS) {
		v.type = VERDICT_NONE;
		v.reason = REASON_IN_BAND;
		return v;
	}
	if (isSuppressed(uptime_secs)) {
		v.type = VERDICT_NONE;
		v.reason = REASON_SUPPRESSED;
		return v;
	}
	if (_stepped_once && (uptime_secs - _last_step_uptime) < STEP_INTERVAL_SECS) {
		v.type = VERDICT_NONE;
		v.reason = REASON_RATE_LIMITED;
		return v;
	}
	if (_pedigree) {
		/* Physics veto: with a trusted sync + continuous uptime since, a
		 * real crystal cannot have drifted further than 300 ppm allows. */
		int64_t envelope = ((int64_t)(uptime_secs - _pedigree_uptime) * PEDIGREE_PPM) / 1000000 +
		                   PEDIGREE_BASE_SECS;
		if (mag > envelope) {
			v.type = VERDICT_ABSTAIN;
			v.reason = REASON_PEDIGREE;
			return v;
		}
	}

	int64_t delta = v.consensus.mid;
	if (delta > STEP_CAP_SECS) delta = STEP_CAP_SECS;
	if (delta < -STEP_CAP_SECS) delta = -STEP_CAP_SECS;
	v.type = VERDICT_STEP;
	v.delta = delta;
	return v;
}

MeshTimeSync::Verdict MeshTimeSync::tick(uint32_t local_time, uint32_t uptime_secs)
{
	Verdict v;
	if (_next_eval_uptime == 0 || uptime_secs < _next_eval_uptime) {
		/* First call arms the pacing timer (skips a junk no-data abstain
		 * at boot); later calls no-op between evaluations. */
		if (_next_eval_uptime == 0) {
			_next_eval_uptime = uptime_secs + EVAL_INTERVAL_SECS;
		}
		memset(&v, 0, sizeof(v));
		return v;
	}
	_next_eval_uptime = uptime_secs + EVAL_INTERVAL_SECS;
	_evals++;

	v = evaluateNow(local_time, uptime_secs);
	if (v.type == VERDICT_ABSTAIN) _abstains++;
	return v;
}

bool MeshTimeSync::runTick(mesh::RTCClock &rtc)
{
	uint32_t up = (uint32_t)(k_uptime_get() / 1000);
	uint32_t now = rtc.getCurrentTime();
	Verdict v = tick(now, up);
	if (v.type != VERDICT_STEP) {
		if (v.type == VERDICT_ABSTAIN) {
			LOG_DBG("abstain (%s)", reasonStr(v.reason));
		}
		return false;
	}
	if (_forward_only && v.delta < 0) {
		/* Forward-only roles: post timestamps feed client sync_since
		 * ordering (room server) / peers hold per-sender replay high-water
		 * marks for our DMs (companion). Report, never apply. */
		_backward_skips++;
		LOG_WRN("backward step %+ld s wanted - skipped (forward-only role)",
		        clampl(v.delta));
		return false;
	}
	int64_t nt = (int64_t)now + v.delta;
	if (nt <= 0 || nt > (int64_t)UINT32_MAX) {
		/* Would wrap the uint32 clock — only reachable with a garbage
		 * bootstrap cluster near the timestamp ceiling. */
		LOG_WRN("implausible step %+ld s refused", clampl(v.delta));
		return false;
	}
	uint32_t new_time = (uint32_t)nt;
	rtc.setCurrentTime(new_time);
	zephcore_rtc_save(new_time);
	time_sync_report(TIME_SYNC_MESH);
	noteStepApplied(v.delta, up, v.bootstrap);
	LOG_WRN("stepped clock %+ld s (%s, votes %u/%u) -> %u",
	        clampl(v.delta), v.bootstrap ? "bootstrap" : "consensus",
	        (unsigned)v.consensus.votes_for, (unsigned)v.consensus.votes_against,
	        (unsigned)new_time);
	return true;
}

void MeshTimeSync::noteManualSync(uint32_t uptime_secs)
{
	_suppress_uptime = uptime_secs;
	_suppressed = true;
	_pedigree_uptime = uptime_secs;
	_pedigree = true;
}

void MeshTimeSync::noteGPSSync(uint32_t uptime_secs)
{
	/* A GPS fix owns the clock exactly like a manual set: arm the same 7-day
	 * suppression window (re-armed on every fix, so a repeater's 48 h GPS
	 * duty cycle keeps it owning the clock) plus the drift-envelope pedigree.
	 * A unit whose GPS goes dead becomes mesh-correctable once the window
	 * lapses without a new fix. */
	_suppress_uptime = uptime_secs;
	_suppressed = true;
	_pedigree_uptime = uptime_secs;
	_pedigree = true;
}

void MeshTimeSync::noteStepApplied(int64_t delta, uint32_t uptime_secs, bool bootstrap)
{
	_last_step_uptime = uptime_secs;
	_stepped_once = true;
	_steps++;
	if (bootstrap) _bootstrap_steps++;
	_last_step_delta = delta;
}

bool MeshTimeSync::isSuppressed(uint32_t uptime_secs) const
{
	return _suppressed && (uptime_secs - _suppress_uptime) < SUPPRESS_SECS;
}

uint32_t MeshTimeSync::suppressRemaining(uint32_t uptime_secs) const
{
	if (!isSuppressed(uptime_secs)) return 0;
	return SUPPRESS_SECS - (uptime_secs - _suppress_uptime);
}

const char *MeshTimeSync::reasonStr(Reason r)
{
	switch (r) {
	case REASON_IN_BAND:      return "in-band";
	case REASON_NO_DATA:      return "no-data";
	case REASON_NO_QUORUM:    return "no-quorum";
	case REASON_NO_MAJORITY:  return "no-majority";
	case REASON_SUPPRESSED:   return "suppressed";
	case REASON_RATE_LIMITED: return "rate-limited";
	case REASON_PEDIGREE:     return "pedigree-veto";
	default:                  return "-";
	}
}

int MeshTimeSync::formatStatus(char *out, size_t cap, uint32_t local_time,
                               uint32_t uptime_secs, bool enabled) const
{
	Verdict v = evaluateNow(local_time, uptime_secs);
	const Consensus &c = v.consensus;

	char verdict[56];
	if (v.type == VERDICT_STEP) {
		/* Annotate steps the shared step policy would refuse, so the
		 * dry-run never claims a step that will not happen. (Suppression —
		 * manual or GPS — surfaces as a hold verdict inside evaluateNow,
		 * so it needs no annotation here.) */
		const char *note = "";
		if (_forward_only && v.delta < 0) {
			note = " (skipped: forward-only)";
		}
		snprintf(verdict, sizeof(verdict), "step%+ld%s%s", clampl(v.delta),
		         v.bootstrap ? " (bootstrap)" : "", note);
	} else if (v.type == VERDICT_NONE && v.reason == REASON_IN_BAND) {
		int64_t mag = c.mid < 0 ? -c.mid : c.mid;
		snprintf(verdict, sizeof(verdict), "%s",
		         mag <= DEAD_BAND_SECS ? "ok" : "in-band");
	} else if (v.type == VERDICT_NONE) {
		snprintf(verdict, sizeof(verdict), "hold (%s)", reasonStr(v.reason));
	} else {
		snprintf(verdict, sizeof(verdict), "abstain (%s)", reasonStr(v.reason));
	}

	int pos = snprintf(out, cap,
	                   "%s%s eligible=%u votes=%u/%u skew=%+lds r=%lds -> %s",
	                   enabled ? "on" : "off",
	                   enabled ? "" : " (dry-run)",
	                   (unsigned)c.eligible, (unsigned)c.votes_for,
	                   (unsigned)c.votes_against,
	                   c.valid ? clampl(c.mid) : 0L,
	                   c.valid ? (long)c.radius : 0L,
	                   verdict);
	if (pos < 0 || (size_t)pos >= cap) goto full;

	if (_steps > 0) {
		pos += snprintf(out + pos, cap - pos, "; steps=%lu boot=%lu last=%+lds",
		                (unsigned long)_steps, (unsigned long)_bootstrap_steps,
		                clampl(_last_step_delta));
		if (pos < 0 || (size_t)pos >= cap) goto full;
	}
	if (_backward_skips > 0) {
		pos += snprintf(out + pos, cap - pos, " backskip=%lu",
		                (unsigned long)_backward_skips);
		if (pos < 0 || (size_t)pos >= cap) goto full;
	}
	if (isSuppressed(uptime_secs)) {
		pos += snprintf(out + pos, cap - pos, " sup=%luh",
		                (unsigned long)(suppressRemaining(uptime_secs) / 3600));
		if (pos < 0 || (size_t)pos >= cap) goto full;
	}
	pos += snprintf(out + pos, cap - pos, " evals=%lu abst=%lu",
	                (unsigned long)_evals, (unsigned long)_abstains);
	if (pos < 0 || (size_t)pos >= cap) goto full;

	{
		/* Evidence table: entries that actually count toward this verdict
		 * print first, then the rest — on a size-capped reply (161 B over
		 * remote admin), the buffer must not run out on still-building-tenure
		 * entries while hiding the ones that explain the verdict.
		 * (Braced so these locals go out of scope before the `full` label
		 * below — a goto may not jump over a variable's initialization.) */
		int total_used = 0;
		for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
			if (_slots[i].used) total_used++;
		}

		int printed = 0;
		for (int pass = 0; pass < 2 && printed < total_used; pass++) {
			for (int i = 0; i < MESHTIMESYNC_TABLE_SIZE; i++) {
				const Slot &s = _slots[i];
				if (!s.used) continue;
				bool participates = slotEligible(s, uptime_secs, v.bootstrap);
				if (participates != (pass == 0)) continue;

				int w = snprintf(out + pos, cap - pos, "\r\n %02x%02x h%u n%u %+lds%s",
				                 s.prefix[0], s.prefix[1], (unsigned)s.hops,
				                 (unsigned)(s.count > 99 ? 99 : s.count),
				                 clampl(slotSkew(s, local_time, uptime_secs)),
				                 participates ? " E" : "");
				if (w < 0 || (size_t)(pos + w) >= cap) {
					out[pos] = 0;  /* drop the partial entry */
					goto truncated;
				}
				pos += w;
				printed++;
			}
		}
		return pos;

	truncated:
		if (printed < total_used) {
			char tail[24];
			int tw = snprintf(tail, sizeof(tail), "\r\n +%d more", total_used - printed);
			if (tw > 0 && (size_t)tw < cap - (size_t)pos) {
				memcpy(out + pos, tail, (size_t)tw + 1);
				pos += tw;
			}
		}
		return pos;
	}

full:
	out[cap - 1] = 0;
	return (int)(cap - 1);
}
