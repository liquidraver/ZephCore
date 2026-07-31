/*
 * SPDX-License-Identifier: MIT
 * MeshTimeSync - mesh clock consensus from Ed25519-signed advert timestamps.
 *
 * Role-agnostic estimator plus shared step policy. Each role feeds it
 * signature-verified adverts (onAdvertHeard) and calls runTick() from its
 * loop; runTick applies the shared policy (7-day suppression armed by any
 * manual or GPS clock set, forward-only roles skip backward steps, ±1 h cap,
 * 6 h rate limit) and steps the given clock. Role-specific bookkeeping (neighbor/ACL timestamp
 * shifts, rate-limiter resets) happens in the role after runTick returns
 * true. ZephCore-only divergence from Arduino MeshCore — design rationale
 * in ARCHITECTURE.md, user-facing doc in MESHTIMESYNC.md.
 *
 * All policy timers anchor on uptime, never wall clock, so the very steps
 * they govern cannot distort them. Main-thread-only, like the mesh classes
 * that own it.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <mesh/RTC.h>

#ifdef CONFIG_ZEPHCORE_TIMESYNC_TABLE_SIZE
  #define MESHTIMESYNC_TABLE_SIZE  CONFIG_ZEPHCORE_TIMESYNC_TABLE_SIZE
#else
  #define MESHTIMESYNC_TABLE_SIZE  32
#endif

#ifdef CONFIG_ZEPHCORE_TIMESYNC_QUORUM
  #define MESHTIMESYNC_QUORUM  CONFIG_ZEPHCORE_TIMESYNC_QUORUM
#else
  #define MESHTIMESYNC_QUORUM  6
#endif

#ifndef FIRMWARE_BUILD_EPOCH
  /* Injected by CMakeLists.txt (build-time UNIX epoch, the "provably dead
   * clock" floor). 0 disables bootstrap mode entirely. */
  #define FIRMWARE_BUILD_EPOCH 0u
#endif

class MeshTimeSync {
public:
	static constexpr uint8_t  HOP_CAP                  = 3;
	static constexpr int32_t  RADIUS_BASE_SECS         = 150;
	static constexpr int32_t  RADIUS_PER_HOP_SECS      = 15;
	static constexpr uint32_t TENURE_SECS              = 60 * 60;
	static constexpr uint16_t TENURE_MIN_ADVERTS       = 2;
	static constexpr uint32_t MAX_SAMPLE_AGE_SECS      = 5 * 24 * 3600;
	static constexpr uint32_t MATURE_SILENT_EVICT_SECS = 24 * 3600;
	static constexpr int64_t  CONSISTENCY_BASE_SECS    = 45;
	static constexpr int64_t  CONSISTENCY_PPM          = 150;
	static constexpr uint32_t EVAL_INTERVAL_SECS       = 15 * 60;
	static constexpr int64_t  DEAD_BAND_SECS           = 5 * 60;
	static constexpr int64_t  STEP_TRIGGER_SECS        = 10 * 60;
	static constexpr int64_t  STEP_CAP_SECS            = 3600;
	static constexpr uint32_t STEP_INTERVAL_SECS       = 6 * 3600;
	static constexpr uint32_t SUPPRESS_SECS            = 7 * 24 * 3600;
	static constexpr int64_t  PEDIGREE_PPM             = 300;
	static constexpr int64_t  PEDIGREE_BASE_SECS       = 10 * 60;
	static constexpr uint8_t  BOOTSTRAP_QUORUM         = 3;

	/* 8-byte prefix is a security floor, not a tuning knob: it is the
	 * sender's identity for tenure/votes while signatures verify the full
	 * key, so a shorter prefix lets an attacker grind keypairs to collide
	 * with a tenured honest voter and reset its tenure with validly-signed
	 * adverts. If RAM ever matters, cut slot count instead. */
	struct Slot {
		uint8_t  prefix[8];
		uint32_t advert_ts;        /* latest advert timestamp = the vote */
		uint32_t arrival_uptime;   /* monotonic anchor: skew is recomputed at
		                            * evaluate time, so a local clock step
		                            * never stales stored samples */
		uint32_t first_uptime;     /* tenure start */
		uint16_t count;            /* adverts this tenure */
		uint8_t  hops;             /* precision hint only, never a trust signal */
		uint8_t  used;
	};

	enum VerdictType : uint8_t { VERDICT_NONE = 0, VERDICT_ABSTAIN, VERDICT_STEP };

	enum Reason : uint8_t {
		REASON_NONE = 0,
		REASON_IN_BAND,       /* |skew| below the step trigger */
		REASON_NO_DATA,       /* no eligible voters */
		REASON_NO_QUORUM,
		REASON_NO_MAJORITY,
		REASON_SUPPRESSED,    /* manual or GPS clock set less than 7 days ago */
		REASON_RATE_LIMITED,  /* < 6 h since last applied step */
		REASON_PEDIGREE,      /* drift-envelope physics veto */
	};

	struct Consensus {
		bool valid;            /* >= 1 eligible vote, intersection computed */
		uint8_t eligible;
		uint8_t votes_for;     /* votes inside the best intersection */
		uint8_t votes_against;
		int64_t mid;           /* consensus skew midpoint (+ = our clock is behind) */
		int32_t radius;        /* intersection half-width */
	};

	struct Verdict {
		VerdictType type;
		Reason reason;
		int64_t delta;         /* seconds to add to the clock (STEP only) */
		bool bootstrap;
		Consensus consensus;
	};

	explicit MeshTimeSync(uint32_t build_epoch = 0, bool forward_only = false)
	{
		reset(build_epoch, forward_only);
	}

	void reset(uint32_t build_epoch, bool forward_only);

	/* Feed one signature-verified advert. hops = flood path length (0 = heard
	 * direct). Samples beyond HOP_CAP are dropped. Callers must not feed
	 * share rebroadcasts (transport codes {0,0}) — those replay stale stored
	 * adverts and would churn the sender's tenure. */
	void onAdvertHeard(const uint8_t *pubkey, uint32_t advert_ts, uint8_t hops,
	                   uint32_t uptime_secs);

	/* Cheap pre-check: would onAdvertHeard even consider this sample?
	 * Lets callers skip the Ed25519 verify for per-sender duplicates
	 * (observers hear every flood copy with no dedup of their own). */
	bool wouldAccept(const uint8_t *pubkey, uint32_t advert_ts) const;

	/* Paced policy run — call from the role's loop with its RTC clock.
	 * Evaluates at most every EVAL_INTERVAL_SECS; applies the shared step
	 * policy and steps the clock (incl. hardware-RTC save + UI time-source
	 * report). Returns true when a step was applied — the role then shifts
	 * its wall-clock-anchored bookkeeping by lastStepDelta(). The caller
	 * gates on its own enable pref. */
	bool runTick(mesh::RTCClock &rtc);

	/* Milliseconds until runTick() would next actually evaluate (it no-ops
	 * in between).  Lets a deadline-driven event loop size its sleep instead
	 * of calling this on a fixed cadence.  Callers still gate on their own
	 * enable pref — a disabled time sync has no deadline at all. */
	uint32_t msUntilNextEval(uint32_t uptime_secs) const
	{
		if (_next_eval_uptime == 0 || uptime_secs >= _next_eval_uptime) {
			return 0;
		}
		return (_next_eval_uptime - uptime_secs) * 1000U;
	}

	/* Manual clock set (CLI time/clock sync, app time set, SNTP): arms the
	 * 7-day suppression window AND drift-envelope pedigree. Suppression
	 * gates bootstrap too. */
	void noteManualSync(uint32_t uptime_secs);
	/* GPS time sync: identical to a manual set — arms the same 7-day
	 * suppression window and drift-envelope pedigree, re-armed on every fix
	 * so a live GPS keeps owning the clock. */
	void noteGPSSync(uint32_t uptime_secs);

	/* Delta of the most recent applied step (for role bookkeeping shifts). */
	int64_t lastStepDelta() const { return _last_step_delta; }

	/* Compact status + evidence-table formatter shared by all role CLIs.
	 * Writes at most `cap` bytes (NUL-terminated), summary first, then as
	 * many per-sender entries as fit. Annotates a backward step a
	 * forward-only role would refuse ("skipped: forward-only"); a suppressed
	 * clock (manual or GPS) surfaces as a "hold (suppressed)" verdict. */
	int formatStatus(char *out, size_t cap, uint32_t local_time,
	                 uint32_t uptime_secs, bool enabled) const;

private:
	bool isBootstrap(uint32_t local_time) const { return local_time < _build_epoch; }

	Slot *findSlot(const uint8_t *prefix);
	bool slotTenured(const Slot &s, uint32_t uptime_secs) const;
	/* Whether this slot counts toward the current evaluation — bootstrap
	 * relaxes tenure, so this takes the mode explicitly rather than
	 * hardcoding normal-mode rules. Single source of truth shared by
	 * computeConsensus (who votes) and formatStatus (who gets the "E"
	 * marker) — they must never diverge on what "counted" means. */
	bool slotEligible(const Slot &s, uint32_t uptime_secs, bool bootstrap) const;
	int64_t slotSkew(const Slot &s, uint32_t local_time, uint32_t uptime_secs) const;

	Consensus computeConsensus(uint32_t local_time, uint32_t uptime_secs,
	                           bool bootstrap) const;
	/* Unpaced full policy evaluation (no counter/pacing side effects). */
	Verdict evaluateNow(uint32_t local_time, uint32_t uptime_secs) const;
	/* Paced evaluation — VERDICT_NONE/REASON_NONE between evaluations. */
	Verdict tick(uint32_t local_time, uint32_t uptime_secs);

	void noteStepApplied(int64_t delta, uint32_t uptime_secs, bool bootstrap);
	bool isSuppressed(uint32_t uptime_secs) const;
	uint32_t suppressRemaining(uint32_t uptime_secs) const;
	static const char *reasonStr(Reason r);

	Slot _slots[MESHTIMESYNC_TABLE_SIZE];
	uint32_t _build_epoch;
	bool _forward_only;

	/* Policy state — uptime-anchored (see header comment). */
	uint32_t _next_eval_uptime;
	uint32_t _suppress_uptime;
	bool _suppressed;
	uint32_t _pedigree_uptime;
	bool _pedigree;
	uint32_t _last_step_uptime;
	bool _stepped_once;

	/* Counters (shown by formatStatus). */
	uint32_t _evals, _abstains, _steps, _bootstrap_steps, _backward_skips;
	int64_t _last_step_delta;
};
