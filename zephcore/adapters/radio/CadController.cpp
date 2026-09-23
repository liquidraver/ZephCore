/*
 * SPDX-License-Identifier: MIT
 * Adaptive CAD controller — see CadController.h.
 */

#include "CadController.h"
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lora_radio_base, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

CadController::CadController(CadHw &hw)
	: _hw(hw), _auto(false), _offset(0), _temp_offset(0), _visiting(false),
	  _busycap_pct(0), _probe_rr(0), _pending_level(INT8_MIN),
	  _pending_deadline_ms(0)
{
	memset(_stats, 0, sizeof(_stats));
}

/* The clamp only binds when base + CAD_LEVEL_MIN would land below it. (Once
 * written `base - CAD_LEVEL_MIN`, which is always above pmin: inert.) */
int8_t CadController::levelMinEff()
{
	uint8_t base = _hw.hwCadBasePeak();
	uint8_t pmin = _hw.hwCadPeakMin();

	if (base == 0 || pmin == 0 ||
	    (int)base + CAD_LEVEL_MIN >= (int)pmin) {
		return CAD_LEVEL_MIN;
	}
	return (int8_t)((int)pmin - (int)base);
}

int8_t CadController::levelMaxEff()
{
	uint8_t base = _hw.hwCadBasePeak();
	uint8_t pmax = _hw.hwCadPeakMax();

	if (base == 0 || pmax == 0 || (int)pmax - (int)base >= CAD_LEVEL_MAX) {
		return CAD_LEVEL_MAX;
	}
	return (int8_t)((int)pmax - (int)base);
}

void CadController::configure(bool auto_enabled, int8_t offset,
			      uint8_t busycap_pct, uint8_t stored_base)
{
	const int8_t lo = levelMinEff();
	const int8_t hi = levelMaxEff();
	const uint8_t base = _hw.hwCadBasePeak();

	/* Re-anchor across a base-table change: the offset is persisted, the
	 * stats behind it are not, so keep the measured PEAK. A peak the new
	 * window cannot reach is clamped below. */
	if (stored_base != 0 && base != 0 && stored_base != base) {
		int adj = (int)offset + (int)stored_base - (int)base;

		LOG_INF("cad: base %u -> %u, re-anchoring offset %d -> %d "
			"(peak %d held)",
			(unsigned)stored_base, (unsigned)base,
			(int)offset, adj, (int)stored_base + (int)offset);
		offset = (int8_t)(adj < -128 ? -128 : (adj > 127 ? 127 : adj));
	}

	if (offset < lo) offset = lo;
	if (offset > hi) offset = hi;

	_auto = auto_enabled;
	_offset = offset;
	_busycap_pct = busycap_pct;

	/* Effective: `set cad.*` may arrive during a visit, and the visited
	 * preset must keep its own threshold. */
	_hw.hwCadSetPeakOffset(effectiveOffset());
}

void CadController::beginVisit(bool new_preset)
{
	_temp_offset = new_preset ? 0 : _offset;
	_visiting = true;
	_hw.hwCadSetPeakOffset(effectiveOffset());
}

void CadController::endVisit()
{
	_visiting = false;
	_hw.hwCadSetPeakOffset(effectiveOffset());
}

void CadController::resetStats()
{
	memset(_stats, 0, sizeof(_stats));
	_probe_rr = 0;

	/* A probe in flight belongs to the table being dropped. */
	_pending_level = INT8_MIN;
	_pending_deadline_ms = 0;
}

void CadController::decayStats()
{
	for (int i = 0; i < CAD_NUM_LEVELS; i++) {
		_stats[i].probes >>= 1;
		_stats[i].busy >>= 1;
		_stats[i].fp >>= 1;
		_stats[i].tp >>= 1;
	}
}

int8_t CadController::pickProbeLevel()
{
	_probe_rr++;

	if (!_auto) {
		/* Parked outside the sweep window: probe where the operator is,
		 * or the safety rung never gets evidence for that level. */
		if (_offset < CAD_SWEEP_MIN || _offset > CAD_SWEEP_MAX) {
			return _offset;
		}

		/* Dry run: an even sweep, so `get cad` shows the whole curve. */
		int span = CAD_SWEEP_MAX - CAD_SWEEP_MIN + 1;

		return (int8_t)(CAD_SWEEP_MIN + (_probe_rr % span));
	}

	/* Auto: operating level half the time, each neighbour a quarter, so the
	 * staircase can read the slope on both sides. Out-of-range neighbours
	 * fall back to the operating level. */
	int8_t lvl;
	switch (_probe_rr & 3) {
	case 1:  lvl = (int8_t)(_offset - 1); break;  /* more sensitive */
	case 3:  lvl = (int8_t)(_offset + 1); break;  /* less sensitive */
	default: lvl = _offset; break;                /* operating (0, 2) */
	}
	if (lvl < levelMinEff() || lvl > levelMaxEff()) {
		lvl = _offset;
	}
	return lvl;
}

void CadController::recordProbe(int8_t level, bool busy)
{
	LevelStats &s = _stats[level - CAD_LEVEL_MIN];

	if (s.probes >= 0xFFF0) {
		decayStats();
	}
	s.probes++;

	if (busy) {
		s.busy++;
	}
}

void CadController::resolvePending(int outcome)
{
	LevelStats &ps = _stats[_pending_level - CAD_LEVEL_MIN];

	if (outcome == 1) {
		ps.tp++;
	} else if (outcome == 2) {
		ps.fp++;
	} else {
		/* No terminal event: an invented verdict would poison the curve,
		 * so withdraw the sample (probe and busy, to keep both rates
		 * over the same population). */
		if (ps.probes) ps.probes--;
		if (ps.busy) ps.busy--;
		LOG_WRN("cad: probe at %+d unresolved past deadline, discarded",
			(int)_pending_level);
	}
	_pending_level = INT8_MIN;
}

/* Per-level FALSE-positive rate in permille, or -1 when too few samples. */
int CadController::fpRate(int idx) const
{
	if (idx < 0 || idx >= CAD_NUM_LEVELS ||
	    _stats[idx].probes < CAD_STEP_MIN_PROBES) {
		return -1;
	}
	return (int)(((uint32_t)_stats[idx].fp * 1000U) / _stats[idx].probes);
}

/* Per-level TOTAL busy (defer) rate in permille — false + real traffic. */
int CadController::busyRate(int idx) const
{
	if (idx < 0 || idx >= CAD_NUM_LEVELS ||
	    _stats[idx].probes < CAD_STEP_MIN_PROBES) {
		return -1;
	}
	return (int)(((uint32_t)_stats[idx].busy * 1000U) / _stats[idx].probes);
}

void CadController::stepOffset(int8_t to)
{
	_offset = to;
	_hw.hwCadSetPeakOffset(_offset);
}

/* Steps toward the knee: the most sensitive level whose FP rate has already
 * bottomed out. Uses slopes, so the decision does not depend on a site's FP
 * floor. */
void CadController::staircaseStep()
{
	int oi = _offset - CAD_LEVEL_MIN;

	int r_op = fpRate(oi);
	if (r_op < 0) {
		return;  /* operating level not warm yet */
	}
	int r_up = fpRate(oi + 1);  /* one step less sensitive */
	int r_dn = fpRate(oi - 1);  /* frontier, one step more sensitive */

	int cap_permille = (int)_busycap_pct * 10;

	/* Up when the level above is markedly cleaner: below the knee. */
	if (_offset < levelMaxEff() && r_up >= 0 &&
	    r_op - r_up >= CAD_KNEE_SLOPE_PERMILLE) {
		stepOffset((int8_t)(_offset + 1));
		LOG_INF("cad: step up -> offset %d (op %d dn->up %d)",
			(int)_offset, r_op, r_up);
		return;
	}

	/* Down only on a flat, already-clean plateau, and not into the airtime
	 * cap (hysteresis keeps it from bouncing straight back up). */
	int b_dn = busyRate(oi - 1);
	bool busy_ok = (cap_permille == 0) ||
		       (b_dn <= cap_permille -
				(cap_permille * CAD_BUSY_DEFER_HYST_PCT) / 100);
	if (_offset > levelMinEff() && r_dn >= 0 &&
	    r_dn - r_op < CAD_KNEE_SLOPE_PERMILLE &&
	    r_op <= CAD_PLATEAU_CLEAN_PERMILLE && busy_ok) {
		stepOffset((int8_t)(_offset - 1));
		LOG_INF("cad: step down -> offset %d (op %d dn %d busy %d)",
			(int)_offset, r_op, r_dn, b_dn);
		return;
	}

	/* Otherwise: at the knee, or a noisy plateau — hold. */
}

/* The proactive half of the TX safety pair: works off probe statistics, so it
 * acts on a silent mesh too, where relaxOnTxStarvation() would wait for the
 * next own transmit. Not gated on auto: it is a safety, not an optimiser. */
bool CadController::safetyStep()
{
	if (_offset >= levelMaxEff()) {
		return false;
	}

	int oi = _offset - CAD_LEVEL_MIN;
	if (oi < 0 || oi >= CAD_NUM_LEVELS) {
		return false;
	}

	uint16_t probes = _stats[oi].probes;
	if (probes == 0) {
		return false;
	}

	int b_op = (int)(((uint32_t)_stats[oi].busy * 1000U) / probes);
	int cap_permille = (int)_busycap_pct * 10;

	/* Pathological: trips on nearly every probe, so it cannot clear a
	 * transmit either. Few samples suffice; the cap is ignored. */
	bool pathological = probes >= CAD_SAFETY_MIN_PROBES &&
			    b_op >= CAD_SAFETY_PATHOLOGICAL_PERMILLE;
	/* Marginal: over the operator's airtime cap (0 = off). */
	bool over_cap = cap_permille && probes >= CAD_STEP_MIN_PROBES &&
			b_op > cap_permille;

	if (!pathological && !over_cap) {
		return false;
	}

	stepOffset((int8_t)(_offset + 1));
	if (pathological) {
		LOG_WRN("cad: safety step up -> offset %d (busy %d permille over "
			"%u probes — detector too sensitive to clear, auto=%d)",
			(int)_offset, b_op, (unsigned)probes, (int)_auto);
	} else {
		LOG_INF("cad: step up -> offset %d (airtime, busy %d cap %d)",
			(int)_offset, b_op, cap_permille);
	}
	return true;
}

/* Ignores auto, the cap and the stats: the caller has seen the harm directly.
 * One step at a time. Runs during a visit too (a muted node is muted for
 * real), stepping the visitor's offset so nothing reaches prefs. */
bool CadController::relaxOnTxStarvation()
{
	int8_t &off = _visiting ? _temp_offset : _offset;

	if (off >= levelMaxEff()) {
		return false;
	}

	off++;
	_hw.hwCadSetPeakOffset(effectiveOffset());
	LOG_WRN("cad: TX starvation override -> offset %d (auto=%d%s) — LBT was "
		"refusing every transmit",
		(int)off, (int)_auto,
		_visiting ? ", temp preset" : "");
	return true;
}

} /* namespace mesh */
