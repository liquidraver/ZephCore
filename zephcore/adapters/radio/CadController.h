/*
 * SPDX-License-Identifier: MIT
 * Adaptive CAD controller: learns the LBT detPeak offset from probe verdicts.
 *
 * Pure algorithm over a four-call hardware interface. LoRaRadio decides
 * when to probe, runs the probe and feeds the verdicts in; this class keeps
 * the per-level statistics and moves the operating offset. Design notes:
 * devdocs/lld/03-radio-contract.md §13 and docs/ADAPTIVE_CAD.md.
 */

#pragma once

#include <stdint.h>
#include "radio_tuning.h"

namespace mesh {

/* The detPeak side of the chip, as the controller sees it. */
class CadHw {
public:
	/* Family base detPeak at the current SF/BW; 0 = no adaptive CAD. */
	virtual uint8_t hwCadBasePeak() = 0;
	/* The driver's absolute detPeak clamp; 0 = none reported. */
	virtual uint8_t hwCadPeakMin() = 0;
	virtual uint8_t hwCadPeakMax() = 0;
	/* Program the offset used by every subsequent LBT CAD. */
	virtual void hwCadSetPeakOffset(int8_t offset) = 0;

protected:
	~CadHw() = default;
};

class CadController {
public:
	struct LevelStats {
		uint16_t probes;   /* probes run at this level */
		uint16_t busy;     /* raw busy verdicts */
		uint16_t fp;       /* busy with nothing decoded after it (false positive) */
		uint16_t tp;       /* busy that turned into a received packet */
	};

	explicit CadController(CadHw &hw);

	/* Apply the CAD prefs. stored_base is the base the persisted offset was
	 * learned against (0 = unknown); across a base-table change the PEAK is
	 * kept, not the offset. Programs the hardware. */
	void configure(bool auto_enabled, int8_t offset, uint8_t busycap_pct,
		       uint8_t stored_base);

	bool autoEnabled() const { return _auto; }
	uint8_t busycapPct() const { return _busycap_pct; }
	/* The configured offset: the value that is persisted. */
	int8_t offset() const { return _offset; }
	/* The offset the chip is programmed with (the visitor's during a visit). */
	int8_t effectiveOffset() const { return _visiting ? _temp_offset : _offset; }

	/* A temporary radio override is up. new_preset: the radio moved to a
	 * preset it was not running, so the visit starts at that preset's base
	 * (offset 0); otherwise the running offset is kept. Adaptation is
	 * suspended and nothing the visit does reaches offset(). */
	void beginVisit(bool new_preset);
	void endVisit();
	bool visiting() const { return _visiting; }

	/* The level window the controller may use: [CAD_LEVEL_MIN, CAD_LEVEL_MAX]
	 * narrowed so every level programs a distinct peak under the driver's
	 * clamp. Stats stay indexed by the static window. */
	int8_t levelMinEff();
	int8_t levelMaxEff();

	void resetStats();
	void decayStats();
	const LevelStats &stats(int level) const { return _stats[level - CAD_LEVEL_MIN]; }

	/* Probing. */
	int8_t pickProbeLevel();
	void recordProbe(int8_t level, bool busy);
	/* A busy probe whose ground truth arrives later (CAD_RX). */
	void setPending(int8_t level, int64_t deadline_ms)
	{
		_pending_level = level;
		_pending_deadline_ms = deadline_ms;
	}
	bool hasPending() const { return _pending_level != INT8_MIN; }
	int8_t pendingLevel() const { return _pending_level; }
	int64_t pendingDeadline() const { return _pending_deadline_ms; }
	/* outcome: 1 = packet (true positive), 2 = chip timeout (false
	 * positive), anything else = unresolved, the sample is withdrawn. */
	void resolvePending(int outcome);

	/* After a verdict: the safety rung first; the optimiser only when the
	 * safety did not act and auto is on. */
	void adapt()
	{
		if (!safetyStep() && _auto) {
			staircaseStep();
		}
	}
	/* Airtime protection. Runs regardless of auto. True when it moved. */
	bool safetyStep();
	/* Knee-seeking optimiser. */
	void staircaseStep();
	/* The node's own TX is being refused: one step less sensitive, whatever
	 * auto and the stats say. During a visit it steps the visitor's offset. */
	bool relaxOnTxStarvation();

private:
	int fpRate(int idx) const;
	int busyRate(int idx) const;
	void stepOffset(int8_t to);

	CadHw &_hw;
	LevelStats _stats[CAD_NUM_LEVELS];
	bool _auto;
	int8_t _offset;
	int8_t _temp_offset;
	bool _visiting;
	uint8_t _busycap_pct;
	uint8_t _probe_rr;
	int8_t _pending_level;
	int64_t _pending_deadline_ms;
};

} /* namespace mesh */
