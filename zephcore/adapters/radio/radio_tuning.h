/*
 * SPDX-License-Identifier: MIT
 * Tuning constants of the radio algorithms (noise-floor estimator, adaptive
 * CAD controller). Zephyr-free so the algorithms build on the host for unit
 * tests. Rationale and measurements: devdocs/lld/03-radio-contract.md §13.
 */

#pragma once

/* --- Noise floor (EMA of median-of-N idle RSSI) --- */
#define NOISE_FLOOR_EMA_SHIFT            3   /* alpha = 1/(1<<3) = 1/8; also the warm-up tick count */
#define NOISE_FLOOR_SAMPLES_PER_TICK     8   /* median of 8 reads per tick */
#define NOISE_FLOOR_UNGUARDED_INTERVAL   16  /* ticks between unfiltered samples (power of 2) */
#define NOISE_FLOOR_SAMPLING_THRESHOLD   14  /* dB above floor to reject as interference */
#define NOISE_FLOOR_CEILING_DBM          (-50) /* upper clamp */
#define DEFAULT_NOISE_FLOOR              0   /* sentinel: seed from first sample */

/* --- Adaptive CAD (LBT detPeak calibration) ---
 * Levels are signed offsets from the chip family's per-SF/per-BW base detPeak.
 * The window must match CAD_OFFSET_MIN/MAX in helpers/NodePrefs.h. */
#define CAD_LEVEL_MIN            (-8)  /* most sensitive probe level */
#define CAD_LEVEL_MAX            12    /* least sensitive probe level */
#define CAD_NUM_LEVELS           (CAD_LEVEL_MAX - CAD_LEVEL_MIN + 1)
#define CAD_SWEEP_MIN            (-4)  /* dry-run sweep window (auto off) */
#define CAD_SWEEP_MAX            4
/* Knee-seeking staircase: read the FP-vs-level slope from three rungs. */
#define CAD_KNEE_SLOPE_PERMILLE     50    /* >=5%/level FP change = steep */
#define CAD_PLATEAU_CLEAN_PERMILLE  50    /* <=5% FP = clean enough to descend */
#define CAD_STEP_MIN_PROBES         120   /* per-level samples before a step call */
/* Descend only if the frontier's busy rate is <= (100 - HYST)% of the cap. */
#define CAD_BUSY_DEFER_HYST_PCT      40
/* Safety rung (runs regardless of cad.auto; see CadController::safetyStep). */
#define CAD_SAFETY_MIN_PROBES          20   /* samples before the fast path may act */
#define CAD_SAFETY_PATHOLOGICAL_PERMILLE 900 /* >=90% busy = detector, not channel */
#define CAD_PROBE_RSSI_GUARD     7     /* dB above floor = channel visibly busy, skip probe */
#define CAD_STATS_DECAY_MS       (6UL * 3600UL * 1000UL)  /* halve counters every 6 h */
