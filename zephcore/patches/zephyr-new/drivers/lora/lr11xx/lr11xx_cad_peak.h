/*
 * SPDX-License-Identifier: MIT
 * LR11xx CAD detPeak base table. Pure, so the host unit tests can pin it.
 */

#ifndef LR11XX_CAD_PEAK_H_
#define LR11XX_CAD_PEAK_H_

#include <stdint.h>

/* Recommended cad_detect_peak, from Semtech's own reference stack:
 * LoRa Basics Modem v4.9.0, ral_lr11xx.c ral_lr11xx_get_lora_cad_det_peak().
 *
 * Provenance matters here, because the table this replaces was wrong twice
 * over.  It was `{56,56,56,58,58,60,64,68}`, labelled "from SX1261/62/68 /
 * LR1110 reference (same silicon IP)" — but that is byte-for-byte the *LR20xx*
 * 2-symbol row (ral_lr20xx.c), i.e. the wrong chip family, sampled at the wrong
 * symbol count.  The SX126x scale is ~20-35 and shares nothing with this one.
 * The error was worst at SF6/SF7, where it read 56 against Semtech's 52, and it
 * is what drove field units eight rungs down to the offset rail: measured on
 * two T1000-E companions at SF7/BW62.5, both pinned at o:-8 with a flat, clean
 * FP curve, one of them also sitting on the driver's own peak clamp.
 *
 * Unlike the LR20xx, this family's detPeak is strongly bandwidth-dependent —
 * at SF7 Semtech spans 52/64/77 across BW125/250/500, ~12 counts per octave,
 * against ~1-3 per octave on the SX126x.  A bandwidth-blind base table is
 * therefore a much larger error here than it is there, which is precisely why
 * the SX1262 in the same room settled at offset -1 while these walked to -8.
 *
 * Below BW125 Semtech returns RAL_STATUS_UNKNOWN_VALUE and offers nothing.  We
 * run BW62.5 by default, so that gap is our normal operating point, and the
 * sub-125 row below is MEASURED rather than published.
 *
 * Provenance and its honest limit: over 2026-08-30..09-01 two LR1110 T1000-Es —
 * one here, one in another country, on different sites — both converged to an
 * absolute detPeak of 44 at SF7/BW62.5, i.e. offset -7 against the BW125 row's
 * 51 (52 minus the 4-symbol correction).  Two independent sites agreeing to the
 * count is what makes this a measurement; three SX1262s in the same campaign
 * landed within one count of each other across a house, a roof and a
 * mountaintop, which is the general finding that the site scales the FP curve
 * without moving its bend.
 *
 * The correction is applied FLAT, as a -7 translation of the whole BW125 row.
 * A constant preserves the per-SF shape Semtech actually measured; scaling each
 * SF by its own bandwidth slope would lean on their noisiest dimension (SF9
 * steps +5 then +15 across the two published octaves) and distort that shape
 * from a single anchor point.  Flat also errs more sensitive at high SF, the
 * safe side of an asymmetric offset range (-8 down, +12 up).
 *
 * Sanity check on the magnitude, since only one SF was measured: Semtech's own
 * SF7 trend is 52/64/77 across BW125/250/500, about 12 counts per octave, so a
 * linear extrapolation one octave down would predict -12 and a base of 40.  The
 * measured -7 sits between zero (what we shipped before) and that, which is the
 * shape expected from a curve flattening at the narrow end.  The measurement is
 * not fighting the trend; it lands inside the bracket the trend allows.
 *
 * ONE SF MEASURED, SEVEN EXTRAPOLATED.  That is the real limit of this row, and
 * it is acceptable only because the closed-loop staircase exists to find the
 * local value from a starting point — this makes the start honest, it does not
 * claim to be the answer. */
static inline uint8_t lr11xx_cad_detect_peak(uint8_t sf, uint16_t bw_khz, uint8_t symb_nb)
{
	/*        SF5 SF6 SF7 SF8 SF9 SF10 SF11 SF12 */
	static const uint8_t bw500[8] = { 65, 70, 77, 85, 78, 80, 79, 82 };
	static const uint8_t bw250[8] = { 60, 61, 64, 72, 63, 71, 73, 75 };
	static const uint8_t bw125[8] = { 56, 52, 52, 58, 58, 62, 66, 68 };
	/* bw125 - 7, measured at SF7/BW62.5 on two sites.  See the note above. */
	static const uint8_t bw_sub125[8] = { 49, 45, 45, 51, 51, 55, 59, 61 };
	const uint8_t *row;
	int peak;

	if (sf < 5 || sf > 12) {
		sf = 9;  /* mid-range fallback */
	}

	if (bw_khz >= 500) {
		row = bw500;
	} else if (bw_khz >= 250) {
		row = bw250;
	} else if (bw_khz >= 125) {
		row = bw125;
	} else {
		/* Narrower than BW125: Semtech publishes nothing, we measured. */
		row = bw_sub125;
	}
	peak = (int)row[sf - 5];

	/* More symbols means more looks at the same correlation, so the same
	 * detection quality is reached at a lower threshold.  Semtech applies
	 * this correction after the table lookup; we run 4 symbols everywhere
	 * (LORA_CAD_SYMB_4 in LoRaRadio::buildModemConfig), so it always
	 * bites, and omitting it was one further count of the SF7 error. */
	if (symb_nb >= 8) {
		peak -= 2;
	} else if (symb_nb >= 4) {
		peak -= 1;
	}

	return (uint8_t)peak;
}

/* The detPeak range this driver will actually program.  Exported through
 * lr11xx_cad_peak_min/max() so the C++ adaptive-CAD controller can narrow its
 * offset window to match: where base+offset falls outside this, several offsets
 * collapse onto one peak and the staircase reads sampling noise between
 * identical configurations as curvature.  That is not hypothetical — it is the
 * documented failure mode on the LR2021 (see LLD 03 §13, level window), and
 * the old 48 floor here reproduced it on the LR1110 at SF7.
 *
 * 40 is DELIBERATELY left where it was when the sub-125 row was measured down
 * to 45 (base 44 after the 4-symbol correction), which means it now binds:
 * 44 + CAD_LEVEL_MIN(-8) = 36 is below it, so CadController::levelMinEff() narrows the
 * offset window to -4..+12 at SF6 and SF7 below BW125.  That narrowing is
 * intended, and it must not be "fixed" by lowering this constant.
 *
 * The reason is what the sub-125 row is built on: two LR1110s, on different
 * sites in different countries, converged to the SAME absolute detPeak.  A base
 * anchored by two independent agreeing measurements does not need eight rungs
 * of downward travel — it needs to be centred, which it now is.  The old -8
 * window was sized for a base that was wrong by seven counts; carrying that
 * much headroom onto a corrected base would be carrying the symptom across the
 * fix.  CAD_SWEEP_MIN is -4, so the dry-run sweep still fits exactly.
 *
 * Only SF6 and SF7 below BW125 narrow at all.  SF5 keeps the full window by one
 * count (48 - 8 = 40), and every other cell sits well clear:
 *
 *   sub-125 base (4 sym)   SF5 48  SF6 44  SF7 44  SF8 50 ... SF12 60
 *   effective min offset       -8      -4      -4      -8         -8
 *
 * SF7/BW62.5 is of course the default preset, so the one configuration that
 * narrows is the one that matters — which is the point, since it is also the
 * only one anyone has measured.  The upper bound is untouched: the highest base
 * is 85 (SF8, BW500) against CAD_LEVEL_MAX +12.
 *
 * Watch item: if a third, quieter LR1110 site ever rails at -4, that is the
 * signal to revisit this — and it is the third data point the sub-125 row wants
 * in any case. */
#define LR11XX_CAD_PEAK_MIN 40
#define LR11XX_CAD_PEAK_MAX 100

#endif /* LR11XX_CAD_PEAK_H_ */
