/*
 * SPDX-License-Identifier: MIT
 */

#include "battery_curve.h"

/*
 * Generic single-cell LiPo OCV curve — 21 points at 5% steps.
 * Derived from measured LiPo discharge data; 11-point base expanded to
 * 21 points by linear interpolation for better knee resolution.
 *
 * Index 0 = 100% (4190 mV), index 20 = 0% (3100 mV).
 * 100% is 4190 mV, not 4200 mV — reflects observed charge ceiling across
 * multiple boards after the charger transitions to maintenance mode.
 */
static const uint16_t ocv_generic[21] = {
	4190, 4120, 4050, 4020, 3990, /* 100 .. 80% */
	3940, 3890, 3845, 3800, 3760, /*  75 .. 55% */
	3720, 3675, 3630, 3580, 3530, /*  50 .. 30% */
	3475, 3420, 3360, 3300, 3200, /*  25 ..  5% */
	3100,                          /*   0%       */
};

__attribute__((weak))
const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_generic,
	.num_points = 21,
	.num_cells  = 1,
};

uint8_t battery_curve_lookup(const battery_curve_t *curve, uint16_t mv)
{
	if (mv == 0) {
		return 0;
	}

	uint16_t cell_mv = mv / curve->num_cells;
	uint8_t  n       = curve->num_points;

	if (cell_mv >= curve->ocv_mv[0]) {
		return 100;
	}
	if (cell_mv <= curve->ocv_mv[n - 1]) {
		return 0;
	}

	/* Each segment spans 100/(n-1) percent. For n=21 this is exactly 5. */
	uint8_t seg_pct = (uint8_t)(100 / (n - 1));

	for (uint8_t i = 1; i < n; i++) {
		if (cell_mv >= curve->ocv_mv[i]) {
			uint8_t  pct_low = (uint8_t)(100 - (uint16_t)i * 100 / (n - 1));
			uint32_t num     = (uint32_t)(cell_mv - curve->ocv_mv[i]) * seg_pct;
			uint16_t den     = curve->ocv_mv[i - 1] - curve->ocv_mv[i];
			return (uint8_t)(pct_low + num / den);
		}
	}

	return 0;
}
