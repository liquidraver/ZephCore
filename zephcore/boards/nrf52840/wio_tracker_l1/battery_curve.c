/*
 * SPDX-License-Identifier: MIT
 *
 * Wio Tracker L1 battery curve — single-cell LiPo, 21 points at 5% steps.
 *
 * 100% = 4190 mV: observed charge ceiling on this board after the ADC
 * multiplier correction (vbat-mv-multiplier = 7236). Voltages above this
 * clamp to 100% in the lookup.
 *
 * The discharge shape below 4190 mV follows the generic LiPo profile.
 * Replace this table with measured discharge data for the specific cell
 * installed if a more accurate curve is needed.
 */

#include "battery_curve.h"

static const uint16_t ocv_wio_tracker_l1[21] = {
	4190, 4120, 4050, 4020, 3990, /* 100 .. 80% */
	3940, 3890, 3845, 3800, 3760, /*  75 .. 55% */
	3720, 3675, 3630, 3580, 3530, /*  50 .. 30% */
	3475, 3420, 3360, 3300, 3200, /*  25 ..  5% */
	3100,                          /*   0%       */
};

const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_wio_tracker_l1,
	.num_points = 21,
	.num_cells  = 1,
};
