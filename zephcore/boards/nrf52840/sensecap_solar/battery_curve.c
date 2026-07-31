/*
 * SPDX-License-Identifier: MIT
 *
 * SenseCAP Solar single-cell LiPo OCV curve — 21 points at 5% steps.
 * 11-point base linearly interpolated to 5% resolution.
 * Index 0 = 100% (4200 mV), index 20 = 0% (2786 mV).
 *
 * Notable shape: steeper overall slope than a standard LiPo, consistent with
 * a high-impedance cell measured under load rather than at true OCV. Low
 * cutoff (2786 mV) reflects the solar node's extended discharge budget.
 */

#include "battery_curve.h"

static const uint16_t ocv_sensecap_solar[21] = {
	4200, 4093, 3986, 3954, 3922, /* 100 .. 80% */
	3867, 3812, 3773, 3734, 3689, /*  75 .. 55% */
	3645, 3586, 3527, 3473, 3420, /*  50 .. 30% */
	3350, 3281, 3184, 3087, 2936, /*  25 ..  5% */
	2786,                          /*   0%       */
};

const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_sensecap_solar,
	.num_points = 21,
	.num_cells  = 1,
};
