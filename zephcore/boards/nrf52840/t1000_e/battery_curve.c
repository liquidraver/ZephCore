/*
 * SPDX-License-Identifier: MIT
 *
 * T1000-E single-cell LiPo OCV curve — 21 points at 5% steps.
 * 11-point base linearly interpolated to 5% resolution.
 * Index 0 = 100% (4190 mV), index 20 = 0% (3100 mV).
 *
 * Notable shape: very flat plateau 3.72–3.82 V, steep cliff below 3.64 V.
 */

#include "battery_curve.h"

static const uint16_t ocv_t1000_e[21] = {
	4190, 4116, 4042, 3999, 3957, /* 100 .. 80% */
	3921, 3885, 3852, 3820, 3798, /*  75 .. 55% */
	3776, 3761, 3746, 3735, 3725, /*  50 .. 30% */
	3710, 3696, 3670, 3644, 3372, /*  25 ..  5% */
	3100,                          /*   0%       */
};

const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_t1000_e,
	.num_points = 21,
	.num_cells  = 1,
};
