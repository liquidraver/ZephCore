/*
 * SPDX-License-Identifier: MIT
 *
 * ThinkNode M9 single-cell LiPo OCV curve — 11 points at 10% steps.
 * Index 0 = 100% (4200 mV), index 10 = 0% (3100 mV).
 */

#include "battery_curve.h"

static const uint16_t ocv_thinknode_m9[11] = {
	4200, 4080, 3980, 3920, 3870, /* 100 .. 60% */
	3820, 3790, 3750, 3700, 3600, /*  50 .. 10% */
	3100,                          /*   0%       */
};

const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_thinknode_m9,
	.num_points = 11,
	.num_cells  = 1,
};
