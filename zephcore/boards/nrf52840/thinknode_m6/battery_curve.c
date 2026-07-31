/*
 * SPDX-License-Identifier: MIT
 *
 * ThinkNode M6 single-cell LiPo OCV curve — 21 points at 5% steps.
 * 11-point base linearly interpolated to 5% resolution.
 * Index 0 = 100% (4080 mV), index 20 = 0% (3450 mV).
 *
 * Notable shape: lower full-charge ceiling (4080 mV) suggesting a conservative
 * charge termination; smooth linear discharge with no sharp cliff at the bottom.
 */

#include "battery_curve.h"

static const uint16_t ocv_thinknode_m6[21] = {
	4080, 4035, 3990, 3962, 3935, /* 100 .. 80% */
	3907, 3880, 3852, 3825, 3797, /*  75 .. 55% */
	3770, 3742, 3715, 3687, 3660, /*  50 .. 30% */
	3632, 3605, 3577, 3550, 3500, /*  25 ..  5% */
	3450,                          /*   0%       */
};

const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_thinknode_m6,
	.num_points = 21,
	.num_cells  = 1,
};
