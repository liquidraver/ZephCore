/*
 * SPDX-License-Identifier: MIT
 *
 * RAK WisMesh Tag single-cell LiPo OCV curve — 21 points at 5% steps.
 * 11-point base linearly interpolated to 5% resolution.
 * Index 0 = 100% (4160 mV), index 20 = 0% (2990 mV).
 *
 * Notable shape: very compressed upper range (100% = 4160 mV, not 4190),
 * flat plateau 3.72–3.76 V, steep cliff below 3.62 V to 2990 mV cutoff.
 */

#include "battery_curve.h"

static const uint16_t ocv_rak_wismesh_tag[21] = {
	4160, 4090, 4020, 3980, 3940, /* 100 .. 80% */
	3905, 3870, 3840, 3810, 3785, /*  75 .. 55% */
	3760, 3750, 3740, 3730, 3720, /*  50 .. 30% */
	3700, 3680, 3650, 3620, 3305, /*  25 ..  5% */
	2990,                          /*   0%       */
};

const battery_curve_t battery_curve_default = {
	.ocv_mv     = ocv_rak_wismesh_tag,
	.num_points = 21,
	.num_cells  = 1,
};
