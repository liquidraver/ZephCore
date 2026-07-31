/*
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	const uint16_t *ocv_mv;    /* descending — index 0 = 100%, last index = 0% */
	uint8_t         num_points;
	uint8_t         num_cells; /* 1 for single-cell packs (almost always 1) */
} battery_curve_t;

/* Default curve — __weak so a board-specific battery_curve.c can override it. */
extern const battery_curve_t battery_curve_default;

/* Returns 0–100 %. Returns 0 if mv == 0 (no battery / ADC absent). */
uint8_t battery_curve_lookup(const battery_curve_t *curve, uint16_t mv);

#ifdef __cplusplus
}
#endif
