/*
 * SPDX-License-Identifier: MIT
 *
 * CayenneLPP telemetry as display text, one field per line (the joystick UI's
 * repeater admin screen). Channel 1 is the node itself (battery, MCU
 * temperature, GPS); every other channel is a sensor, shown with a "cN "
 * prefix. Types it does not show are skipped by their size (upstream's
 * LPPData), never a reason to stop. No Zephyr dependencies (host-tested).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/* Writes a NUL-terminated string; "no telemetry" when nothing was shown. */
void telemetry_text(const uint8_t *data, uint8_t len, char *out, size_t cap);
