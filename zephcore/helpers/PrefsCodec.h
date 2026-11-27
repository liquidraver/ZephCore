/*
 * SPDX-License-Identifier: MIT
 * PrefsCodec - decoders for the two legacy binary prefs files, read once when a
 * node migrates to prefs.json.
 *
 * Companion /lfs/new_prefs and server /lfs/repeater/prefs are read field by
 * field, not as struct dumps; the offsets are the file layout (byte maps in
 * memory prefs-format.md, tables in tests/support/prefs_layout.h).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "NodePrefs.h"

#define COMPANION_PREFS_SIZE  272
#define SERVER_PREFS_SIZE     311

/* Companion layout, upstream's former new_prefs up to offset 92. Returns false
 * and leaves p untouched when the data cannot be ours: shorter than 90 bytes,
 * or radio parameters outside ZC_RADIO_*. Fields past len keep their value in
 * p, which the caller has filled with initNodePrefs(). */
bool companionPrefsDecode(NodePrefs &p, const uint8_t *buf, size_t len);

/* Repeater / room server / observer layout (upstream CommonCLI order to 290,
 * then ZephCore's tail). Fields past len keep their value in p. Radio
 * parameters outside ZC_RADIO_* are replaced by the LoRaConfig defaults;
 * everything is sanitized. */
void serverPrefsDecode(NodePrefs &p, const uint8_t *buf, size_t len);
