/*
 * SPDX-License-Identifier: MIT
 * PrefsCodec - byte codecs for the two binary prefs files.
 *
 * Companion /lfs/new_prefs and server /lfs/repeater/prefs are serialized field
 * by field, not as struct dumps; the offsets are the file layout (byte maps in
 * memory prefs-format.md). Zephyr-free so the host tests can pin both layouts.
 * The stores only move bytes between these and the filesystem.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "NodePrefs.h"

#define COMPANION_PREFS_SIZE  272
#define SERVER_PREFS_SIZE     311

/* Companion file layout, including the fields shared with upstream's former
 * new_prefs up to offset 92. Returns the byte count (COMPANION_PREFS_SIZE),
 * or 0 if cap is too small. */
size_t companionPrefsEncode(const NodePrefs &p, uint8_t *buf, size_t cap);

/* Returns false and leaves p untouched when the data cannot be ours: shorter
 * than 90 bytes, or radio parameters outside ZC_RADIO_*. Fields past len keep
 * their value in p, which the caller has filled with initNodePrefs(). */
bool companionPrefsDecode(NodePrefs &p, const uint8_t *buf, size_t len);

/* Repeater / room server / observer layout (upstream CommonCLI order to 290,
 * then ZephCore's tail). Returns SERVER_PREFS_SIZE, or 0 if cap is too small. */
size_t serverPrefsEncode(const NodePrefs &p, uint8_t *buf, size_t cap);

/* Fields past len keep their value in p. Radio parameters outside ZC_RADIO_*
 * are replaced by the LoRaConfig defaults; everything is sanitized. */
void serverPrefsDecode(NodePrefs &p, const uint8_t *buf, size_t len);
