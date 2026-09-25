/*
 * SPDX-License-Identifier: MIT
 * PrefsFile - read and write a prefs.json file for either role's store.
 */

#pragma once

#include <PrefsJson.h>

typedef bool (*PrefsFromJsonFn)(NodePrefs &p, Stream &in);
typedef bool (*PrefsToJsonFn)(const NodePrefs &p, Stream &out);

/* False if the file is absent, empty or does not parse; p is then untouched. */
bool zephcore_prefs_json_load(const char *path, NodePrefs &p, PrefsFromJsonFn from_json);

/* Power-safe replace; false (and the old file kept) on any short write. */
bool zephcore_prefs_json_save(const char *path, const NodePrefs &p, PrefsToJsonFn to_json);
