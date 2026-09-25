/*
 * SPDX-License-Identifier: MIT
 * PrefsJson - prefs.json for both roles, through upstream's ConfigSerializer.
 *
 * Fields whose meaning matches upstream's use its keys and nesting
 * (examples/companion_radio/NodePrefs.h for the companion, CommonCLI.h for
 * the server roles), so a file is readable by either tree for those fields.
 * ZephCore's own fields live under "zc". Unknown keys are ignored on load,
 * missing keys keep the value already in the struct.
 */

#pragma once

#include "ConfigSerializer.h"
#include "NodePrefs.h"

bool companionPrefsToJson(const NodePrefs &p, Stream &out);
bool serverPrefsToJson(const NodePrefs &p, Stream &out);

/* false on a parse error, and p is left untouched. On success p is
 * sanitized, and a radio preset outside ZC_RADIO_* is replaced by the
 * LoRaConfig defaults. */
bool companionPrefsFromJson(NodePrefs &p, Stream &in);
bool serverPrefsFromJson(NodePrefs &p, Stream &in);
