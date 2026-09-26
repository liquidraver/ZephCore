/*
 * SPDX-License-Identifier: MIT
 * Observer runtime credentials — stored in LittleFS, configured via serial CLI.
 *
 * All connection parameters are runtime-configurable so no credentials ever
 * appear in the codebase. File path: /lfs/repeater/obs_creds
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "../adapters/mqtt/uplink_creds.h"  /* struct ObserverCreds, observer_creds_init() */

#ifdef __cplusplus
extern "C" {
#endif

/* Load creds from /lfs/repeater/obs_creds.
 * Returns true on success; on failure fills struct with safe zero defaults. */
bool observer_creds_load(struct ObserverCreds *creds, const char *base_path);

/* Save creds to /lfs/repeater/obs_creds. Returns true on success. */
bool observer_creds_save(const struct ObserverCreds *creds, const char *base_path);

#ifdef __cplusplus
}
#endif
