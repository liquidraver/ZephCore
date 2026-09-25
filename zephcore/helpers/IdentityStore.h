/*
 * SPDX-License-Identifier: MIT
 * IdentityStore.h - Re-exports identity/crypto constants from MeshCore, and
 * FILESYSTEM for storage code ported from upstream (which defines it here as
 * fs::FS or Adafruit_LittleFS).
 */

#pragma once

#include <mesh/MeshCore.h>
#include <mesh/Identity.h>
#include <ZephyrFS.h>

#define FILESYSTEM  ZephyrFS
