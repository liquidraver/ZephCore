# ADR 0008 — Storage layout, power-loss writes, BLE bonds in NVS

Status: accepted (recorded 2026-09-25)

## Context

- `/lfs` — internal flash: identity (`_main.id`), prefs (`prefs.json`, upstream's `ConfigSerializer` format;
  the legacy binary `new_prefs` is migrated once and kept for downgrades)
- `/ext` — QSPI external flash (when present): contacts, channels, blob LRU
- **Power-loss writes:** identity, prefs, and `channels2` use atomic replace (`.tmp` + `fs_sync` + `fs_rename`,
  one implementation: `adapters/datastore/ZephyrFsUtil.c`); `contacts3` only on `/ext` (two copies do not fit a
  128 KB `/lfs`); repeater `/lfs/repeater/` identity and prefs use the same pattern. BLE bonds are handled by the
  Zephyr settings subsystem, not `ZephyrDataStore`, and the backend is **platform-dependent**:
  `zephcore_common.conf` defaults to the file backend at `/lfs/settings`, but `nrf52_common.conf` and
  `esp32_common.conf` both override it to `CONFIG_SETTINGS_NVS=y` (`SETTINGS_FILE=n`), putting bonds in a
  dedicated `storage_partition` NVS region isolated from the `/lfs` LittleFS volume, so a busy bond store cannot
  corrupt user data. The file backend therefore only applies to platforms that do not override it. This moved in
  1.16.2; `/lfs/settings` surviving on disk is the pre-1.16.2 marker that
  `ZephyrDataStore::hasOldSettingsFile()` tests for.
- The shutdown-reason marker `/lfs/shutdn` (one byte) is written by `adapters/board/zephyr_poweroff.c`.

## Decision

As above: atomic replace for everything that must survive a power cut mid-write; bonds isolated in NVS on
nRF52/ESP32.

## Consequences

A role switch reformats `/lfs` (identity lost unless exported first); a merged-bin ESP32 flash never touches
`storage_partition`/`lfs_partition` (ADR 0001).
