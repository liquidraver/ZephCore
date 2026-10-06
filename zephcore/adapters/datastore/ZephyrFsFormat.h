/*
 * SPDX-License-Identifier: MIT
 * ZephyrFsFormat - shared factory-format of every ZephCore storage region.
 *
 * Lives outside ZephyrDataStore because the repeater, room server and observer
 * build RepeaterDataStore instead (see CMakeLists role blocks) and must not
 * each grow their own half-implementation of this.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Erase every ZephCore storage region and remount the filesystems.
 *
 * Unmounts /lfs (and /ext when the board has QSPI), flattens lfs_partition,
 * storage_partition (BLE bonds NVS) and qspi_storage_partition where each
 * exists, then remounts.  A blank LittleFS partition is auto-formatted by
 * fs_mount(), so the volume comes back empty rather than corrupt.
 *
 * This is the only path that erases the LittleFS *volume*.  Deleting files
 * cannot recover a volume another firmware has written into — on nRF52840 the
 * Adafruit core's InternalFileSystem lives at 0xED000, inside our 0xD4000
 * lfs_partition, and its format() scribbles our top 7 blocks.
 *
 * @param out_ext_mounted  optional; receives whether /ext came back mounted.
 *                         Set to false on boards with no QSPI.
 * @return true if /lfs is mounted afterwards.
 */
bool zephcore_fs_format_all(bool *out_ext_mounted);

/**
 * @brief Mount /ext (QSPI) on first use, initialising its deferred flash.
 *
 * Idempotent. The flash is zephyr,deferred-init (qspi-ext.dtsi) and /ext is
 * not automounted, so this is the only way /ext comes up.
 *
 * @return true if /ext is mounted; always false on boards with no QSPI.
 */
bool zephcore_fs_mount_ext(void);

/**
 * @brief Put the /ext flash into deep power-down, for a role that keeps
 * nothing on it (repeater, room server, observer).
 *
 * Unmounts /ext if a format left it mounted, initialises the deferred flash,
 * then suspends it: the part gets its Deep Power-Down command and the bus
 * pads go to their sleep state.  Idempotent; a no-op on boards with no QSPI.
 * zephcore_fs_format_all() and zephcore_fs_mount_ext() resume the part.
 */
void zephcore_fs_ext_power_down(void);

#ifdef __cplusplus
}
#endif
