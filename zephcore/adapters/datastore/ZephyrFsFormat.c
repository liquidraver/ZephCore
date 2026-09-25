/*
 * SPDX-License-Identifier: MIT
 * ZephyrFsFormat - shared factory-format of every ZephCore storage region.
 */

#include "ZephyrFsFormat.h"
#include "ZephyrFsUtil.h"

#include <zephyr/devicetree.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_fs_format, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

#define LFS_MNT_POINT "/lfs"
#define EXT_MNT_POINT "/ext"

/* fs_mount() can return 0 having mounted nothing useful, and a remount that
 * silently failed would leave the caller reporting a healthy store over an
 * unmounted volume.  Ask the VFS instead of trusting the return code. */
static void flatten(uint8_t id, const char *tag)
{
	const struct flash_area *fap;
	int rc = flash_area_open(id, &fap);

	if (rc != 0) {
		LOG_WRN("format: flash_area_open(%s) failed: %d", tag, rc);
		return;
	}
	LOG_INF("format: erasing %s (%u bytes)", tag, (unsigned)fap->fa_size);
	rc = flash_area_flatten(fap, 0, fap->fa_size);
	if (rc != 0) {
		LOG_ERR("format: flatten(%s) failed: %d", tag, rc);
	}
	flash_area_close(fap);
}

bool zephcore_fs_format_all(bool *out_ext_mounted)
{
	LOG_INF("zephcore_fs_format_all: starting...");

	if (out_ext_mounted) {
		*out_ext_mounted = false;
	}

	/* Properly unmount from Zephyr's VFS before erasing flash.  Clearing a
	 * local "mounted" flag is not enough — Zephyr would still hold /lfs
	 * mounted, so flash_area_flatten destroys the on-flash superblock while
	 * LittleFS considers itself active.  Every subsequent file op then hits
	 * the erased blocks and logs "Corrupted dir pair at {0x0, 0x1}".
	 * FS_FSTAB_DECLARE_ENTRY exposes the non-static mount struct generated
	 * from the DTS fstab; fs_mount() on a blank partition auto-formats
	 * (littlefs_fs.c: lfs_mount fail -> lfs_format -> lfs_mount). */
	FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(lfs));
	fs_unmount(&FS_FSTAB_ENTRY(DT_NODELABEL(lfs)));

#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
	FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
	fs_unmount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
#endif

#if FIXED_PARTITION_EXISTS(lfs_partition)
	flatten(PARTITION_ID(lfs_partition), "lfs_partition");
#endif

#if FIXED_PARTITION_EXISTS(storage_partition)
	/* BLE bonds (NVS).  A factory reset should clear them too; the caller
	 * reboots so NVS and the BT stack re-init clean. */
	flatten(PARTITION_ID(storage_partition), "storage_partition");
#endif

#if FIXED_PARTITION_EXISTS(qspi_storage_partition)
	flatten(PARTITION_ID(qspi_storage_partition), "qspi_storage_partition");
#endif

	/* Remount: littlefs_mount() auto-formats blank flash, then mounts. */
	int rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(lfs)));
	bool mounted = zephcore_fs_is_mounted(LFS_MNT_POINT);

	if (mounted) {
		LOG_INF("format: %s remounted (rc=%d)", LFS_MNT_POINT, rc);
	} else {
		LOG_ERR("format: %s remount FAILED (rc=%d)", LFS_MNT_POINT, rc);
	}

#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
	/* Remount external QSPI too.  We unmounted it above and flattened its
	 * partition, so it must be re-mounted here — otherwise a runtime format
	 * (factory reset, or the first-boot "no prefs" auto-format) leaves /ext
	 * unmounted for the rest of the session.  begin() then reads
	 * ext_lfs_mounted=false and the store falls back to internal /lfs, so
	 * contacts/channels save to /lfs and get needlessly migrated back to
	 * /ext on the next boot ("Migrating contacts to external storage"). */
	{
		int ext_rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
		bool ext_mounted = zephcore_fs_is_mounted(EXT_MNT_POINT);

		if (ext_mounted) {
			LOG_INF("format: %s remounted (rc=%d)", EXT_MNT_POINT, ext_rc);
		} else {
			LOG_ERR("format: %s remount failed (rc=%d)", EXT_MNT_POINT, ext_rc);
		}
		if (out_ext_mounted) {
			*out_ext_mounted = ext_mounted;
		}
	}
#endif

	return mounted;
}
