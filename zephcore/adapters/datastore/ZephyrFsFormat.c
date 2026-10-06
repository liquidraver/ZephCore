/*
 * SPDX-License-Identifier: MIT
 * ZephyrFsFormat - shared factory-format of every ZephCore storage region.
 */

#include "ZephyrFsFormat.h"
#include "ZephyrFsUtil.h"

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/pm/device.h>
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

#if PARTITION_EXISTS(qspi_storage_partition)
/* qspi-ext.dtsi makes the flash under /ext zephyr,deferred-init, so nothing
 * probes it at boot. The flash sits behind a GPIO-switched rail on several
 * boards, and the boot-time JEDEC read went out before a cold rail had come
 * up: the driver failed, /ext stayed unmounted and contacts silently fell
 * back to internal flash. A warm reboot keeps the rail charged; the first
 * boot after a UF2 update (seconds in the bootloader, rail off) did not.
 * Bringing the part up here, on first use, is long after the rail. Each role
 * reaches this on its own path, so it is idempotent: -EALREADY means an
 * earlier call already ran the init, whatever the result was.
 *
 * A role that does not use /ext parks the part in deep power-down
 * (zephcore_fs_ext_power_down()); a format still has to reach it, so a
 * suspended flash is resumed here. */
static void ext_flash_init(void)
{
	const struct device *dev = PARTITION_DEVICE(qspi_storage_partition);
	enum pm_device_state state;

	if (!device_is_ready(dev)) {
		int rc = device_init(dev);

		LOG_INF("%s flash init: rc=%d ready=%d", EXT_MNT_POINT, rc,
			(int)device_is_ready(dev));
	} else if (pm_device_state_get(dev, &state) == 0 &&
		   state == PM_DEVICE_STATE_SUSPENDED) {
		int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);

		LOG_INF("%s flash resume: rc=%d", EXT_MNT_POINT, rc);
	}
}
#endif

void zephcore_fs_ext_power_down(void)
{
#if PARTITION_EXISTS(qspi_storage_partition)
	const struct device *dev = PARTITION_DEVICE(qspi_storage_partition);
	enum pm_device_state state;
	int rc;

	if (device_is_ready(dev) && pm_device_state_get(dev, &state) == 0 &&
	    state == PM_DEVICE_STATE_SUSPENDED) {
		return;
	}

#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
	/* A first-boot format leaves /ext mounted (zephcore_fs_format_all()). */
	if (zephcore_fs_is_mounted(EXT_MNT_POINT)) {
		FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
		fs_unmount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
	}
#endif

	/* The part has to be brought up to be put down: left deferred, nothing
	 * ever configures its pads, and a floating CS# reads low -- the flash
	 * sits selected for as long as the node runs (measured on a Wio Tracker
	 * L1 repeater).  Init drives CS# and sends the part its commands;
	 * suspend sends Deep Power-Down and applies the sleep pinctrl state,
	 * which must keep a pull-up on CS#. */
	ext_flash_init();
	if (!device_is_ready(dev)) {
		LOG_WRN("%s flash not ready, not powered down", EXT_MNT_POINT);
		return;
	}
	rc = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
	if (rc == 0) {
		LOG_INF("%s flash in deep power-down", EXT_MNT_POINT);
	} else {
		LOG_WRN("%s flash power-down failed: %d", EXT_MNT_POINT, rc);
	}
#endif
}

bool zephcore_fs_mount_ext(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
	if (zephcore_fs_is_mounted(EXT_MNT_POINT)) {
		return true;
	}
	ext_flash_init();

	/* fs_mount() auto-formats blank flash and mounts valid data untouched. */
	FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
	int rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
	bool mounted = zephcore_fs_is_mounted(EXT_MNT_POINT);

	if (mounted) {
		LOG_INF("%s mounted (rc=%d)", EXT_MNT_POINT, rc);
	} else {
		LOG_ERR("%s mount failed (rc=%d)", EXT_MNT_POINT, rc);
	}
	return mounted;
#else
	return false;
#endif
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

#if PARTITION_EXISTS(lfs_partition)
	flatten(PARTITION_ID(lfs_partition), "lfs_partition");
#endif

#if PARTITION_EXISTS(storage_partition)
	/* BLE bonds (NVS).  A factory reset should clear them too; the caller
	 * reboots so NVS and the BT stack re-init clean. */
	flatten(PARTITION_ID(storage_partition), "storage_partition");
#endif

#if PARTITION_EXISTS(qspi_storage_partition)
	/* A repeater never mounts /ext, so its flash is still uninitialised. */
	ext_flash_init();
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
		bool ext_mounted = zephcore_fs_mount_ext();

		if (out_ext_mounted) {
			*out_ext_mounted = ext_mounted;
		}
	}
#endif

	return mounted;
}
