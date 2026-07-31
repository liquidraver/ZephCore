/*
 * SPDX-License-Identifier: MIT
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 *
 * All platforms use DTS-automounted /lfs.
 * QSPI /ext overrides contacts mount when available.
 */

#include "ZephyrDataStore.h"
#include <AdvertDataHelpers.h>   // ADV_TYPE_NONE (transient/anon contacts)
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_datastore, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

#define MAX_ADVERT_PKT_LEN (2 + 32 + PUB_KEY_SIZE + 4 + SIGNATURE_SIZE + MAX_ADVERT_DATA_SIZE)

struct BlobRec {
	uint32_t timestamp;
	uint8_t key[7];
	uint8_t len;
	uint8_t data[MAX_ADVERT_PKT_LEN];
};

typedef bool (*AtomicWriteFn)(struct fs_file_t *file, void *ctx);

static bool atomicWriteTempFile(const char *path, AtomicWriteFn write_fn, void *ctx, const char *op_tag)
{
	char tmp_path[56];
	int pl = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
	if (pl <= 0 || pl >= (int)sizeof(tmp_path)) {
		LOG_ERR("%s: path too long", op_tag);
		return false;
	}

	fs_unlink(tmp_path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("%s: open %s failed: %d", op_tag, tmp_path, rc);
		return false;
	}

	bool write_ok = write_fn(&file, ctx);
	int sync_rc = fs_sync(&file);
	fs_close(&file);
	if (!write_ok || sync_rc < 0) {
		if (!write_ok) {
			LOG_ERR("%s: write failed", op_tag);
		}
		if (sync_rc < 0) {
			LOG_ERR("%s: sync failed: %d", op_tag, sync_rc);
		}
		fs_unlink(tmp_path);
		return false;
	}

	rc = fs_rename(tmp_path, path);
	if (rc < 0) {
		LOG_ERR("%s: rename %s -> %s failed: %d", op_tag, tmp_path, path, rc);
		fs_unlink(tmp_path);
		return false;
	}
	return true;
}

/* Track mount status - filesystems are automounted via DTS fstab */
static bool lfs_mounted;
static bool ext_lfs_mounted;

/* Check if a filesystem is mounted using fs_statvfs */
static bool is_mounted(const char *mount_point)
{
	struct fs_statvfs stat;
	return fs_statvfs(mount_point, &stat) == 0;
}

bool ZephyrDataStore::mount()
{
	if (lfs_mounted) {
		return true;
	}

	/* Check if internal LFS was automounted */
	if (is_mounted(mountPoint())) {
		lfs_mounted = true;
		LOG_INF("Internal LittleFS at %s (automounted)", mountPoint());
	} else {
		LOG_ERR("Internal LittleFS NOT mounted at %s - check DTS fstab!", mountPoint());
		return false;
	}

	/* Check if external QSPI was automounted */
	if (is_mounted(extMountPoint())) {
		ext_lfs_mounted = true;
		LOG_INF("External QSPI LittleFS at %s (automounted, 100 blobs)", extMountPoint());
	} else {
#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
		/* Boot-time automount can miss the QSPI on the first boot after a
		 * factory-erase (blank flash) or an early-boot timing race with QSPI
		 * init.  Retry the mount explicitly (fs_mount auto-formats blank flash,
		 * and mounts valid data without touching it) so contacts/channels land
		 * on /ext.  Without this the store silently falls back to internal /lfs,
		 * and the next boot that does mount /ext runs a needless contact
		 * migration — the "Migrating contacts to external storage" churn. */
		FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
		int rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
		if (is_mounted(extMountPoint())) {
			ext_lfs_mounted = true;
			LOG_INF("External QSPI LittleFS at %s (mounted on retry, rc=%d)",
				extMountPoint(), rc);
		} else {
			ext_lfs_mounted = false;
			LOG_WRN("External QSPI mount retry failed (rc=%d) - using internal only (20 blobs)", rc);
		}
#else
		ext_lfs_mounted = false;
		LOG_INF("External QSPI NOT mounted at %s - using internal only (20 blobs)", extMountPoint());
#endif
	}

	return true;
}

void ZephyrDataStore::unmount()
{
	/* With automount, filesystems are managed by Zephyr - just clear our flags */
	lfs_mounted = false;
	ext_lfs_mounted = false;
}

ZephyrDataStore::ZephyrDataStore(mesh::RTCClock &clock)
	: _clock(&clock), _has_ext_fs(false)
{
}

void ZephyrDataStore::begin()
{
	_has_ext_fs = ext_lfs_mounted;
	LOG_INF("_has_ext_fs=%d (ext_lfs_mounted=%d)", _has_ext_fs ? 1 : 0, ext_lfs_mounted ? 1 : 0);
	LOG_INF("contacts path=%s, channels path=%s", contactsFile(), channelsFile());

	if (_has_ext_fs) {
		migrateToExternalFS();
	}

	checkAdvBlobFile();
}

bool ZephyrDataStore::exists(const char *path) const
{
	struct fs_dirent ent;
	return fs_stat(path, &ent) == 0;
}

bool ZephyrDataStore::removeFile(const char *path)
{
	return fs_unlink(path) == 0;
}

bool ZephyrDataStore::openRead(const char *path, uint8_t *buf, size_t buf_sz, size_t &out_len) const
{
	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_READ);
	if (rc < 0) {
		return false;
	}
	ssize_t n = fs_read(&file, buf, buf_sz);
	fs_close(&file);
	if (n < 0) {
		return false;
	}
	out_len = (size_t)n;
	return true;
}

struct AtomicReplaceCtx {
	const uint8_t *buf;
	size_t len;
};

static bool atomicReplaceWriter(struct fs_file_t *file, void *ctx)
{
	AtomicReplaceCtx *c = static_cast<AtomicReplaceCtx *>(ctx);
	ssize_t n = fs_write(file, c->buf, c->len);
	return !(n < 0 || (size_t)n != c->len);
}

/* Power-safe replace helper used for identity + prefs. */
bool ZephyrDataStore::atomicReplaceFile(const char *path, const uint8_t *buf, size_t len)
{
	AtomicReplaceCtx ctx = {
		.buf = buf,
		.len = len,
	};
	return atomicWriteTempFile(path, atomicReplaceWriter, &ctx, "atomicReplaceFile");
}

bool ZephyrDataStore::copyFile(const char *src, const char *dst)
{
	char tmp_path[64];
	int pl = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", dst);
	if (pl <= 0 || pl >= (int)sizeof(tmp_path)) {
		return false;
	}

	fs_unlink(tmp_path);

	struct fs_file_t src_file, dst_file;
	fs_file_t_init(&src_file);
	fs_file_t_init(&dst_file);

	if (fs_open(&src_file, src, FS_O_READ) < 0) {
		return false;
	}

	if (fs_open(&dst_file, tmp_path, FS_O_CREATE | FS_O_WRITE) < 0) {
		fs_close(&src_file);
		return false;
	}

	uint8_t buf[64];
	ssize_t n;
	bool ok = true;
	while ((n = fs_read(&src_file, buf, sizeof(buf))) > 0) {
		if (fs_write(&dst_file, buf, n) != n) {
			ok = false;
			break;
		}
	}

	fs_close(&src_file);
	if (!ok || n < 0) {
		fs_close(&dst_file);
		fs_unlink(tmp_path);
		return false;
	}

	int rc = fs_sync(&dst_file);
	fs_close(&dst_file);
	if (rc < 0) {
		fs_unlink(tmp_path);
		return false;
	}

	if (fs_rename(tmp_path, dst) < 0) {
		fs_unlink(tmp_path);
		return false;
	}
	return true;
}

void ZephyrDataStore::migrateToExternalFS()
{
	/* Migrate contacts from internal to external if not present */
	if (!exists(EXT_CONTACTS_FILE) && exists(INT_CONTACTS_FILE)) {
		LOG_INF("Migrating contacts to external storage");
		if (copyFile(INT_CONTACTS_FILE, EXT_CONTACTS_FILE)) {
			removeFile(INT_CONTACTS_FILE);
		}
	}

	/* Migrate channels */
	if (!exists(EXT_CHANNELS_FILE) && exists(INT_CHANNELS_FILE)) {
		LOG_INF("Migrating channels to QSPI");
		if (copyFile(INT_CHANNELS_FILE, EXT_CHANNELS_FILE)) {
			removeFile(INT_CHANNELS_FILE);
		}
	}

	/* Migrate adv_blobs (extend to 100 records) */
	if (!exists(EXT_ADV_BLOBS_FILE) && exists(INT_ADV_BLOBS_FILE)) {
		LOG_INF("Migrating adv_blobs to QSPI (20 -> 100 slots)");
		if (copyFile(INT_ADV_BLOBS_FILE, EXT_ADV_BLOBS_FILE)) {
			removeFile(INT_ADV_BLOBS_FILE);
			struct fs_file_t file;
			fs_file_t_init(&file);
			if (fs_open(&file, EXT_ADV_BLOBS_FILE, FS_O_RDWR) == 0) {
				fs_seek(&file, 0, FS_SEEK_END);
				BlobRec zeroes;
				memset(&zeroes, 0, sizeof(zeroes));
				for (int i = 20; i < 100; i++) {
					fs_write(&file, &zeroes, sizeof(zeroes));
				}
				fs_close(&file);
			}
		}
	}

	/* Clean up old files on internal if they exist on external */
	if (exists(EXT_CONTACTS_FILE) && exists(INT_CONTACTS_FILE)) {
		removeFile(INT_CONTACTS_FILE);
	}
	if (exists(EXT_CHANNELS_FILE) && exists(INT_CHANNELS_FILE)) {
		removeFile(INT_CHANNELS_FILE);
	}
	if (exists(EXT_ADV_BLOBS_FILE) && exists(INT_ADV_BLOBS_FILE)) {
		removeFile(INT_ADV_BLOBS_FILE);
	}
}

void ZephyrDataStore::checkAdvBlobFile()
{
	const char *path = advBlobsFile();
	if (exists(path)) {
		return;
	}
	BlobRec zeroes;
	memset(&zeroes, 0, sizeof(zeroes));
	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("Failed to create adv_blobs file: %d", rc);
		return;
	}
	int recs = maxBlobRecs();
	for (int i = 0; i < recs; i++) {
		fs_write(&file, &zeroes, sizeof(zeroes));
	}
	fs_close(&file);
}

/* ── Format / Factory Reset ────────────────────────────────────────── */

bool ZephyrDataStore::formatFileSystem()
{
	LOG_INF("formatFileSystem: starting...");

	/* Properly unmount from Zephyr's VFS before erasing flash.
	 * The old unmount() only cleared flags — Zephyr still held /lfs mounted,
	 * so flash_area_flatten silently destroyed the on-flash superblock while
	 * LittleFS considered itself active.  Every subsequent file op then hit
	 * the erased blocks and logged "Corrupted dir pair at {0x0, 0x1}".
	 * FS_FSTAB_DECLARE_ENTRY exposes the non-static mount struct generated
	 * from the DTS fstab; fs_mount() on a blank partition auto-formats
	 * (littlefs_fs.c: lfs_mount fail → lfs_format → lfs_mount). */
	FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(lfs));
	fs_unmount(&FS_FSTAB_ENTRY(DT_NODELABEL(lfs)));
	lfs_mounted = false;

#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
	FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
	fs_unmount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
#endif
	ext_lfs_mounted = false;

	const struct flash_area *fap;
	int rc;

#if FIXED_PARTITION_EXISTS(lfs_partition)
	rc = flash_area_open(PARTITION_ID(lfs_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting LFS partition (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

#if FIXED_PARTITION_EXISTS(storage_partition)
	rc = flash_area_open(PARTITION_ID(storage_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting NVS storage (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

#if FIXED_PARTITION_EXISTS(qspi_storage_partition)
	/* QSPI if present (any platform) */
	rc = flash_area_open(PARTITION_ID(qspi_storage_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting QSPI (%u bytes, may take a while)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

	/* Remount: littlefs_mount() auto-formats on blank flash, then mounts. */
	rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(lfs)));
	bool mounted = (rc == 0);
	if (mounted) {
		lfs_mounted = true;
	}

#if DT_NODE_EXISTS(DT_NODELABEL(qspi_lfs))
	/* Remount external QSPI too.  We unmounted it above and flattened its
	 * partition, so it must be re-mounted here — otherwise a runtime format
	 * (factory reset, or the first-boot "no prefs" auto-format) leaves /ext
	 * unmounted for the rest of the session.  begin() then reads
	 * ext_lfs_mounted=false and the store falls back to internal /lfs, so
	 * contacts/channels save to /lfs and get needlessly migrated back to /ext
	 * on the next boot ("Migrating contacts to external storage" churn). */
	{
		FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
		int ext_rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
		if (is_mounted(extMountPoint())) {
			ext_lfs_mounted = true;
			LOG_INF("formatFileSystem: /ext remounted (rc=%d)", ext_rc);
		} else {
			ext_lfs_mounted = false;
			LOG_ERR("formatFileSystem: /ext remount failed (rc=%d)", ext_rc);
		}
	}
#endif

	LOG_INF("formatFileSystem: mount() returned %d", mounted ? 1 : 0);
	return mounted;
}

void ZephyrDataStore::factoryReset()
{
	LOG_INF("=== FACTORY RESET STARTING ===");
	if (formatFileSystem()) {
		/* Mark the freshly-formatted FS as ZephCore-initialised so the
		 * post-reboot first-boot check (no prefs → format) doesn't format it a
		 * SECOND time. That redundant format re-ran formatFileSystem() without
		 * a following mount(), which is what used to leave /ext unmounted and
		 * push contacts/channels onto internal flash. */
		writeInitMarker();
		LOG_INF("=== FACTORY RESET COMPLETE - REBOOT REQUIRED ===");
	} else {
		LOG_ERR("=== FACTORY RESET FAILED ===");
	}
}

/* ── First-boot migration ──────────────────────────────────────────── */

/* Marker written after the first clean ZephCore boot to prevent
 * repeated auto-format on subsequent boots. */
static constexpr const char *ZC_INIT_MARKER = "/lfs/_zc_init";

bool ZephyrDataStore::hasInitMarker() const
{
	return exists(ZC_INIT_MARKER);
}

void ZephyrDataStore::writeInitMarker()
{
	struct fs_file_t f;
	fs_file_t_init(&f);
	if (fs_open(&f, ZC_INIT_MARKER, FS_O_CREATE | FS_O_WRITE) == 0) {
		fs_close(&f);
	}
}

bool ZephyrDataStore::hasPrefs() const
{
	return exists(PREFS_FILE);
}

/* Erase only the NVS (BLE bonds) partition — used when upgrading from
 * firmware that had the storage_partition region as app code.  That leaves
 * bytes at 0xD0000 that can accidentally pass Zephyr NVS sector validation,
 * causing settings_load() to hang and blocking bt_enable(). */
void ZephyrDataStore::formatNVSOnly()
{
#if FIXED_PARTITION_EXISTS(storage_partition)
	const struct flash_area *fap;
	int rc = flash_area_open(PARTITION_ID(storage_partition), &fap);
	if (rc == 0) {
		LOG_INF("formatNVSOnly: erasing NVS storage (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	} else {
		LOG_WRN("formatNVSOnly: flash_area_open(storage_partition) failed: %d", rc);
	}
#else
	LOG_DBG("formatNVSOnly: no storage_partition on this platform, skipped");
#endif
}

/* Returns true if the prefs file was written by Arduino MeshCore.
 * Arduino's layout omits node_lat/node_lon (16 bytes inserted by ZephCore
 * after node_name at offset 36), so freq/sf/bw land at the wrong offsets
 * and produce values outside the physical RF ranges used as the signal. */
bool ZephyrDataStore::prefsLookLikeArduino() const
{
	uint8_t buf[72];
	size_t len = 0;
	if (!openRead(PREFS_FILE, buf, sizeof(buf), len) || len < 68) {
		return false;
	}
	float freq, bw;
	uint8_t sf;
	memcpy(&freq, &buf[56], sizeof(float));
	sf = buf[60];
	memcpy(&bw, &buf[64], sizeof(float));
	return (freq < 300.0f || freq > 960.0f ||
	        sf < 5 || sf > 12 ||
	        bw < 6.0f || bw > 510.0f);
}

/* Returns true if the old file-based BLE bonds file exists.
 * Pre-NVS ZephCore (≤1.16.1) stored bonds via CONFIG_SETTINGS_FILE at this
 * path; ≥1.16.2 moved to NVS.  Presence means 0xD0000 has old app code. */
bool ZephyrDataStore::hasOldSettingsFile() const
{
	return exists("/lfs/settings");
}

/* ── Identity ──────────────────────────────────────────────────────── */

bool ZephyrDataStore::loadMainIdentity(mesh::LocalIdentity &identity)
{
	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t len = 0;
	if (!openRead(MAIN_ID_FILE, buf, sizeof(buf), len) || len < PRV_KEY_SIZE + PUB_KEY_SIZE) {
		return false;
	}
	return identity.readFrom(buf, len);
}

bool ZephyrDataStore::saveMainIdentity(const mesh::LocalIdentity &identity)
{
	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t n = identity.writeTo(buf, sizeof(buf));
	if (n == 0) {
		return false;
	}
	return atomicReplaceFile(MAIN_ID_FILE, buf, n);
}

/* ── Shutdown-reason breadcrumb ────────────────────────────────────── */

void ZephyrDataStore::saveShutdownReason(uint8_t code)
{
	/* Best-effort: called at a software power-off, possibly at low battery. */
	(void)atomicReplaceFile(SHUTDOWN_FILE, &code, 1);
}

uint8_t ZephyrDataStore::takeShutdownReason()
{
	uint8_t code = 0;
	size_t len = 0;

	if (openRead(SHUTDOWN_FILE, &code, sizeof(code), len) && len >= 1) {
		removeFile(SHUTDOWN_FILE);
		return code;
	}
	/* Stray/empty file — clear it so it can't linger. */
	removeFile(SHUTDOWN_FILE);
	return 0;
}

/* ── Preferences ───────────────────────────────────────────────────── */

void ZephyrDataStore::loadPrefs(NodePrefs &prefs)
{
	/* Save caller's defaults — restored if the file contains invalid radio
	 * params (e.g. an Arduino MeshCore new_prefs whose layout diverges from
	 * ZephCore at the freq/sf/bw offsets due to the inserted lat/lon fields). */
	NodePrefs saved_defaults = prefs;

	bool prefs_exists = exists(PREFS_FILE);
	if (!prefs_exists) {
		LOG_DBG("loadPrefs: no prefs file found, persisting defaults");
		/* Persist defaults so flash always has a prefs file from boot 1.
		 * Lets later code (e.g. tempradio revert) trust that flash is
		 * authoritative without a "first run" special case. */
		savePrefs(prefs);
		return;
	}

	uint8_t buf[256];
	size_t len = 0;
	if (!openRead(PREFS_FILE, buf, sizeof(buf), len)) {
		LOG_ERR("loadPrefs: read failed");
		return;
	}
	if (len < 90) {
		/* gps_interval occupies bytes 86-89, so a shorter blob would read
		 * its top bytes from uninitialized stack — reject rather than load a
		 * garbage interval. */
		LOG_ERR("loadPrefs: file too small (%d bytes, need 90)", (int)len);
		return;
	}
	LOG_DBG("loadPrefs: loaded %d bytes from %s", (int)len, PREFS_FILE);

	size_t off = 0;
	memcpy(&prefs.airtime_factor, &buf[off], sizeof(float));
	off += 4;
	memcpy(prefs.node_name, &buf[off], 32);
	off += 36;  /* 32 name + 4 pad */
	memcpy(&prefs.node_lat, &buf[off], sizeof(double));
	off += 8;
	memcpy(&prefs.node_lon, &buf[off], sizeof(double));
	off += 8;
	memcpy(&prefs.freq, &buf[off], sizeof(float));
	off += 4;
	prefs.sf = buf[off++];
	prefs.cr = buf[off++];
	/* Offset 62: client_repeat (Arduino-compatible placement) */
	prefs.client_repeat = buf[off++];
	prefs.manual_add_contacts = buf[off++];
	memcpy(&prefs.bw, &buf[off], sizeof(float));
	off += 4;

	/* Sanity-check core radio params before consuming the rest of the file.
	 * An Arduino MeshCore new_prefs is layout-incompatible: ZephCore inserts
	 * node_lat (8) + node_lon (8) after node_name, shifting freq/sf/bw by
	 * +16 bytes.  The misread values are freq≈0, sf≤1, bw=garbage — all
	 * outside the physical RF ranges below.  Revert to the caller's defaults
	 * so the radio starts on the correct channel and the user can pair via
	 * BLE and reconfigure. */
	if (prefs.freq < 300.0f || prefs.freq > 960.0f ||
	    prefs.sf < 5 || prefs.sf > 12 ||
	    prefs.bw < 6.0f || prefs.bw > 510.0f) {
		LOG_WRN("loadPrefs: radio params out of range "
			"(freq=%.1f sf=%d bw=%.1f) — ignoring prefs (incompatible format?)",
			(double)prefs.freq, (int)prefs.sf, (double)prefs.bw);
		prefs = saved_defaults;
		return;
	}

	prefs.tx_power_dbm = buf[off++];
	prefs.telemetry_mode_base = buf[off++];
	prefs.telemetry_mode_loc = buf[off++];
	prefs.telemetry_mode_env = buf[off++];
	memcpy(&prefs.rx_delay_base, &buf[off], sizeof(float));
	off += 4;
	prefs.advert_loc_policy = buf[off++];
	prefs.multi_acks = buf[off++];
	/* Offset 78: path_hash_mode (Arduino treats as pad — harmless) */
	prefs.path_hash_mode = buf[off++];
	off += 1;  /* pad */
	memcpy(&prefs.ble_pin, &buf[off], sizeof(uint32_t));
	off += 4;
	prefs.buzzer_quiet = buf[off++];
	prefs.gps_enabled = buf[off++];
	memcpy(&prefs.gps_interval, &buf[off], sizeof(uint32_t));
	off += 4;
	prefs.autoadd_config = buf[off++];

	/* Offset 91: autoadd_max_hops (matches Arduino layout) */
	if (off < len) {
		prefs.autoadd_max_hops = buf[off++];
	}

	/* Offset 92: rx_boost (ZephCore extension — Arduino stops at 92 bytes) */
	if (off < len) {
		prefs.rx_boost = buf[off++];
	} else {
		prefs.rx_boost = 1;  /* Default to boosted for better sensitivity */
	}

	/* Offset 93: leds_disabled (ZephCore extension) */
	if (off < len) {
		prefs.leds_disabled = buf[off++];
	} else {
		prefs.leds_disabled = 0;  /* Default: LEDs on */
	}

	/* Offsets 94-95: RESERVED — formerly apc_enabled / apc_margin (APC,
	 * removed in 1.16.6). Still consumed so offset 96 onward keeps landing
	 * where already-deployed nodes wrote it. Values are ignored. */
	if (off < len) {
		prefs._reserved_apc_enabled = buf[off++];
	}
	if (off < len) {
		prefs._reserved_apc_margin = buf[off++];
	}

	/* Offset 96: default_scope_name (31 bytes) — v11 FIRMWARE_VER_CODE */
	if (off + 31 <= len) {
		memcpy(prefs.default_scope_name, &buf[off], 31);
		off += 31;
	} else {
		memset(prefs.default_scope_name, 0, sizeof(prefs.default_scope_name));
	}

	/* Offset 127: default_scope_key (16 bytes) */
	if (off + 16 <= len) {
		memcpy(prefs.default_scope_key, &buf[off], 16);
		off += 16;
	} else {
		memset(prefs.default_scope_key, 0, sizeof(prefs.default_scope_key));
	}

	/* Offset 143: ble_disabled (ZephCore extension) */
	if (off < len) {
		prefs.ble_disabled = buf[off++];
	} else {
		prefs.ble_disabled = 0;
	}

	/* Offset 144: display_brightness (ZephCore extension, 0 = default 100%) */
	if (off < len) {
		prefs.display_brightness = buf[off++];
	} else {
		prefs.display_brightness = 0;
	}

	/* Offset 145: wake_on_msg (ZephCore extension, 0 = don't wake, 1 = wake on message) */
	if (off < len) {
		prefs.wake_on_msg = buf[off++];
	} else {
		prefs.wake_on_msg = 1;
	}

	/* Offset 146: screen_off_secs (ZephCore extension, 2 bytes LE, 0 = Kconfig default) */
	if (off + 2 <= len) {
		prefs.screen_off_secs = (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
		off += 2;
	} else {
		prefs.screen_off_secs = 0;
	}

	/* Offset 148: auto_shutdown_mv (ZephCore extension, 2 bytes LE).
	 * Absent in pre-existing files → fall back to the Kconfig default so
	 * upgrades inherit the board's built-in threshold. */
	if (off + 2 <= len) {
		prefs.auto_shutdown_mv = (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
		off += 2;
	} else {
		prefs.auto_shutdown_mv = CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS;
	}

	/* Offset 150: rx_duty_cycle (ZephCore extension).  Absent in pre-existing
	 * files → leave the caller's in-RAM default (0 = continuous RX) untouched
	 * so the no-op past-EOF read can't force it on. */
	if (off < len) {
		prefs.rx_duty_cycle = buf[off++];
		if (prefs.rx_duty_cycle > 1) {
			prefs.rx_duty_cycle = 0;
		}
	}

	/* Offset 151: meshtimesync (ZephCore extension, default 0 = off) */
	if (off < len) {
		prefs.meshtimesync = buf[off++];
		if (prefs.meshtimesync > 1) {
			prefs.meshtimesync = 0;
		}
	}

	/* Offset 152: v_contact_enabled (ZephCore extension, default 1 = on) */
	if (off < len) {
		prefs.v_contact_enabled = buf[off++];
		if (prefs.v_contact_enabled > 1) {
			prefs.v_contact_enabled = 1;
		}
	} else {
		prefs.v_contact_enabled = 1;
	}

	/* Offset 153: v_battery_alert_mv (ZephCore extension, 2 bytes LE).
	 * 0xFFFF sentinel = derive from board auto-shutdown threshold; 0 = off. */
	if (off + 2 <= len) {
		prefs.v_battery_alert_mv = (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
		off += 2;
	} else {
		prefs.v_battery_alert_mv = 0xFFFF;
	}

	/* Offset 155: cad_auto (ZephCore extension, default 0 = dry-run) */
	if (off < len) {
		prefs.cad_auto = buf[off++];
		if (prefs.cad_auto > 1) {
			prefs.cad_auto = 0;
		}
	}

	/* Offset 156: cad_offset (ZephCore extension, signed, default 0) */
	if (off < len) {
		prefs.cad_offset = (int8_t)buf[off++];
		if (prefs.cad_offset < CAD_OFFSET_MIN || prefs.cad_offset > CAD_OFFSET_MAX) {
			prefs.cad_offset = 0;
		}
	}

	/* Offset 157: probe_interval (ZephCore extension, seconds; 0 = off).
	 * Absent in pre-existing files → keep the in-RAM default (60). */
	if (off < len) {
		prefs.probe_interval = buf[off++];
		if (prefs.probe_interval != 0 && prefs.probe_interval < 10) {
			prefs.probe_interval = 10;
		}
	}

	/* Offset 158: cad_busycap (ZephCore extension, percent; 0 = off).
	 * Absent in pre-existing files → keep the in-RAM default (25). */
	if (off < len) {
		prefs.cad_busycap = buf[off++];
		if (prefs.cad_busycap > 90) {
			prefs.cad_busycap = 90;
		}
	}

	/* Offset 159: adc_multiplier (ZephCore extension, float LE, 0 = board
	 * DT default).  Absent in pre-existing files → keep 0.0 so the DT
	 * default stays in effect.  Same range guard as the repeater CLI path
	 * (CommonCLI constrains 0..30000); NaN/garbage resets to default. */
	if (off + 4 <= len) {
		memcpy(&prefs.adc_multiplier, &buf[off], sizeof(float));
		off += 4;
		if (prefs.adc_multiplier != prefs.adc_multiplier ||
		    prefs.adc_multiplier < 0.0f || prefs.adc_multiplier > 30000.0f) {
			prefs.adc_multiplier = 0.0f;
		}
	}
}

void ZephyrDataStore::savePrefs(const NodePrefs &prefs)
{
	uint8_t buf[256];
	uint8_t pad[8] = {0};
	size_t off = 0;
	memcpy(&buf[off], &prefs.airtime_factor, sizeof(float));
	off += 4;
	memcpy(&buf[off], prefs.node_name, 32);
	off += 32;
	memcpy(&buf[off], pad, 4);
	off += 4;
	memcpy(&buf[off], &prefs.node_lat, sizeof(double));
	off += 8;
	memcpy(&buf[off], &prefs.node_lon, sizeof(double));
	off += 8;
	memcpy(&buf[off], &prefs.freq, sizeof(float));
	off += 4;
	buf[off++] = prefs.sf;
	buf[off++] = prefs.cr;
	/* Offset 62: client_repeat (Arduino-compatible placement) */
	buf[off++] = prefs.client_repeat;
	buf[off++] = prefs.manual_add_contacts;
	memcpy(&buf[off], &prefs.bw, sizeof(float));
	off += 4;
	buf[off++] = prefs.tx_power_dbm;
	buf[off++] = prefs.telemetry_mode_base;
	buf[off++] = prefs.telemetry_mode_loc;
	buf[off++] = prefs.telemetry_mode_env;
	memcpy(&buf[off], &prefs.rx_delay_base, sizeof(float));
	off += 4;
	buf[off++] = prefs.advert_loc_policy;
	buf[off++] = prefs.multi_acks;
	/* Offset 78: path_hash_mode (Arduino treats as pad — harmless) */
	buf[off++] = prefs.path_hash_mode;
	buf[off++] = 0;  /* pad */
	memcpy(&buf[off], &prefs.ble_pin, sizeof(uint32_t));
	off += 4;
	buf[off++] = prefs.buzzer_quiet;
	buf[off++] = prefs.gps_enabled;
	memcpy(&buf[off], &prefs.gps_interval, sizeof(uint32_t));
	off += 4;
	buf[off++] = prefs.autoadd_config;
	/* Offset 91: autoadd_max_hops (matches Arduino layout) */
	buf[off++] = prefs.autoadd_max_hops;
	/* Offset 92: rx_boost (ZephCore extension — Arduino ignores) */
	buf[off++] = prefs.rx_boost;
	/* Offset 93: leds_disabled (ZephCore extension) */
	buf[off++] = prefs.leds_disabled;
	/* Offsets 94-95: RESERVED — formerly apc_enabled / apc_margin (removed
	 * in 1.16.6). Written back unchanged to hold the layout. */
	buf[off++] = prefs._reserved_apc_enabled;
	buf[off++] = prefs._reserved_apc_margin;
	/* Offset 96: default_scope_name (31 bytes) — v11 FIRMWARE_VER_CODE */
	memcpy(&buf[off], prefs.default_scope_name, 31);
	off += 31;
	/* Offset 127: default_scope_key (16 bytes) */
	memcpy(&buf[off], prefs.default_scope_key, 16);
	off += 16;
	/* Offset 143: ble_disabled (ZephCore extension) */
	buf[off++] = prefs.ble_disabled;
	/* Offset 144: display_brightness (ZephCore extension) */
	buf[off++] = prefs.display_brightness;
	/* Offset 145: wake_on_msg (ZephCore extension) */
	buf[off++] = prefs.wake_on_msg;
	/* Offset 146: screen_off_secs (ZephCore extension, 2 bytes LE) */
	buf[off++] = prefs.screen_off_secs & 0xFF;
	buf[off++] = (prefs.screen_off_secs >> 8) & 0xFF;
	/* Offset 148: auto_shutdown_mv (ZephCore extension, 2 bytes LE) */
	buf[off++] = prefs.auto_shutdown_mv & 0xFF;
	buf[off++] = (prefs.auto_shutdown_mv >> 8) & 0xFF;
	/* Offset 150: rx_duty_cycle (ZephCore extension) */
	buf[off++] = prefs.rx_duty_cycle;
	/* Offset 151: meshtimesync (ZephCore extension) */
	buf[off++] = prefs.meshtimesync;
	/* Offset 152: v_contact_enabled (ZephCore extension) */
	buf[off++] = prefs.v_contact_enabled;
	/* Offset 153: v_battery_alert_mv (ZephCore extension, 2 bytes LE) */
	buf[off++] = prefs.v_battery_alert_mv & 0xFF;
	buf[off++] = (prefs.v_battery_alert_mv >> 8) & 0xFF;
	/* Offset 155: cad_auto (ZephCore extension) */
	buf[off++] = prefs.cad_auto;
	/* Offset 156: cad_offset (ZephCore extension, signed) */
	buf[off++] = (uint8_t)prefs.cad_offset;
	/* Offset 157: probe_interval (ZephCore extension, seconds) */
	buf[off++] = prefs.probe_interval;
	/* Offset 158: cad_busycap (ZephCore extension, percent) */
	buf[off++] = prefs.cad_busycap;
	/* Offset 159: adc_multiplier (ZephCore extension, float LE, 0 = board
	 * DT default).  Was applied at boot but never serialized before this
	 * field existed — battery calibration silently reset every reboot. */
	memcpy(&buf[off], &prefs.adc_multiplier, sizeof(float));
	off += 4;
	/* Total: 163 bytes */

	bool ok = atomicReplaceFile(PREFS_FILE, buf, off);
	LOG_DBG("savePrefs: wrote %s, ok=%d (%d bytes), name='%.16s'",
		PREFS_FILE, ok ? 1 : 0, (int)off, prefs.node_name);
}

/* ── Contacts: contacts3 (152B records, Arduino-compatible) ────────── */

static constexpr size_t CONTACT_DATA_SZ = 152;  /* 32+32+1+1+1+4+1+4+64+4+4+4 */

/* Pack a ContactInfo into the 152-byte wire format (Arduino contacts3) */
static void contact_to_record(const ContactInfo &c, uint8_t rec[CONTACT_DATA_SZ])
{
	uint8_t *p = rec;
	uint8_t unused = 0;
	memcpy(p, c.id.pub_key, 32);  p += 32;
	memcpy(p, c.name, 32);        p += 32;
	*p++ = c.type;
	*p++ = c.flags;
	*p++ = unused;
	memcpy(p, &c.sync_since, 4);             p += 4;
	*p++ = c.out_path_len;
	memcpy(p, &c.last_advert_timestamp, 4);  p += 4;
	memcpy(p, c.out_path, 64);               p += 64;
	memcpy(p, &c.lastmod, 4);                p += 4;
	memcpy(p, &c.gps_lat, 4);                p += 4;
	memcpy(p, &c.gps_lon, 4);                p += 4;
}

/* Unpack 152-byte wire format into a ContactInfo */
static void record_to_contact(const uint8_t rec[CONTACT_DATA_SZ], ContactInfo &c)
{
	const uint8_t *p = rec;
	uint8_t pub_key[32];
	uint8_t unused;
	memcpy(pub_key, p, 32);    p += 32;
	memcpy(c.name, p, 32);    p += 32;
	c.type = *p++;
	c.flags = *p++;
	unused = *p++;  (void)unused;
	memcpy(&c.sync_since, p, 4);             p += 4;
	c.out_path_len = *p++;
	memcpy(&c.last_advert_timestamp, p, 4);  p += 4;
	memcpy(c.out_path, p, 64);               p += 64;
	memcpy(&c.lastmod, p, 4);                p += 4;
	memcpy(&c.gps_lat, p, 4);                p += 4;
	memcpy(&c.gps_lon, p, 4);                p += 4;
	c.id = mesh::Identity(pub_key);
	c.shared_secret_valid = false;
}

void ZephyrDataStore::loadContacts(DataStoreHost *host)
{
	const char *path = contactsFile();

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_READ);
	if (rc < 0) {
		LOG_DBG("loadContacts: no contacts file found");
		return;
	}

	uint32_t count = 0;
	uint8_t rec[CONTACT_DATA_SZ];

	for (;;) {
		ssize_t n = fs_read(&file, rec, CONTACT_DATA_SZ);
		if (n <= 0) break;
		if (n != (ssize_t)CONTACT_DATA_SZ) {
			LOG_WRN("loadContacts: truncated record at #%u (%d bytes)",
				count, (int)n);
			break;
		}

		ContactInfo c;
		record_to_contact(rec, c);
		if (!host->onContactLoaded(c)) break;
		count++;
	}

	fs_close(&file);
	LOG_INF("loadContacts: loaded %u contacts from %s", count, path);
}

void ZephyrDataStore::saveContacts(DataStoreHost *host)
{
	const char *path = contactsFile();
	bool use_atomic = _has_ext_fs;
	const char *save_mode = use_atomic ? "atomic" : "direct";

	if (!use_atomic && exists(path)) {
		fs_unlink(path);
	}

	struct fs_file_t file;
	uint8_t rec[CONTACT_DATA_SZ];
	uint32_t idx = 0;       // contacts iterated (incl. skipped anon)
	uint32_t written = 0;   // records actually written to the file
	ContactInfo c;
	bool write_ok = true;

	auto write_contacts = [&](struct fs_file_t *dst) -> bool {
		while (host->getContactForSave(idx, c)) {
			// Don't persist transient/anon contacts (non-contact requests)
			if (c.type == ADV_TYPE_NONE) {
				idx++;
				continue;
			}
			contact_to_record(c, rec);
			if (fs_write(dst, rec, CONTACT_DATA_SZ) != (ssize_t)CONTACT_DATA_SZ) {
				LOG_ERR("saveContacts: write failed at record %u", idx);
				return false;
			}
			idx++;
			written++;
		}
		return true;
	};

	if (use_atomic) {
		struct ContactsWriterCtx {
			decltype(write_contacts) *fn;
		} ctx = { &write_contacts };
		auto atomic_contacts_writer = [](struct fs_file_t *dst, void *arg) -> bool {
			ContactsWriterCtx *cctx = static_cast<ContactsWriterCtx *>(arg);
			return (*cctx->fn)(dst);
		};
		write_ok = atomicWriteTempFile(path, atomic_contacts_writer, &ctx, "saveContacts");
	} else {
		fs_file_t_init(&file);
		int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
		if (rc < 0) {
			LOG_ERR("saveContacts: fs_open(%s) failed: %d", path, rc);
			return;
		}
		write_ok = write_contacts(&file);
		int sync_rc = fs_sync(&file);
		fs_close(&file);
		if (!write_ok || sync_rc < 0) {
			if (sync_rc < 0) {
				LOG_ERR("saveContacts: sync failed: %d", sync_rc);
			}
			return;
		}
	}

	if (!write_ok) {
		return;
	}
	LOG_INF("saveContacts: mode=%s saved %u contacts to %s (%u bytes)",
		save_mode, written, path, written * CONTACT_DATA_SZ);
}

/* ── Channels ──────────────────────────────────────────────────────── */

void ZephyrDataStore::loadChannels(DataStoreHost *host)
{
	const char *path = channelsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		return;
	}
	uint8_t channel_idx = 0;
	for (;;) {
		ChannelDetails ch;
		uint8_t unused[4];
		ssize_t n = fs_read(&file, unused, 4);
		if (n != 4) break;
		n = fs_read(&file, (uint8_t *)ch.name, 32);
		if (n != 32) break;
		n = fs_read(&file, (uint8_t *)ch.channel.secret, 32);
		if (n != 32) break;
		if (host->onChannelLoaded(channel_idx, ch)) {
			channel_idx++;
		} else {
			break;
		}
	}
	fs_close(&file);
}

void ZephyrDataStore::saveChannels(DataStoreHost *host)
{
	const char *path = channelsFile();
	uint8_t channel_idx = 0;
	ChannelDetails ch;
	uint8_t unused[4] = {0};
	struct ChannelsWriterCtx {
		DataStoreHost *host;
		uint8_t *channel_idx;
		ChannelDetails *ch;
		uint8_t *unused;
	};
	ChannelsWriterCtx ctx = {
		.host = host,
		.channel_idx = &channel_idx,
		.ch = &ch,
		.unused = unused,
	};
	auto channels_writer = [](struct fs_file_t *file, void *arg) -> bool {
		ChannelsWriterCtx *c = static_cast<ChannelsWriterCtx *>(arg);
		while (c->host->getChannelForSave(*c->channel_idx, *c->ch)) {
			if (fs_write(file, c->unused, 4) != 4 ||
			    fs_write(file, (uint8_t *)c->ch->name, 32) != 32 ||
			    fs_write(file, (uint8_t *)c->ch->channel.secret, 32) != 32) {
				return false;
			}
			(*c->channel_idx)++;
		}
		return true;
	};
	if (!atomicWriteTempFile(path, channels_writer, &ctx, "saveChannels")) {
		return;
	}
	LOG_INF("saveChannels: saved %u channels to %s", channel_idx, path);
}

/* ── Blobs ─────────────────────────────────────────────────────────── */

uint8_t ZephyrDataStore::getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[])
{
	(void)key_len;
	const char *path = advBlobsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		return 0;
	}
	BlobRec tmp;
	uint8_t len = 0;
	while (fs_read(&file, (uint8_t *)&tmp, sizeof(tmp)) == (ssize_t)sizeof(tmp)) {
		if (memcmp(key, tmp.key, 7) == 0) {
			len = tmp.len;
			memcpy(dest_buf, tmp.data, len);
			break;
		}
	}
	fs_close(&file);
	return len;
}

bool ZephyrDataStore::putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len)
{
	(void)key_len;
	if (len < PUB_KEY_SIZE + 4 + SIGNATURE_SIZE || len > MAX_ADVERT_PKT_LEN) {
		return false;
	}
	checkAdvBlobFile();
	const char *path = advBlobsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_RDWR) < 0) {
		return false;
	}
	uint32_t pos = 0, found_pos = 0;
	uint32_t min_timestamp = 0xFFFFFFFF;
	BlobRec tmp;
	while (fs_read(&file, (uint8_t *)&tmp, sizeof(tmp)) == (ssize_t)sizeof(tmp)) {
		if (memcmp(key, tmp.key, 7) == 0) {
			found_pos = pos;
			break;
		}
		if (tmp.timestamp < min_timestamp) {
			min_timestamp = tmp.timestamp;
			found_pos = pos;
		}
		pos += sizeof(tmp);
	}
	memcpy(tmp.key, key, 7);
	memcpy(tmp.data, src_buf, len);
	tmp.len = len;
	tmp.timestamp = _clock->getCurrentTime();
	fs_seek(&file, found_pos, FS_SEEK_SET);
	fs_write(&file, (uint8_t *)&tmp, sizeof(tmp));
	fs_close(&file);
	return true;
}

bool ZephyrDataStore::deleteBlobByKey(const uint8_t key[], int key_len)
{
	(void)key;
	(void)key_len;
	/* Stub: MeshCore nRF-style — slot reused on next putBlobByKey, no erase. */
	return true;
}

/* ── Storage stats ─────────────────────────────────────────────────── */

uint32_t ZephyrDataStore::getStorageUsedKb() const
{
	/* Match Arduino DataStore: stats follow contacts/channels mount (/ext if present). */
	const char *mp = _has_ext_fs ? EXT_MNT_POINT : MNT_POINT;
	struct fs_statvfs sbuf;
	if (fs_statvfs(mp, &sbuf) != 0) {
		return 0;
	}
	uint32_t total = sbuf.f_blocks * sbuf.f_frsize;
	uint32_t free = sbuf.f_bfree * sbuf.f_frsize;
	return (total - free) / 1024;
}

uint32_t ZephyrDataStore::getStorageTotalKb() const
{
	const char *mp = _has_ext_fs ? EXT_MNT_POINT : MNT_POINT;
	struct fs_statvfs sbuf;
	if (fs_statvfs(mp, &sbuf) != 0) {
		return 0;
	}
	return (sbuf.f_blocks * sbuf.f_frsize) / 1024;
}

uint32_t ZephyrDataStore::getExternalStorageKb() const
{
	if (!_has_ext_fs) {
		return 0;
	}
	struct fs_statvfs sbuf;
	if (fs_statvfs(EXT_MNT_POINT, &sbuf) < 0) {
		return 0;
	}
	return (sbuf.f_blocks * sbuf.f_frsize) / 1024;
}
