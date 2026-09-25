/*
 * SPDX-License-Identifier: MIT
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 *
 * All platforms use DTS-automounted /lfs.
 * QSPI /ext overrides contacts mount when available.
 */

#include "ZephyrDataStore.h"
#include "ZephyrFsFormat.h"
#include "ZephyrFsUtil.h"
#include "IdentityFile.h"
#include "PrefsFile.h"
#include <PrefsCodec.h>
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

/* Track mount status - filesystems are automounted via DTS fstab */
static bool lfs_mounted;
static bool ext_lfs_mounted;

bool ZephyrDataStore::mount()
{
	if (lfs_mounted) {
		return true;
	}

	/* Check if internal LFS was automounted */
	if (zephcore_fs_is_mounted(mountPoint())) {
		lfs_mounted = true;
		LOG_INF("Internal LittleFS at %s (automounted)", mountPoint());
	} else {
		LOG_ERR("Internal LittleFS NOT mounted at %s - check DTS fstab!", mountPoint());
		return false;
	}

	/* Check if external QSPI was automounted */
	if (zephcore_fs_is_mounted(extMountPoint())) {
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
#if DT_NODE_HAS_PROP(DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_qspi_nor), zephyr_deferred_init)
		/* The flash is deferred-init: probing it at boot raced its own
		 * power rail and the JEDEC read came back 00 00 00, so the
		 * driver failed and /ext could never mount. Initialising it here
		 * instead means the part has had until first use to wake up —
		 * over a second — rather than us guessing a settling delay. */
		{
			const struct device *qspi_dev =
				DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_qspi_nor));

			if (!device_is_ready(qspi_dev)) {
				int drc = device_init(qspi_dev);

				LOG_INF("QSPI flash deferred init: rc=%d ready=%d",
					drc, (int)device_is_ready(qspi_dev));
			}
		}
#endif

		FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(qspi_lfs));
		int rc = fs_mount(&FS_FSTAB_ENTRY(DT_NODELABEL(qspi_lfs)));
		if (zephcore_fs_is_mounted(extMountPoint())) {
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

static bool copy_writer(struct fs_file_t *dst, void *ctx)
{
	struct fs_file_t *src = static_cast<struct fs_file_t *>(ctx);
	uint8_t buf[64];
	ssize_t n;

	while ((n = fs_read(src, buf, sizeof(buf))) > 0) {
		if (fs_write(dst, buf, n) != n) {
			return false;
		}
	}
	return n == 0;
}

bool ZephyrDataStore::copyFile(const char *src, const char *dst)
{
	struct fs_file_t src_file;

	fs_file_t_init(&src_file);
	if (fs_open(&src_file, src, FS_O_READ) < 0) {
		return false;
	}
	bool ok = zephcore_fs_atomic_write(dst, copy_writer, &src_file, "copyFile");

	fs_close(&src_file);
	return ok;
}

void ZephyrDataStore::migrateToExternalFS()
{
	/* Migrate contacts from internal to external if not present */
	if (!zephcore_fs_exists(EXT_CONTACTS_FILE) && zephcore_fs_exists(INT_CONTACTS_FILE)) {
		LOG_INF("Migrating contacts to external storage");
		if (copyFile(INT_CONTACTS_FILE, EXT_CONTACTS_FILE)) {
			zephcore_fs_remove(INT_CONTACTS_FILE);
		}
	}

	/* Migrate channels */
	if (!zephcore_fs_exists(EXT_CHANNELS_FILE) && zephcore_fs_exists(INT_CHANNELS_FILE)) {
		LOG_INF("Migrating channels to QSPI");
		if (copyFile(INT_CHANNELS_FILE, EXT_CHANNELS_FILE)) {
			zephcore_fs_remove(INT_CHANNELS_FILE);
		}
	}

	/* Migrate adv_blobs (extend to 100 records) */
	if (!zephcore_fs_exists(EXT_ADV_BLOBS_FILE) && zephcore_fs_exists(INT_ADV_BLOBS_FILE)) {
		LOG_INF("Migrating adv_blobs to QSPI (20 -> 100 slots)");
		if (copyFile(INT_ADV_BLOBS_FILE, EXT_ADV_BLOBS_FILE)) {
			zephcore_fs_remove(INT_ADV_BLOBS_FILE);
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
	if (zephcore_fs_exists(EXT_CONTACTS_FILE) && zephcore_fs_exists(INT_CONTACTS_FILE)) {
		zephcore_fs_remove(INT_CONTACTS_FILE);
	}
	if (zephcore_fs_exists(EXT_CHANNELS_FILE) && zephcore_fs_exists(INT_CHANNELS_FILE)) {
		zephcore_fs_remove(INT_CHANNELS_FILE);
	}
	if (zephcore_fs_exists(EXT_ADV_BLOBS_FILE) && zephcore_fs_exists(INT_ADV_BLOBS_FILE)) {
		zephcore_fs_remove(INT_ADV_BLOBS_FILE);
	}
}

void ZephyrDataStore::checkAdvBlobFile()
{
	const char *path = advBlobsFile();
	if (zephcore_fs_exists(path)) {
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
	/* The erase/remount itself lives in ZephyrFsFormat.c so the repeater,
	 * room server and observer — which build RepeaterDataStore and never
	 * compile this file — get the identical implementation. */
	bool ext_mounted = false;
	bool mounted = zephcore_fs_format_all(&ext_mounted);

	lfs_mounted = mounted;
	ext_lfs_mounted = ext_mounted;

	LOG_INF("formatFileSystem: mount() returned %d", mounted ? 1 : 0);
	return mounted;
}

bool ZephyrDataStore::factoryReset()
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
		return true;
	}
	LOG_ERR("=== FACTORY RESET FAILED ===");
	return false;
}

/* ── First-boot migration ──────────────────────────────────────────── */

/* Marker written after the first clean ZephCore boot to prevent
 * repeated auto-format on subsequent boots. */
static constexpr const char *ZC_INIT_MARKER = "/lfs/_zc_init";

bool ZephyrDataStore::hasInitMarker() const
{
	return zephcore_fs_exists(ZC_INIT_MARKER);
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
	return zephcore_fs_exists(PREFS_JSON_FILE) || zephcore_fs_exists(PREFS_FILE);
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

/* True if the legacy prefs file's radio preset (freq/sf/bw at our offsets) is
 * outside ZC_RADIO_*, i.e. the file is not ours. Upstream's new_prefs has
 * shared our offsets up to 92 since 2025-02 (lat/lon included), so this is a
 * plausibility test, not an Arduino detector. A prefs.json is always ours. */
bool ZephyrDataStore::prefsRadioImplausible() const
{
	if (zephcore_fs_exists(PREFS_JSON_FILE)) {
		return false;
	}
	uint8_t buf[72];
	size_t len = 0;
	if (!zephcore_fs_read_file(PREFS_FILE, buf, sizeof(buf), &len) || len < 68) {
		return false;
	}
	float freq, bw;
	uint8_t sf;
	memcpy(&freq, &buf[56], sizeof(float));
	sf = buf[60];
	memcpy(&bw, &buf[64], sizeof(float));
	return (freq < ZC_RADIO_FREQ_MIN_MHZ || freq > ZC_RADIO_FREQ_MAX_MHZ ||
	        sf < 5 || sf > 12 ||
	        bw < ZC_RADIO_BW_MIN_KHZ || bw > ZC_RADIO_BW_MAX_KHZ);
}

/* Returns true if the old file-based BLE bonds file exists.
 * Pre-NVS ZephCore (≤1.16.1) stored bonds via CONFIG_SETTINGS_FILE at this
 * path; ≥1.16.2 moved to NVS.  Presence means 0xD0000 has old app code. */
bool ZephyrDataStore::hasOldSettingsFile() const
{
	return zephcore_fs_exists("/lfs/settings");
}

void ZephyrDataStore::adoptVolume()
{
	/* /lfs/_zc_init is written after the first clean boot of ZephCore, so
	 * this runs once per volume.
	 *
	 *  - No prefs, or prefs whose radio preset is implausible (a file in
	 *    some other layout): format everything, bonds included.
	 *  - Our prefs plus /lfs/settings: an upgrade from ZephCore <= 1.16.1 on
	 *    nRF52, whose file-based bond store left old app code where the NVS
	 *    bond partition now is. Bytes there can pass NVS sector validation
	 *    and hang settings_load(), so BLE never advertises: erase the NVS
	 *    only; identity, prefs and contacts stay, the phone re-pairs.
	 *    (On nRF54L and MG24 /lfs/settings is the live bond store, but a
	 *    volume that has booted this firmware once has the marker.)
	 *  - Our prefs, no /lfs/settings: bonds already live in NVS, keep all. */
	if (hasInitMarker()) {
		return;
	}
	if (!hasPrefs() || prefsRadioImplausible()) {
		LOG_WRN("First ZephCore boot (%s) - formatting LFS + NVS",
			hasPrefs() ? "foreign prefs" : "no prefs");
		formatFileSystem();
		begin();
	} else if (hasOldSettingsFile()) {
		LOG_WRN("Pre-NVS ZephCore upgrade (found /lfs/settings) - erasing NVS");
		formatNVSOnly();
	} else {
		LOG_INF("ZephCore upgrade with valid NVS - skipping format, bonds preserved");
	}
	writeInitMarker();
}

/* ── Identity ──────────────────────────────────────────────────────── */

bool ZephyrDataStore::loadMainIdentity(mesh::LocalIdentity &identity)
{
	return zephcore_identity_load(MAIN_ID_FILE, identity);
}

bool ZephyrDataStore::saveMainIdentity(const mesh::LocalIdentity &identity)
{
	return zephcore_identity_save(MAIN_ID_FILE, identity);
}

/* ── Preferences ───────────────────────────────────────────────────── */

void ZephyrDataStore::loadPrefs(NodePrefs &prefs)
{
	if (zephcore_prefs_json_load(PREFS_JSON_FILE, prefs, companionPrefsFromJson)) {
		return;
	}
	bool had_json = zephcore_fs_exists(PREFS_JSON_FILE);

	/* No usable prefs.json: the legacy binary file, read once and migrated.
	 * It is kept, so firmware from before prefs.json still boots with the
	 * settings as they were at the upgrade. */
	if (zephcore_fs_exists(PREFS_FILE)) {
		if (loadLegacyPrefs(prefs)) {
			LOG_INF("loadPrefs: %s %s from %s", had_json ? "recovered" : "migrated",
				PREFS_JSON_FILE, PREFS_FILE);
		}
	} else {
		/* First boot: persist the defaults, so later code (e.g. tempradio
		 * revert) can trust flash without a "first run" special case. */
		LOG_DBG("loadPrefs: no prefs file found, persisting defaults");
	}
	savePrefs(prefs);
}

bool ZephyrDataStore::loadLegacyPrefs(NodePrefs &prefs)
{
	uint8_t buf[COMPANION_PREFS_SIZE + 48];
	size_t len = 0;

	if (!zephcore_fs_read_file(PREFS_FILE, buf, sizeof(buf), &len)) {
		LOG_ERR("loadPrefs: read of %s failed", PREFS_FILE);
		return false;
	}
	if (len < 90) {
		LOG_ERR("loadPrefs: %s too small (%d bytes, need 90)", PREFS_FILE, (int)len);
		return false;
	}
	if (!companionPrefsDecode(prefs, buf, len)) {
		float freq, bw;

		memcpy(&freq, &buf[56], sizeof(freq));
		memcpy(&bw, &buf[64], sizeof(bw));
		LOG_WRN("loadPrefs: radio params out of range (freq=%.1f sf=%d bw=%.1f) - "
			"ignoring %s (incompatible format?)",
			(double)freq, (int)buf[60], (double)bw, PREFS_FILE);
		return false;
	}
	/* That firmware's 3300 mV default is 3200 now (NodePrefs.h). */
	prefs.auto_shutdown_set = 0;
	auto_shutdown_upgrade(&prefs);
	return true;
}

void ZephyrDataStore::savePrefs(const NodePrefs &prefs)
{
	bool ok = zephcore_prefs_json_save(PREFS_JSON_FILE, prefs, companionPrefsToJson);

	LOG_DBG("savePrefs: wrote %s, ok=%d, name='%.16s'", PREFS_JSON_FILE, ok ? 1 : 0,
		prefs.node_name);
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

	/* Probe first: this file does not exist until a contact is stored, and
	 * fs_open() on a missing path is logged at ERR by Zephyr's FS layer no
	 * matter how gracefully we handle the return.  The open below is kept as
	 * the real error path (a file that exists but cannot be opened). */
	if (!zephcore_fs_exists(path)) {
		LOG_DBG("loadContacts: no contacts file found");
		return;
	}

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

	/* Atomic replace ONLY where there is external flash — deliberately, and
	 * not to be "fixed" later.
	 *
	 * zephcore_fs_atomic_write() needs room for a second full copy before the
	 * rename.  contacts3 is by far the largest store here (152 B per record,
	 * ~47 KB at 313 contacts) and internal /lfs on these boards is 128 KB
	 * total, shared with identity, prefs and channels2.  Two copies would sit
	 * at ~94 KB of 128 KB before LittleFS metadata, so the atomic path could
	 * fail with ENOSPC exactly when it is most needed — a worse failure than
	 * the one it prevents.
	 *
	 * channels2, identity and prefs are atomic everywhere because they are
	 * small enough for the second copy to be free.  Only contacts is gated.
	 *
	 * The non-atomic branch below is therefore the constrained-board path,
	 * and it writes in place and truncates rather than unlinking first — see
	 * the note there. */
	bool use_atomic = _has_ext_fs;
	const char *save_mode = use_atomic ? "atomic" : "direct";


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
		write_ok = zephcore_fs_atomic_write(path, atomic_contacts_writer, &ctx, "saveContacts");
	} else {
		/* Overwrite in place, then truncate — never unlink first.
		 *
		 * This branch runs on boards with no external flash, i.e. the ones
		 * that cannot afford the atomic temp-file dance.  It used to
		 * fs_unlink() the contacts file before recreating it, which left a
		 * window spanning the whole ~47 KB write where contacts3 did not
		 * exist at all: a power cut there lost every contact rather than
		 * corrupting some.  The unlink was only ever a way to truncate.
		 *
		 * Truncating afterwards is the same guarantee without the window —
		 * the file is always present, and at worst briefly longer than its
		 * new contents (stale records past the end, which the truncate then
		 * removes).  FS_O_TRUNC is NOT usable here: Zephyr's LittleFS
		 * backend maps only CREATE/READ/WRITE/APPEND and drops TRUNC
		 * silently, so asking for it would leave the stale tail in place. */
		fs_file_t_init(&file);
		int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
		if (rc < 0) {
			LOG_ERR("saveContacts: fs_open(%s) failed: %d", path, rc);
			return;
		}
		write_ok = write_contacts(&file);

		int trunc_rc = 0;

		if (write_ok) {
			trunc_rc = fs_truncate(&file,
					       (off_t)written * CONTACT_DATA_SZ);
			if (trunc_rc < 0) {
				LOG_ERR("saveContacts: truncate to %u failed: %d",
					(unsigned)(written * CONTACT_DATA_SZ),
					trunc_rc);
				write_ok = false;
			}
		}

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

	/* Probe first — same reason as loadContacts(): absent until a channel is
	 * configured, and a missing-path fs_open() is logged at ERR by the FS
	 * layer regardless of us handling it. */
	if (!zephcore_fs_exists(path)) {
		return;
	}

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
	if (!zephcore_fs_atomic_write(path, channels_writer, &ctx, "saveChannels")) {
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
