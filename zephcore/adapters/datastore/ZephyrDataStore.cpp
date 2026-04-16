/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 *
 * All platforms use DTS-automounted /lfs.
 * QSPI /ext overrides contacts mount when available.
 */

#include "ZephyrDataStore.h"
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
		ext_lfs_mounted = false;
		LOG_INF("External QSPI NOT mounted at %s - using internal only (20 blobs)", extMountPoint());
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

bool ZephyrDataStore::exists(const char *path)
{
	struct fs_dirent ent;
	return fs_stat(path, &ent) == 0;
}

bool ZephyrDataStore::removeFile(const char *path)
{
	return fs_unlink(path) == 0;
}

bool ZephyrDataStore::openRead(const char *path, uint8_t *buf, size_t buf_sz, size_t &out_len)
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

/* Power-safe replace: used for identity + prefs; channels use the same .tmp/sync/rename in saveChannels(). Contacts stay direct write (space). */
bool ZephyrDataStore::atomicReplaceFile(const char *path, const uint8_t *buf, size_t len)
{
	char tmp_path[56];
	int pl = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
	if (pl <= 0 || pl >= (int)sizeof(tmp_path)) {
		LOG_ERR("atomicReplaceFile: path too long");
		return false;
	}

	fs_unlink(tmp_path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("atomicReplaceFile: open %s failed: %d", tmp_path, rc);
		return false;
	}

	ssize_t n = fs_write(&file, buf, len);
	if (n < 0 || (size_t)n != len) {
		LOG_ERR("atomicReplaceFile: write %s failed (%d)", tmp_path, (int)n);
		fs_close(&file);
		fs_unlink(tmp_path);
		return false;
	}

	rc = fs_sync(&file);
	fs_close(&file);
	if (rc < 0) {
		LOG_ERR("atomicReplaceFile: sync %s failed: %d", tmp_path, rc);
		fs_unlink(tmp_path);
		return false;
	}

	rc = fs_rename(tmp_path, path);
	if (rc < 0) {
		LOG_ERR("atomicReplaceFile: rename %s -> %s failed: %d", tmp_path, path, rc);
		fs_unlink(tmp_path);
		return false;
	}
	return true;
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
	unmount();

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

	bool mounted = mount();
	LOG_INF("formatFileSystem: mount() returned %d", mounted ? 1 : 0);
	return mounted;
}

void ZephyrDataStore::factoryReset()
{
	LOG_INF("=== FACTORY RESET STARTING ===");
	if (formatFileSystem()) {
		LOG_INF("=== FACTORY RESET COMPLETE - REBOOT REQUIRED ===");
	} else {
		LOG_ERR("=== FACTORY RESET FAILED ===");
	}
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

/* ── Preferences ───────────────────────────────────────────────────── */

void ZephyrDataStore::loadPrefs(NodePrefs &prefs)
{
	bool prefs_exists = exists(PREFS_FILE);
	if (!prefs_exists) {
		LOG_DBG("loadPrefs: no prefs file found");
		return;
	}

	uint8_t buf[256];
	size_t len = 0;
	if (!openRead(PREFS_FILE, buf, sizeof(buf), len)) {
		LOG_ERR("loadPrefs: read failed");
		return;
	}
	if (len < 88) {
		LOG_ERR("loadPrefs: file too small (%d bytes, need 88)", (int)len);
		return;
	}
	LOG_DBG("loadPrefs: loaded %d bytes from %s", (int)len, PREFS_FILE);

	size_t off = 0;
	memcpy(&prefs.airtime_factor, &buf[off], sizeof(float));
	off += 4;
	/* Migrate old AF multiplier (0-9) to duty cycle percentage (0-99) */
	if (prefs.airtime_factor > 0.0f && prefs.airtime_factor <= 9.0f) {
		prefs.airtime_factor *= 10.0f;
	}
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

	/* Offset 94: apc_enabled (ZephCore extension) */
	if (off < len) {
		prefs.apc_enabled = buf[off++];
	}

	/* Offset 95: apc_margin (ZephCore extension) */
	if (off < len) {
		prefs.apc_margin = buf[off++];
		if (prefs.apc_margin < 6 || prefs.apc_margin > 30) {
			prefs.apc_margin = 20;  /* companion default */
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
	/* Offset 94: apc_enabled (ZephCore extension) */
	buf[off++] = prefs.apc_enabled;
	/* Offset 95: apc_margin (ZephCore extension) */
	buf[off++] = prefs.apc_margin;
	/* Total: 96 bytes (Arduino reads 92, ZephCore reads 96) */

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

	if (exists(path)) {
		fs_unlink(path);
	}

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("saveContacts: fs_open(%s) failed: %d", path, rc);
		return;
	}

	uint8_t rec[CONTACT_DATA_SZ];
	uint32_t idx = 0;
	ContactInfo c;

	while (host->getContactForSave(idx, c)) {
		contact_to_record(c, rec);

		if (fs_write(&file, rec, CONTACT_DATA_SZ) != (ssize_t)CONTACT_DATA_SZ) {
			LOG_ERR("saveContacts: write failed at record %u", idx);
			break;
		}
		idx++;
	}

	fs_sync(&file);
	fs_close(&file);
	LOG_INF("saveContacts: saved %u contacts to %s (%u bytes)",
		idx, path, idx * CONTACT_DATA_SZ);
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
	char tmp_path[56];
	int pl = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
	if (pl <= 0 || pl >= (int)sizeof(tmp_path)) {
		LOG_ERR("saveChannels: path too long");
		return;
	}

	fs_unlink(tmp_path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE) < 0) {
		LOG_ERR("saveChannels: fs_open(%s) failed", tmp_path);
		return;
	}
	uint8_t channel_idx = 0;
	ChannelDetails ch;
	uint8_t unused[4] = {0};
	bool write_ok = true;
	while (host->getChannelForSave(channel_idx, ch)) {
		if (fs_write(&file, unused, 4) != 4 ||
		    fs_write(&file, (uint8_t *)ch.name, 32) != 32 ||
		    fs_write(&file, (uint8_t *)ch.channel.secret, 32) != 32) {
			write_ok = false;
			break;
		}
		channel_idx++;
	}
	int sync_rc = fs_sync(&file);
	fs_close(&file);
	if (!write_ok || sync_rc < 0) {
		LOG_ERR("saveChannels: write/sync failed");
		fs_unlink(tmp_path);
		return;
	}
	if (fs_rename(tmp_path, path) < 0) {
		LOG_ERR("saveChannels: rename %s failed", tmp_path);
		fs_unlink(tmp_path);
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
	struct fs_statvfs sbuf;
	if (fs_statvfs(MNT_POINT, &sbuf) != 0) {
		return 0;
	}
	uint32_t total = sbuf.f_blocks * sbuf.f_frsize;
	uint32_t free = sbuf.f_bfree * sbuf.f_frsize;
	return (total - free) / 1024;
}

uint32_t ZephyrDataStore::getStorageTotalKb() const
{
	struct fs_statvfs sbuf;
	if (fs_statvfs(MNT_POINT, &sbuf) != 0) {
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
