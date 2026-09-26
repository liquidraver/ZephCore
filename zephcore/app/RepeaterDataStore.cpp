/*
 * SPDX-License-Identifier: MIT
 * RepeaterDataStore - Filesystem storage for repeater
 */

#include "RepeaterDataStore.h"
#include "../adapters/datastore/ZephyrFsFormat.h"
#include "../adapters/datastore/ZephyrFsUtil.h"
#include "../adapters/datastore/IdentityFile.h"
#include "../adapters/datastore/PrefsFile.h"
#include <helpers/PrefsCodec.h>
#include <helpers/LoRaConfig.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(zephcore_repeater_store, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

RepeaterDataStore::RepeaterDataStore() : _initialized(false) {
}

bool RepeaterDataStore::begin() {
	if (_initialized) return true;

	/* Create repeater directory if it doesn't exist */
	struct fs_dirent entry;
	int ret = fs_stat(BASE_PATH, &entry);
	if (ret < 0) {
		ret = fs_mkdir(BASE_PATH);
		if (ret < 0 && ret != -EEXIST) {
			LOG_ERR("Failed to create %s: %d", BASE_PATH, ret);
			return false;
		}
		LOG_INF("Created %s directory", BASE_PATH);
	}

	_initialized = true;
	LOG_INF("RepeaterDataStore initialized at %s", BASE_PATH);
	return true;
}

const char* RepeaterDataStore::getBasePath() const { return BASE_PATH; }

bool RepeaterDataStore::hasRoleData() const {
	char path[64];

	/* Only THIS role's files count.  A companion volume does not: the roles
	 * are deliberately not interchangeable, and a companion's contacts and
	 * blob cache would eat into the same 128 KB the repeater needs, so a
	 * repeater booting onto a companion volume formats it.  The reverse
	 * already happens — ZephyrDataStore::hasPrefs() tests /lfs/new_prefs,
	 * which a repeater volume never has.
	 *
	 * Repeater, room server and observer DO share this store and base path;
	 * they use the same prefs layout, so switching among them keeps the
	 * node's identity, which is what an operator wants.
	 *
	 * Self-limiting: loadPrefs() persists defaults on boot 1 and main_*.cpp
	 * saves a generated identity on the same boot, so after one successful
	 * boot at least one of these exists and the check never fires again. */
	static const char* const ours[] = { "prefs.json", "prefs", "_main.id" };
	for (size_t i = 0; i < ARRAY_SIZE(ours); i++) {
		snprintf(path, sizeof(path), "%s/%s", BASE_PATH, ours[i]);
		if (zephcore_fs_exists(path)) return true;
	}

	return false;
}

bool RepeaterDataStore::loadIdentity(mesh::LocalIdentity& id) {
	char path[48];
	snprintf(path, sizeof(path), "%s/_main.id", BASE_PATH);
	return zephcore_identity_load(path, id);
}

bool RepeaterDataStore::saveIdentity(const mesh::LocalIdentity& id) {
	if (!_initialized) begin();

	char path[48];
	snprintf(path, sizeof(path), "%s/_main.id", BASE_PATH);
	return zephcore_identity_save(path, id);
}

bool RepeaterDataStore::loadPrefs(NodePrefs& prefs) {
	char path[48];
	snprintf(path, sizeof(path), "%s/prefs.json", BASE_PATH);

	if (zephcore_prefs_json_load(path, prefs, serverPrefsFromJson)) {
		LOG_INF("Loaded prefs from %s", path);
		return true;
	}
	bool had_json = zephcore_fs_exists(path);

	/* No usable prefs.json: the legacy binary file, read once and migrated.
	 * It is kept, so firmware from before prefs.json still boots with the
	 * settings as they were at the upgrade. With neither, the caller's prefs
	 * are the role's defaults (set in its constructor). Either way they are
	 * saved, so flash always holds a prefs file from boot 1 and later code
	 * (e.g. tempradio revert) can trust it without a "first run" case. */
	char legacy[48];
	snprintf(legacy, sizeof(legacy), "%s/prefs", BASE_PATH);
	if (zephcore_fs_exists(legacy)) {
		if (loadLegacyPrefs(legacy, prefs)) {
			LOG_INF("loadPrefs: %s %s from %s", had_json ? "recovered" : "migrated",
				path, legacy);
		}
	} else {
		LOG_DBG("No prefs file at %s, saving the role defaults", path);
	}
	savePrefs(prefs);
	return true;
}

bool RepeaterDataStore::loadLegacyPrefs(const char* path, NodePrefs& prefs) {
	uint8_t buf[SERVER_PREFS_SIZE + 32];
	size_t len = 0;
	if (!zephcore_fs_read_file(path, buf, sizeof(buf), &len)) {
		LOG_ERR("loadPrefs: read of %s failed, using defaults", path);
		return false;
	}
	LOG_DBG("loadPrefs: file size = %d bytes", (int)len);

	if (len >= 120) {
		float freq, bw;
		memcpy(&freq, &buf[72], sizeof(freq));
		memcpy(&bw, &buf[116], sizeof(bw));
		if (freq < ZC_RADIO_FREQ_MIN_MHZ || freq > ZC_RADIO_FREQ_MAX_MHZ ||
		    buf[112] < 5 || buf[112] > 12 ||
		    bw < ZC_RADIO_BW_MIN_KHZ || bw > ZC_RADIO_BW_MAX_KHZ) {
			LOG_WRN("Invalid radio params in prefs, using defaults: freq=%.3f sf=%u bw=%.1f",
				(double)freq, buf[112], (double)bw);
		}
	}
	/* Radio params outside ZC_RADIO_* fall back to the LoRaConfig defaults,
	 * everything else is sanitized; fields past the end keep the caller's. */
	serverPrefsDecode(prefs, buf, len);

	LOG_DBG("  name='%s' freq=%.3f sf=%u bw=%.1f tx_pwr=%d",
		prefs.node_name, (double)prefs.freq, prefs.sf, (double)prefs.bw, prefs.tx_power_dbm);

	/* Old files (< 294 bytes) never saved the ZephCore extension fields, and
	 * stored path_hash_mode/loop_detect as zero padding: the repeater defaults. */
	if (len < 294) {
		prefs.rx_boost = 1;
		prefs.path_hash_mode = 1;
		prefs.loop_detect = LOOP_DETECT_MODERATE;
		LOG_INF("loadPrefs: upgraded an old %d-byte prefs file", (int)len);
	}

	/* Before the repeater honoured gps_interval it ran a fixed 48 h, so a
	 * stored companion default (300) was never a choice: the repeater
	 * default instead. Legacy files only; prefs.json is past this. */
	if (prefs.gps_interval == CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC) {
		prefs.gps_interval = CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC;
	}
	/* Likewise gps_enabled: that firmware ran the GPS whatever it said. */
	prefs.gps_enabled = 1;
	prefs.gps_enabled_set = 1;
	/* And powersaving, which it stored but never acted on. */
	prefs.powersaving_set = 0;
	powersaving_upgrade(&prefs);
	return true;
}

bool RepeaterDataStore::savePrefs(const NodePrefs& prefs) {
	if (!_initialized) begin();

	char path[48];
	snprintf(path, sizeof(path), "%s/prefs.json", BASE_PATH);
	if (!zephcore_prefs_json_save(path, prefs, serverPrefsToJson)) {
		return false;
	}
	LOG_INF("Saved prefs to %s", path);
	return true;
}

bool RepeaterDataStore::formatFileSystem() {
	LOG_WRN("Factory reset: erasing all storage");

	/* Erase the LittleFS *volume*, not just our files.  The old loop walked
	 * /lfs/repeater/ with fs_unlink, which left the volume itself untouched:
	 * it could not recover a volume another firmware had written into (on
	 * nRF52840 the Adafruit core's filesystem overlaps the top of ours), and
	 * it left /lfs/settings, stale companion files and all of /ext behind.
	 * Shared with the companion so all four roles erase the same regions. */
	bool mounted = zephcore_fs_format_all(nullptr);
	if (!mounted) {
		LOG_ERR("Factory reset: /lfs did not remount");
		return false;
	}

	/* The format took /lfs/repeater with it.  Re-create it now rather than
	 * relying on the reboot: the CLI defers the reset so the reply can be
	 * transmitted, and anything that saves in that window needs the dir. */
	_initialized = false;
	if (!begin()) {
		LOG_ERR("Factory reset: could not re-create %s", BASE_PATH);
		return false;
	}

	LOG_INF("Repeater data erased");
	return true;
}
