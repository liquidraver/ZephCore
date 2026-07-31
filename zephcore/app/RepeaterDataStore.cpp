/*
 * SPDX-License-Identifier: MIT
 * RepeaterDataStore - Filesystem storage for repeater
 */

#include "RepeaterDataStore.h"
#include <zephyr/fs/fs.h>
#include <zephyr/storage/flash_map.h>
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

const char* RepeaterDataStore::getAclPath() const {
    static char buf[48];
    snprintf(buf, sizeof(buf), "%s/acl", BASE_PATH);
    return buf;
}

const char* RepeaterDataStore::getRegionsPath() const {
    static char buf[48];
    snprintf(buf, sizeof(buf), "%s/regions2", BASE_PATH);
    return buf;
}

bool RepeaterDataStore::loadIdentity(mesh::LocalIdentity& id) {
    char path[48];
    snprintf(path, sizeof(path), "%s/_main.id", BASE_PATH);

    struct fs_file_t file;
    fs_file_t_init(&file);

    int ret = fs_open(&file, path, FS_O_READ);
    if (ret < 0) {
        LOG_DBG("No identity file at %s", path);
        return false;
    }

    uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
    ssize_t n = fs_read(&file, buf, sizeof(buf));
    fs_close(&file);

    LOG_DBG("loadIdentity: read %d bytes from %s", (int)n, path);

    if (n >= PRV_KEY_SIZE) {
        if (id.readFrom(buf, n)) {
            LOG_INF("Loaded identity from %s", path);
            return true;
        }
        LOG_ERR("loadIdentity: readFrom failed");
    }

    LOG_ERR("Identity file corrupt");
    return false;
}

bool RepeaterDataStore::saveIdentity(const mesh::LocalIdentity& id) {
    if (!_initialized) begin();

    char path[48];
    char tmp_path[56];
    snprintf(path, sizeof(path), "%s/_main.id", BASE_PATH);
    if (snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path) >= (int)sizeof(tmp_path)) {
        return false;
    }

    fs_unlink(tmp_path);

    struct fs_file_t file;
    fs_file_t_init(&file);

    int ret = fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to open %s for write: %d", tmp_path, ret);
        return false;
    }

    uint8_t buf[PRV_KEY_SIZE];
    int len = id.writeTo(buf, sizeof(buf));
    ssize_t n = fs_write(&file, buf, len);
    ret = fs_sync(&file);
    fs_close(&file);

    if (n != len || ret < 0) {
        LOG_ERR("Failed to write identity: wrote %d of %d sync=%d", (int)n, len, ret);
        fs_unlink(tmp_path);
        return false;
    }

    if (fs_rename(tmp_path, path) < 0) {
        LOG_ERR("saveIdentity: rename failed");
        fs_unlink(tmp_path);
        return false;
    }
    LOG_INF("Saved identity to %s", path);
    return true;
}

bool RepeaterDataStore::loadPrefs(NodePrefs& prefs) {
    char path[48];
    snprintf(path, sizeof(path), "%s/prefs", BASE_PATH);

    struct fs_file_t file;
    fs_file_t_init(&file);

    int ret = fs_open(&file, path, FS_O_READ);
    if (ret < 0) {
        LOG_DBG("No prefs file at %s, using defaults", path);
        initNodePrefs(&prefs);
        strcpy(prefs.node_name, "Repeater");
        prefs.advert_loc_policy = ADVERT_LOC_PREFS;
        prefs.loop_detect = LOOP_DETECT_MODERATE;
        prefs.path_hash_mode = 1;
        prefs.gps_interval = CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC;  // repeater default (48h)
        /* Persist defaults so flash always has a prefs file from boot 1.
         * Lets later code (e.g. tempradio revert) trust that flash is
         * authoritative without a "first run" special case. */
        savePrefs(prefs);
        return true;
    }

    struct fs_dirent entry;
    ret = fs_stat(path, &entry);
    LOG_DBG("loadPrefs: file size = %d bytes", ret < 0 ? 0 : (int)entry.size);

    uint8_t pad[25];

    /* Read prefs in same format as Arduino CommonCLI for compatibility */
    fs_read(&file, &prefs.airtime_factor, sizeof(prefs.airtime_factor));
    fs_read(&file, &prefs.node_name, sizeof(prefs.node_name));
    fs_read(&file, pad, 4);
    fs_read(&file, &prefs.node_lat, sizeof(prefs.node_lat));
    fs_read(&file, &prefs.node_lon, sizeof(prefs.node_lon));
    fs_read(&file, &prefs.password, sizeof(prefs.password));
    fs_read(&file, &prefs.freq, sizeof(prefs.freq));
    fs_read(&file, &prefs.tx_power_dbm, sizeof(prefs.tx_power_dbm));
    fs_read(&file, &prefs.disable_fwd, sizeof(prefs.disable_fwd));
    fs_read(&file, &prefs.advert_interval, sizeof(prefs.advert_interval));
    fs_read(&file, pad, 1);
    fs_read(&file, &prefs.rx_delay_base, sizeof(prefs.rx_delay_base));
    fs_read(&file, &prefs.tx_delay_factor, sizeof(prefs.tx_delay_factor));
    fs_read(&file, &prefs.guest_password, sizeof(prefs.guest_password));
    fs_read(&file, &prefs.direct_tx_delay_factor, sizeof(prefs.direct_tx_delay_factor));
    fs_read(&file, &prefs.backoff_multiplier, sizeof(prefs.backoff_multiplier));
    fs_read(&file, &prefs.sf, sizeof(prefs.sf));
    fs_read(&file, &prefs.cr, sizeof(prefs.cr));
    fs_read(&file, &prefs.allow_read_only, sizeof(prefs.allow_read_only));
    fs_read(&file, &prefs.multi_acks, sizeof(prefs.multi_acks));
    fs_read(&file, &prefs.bw, sizeof(prefs.bw));
    /* 120: leds_disabled, magic-encoded. Formerly agc_reset_interval — see the
     * LEDS_PREF_* comment in NodePrefs.h for why this is not a bare 0/1.
     * leds_byte stays 0 (→ LEDs on) if the file is short. */
    uint8_t leds_byte = 0;
    fs_read(&file, &leds_byte, sizeof(leds_byte));
    fs_read(&file, &prefs.path_hash_mode, sizeof(prefs.path_hash_mode));
    fs_read(&file, &prefs.loop_detect, sizeof(prefs.loop_detect));
    fs_read(&file, pad, 1);
    fs_read(&file, &prefs.flood_max, sizeof(prefs.flood_max));
    fs_read(&file, &prefs.flood_advert_interval, sizeof(prefs.flood_advert_interval));
    fs_read(&file, &prefs.interference_threshold, sizeof(prefs.interference_threshold));
    fs_read(&file, pad, 25);  // skip bridge settings
    fs_read(&file, &prefs.powersaving_enabled, sizeof(prefs.powersaving_enabled));
    fs_read(&file, pad, 3);
    fs_read(&file, &prefs.gps_enabled, sizeof(prefs.gps_enabled));
    fs_read(&file, &prefs.gps_interval, sizeof(prefs.gps_interval));
    fs_read(&file, &prefs.advert_loc_policy, sizeof(prefs.advert_loc_policy));
    fs_read(&file, &prefs.discovery_mod_timestamp, sizeof(prefs.discovery_mod_timestamp));
    fs_read(&file, &prefs.adc_multiplier, sizeof(prefs.adc_multiplier));
    fs_read(&file, prefs.owner_info, sizeof(prefs.owner_info));
    /* ZephCore extensions — absent in old 290-byte files; fs_read past EOF is a
     * no-op so these fields keep the initNodePrefs() defaults the caller passed
     * in (rx_boost=1, rx_duty_cycle=0). The upgrade block below forces
     * repeater-specific values for old files. */
    fs_read(&file, &prefs.rx_boost, sizeof(prefs.rx_boost));
    fs_read(&file, &prefs.rx_duty_cycle, sizeof(prefs.rx_duty_cycle));
    /* RESERVED — formerly apc_enabled / apc_margin (APC, removed in 1.16.6).
     * Still consumed so the fields after them stay at their stored offsets. */
    fs_read(&file, &prefs._reserved_apc_enabled, sizeof(prefs._reserved_apc_enabled));
    fs_read(&file, &prefs._reserved_apc_margin, sizeof(prefs._reserved_apc_margin));
    /* Flood hop-ceiling extensions (absent in <296-byte files; the no-op EOF
     * read leaves the constructor defaults flood_max_unscoped=64, flood_max_advert=8). */
    fs_read(&file, &prefs.flood_max_unscoped, sizeof(prefs.flood_max_unscoped));
    fs_read(&file, &prefs.flood_max_advert, sizeof(prefs.flood_max_advert));
    /* Mesh time sync (absent in <297-byte files; no-op EOF read keeps default 0 = off) */
    fs_read(&file, &prefs.meshtimesync, sizeof(prefs.meshtimesync));
    /* Adaptive CAD (absent in <300-byte files; no-op EOF reads keep defaults
     * auto=0, offset=0, probe_interval=60) */
    fs_read(&file, &prefs.cad_auto, sizeof(prefs.cad_auto));
    fs_read(&file, &prefs.cad_offset, sizeof(prefs.cad_offset));
    fs_read(&file, &prefs.probe_interval, sizeof(prefs.probe_interval));
    /* cad_busycap absent in <301-byte files; EOF read keeps default 25 */
    fs_read(&file, &prefs.cad_busycap, sizeof(prefs.cad_busycap));

    fs_close(&file);

    /* Only the explicit "off" magic disables LEDs; a legacy AGC interval or an
     * unwritten byte both mean "on". */
    prefs.leds_disabled = (leds_byte == LEDS_PREF_OFF) ? 1 : 0;

    /* Migrate uninitialized backoff_multiplier (0.0 or NaN) to default */
    if (prefs.backoff_multiplier == 0.0f || prefs.backoff_multiplier != prefs.backoff_multiplier) {
        prefs.backoff_multiplier = 0.2f;
    }

    LOG_INF("Loaded prefs from %s", path);
    LOG_DBG("  name='%s' freq=%.3f sf=%u bw=%.1f tx_pwr=%d",
            prefs.node_name, (double)prefs.freq, prefs.sf, (double)prefs.bw, prefs.tx_power_dbm);

    /* Validate radio params - use defaults if garbage */
    if (prefs.freq < 300.0f || prefs.freq > 1000.0f ||
        prefs.sf < 5 || prefs.sf > 12 ||
        prefs.bw < 7.0f || prefs.bw > 500.0f) {
        LOG_WRN("Invalid radio params in prefs, using defaults: freq=%.3f sf=%u bw=%.1f",
                (double)prefs.freq, prefs.sf, (double)prefs.bw);
        prefs.freq = 869.618f;
        prefs.bw = 62.5f;
        prefs.sf = 8;
        prefs.cr = 8;
        prefs.tx_power_dbm = 22;
    }
    if (prefs.path_hash_mode > 2) prefs.path_hash_mode = 0;
    if (prefs.loop_detect > LOOP_DETECT_STRICT) prefs.loop_detect = LOOP_DETECT_MINIMAL;
    if (prefs.rx_boost > 1) prefs.rx_boost = 0;
    if (prefs.rx_duty_cycle > 1) prefs.rx_duty_cycle = 0;
    if (prefs.meshtimesync > 1) prefs.meshtimesync = 0;
    if (prefs.cad_auto > 1) prefs.cad_auto = 0;
    if (prefs.cad_offset < CAD_OFFSET_MIN || prefs.cad_offset > CAD_OFFSET_MAX) prefs.cad_offset = 0;
    if (prefs.probe_interval != 0 && prefs.probe_interval < 10) prefs.probe_interval = 10;
    if (prefs.cad_busycap > 90) prefs.cad_busycap = 90;

    /* One-time format upgrade: old files (< 294 bytes) never saved the ZephCore
     * extension fields, and stored path_hash_mode/loop_detect as zero padding.
     * Apply repeater defaults and re-save so values survive subsequent reboots. */
    if (ret >= 0 && entry.size < 294) {
        prefs.rx_boost = 1;
        prefs.path_hash_mode = 1;
        prefs.loop_detect = LOOP_DETECT_MODERATE;
        savePrefs(prefs);
        LOG_INF("loadPrefs: upgraded prefs format (%d -> 297 bytes)", (int)entry.size);
    }

    /* Repeater GPS-interval unification migration: before this firmware the
     * repeater ignored gps_interval (hardcoded 48h), so a stored companion
     * default (300) was never a deliberate choice. Bump it to the repeater
     * default once, so now-honoring the field doesn't silently switch existing
     * units to 5-min GPS polling. (Triggers only on exactly 300; after the
     * one-time rewrite it won't re-fire. A deliberate 300 on a repeater isn't
     * reachable via the CLI — use 299/301 if you really want ~5 min.) */
    if (prefs.gps_interval == CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC) {
        prefs.gps_interval = CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC;
        savePrefs(prefs);
    }

    return true;
}

bool RepeaterDataStore::savePrefs(const NodePrefs& prefs) {
    if (!_initialized) begin();

    char path[48];
    char tmp_path[56];
    snprintf(path, sizeof(path), "%s/prefs", BASE_PATH);
    if (snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path) >= (int)sizeof(tmp_path)) {
        return false;
    }

    fs_unlink(tmp_path);

    struct fs_file_t file;
    fs_file_t_init(&file);

    int ret = fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to open %s for write: %d", tmp_path, ret);
        return false;
    }

    uint8_t pad[25];
    memset(pad, 0, sizeof(pad));

    /* Write prefs in same format as Arduino CommonCLI for compatibility */
    fs_write(&file, &prefs.airtime_factor, sizeof(prefs.airtime_factor));
    fs_write(&file, &prefs.node_name, sizeof(prefs.node_name));
    fs_write(&file, pad, 4);
    fs_write(&file, &prefs.node_lat, sizeof(prefs.node_lat));
    fs_write(&file, &prefs.node_lon, sizeof(prefs.node_lon));
    fs_write(&file, &prefs.password, sizeof(prefs.password));
    fs_write(&file, &prefs.freq, sizeof(prefs.freq));
    fs_write(&file, &prefs.tx_power_dbm, sizeof(prefs.tx_power_dbm));
    fs_write(&file, &prefs.disable_fwd, sizeof(prefs.disable_fwd));
    fs_write(&file, &prefs.advert_interval, sizeof(prefs.advert_interval));
    fs_write(&file, pad, 1);
    fs_write(&file, &prefs.rx_delay_base, sizeof(prefs.rx_delay_base));
    fs_write(&file, &prefs.tx_delay_factor, sizeof(prefs.tx_delay_factor));
    fs_write(&file, &prefs.guest_password, sizeof(prefs.guest_password));
    fs_write(&file, &prefs.direct_tx_delay_factor, sizeof(prefs.direct_tx_delay_factor));
    fs_write(&file, &prefs.backoff_multiplier, sizeof(prefs.backoff_multiplier));
    fs_write(&file, &prefs.sf, sizeof(prefs.sf));
    fs_write(&file, &prefs.cr, sizeof(prefs.cr));
    fs_write(&file, &prefs.allow_read_only, sizeof(prefs.allow_read_only));
    fs_write(&file, &prefs.multi_acks, sizeof(prefs.multi_acks));
    fs_write(&file, &prefs.bw, sizeof(prefs.bw));
    /* 120: leds_disabled, magic-encoded (was agc_reset_interval). */
    {
        uint8_t leds_byte = prefs.leds_disabled ? LEDS_PREF_OFF : LEDS_PREF_ON;
        fs_write(&file, &leds_byte, sizeof(leds_byte));
    }
    fs_write(&file, &prefs.path_hash_mode, sizeof(prefs.path_hash_mode));
    fs_write(&file, &prefs.loop_detect, sizeof(prefs.loop_detect));
    fs_write(&file, pad, 1);
    fs_write(&file, &prefs.flood_max, sizeof(prefs.flood_max));
    fs_write(&file, &prefs.flood_advert_interval, sizeof(prefs.flood_advert_interval));
    fs_write(&file, &prefs.interference_threshold, sizeof(prefs.interference_threshold));
    fs_write(&file, pad, 25);  // skip bridge settings
    fs_write(&file, &prefs.powersaving_enabled, sizeof(prefs.powersaving_enabled));
    fs_write(&file, pad, 3);
    fs_write(&file, &prefs.gps_enabled, sizeof(prefs.gps_enabled));
    fs_write(&file, &prefs.gps_interval, sizeof(prefs.gps_interval));
    fs_write(&file, &prefs.advert_loc_policy, sizeof(prefs.advert_loc_policy));
    fs_write(&file, &prefs.discovery_mod_timestamp, sizeof(prefs.discovery_mod_timestamp));
    fs_write(&file, &prefs.adc_multiplier, sizeof(prefs.adc_multiplier));
    fs_write(&file, prefs.owner_info, sizeof(prefs.owner_info));
    /* ZephCore extensions */
    fs_write(&file, &prefs.rx_boost, sizeof(prefs.rx_boost));
    fs_write(&file, &prefs.rx_duty_cycle, sizeof(prefs.rx_duty_cycle));
    /* RESERVED — formerly apc_enabled / apc_margin (removed in 1.16.6).
     * Written back unchanged to hold the layout. */
    fs_write(&file, &prefs._reserved_apc_enabled, sizeof(prefs._reserved_apc_enabled));
    fs_write(&file, &prefs._reserved_apc_margin, sizeof(prefs._reserved_apc_margin));
    /* Flood hop-ceiling extensions (extend the format past 294 bytes) */
    fs_write(&file, &prefs.flood_max_unscoped, sizeof(prefs.flood_max_unscoped));
    fs_write(&file, &prefs.flood_max_advert, sizeof(prefs.flood_max_advert));
    /* Mesh time sync on/off (offset 296) */
    fs_write(&file, &prefs.meshtimesync, sizeof(prefs.meshtimesync));
    /* Adaptive CAD (offsets 297-300) */
    fs_write(&file, &prefs.cad_auto, sizeof(prefs.cad_auto));
    fs_write(&file, &prefs.cad_offset, sizeof(prefs.cad_offset));
    fs_write(&file, &prefs.probe_interval, sizeof(prefs.probe_interval));
    fs_write(&file, &prefs.cad_busycap, sizeof(prefs.cad_busycap));

    ret = fs_sync(&file);
    fs_close(&file);
    if (ret < 0) {
        LOG_ERR("savePrefs: sync failed: %d", ret);
        fs_unlink(tmp_path);
        return false;
    }

    if (fs_rename(tmp_path, path) < 0) {
        LOG_ERR("savePrefs: rename failed");
        fs_unlink(tmp_path);
        return false;
    }
    LOG_INF("Saved prefs to %s", path);
    return true;
}

bool RepeaterDataStore::formatFileSystem() {
    LOG_WRN("Factory reset: erasing repeater data at %s", BASE_PATH);

    struct fs_dir_t dir;
    fs_dir_t_init(&dir);

    int ret = fs_opendir(&dir, BASE_PATH);
    if (ret < 0) {
        LOG_WRN("No repeater directory to erase");
        return true;
    }

    struct fs_dirent entry;
    char path[280];

    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        snprintf(path, sizeof(path), "%s/%s", BASE_PATH, entry.name);
        LOG_INF("Deleting %s", path);
        fs_unlink(path);
    }
    fs_closedir(&dir);

#if FIXED_PARTITION_EXISTS(storage_partition)
    /* Erase the NVS bonds partition too — a factory reset should clear BLE
     * bonds, not just repeater files.  Caller reboots so NVS re-inits clean. */
    const struct flash_area *fap;
    if (flash_area_open(PARTITION_ID(storage_partition), &fap) == 0) {
        LOG_INF("Formatting NVS storage (%u bytes)", (unsigned)fap->fa_size);
        flash_area_flatten(fap, 0, fap->fa_size);
        flash_area_close(fap);
    }
#endif

    LOG_INF("Repeater data erased");
    return true;
}
