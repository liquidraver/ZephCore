/*
 * SPDX-License-Identifier: Apache-2.0
 * CommonCLI - Common CLI command handlers for repeaters
 */

#include "CommonCLI.h"
#include <helpers/TxtDataHelpers.h>
#include <helpers/AdvertDataHelpers.h>
#include <adapters/board/ZephyrBoard.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_WIFI_OTA)
#include "wifi_ota.h"
#endif

LOG_MODULE_REGISTER(zephcore_cli, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

// Helper: robust atoi
static uint32_t _atoi(const char* sp) {
    uint32_t n = 0;
    while (*sp && *sp >= '0' && *sp <= '9') {
        n *= 10;
        n += (*sp++ - '0');
    }
    return n;
}

static bool isValidName(const char* n) {
    while (*n) {
        if (*n == '[' || *n == ']' || *n == '\\' || *n == ':' ||
            *n == ',' || *n == '?' || *n == '*') return false;
        n++;
    }
    return true;
}

// Constrain helper
template<typename T>
static T constrain(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/* Read exactly 'len' bytes; short/error read stops the chain via ok flag */
static inline bool prefs_read(struct fs_file_t *f, void *dest, size_t len) {
    return fs_read(f, dest, len) == (ssize_t)len;
}

void CommonCLI::loadPrefs(const char* path) {
    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, path, FS_O_READ) < 0) {
        LOG_DBG("No prefs file at %s, using defaults", path);
        return;
    }

    uint8_t pad[8];
    bool ok = true;

    /* Read fields in Arduino-compatible binary order.
     * On truncated file, short-circuit at first failure
     * so remaining fields keep their default values. */
    ok = ok && prefs_read(&file, &_prefs->airtime_factor, sizeof(_prefs->airtime_factor));   // 0
    ok = ok && prefs_read(&file, &_prefs->node_name, sizeof(_prefs->node_name));             // 4
    ok = ok && prefs_read(&file, pad, 4);                                                     // 36
    ok = ok && prefs_read(&file, &_prefs->node_lat, sizeof(_prefs->node_lat));               // 40
    ok = ok && prefs_read(&file, &_prefs->node_lon, sizeof(_prefs->node_lon));               // 48
    ok = ok && prefs_read(&file, &_prefs->password[0], sizeof(_prefs->password));            // 56
    ok = ok && prefs_read(&file, &_prefs->freq, sizeof(_prefs->freq));                       // 72
    ok = ok && prefs_read(&file, &_prefs->tx_power_dbm, sizeof(_prefs->tx_power_dbm));       // 76
    ok = ok && prefs_read(&file, &_prefs->disable_fwd, sizeof(_prefs->disable_fwd));         // 77
    ok = ok && prefs_read(&file, &_prefs->advert_interval, sizeof(_prefs->advert_interval)); // 78
    ok = ok && prefs_read(&file, pad, 1);                                                     // 79
    ok = ok && prefs_read(&file, &_prefs->rx_delay_base, sizeof(_prefs->rx_delay_base));     // 80
    ok = ok && prefs_read(&file, &_prefs->tx_delay_factor, sizeof(_prefs->tx_delay_factor)); // 84
    ok = ok && prefs_read(&file, &_prefs->guest_password[0], sizeof(_prefs->guest_password)); // 88
    ok = ok && prefs_read(&file, &_prefs->direct_tx_delay_factor, sizeof(_prefs->direct_tx_delay_factor)); // 104
    ok = ok && prefs_read(&file, &_prefs->backoff_multiplier, sizeof(_prefs->backoff_multiplier)); // 108
    ok = ok && prefs_read(&file, &_prefs->sf, sizeof(_prefs->sf));                           // 112
    ok = ok && prefs_read(&file, &_prefs->cr, sizeof(_prefs->cr));                           // 113
    ok = ok && prefs_read(&file, &_prefs->allow_read_only, sizeof(_prefs->allow_read_only)); // 114
    ok = ok && prefs_read(&file, &_prefs->multi_acks, sizeof(_prefs->multi_acks));           // 115
    ok = ok && prefs_read(&file, &_prefs->bw, sizeof(_prefs->bw));                           // 116
    ok = ok && prefs_read(&file, &_prefs->agc_reset_interval, sizeof(_prefs->agc_reset_interval)); // 120
    ok = ok && prefs_read(&file, &_prefs->path_hash_mode, sizeof(_prefs->path_hash_mode));    // 121
    ok = ok && prefs_read(&file, &_prefs->loop_detect, sizeof(_prefs->loop_detect));          // 122
    ok = ok && prefs_read(&file, pad, 1);                                                     // 123
    ok = ok && prefs_read(&file, &_prefs->flood_max, sizeof(_prefs->flood_max));             // 124
    ok = ok && prefs_read(&file, &_prefs->flood_advert_interval, sizeof(_prefs->flood_advert_interval)); // 125
    ok = ok && prefs_read(&file, &_prefs->interference_threshold, sizeof(_prefs->interference_threshold)); // 126
    ok = ok && prefs_read(&file, pad, 1);  // skip bridge_enabled                             // 127
    ok = ok && prefs_read(&file, pad, 2);  // skip bridge_delay                               // 128
    ok = ok && prefs_read(&file, pad, 1);  // skip bridge_pkt_src                             // 130
    ok = ok && prefs_read(&file, pad, 4);  // skip bridge_baud                                // 131
    ok = ok && prefs_read(&file, pad, 1);  // skip bridge_channel                             // 135
    ok = ok && prefs_read(&file, pad, 16); // skip bridge_secret                              // 136
    ok = ok && prefs_read(&file, &_prefs->powersaving_enabled, sizeof(_prefs->powersaving_enabled)); // 152
    ok = ok && prefs_read(&file, pad, 3);                                                     // 153
    ok = ok && prefs_read(&file, &_prefs->gps_enabled, sizeof(_prefs->gps_enabled));         // 156
    ok = ok && prefs_read(&file, &_prefs->gps_interval, sizeof(_prefs->gps_interval));       // 157
    ok = ok && prefs_read(&file, &_prefs->advert_loc_policy, sizeof(_prefs->advert_loc_policy)); // 161
    ok = ok && prefs_read(&file, &_prefs->discovery_mod_timestamp, sizeof(_prefs->discovery_mod_timestamp)); // 162
    ok = ok && prefs_read(&file, &_prefs->adc_multiplier, sizeof(_prefs->adc_multiplier));   // 166
    ok = ok && prefs_read(&file, _prefs->owner_info, sizeof(_prefs->owner_info));            // 170
    ok = ok && prefs_read(&file, &_prefs->rx_boost, sizeof(_prefs->rx_boost));               // 290
    ok = ok && prefs_read(&file, &_prefs->rx_duty_cycle, sizeof(_prefs->rx_duty_cycle));     // 291
    ok = ok && prefs_read(&file, &_prefs->apc_enabled, sizeof(_prefs->apc_enabled));         // 292
    ok = ok && prefs_read(&file, &_prefs->apc_margin, sizeof(_prefs->apc_margin));           // 293

    if (!ok) {
        LOG_WRN("Prefs file %s truncated, some fields use defaults", path);
    }

    fs_close(&file);

    // Sanitise bad pref values
    _prefs->rx_delay_base = constrain(_prefs->rx_delay_base, 0.0f, 20.0f);
    _prefs->tx_delay_factor = constrain(_prefs->tx_delay_factor, 0.0f, 2.0f);
    _prefs->direct_tx_delay_factor = constrain(_prefs->direct_tx_delay_factor, 0.0f, 2.0f);
    /* Migrate uninitialized pad bytes: NaN or out-of-range → default 0.5.
     * 0.0 is valid (disables reactive backoff). Old firmware upgrading
     * with zeroed pad bytes will get 0.0 = disabled; user can set explicitly. */
    if (_prefs->backoff_multiplier != _prefs->backoff_multiplier ||
        _prefs->backoff_multiplier < 0.0f || _prefs->backoff_multiplier > 10.0f) {
        _prefs->backoff_multiplier = 0.5f;
    }
    _prefs->backoff_multiplier = constrain(_prefs->backoff_multiplier, 0.0f, 2.0f);
    /* Migrate old AF multiplier (0-9) to duty cycle percentage (0-99) */
    if (_prefs->airtime_factor > 0.0f && _prefs->airtime_factor <= 9.0f) {
        _prefs->airtime_factor *= 10.0f;
    }
    _prefs->airtime_factor = constrain(_prefs->airtime_factor, 0.0f, 99.0f);
    _prefs->freq = constrain(_prefs->freq, 150.0f, 2500.0f);
    _prefs->bw = constrain(_prefs->bw, 7.8f, 500.0f);
    _prefs->sf = constrain(_prefs->sf, (uint8_t)5, (uint8_t)12);
    _prefs->cr = constrain(_prefs->cr, (uint8_t)5, (uint8_t)8);
    _prefs->tx_power_dbm = constrain(_prefs->tx_power_dbm, (int8_t)-9, (int8_t)30);
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
    if (_prefs->tx_power_dbm > CONFIG_ZEPHCORE_MAX_TX_POWER_DBM) {
        _prefs->tx_power_dbm = (int8_t)CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
    }
#endif
    _prefs->multi_acks = constrain(_prefs->multi_acks, (uint8_t)0, (uint8_t)1);
    _prefs->adc_multiplier = constrain(_prefs->adc_multiplier, 0.0f, 10.0f);
    _prefs->path_hash_mode = constrain(_prefs->path_hash_mode, (uint8_t)0, (uint8_t)2);
    _prefs->powersaving_enabled = constrain(_prefs->powersaving_enabled, (uint8_t)0, (uint8_t)1);
    _prefs->gps_enabled = constrain(_prefs->gps_enabled, (uint8_t)0, (uint8_t)1);
    _prefs->advert_loc_policy = constrain(_prefs->advert_loc_policy, (uint8_t)0, (uint8_t)2);
    _prefs->rx_boost = constrain(_prefs->rx_boost, (uint8_t)0, (uint8_t)1);
    _prefs->rx_duty_cycle = constrain(_prefs->rx_duty_cycle, (uint8_t)0, (uint8_t)1);
    _prefs->apc_enabled = constrain(_prefs->apc_enabled, (uint8_t)0, (uint8_t)1);
    _prefs->apc_margin = constrain(_prefs->apc_margin, (uint8_t)6, (uint8_t)30);

    LOG_INF("Loaded prefs from %s", path);
}

void CommonCLI::savePrefs(const char* path) {
    // Remove old file first
    fs_unlink(path);

    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, path, FS_O_CREATE | FS_O_WRITE) < 0) {
        LOG_ERR("Failed to open %s for write", path);
        return;
    }

    uint8_t pad[16];
    memset(pad, 0, sizeof(pad));

    fs_write(&file, &_prefs->airtime_factor, sizeof(_prefs->airtime_factor));
    fs_write(&file, &_prefs->node_name, sizeof(_prefs->node_name));
    fs_write(&file, pad, 4);
    fs_write(&file, &_prefs->node_lat, sizeof(_prefs->node_lat));
    fs_write(&file, &_prefs->node_lon, sizeof(_prefs->node_lon));
    fs_write(&file, &_prefs->password[0], sizeof(_prefs->password));
    fs_write(&file, &_prefs->freq, sizeof(_prefs->freq));
    fs_write(&file, &_prefs->tx_power_dbm, sizeof(_prefs->tx_power_dbm));
    fs_write(&file, &_prefs->disable_fwd, sizeof(_prefs->disable_fwd));
    fs_write(&file, &_prefs->advert_interval, sizeof(_prefs->advert_interval));
    fs_write(&file, pad, 1);
    fs_write(&file, &_prefs->rx_delay_base, sizeof(_prefs->rx_delay_base));
    fs_write(&file, &_prefs->tx_delay_factor, sizeof(_prefs->tx_delay_factor));
    fs_write(&file, &_prefs->guest_password[0], sizeof(_prefs->guest_password));
    fs_write(&file, &_prefs->direct_tx_delay_factor, sizeof(_prefs->direct_tx_delay_factor));
    fs_write(&file, &_prefs->backoff_multiplier, sizeof(_prefs->backoff_multiplier));
    fs_write(&file, &_prefs->sf, sizeof(_prefs->sf));
    fs_write(&file, &_prefs->cr, sizeof(_prefs->cr));
    fs_write(&file, &_prefs->allow_read_only, sizeof(_prefs->allow_read_only));
    fs_write(&file, &_prefs->multi_acks, sizeof(_prefs->multi_acks));
    fs_write(&file, &_prefs->bw, sizeof(_prefs->bw));
    fs_write(&file, &_prefs->agc_reset_interval, sizeof(_prefs->agc_reset_interval));
    fs_write(&file, &_prefs->path_hash_mode, sizeof(_prefs->path_hash_mode));
    fs_write(&file, &_prefs->loop_detect, sizeof(_prefs->loop_detect));
    fs_write(&file, pad, 1);
    fs_write(&file, &_prefs->flood_max, sizeof(_prefs->flood_max));
    fs_write(&file, &_prefs->flood_advert_interval, sizeof(_prefs->flood_advert_interval));
    fs_write(&file, &_prefs->interference_threshold, sizeof(_prefs->interference_threshold));
    fs_write(&file, pad, 1);  // bridge_enabled
    fs_write(&file, pad, 2);  // bridge_delay
    fs_write(&file, pad, 1);  // bridge_pkt_src
    fs_write(&file, pad, 4);  // bridge_baud
    fs_write(&file, pad, 1);  // bridge_channel
    fs_write(&file, pad, 16); // bridge_secret
    fs_write(&file, &_prefs->powersaving_enabled, sizeof(_prefs->powersaving_enabled));
    fs_write(&file, pad, 3);
    fs_write(&file, &_prefs->gps_enabled, sizeof(_prefs->gps_enabled));
    fs_write(&file, &_prefs->gps_interval, sizeof(_prefs->gps_interval));
    fs_write(&file, &_prefs->advert_loc_policy, sizeof(_prefs->advert_loc_policy));
    fs_write(&file, &_prefs->discovery_mod_timestamp, sizeof(_prefs->discovery_mod_timestamp));
    fs_write(&file, &_prefs->adc_multiplier, sizeof(_prefs->adc_multiplier));
    fs_write(&file, _prefs->owner_info, sizeof(_prefs->owner_info));
    fs_write(&file, &_prefs->rx_boost, sizeof(_prefs->rx_boost));
    fs_write(&file, &_prefs->rx_duty_cycle, sizeof(_prefs->rx_duty_cycle));
    fs_write(&file, &_prefs->apc_enabled, sizeof(_prefs->apc_enabled));
    fs_write(&file, &_prefs->apc_margin, sizeof(_prefs->apc_margin));

    fs_close(&file);
    LOG_INF("Saved prefs to %s", path);
}

#define MIN_LOCAL_ADVERT_INTERVAL   60

void CommonCLI::savePrefs() {
    if (_prefs->advert_interval * 2 < MIN_LOCAL_ADVERT_INTERVAL) {
        _prefs->advert_interval = 0;  // turn off, now that device has been manually configured
    }
    _callbacks->savePrefs();
}

uint8_t CommonCLI::buildAdvertData(uint8_t node_type, uint8_t* app_data) {
    if (_prefs->advert_loc_policy == ADVERT_LOC_NONE) {
        AdvertDataBuilder builder(node_type, _prefs->node_name);
        return builder.encodeTo(app_data);
    } else if (_prefs->advert_loc_policy == ADVERT_LOC_SHARE) {
        AdvertDataBuilder builder(node_type, _prefs->node_name,
                                 _callbacks->getNodeLat(), _callbacks->getNodeLon());
        return builder.encodeTo(app_data);
    } else {
        AdvertDataBuilder builder(node_type, _prefs->node_name,
                                 _prefs->node_lat, _prefs->node_lon);
        return builder.encodeTo(app_data);
    }
}

void CommonCLI::rebootWorkHandler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    CommonCLI *self = CONTAINER_OF(dwork, CommonCLI, _reboot_work);

    switch (self->_pending_reboot) {
    case REBOOT_DFU:
        static_cast<mesh::ZephyrBoard*>(self->_board)->rebootToBootloader();
        break;
    case REBOOT_OTA:
        /* reply already sent; startOTAUpdate will reset */
        char dummy[80];
        self->_board->startOTAUpdate(self->_prefs->node_name, dummy);
        break;
    case REBOOT_NORMAL:
    default:
        self->_board->reboot();
        break;
    }
}

void CommonCLI::scheduleReboot(uint8_t type)
{
    _pending_reboot = type;
    /* 2 second delay - enough for LoRa reply to be transmitted */
    k_work_schedule(&_reboot_work, K_SECONDS(2));
}

void CommonCLI::handleCommand(uint32_t sender_timestamp, const char* command, char* reply) {
    if (strcmp(command, "start dfu") == 0) {
        /* Reboot into UF2 bootloader for firmware update */
        strcpy(reply, "OK - rebooting to UF2 DFU");
        scheduleReboot(REBOOT_DFU);
    } else if (memcmp(command, "start ota", 9) == 0) {
#if IS_ENABLED(CONFIG_ZEPHCORE_WIFI_OTA)
        /* ESP32: Start WiFi AP + HTTP OTA server (no reboot) */
        int ota_ret = wifi_ota_start(_prefs->node_name, _board->getManufacturerName());
        if (ota_ret == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "Started: http://%s/update",
                     CONFIG_ZEPHCORE_OTA_AP_IP);
        } else if (ota_ret == -EALREADY) {
            strcpy(reply, "OTA already active");
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "Error starting OTA: %d", ota_ret);
        }
#else
        /* nRF52: Reboot into Adafruit BLE OTA DFU mode */
        strcpy(reply, "OK - rebooting to BLE OTA DFU");
        scheduleReboot(REBOOT_OTA);
#endif
    } else if (memcmp(command, "stop ota", 8) == 0) {
#if IS_ENABLED(CONFIG_ZEPHCORE_WIFI_OTA)
        if (wifi_ota_is_active()) {
            wifi_ota_stop();
            strcpy(reply, "OTA stopped");
        } else {
            strcpy(reply, "OTA not active");
        }
#else
        strcpy(reply, "Not supported");
#endif
    } else if (memcmp(command, "reboot", 6) == 0) {
        strcpy(reply, "OK - rebooting");
        scheduleReboot(REBOOT_NORMAL);
    } else if (memcmp(command, "clkreboot", 9) == 0) {
        getRTCClock()->setCurrentTime(1715770351);  // 15 May 2024, 8:50pm
        _board->reboot();
    } else if (memcmp(command, "advert.zerohop", 14) == 0) {
        _callbacks->sendSelfAdvertisement(1500, false);  // 0-hop (direct) advert
        strcpy(reply, "OK - zerohop advert sent");
    } else if (memcmp(command, "advert", 6) == 0) {
        _callbacks->sendSelfAdvertisement(1500, true);
        strcpy(reply, "OK - Advert sent");
    } else if (memcmp(command, "clock sync", 10) == 0) {
        uint32_t curr = getRTCClock()->getCurrentTime();
        if (sender_timestamp > curr) {
            getRTCClock()->setCurrentTime(sender_timestamp + 1);
            uint32_t now = getRTCClock()->getCurrentTime();
            time_t t = (time_t)now;
            struct tm *tm = gmtime(&t);
            snprintf(reply, CLI_REPLY_SIZE, "OK - clock set: %02d:%02d - %d/%d/%d UTC",
                     tm->tm_hour, tm->tm_min, tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900);
        } else {
            strcpy(reply, "ERR: clock cannot go backwards");
        }
    } else if (memcmp(command, "clock", 5) == 0) {
        uint32_t now = getRTCClock()->getCurrentTime();
        time_t t = (time_t)now;
        struct tm *tm = gmtime(&t);
        snprintf(reply, CLI_REPLY_SIZE, "Clock: %02d:%02d - %d/%d/%d UTC",
                 tm->tm_hour, tm->tm_min, tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900);
    } else if (memcmp(command, "time ", 5) == 0) {
        uint32_t secs = _atoi(&command[5]);
        uint32_t curr = getRTCClock()->getCurrentTime();
        if (secs > curr) {
            getRTCClock()->setCurrentTime(secs);
            time_t t = (time_t)secs;
            struct tm *tm = gmtime(&t);
            snprintf(reply, CLI_REPLY_SIZE, "OK - clock set: %02d:%02d - %d/%d/%d UTC",
                     tm->tm_hour, tm->tm_min, tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900);
        } else {
            strcpy(reply, "(ERR: clock cannot go backwards)");
        }
    } else if (memcmp(command, "neighbors", 9) == 0) {
        _callbacks->formatNeighborsReply(reply);
    } else if (memcmp(command, "neighbor.remove ", 16) == 0) {
        const char* hex = &command[16];
        uint8_t pubkey[PUB_KEY_SIZE];
        int hex_len = strlen(hex);
        if (hex_len > PUB_KEY_SIZE * 2) hex_len = PUB_KEY_SIZE * 2;
        int pubkey_len = hex_len / 2;
        if (mesh::Utils::fromHex(pubkey, pubkey_len, hex)) {
            _callbacks->removeNeighbor(pubkey, pubkey_len);
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "ERR: bad pubkey");
        }
    } else if (memcmp(command, "tempradio ", 10) == 0) {
        strcpy(tmp, &command[10]);
        const char* parts[5];
        int num = mesh::Utils::parseTextParts(tmp, parts, 5);
        float freq = num > 0 ? strtof(parts[0], nullptr) : 0.0f;
        float bw = num > 1 ? strtof(parts[1], nullptr) : 0.0f;
        uint8_t sf = num > 2 ? atoi(parts[2]) : 0;
        uint8_t cr = num > 3 ? atoi(parts[3]) : 0;
        int temp_timeout_mins = num > 4 ? atoi(parts[4]) : 0;
        if (freq >= 150.0f && freq <= 2500.0f && sf >= 5 && sf <= 12 &&
            cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= 500.0f && temp_timeout_mins > 0) {
            _callbacks->applyTempRadioParams(freq, bw, sf, cr, temp_timeout_mins);
            snprintf(reply, CLI_REPLY_SIZE, "OK - temp params for %d mins", temp_timeout_mins);
        } else {
            strcpy(reply, "Error: freq 150-2500, bw 7-500, sf 5-12, cr 5-8, timeout>0");
        }
    } else if (memcmp(command, "password ", 9) == 0) {
        StrHelper::strncpy(_prefs->password, &command[9], sizeof(_prefs->password));
        savePrefs();
        snprintf(reply, CLI_REPLY_SIZE, "password now: %s", _prefs->password);
    } else if (memcmp(command, "clear stats", 11) == 0) {
        _callbacks->clearStats();
        strcpy(reply, "(OK - stats reset)");
    /*
     * GET commands
     */
    } else if (memcmp(command, "get ", 4) == 0) {
        const char* config = &command[4];
        if (memcmp(config, "dutycycle", 9) == 0) {
            float dc = 100.0f / (_prefs->airtime_factor + 1.0f);
            int dc_int = (int)dc;
            int dc_frac = (int)((dc - dc_int) * 10.0f + 0.5f);
            snprintf(reply, CLI_REPLY_SIZE, "> %d.%d%%", dc_int, dc_frac);
        } else if (memcmp(config, "af", 2) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %.2f", (double)_prefs->airtime_factor);
        } else if (memcmp(config, "int.thresh", 10) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->interference_threshold);
        } else if (memcmp(config, "agc.reset.interval", 18) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", ((uint32_t)_prefs->agc_reset_interval) * 4);
        } else if (memcmp(config, "multi.acks", 10) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->multi_acks);
        } else if (memcmp(config, "allow.read.only", 15) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->allow_read_only ? "on" : "off");
        } else if (memcmp(config, "flood.advert.interval", 21) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->flood_advert_interval);
        } else if (memcmp(config, "advert.interval", 15) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", ((uint32_t)_prefs->advert_interval) * 2);
        } else if (memcmp(config, "guest.password", 14) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->guest_password);
        } else if (sender_timestamp == 0 && memcmp(config, "prv.key", 7) == 0) {
            uint8_t prv_key[PRV_KEY_SIZE];
            int len = _callbacks->getSelfId().writeTo(prv_key, PRV_KEY_SIZE);
            mesh::Utils::toHex(tmp, prv_key, len);
            snprintf(reply, CLI_REPLY_SIZE, "> %s", tmp);
        } else if (memcmp(config, "name", 4) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->node_name);
        } else if (memcmp(config, "repeat", 6) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->disable_fwd ? "off" : "on");
        } else if (memcmp(config, "lat", 3) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %.6f", _prefs->node_lat);
        } else if (memcmp(config, "lon", 3) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %.6f", _prefs->node_lon);
        } else if (memcmp(config, "radio.rxgain", 12) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (int)_prefs->rx_boost);
        } else if (memcmp(config, "radio", 5) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %.3f,%.1f,%u,%u",
                   (double)_prefs->freq, (double)_prefs->bw,
                   (uint32_t)_prefs->sf, (uint32_t)_prefs->cr);
        } else if (memcmp(config, "rxdelay", 7) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> adaptive (rxdelay deprecated)");
        } else if (memcmp(config, "txdelay", 7) == 0) {
            float est = _callbacks->getContentionEstimate();
            float ff = _callbacks->getFloodDelayFactor();
            snprintf(reply, CLI_REPLY_SIZE, "> adaptive (est=%.1f flood=%.2f)",
                     (double)est, (double)ff);
        } else if (memcmp(config, "apc.margin", 10) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d dB", (int)_callbacks->getAPCTargetMargin());
        } else if (memcmp(config, "flood.max", 9) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->flood_max);
        } else if (memcmp(config, "direct.txdelay", 14) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> adaptive (direct.txdelay deprecated)");
        } else if (memcmp(config, "backoff.multiplier", 18) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %.2f", (double)_prefs->backoff_multiplier);
        } else if (memcmp(config, "owner.info", 10) == 0) {
            *reply++ = '>';
            *reply++ = ' ';
            const char* sp = _prefs->owner_info;
            while (*sp) {
                *reply++ = (*sp == '\n') ? '|' : *sp;
                sp++;
            }
            *reply = 0;
        } else if (memcmp(config, "path.hash.mode", 14) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (uint32_t)_prefs->path_hash_mode);
        } else if (memcmp(config, "loop.detect", 11) == 0) {
            if (_prefs->loop_detect == LOOP_DETECT_OFF) {
                strcpy(reply, "> off");
            } else if (_prefs->loop_detect == LOOP_DETECT_MINIMAL) {
                strcpy(reply, "> minimal");
            } else if (_prefs->loop_detect == LOOP_DETECT_MODERATE) {
                strcpy(reply, "> moderate");
            } else {
                strcpy(reply, "> strict");
            }
        } else if (memcmp(config, "tx", 2) == 0 && (config[2] == 0 || config[2] == ' ')) {
            if (_callbacks->isAPCEnabled()) {
                int8_t apc = _callbacks->getAPCReduction();
                float margin = _callbacks->getAPCMargin();
                int effective = (int)_prefs->tx_power_dbm - (int)apc;
                snprintf(reply, CLI_REPLY_SIZE, "> %ddBm (max=%d apc=-%d margin=%.1f target=%d)",
                         effective, (int)_prefs->tx_power_dbm, (int)apc, (double)margin,
                         (int)_callbacks->getAPCTargetMargin());
            } else {
                snprintf(reply, CLI_REPLY_SIZE, "> %ddBm (apc=off)",
                         (int)_prefs->tx_power_dbm);
            }
        } else if (memcmp(config, "freq", 4) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %.3f", (double)_prefs->freq);
        } else if (memcmp(config, "public.key", 10) == 0) {
            strcpy(reply, "> ");
            mesh::Utils::toHex(&reply[2], _callbacks->getSelfId().pub_key, PUB_KEY_SIZE);
        } else if (memcmp(config, "role", 4) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _callbacks->getRole());
        } else if (memcmp(config, "bootloader.ver", 14) == 0) {
            char ver[32];
            if (_board->getBootloaderVersion(ver, sizeof(ver))) {
                snprintf(reply, CLI_REPLY_SIZE, "> %s", ver);
            } else {
                strcpy(reply, "> unknown");
            }
        } else if (memcmp(config, "adc.multiplier", 14) == 0) {
            float adc_mult = _board->getAdcMultiplier();
            if (adc_mult == 0.0f) {
                strcpy(reply, "Error: unsupported by this board");
            } else {
                snprintf(reply, CLI_REPLY_SIZE, "> %.3f", (double)adc_mult);
            }
        } else if (memcmp(config, "rxduty", 6) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (int)_prefs->rx_duty_cycle);
        } else if (memcmp(config, "dc.restarts", 11) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u",
                     (uint32_t)_callbacks->getDutyCycleTimeoutRestarts());
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "??: %s", config);
        }
    /*
     * SET commands
     */
    } else if (memcmp(command, "set ", 4) == 0) {
        const char* config = &command[4];
        if (memcmp(config, "dutycycle ", 10) == 0) {
            float dc = atof(&config[10]);
            if (dc < 1 || dc > 100) {
                strcpy(reply, "ERROR: dutycycle must be 1-100");
            } else {
                _prefs->airtime_factor = (100.0f / dc) - 1.0f;
                savePrefs();
                float actual = 100.0f / (_prefs->airtime_factor + 1.0f);
                int a_int = (int)actual;
                int a_frac = (int)((actual - a_int) * 10.0f + 0.5f);
                snprintf(reply, CLI_REPLY_SIZE, "OK - %d.%d%%", a_int, a_frac);
            }
        } else if (memcmp(config, "af ", 3) == 0) {
            _prefs->airtime_factor = atof(&config[3]);
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "int.thresh ", 11) == 0) {
            _prefs->interference_threshold = atoi(&config[11]);
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "agc.reset.interval ", 19) == 0) {
            _prefs->agc_reset_interval = atoi(&config[19]) / 4;
            savePrefs();
            snprintf(reply, CLI_REPLY_SIZE, "OK - interval rounded to %u", ((uint32_t)_prefs->agc_reset_interval) * 4);
        } else if (memcmp(config, "multi.acks ", 11) == 0) {
            int val = atoi(&config[11]);
            if (val == 0 || val == 1) {
                _prefs->multi_acks = (uint8_t)val;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: must be 0 or 1");
            }
        } else if (memcmp(config, "allow.read.only ", 16) == 0) {
            if (memcmp(&config[16], "on", 2) == 0) {
                _prefs->allow_read_only = 1;
                savePrefs();
                strcpy(reply, "OK");
            } else if (memcmp(&config[16], "off", 3) == 0) {
                _prefs->allow_read_only = 0;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: must be on or off");
            }
        } else if (memcmp(config, "flood.advert.interval ", 22) == 0) {
            int hours = _atoi(&config[22]);
            if ((hours > 0 && hours < 3) || (hours > 168)) {
                strcpy(reply, "Error: interval range is 3-168 hours");
            } else {
                _prefs->flood_advert_interval = (uint8_t)hours;
                _callbacks->updateFloodAdvertTimer();
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "advert.interval ", 16) == 0) {
            int mins = _atoi(&config[16]);
            if ((mins > 0 && mins < MIN_LOCAL_ADVERT_INTERVAL) || (mins > 240)) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: interval range is %d-240 minutes", MIN_LOCAL_ADVERT_INTERVAL);
            } else {
                _prefs->advert_interval = (uint8_t)(mins / 2);
                _callbacks->updateAdvertTimer();
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "guest.password ", 15) == 0) {
            StrHelper::strncpy(_prefs->guest_password, &config[15], sizeof(_prefs->guest_password));
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "prv.key ", 8) == 0) {
            uint8_t prv_key[PRV_KEY_SIZE];
            bool success = mesh::Utils::fromHex(prv_key, PRV_KEY_SIZE, &config[8]);
            if (success && mesh::LocalIdentity::validatePrivateKey(prv_key)) {
                mesh::LocalIdentity new_id;
                new_id.readFrom(prv_key, PRV_KEY_SIZE);
                _callbacks->saveIdentity(new_id);
                strcpy(reply, "OK, reboot to apply! New pubkey: ");
                mesh::Utils::toHex(&reply[33], new_id.pub_key, PUB_KEY_SIZE);
            } else {
                strcpy(reply, "Error, bad key");
            }
        } else if (memcmp(config, "name ", 5) == 0) {
            if (isValidName(&config[5])) {
                StrHelper::strncpy(_prefs->node_name, &config[5], sizeof(_prefs->node_name));
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: name cannot contain [ ] \\ : , ? *");
            }
        } else if (memcmp(config, "repeat ", 7) == 0) {
            if (memcmp(&config[7], "on", 2) == 0) {
                _prefs->disable_fwd = 0;
                savePrefs();
                strcpy(reply, "OK - repeat is now ON");
            } else if (memcmp(&config[7], "off", 3) == 0) {
                _prefs->disable_fwd = 1;
                savePrefs();
                strcpy(reply, "OK - repeat is now OFF");
            } else {
                strcpy(reply, "Error: must be on or off");
            }
        } else if (memcmp(config, "radio ", 6) == 0) {
            strcpy(tmp, &config[6]);
            const char* parts[4];
            int num = mesh::Utils::parseTextParts(tmp, parts, 4);
            float freq = num > 0 ? strtof(parts[0], nullptr) : 0.0f;
            float bw = num > 1 ? strtof(parts[1], nullptr) : 0.0f;
            uint8_t sf = num > 2 ? atoi(parts[2]) : 0;
            uint8_t cr = num > 3 ? atoi(parts[3]) : 0;
            if (freq >= 150.0f && freq <= 2500.0f && sf >= 5 && sf <= 12 &&
                cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= 500.0f) {
                _prefs->freq = freq;
                _prefs->bw   = bw;
                _prefs->sf   = sf;
                _prefs->cr   = cr;
                _callbacks->savePrefs();
                strcpy(reply, "OK - reboot to apply");
            } else {
                strcpy(reply, "Error: freq 150-2500, bw 7-500, sf 5-12, cr 5-8");
            }
        } else if (memcmp(config, "lat ", 4) == 0) {
            _prefs->node_lat = atof(&config[4]);
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "lon ", 4) == 0) {
            _prefs->node_lon = atof(&config[4]);
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "rxdelay ", 8) == 0) {
            _prefs->rx_delay_base = atof(&config[8]);
            savePrefs();
            strcpy(reply, "OK (ignored: rxdelay is now adaptive)");
        } else if (memcmp(config, "txdelay ", 8) == 0) {
            _prefs->tx_delay_factor = atof(&config[8]);
            savePrefs();
            strcpy(reply, "OK (ignored: txdelay is now adaptive)");
        } else if (memcmp(config, "flood.max ", 10) == 0) {
            int m = atoi(&config[10]);
            if (m >= 0 && m <= 64) {
                _prefs->flood_max = (uint8_t)m;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: range 0-64");
            }
        } else if (memcmp(config, "direct.txdelay ", 15) == 0) {
            _prefs->direct_tx_delay_factor = atof(&config[15]);
            savePrefs();
            strcpy(reply, "OK (ignored: direct.txdelay is now adaptive)");
        } else if (memcmp(config, "backoff.multiplier ", 19) == 0) {
            float f = atof(&config[19]);
            if (f >= 0.0f && f <= 2.0f) {
                _prefs->backoff_multiplier = f;
                _callbacks->setBackoffMultiplier(f);
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error, range 0.0-2.0");
            }
        } else if (memcmp(config, "owner.info ", 11) == 0) {
            config += 11;
            char* dp = _prefs->owner_info;
            while (*config && dp - _prefs->owner_info < (int)sizeof(_prefs->owner_info) - 1) {
                *dp++ = (*config == '|') ? '\n' : *config;
                config++;
            }
            *dp = 0;
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "path.hash.mode ", 15) == 0) {
            config += 15;
            uint8_t mode = atoi(config);
            if (mode < 3) {
                _prefs->path_hash_mode = mode;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error, must be 0,1, or 2");
            }
        } else if (memcmp(config, "loop.detect ", 12) == 0) {
            config += 12;
            uint8_t mode;
            if (memcmp(config, "off", 3) == 0) {
                mode = LOOP_DETECT_OFF;
            } else if (memcmp(config, "minimal", 7) == 0) {
                mode = LOOP_DETECT_MINIMAL;
            } else if (memcmp(config, "moderate", 8) == 0) {
                mode = LOOP_DETECT_MODERATE;
            } else if (memcmp(config, "strict", 6) == 0) {
                mode = LOOP_DETECT_STRICT;
            } else {
                mode = 0xFF;
                strcpy(reply, "Error, must be: off, minimal, moderate, or strict");
            }
            if (mode != 0xFF) {
                _prefs->loop_detect = mode;
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "apc.margin ", 11) == 0) {
            int val = atoi(&config[11]);
            if (val >= 6 && val <= 30) {
                _prefs->apc_margin = (uint8_t)val;
                _callbacks->setAPCTargetMargin((uint8_t)val);
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - APC target margin=%d dB", val);
            } else {
                strcpy(reply, "Error: range 6-30 dB");
            }
        } else if (memcmp(config, "tx ", 3) == 0) {
            if (memcmp(&config[3], "apc", 3) == 0) {
                _prefs->apc_enabled = 1;
                _callbacks->setAPCEnabled(true);
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - tx power=%d dBm (apc=on)",
                         (int)_prefs->tx_power_dbm);
            } else {
                int val = atoi(&config[3]);
                int max_tx = 30;
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
                max_tx = CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
#endif
                if (val < -9 || val > max_tx) {
                    snprintf(reply, CLI_REPLY_SIZE, "Error: range -9 to %d dBm, or 'apc'", max_tx);
                } else {
                    _prefs->apc_enabled = 0;
                    _prefs->tx_power_dbm = (int8_t)val;
                    savePrefs();
                    _callbacks->setAPCEnabled(false);
                    _callbacks->setTxPower(_prefs->tx_power_dbm);
                    snprintf(reply, CLI_REPLY_SIZE, "OK - tx power=%d dBm (apc=off)",
                             (int)_prefs->tx_power_dbm);
                }
            }
        } else if (sender_timestamp == 0 && memcmp(config, "freq ", 5) == 0) {
            float f = atof(&config[5]);
            if (f >= 150.0f && f <= 2500.0f) {
                float old_freq = _prefs->freq;
                _prefs->freq = f;
                savePrefs();
                _prefs->freq = old_freq;
                strcpy(reply, "OK - reboot to apply");
            } else {
                strcpy(reply, "Error: range 150-2500 MHz");
            }
        } else if (memcmp(config, "adc.multiplier ", 15) == 0) {
            _prefs->adc_multiplier = atof(&config[15]);
            if (_board->setAdcMultiplier(_prefs->adc_multiplier)) {
                savePrefs();
                if (_prefs->adc_multiplier == 0.0f) {
                    strcpy(reply, "OK - using default board multiplier");
                } else {
                    snprintf(reply, CLI_REPLY_SIZE, "OK - multiplier set to %.3f", (double)_prefs->adc_multiplier);
                }
            } else {
                _prefs->adc_multiplier = 0.0f;
                strcpy(reply, "Error: unsupported by this board");
            }
        } else if (memcmp(config, "radio.rxgain ", 13) == 0) {
            int val = atoi(&config[13]);
            if (val == 0 || val == 1) {
                _prefs->rx_boost = (uint8_t)val;
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - radio.rxgain=%d (reboot to apply)", _prefs->rx_boost);
            } else {
                strcpy(reply, "Error: must be 0 or 1");
            }
        } else if (memcmp(config, "rxduty ", 7) == 0) {
            int val = atoi(&config[7]);
            if (val == 0 || val == 1) {
                _prefs->rx_duty_cycle = (uint8_t)val;
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - rxduty=%d (reboot to apply)", _prefs->rx_duty_cycle);
            } else {
                strcpy(reply, "Error: must be 0 or 1");
            }
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "unknown config: %s", config);
        }
    } else if (sender_timestamp == 0 && strcmp(command, "erase") == 0) {
        bool s = _callbacks->formatFileSystem();
        snprintf(reply, CLI_REPLY_SIZE, "File system erase: %s", s ? "OK" : "Err");
    } else if (memcmp(command, "ver", 3) == 0) {
        snprintf(reply, CLI_REPLY_SIZE, "%s (Build: %s)", _callbacks->getFirmwareVer(), _callbacks->getBuildDate());
    } else if (memcmp(command, "board", 5) == 0) {
        snprintf(reply, CLI_REPLY_SIZE, "%s", _board->getManufacturerName());
    } else if (memcmp(command, "sensor get ", 11) == 0) {
        const char* key = command + 11;
        const char* val = _callbacks->getSensorSettingByKey(key);
        if (val != nullptr) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", val);
        } else {
            strcpy(reply, "null");
        }
    } else if (memcmp(command, "sensor set ", 11) == 0) {
        strcpy(tmp, &command[11]);
        const char* parts[2];
        int num = mesh::Utils::parseTextParts(tmp, parts, 2, ' ');
        const char* key = (num > 0) ? parts[0] : "";
        const char* value = (num > 1) ? parts[1] : "null";
        if (_callbacks->setSensorSettingValue(key, value)) {
            strcpy(reply, "ok");
        } else {
            strcpy(reply, "can't find custom var");
        }
    } else if (memcmp(command, "sensor list", 11) == 0) {
        char* dp = reply;
        int start = 0;
        int end = _callbacks->getNumSensorSettings();
        if (strlen(command) > 11) {
            start = _atoi(command + 12);
        }
        if (start >= end) {
            strcpy(reply, "no custom var");
        } else {
            snprintf(dp, CLI_REPLY_SIZE - (dp - reply), "%d vars\n", end);
            dp = strchr(dp, 0);
            int i;
            for (i = start; i < end && (dp - reply < 134); i++) {
                snprintf(dp, CLI_REPLY_SIZE - (dp - reply), "%s=%s\n",
                    _callbacks->getSensorSettingName(i),
                    _callbacks->getSensorSettingValue(i));
                dp = strchr(dp, 0);
            }
            if (i < end) {
                snprintf(dp, CLI_REPLY_SIZE - (dp - reply), "... next:%d", i);
            } else {
                *(dp - 1) = 0;  // remove last CR
            }
        }
    } else if (memcmp(command, "gps on", 6) == 0) {
        if (_callbacks->setGpsEnabled(true)) {
            _prefs->gps_enabled = 1;
            savePrefs();
            strcpy(reply, "ok");
        } else {
            strcpy(reply, "gps toggle not found");
        }
    } else if (memcmp(command, "gps off", 7) == 0) {
        if (_callbacks->setGpsEnabled(false)) {
            _prefs->gps_enabled = 0;
            savePrefs();
            strcpy(reply, "ok");
        } else {
            strcpy(reply, "gps toggle not found");
        }
    } else if (memcmp(command, "gps setloc", 10) == 0) {
        _prefs->node_lat = _callbacks->getNodeLat();
        _prefs->node_lon = _callbacks->getNodeLon();
        savePrefs();
        strcpy(reply, "ok");
    } else if (memcmp(command, "gps advert", 10) == 0) {
        if (strlen(command) == 10) {
            switch (_prefs->advert_loc_policy) {
                case ADVERT_LOC_NONE:  strcpy(reply, "> none"); break;
                case ADVERT_LOC_PREFS: strcpy(reply, "> prefs"); break;
                case ADVERT_LOC_SHARE: strcpy(reply, "> share"); break;
                default: strcpy(reply, "error");
            }
        } else if (memcmp(command + 11, "none", 4) == 0) {
            _prefs->advert_loc_policy = ADVERT_LOC_NONE;
            savePrefs();
            strcpy(reply, "ok");
        } else if (memcmp(command + 11, "share", 5) == 0) {
            _prefs->advert_loc_policy = ADVERT_LOC_SHARE;
            savePrefs();
            strcpy(reply, "ok");
        } else if (memcmp(command + 11, "prefs", 5) == 0) {
            _prefs->advert_loc_policy = ADVERT_LOC_PREFS;
            savePrefs();
            strcpy(reply, "ok");
        } else {
            strcpy(reply, "error");
        }
    } else if (memcmp(command, "gps", 3) == 0) {
        _callbacks->formatGpsStatsReply(reply);
    } else if (memcmp(command, "powersaving", 11) == 0) {
        strcpy(reply, "Not implemented");
    } else if (memcmp(command, "log start", 9) == 0) {
        _callbacks->setLoggingOn(true);
        strcpy(reply, "   logging on");
    } else if (memcmp(command, "log stop", 8) == 0) {
        _callbacks->setLoggingOn(false);
        strcpy(reply, "   logging off");
    } else if (memcmp(command, "log erase", 9) == 0) {
        _callbacks->eraseLogFile();
        strcpy(reply, "   log erased");
    } else if (sender_timestamp == 0 && memcmp(command, "log", 3) == 0) {
        _callbacks->dumpLogFile();
        strcpy(reply, "   EOF");
    } else if (sender_timestamp == 0 && memcmp(command, "stats-packets", 13) == 0 &&
               (command[13] == 0 || command[13] == ' ')) {
        _callbacks->formatPacketStatsReply(reply);
    } else if (sender_timestamp == 0 && memcmp(command, "stats-radio", 11) == 0 &&
               (command[11] == 0 || command[11] == ' ')) {
        _callbacks->formatRadioStatsReply(reply);
    } else if (sender_timestamp == 0 && memcmp(command, "stats-core", 10) == 0 &&
               (command[10] == 0 || command[10] == ' ')) {
        _callbacks->formatStatsReply(reply);
    } else {
        strcpy(reply, "Unknown command");
    }
}
