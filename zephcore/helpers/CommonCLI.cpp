/*
 * SPDX-License-Identifier: MIT
 * CommonCLI - Common CLI command handlers for repeaters
 */

#include "CommonCLI.h"
#include "battery_curve.h"
#include "led_gate.h"
#include "buzzer_gate.h"
#include <helpers/ui/ui_task.h>
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
#include <helpers/ui/display.h>
#endif
#include <helpers/MeshTimeSync.h>
#include <helpers/time_sync.h>
#include <adapters/clock/ZephyrRTCDiscover.h>
#include <helpers/TxtDataHelpers.h>
#include <helpers/AdvertDataHelpers.h>
#include <adapters/board/ZephyrBoard.h>
#include <adapters/gps/ZephyrGPSManager.h>
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

/* ---- "default" keyword + strict numeric parsing for the `set` path ----
 *
 * Bare atoi()/atof() fold every non-numeric string to 0, the word "default"
 * included.  On the knobs where 0 is itself legal and means "off" —
 * probe.interval, cad.busycap, flood.max, rxduty — that silently switched the
 * feature off and still answered OK.  `set probe.interval default` disabling
 * probing is the report that prompted this.
 *
 * Every `set` that has a default now takes the literal `default`, and rejects
 * trailing garbage rather than turning it into a zero. */

/* The defaults are read out of initNodePrefs() itself rather than restated as
 * constants here, so `set <x> default` cannot drift from what a factory-fresh
 * node actually boots with.  File-scope statics (not a function-local one) to
 * avoid emitting a __cxa_guard for the lazy init. */
static NodePrefs s_cli_defaults;
static bool s_cli_defaults_ready = false;

static const NodePrefs* cliDefaults() {
    if (!s_cli_defaults_ready) {
        initNodePrefs(&s_cli_defaults);
        s_cli_defaults_ready = true;
    }
    return &s_cli_defaults;
}

/* ---- leds.radio / leds.hb mode names ----------------------------------- */
/*
 * Which LEDs this board actually has, so the CLI can tell the user when a
 * setting it just accepted will not do anything here.  The pref is still
 * stored either way: the same prefs file follows a node onto a board that does
 * have the LED, and silently dropping the value would be worse than storing a
 * setting that is dormant.
 *
 * These mirror the aliases the drivers use — lora-tx-led in ZephyrBoard.cpp,
 * led0 with an led1 fallback in helpers/ui/ui_common.c.
 */
#define CLI_HAS_RADIO_LED  DT_NODE_EXISTS(DT_ALIAS(lora_tx_led))
#define CLI_HAS_HB_LED     (DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios) || \
                            DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios))

struct CliModeName { const char* name; uint8_t val; };

static const CliModeName LEDS_RADIO_NAMES[] = {
    { "tx",  LEDS_RADIO_TX  },
    { "rx",  LEDS_RADIO_RX  },
    { "all", LEDS_RADIO_ALL },
    { "off", LEDS_RADIO_OFF },
};

static const CliModeName LEDS_HB_NAMES[] = {
    { "all",    LEDS_HB_ALL    },
    { "hb",     LEDS_HB_HB     },
    { "unread", LEDS_HB_UNREAD },
    { "off",    LEDS_HB_OFF    },
};

static const char* cliModeName(const CliModeName* tbl, size_t n, uint8_t v) {
    for (size_t i = 0; i < n; i++) {
        if (tbl[i].val == v) return tbl[i].name;
    }
    return "?";
}

/* Returns the mode value, or -1 if the argument matches nothing. "default"
 * resolves through cliDefaults() like every other setting. */
static int cliModeValue(const CliModeName* tbl, size_t n, const char* arg, uint8_t def_val);

static const char* cliSkipSpace(const char* s) {
    while (*s == ' ') s++;
    return s;
}

static bool cliIsDefault(const char* arg) {
    const char* s = cliSkipSpace(arg);
    if (strncmp(s, "default", 7) != 0) return false;
    return *cliSkipSpace(s + 7) == '\0';
}

static int cliModeValue(const CliModeName* tbl, size_t n, const char* arg, uint8_t def_val) {
    const char* s = cliSkipSpace(arg);
    if (cliIsDefault(s)) return (int)def_val;
    for (size_t i = 0; i < n; i++) {
        if (strcmp(s, tbl[i].name) == 0) return (int)tbl[i].val;
    }
    return -1;
}

/* Integer argument, or "default".  false = unparseable; the caller reports
 * usage rather than acting on a silently-zeroed value. */
static bool cliNum(const char* arg, long def_val, long* out) {
    const char* s = cliSkipSpace(arg);
    if (cliIsDefault(s)) { *out = def_val; return true; }
    char* end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *cliSkipSpace(end) != '\0') return false;
    *out = v;
    return true;
}

/* Float argument, or "default".  NaN is rejected here so range checks in the
 * callers (every comparison against NaN is false) cannot pass it through. */
static bool cliFloat(const char* arg, float def_val, float* out) {
    const char* s = cliSkipSpace(arg);
    if (cliIsDefault(s)) { *out = def_val; return true; }
    char* end = NULL;
    float v = strtof(s, &end);
    if (end == s || *cliSkipSpace(end) != '\0') return false;
    if (v != v) return false;
    *out = v;
    return true;
}

/* on / off / 1 / 0 / default -> 1 or 0; -1 when unrecognised.  Replaces the
 * hand-rolled copy of this ladder in a dozen setters, each of which accepted a
 * bare leading '0'/'1' and ignored whatever followed. */
static int cliOnOff(const char* arg, int def_val) {
    const char* s = cliSkipSpace(arg);
    if (cliIsDefault(s)) return def_val;
    if (strcmp(s, "on") == 0 || strcmp(s, "1") == 0) return 1;
    if (strcmp(s, "off") == 0 || strcmp(s, "0") == 0) return 0;
    return -1;
}

static bool isValidName(const char* n) {
    while (*n) {
        if (*n == '[' || *n == ']' || *n == '\\' || *n == ':' ||
            *n == ',' || *n == '?' || *n == '*') return false;
        n++;
    }
    return true;
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

/* How long past the initial delay we keep waiting for the transport to drain,
 * and how often we look.  The poll MUST re-schedule rather than sleep: this
 * handler runs on the system work queue, which is the same queue that drains
 * the BLE TX ring — blocking here would stall the very thing being waited on
 * and guarantee the timeout. */
#define REBOOT_TX_DRAIN_GRACE_MS 3000
#define REBOOT_TX_POLL_MS          20

void CommonCLI::rebootWorkHandler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    CommonCLI *self = CONTAINER_OF(dwork, CommonCLI, _reboot_work);

    /* Hold the reset until the companion app has actually been told what
     * happened — the CLI reply and, more importantly, the delivery-ack for the
     * command that asked for this reboot.  Losing that ack is what makes the
     * app resend the command, and a resend of "reboot" arrives after RAM has
     * been zeroed, so the v-contact dedup ring cannot recognise it and the
     * command runs a second time. */
    if (!self->_callbacks->transportTxIdle() &&
        k_uptime_get() < self->_reboot_deadline_ms) {
        k_work_reschedule(&self->_reboot_work, K_MSEC(REBOOT_TX_POLL_MS));
        return;
    }

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
    /* 2 second delay - enough for LoRa reply to be transmitted.  On a
     * companion the handler then keeps deferring in REBOOT_TX_POLL_MS steps
     * until the BLE/USB transport has drained, up to the grace below. */
    _reboot_deadline_ms = k_uptime_get() + 2000 + REBOOT_TX_DRAIN_GRACE_MS;
    k_work_schedule(&_reboot_work, K_SECONDS(2));
}

/* CLI commands are case-sensitive, matching upstream Arduino MeshCore.
 *
 * A case-insensitive normalizer lived here from 2026-07-12 until 2026-07-19.
 * It lowercased the first two whitespace-delimited tokens before matching, on
 * the assumption that a value never appears before the third token.  That is
 * false for "password <value>", whose value IS token 1 -- so any admin
 * password containing uppercase was silently stored folded to lowercase and
 * could never be used to log in again.  ("set guest.password <value>" was
 * unaffected: three tokens.)
 *
 * Do not reintroduce input folding here.  Any scheme that rewrites the buffer
 * before dispatch has to guess where keywords end and arguments begin, and
 * that guess is what broke.  If case-insensitivity is wanted again, do it at
 * the comparison sites so argument bytes are never touched.
 */
void CommonCLI::handleCommand(uint32_t sender_timestamp, const char* command, char* reply) {
    if (strcmp(command, "start dfu") == 0) {
        /* Reboot into the chip's own firmware-update mode.  nRF52: the Adafruit
         * UF2 bootloader.  ESP32-S3: the ROM download mode, which is the only
         * way to get the USB port back from the CDC companion transport for
         * esptool (see ZephyrBoard::rebootToBootloader).
         *
         * Everything else has no bootloader this command can reach, so it must
         * NOT reboot.  rebootToBootloader() would just reset back into the app,
         * dropping the node off the mesh for nothing while replying that it had
         * gone somewhere it cannot go.  C3/C6 and classic ESP32 do not need it
         * anyway -- they have no DWC2 controller, so USB-Serial-JTAG never
         * loses the port and esptool resets them itself; nRF54L15/MG24/STM32WL
         * have no USB device peripheral at all and are flashed over SWD. */
#if defined(CONFIG_SOC_SERIES_ESP32S3)
        strcpy(reply, "OK - rebooting to ESP32 download mode");
        scheduleReboot(REBOOT_DFU);
#elif defined(CONFIG_SOC_SERIES_NRF52)
        strcpy(reply, "OK - rebooting to UF2 DFU");
        scheduleReboot(REBOOT_DFU);
#elif defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
        strcpy(reply, "Err: no DFU mode here - esptool resets this chip itself");
#else
        strcpy(reply, "Err: no DFU bootloader on this chip - flash over SWD");
#endif
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
        /* Deferred like every other reboot path: called inline this reset the
         * board before the reply — and before any delivery-ack — could leave
         * the TX queue. */
        strcpy(reply, "OK - clock reset, rebooting");
        scheduleReboot(REBOOT_NORMAL);
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
            time_sync_report(TIME_SYNC_CLI);
            zephcore_rtc_save(sender_timestamp + 1);  /* persist to hardware RTC */
            MeshTimeSync* ts = _callbacks->getMeshTimeSync();
            if (ts) ts->noteManualSync((uint32_t)(k_uptime_get() / 1000));
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
        snprintf(reply, CLI_REPLY_SIZE, "%02d:%02d - %d/%d/%d UTC",
                 tm->tm_hour, tm->tm_min, tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900);
    } else if (memcmp(command, "time ", 5) == 0) {
        uint32_t secs = _atoi(&command[5]);
        uint32_t curr = getRTCClock()->getCurrentTime();
        if (secs > curr) {
            getRTCClock()->setCurrentTime(secs);
            time_sync_report(TIME_SYNC_CLI);
            zephcore_rtc_save(secs);  /* persist to hardware RTC */
            MeshTimeSync* ts = _callbacks->getMeshTimeSync();
            if (ts) ts->noteManualSync((uint32_t)(k_uptime_get() / 1000));
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
        snprintf(tmp, sizeof(tmp), "%.*s", (int)(sizeof(tmp) - 1), &command[10]);
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
        StrHelper::strzcpy(_prefs->password, &command[9], sizeof(_prefs->password));
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
        /* MUST stay above the "leds" branch: that one compares only the first
         * four characters, so "leds.radio"/"leds.hb"/"leds.brightness" all
         * match it and would otherwise return the master switch instead. */
        } else if (memcmp(config, "leds.radio", 10) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s%s",
                     cliModeName(LEDS_RADIO_NAMES, 4, _prefs->leds_radio_mode),
                     CLI_HAS_RADIO_LED ? "" : " (no radio LED on this board)");
        } else if (memcmp(config, "leds.hb", 7) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s%s",
                     cliModeName(LEDS_HB_NAMES, 4, _prefs->leds_hb_mode),
                     CLI_HAS_HB_LED ? "" : " (no heartbeat LED on this board)");
        } else if (memcmp(config, "leds.brightness", 15) == 0) {
            /* Separate from "leds" on purpose: keeps the on/off switch a
             * pure word command and the dimmer a pure number command,
             * instead of one command parsing both — see led_gate.h. Read
             * from _prefs, same as leds.radio/leds.hb above, not from the
             * RAM getter directly -- the two are always in sync, the "set"
             * handler below writes both together. */
            snprintf(reply, CLI_REPLY_SIZE, "> %u%%", (unsigned)_prefs->led_brightness);
        } else if (memcmp(config, "leds", 4) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->leds_disabled ? "off" : "on");
#ifndef ZEPHCORE_REPEATER
        } else if (memcmp(config, "buzzer", 6) == 0) {
            uint8_t mode = zephcore_buzzer_mode_from_prefs(_prefs->buzzer_quiet);
            snprintf(reply, CLI_REPLY_SIZE, "> %u (%s)", mode,
                     zephcore_buzzer_mode_name(mode));
#endif
        } else if (memcmp(config, "agc.reset.interval", 18) == 0) {
            strcpy(reply, "Removed - Automatic AGC reset is on");
        } else if (memcmp(config, "multi.acks", 10) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->multi_acks);
#ifdef CONFIG_ZEPHCORE_ROLE_ROOM_SERVER
        /* Room server only.  RoomServerMesh.cpp is the sole consumer of
         * allow_read_only; RepeaterMesh.cpp never reads it, so on a repeater
         * this advertised a setting that silently did nothing.  Note this is a
         * deliberate divergence from Arduino MeshCore, whose shared CommonCLI
         * exposes the knob on every role for the same reason ours used to.
         *
         * The pref itself stays unconditional -- it is byte 114 of the on-flash
         * prefs layout, identical to Arduino's, so dropping it would shift every
         * field after it and invalidate existing prefs files. */
        } else if (memcmp(config, "allow.read.only", 15) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->allow_read_only ? "on" : "off");
#endif
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
        } else if (memcmp(config, "radio.fem.rxgain", 16) == 0) {
            /* on/off, not 0/1 -- MeshCore apps parse this as a boolean word. */
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->fem_rxgain ? "on" : "off");
        } else if (memcmp(config, "radio.rxgain", 12) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _prefs->rx_boost ? "on" : "off");
        } else if (memcmp(config, "radio", 5) == 0) {
            /* Arduino renders bw with a trailing-zero-stripped formatter, so
             * 250 kHz prints as "250", not "250.0".  Match it. */
            char bw[16];
            snprintf(bw, sizeof(bw), "%.3f", (double)_prefs->bw);
            StrHelper::stripTrailingZeros(bw);
            snprintf(reply, CLI_REPLY_SIZE, "> %.3f,%s,%u,%u",
                   (double)_prefs->freq, bw,
                   (uint32_t)_prefs->sf, (uint32_t)_prefs->cr);
        } else if (memcmp(config, "rxdelay", 7) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> adaptive (rxdelay deprecated)");
        } else if (memcmp(config, "txdelay", 7) == 0) {
            float est = _callbacks->getContentionEstimate();
            float ff = _callbacks->getFloodDelayFactor();
            snprintf(reply, CLI_REPLY_SIZE, "> adaptive (est=%.1f flood=%.2f)",
                     (double)est, (double)ff);
        } else if (memcmp(config, "flood.max.advert", 16) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->flood_max_advert);
        } else if (memcmp(config, "flood.max.unscoped", 18) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (uint32_t)_prefs->flood_max_unscoped);
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
        } else if (strcmp(config, "tx") == 0) {
            /* Plain number, matching upstream Arduino MeshCore's "> %d". */
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (int)_prefs->tx_power_dbm);
        } else if (memcmp(config, "freqerr", 7) == 0) {
            /* MUST stay above "freq" — that is a 4-char prefix match and
             * would swallow this one.
             *
             * Carrier frequency error measured on received packets, LR2021
             * only.  Diagnostic: nothing acts on it.  The mean approximates
             * THIS node's reference error only once averaged over many
             * different peers (theirs cancel, ours does not), which is why
             * the spread and packet count are shown alongside it. */
            int n = snprintf(reply, CLI_REPLY_SIZE, "> ");
            if (_callbacks->formatFreqErrorStatus(reply + n,
                                                  CLI_REPLY_SIZE - n) == 0) {
                strcpy(reply, "not available");
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
                uint16_t mv = _board->getBattMilliVolts();
                uint16_t target_mv = battery_curve_default.ocv_mv[0];
                if (mv > 0) {
                    snprintf(reply, CLI_REPLY_SIZE, "> %.3f  (%u mV, target >= %u mV for 100%%)",
                             (double)adc_mult, mv, target_mv);
                } else {
                    snprintf(reply, CLI_REPLY_SIZE, "> %.3f  (no ADC reading)", (double)adc_mult);
                }
            }
        } else if (memcmp(config, "rxduty", 6) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (int)_prefs->rx_duty_cycle);
        } else if (memcmp(config, "display.rotate", 14) == 0) {
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY) && MC_DISPLAY_ROTATE_SUPPORTED
            // Report the live panel state, not the stored byte: they only
            // differ if a rotation was refused, and that is exactly the case
            // worth seeing.
            snprintf(reply, CLI_REPLY_SIZE, "> %d", mc_display_is_rotated() ? 1 : 0);
#else
            strcpy(reply, "> unsupported (panel cannot rotate)");
#endif
        } else if (memcmp(config, "input.rotate", 12) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", zephcore_input_is_flipped() ? 1 : 0);
        } else if (memcmp(config, "tz.offset", 9) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (int)_prefs->tz_offset);
        } else if (memcmp(config, "gps diag", 8) == 0) {
            // What the last module-configuration attempt actually did.
            reply[0] = '>'; reply[1] = ' ';
            gps_get_diag_report(reply + 2, CLI_REPLY_SIZE - 2);
        } else if (memcmp(config, "gps duty", 8) == 0) {
            uint32_t s = gps_get_poll_interval_sec();  // now-effective value
            if (s == 0) strcpy(reply, "> always on (0)");
            else snprintf(reply, CLI_REPLY_SIZE, "> %u", (unsigned)s);
        } else if (memcmp(config, "dc.restarts", 11) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u",
                     (uint32_t)_callbacks->getDutyCycleTimeoutRestarts());
        } else if (memcmp(config, "probe.interval", 14) == 0) {
            /* Seconds between periodic radio measurements — the noise-floor
             * sample and the CAD probe that consumes it.  0 = probing off. */
            snprintf(reply, CLI_REPLY_SIZE, "> %u",
                     (uint32_t)_prefs->probe_interval);
        } else if (memcmp(config, "cad.stats", 9) == 0) {
            /* Runtime state + per-level probe stats live in the radio.
             * ZephCore-only; must stay ahead of the "cad" prefix match below.
             * Remote replies get the truncated buffer like meshtimesync. */
            size_t cap = (sender_timestamp == 0) ? CLI_REPLY_SIZE
                                                 : CLI_REMOTE_REPLY_SIZE;
            int n = snprintf(reply, cap, "> ");
            if (_callbacks->formatCadStatus(reply + n, (int)cap - n) == 0) {
                strcpy(reply, "not available");
            }
        } else if (memcmp(config, "cad.auto", 8) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s",
                     _prefs->cad_auto ? "on" : "off");
        } else if (memcmp(config, "cad.offset", 10) == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %d", (int)_prefs->cad_offset);
        } else if (memcmp(config, "cad.busycap", 11) == 0) {
            if (_prefs->cad_busycap == 0) {
                strcpy(reply, "> 0 (off)");
            } else {
                snprintf(reply, CLI_REPLY_SIZE, "> %u",
                         (unsigned)_prefs->cad_busycap);
            }
        } else if (strcmp(config, "cad") == 0) {
            /* Arduino exposes this as a boolean knob; ZephCore always does CAD,
             * so the answer is a constant "on".  Apps parse the word.
             *
             * EXACT match, not the 3-byte prefix this used to be.  Every branch
             * in this chain is a memcmp prefix test, so "cad" matched every
             * `get cad.*` that had no branch of its own and answered "on" for
             * all of them -- a percentage, a signed offset and an action all
             * came back as a boolean.  cad.stats escaped only by being ordered
             * above it, which is a fragile way to stay correct.  Anchoring this
             * one fixes the whole family at once: anything under cad. without a
             * getter now falls through to the honest "??:" reply instead of
             * being answered wrongly. */
            strcpy(reply, "> on");
        } else if (memcmp(config, "extra.sf", 8) == 0) {
            /* No "> " prefix, and the empty case is a sentence -- both match
             * Arduino MeshCore exactly. */
            char* dp = reply;
            int shown = 0;
            for (int i = 0; i < EXTRA_SF_MAX && _prefs->extra_sf[i] != 0; i++) {
                dp += sprintf(dp, "%s%u", shown++ ? "," : "",
                              (unsigned)_prefs->extra_sf[i]);
            }
            if (shown == 0) {
                strcpy(reply, "No extra SF configured");
            }
        } else if (memcmp(config, "meshtimesync", 12) == 0) {
            MeshTimeSync* ts = _callbacks->getMeshTimeSync();
            if (ts == nullptr) {
                strcpy(reply, "not available");
            } else {
                /* Only the local USB CLI (sender_timestamp == 0) gets the
                 * full evidence table; remote replies are truncated to the
                 * packet buffer. */
                size_t cap = (sender_timestamp == 0) ? CLI_REPLY_SIZE
                                                     : CLI_REMOTE_REPLY_SIZE;
                ts->formatStatus(reply, cap, getRTCClock()->getCurrentTime(),
                                 (uint32_t)(k_uptime_get() / 1000),
                                 _prefs->meshtimesync != 0);
            }
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "??: %s", config);
        }
    /*
     * SET commands
     */
    } else if (memcmp(command, "set ", 4) == 0) {
        const char* config = &command[4];
        if (memcmp(config, "dutycycle ", 10) == 0) {
            float dc;
            if (!cliFloat(&config[10], 100.0f / (cliDefaults()->airtime_factor + 1.0f), &dc)) {
                strcpy(reply, "ERROR: dutycycle must be 1-100, or default");
            } else if (dc < 1 || dc > 100) {
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
            float af;
            if (!cliFloat(&config[3], cliDefaults()->airtime_factor, &af)) {
                strcpy(reply, "Error: expected a number or default");
            } else {
                _prefs->airtime_factor = af;
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "int.thresh ", 11) == 0) {
            /* Companion runtime never reads this (getInterferenceThreshold is
             * only overridden in Repeater/RoomServer) — reject instead of a
             * false OK. */
            if (strcmp(_callbacks->getRole(), "companion") == 0) {
                strcpy(reply, "Error: not supported on companion");
            } else {
                long v;
                if (!cliNum(&config[11], cliDefaults()->interference_threshold, &v)) {
                    strcpy(reply, "Error: expected a number or default");
                } else {
                    _prefs->interference_threshold = (uint8_t)v;
                    savePrefs();
                    strcpy(reply, "OK");
                }
            }
        } else if (memcmp(config, "leds.radio ", 11) == 0) {
            /* What the LoRa activity LED reacts to. Below the master switch:
             * "set leds off" keeps it dark whatever this says. */
            int m = cliModeValue(LEDS_RADIO_NAMES, 4, &config[11],
                                 cliDefaults()->leds_radio_mode);
            if (m < 0) {
                strcpy(reply, "Error: must be tx, rx, all, off or default");
            } else {
                _prefs->leds_radio_mode = (uint8_t)m;
                zephcore_leds_set_radio_mode((uint8_t)m);
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK%s",
                         CLI_HAS_RADIO_LED ? "" : " (no radio LED on this board)");
            }
        } else if (memcmp(config, "leds.hb ", 8) == 0) {
            /* What the heartbeat LED reacts to. "unread" is the same cycle
             * widening its pulse, not a separate blink — see led_gate.h. */
            int m = cliModeValue(LEDS_HB_NAMES, 4, &config[8],
                                 cliDefaults()->leds_hb_mode);
            if (m < 0) {
                strcpy(reply, "Error: must be hb, unread, all, off or default");
            } else {
                _prefs->leds_hb_mode = (uint8_t)m;
                zephcore_leds_set_hb_mode((uint8_t)m);
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK%s",
                         CLI_HAS_HB_LED ? "" : " (no heartbeat LED on this board)");
            }
        } else if (memcmp(config, "leds.brightness ", 16) == 0) {
            /* Separate from "leds" on purpose: keeps the on/off switch a pure
             * word command and the dimmer a pure number command, instead of
             * one command parsing both. Persisted (since 1.17.4): a fresh
             * node still starts at ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT (the
             * append-only prefs format leaves this byte at the
             * initNodePrefs() default on any file that predates this field),
             * but once set it survives reboots and firmware updates like
             * leds.radio/leds.hb above. Checked before "leds " below so this
             * longer, more specific prefix is never shadowed by it. */
            const char *val = &config[16];
            char *endptr = (char *)val;
            long pct = strtol(val, &endptr, 10);
            bool trailing_pct = (*endptr == '%' && *(endptr + 1) == '\0');
            if (endptr == val || (*endptr != '\0' && !trailing_pct) ||
                pct < 0 || pct > 100) {
                strcpy(reply, "Error: must be 0-100");
            } else {
                _prefs->led_brightness = (uint8_t)pct;
                zephcore_led_set_brightness_pct((uint8_t)pct);
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - leds.brightness=%ld%%", pct);
            }
        } else if (memcmp(config, "leds ", 5) == 0) {
            /* Master switch (persisted) for every LED on the node: heartbeat,
             * unread-message and LoRa TX activity, plus the message and
             * shutdown flashes. Not the display backlight — that has its own
             * UI brightness setting. "on"/"off" words only, no "1"/"0"
             * aliases (removed from led.tx/led.hb on 2026-08-16 for the same
             * reason: they used to collide with the 1% and 0% brightness
             * values on that branch — moot here since brightness now lives
             * under "leds.brightness" instead, but kept out anyway for
             * consistency and to reject typos cleanly). */
            const char* val = &config[5];
            if (memcmp(val, "on", 2) == 0 && val[2] == '\0') {
                _prefs->leds_disabled = 0;
                zephcore_leds_set_disabled(false);
                savePrefs();
                strcpy(reply, "OK");
            } else if (memcmp(val, "off", 3) == 0 && val[3] == '\0') {
                _prefs->leds_disabled = 1;
                zephcore_leds_set_disabled(true);
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: must be on or off");
            }
#ifndef ZEPHCORE_REPEATER
        } else if (memcmp(config, "buzzer ", 7) == 0) {
            /* 0 = silent, 1 = sound + vibration, 2 = vibration only,
             * 3 = sound only. Modes 2 and 3 only mean something on a board
             * with a vibration motor, which most boards don't have. */
            const char* val = &config[7];
            int mode;
            if (memcmp(val, "vibrate", 7) == 0 || val[0] == '2') {
                mode = ZEPHCORE_BUZZER_VIBRATE;
            } else if (memcmp(val, "sound", 5) == 0 || val[0] == '3') {
                mode = ZEPHCORE_BUZZER_SOUND;
            } else if (memcmp(val, "on", 2) == 0 || val[0] == '1') {
                mode = ZEPHCORE_BUZZER_ON;
            } else if (memcmp(val, "off", 3) == 0 || val[0] == '0') {
                mode = ZEPHCORE_BUZZER_OFF;
            } else if (cliIsDefault(val)) {
                mode = zephcore_buzzer_mode_from_prefs(cliDefaults()->buzzer_quiet);
            } else {
                mode = -1;
            }
            if (mode < 0) {
                strcpy(reply, "Error: 0 (silent), 1 (sound+vib), 2 (vibrate), 3 (sound) or default");
            } else if ((mode == ZEPHCORE_BUZZER_VIBRATE || mode == ZEPHCORE_BUZZER_SOUND) &&
                       !zephcore_buzzer_has_vibrate()) {
                strcpy(reply, "Error: no vibration motor on this board - use 0 or 1");
            } else {
                _prefs->buzzer_quiet = zephcore_buzzer_prefs_from_mode((uint8_t)mode);
                zephcore_buzzer_set_mode((uint8_t)mode, false);
                ui_set_buzzer_mode((uint8_t)mode);
                savePrefs();
                strcpy(reply, "OK");
            }
#endif
        } else if (memcmp(config, "agc.reset.interval ", 19) == 0) {
            /* Periodic AGC recalibration was removed: it reset the noise floor
             * to its unseeded sentinel on every fire, forcing a fresh seed and
             * a full EMA warmup, and it was already forced off under RX duty
             * cycle.  RX duty cycle is the supported way to cut RX current.
             * The prefs BYTE is retained (read/written, never acted on) — the
             * on-disk layout is byte-exact and shifting it would corrupt every
             * existing node's prefs. */
            strcpy(reply, "Removed - Automatic AGC reset is on");
        } else if (memcmp(config, "cad.auto ", 9) == 0) {
            int on = cliOnOff(&config[9], cliDefaults()->cad_auto);
            if (on >= 0) {
                _prefs->cad_auto = (uint8_t)on;
                _callbacks->applyCadPrefs();
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: must be on, off or default");
            }
        } else if (memcmp(config, "cad.offset ", 11) == 0) {
            long val;
            if (!cliNum(&config[11], cliDefaults()->cad_offset, &val)) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: expected %d..%d or default",
                         CAD_OFFSET_MIN, CAD_OFFSET_MAX);
            } else if (val < CAD_OFFSET_MIN || val > CAD_OFFSET_MAX) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: offset range is %d..%d",
                         CAD_OFFSET_MIN, CAD_OFFSET_MAX);
            } else {
                _prefs->cad_offset = (int8_t)val;
                _callbacks->applyCadPrefs();
                savePrefs();
                strcpy(reply, "OK");
            }
        /* Governs every periodic radio measurement, not just CAD — the
         * noise-floor sampler and the CAD probe share one reading. */
        } else if (memcmp(config, "probe.interval ", 15) == 0) {
            long val;
            if (!cliNum(&config[15], cliDefaults()->probe_interval, &val)) {
                strcpy(reply, "Error: interval is 0 (probing off), 10-255 seconds, or default");
            } else if (val != 0 && (val < 10 || val > 255)) {
                strcpy(reply, "Error: interval is 0 (probing off) or 10-255 seconds");
            } else {
                _prefs->probe_interval = (uint8_t)val;
                _callbacks->applyCadPrefs();
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "cad.busycap ", 12) == 0) {
            long val;
            if (!cliNum(&config[12], cliDefaults()->cad_busycap, &val)) {
                strcpy(reply, "Error: busycap is 0 (off), 10-90 percent, or default");
            } else if (val != 0 && (val < 10 || val > 90)) {
                strcpy(reply, "Error: busycap is 0 (off) or 10-90 percent");
            } else {
                _prefs->cad_busycap = (uint8_t)val;
                _callbacks->applyCadPrefs();
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "cad.reset", 9) == 0) {
            /* Clearing the probe statistics alone left the node re-converging
             * FROM wherever the staircase had already walked detPeak, with no
             * evidence left to justify sitting there — the opposite of a reset,
             * and useless for the one job this command has (recovering after a
             * base-table change).  The operating offset lives in two places:
             * _prefs->cad_offset, and _cad_offset inside the radio, which
             * applyCadPrefs() reloads through setCadParams(). */
            _prefs->cad_offset = cliDefaults()->cad_offset;
            /* Clear the recorded base too.  applyCadPrefs() below re-stamps it
             * from the radio, so a reset always leaves offset and base
             * describing the same configuration -- leaving a stale base here
             * would make the NEXT base-table change re-anchor a freshly reset
             * offset away from zero. */
            _prefs->cad_base = 0;
            _callbacks->resetCadStats();
            _callbacks->applyCadPrefs();
            savePrefs();
            snprintf(reply, CLI_REPLY_SIZE,
                     "OK - CAD probe stats cleared, detPeak offset reset to %d",
                     (int)_prefs->cad_offset);
        } else if (memcmp(config, "extra.sf ", 9) == 0) {
            /* LR2021 side detectors: up to 3 extra SFs received alongside
             * `sf`.  "0" / "off" clears the set.  The chip-side constraints
             * (each > sf, distinct, spread <= 4, BW>=500 caps the count) are
             * enforced in the driver, so an accepted set is a valid one. */
            char tmp[32];
            const char* parts[EXTRA_SF_MAX + 1];
            uint8_t sfs[EXTRA_SF_MAX] = {0};
            StrHelper::strncpy(tmp, &config[9], sizeof(tmp));
            int num = mesh::Utils::parseTextParts(tmp, parts, EXTRA_SF_MAX + 1, ' ');
            if (num == 1 && (strcmp(parts[0], "0") == 0 || strcmp(parts[0], "off") == 0)) {
                num = 0;
            }
            if (num > EXTRA_SF_MAX) {
                sprintf(reply, "Error: at most %d extra SFs", EXTRA_SF_MAX);
            } else {
                for (int i = 0; i < num; i++) sfs[i] = (uint8_t)atoi(parts[i]);
                if (_callbacks->configSideDetectors(sfs, (uint8_t)num)) {
                    memset(_prefs->extra_sf, 0, sizeof(_prefs->extra_sf));
                    for (int i = 0; i < num; i++) _prefs->extra_sf[i] = sfs[i];
                    savePrefs();
                    strcpy(reply, num ? "OK - extra SFs set" : "OK - extra SFs cleared");
                } else {
                    strcpy(reply, "Error: unsupported or invalid extra SF config");
                }
            }
        } else if (memcmp(config, "multi.acks ", 11) == 0) {
            long val;
            if (cliNum(&config[11], cliDefaults()->multi_acks, &val) &&
                (val == 0 || val == 1)) {
                _prefs->multi_acks = (uint8_t)val;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: must be 0, 1 or default");
            }
#ifdef CONFIG_ZEPHCORE_ROLE_ROOM_SERVER
        /* Room server only -- see the matching guard on the `get` side. */
        } else if (memcmp(config, "allow.read.only ", 16) == 0) {
            int on = cliOnOff(&config[16], cliDefaults()->allow_read_only);
            if (on >= 0) {
                _prefs->allow_read_only = (uint8_t)on;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: must be on, off or default");
            }
#endif
        } else if (memcmp(config, "flood.advert.interval ", 22) == 0) {
            long hours;
            if (!cliNum(&config[22], cliDefaults()->flood_advert_interval, &hours)) {
                strcpy(reply, "Error: expected 0, 3-168 hours, or default");
            } else if ((hours > 0 && hours < 3) || (hours > 168)) {
                strcpy(reply, "Error: interval range is 3-168 hours");
            } else {
                _prefs->flood_advert_interval = (uint8_t)hours;
                _callbacks->updateFloodAdvertTimer();
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "advert.interval ", 16) == 0) {
            long mins;
            if (!cliNum(&config[16], (long)cliDefaults()->advert_interval * 2, &mins)) {
                snprintf(reply, CLI_REPLY_SIZE,
                         "Error: expected 0, %d-240 minutes, or default",
                         MIN_LOCAL_ADVERT_INTERVAL);
            } else if ((mins > 0 && mins < MIN_LOCAL_ADVERT_INTERVAL) || (mins > 240)) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: interval range is %d-240 minutes", MIN_LOCAL_ADVERT_INTERVAL);
            } else {
                _prefs->advert_interval = (uint8_t)(mins / 2);
                _callbacks->updateAdvertTimer();
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "guest.password ", 15) == 0) {
            StrHelper::strzcpy(_prefs->guest_password, &config[15], sizeof(_prefs->guest_password));
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
            snprintf(tmp, sizeof(tmp), "%.*s", (int)(sizeof(tmp) - 1), &config[6]);
            const char* parts[4];
            int num = mesh::Utils::parseTextParts(tmp, parts, 4);
            /* "set radio default" restores all four together — they are one
             * interop-critical set and resetting them piecemeal can leave a
             * node on a combination no other node uses. */
            bool want_def = (num == 1 && cliIsDefault(parts[0]));
            float freq = want_def ? cliDefaults()->freq : (num > 0 ? strtof(parts[0], nullptr) : 0.0f);
            float bw = want_def ? cliDefaults()->bw : (num > 1 ? strtof(parts[1], nullptr) : 0.0f);
            uint8_t sf = want_def ? cliDefaults()->sf : (num > 2 ? (uint8_t)atoi(parts[2]) : 0);
            uint8_t cr = want_def ? cliDefaults()->cr : (num > 3 ? (uint8_t)atoi(parts[3]) : 0);
            if (freq >= 150.0f && freq <= 2500.0f && sf >= 5 && sf <= 12 &&
                cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= 500.0f) {
                /* Snapshot old params, then mutate _prefs and save so later
                 * savePrefs() calls (set af, set name, ...) don't clobber
                 * the new values with stale RAM. Freeze the running radio on
                 * the old params via override so the on-air config doesn't
                 * change until reboot. */
                float   old_freq = _prefs->freq;
                float   old_bw   = _prefs->bw;
                uint8_t old_sf   = _prefs->sf;
                uint8_t old_cr   = _prefs->cr;
                _prefs->freq = freq;
                _prefs->bw   = bw;
                _prefs->sf   = sf;
                _prefs->cr   = cr;
                _callbacks->savePrefs();
                _callbacks->freezeRadioParams(old_freq, old_bw, old_sf, old_cr);
                strcpy(reply, "OK - reboot to apply");
            } else {
                strcpy(reply, "Error: freq 150-2500, bw 7-500, sf 5-12, cr 5-8, or default");
            }
        } else if (memcmp(config, "lat ", 4) == 0) {
            _prefs->node_lat = cliIsDefault(&config[4]) ? cliDefaults()->node_lat
                                                        : atof(&config[4]);
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "lon ", 4) == 0) {
            _prefs->node_lon = cliIsDefault(&config[4]) ? cliDefaults()->node_lon
                                                        : atof(&config[4]);
            savePrefs();
            strcpy(reply, "OK");
        } else if (memcmp(config, "rxdelay ", 8) == 0) {
            _prefs->rx_delay_base = cliIsDefault(&config[8]) ? cliDefaults()->rx_delay_base
                                                             : (float)atof(&config[8]);
            savePrefs();
            strcpy(reply, "OK (ignored: rxdelay is now adaptive)");
        } else if (memcmp(config, "txdelay ", 8) == 0) {
            _prefs->tx_delay_factor = cliIsDefault(&config[8]) ? cliDefaults()->tx_delay_factor
                                                               : (float)atof(&config[8]);
            savePrefs();
            strcpy(reply, "OK (ignored: txdelay is now adaptive)");
        } else if (memcmp(config, "flood.max.advert ", 17) == 0) {
            long m;
            if (cliNum(&config[17], cliDefaults()->flood_max_advert, &m) && m >= 0 && m <= 64) {
                _prefs->flood_max_advert = (uint8_t)m;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: range 0-64, or default");
            }
        } else if (memcmp(config, "flood.max.unscoped ", 19) == 0) {
            long m;
            if (cliNum(&config[19], cliDefaults()->flood_max_unscoped, &m) && m >= 0 && m <= 64) {
                _prefs->flood_max_unscoped = (uint8_t)m;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: range 0-64, or default");
            }
        } else if (memcmp(config, "flood.max ", 10) == 0) {
            long m;
            if (cliNum(&config[10], cliDefaults()->flood_max, &m) && m >= 0 && m <= 64) {
                _prefs->flood_max = (uint8_t)m;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error: range 0-64, or default");
            }
        } else if (memcmp(config, "direct.txdelay ", 15) == 0) {
            _prefs->direct_tx_delay_factor = cliIsDefault(&config[15])
                                             ? cliDefaults()->direct_tx_delay_factor
                                             : (float)atof(&config[15]);
            savePrefs();
            strcpy(reply, "OK (ignored: direct.txdelay is now adaptive)");
        } else if (memcmp(config, "backoff.multiplier ", 19) == 0) {
            /* Companion's setBackoffMultiplier callback is the base-class
             * no-op and the value isn't restored at boot — reject instead of
             * a false OK. */
            if (strcmp(_callbacks->getRole(), "companion") == 0) {
                strcpy(reply, "Error: not supported on companion");
            } else {
                /* initNodePrefs() leaves this at 0.0 (memset) even though the
                 * live default is ContentionTracker::DEFAULT_BACKOFF_MULT and
                 * RepeaterDataStore migrates 0.0 -> 0.2 on load, so `default`
                 * uses the real value rather than the prefs blank. */
                float f;
                if (!cliFloat(&config[19], 0.2f, &f)) {
                    strcpy(reply, "Error, range 0.0-2.0, or default");
                } else if (f >= 0.0f && f <= 2.0f) {
                    _prefs->backoff_multiplier = f;
                    _callbacks->setBackoffMultiplier(f);
                    savePrefs();
                    strcpy(reply, "OK");
                } else {
                    strcpy(reply, "Error, range 0.0-2.0");
                }
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
            long mode;
            if (cliNum(config, cliDefaults()->path_hash_mode, &mode) &&
                mode >= 0 && mode < 3) {
                _prefs->path_hash_mode = (uint8_t)mode;
                savePrefs();
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Error, must be 0, 1, 2 or default");
            }
        } else if (memcmp(config, "loop.detect ", 12) == 0) {
            /* Loop detection runs only in the Repeater/RoomServer forward
             * path — companions never consult loop_detect. */
            if (strcmp(_callbacks->getRole(), "companion") == 0) {
                strcpy(reply, "Error: not supported on companion");
                return;
            }
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
            } else if (cliIsDefault(config)) {
                mode = cliDefaults()->loop_detect;
            } else {
                mode = 0xFF;
                strcpy(reply, "Error, must be: off, minimal, moderate, strict, or default");
            }
            if (mode != 0xFF) {
                _prefs->loop_detect = mode;
                savePrefs();
                strcpy(reply, "OK");
            }
        } else if (memcmp(config, "tx ", 3) == 0) {
            long parsed;
            int max_tx = 30;
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
            max_tx = CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
#endif
            if (!cliNum(&config[3], cliDefaults()->tx_power_dbm, &parsed) ||
                parsed < -9 || parsed > max_tx) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: range -9 to %d dBm, or default", max_tx);
            } else {
                _prefs->tx_power_dbm = (int8_t)parsed;
                savePrefs();
                _callbacks->setTxPower(_prefs->tx_power_dbm);
                snprintf(reply, CLI_REPLY_SIZE, "OK - tx power=%d dBm",
                         (int)_prefs->tx_power_dbm);
            }
        } else if (sender_timestamp == 0 && memcmp(config, "freq ", 5) == 0) {
            float f;
            if (cliFloat(&config[5], cliDefaults()->freq, &f) &&
                f >= 150.0f && f <= 2500.0f) {
                float old_freq = _prefs->freq;
                _prefs->freq = f;
                savePrefs();
                /* Keep _prefs->freq = f in RAM so a later savePrefs() (from any
                 * other "set" command before reboot) can't rewrite the old freq
                 * back; freeze the running radio on the old freq until reboot,
                 * mirroring the "set radio" handler above. */
                _callbacks->freezeRadioParams(old_freq, _prefs->bw, _prefs->sf, _prefs->cr);
                strcpy(reply, "OK - reboot to apply");
            } else {
                strcpy(reply, "Error: range 150-2500 MHz, or default");
            }
        } else if (strcmp(config, "adc.multiplier target") == 0) {
            strcpy(reply, "Error: need mV target  (e.g. set adc.multiplier target 4173)");
        } else if (memcmp(config, "adc.multiplier target ", 22) == 0) {
            /* Calibrate against a known voltage measured with a multimeter. */
            uint16_t target_mv = (uint16_t)atoi(&config[22]);
            uint16_t current_mv = _board->getBattMilliVolts();
            if (current_mv == 0) {
                strcpy(reply, "Error: no ADC reading on this board");
            } else if (target_mv < 3000 || target_mv > 4400) {
                strcpy(reply, "Error: target out of range (3000-4400 mV)");
            } else {
                float current_mult = _board->getAdcMultiplier();
                float new_mult = current_mult * (float)target_mv / (float)current_mv;
                _prefs->adc_multiplier = new_mult;
                if (_board->setAdcMultiplier(new_mult)) {
                    savePrefs();
                    snprintf(reply, CLI_REPLY_SIZE,
                             "OK - multiplier %.3f -> %.3f  (%u -> %u mV)",
                             (double)current_mult, (double)new_mult,
                             current_mv, target_mv);
                } else {
                    _prefs->adc_multiplier = 0.0f;
                    strcpy(reply, "Error: unsupported by this board");
                }
            }
        } else if (memcmp(config, "adc.multiplier full", 19) == 0) {
            /* Calibrate: board must be on a full charge. Scales the current
             * multiplier so the ADC reads the board's curve 100% point. */
            uint16_t current_mv = _board->getBattMilliVolts();
            if (current_mv == 0) {
                strcpy(reply, "Error: no ADC reading on this board");
            } else {
                uint16_t target_mv = battery_curve_default.ocv_mv[0];
                float current_mult = _board->getAdcMultiplier();
                float new_mult = current_mult * (float)target_mv / (float)current_mv;
                _prefs->adc_multiplier = new_mult;
                if (_board->setAdcMultiplier(new_mult)) {
                    savePrefs();
                    snprintf(reply, CLI_REPLY_SIZE,
                             "OK - multiplier %.3f -> %.3f  (%u -> %u mV)",
                             (double)current_mult, (double)new_mult,
                             current_mv, target_mv);
                } else {
                    _prefs->adc_multiplier = 0.0f;
                    strcpy(reply, "Error: unsupported by this board");
                }
            }
        } else if (memcmp(config, "adc.multiplier ", 15) == 0) {
            const char *arg = &config[15];
            /* 0 is valid (resets to the DTS default). cliFloat() rejects the
             * non-numeric input that atof() used to fold into a 0 here, so the
             * old "distinguish from literal 0" dance is gone. Upper bound
             * covers all real divider/reference combinations with margin. */
            float val;
            bool bad = !cliFloat(arg, cliDefaults()->adc_multiplier, &val);
            if (!bad) bad = (val != 0.0f && val < 100.0f) || val > 30000.0f || val < 0.0f;
            if (bad) {
                strcpy(reply, "Error: invalid multiplier (0 to reset, 100-30000, or default)");
            } else if (_board->setAdcMultiplier(val)) {
                _prefs->adc_multiplier = val;
                savePrefs();
                if (val == 0.0f) {
                    strcpy(reply, "OK - using default board multiplier");
                } else {
                    snprintf(reply, CLI_REPLY_SIZE, "OK - multiplier set to %.3f", (double)val);
                }
            } else {
                strcpy(reply, "Error: unsupported by this board");
            }
        } else if (memcmp(config, "radio.fem.rxgain ", 17) == 0) {
            int val = cliOnOff(&config[17], cliDefaults()->fem_rxgain);
            if (val == 0 || val == 1) {
                /* Same shape as radio.rxgain: always save, then apply live and
                 * report when the radio driver has no FEM gate. */
                _prefs->fem_rxgain = (uint8_t)val;
                savePrefs();
                if (_callbacks->setFemRxGain(val == 1)) {
                    snprintf(reply, CLI_REPLY_SIZE, "OK - radio.fem.rxgain=%d", _prefs->fem_rxgain);
                } else {
                    strcpy(reply, "Error: unsupported");
                }
            } else {
                strcpy(reply, "Error: must be 0, 1, on, or off");
            }
        } else if (memcmp(config, "radio.rxgain ", 13) == 0) {
            int val = cliOnOff(&config[13], cliDefaults()->rx_boost);
            if (val == 0 || val == 1) {
                /* Always save (upstream f3d4d8cd), then apply live and
                 * report when the radio has no RX boost feature. */
                _prefs->rx_boost = (uint8_t)val;
                savePrefs();
                if (_callbacks->setRxBoostedGain(val == 1)) {
                    snprintf(reply, CLI_REPLY_SIZE, "OK - radio.rxgain=%d", _prefs->rx_boost);
                } else {
                    strcpy(reply, "Error: unsupported");
                }
            } else {
                strcpy(reply, "Error: must be 0, 1, on, or off");
            }
        } else if (memcmp(config, "rxduty ", 7) == 0) {
            int val = cliOnOff(&config[7], cliDefaults()->rx_duty_cycle);
            if (val == 0 || val == 1) {
                _prefs->rx_duty_cycle = (uint8_t)val;
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - rxduty=%d (reboot to apply)", _prefs->rx_duty_cycle);
            } else {
                strcpy(reply, "Error: must be 0, 1, on, or off");
            }
        } else if (memcmp(config, "display.rotate ", 15) == 0) {
            // Rotate the panel 180 degrees for cases that mount it upside down.
            // Applied immediately -- the SSD1306/SH1106 remap is two bytes on
            // the wire and the next frame comes out flipped, no redraw needed.
            int val = cliOnOff(&config[15], cliDefaults()->display_rotate);
            if (val != 0 && val != 1) {
                strcpy(reply, "Error: must be 0, 1, on, or off");
            } else {
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
                int ret = mc_display_set_rotated(val == 1);
                if (ret == 0) {
                    // Persist only what the panel actually did, so a stored
                    // value can never disagree with what the screen shows.
                    _prefs->display_rotate = (uint8_t)val;
                    savePrefs();
                    snprintf(reply, CLI_REPLY_SIZE, "OK - display.rotate=%d", val);
                } else if (ret == -ENOTSUP) {
                    strcpy(reply, "Error: this panel cannot rotate");
                } else {
                    snprintf(reply, CLI_REPLY_SIZE, "Error: rotate failed (%d)", ret);
                }
#else
                strcpy(reply, "Error: no display");
#endif
            }
        } else if (memcmp(config, "input.rotate ", 13) == 0) {
            // Swap the joystick/D-pad axes to match an upside-down mount.
            // Deliberately separate from display.rotate: a case can flip the
            // screen without flipping the stick, and boards whose panel cannot
            // rotate can still need the axis swap.
            int val = cliOnOff(&config[13], cliDefaults()->input_rotate);
            if (val == 0 || val == 1) {
                zephcore_input_set_flipped(val == 1);
                _prefs->input_rotate = (uint8_t)val;
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - input.rotate=%d", val);
            } else {
                strcpy(reply, "Error: must be 0, 1, on, or off");
            }
        } else if (memcmp(config, "tz.offset ", 10) == 0) {
            // Whole-hour offset from UTC for the ON-DEVICE CLOCK DISPLAY only.
            // The RTC, `clock` and `time <epoch>` all stay UTC: they round-trip
            // with each other, apps parse them, and a timezone that reached the
            // clock would look like a backward jump to every timestamp consumer
            // on the node (see NodePrefs::tz_offset).
            //
            // Whole hours, matching upstream MeshCore's command of the same
            // name, so an app that speaks to both trees behaves identically.
            // Half-hour zones (+5:30, -3:30) cannot be expressed.
            long val;
            if (!cliNum(&config[10], cliDefaults()->tz_offset, &val)) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: expected %d..%d or default",
                         TZ_OFFSET_MIN, TZ_OFFSET_MAX);
            } else if (val < TZ_OFFSET_MIN || val > TZ_OFFSET_MAX) {
                snprintf(reply, CLI_REPLY_SIZE, "Error: offset range is %d..%d",
                         TZ_OFFSET_MIN, TZ_OFFSET_MAX);
            } else {
                _prefs->tz_offset = (int8_t)val;
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - tz.offset=%d", (int)val);
            }
        } else if (memcmp(config, "gps diag", 8) == 0) {
            // set gps diag <0|1|on|off> — arm module-configuration reporting.
            // Not persisted: clears on reboot, by design.
            /* Not persisted, so "default" means the boot state: off. */
            int val = cliOnOff(config + 8, 0);
            if (val == 0 || val == 1) {
                gps_set_diag(val == 1);
                if (val == 1) {
                    strcpy(reply, "OK - gps diag on; run 'gps off' then 'gps on', "
                                  "then 'get gps diag'");
                } else {
                    strcpy(reply, "OK - gps diag off");
                }
            } else {
                strcpy(reply, "usage: set gps diag <0|1|on|off>");
            }
        } else if (memcmp(config, "gps duty", 8) == 0) {
            // set gps duty <seconds> | default   (0 = always on)
            const char* arg = config + 8;
            while (*arg == ' ') arg++;
            uint32_t val = 0;
            bool ok = true;
            if (*arg == '\0') {
                ok = false;
            } else if (strcmp(arg, "default") == 0) {
                val = _callbacks->getDefaultGpsIntervalSec();
            } else {
                char* end = NULL;
                unsigned long parsed = strtoul(arg, &end, 10);
                // reject non-numeric, fractions, or trailing garbage
                if (end == arg || *end != '\0') ok = false;
                else val = (uint32_t)parsed;
            }
            if (!ok) {
                strcpy(reply, "usage: set gps duty <seconds> | default  (0 = always on)");
            } else {
                if (val > 604800UL) val = 604800UL;      // cap at 1 week
                else if (val != 0 && val < 10) val = 10; // floor 10s (0 = always on)
                _prefs->gps_interval = val;
                gps_set_poll_interval_sec(val);          // apply live
                savePrefs();
                if (val == 0) strcpy(reply, "OK - gps duty=0 (always on)");
                else snprintf(reply, CLI_REPLY_SIZE, "OK - gps duty=%u s", (unsigned)val);
            }
        } else if (memcmp(config, "meshtimesync ", 13) == 0) {
            int on = cliOnOff(&config[13], cliDefaults()->meshtimesync);
            if (_callbacks->getMeshTimeSync() == nullptr) {
                strcpy(reply, "not available");
            } else if (on < 0) {
                strcpy(reply, "Error: must be on, off or default");
            } else {
                _prefs->meshtimesync = (uint8_t)on;
                savePrefs();
                snprintf(reply, CLI_REPLY_SIZE, "OK - meshtimesync %s", on ? "on" : "off");
            }
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "unknown config: %.230s", config);
        }
    } else if (sender_timestamp == 0 && strcmp(command, "erase") == 0) {
        bool s = _callbacks->formatFileSystem();
        if (s) {
            /* formatFileSystem() flattens the mounted NVS bonds partition,
             * leaving stale in-RAM bond state.  Reboot (deferred so this
             * reply transmits first) so NVS + the BT stack re-init cleanly. */
            snprintf(reply, CLI_REPLY_SIZE, "File system erase: OK - rebooting");
            scheduleReboot(REBOOT_NORMAL);
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "File system erase: Err");
        }
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
        snprintf(tmp, sizeof(tmp), "%.*s", (int)(sizeof(tmp) - 1), &command[11]);
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
