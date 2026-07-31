/*
 * SPDX-License-Identifier: MIT
 * RepeaterMesh MQTT uplink — WiFi/MQTT telemetry publishing for repeaters.
 *
 * Split out of RepeaterMesh.cpp for readability. Defines the RepeaterMesh
 * uplink methods; the uplink *init* (WiFi/MQTT start, topic strings) stays in
 * RepeaterMesh::begin(). Compiled only when both CONFIG_ZEPHCORE_REPEATER_UPLINK
 * and CONFIG_MQTT_LIB are set — added to target_sources in that CMake branch.
 */

#include "RepeaterMesh.h"

#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)

#include <mesh/Utils.h>
#include <helpers/MeshcoreJson.h>
#include <helpers/TxtDataHelpers.h>
#include <adapters/radio/LoRaRadioBase.h>
#include "observer_creds.h"
#include <ZephyrWiFiStation.h>
#include <ZephyrMQTTPublisher.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool RepeaterMesh::saveUplinkCreds()
{
    if (!_store) return false;
    return observer_creds_save(&_uplink_creds, _store->getBasePath());
}

bool RepeaterMesh::handleUplinkCommand(const char *command, char *reply)
{
    if (memcmp(command, "get uplink.", 11) == 0) {
        const char *key = command + 11;
        if (strcmp(key, "enable") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", isUplinkEnabled() ? "on" : "off");
        } else if (strcmp(key, "wifi.ssid") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _uplink_creds.wifi_ssid[0] ? _uplink_creds.wifi_ssid : "(not set)");
        } else if (strcmp(key, "mqtt.host") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _uplink_creds.mqtt_host[0] ? _uplink_creds.mqtt_host : "(not set)");
        } else if (strcmp(key, "mqtt.port") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (unsigned)_uplink_creds.mqtt_port);
        } else if (strcmp(key, "mqtt.tls") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %u", (unsigned)_uplink_creds.mqtt_tls);
        } else if (strcmp(key, "mqtt.user") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _uplink_creds.mqtt_user[0] ? _uplink_creds.mqtt_user : "(not set)");
        } else if (strcmp(key, "mqtt.iata") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> %s", _uplink_creds.mqtt_iata[0] ? _uplink_creds.mqtt_iata : "(not set)");
        } else if (strcmp(key, "status") == 0) {
            snprintf(reply, CLI_REPLY_SIZE, "> enabled=%s wifi=%s mqtt=%s reboot_required=%s",
                     isUplinkEnabled() ? "yes" : "no",
                     zc_wifi_station_is_connected() ? "up" : "down",
                     mqtt_publisher_is_connected() ? "up" : "down",
                     _uplink_reboot_required ? "yes" : "no");
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "unknown config: uplink.%s", key);
        }
        return true;
    }

    if (memcmp(command, "set uplink.", 11) == 0) {
        const char *cfg = command + 11;
        const char *val = strchr(cfg, ' ');
        if (!val) {
            strcpy(reply, "Error: value required");
            return true;
        }
        int key_len = (int)(val - cfg);
        val++;

        if (key_len == 6 && memcmp(cfg, "enable", 6) == 0) {
            if (strcmp(val, "on") == 0) {
                setUplinkEnabled(true);
            } else if (strcmp(val, "off") == 0) {
                setUplinkEnabled(false);
            } else {
                strcpy(reply, "Error: must be on or off");
                return true;
            }
        } else if (key_len == 9 && memcmp(cfg, "wifi.ssid", 9) == 0) {
            StrHelper::strncpy(_uplink_creds.wifi_ssid, val, sizeof(_uplink_creds.wifi_ssid));
        } else if (key_len == 8 && memcmp(cfg, "wifi.psk", 8) == 0) {
            StrHelper::strncpy(_uplink_creds.wifi_psk, val, sizeof(_uplink_creds.wifi_psk));
        } else if (key_len == 9 && memcmp(cfg, "mqtt.host", 9) == 0) {
            StrHelper::strncpy(_uplink_creds.mqtt_host, val, sizeof(_uplink_creds.mqtt_host));
        } else if (key_len == 9 && memcmp(cfg, "mqtt.port", 9) == 0) {
            int port = atoi(val);
            if (port < 1 || port > 65535) {
                strcpy(reply, "Error: port range 1-65535");
                return true;
            }
            _uplink_creds.mqtt_port = (uint16_t)port;
        } else if (key_len == 8 && memcmp(cfg, "mqtt.tls", 8) == 0) {
            _uplink_creds.mqtt_tls = (atoi(val) != 0) ? 1 : 0;
        } else if (key_len == 9 && memcmp(cfg, "mqtt.user", 9) == 0) {
            StrHelper::strncpy(_uplink_creds.mqtt_user, val, sizeof(_uplink_creds.mqtt_user));
        } else if (key_len == 13 && memcmp(cfg, "mqtt.password", 13) == 0) {
            StrHelper::strncpy(_uplink_creds.mqtt_password, val, sizeof(_uplink_creds.mqtt_password));
        } else if (key_len == 9 && memcmp(cfg, "mqtt.iata", 9) == 0) {
            StrHelper::strncpy(_uplink_creds.mqtt_iata, val, sizeof(_uplink_creds.mqtt_iata));
        } else {
            snprintf(reply, CLI_REPLY_SIZE, "unknown config: uplink.%.*s", key_len, cfg);
            return true;
        }

        if (!saveUplinkCreds()) {
            strcpy(reply, "Error: save failed");
            return true;
        }
        markUplinkRebootRequired();
        strcpy(reply, "OK - reboot to apply");
        return true;
    }

    return false;
}

void RepeaterMesh::publishUplinkPacket(mesh::Packet *pkt)
{
    if (!isUplinkEnabled() || !mqtt_publisher_is_connected()) return;
    if (_uplink_packets_topic[0] == '\0') return;

    char raw_hex[MAX_TRANS_UNIT * 2 + 1];
    mesh::Utils::toHex(raw_hex, _uplink_last_raw, _uplink_last_raw_len);
    raw_hex[_uplink_last_raw_len * 2] = '\0';

    uint8_t hash_bytes[MAX_HASH_SIZE];
    char hash_hex[MAX_HASH_SIZE * 2 + 1];
    pkt->calculatePacketHash(hash_bytes);
    mesh::Utils::toHex(hash_hex, hash_bytes, MAX_HASH_SIZE);
    hash_hex[MAX_HASH_SIZE * 2] = '\0';

    uint32_t now_epoch = getRTCClock()->getCurrentTime();

    size_t json_cap;
    char *json_buf = mqtt_publisher_stage(&json_cap);
    struct MeshcorePacketJson pj = {
        _prefs.node_name,
        _uplink_pubkey_hex,
        now_epoch,
        _uplink_last_raw_len,
        (unsigned)pkt->getPayloadType(),
        pkt->isRouteDirect() ? "D" : "F",
        (unsigned)pkt->payload_len,
        raw_hex,
        (int)pkt->getSNR(),
        (int)_uplink_last_rssi,
        (int)(_uplink_last_score * 1000.0f),
        hash_hex,
    };
    int json_len = meshcore_build_packet_json(json_buf, json_cap, &pj);

    if (json_len <= 0 || json_len >= (int)json_cap) {
        return;
    }
    mqtt_publisher_commit(MQTT_PUB_TOPIC_PACKETS, json_len);
}

void RepeaterMesh::publishUplinkStatus(const char *status)
{
    /* The connected gate matters here: without it the 300 s status timer
     * fills the publish queue with stale "online" messages while WiFi or the
     * broker is down, and they all flush with old timestamps on reconnect. */
    if (!isUplinkEnabled() || !mqtt_publisher_is_connected()) return;
    if (_uplink_status_topic[0] == '\0') return;

    auto& radio_driver = *static_cast<mesh::LoRaRadioBase *>(_radio);
    uint32_t now_epoch = getRTCClock()->getCurrentTime();

    char radio_buf[48];
    snprintf(radio_buf, sizeof(radio_buf), "%.3f,%.1f,%u,%u",
             (double)_prefs.freq, (double)_prefs.bw,
             (unsigned)_prefs.sf, (unsigned)_prefs.cr);

    size_t json_cap;
    char *json_buf = mqtt_publisher_stage(&json_cap);
    struct MeshcoreStatusJson sj = {
        status,
        now_epoch,
        _prefs.node_name,
        _uplink_pubkey_hex,
        radio_buf,
#ifdef CONFIG_ZEPHCORE_BOARD_NAME
        CONFIG_ZEPHCORE_BOARD_NAME,
#else
        "unknown",
#endif
        FIRMWARE_VERSION,
        (unsigned)_board.getBattMilliVolts(),
        (unsigned)(uptime_millis / 1000),
        (unsigned)_err_flags,
        (unsigned)_mgr->getOutboundTotal(),
        _radio->getNoiseFloor(),
        (unsigned)(getTotalAirTime() / 1000),
        (unsigned)(getReceiveAirTime() / 1000),
        (unsigned)radio_driver.getPacketsRecvErrors(),
    };
    int json_len = meshcore_build_status_json(json_buf, json_cap, &sj);

    if (json_len <= 0 || json_len >= (int)json_cap) {
        return;
    }
    mqtt_publisher_commit(MQTT_PUB_TOPIC_STATUS, json_len);
}

#endif /* CONFIG_ZEPHCORE_REPEATER_UPLINK && CONFIG_MQTT_LIB */
