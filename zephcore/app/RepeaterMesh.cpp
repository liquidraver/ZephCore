/*
 * SPDX-License-Identifier: Apache-2.0
 * RepeaterMesh - LoRa mesh repeater implementation
 */

#include "RepeaterMesh.h"
#include <mesh/Utils.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
#include <helpers/MeshcoreJson.h>
#include <adapters/radio/LoRaRadioBase.h>
#include <adapters/sensors/SimpleLPP.h>
#include <adapters/sensors/ZephyrEnvSensors.h>
#include <adapters/gps/ZephyrGPSManager.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
#include "observer_creds.h"
#include <ZephyrWiFiStation.h>
#include <ZephyrMQTTPublisher.h>
#endif

/* Helper to get radio driver for stats — uses LoRaRadioBase (works for SX126x and LR1110) */
static inline mesh::LoRaRadioBase& getRadioDriver(mesh::Radio* radio) {
    return *static_cast<mesh::LoRaRadioBase*>(radio);
}

/* Simple sort helper since <algorithm> is not available in Zephyr minimal C++ */
template<typename T, typename Comparator>
static void simple_sort(T* arr, int count, Comparator cmp) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (cmp(arr[j], arr[i])) {
                T temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }
    }
}

LOG_MODULE_REGISTER(zephcore_repeater, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
static RepeaterMesh *s_uplink_mesh;
static void uplink_time_sync_cb(uint32_t unix_ts)
{
	if (s_uplink_mesh) {
		s_uplink_mesh->getRTCClock()->setCurrentTime(unix_ts);
	}
}
#endif

/* Protocol constants */
#define FIRMWARE_VER_LEVEL       2

#define REQ_TYPE_GET_STATUS         0x01
#define REQ_TYPE_KEEP_ALIVE         0x02
#define REQ_TYPE_GET_TELEMETRY_DATA 0x03
#define REQ_TYPE_GET_ACCESS_LIST    0x05
#define REQ_TYPE_GET_NEIGHBOURS     0x06
#define REQ_TYPE_GET_OWNER_INFO     0x07

#define RESP_SERVER_LOGIN_OK        0

#define ANON_REQ_TYPE_REGIONS      0x01
#define ANON_REQ_TYPE_OWNER        0x02
#define ANON_REQ_TYPE_BASIC        0x03

#define CLI_REPLY_DELAY_MILLIS      600
#define LAZY_CONTACTS_WRITE_DELAY   5000
#define SERVER_RESPONSE_DELAY       300
#define TXT_ACK_DELAY               200

/* Helper: futureMillis */
static inline unsigned long futureMillis(uint32_t delta_ms) {
    return k_uptime_get() + delta_ms;
}

static inline bool millisHasNowPassed(unsigned long target) {
    return (int64_t)k_uptime_get() >= (int64_t)target;
}

static void radio_set_tx_power(uint8_t power_dbm) {
    /* TX power is configured as part of lora_config() in radio_set_params
     * The Zephyr LoRa driver doesn't have a separate lora_set_tx_power API.
     * Instead, we log that TX power setting is requested. The actual power
     * is set in the board defconfig via CONFIG_LORA_TX_POWER. */
    LOG_INF("TX power %d dBm requested (configured via board defconfig)", power_dbm);
}

void RepeaterMesh::putNeighbour(const mesh::Identity& id, uint32_t timestamp, float snr) {
#if MAX_NEIGHBOURS > 0
    uint32_t oldest_timestamp = 0xFFFFFFFF;
    NeighbourInfo* neighbour = &neighbours[0];

    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        if (id.matches(neighbours[i].id)) {
            neighbour = &neighbours[i];
            break;
        }
        if (neighbours[i].heard_timestamp < oldest_timestamp) {
            neighbour = &neighbours[i];
            oldest_timestamp = neighbour->heard_timestamp;
        }
    }

    neighbour->id = id;
    neighbour->advert_timestamp = timestamp;
    neighbour->heard_timestamp = getRTCClock()->getCurrentTime();
    neighbour->snr = (int8_t)(snr * 4);
#endif
}

uint8_t RepeaterMesh::handleLoginReq(const mesh::Identity& sender, const uint8_t* secret, uint32_t sender_timestamp, const uint8_t* data, bool is_flood) {
    ClientInfo* client = nullptr;

    if (data[0] == 0) {
        client = acl.getClient(sender.pub_key, PUB_KEY_SIZE);
    }

    if (client == nullptr) {
        uint8_t perms;

        /* Constant-time comparison: pad the received password to the full
         * 16-byte storage size with zeros, then compare against both
         * stored passwords (which are already zero-padded by initNodePrefs).
         * Compare both unconditionally so timing is identical for any
         * wrong password regardless of which (admin/guest) it most
         * resembles. */
        uint8_t received[sizeof(_prefs.password)] = {0};
        size_t r_len = strnlen((const char *)data, sizeof(received) - 1);
        memcpy(received, data, r_len);

        bool admin_match = mesh::Utils::constantTimeEqual(received,
                                                          _prefs.password,
                                                          sizeof(received));
        bool guest_match = mesh::Utils::constantTimeEqual(received,
                                                          _prefs.guest_password,
                                                          sizeof(received));

        if (admin_match) {
            perms = PERM_ACL_ADMIN;
        } else if (guest_match) {
            perms = PERM_ACL_GUEST;
        } else {
            /* Apply global failed-login rate limit. The check itself is
             * unconditional regardless of admin/guest path so its timing
             * doesn't leak which credential the attempt was closer to. */
            if (!login_fail_limiter.allow(getRTCClock()->getCurrentTime())) {
                LOG_WRN("Login rate-limited (failed attempts exceeded)");
            } else {
                LOG_WRN("Invalid password");
            }
            return 0;
        }

        client = acl.putClient(sender, 0);
        if (sender_timestamp <= client->last_timestamp) {
            LOG_WRN("Possible login replay attack!");
            return 0;
        }

        LOG_INF("Login success");
        client->last_timestamp = sender_timestamp;
        client->last_activity = getRTCClock()->getCurrentTime();
        client->permissions &= ~0x03;
        client->permissions |= perms;
        memcpy(client->shared_secret, secret, PUB_KEY_SIZE);

        if (perms != PERM_ACL_GUEST) {
            if (!dirty_contacts_expiry) dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
        }
    }

    if (is_flood) {
        client->out_path_len = OUT_PATH_UNKNOWN;
    }

    uint32_t now = getRTCClock()->getCurrentTimeUnique();
    memcpy(reply_data, &now, 4);
    reply_data[4] = RESP_SERVER_LOGIN_OK;
    reply_data[5] = 0;
    reply_data[6] = client->isAdmin() ? 1 : 0;
    reply_data[7] = client->permissions;
    getRNG()->random(&reply_data[8], 4);
    reply_data[12] = FIRMWARE_VER_LEVEL;

    return 13;
}

uint8_t RepeaterMesh::handleAnonRegionsReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data, size_t data_len) {
    if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
        if (data_len < 1) return 0;
        reply_path_len = *data++;
        data_len--;
        /* data is anon-req-supplied; bound copy with remaining data_len.
         * If the claimed path is longer than the bytes provided, copyPath
         * rejects (returns 0) and leaves reply_path stale — fall back to a
         * flood reply instead of emitting a corrupt direct path (F2).
         * path_len == 0 is a valid zero-hop direct reply, so don't treat
         * its (also-0) return as a rejection. */
        if (reply_path_len != 0 &&
            mesh::Packet::copyPath(reply_path, data, data_len, reply_path_len) == 0) {
            reply_path_len = OUT_PATH_UNKNOWN;
        }

        memcpy(reply_data, &sender_timestamp, 4);
        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(&reply_data[4], &now, 4);

        return 8 + region_map.exportNamesTo((char*)&reply_data[8], sizeof(reply_data) - 12, REGION_DENY_FLOOD);
    }
    return 0;
}

uint8_t RepeaterMesh::handleAnonOwnerReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data, size_t data_len) {
    if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
        if (data_len < 1) return 0;
        reply_path_len = *data++;
        data_len--;
        /* data is anon-req-supplied; bound copy with remaining data_len.
         * If the claimed path is longer than the bytes provided, copyPath
         * rejects (returns 0) and leaves reply_path stale — fall back to a
         * flood reply instead of emitting a corrupt direct path (F2).
         * path_len == 0 is a valid zero-hop direct reply, so don't treat
         * its (also-0) return as a rejection. */
        if (reply_path_len != 0 &&
            mesh::Packet::copyPath(reply_path, data, data_len, reply_path_len) == 0) {
            reply_path_len = OUT_PATH_UNKNOWN;
        }

        memcpy(reply_data, &sender_timestamp, 4);
        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(&reply_data[4], &now, 4);
        sprintf((char*)&reply_data[8], "%s\n%s", _prefs.node_name, _prefs.owner_info);

        return 8 + strlen((char*)&reply_data[8]);
    }
    return 0;
}

uint8_t RepeaterMesh::handleAnonClockReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data, size_t data_len) {
    if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
        if (data_len < 1) return 0;
        reply_path_len = *data++;
        data_len--;
        /* data is anon-req-supplied; bound copy with remaining data_len.
         * If the claimed path is longer than the bytes provided, copyPath
         * rejects (returns 0) and leaves reply_path stale — fall back to a
         * flood reply instead of emitting a corrupt direct path (F2).
         * path_len == 0 is a valid zero-hop direct reply, so don't treat
         * its (also-0) return as a rejection. */
        if (reply_path_len != 0 &&
            mesh::Packet::copyPath(reply_path, data, data_len, reply_path_len) == 0) {
            reply_path_len = OUT_PATH_UNKNOWN;
        }

        memcpy(reply_data, &sender_timestamp, 4);
        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(&reply_data[4], &now, 4);
        reply_data[8] = 0;  // features
        if (_prefs.disable_fwd) {
            reply_data[8] |= 0x80;
        }
        return 9;
    }
    return 0;
}

int RepeaterMesh::handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len) {
    memcpy(reply_data, &sender_timestamp, 4);

    if (payload[0] == REQ_TYPE_GET_STATUS) {
        auto& radio_driver = getRadioDriver(_radio);
        RepeaterStats stats;
        stats.batt_milli_volts = _board.getBattMilliVolts();
        stats.curr_tx_queue_len = _mgr->getOutboundTotal();
        stats.noise_floor = (int16_t)_radio->getNoiseFloor();
        stats.last_rssi = (int16_t)radio_driver.getLastRSSI();
        stats.n_packets_recv = radio_driver.getPacketsRecv();
        stats.n_packets_sent = radio_driver.getPacketsSent();
        stats.total_air_time_secs = getTotalAirTime() / 1000;
        stats.total_up_time_secs = uptime_millis / 1000;
        stats.n_sent_flood = getNumSentFlood();
        stats.n_sent_direct = getNumSentDirect();
        stats.n_recv_flood = getNumRecvFlood();
        stats.n_recv_direct = getNumRecvDirect();
        stats.err_events = _err_flags;
        stats.last_snr = (int16_t)(radio_driver.getLastSNR() * 4);
        stats.n_direct_dups = ((mesh::SimpleMeshTables *)getTables())->getNumDirectDups();
        stats.n_flood_dups = ((mesh::SimpleMeshTables *)getTables())->getNumFloodDups();
        stats.total_rx_air_time_secs = getReceiveAirTime() / 1000;
        stats.n_recv_errors = radio_driver.getPacketsRecvErrors();
        memcpy(&reply_data[4], &stats, sizeof(stats));
        return 4 + sizeof(stats);
    }

    if (payload[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
        /* CayenneLPP telemetry response using SimpleLPP encoder */
        SimpleLPP lpp(&reply_data[4], sizeof(reply_data) - 4);

        /* Battery voltage — channel 1 = TELEM_CHANNEL_SELF (matches Arduino) */
        const uint8_t CH_SELF = 1;
        uint16_t batt_mv = _board.getBattMilliVolts();
        lpp.addVoltage(CH_SELF, batt_mv / 1000.0f);

        /* Environment sensors — prefer external, fallback to MCU die temp */
        struct env_data env;
        if (env_sensors_read(&env) == 0) {
            if (env.has_temperature) {
                lpp.addTemperature(CH_SELF, env.temperature_c);
            } else if (env.has_mcu_temperature) {
                lpp.addTemperature(CH_SELF, env.mcu_temperature_c);
            } else {
                /* Last resort: MCU temp from board API */
                float mcu_temp = _board.getMCUTemperature();
                if (!isnan(mcu_temp)) {
                    lpp.addTemperature(CH_SELF, mcu_temp);
                }
            }
            if (env.has_humidity) {
                lpp.addRelativeHumidity(CH_SELF, env.humidity_pct);
            }
            if (env.has_pressure) {
                lpp.addBarometricPressure(CH_SELF, env.pressure_hpa);
            }
        } else {
            /* No env sensors at all — try MCU temp directly */
            float mcu_temp = _board.getMCUTemperature();
            if (!isnan(mcu_temp)) {
                lpp.addTemperature(CH_SELF, mcu_temp);
            }
        }

        /* Power monitors (INA219/INA3221/ina2xx) */
        if (power_sensors_available()) {
            struct power_data pwr;
            if (power_sensors_read(&pwr) == 0) {
                uint8_t ch = CH_SELF + 1;
                for (int j = 0; j < pwr.num_channels; j++) {
                    if (pwr.channels[j].valid) {
                        lpp.addVoltage(ch, pwr.channels[j].voltage_v);
                        lpp.addCurrent(ch, pwr.channels[j].current_a);
                        lpp.addPower(ch, pwr.channels[j].power_w);
                        ch++;
                    }
                }
            }
        }

        /* GPS precise position — only shared via telemetry, not adverts */
        struct gps_position gpos;
        if (gps_get_last_known_position(&gpos)) {
            lpp.addGPS(CH_SELF,
                (float)(gpos.latitude_ndeg / 1e9),
                (float)(gpos.longitude_ndeg / 1e9),
                gpos.altitude_mm / 1000.0f);
        }

        /* Wake GPS / extend acquire window so the next telemetry poll has
         * a fresher fix. In repeater mode GPS is normally off between the
         * 48h time-sync cycles — this opportunistically rearms acquire
         * when someone actually cares about our position. No-op if GPS
         * is disabled in prefs. */
        if (gps_is_available() && gps_is_enabled()) {
            gps_request_fresh_fix();
        }

        return 4 + lpp.getSize();
    }

    if (payload[0] == REQ_TYPE_GET_ACCESS_LIST && sender->isAdmin()) {
        uint8_t res1 = payload[1];
        uint8_t res2 = payload[2];
        if (res1 == 0 && res2 == 0) {
            uint8_t ofs = 4;
            for (int i = 0; i < acl.getNumClients() && (size_t)(ofs + 7) <= sizeof(reply_data) - 4; i++) {
                auto c = acl.getClientByIdx(i);
                if (c->permissions == 0) continue;
                memcpy(&reply_data[ofs], c->id.pub_key, 6);
                ofs += 6;
                reply_data[ofs++] = c->permissions;
            }
            return ofs;
        }
    }

    if (payload[0] == REQ_TYPE_GET_NEIGHBOURS) {
#if MAX_NEIGHBOURS > 0
        uint8_t request_version = payload[1];
        if (request_version == 0) {
            int reply_offset = 4;
            uint8_t count = payload[2];
            uint16_t offset;
            memcpy(&offset, &payload[3], 2);
            uint8_t order_by = payload[5];
            uint8_t pubkey_prefix_length = payload[6];

            if (pubkey_prefix_length > PUB_KEY_SIZE) {
                pubkey_prefix_length = PUB_KEY_SIZE;
            }

            // Create sorted copy
            int16_t neighbours_count = 0;
            NeighbourInfo* sorted_neighbours[MAX_NEIGHBOURS];
            for (int i = 0; i < MAX_NEIGHBOURS; i++) {
                if (neighbours[i].heard_timestamp > 0) {
                    sorted_neighbours[neighbours_count++] = &neighbours[i];
                }
            }

            // Sort
            if (order_by == 0) {
                simple_sort(sorted_neighbours, (int)neighbours_count,
                    [](const NeighbourInfo* a, const NeighbourInfo* b) {
                        return a->heard_timestamp > b->heard_timestamp;
                    });
            } else if (order_by == 1) {
                simple_sort(sorted_neighbours, (int)neighbours_count,
                    [](const NeighbourInfo* a, const NeighbourInfo* b) {
                        return a->heard_timestamp < b->heard_timestamp;
                    });
            } else if (order_by == 2) {
                simple_sort(sorted_neighbours, (int)neighbours_count,
                    [](const NeighbourInfo* a, const NeighbourInfo* b) {
                        return a->snr > b->snr;
                    });
            } else if (order_by == 3) {
                simple_sort(sorted_neighbours, (int)neighbours_count,
                    [](const NeighbourInfo* a, const NeighbourInfo* b) {
                        return a->snr < b->snr;
                    });
            }

            // Build results
            int results_count = 0;
            int results_offset = 0;
            uint8_t results_buffer[130];
            for (int index = 0; index < count && index + offset < neighbours_count; index++) {
                int entry_size = pubkey_prefix_length + 4 + 1;
                if (results_offset + entry_size > (int)sizeof(results_buffer)) break;

                auto neighbour = sorted_neighbours[index + offset];
                uint32_t heard_seconds_ago = getRTCClock()->getCurrentTime() - neighbour->heard_timestamp;
                memcpy(&results_buffer[results_offset], neighbour->id.pub_key, pubkey_prefix_length);
                results_offset += pubkey_prefix_length;
                memcpy(&results_buffer[results_offset], &heard_seconds_ago, 4);
                results_offset += 4;
                memcpy(&results_buffer[results_offset], &neighbour->snr, 1);
                results_offset += 1;
                results_count++;
            }

            memcpy(&reply_data[reply_offset], &neighbours_count, 2);
            reply_offset += 2;
            memcpy(&reply_data[reply_offset], &results_count, 2);
            reply_offset += 2;
            memcpy(&reply_data[reply_offset], results_buffer, results_offset);
            reply_offset += results_offset;

            return reply_offset;
        }
#endif
    }

    if (payload[0] == REQ_TYPE_GET_OWNER_INFO) {
        sprintf((char*)&reply_data[4], "%s\n%s\n%s", FIRMWARE_VERSION, _prefs.node_name, _prefs.owner_info);
        return 4 + strlen((char*)&reply_data[4]);
    }

    return 0;
}

mesh::Packet* RepeaterMesh::createSelfAdvert() {
    uint8_t app_data[MAX_ADVERT_DATA_SIZE];
    uint8_t app_data_len = _cli.buildAdvertData(ADV_TYPE_REPEATER, app_data);
    return createAdvert(self_id, app_data, app_data_len);
}

static uint8_t max_loop_minimal[]  = { 0, /* 1-byte */  4, /* 2-byte */  2, /* 3-byte */  1 };
static uint8_t max_loop_moderate[] = { 0, /* 1-byte */  2, /* 2-byte */  1, /* 3-byte */  1 };
static uint8_t max_loop_strict[]   = { 0, /* 1-byte */  1, /* 2-byte */  1, /* 3-byte */  1 };

bool RepeaterMesh::isLooped(const mesh::Packet* packet, const uint8_t max_counters[]) {
    uint8_t hash_size = packet->getPathHashSize();
    uint8_t hash_count = packet->getPathHashCount();
    uint8_t n = 0;
    const uint8_t* path = packet->path;
    while (hash_count > 0) {
        if (self_id.isHashMatch(path, hash_size)) n++;
        hash_count--;
        path += hash_size;
    }
    return n >= max_counters[hash_size];
}

void RepeaterMesh::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size) {
    if (scope.isNull()) {
        sendFlood(pkt, delay_millis, path_hash_size);
    } else {
        uint16_t codes[2];
        codes[0] = scope.calcTransportCode(pkt);
        codes[1] = 0;  // REVISIT: set to 'home' Region, for sender/return region?
        sendFlood(pkt, codes, delay_millis, path_hash_size);
    }
}

void RepeaterMesh::sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size) {
    if (recv_pkt_region && !recv_pkt_region->isWildcard()) {  // if _request_ packet scope is known, send reply with same scope
        TransportKey scope;
        if (region_map.getTransportKeysFor(*recv_pkt_region, &scope, 1) > 0) {
            sendFloodScoped(scope, packet, delay_millis, path_hash_size);
        } else {
            sendFlood(packet, delay_millis, path_hash_size);  // send un-scoped
        }
    } else {
        sendFlood(packet, delay_millis, path_hash_size);  // send un-scoped
    }
}

bool RepeaterMesh::allowPacketForward(const mesh::Packet* packet) {
    if (_prefs.disable_fwd) return false;
    if (packet->isRouteFlood() && packet->getPathHashCount() >= _prefs.flood_max) return false;
    if (packet->isRouteFlood() && recv_pkt_region == nullptr) return false;
    if (packet->isRouteFlood() && _prefs.loop_detect != LOOP_DETECT_OFF) {
        const uint8_t* maximums;
        if (_prefs.loop_detect == LOOP_DETECT_MINIMAL) {
            maximums = max_loop_minimal;
        } else if (_prefs.loop_detect == LOOP_DETECT_MODERATE) {
            maximums = max_loop_moderate;
        } else {
            maximums = max_loop_strict;
        }
        if (isLooped(packet, maximums)) {
            MESH_DEBUG_PRINTLN("allowPacketForward: FLOOD packet loop detected!");
            return false;
        }
    }
    return true;
}

const char* RepeaterMesh::getLogDateTime() {
    static char tmp[48];
    uint32_t now = getRTCClock()->getCurrentTime();
    /* Match Arduino format: "HH:MM:SS - D/M/YYYY U" */
    time_t t = (time_t)now;
    struct tm* tm = gmtime(&t);
    if (tm) {
        snprintf(tmp, sizeof(tmp), "%02d:%02d:%02d - %d/%d/%d U",
                tm->tm_hour, tm->tm_min, tm->tm_sec,
                tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900);
    } else {
        snprintf(tmp, sizeof(tmp), "%u", now);
    }
    return tmp;
}

void RepeaterMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
    /* Arduino-compatible RAW packet hex dump */
    static char hex_buf[MAX_TRANS_UNIT * 2 + 1];
    mesh::Utils::toHex(hex_buf, raw, len);
    printk("%s RAW: %s\n", getLogDateTime(), hex_buf);
#endif
    (void)snr;
    (void)rssi;
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
    _uplink_last_rssi = rssi;
    _uplink_last_raw_len = len <= (int)sizeof(_uplink_last_raw) ? len : (int)sizeof(_uplink_last_raw);
    if (_uplink_last_raw_len > 0) {
        memcpy(_uplink_last_raw, raw, _uplink_last_raw_len);
    }
#endif
}

void RepeaterMesh::logRx(mesh::Packet* pkt, int len, float score) {
    if (_logging) {
        LOG_INF("RX len=%d type=%d route=%s payload_len=%d SNR=%d RSSI=%d",
                len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
                pkt->payload_len, (int)_radio->getLastSNR(), (int)_radio->getLastRSSI());
    }
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
    _uplink_last_score = score;
    publishUplinkPacket(pkt);
#endif
}

void RepeaterMesh::logTx(mesh::Packet* pkt, int len) {
    if (_logging) {
        LOG_INF("TX len=%d type=%d route=%s payload_len=%d",
                len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
                pkt->payload_len);
    }
}

void RepeaterMesh::logTxFail(mesh::Packet* pkt, int len) {
    if (_logging) {
        LOG_WRN("TX FAIL len=%d type=%d route=%s payload_len=%d",
                len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
                pkt->payload_len);
    }
}

uint32_t RepeaterMesh::getRetransmitDelay(const mesh::Packet* packet) {
    return computeAdaptiveFloodDelay(packet);
}

uint32_t RepeaterMesh::getDirectRetransmitDelay(const mesh::Packet* packet) {
    return computeAdaptiveDirectDelay(packet);
}

bool RepeaterMesh::filterRecvFloodPacket(mesh::Packet* pkt) {
    if (pkt->getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD) {
        recv_pkt_region = region_map.findMatch(pkt, REGION_DENY_FLOOD);
    } else if (pkt->getRouteType() == ROUTE_TYPE_FLOOD) {
        if (region_map.getWildcard().flags & REGION_DENY_FLOOD) {
            recv_pkt_region = nullptr;
        } else {
            recv_pkt_region = &region_map.getWildcard();
        }
    } else {
        recv_pkt_region = nullptr;
    }
    return false;
}

void RepeaterMesh::onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) {
    if (packet->getPayloadType() == PAYLOAD_TYPE_ANON_REQ) {
        uint32_t timestamp;
        memcpy(&timestamp, data, 4);

        data[len] = 0;
        uint8_t reply_len;

        reply_path_len = OUT_PATH_UNKNOWN;
        if (data[4] == 0 || data[4] >= ' ') {
            reply_len = handleLoginReq(sender, secret, timestamp, &data[4], packet->isRouteFlood());
        } else if (data[4] == ANON_REQ_TYPE_REGIONS && packet->isRouteDirect()) {
            reply_len = handleAnonRegionsReq(sender, timestamp, &data[5], (len > 5) ? (len - 5) : 0);
        } else if (data[4] == ANON_REQ_TYPE_OWNER && packet->isRouteDirect()) {
            reply_len = handleAnonOwnerReq(sender, timestamp, &data[5], (len > 5) ? (len - 5) : 0);
        } else if (data[4] == ANON_REQ_TYPE_BASIC && packet->isRouteDirect()) {
            reply_len = handleAnonClockReq(sender, timestamp, &data[5], (len > 5) ? (len - 5) : 0);
        } else {
            reply_len = 0;
        }

        if (reply_len == 0) return;

        if (packet->isRouteFlood()) {
            mesh::Packet* path = createPathReturn(sender, secret, packet->path, packet->path_len,
                                                  PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
            if (path) sendFloodReply(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
        } else if (reply_path_len == OUT_PATH_UNKNOWN) {
            mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
            if (reply) sendFloodReply(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
        } else {
            mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
            if (reply) sendDirect(reply, reply_path, reply_path_len, SERVER_RESPONSE_DELAY);
        }
    }
}

int RepeaterMesh::searchPeersByHash(const uint8_t* hash) {
    int n = 0;
    for (int i = 0; i < acl.getNumClients(); i++) {
        if (acl.getClientByIdx(i)->id.isHashMatch(hash)) {
            matching_peer_indexes[n++] = i;
        }
    }
    return n;
}

void RepeaterMesh::getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) {
    int i = matching_peer_indexes[peer_idx];
    if (i >= 0 && i < acl.getNumClients()) {
        memcpy(dest_secret, acl.getClientByIdx(i)->shared_secret, PUB_KEY_SIZE);
    }
}

static bool isShare(const mesh::Packet* packet) {
    if (packet->hasTransportCodes()) {
        return packet->transport_codes[0] == 0 && packet->transport_codes[1] == 0;
    }
    return false;
}

void RepeaterMesh::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp,
                                const uint8_t* app_data, size_t app_data_len) {
    mesh::Mesh::onAdvertRecv(packet, id, timestamp, app_data, app_data_len);

    if (packet->getPathHashCount() == 0 && !isShare(packet)) {
        AdvertDataParser parser(app_data, app_data_len);
        if (parser.isValid() && parser.getType() == ADV_TYPE_REPEATER) {
            putNeighbour(id, timestamp, packet->getSNR());
        }
    }
}

void RepeaterMesh::onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx,
                                  const uint8_t* secret, uint8_t* data, size_t len) {
    int i = matching_peer_indexes[sender_idx];
    if (i < 0 || i >= acl.getNumClients()) {
        LOG_WRN("onPeerDataRecv: invalid peer idx: %d", i);
        return;
    }
    ClientInfo* client = acl.getClientByIdx(i);

    if (type == PAYLOAD_TYPE_REQ) {
        uint32_t timestamp;
        memcpy(&timestamp, data, 4);

        if (timestamp > client->last_timestamp) {
            int reply_len = handleRequest(client, timestamp, &data[4], len - 4);
            if (reply_len == 0) return;

            client->last_timestamp = timestamp;
            client->last_activity = getRTCClock()->getCurrentTime();

            if (packet->isRouteFlood()) {
                mesh::Packet* path = createPathReturn(client->id, secret, packet->path, packet->path_len,
                                                      PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
                if (path) sendFloodReply(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
            } else {
                mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, client->id, secret, reply_data, reply_len);
                if (reply) {
                    if (client->out_path_len != OUT_PATH_UNKNOWN) {
                        sendDirect(reply, client->out_path, client->out_path_len, SERVER_RESPONSE_DELAY);
                    } else {
                        sendFloodReply(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
                    }
                }
            }
        } else {
            LOG_DBG("onPeerDataRecv: possible replay attack");
        }
    } else if (type == PAYLOAD_TYPE_TXT_MSG && len > 5 && client->isAdmin()) {
        uint32_t sender_timestamp;
        memcpy(&sender_timestamp, data, 4);
        uint8_t flags = (data[4] >> 2);

        if (!(flags == TXT_TYPE_PLAIN || flags == TXT_TYPE_CLI_DATA)) {
            LOG_DBG("onPeerDataRecv: unsupported text type: flags=%02x", flags);
        } else if (sender_timestamp >= client->last_timestamp) {
            bool is_retry = (sender_timestamp == client->last_timestamp);
            client->last_timestamp = sender_timestamp;
            client->last_activity = getRTCClock()->getCurrentTime();

            data[len] = 0;

            if (flags == TXT_TYPE_PLAIN) {
                uint32_t ack_hash;
                mesh::Utils::sha256((uint8_t*)&ack_hash, 4, data, 5 + strlen((char*)&data[5]),
                                   client->id.pub_key, PUB_KEY_SIZE);
                mesh::Packet* ack = createAck(ack_hash);
                if (ack) {
                    if (client->out_path_len == OUT_PATH_UNKNOWN) {
                        sendFloodReply(ack, TXT_ACK_DELAY, packet->getPathHashSize());
                    } else {
                        sendDirect(ack, client->out_path, client->out_path_len, TXT_ACK_DELAY);
                    }
                }
            }

            uint8_t temp[166];
            char* command = (char*)&data[5];
            char* reply = (char*)&temp[5];
            if (is_retry) {
                *reply = 0;
            } else {
                handleCommand(sender_timestamp, command, reply);
            }

            int text_len = strlen(reply);
            if (text_len > 0) {
                uint32_t timestamp = getRTCClock()->getCurrentTimeUnique();
                if (timestamp == sender_timestamp) timestamp++;
                memcpy(temp, &timestamp, 4);
                temp[4] = (TXT_TYPE_CLI_DATA << 2);

                auto reply_pkt = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, secret, temp, 5 + text_len);
                if (reply_pkt) {
                    if (client->out_path_len == OUT_PATH_UNKNOWN) {
                        sendFloodReply(reply_pkt, CLI_REPLY_DELAY_MILLIS, packet->getPathHashSize());
                    } else {
                        sendDirect(reply_pkt, client->out_path, client->out_path_len, CLI_REPLY_DELAY_MILLIS);
                    }
                }
            }
        } else {
            LOG_DBG("onPeerDataRecv: possible replay attack");
        }
    }
}

bool RepeaterMesh::onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret,
                                  uint8_t* path, uint8_t path_len, uint8_t extra_type,
                                  uint8_t* extra, uint8_t extra_len) {
    int i = matching_peer_indexes[sender_idx];
    if (i >= 0 && i < acl.getNumClients()) {
        LOG_DBG("PATH to client, path_len=%d", path_len);
        auto client = acl.getClientByIdx(i);
        /* path source bounded by upstream packet parser; client->out_path
         * is MAX_PATH_SIZE-sized. */
        client->out_path_len = mesh::Packet::copyPath(client->out_path, path, MAX_PATH_SIZE, path_len);
        client->last_activity = getRTCClock()->getCurrentTime();
    }
    return false;
}

void RepeaterMesh::onControlDataRecv(mesh::Packet* packet) {
    uint8_t type = packet->payload[0] & 0xF0;
    if (type == CTL_TYPE_NODE_DISCOVER_REQ && packet->payload_len >= 6 &&
        !_prefs.disable_fwd && discover_limiter.allow(getRTCClock()->getCurrentTime())) {
        int i = 1;
        uint8_t filter = packet->payload[i++];
        uint32_t tag;
        memcpy(&tag, &packet->payload[i], 4);
        i += 4;
        uint32_t since = 0;
        if (packet->payload_len >= i + 4) {
            memcpy(&since, &packet->payload[i], 4);
            i += 4;
        }

        if ((filter & (1 << ADV_TYPE_REPEATER)) != 0 && _prefs.discovery_mod_timestamp >= since) {
            bool prefix_only = packet->payload[0] & 1;
            uint8_t data[6 + PUB_KEY_SIZE];
            data[0] = CTL_TYPE_NODE_DISCOVER_RESP | ADV_TYPE_REPEATER;
            data[1] = packet->_snr;
            memcpy(&data[2], &tag, 4);
            memcpy(&data[6], self_id.pub_key, PUB_KEY_SIZE);
            auto resp = createControlData(data, prefix_only ? 6 + 8 : 6 + PUB_KEY_SIZE);
            if (resp) {
                sendZeroHop(resp, getRetransmitDelay(resp) * 4);
            }
        }
    } else if (type == CTL_TYPE_NODE_DISCOVER_RESP && packet->payload_len >= 6) {
        uint8_t node_type = packet->payload[0] & 0x0F;
        if (node_type != ADV_TYPE_REPEATER) return;
        if (packet->payload_len < 6 + PUB_KEY_SIZE) return;

        /* Only accept responses matching our pending discover tag */
        if (pending_discover_tag == 0 || millisHasNowPassed(pending_discover_until)) {
            pending_discover_tag = 0;
            return;
        }
        uint32_t tag;
        memcpy(&tag, &packet->payload[2], 4);
        if (tag != pending_discover_tag) return;

        mesh::Identity id(&packet->payload[6]);
        if (id.matches(self_id)) return;
        putNeighbour(id, getRTCClock()->getCurrentTime(), packet->getSNR());
    }
}

void RepeaterMesh::sendNodeDiscoverReq() {
    uint8_t data[10];
    data[0] = CTL_TYPE_NODE_DISCOVER_REQ;  // prefix_only=0
    data[1] = (1 << ADV_TYPE_REPEATER);
    getRNG()->random(&data[2], 4);  // tag
    memcpy(&pending_discover_tag, &data[2], 4);
    pending_discover_until = futureMillis(30000);
    uint32_t since = 0;
    memcpy(&data[6], &since, 4);

    auto pkt = createControlData(data, sizeof(data));
    if (pkt) {
        sendZeroHop(pkt);
    }
}

RepeaterMesh::RepeaterMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
                           mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
    : mesh::Mesh(radio, ms, rng, rtc, *new mesh::StaticPoolPacketManager(), tables),
      _board(board),
      _cli(board, rtc, acl, &_prefs, this),
      region_map(key_store), temp_map(key_store),
      discover_limiter(4, 120),
      anon_limiter(4, 180),
      /* Failed-login rate limit: 4 wrong-password attempts per 180s. Matches
       * anon_limiter's shape so legitimate operators don't notice; brute-force
       * attempts hit the cap quickly and trip the LOG_WRN below. Global rate
       * (not per-sender) — trade-off documented in CRYPTO_AUDIT_INDEX.md
       * Phase 4 (mitigation for upstream MeshCore#2556). */
      login_fail_limiter(4, 180) {

    _store = nullptr;
    last_millis = 0;
    uptime_millis = 0;
    next_local_advert = next_flood_advert = 0;
    dirty_contacts_expiry = 0;
    set_radio_at = revert_radio_at = 0;
    _logging = false;
    region_load_active = false;
    recv_pkt_region = nullptr;
    memset(default_scope.key, 0, sizeof(default_scope.key));
    pending_discover_tag = 0;
    pending_discover_until = 0;

#if MAX_NEIGHBOURS > 0
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        neighbours[i].clear();
    }
#endif

    initNodePrefs(&_prefs);
    strcpy(_prefs.node_name, "Repeater");
    _prefs.advert_loc_policy = ADVERT_LOC_PREFS;  // Repeaters always advertise prefs coordinates
    _prefs.loop_detect = LOOP_DETECT_MODERATE;
    _prefs.path_hash_mode = 1;
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
    memset(&_uplink_creds, 0, sizeof(_uplink_creds));
    observer_creds_init(&_uplink_creds);
    _uplink_reboot_required = false;
    memset(_uplink_pubkey_hex, 0, sizeof(_uplink_pubkey_hex));
    memset(_uplink_packets_topic, 0, sizeof(_uplink_packets_topic));
    memset(_uplink_status_topic, 0, sizeof(_uplink_status_topic));
    _uplink_last_score = 0.0f;
    _uplink_last_rssi = 0.0f;
    _uplink_last_raw_len = 0;
    _uplink_next_status_at = 0;
#endif
}

void RepeaterMesh::begin(RepeaterDataStore* store) {
    _store = store;

    /* Prefs and identity are loaded by the caller (main_repeater.cpp) before
     * begin() — the radio reads freq/bw/sf/cr through _prefs during
     * Mesh::begin() → Dispatcher::begin() → Radio::begin(). */
    mesh::Mesh::begin();
    _contention.setBackoffMultiplier(_prefs.backoff_multiplier);
#ifdef CONFIG_ZEPHCORE_APC
    _power_ctrl.setSF(_prefs.sf);
    _power_ctrl.setTargetMargin(_prefs.apc_margin);
    _power_ctrl.setEnabled(_prefs.apc_enabled != 0);
#endif
    acl.load(_store->getAclPath(), self_id);
    region_map.load(_store->getRegionsPath());

    // establish default-scope from persisted default region (if any)
    {
        RegionEntry* r = region_map.getDefaultRegion();
        if (r) {
            region_map.getTransportKeysFor(*r, &default_scope, 1);
        }
    }

    /* NOTE: Radio configuration is handled by SX126xRadio adapter using
     * LoRaConfig defaults. The repeater uses the same radio params as companion.
     * Dynamic radio reconfiguration (via CLI) is not yet supported - radio
     * uses compile-time defaults from LoRaConfig. This avoids EBUSY errors
     * from trying to reconfigure while radio is in async RX mode. */

    updateAdvertTimer();
    updateFloodAdvertTimer();

    _board.setAdcMultiplier(_prefs.adc_multiplier);

    LOG_INF("RepeaterMesh started: %s (freq=%.2f bw=%.0f sf=%d cr=%d)",
            _prefs.node_name, (double)_prefs.freq, (double)_prefs.bw, _prefs.sf, _prefs.cr);
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
    observer_creds_load(&_uplink_creds, _store->getBasePath());
    mesh::Utils::toHex(_uplink_pubkey_hex, self_id.pub_key, PUB_KEY_SIZE);
    _uplink_pubkey_hex[PUB_KEY_SIZE * 2] = '\0';

    const char *iata = _uplink_creds.mqtt_iata[0] ? _uplink_creds.mqtt_iata : "XXX";
    snprintf(_uplink_packets_topic, sizeof(_uplink_packets_topic),
             "meshcore/%s/%s/packets", iata, _uplink_pubkey_hex);
    snprintf(_uplink_status_topic, sizeof(_uplink_status_topic),
             "meshcore/%s/%s/status", iata, _uplink_pubkey_hex);

    if (isUplinkEnabled() && _uplink_creds.wifi_ssid[0] && _uplink_creds.mqtt_host[0]) {
        s_uplink_mesh = this;
        zc_wifi_station_start(&_uplink_creds, uplink_time_sync_cb);
        mqtt_publisher_start(&_uplink_creds, _prefs.node_name,
                             _uplink_status_topic, _uplink_packets_topic);
        mqtt_publisher_set_connect_cb([]() {
            if (s_uplink_mesh) {
                s_uplink_mesh->publishUplinkStatus("online");
            }
        });
        _uplink_next_status_at = futureMillis(300000);
        LOG_INF("Repeater uplink active: %s", _uplink_packets_topic);
    } else {
        LOG_INF("Repeater uplink inactive");
    }
#endif
}

double RepeaterMesh::getNodeLat() const {
    struct gps_position pos;
    if (gps_get_last_known_position(&pos)) {
        return pos.latitude_ndeg / 1e9;
    }
    return _prefs.node_lat;
}

double RepeaterMesh::getNodeLon() const {
    struct gps_position pos;
    if (gps_get_last_known_position(&pos)) {
        return pos.longitude_ndeg / 1e9;
    }
    return _prefs.node_lon;
}

bool RepeaterMesh::setGpsEnabled(bool enabled) {
    if (!gps_is_available()) return false;
    gps_enable(enabled);
    return true;
}

bool RepeaterMesh::isGpsEnabled() const {
    return gps_is_enabled();
}

void RepeaterMesh::formatGpsStatsReply(char* reply) {
    if (!gps_is_enabled()) {
        strcpy(reply, "off");
        return;
    }

    struct gps_state_info gsi;
    gps_get_state_info(&gsi);

    static const char* const state_str[] = { "off", "standby", "acquiring" };
    const char* state = gsi.state < 3 ? state_str[gsi.state] : "unknown";

    struct gps_position pos;
    bool has_pos = gps_get_last_known_position(&pos);

    if (has_pos) {
        snprintf(reply, CLI_REPLY_SIZE,
            "on state=%s sats=%u fix=%us ago lat=%.6f lon=%.6f",
            state, gsi.satellites, gsi.last_fix_age_s,
            pos.latitude_ndeg / 1e9, pos.longitude_ndeg / 1e9);
    } else if (gsi.next_search_s > 0) {
        snprintf(reply, CLI_REPLY_SIZE,
            "on state=%s sats=%u no fix next=%us",
            state, gsi.satellites, gsi.next_search_s);
    } else {
        snprintf(reply, CLI_REPLY_SIZE,
            "on state=%s sats=%u no fix",
            state, gsi.satellites);
    }
}

void RepeaterMesh::savePrefs() {
    if (_store) {
        _store->savePrefs(_prefs);
    }
}

void RepeaterMesh::applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) {
    set_radio_at = futureMillis(2000);
    pending_freq = freq;
    pending_bw = bw;
    pending_sf = sf;
    pending_cr = cr;
    revert_radio_at = futureMillis(2000 + timeout_mins * 60 * 1000);
}

void RepeaterMesh::freezeRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) {
    auto& radio = getRadioDriver(_radio);
    if (!radio.hasRadioOverride()) {
        radio.setRadioOverride(freq, bw, sf, cr);
    }
}

bool RepeaterMesh::formatFileSystem() {
    if (_store) {
        return _store->formatFileSystem();
    }
    return false;
}

void RepeaterMesh::sendSelfAdvertisement(int delay_millis, bool flood) {
    mesh::Packet* pkt = createSelfAdvert();
    if (pkt) {
        if (flood) {
            sendFloodScoped(default_scope, pkt, delay_millis, _prefs.path_hash_mode + 1);
        } else {
            sendZeroHop(pkt, delay_millis);
        }
    } else {
        LOG_ERR("Unable to create advertisement packet");
    }
}

void RepeaterMesh::updateAdvertTimer() {
    if (_prefs.advert_interval > 0) {
        next_local_advert = futureMillis(((uint32_t)_prefs.advert_interval) * 2 * 60 * 1000);
    } else {
        next_local_advert = 0;
    }
}

void RepeaterMesh::updateFloodAdvertTimer() {
    if (_prefs.flood_advert_interval > 0) {
        next_flood_advert = futureMillis(((uint32_t)_prefs.flood_advert_interval) * 60 * 60 * 1000);
    } else {
        next_flood_advert = 0;
    }
}

void RepeaterMesh::eraseLogFile() {
    // Logging to file not implemented in Zephyr version
    LOG_INF("Log erased");
}

void RepeaterMesh::dumpLogFile() {
    // Logging to file not implemented in Zephyr version
    LOG_INF("Log dump not implemented");
}

void RepeaterMesh::setTxPower(int8_t power_dbm) {
    radio_set_tx_power(power_dbm);
}

void RepeaterMesh::formatNeighborsReply(char* reply) {
    char* dp = reply;

#if MAX_NEIGHBOURS > 0
    int16_t neighbours_count = 0;
    NeighbourInfo* sorted_neighbours[MAX_NEIGHBOURS];
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        if (neighbours[i].heard_timestamp > 0) {
            sorted_neighbours[neighbours_count++] = &neighbours[i];
        }
    }

    simple_sort(sorted_neighbours, (int)neighbours_count,
        [](const NeighbourInfo* a, const NeighbourInfo* b) {
            return a->heard_timestamp > b->heard_timestamp;
        });

    for (int i = 0; i < neighbours_count && dp - reply < 134; i++) {
        NeighbourInfo* neighbour = sorted_neighbours[i];

        if (i > 0) *dp++ = '\n';

        char hex[10];
        mesh::Utils::toHex(hex, neighbour->id.pub_key, 4);

        uint32_t secs_ago = getRTCClock()->getCurrentTime() - neighbour->heard_timestamp;
        sprintf(dp, "%s:%u:%d", hex, secs_ago, neighbour->snr);
        while (*dp) dp++;
    }
#endif

    if (dp == reply) {
        strcpy(dp, "-none-");
        dp += 6;
    }
    *dp = 0;
}

void RepeaterMesh::removeNeighbor(const uint8_t* pubkey, int key_len) {
#if MAX_NEIGHBOURS > 0
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        if (memcmp(neighbours[i].id.pub_key, pubkey, key_len) == 0) {
            neighbours[i].clear();
        }
    }
#endif
}

void RepeaterMesh::formatStatsReply(char* reply) {
    StatsFormatHelper::formatCoreStats(reply, _board, *_ms, _err_flags, _mgr);
}

void RepeaterMesh::formatRadioStatsReply(char* reply) {
    auto& radio_driver = getRadioDriver(_radio);
    StatsFormatHelper::formatRadioStats(reply, _radio, radio_driver, getTotalAirTime(), getReceiveAirTime());
}

void RepeaterMesh::formatPacketStatsReply(char* reply) {
    auto& radio_driver = getRadioDriver(_radio);
    StatsFormatHelper::formatPacketStats(reply, radio_driver, getNumSentFlood(), getNumSentDirect(),
                                         getNumRecvFlood(), getNumRecvDirect());
}

void RepeaterMesh::saveIdentity(const mesh::LocalIdentity& new_id) {
    if (_store) {
        _store->saveIdentity(new_id);
    }
}

void RepeaterMesh::clearStats() {
    auto& radio_driver = getRadioDriver(_radio);
    radio_driver.resetStats();
    radio_driver.resetDutyCycleTimeoutRestarts();
    resetStats();
    ((mesh::SimpleMeshTables *)getTables())->resetStats();
}

uint32_t RepeaterMesh::getDutyCycleTimeoutRestarts() const {
    return getRadioDriver(_radio).getDutyCycleTimeoutRestarts();
}

void RepeaterMesh::resetDutyCycleTimeoutRestarts() {
    getRadioDriver(_radio).resetDutyCycleTimeoutRestarts();
}

/* Region-def CLI (handleRegionLoadLine / handleRegionCommand) and its static
 * parser helpers live in app/RepeaterRegionCLI.cpp. */

void RepeaterMesh::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
    if (region_load_active) {
        handleRegionLoadLine(command, reply);
        return;
    }

    while (*command == ' ') command++;

    if (strlen(command) > 4 && command[2] == '|') {
        memcpy(reply, command, 3);
        reply += 3;
        command += 3;
    }

#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
    if (handleUplinkCommand(command, reply)) {
        return;
    }
#endif

    // ACL commands - supports BOTH formats for app compatibility:
    //   Old Arduino: setperm {pubkey-hex} {permissions}   (pubkey is long, perms is short)
    //   MeshCore App: setperm {permissions} {pubkey-hex}  (perms is short 2-char hex, pubkey is long)
    // Detection: if first part is <= 2 chars, it's permissions; otherwise it's pubkey
    if (memcmp(command, "setperm ", 8) == 0) {
        char* first = &command[8];
        char* sp = strchr(first, ' ');
        if (sp == nullptr) {
            strcpy(reply, "Err - bad params");
        } else {
            *sp++ = 0;  // null terminate first part
            char* second = sp;

            // Detect format: if first part is short (1-2 chars), it's permissions
            int first_len = strlen(first);
            char* hex;
            uint8_t perms;

            if (first_len <= 2) {
                // App format: setperm {perms} {pubkey}
                perms = (uint8_t)strtol(first, nullptr, 16);
                hex = second;
            } else {
                // Arduino format: setperm {pubkey} {perms}
                hex = first;
                perms = (uint8_t)atoi(second);
            }

            uint8_t pubkey[PUB_KEY_SIZE];
            int hex_len = strlen(hex);
            if (hex_len > PUB_KEY_SIZE * 2) hex_len = PUB_KEY_SIZE * 2;
            if (mesh::Utils::fromHex(pubkey, hex_len / 2, hex)) {
                if (acl.applyPermissions(self_id, pubkey, hex_len / 2, perms)) {
                    if (!dirty_contacts_expiry) dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
                    strcpy(reply, "OK");
                } else {
                    strcpy(reply, "Err - invalid params");
                }
            } else {
                strcpy(reply, "Err - bad pubkey");
            }
        }
    } else if (sender_timestamp == 0 && strcmp(command, "get acl") == 0) {
        LOG_INF("ACL:");
        for (int i = 0; i < acl.getNumClients(); i++) {
            auto c = acl.getClientByIdx(i);
            if (c->permissions == 0) continue;
            char hex[PUB_KEY_SIZE * 2 + 1];
            mesh::Utils::toHex(hex, c->id.pub_key, PUB_KEY_SIZE);
            LOG_INF("  %02X %s", c->permissions, hex);
        }
        reply[0] = 0;
    } else if (memcmp(command, "region", 6) == 0) {
        handleRegionCommand(command, reply);
    } else if (memcmp(command, "discover.neighbors", 18) == 0) {
        const char* sub = command + 18;
        while (*sub == ' ') sub++;
        if (*sub != 0) {
            strcpy(reply, "Err - discover.neighbors has no options");
        } else {
            sendNodeDiscoverReq();
            strcpy(reply, "OK - Discover sent");
        }
    } else {
        _cli.handleCommand(sender_timestamp, command, reply);
    }
}

/* MQTT uplink methods (saveUplinkCreds / handleUplinkCommand /
 * publishUplinkPacket / publishUplinkStatus) live in app/RepeaterUplink.cpp,
 * compiled only when CONFIG_ZEPHCORE_REPEATER_UPLINK && CONFIG_MQTT_LIB.
 * The uplink init (WiFi/MQTT start + topic strings) stays in begin() above. */

void RepeaterMesh::loop() {
    mesh::Mesh::loop();

    if (next_flood_advert && millisHasNowPassed(next_flood_advert)) {
        mesh::Packet* pkt = createSelfAdvert();
        if (pkt) sendFloodScoped(default_scope, pkt, (uint32_t)0, _prefs.path_hash_mode + 1);
        updateFloodAdvertTimer();
        updateAdvertTimer();
    } else if (next_local_advert && millisHasNowPassed(next_local_advert)) {
        mesh::Packet* pkt = createSelfAdvert();
        if (pkt) sendZeroHop(pkt);
        updateAdvertTimer();
    }

    if (set_radio_at && millisHasNowPassed(set_radio_at)) {
        set_radio_at = 0;
        getRadioDriver(_radio).setRadioOverride(pending_freq, pending_bw, pending_sf, pending_cr);
        LOG_INF("Temp radio params applied");
    }

    if (revert_radio_at && millisHasNowPassed(revert_radio_at)) {
        revert_radio_at = 0;
        getRadioDriver(_radio).clearRadioOverride();
        LOG_INF("Radio params restored");
    }

    if (dirty_contacts_expiry && millisHasNowPassed(dirty_contacts_expiry)) {
        acl.save(_store->getAclPath());
        dirty_contacts_expiry = 0;
    }

#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
    if (_uplink_next_status_at && millisHasNowPassed(_uplink_next_status_at)) {
        publishUplinkStatus("online");
        _uplink_next_status_at = futureMillis(300000);
    }
#endif

    uint32_t now = k_uptime_get();
    uptime_millis += now - last_millis;
    last_millis = now;
}

bool RepeaterMesh::hasPendingWork() const {
    return _mgr->getOutboundTotal() > 0;
}
