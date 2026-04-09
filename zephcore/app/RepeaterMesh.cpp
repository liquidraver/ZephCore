/*
 * SPDX-License-Identifier: Apache-2.0
 * RepeaterMesh - LoRa mesh repeater implementation
 */

#include "RepeaterMesh.h"
#include <mesh/Utils.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
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

#define CTL_TYPE_NODE_DISCOVER_REQ   0x80
#define CTL_TYPE_NODE_DISCOVER_RESP  0x90

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
        if (strcmp((char*)data, _prefs.password) == 0) {
            perms = PERM_ACL_ADMIN;
        } else if (strcmp((char*)data, _prefs.guest_password) == 0) {
            perms = PERM_ACL_GUEST;
        } else {
            LOG_WRN("Invalid password");
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

uint8_t RepeaterMesh::handleAnonRegionsReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
    if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
        reply_path_len = *data++;
        mesh::Packet::copyPath(reply_path, data, reply_path_len);

        memcpy(reply_data, &sender_timestamp, 4);
        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(&reply_data[4], &now, 4);

        return 8 + region_map.exportNamesTo((char*)&reply_data[8], sizeof(reply_data) - 12, REGION_DENY_FLOOD);
    }
    return 0;
}

uint8_t RepeaterMesh::handleAnonOwnerReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
    if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
        reply_path_len = *data++;
        mesh::Packet::copyPath(reply_path, data, reply_path_len);

        memcpy(reply_data, &sender_timestamp, 4);
        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(&reply_data[4], &now, 4);
        sprintf((char*)&reply_data[8], "%s\n%s", _prefs.node_name, _prefs.owner_info);

        return 8 + strlen((char*)&reply_data[8]);
    }
    return 0;
}

uint8_t RepeaterMesh::handleAnonClockReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
    if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
        reply_path_len = *data++;
        mesh::Packet::copyPath(reply_path, data, reply_path_len);

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
    float factor = getContentionTracker().getFloodDelayFactor();
    uint32_t t = (uint32_t)(_radio->getEstAirtimeFor(
        packet->getPathByteLen() + packet->payload_len + 2) * factor);
    uint32_t max_jitter = 5 * t;
    /* Cap jitter to 2000ms to avoid excessive latency in very dense areas.
     * Reactive backoff will fine-tune further if needed. */
    if (max_jitter > 2000) max_jitter = 2000;
    /* Floor: give downstream nodes time to finish RX processing
     * and return to RX mode before we TX (~20ms settle) */
    return 20 + getRNG()->nextInt(0, max_jitter + 1);
}

uint32_t RepeaterMesh::getDirectRetransmitDelay(const mesh::Packet* packet) {
    uint32_t t = _radio->getEstAirtimeFor(
        packet->getPathByteLen() + packet->payload_len + 2);
    /* Floor: give downstream nodes time to finish RX processing
     * and return to RX mode before we TX (~20ms settle + jitter) */
    return 20 + getRNG()->nextInt(0, t / 10 + 1);
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
            reply_len = handleAnonRegionsReq(sender, timestamp, &data[5]);
        } else if (data[4] == ANON_REQ_TYPE_OWNER && packet->isRouteDirect()) {
            reply_len = handleAnonOwnerReq(sender, timestamp, &data[5]);
        } else if (data[4] == ANON_REQ_TYPE_BASIC && packet->isRouteDirect()) {
            reply_len = handleAnonClockReq(sender, timestamp, &data[5]);
        } else {
            reply_len = 0;
        }

        if (reply_len == 0) return;

        if (packet->isRouteFlood()) {
            mesh::Packet* path = createPathReturn(sender, secret, packet->path, packet->path_len,
                                                  PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
            if (path) sendFlood(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
        } else if (reply_path_len == OUT_PATH_UNKNOWN) {
            mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
            if (reply) sendFlood(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
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
                if (path) sendFlood(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
            } else {
                mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, client->id, secret, reply_data, reply_len);
                if (reply) {
                    if (client->out_path_len != OUT_PATH_UNKNOWN) {
                        sendDirect(reply, client->out_path, client->out_path_len, SERVER_RESPONSE_DELAY);
                    } else {
                        sendFlood(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
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
                        sendFlood(ack, TXT_ACK_DELAY, packet->getPathHashSize());
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
                        sendFlood(reply_pkt, CLI_REPLY_DELAY_MILLIS, packet->getPathHashSize());
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
        client->out_path_len = mesh::Packet::copyPath(client->out_path, path, path_len);
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
      anon_limiter(4, 180) {

    _store = nullptr;
    last_millis = 0;
    uptime_millis = 0;
    next_local_advert = next_flood_advert = 0;
    dirty_contacts_expiry = 0;
    set_radio_at = revert_radio_at = 0;
    _logging = false;
    region_load_active = false;
    recv_pkt_region = nullptr;
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
    mesh::Mesh::begin();
    _store = store;

    /* Load persisted data.
     * NOTE: Identity is loaded in main_repeater.cpp before begin() is called,
     * so we skip loading it here (self_id should already be set). */
    _store->loadPrefs(_prefs);
    _contention.setBackoffMultiplier(_prefs.backoff_multiplier);
#ifdef CONFIG_ZEPHCORE_APC
    _power_ctrl.setSF(_prefs.sf);
    _power_ctrl.setTargetMargin(_prefs.apc_margin);
    _power_ctrl.setEnabled(_prefs.apc_enabled != 0);
#endif
    acl.load(_store->getAclPath(), self_id);
    region_map.load(_store->getRegionsPath());

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
            sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);
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
    resetStats();
    ((mesh::SimpleMeshTables *)getTables())->resetStats();
}

void RepeaterMesh::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
    if (region_load_active) {
        if (StrHelper::isBlank(command)) {
            region_map = temp_map;
            region_load_active = false;
            sprintf(reply, "OK - loaded %d regions", region_map.getCount());
        } else {
            char* np = command;
            while (*np == ' ') np++;
            int indent = np - command;

            char* ep = np;
            while (RegionMap::is_name_char(*ep)) ep++;
            if (*ep) { *ep++ = 0; }

            while (*ep && *ep != 'F') ep++;

            if (indent > 0 && indent < 8 && strlen(np) > 0) {
                auto parent = load_stack[indent - 1];
                if (parent) {
                    auto old = region_map.findByName(np);
                    auto nw = temp_map.putRegion(np, parent->id, old ? old->id : 0);
                    if (nw) {
                        nw->flags = old ? old->flags : (*ep == 'F' ? 0 : REGION_DENY_FLOOD);
                        load_stack[indent] = nw;
                    }
                }
            }
            reply[0] = 0;
        }
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
        reply[0] = 0;
        const char* parts[4];
        int n = mesh::Utils::parseTextParts(command, parts, 4, ' ');

        if (n == 1) {
            region_map.exportTo(reply, 160);
        } else if (n >= 2 && strcmp(parts[1], "load") == 0) {
            temp_map.resetFrom(region_map);
            memset(load_stack, 0, sizeof(load_stack));
            load_stack[0] = &temp_map.getWildcard();
            region_load_active = true;
        } else if (n >= 2 && strcmp(parts[1], "save") == 0) {
            _prefs.discovery_mod_timestamp = getRTCClock()->getCurrentTime();
            savePrefs();
            bool success = region_map.save(_store->getRegionsPath());
            strcpy(reply, success ? "OK" : "Err - save failed");
        } else if (n >= 3 && strcmp(parts[1], "allowf") == 0) {
            auto region = region_map.findByNamePrefix(parts[2]);
            if (region) {
                region->flags &= ~REGION_DENY_FLOOD;
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Err - unknown region");
            }
        } else if (n >= 3 && strcmp(parts[1], "denyf") == 0) {
            auto region = region_map.findByNamePrefix(parts[2]);
            if (region) {
                region->flags |= REGION_DENY_FLOOD;
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Err - unknown region");
            }
        } else if (n >= 3 && strcmp(parts[1], "get") == 0) {
            auto region = region_map.findByNamePrefix(parts[2]);
            if (region) {
                auto parent = region_map.findById(region->parent);
                if (parent && parent->id != 0) {
                    sprintf(reply, " %s (%s) %s", region->name, parent->name, (region->flags & REGION_DENY_FLOOD) ? "" : "F");
                } else {
                    sprintf(reply, " %s %s", region->name, (region->flags & REGION_DENY_FLOOD) ? "" : "F");
                }
            } else {
                strcpy(reply, "Err - unknown region");
            }
        } else if (n >= 3 && strcmp(parts[1], "home") == 0) {
            auto home = region_map.findByNamePrefix(parts[2]);
            if (home) {
                region_map.setHomeRegion(home);
                sprintf(reply, " home is now %s", home->name);
            } else {
                strcpy(reply, "Err - unknown region");
            }
        } else if (n == 2 && strcmp(parts[1], "home") == 0) {
            auto home = region_map.getHomeRegion();
            sprintf(reply, " home is %s", home ? home->name : "*");
        } else if (n >= 3 && strcmp(parts[1], "put") == 0) {
            auto parent = n >= 4 ? region_map.findByNamePrefix(parts[3]) : &region_map.getWildcard();
            if (parent == nullptr) {
                strcpy(reply, "Err - unknown parent");
            } else {
                auto region = region_map.putRegion(parts[2], parent->id);
                if (region == nullptr) {
                    strcpy(reply, "Err - unable to put");
                } else {
                    strcpy(reply, "OK");
                }
            }
        } else if (n >= 3 && strcmp(parts[1], "remove") == 0) {
            auto region = region_map.findByName(parts[2]);
            if (region) {
                if (region_map.removeRegion(*region)) {
                    strcpy(reply, "OK");
                } else {
                    strcpy(reply, "Err - not empty");
                }
            } else {
                strcpy(reply, "Err - not found");
            }
        } else if (n >= 3 && strcmp(parts[1], "list") == 0) {
            uint8_t mask = 0;
            bool invert = false;
            if (strcmp(parts[2], "allowed") == 0) {
                mask = REGION_DENY_FLOOD;
                invert = false;
            } else if (strcmp(parts[2], "denied") == 0) {
                mask = REGION_DENY_FLOOD;
                invert = true;
            } else {
                strcpy(reply, "Err - use 'allowed' or 'denied'");
                return;
            }
            int len = region_map.exportNamesTo(reply, 160, mask, invert);
            if (len == 0) {
                strcpy(reply, "-none-");
            }
        } else {
            strcpy(reply, "Err - ??");
        }
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

#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK) && IS_ENABLED(CONFIG_MQTT_LIB)
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
    struct tm tm_now;
    time_t t = (time_t)now_epoch;
    gmtime_r(&t, &tm_now);

    char ts_buf[48];
    char time_buf[12], date_buf[32];
    snprintf(ts_buf, sizeof(ts_buf), "%04d-%02d-%02dT%02d:%02d:%02d.000000",
             tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d",
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    snprintf(date_buf, sizeof(date_buf), "%d/%d/%04d",
             tm_now.tm_mday, tm_now.tm_mon + 1, tm_now.tm_year + 1900);

    static char json_buf[1024];
    int json_len = snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"type\":\"PACKET\","
        "\"origin\":\"%s\","
        "\"origin_id\":\"%s\","
        "\"timestamp\":\"%s\","
        "\"direction\":\"rx\","
        "\"time\":\"%s\","
        "\"date\":\"%s\","
        "\"len\":\"%d\","
        "\"packet_type\":\"%u\","
        "\"route\":\"%s\","
        "\"payload_len\":\"%u\","
        "\"raw\":\"%s\","
        "\"SNR\":\"%d\","
        "\"RSSI\":\"%d\","
        "\"score\":\"%d\","
        "\"hash\":\"%s\""
        "}",
        _prefs.node_name,
        _uplink_pubkey_hex,
        ts_buf,
        time_buf,
        date_buf,
        _uplink_last_raw_len,
        (unsigned)pkt->getPayloadType(),
        pkt->isRouteDirect() ? "D" : "F",
        (unsigned)pkt->payload_len,
        raw_hex,
        (int)pkt->getSNR(),
        (int)_uplink_last_rssi,
        (int)(_uplink_last_score * 1000.0f),
        hash_hex);

    if (json_len <= 0 || json_len >= (int)sizeof(json_buf)) {
        return;
    }
    mqtt_publisher_enqueue(_uplink_packets_topic, json_buf, json_len);
}

void RepeaterMesh::publishUplinkStatus(const char *status)
{
    if (!isUplinkEnabled()) return;
    if (_uplink_status_topic[0] == '\0') return;

    auto& radio_driver = getRadioDriver(_radio);
    uint32_t now_epoch = getRTCClock()->getCurrentTime();
    struct tm tm_now;
    time_t t = (time_t)now_epoch;
    gmtime_r(&t, &tm_now);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04d-%02d-%02dT%02d:%02d:%02d.000000",
             tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

    char radio_buf[48];
    snprintf(radio_buf, sizeof(radio_buf), "%.3f,%.1f,%u,%u",
             (double)_prefs.freq, (double)_prefs.bw,
             (unsigned)_prefs.sf, (unsigned)_prefs.cr);

    static char json_buf[768];
    int json_len = snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"status\":\"%s\","
        "\"timestamp\":\"%s\","
        "\"origin\":\"%s\","
        "\"origin_id\":\"%s\","
        "\"radio\":\"%s\","
        "\"model\":\"%s\","
        "\"firmware_version\":\"%s\","
        "\"client_version\":\"zephcoretomqtt/1.1\","
        "\"stats\":{"
            "\"battery_mv\":%u,"
            "\"uptime_secs\":%u,"
            "\"debug_flags\":%u,"
            "\"queue_len\":%u,"
            "\"noise_floor\":%d,"
            "\"tx_air_secs\":%u,"
            "\"rx_air_secs\":%u,"
            "\"recv_errors\":%u"
        "}"
        "}",
        status,
        ts_buf,
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
        (unsigned)radio_driver.getPacketsRecvErrors());

    if (json_len <= 0 || json_len >= (int)sizeof(json_buf)) {
        return;
    }
    mqtt_publisher_enqueue(_uplink_status_topic, json_buf, json_len);
}
#endif

void RepeaterMesh::loop() {
    mesh::Mesh::loop();

    if (next_flood_advert && millisHasNowPassed(next_flood_advert)) {
        mesh::Packet* pkt = createSelfAdvert();
        if (pkt) sendFlood(pkt, (uint32_t)0, _prefs.path_hash_mode + 1);
        updateFloodAdvertTimer();
        updateAdvertTimer();
    } else if (next_local_advert && millisHasNowPassed(next_local_advert)) {
        mesh::Packet* pkt = createSelfAdvert();
        if (pkt) sendZeroHop(pkt);
        updateAdvertTimer();
    }

    if (set_radio_at && millisHasNowPassed(set_radio_at)) {
        set_radio_at = 0;
        getRadioDriver(_radio).reconfigureWithParams(pending_freq, pending_bw, pending_sf, pending_cr);
        LOG_INF("Temp radio params applied");
    }

    if (revert_radio_at && millisHasNowPassed(revert_radio_at)) {
        revert_radio_at = 0;
        getRadioDriver(_radio).reconfigureWithParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
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
