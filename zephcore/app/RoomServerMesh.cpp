/*
 * SPDX-License-Identifier: Apache-2.0
 * RoomServerMesh - LoRa mesh shared-room (BBS) server
 *
 * A store-and-forward shared message room. Clients log in with an admin or
 * guest password and post messages; the server pushes each new post to all
 * other logged-in clients (round-robin, per-client sync cursor, ACK + retry).
 * Structured as a near-clone of RepeaterMesh (shared ACL/region/CLI/adverts)
 * with the post buffer + push engine added. Ported from upstream MeshCore's
 * simple_room_server.
 */

#include "RoomServerMesh.h"
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
static RoomServerMesh *s_uplink_mesh;
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

/* Room server: post push/sync timing. Upstream defaults are conservative
 * (PUSH_NOTIFY 2000, POST_SYNC 6) which adds ~6-7s of delivery lag; lowered
 * here for a more responsive room without changing wire formats. */
#define PUSH_NOTIFY_DELAY_MILLIS    1000
#define SYNC_PUSH_INTERVAL          1200
#define PUSH_ACK_TIMEOUT_FLOOD      12000
#define PUSH_TIMEOUT_BASE           4000
#define PUSH_ACK_TIMEOUT_FACTOR     2000
#define POST_SYNC_DELAY_SECS        2

/* Stats blob returned for REQ_TYPE_GET_STATUS on a room server.  Mirrors the
 * repeater's RepeaterStats but reports posted/pushed counts in the trailing
 * two fields (matches upstream MeshCore's ServerStats wire layout). */
struct ServerStats {
    uint16_t batt_milli_volts;
    uint16_t curr_tx_queue_len;
    int16_t noise_floor;
    int16_t last_rssi;
    uint32_t n_packets_recv;
    uint32_t n_packets_sent;
    uint32_t total_air_time_secs;
    uint32_t total_up_time_secs;
    uint32_t n_sent_flood, n_sent_direct;
    uint32_t n_recv_flood, n_recv_direct;
    uint16_t err_events;
    int16_t last_snr;
    uint16_t n_direct_dups, n_flood_dups;
    uint16_t n_posted, n_post_push;
};

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

void RoomServerMesh::putNeighbour(const mesh::Identity& id, uint32_t timestamp, float snr) {
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

int RoomServerMesh::handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len) {
    memcpy(reply_data, &sender_timestamp, 4);

    if (payload[0] == REQ_TYPE_GET_STATUS) {
        auto& radio_driver = getRadioDriver(_radio);
        ServerStats stats;
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
        stats.n_posted = _num_posted;
        stats.n_post_push = _num_post_pushes;
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

mesh::Packet* RoomServerMesh::createSelfAdvert() {
    uint8_t app_data[MAX_ADVERT_DATA_SIZE];
    uint8_t app_data_len = _cli.buildAdvertData(ADV_TYPE_ROOM, app_data);
    return createAdvert(self_id, app_data, app_data_len);
}

/* ---- Room server: shared-post buffer + push-to-client sync ---- */

void RoomServerMesh::addPost(ClientInfo* client, const char* postData) {
    posts[next_post_idx].author = client->id;
    strncpy(posts[next_post_idx].text, postData, MAX_POST_TEXT_LEN);
    posts[next_post_idx].text[MAX_POST_TEXT_LEN] = '\0';
    posts[next_post_idx].post_timestamp = getRTCClock()->getCurrentTimeUnique();
    next_post_idx = (next_post_idx + 1) % MAX_UNSYNCED_POSTS;

    next_push = futureMillis(PUSH_NOTIFY_DELAY_MILLIS);
    _num_posted++;
}

void RoomServerMesh::pushPostToClient(ClientInfo* client, PostInfo& post) {
    int len = 0;
    memcpy(&reply_data[len], &post.post_timestamp, 4);
    len += 4;

    uint8_t attempt;
    getRNG()->random(&attempt, 1);  // vary the packet hash (and ACK) across retries
    reply_data[len++] = (TXT_TYPE_SIGNED_PLAIN << 2) | (attempt & 3);

    memcpy(&reply_data[len], post.author.pub_key, 4);  // author prefix
    len += 4;

    int text_len = strlen(post.text);
    memcpy(&reply_data[len], post.text, text_len);
    len += text_len;

    /* Expected ACK = sha256(pushed message) keyed by the client's pubkey. */
    mesh::Utils::sha256((uint8_t*)&client->extra.room.pending_ack, 4, reply_data, len,
                       client->id.pub_key, PUB_KEY_SIZE);
    client->extra.room.push_post_timestamp = post.post_timestamp;

    mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, client->shared_secret, reply_data, len);
    if (reply) {
        if (client->out_path_len == OUT_PATH_UNKNOWN) {
            sendFloodScoped(default_scope, reply, (uint32_t)0, _prefs.path_hash_mode + 1);
            client->extra.room.ack_timeout = futureMillis(PUSH_ACK_TIMEOUT_FLOOD);
        } else {
            sendDirect(reply, client->out_path, client->out_path_len);
            uint8_t path_hash_count = client->out_path_len & 63;
            client->extra.room.ack_timeout =
                futureMillis(PUSH_TIMEOUT_BASE + PUSH_ACK_TIMEOUT_FACTOR * (path_hash_count + 1));
        }
        _num_post_pushes++;
    } else {
        client->extra.room.pending_ack = 0;
        LOG_DBG("Unable to push post to client");
    }
}

uint8_t RoomServerMesh::getUnsyncedCount(ClientInfo* client) {
    uint8_t count = 0;
    for (int k = 0; k < MAX_UNSYNCED_POSTS; k++) {
        if (posts[k].post_timestamp > client->extra.room.sync_since &&
            !posts[k].author.matches(client->id)) {
            count++;
        }
    }
    return count;
}

bool RoomServerMesh::processAck(const uint8_t* data) {
    for (int i = 0; i < acl.getNumClients(); i++) {
        ClientInfo* client = acl.getClientByIdx(i);
        if (client->extra.room.pending_ack && memcmp(data, &client->extra.room.pending_ack, 4) == 0) {
            client->extra.room.pending_ack = 0;
            client->extra.room.push_failures = 0;
            client->extra.room.sync_since = client->extra.room.push_post_timestamp;  // advance cursor
            return true;
        }
    }
    return false;
}

void RoomServerMesh::onAckRecv(mesh::Packet* packet, uint32_t ack_crc) {
    if (processAck((uint8_t*)&ack_crc)) {
        packet->markDoNotRetransmit();  // this ACK was for us
    }
}

bool RoomServerMesh::saveFilter(ClientInfo* client) {
    return client->isAdmin();  // only persist admins; guests/read-write re-login
}

static uint8_t max_loop_minimal[]  = { 0, /* 1-byte */  4, /* 2-byte */  2, /* 3-byte */  1 };
static uint8_t max_loop_moderate[] = { 0, /* 1-byte */  2, /* 2-byte */  1, /* 3-byte */  1 };
static uint8_t max_loop_strict[]   = { 0, /* 1-byte */  1, /* 2-byte */  1, /* 3-byte */  1 };

bool RoomServerMesh::isLooped(const mesh::Packet* packet, const uint8_t max_counters[]) {
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

void RoomServerMesh::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size) {
    if (scope.isNull()) {
        sendFlood(pkt, delay_millis, path_hash_size);
    } else {
        uint16_t codes[2];
        codes[0] = scope.calcTransportCode(pkt);
        codes[1] = 0;  // REVISIT: set to 'home' Region, for sender/return region?
        sendFlood(pkt, codes, delay_millis, path_hash_size);
    }
}

void RoomServerMesh::sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size) {
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

bool RoomServerMesh::allowPacketForward(const mesh::Packet* packet) {
    if (_prefs.disable_fwd) return false;
    if (packet->isRouteFlood()) {
        if (packet->getPathHashCount() >= _prefs.flood_max) return false;
        // un-scoped floods can be clamped to a lower hop limit than scoped (transport) floods
        if (packet->getRouteType() == ROUTE_TYPE_FLOOD && packet->getPathHashCount() >= _prefs.flood_max_unscoped) return false;
    }
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

const char* RoomServerMesh::getLogDateTime() {
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

void RoomServerMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
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

void RoomServerMesh::logRx(mesh::Packet* pkt, int len, float score) {
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

void RoomServerMesh::logTx(mesh::Packet* pkt, int len) {
    if (_logging) {
        LOG_INF("TX len=%d type=%d route=%s payload_len=%d",
                len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
                pkt->payload_len);
    }
}

void RoomServerMesh::logTxFail(mesh::Packet* pkt, int len) {
    if (_logging) {
        LOG_WRN("TX FAIL len=%d type=%d route=%s payload_len=%d",
                len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
                pkt->payload_len);
    }
}

uint32_t RoomServerMesh::getRetransmitDelay(const mesh::Packet* packet) {
    return computeAdaptiveFloodDelay(packet);
}

uint32_t RoomServerMesh::getDirectRetransmitDelay(const mesh::Packet* packet) {
    return computeAdaptiveDirectDelay(packet);
}

bool RoomServerMesh::filterRecvFloodPacket(mesh::Packet* pkt) {
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

void RoomServerMesh::onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) {
    if (packet->getPayloadType() != PAYLOAD_TYPE_ANON_REQ) return;

    /* Room login request layout: [timestamp(4)][sync_since(4)][password...].
     * (This differs from the repeater's ANON_REQ, which has no sync_since.) */
    uint32_t sender_timestamp, sender_sync_since;
    memcpy(&sender_timestamp, data, 4);
    memcpy(&sender_sync_since, &data[4], 4);
    data[len] = 0;  // null-terminate the password

    ClientInfo* client = nullptr;
    if (data[8] == 0) {  // blank password -> must already be a known client
        client = acl.getClient(sender.pub_key, PUB_KEY_SIZE);
    }

    if (client == nullptr) {
        /* Constant-time compare against both stored passwords.  Admin grants
         * ADMIN; the guest/room password grants READ_WRITE (so guests may
         * post); allow_read_only downgrades any other login to GUEST.
         *
         * Zero-pad BOTH operands into cleared buffers first: the CLI's
         * StrHelper::strncpy null-terminates but does NOT clear the rest of
         * the 16-byte buffer, so a password set over a longer previous value
         * leaves trailing garbage. Comparing the raw stored buffer full-width
         * against the (zero-padded) received bytes would then fail to match a
         * correct password. Copy only up to the NUL so the compare reflects
         * the actual string while staying constant-time over the full width. */
        uint8_t received[sizeof(_prefs.password)] = {0};
        uint8_t admin_pw[sizeof(_prefs.password)] = {0};
        uint8_t guest_pw[sizeof(_prefs.guest_password)] = {0};
        size_t r_len = strnlen((const char*)&data[8], sizeof(received) - 1);
        memcpy(received, &data[8], r_len);
        memcpy(admin_pw, _prefs.password, strnlen(_prefs.password, sizeof(admin_pw) - 1));
        memcpy(guest_pw, _prefs.guest_password, strnlen(_prefs.guest_password, sizeof(guest_pw) - 1));
        bool admin_match = mesh::Utils::constantTimeEqual(received, admin_pw, sizeof(received));
        bool guest_match = mesh::Utils::constantTimeEqual(received, guest_pw, sizeof(received));

        uint8_t perms;
        if (admin_match) {
            perms = PERM_ACL_ADMIN;
        } else if (guest_match) {
            perms = PERM_ACL_READ_WRITE;
        } else if (_prefs.allow_read_only) {
            perms = PERM_ACL_GUEST;
        } else {
            if (!login_fail_limiter.allow(getRTCClock()->getCurrentTime())) {
                LOG_WRN("Room login rate-limited");
            } else {
                LOG_WRN("Incorrect room password");
            }
            return;
        }

        client = acl.putClient(sender, 0);
        if (sender_timestamp <= client->last_timestamp) {
            LOG_WRN("Possible login replay attack!");
            return;
        }

        LOG_INF("Room login success");
        client->last_timestamp = sender_timestamp;
        client->extra.room.sync_since = sender_sync_since;
        client->extra.room.pending_ack = 0;
        client->extra.room.push_failures = 0;
        client->last_activity = getRTCClock()->getCurrentTime();
        client->permissions &= ~0x03;
        client->permissions |= perms;
        memcpy(client->shared_secret, secret, PUB_KEY_SIZE);

        if (perms != PERM_ACL_GUEST) {
            if (!dirty_contacts_expiry) dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
        }
    }

    if (packet->isRouteFlood()) {
        client->out_path_len = OUT_PATH_UNKNOWN;  // need to rediscover the path
    }

    uint32_t now = getRTCClock()->getCurrentTimeUnique();
    memcpy(reply_data, &now, 4);
    reply_data[4] = RESP_SERVER_LOGIN_OK;
    reply_data[5] = 0;  // legacy: recommended keep-alive interval
    reply_data[6] = (client->isAdmin() ? 1 : (client->permissions == 0 ? 2 : 0));
    reply_data[7] = client->permissions;
    getRNG()->random(&reply_data[8], 4);
    reply_data[12] = FIRMWARE_VER_LEVEL;

    next_push = futureMillis(PUSH_NOTIFY_DELAY_MILLIS);  // let the RESPONSE land before pushing

    if (packet->isRouteFlood()) {
        mesh::Packet* path = createPathReturn(sender, client->shared_secret, packet->path, packet->path_len,
                                              PAYLOAD_TYPE_RESPONSE, reply_data, 13);
        if (path) sendFloodReply(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
    } else {
        mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, client->shared_secret, reply_data, 13);
        if (reply) {
            if (client->out_path_len != OUT_PATH_UNKNOWN) {
                sendDirect(reply, client->out_path, client->out_path_len, SERVER_RESPONSE_DELAY);
            } else {
                sendFloodReply(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
            }
        }
    }
}

int RoomServerMesh::searchPeersByHash(const uint8_t* hash) {
    int n = 0;
    for (int i = 0; i < acl.getNumClients(); i++) {
        if (acl.getClientByIdx(i)->id.isHashMatch(hash)) {
            matching_peer_indexes[n++] = i;
        }
    }
    return n;
}

void RoomServerMesh::getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) {
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

void RoomServerMesh::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp,
                                const uint8_t* app_data, size_t app_data_len) {
    mesh::Mesh::onAdvertRecv(packet, id, timestamp, app_data, app_data_len);

    if (packet->getPathHashCount() == 0 && !isShare(packet)) {
        AdvertDataParser parser(app_data, app_data_len);
        if (parser.isValid() && parser.getType() == ADV_TYPE_REPEATER) {
            putNeighbour(id, timestamp, packet->getSNR());
        }
    }
}

void RoomServerMesh::onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx,
                                  const uint8_t* secret, uint8_t* data, size_t len) {
    int i = matching_peer_indexes[sender_idx];
    if (i < 0 || i >= acl.getNumClients()) {
        LOG_WRN("onPeerDataRecv: invalid peer idx: %d", i);
        return;
    }
    ClientInfo* client = acl.getClientByIdx(i);

    if (type == PAYLOAD_TYPE_TXT_MSG && len > 5) {  // a CLI command or a new post
        uint32_t sender_timestamp;
        memcpy(&sender_timestamp, data, 4);
        uint8_t flags = (data[4] >> 2);

        if (!(flags == TXT_TYPE_PLAIN || flags == TXT_TYPE_CLI_DATA)) {
            LOG_DBG("onPeerDataRecv: unsupported text type: flags=%02x", flags);
        } else if (sender_timestamp >= client->last_timestamp) {
            bool is_retry = (sender_timestamp == client->last_timestamp);
            client->last_timestamp = sender_timestamp;
            client->last_activity = getRTCClock()->getCurrentTime();
            client->extra.room.push_failures = 0;  // peer is alive -> resume pushes

            data[len] = 0;  // null-terminate the text

            /* ACK proves to the sender we received the message. */
            uint32_t ack_hash;
            mesh::Utils::sha256((uint8_t*)&ack_hash, 4, data, 5 + strlen((char*)&data[5]),
                               client->id.pub_key, PUB_KEY_SIZE);

            uint8_t temp[166];
            bool send_ack;
            if (flags == TXT_TYPE_CLI_DATA) {  // admin CLI over the air
                if (client->isAdmin()) {
                    if (is_retry) {
                        temp[5] = 0;
                    } else {
                        handleCommand(sender_timestamp, (char*)&data[5], (char*)&temp[5]);
                        temp[4] = (TXT_TYPE_CLI_DATA << 2);
                    }
                } else {
                    temp[5] = 0;  // non-admin: no CLI reply
                }
                send_ack = false;  // CLI replies are sent as text, not ACKed
            } else {  // TXT_TYPE_PLAIN -> a post
                if ((client->permissions & PERM_ACL_ROLE_MASK) == PERM_ACL_GUEST) {
                    temp[5] = 0;       // read-only guests can't post
                    send_ack = false;
                } else {
                    if (!is_retry) addPost(client, (const char*)&data[5]);
                    temp[5] = 0;       // the ACK is the only reply
                    send_ack = true;
                }
            }

            uint32_t delay_millis;
            if (send_ack) {
                if (client->out_path_len == OUT_PATH_UNKNOWN) {
                    mesh::Packet* ack = createAck(ack_hash);
                    if (ack) sendFloodReply(ack, TXT_ACK_DELAY, packet->getPathHashSize());
                    delay_millis = TXT_ACK_DELAY + CLI_REPLY_DELAY_MILLIS;
                } else {
                    uint32_t d = TXT_ACK_DELAY;
                    if (getExtraAckTransmitCount() > 0) {
                        mesh::Packet* a1 = createMultiAck(ack_hash, 1);
                        if (a1) sendDirect(a1, client->out_path, client->out_path_len, d);
                        d += 300;
                    }
                    mesh::Packet* a2 = createAck(ack_hash);
                    if (a2) sendDirect(a2, client->out_path, client->out_path_len, d);
                    delay_millis = d + CLI_REPLY_DELAY_MILLIS;
                }
            } else {
                delay_millis = 0;
            }

            int text_len = strlen((char*)&temp[5]);
            if (text_len > 0) {  // a CLI reply to send back
                uint32_t now = getRTCClock()->getCurrentTimeUnique();
                if (now == sender_timestamp) now++;
                memcpy(temp, &now, 4);

                mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, secret, temp, 5 + text_len);
                if (reply) {
                    if (client->out_path_len == OUT_PATH_UNKNOWN) {
                        sendFloodReply(reply, delay_millis + SERVER_RESPONSE_DELAY, packet->getPathHashSize());
                    } else {
                        sendDirect(reply, client->out_path, client->out_path_len, delay_millis + SERVER_RESPONSE_DELAY);
                    }
                }
            }
        } else {
            LOG_DBG("onPeerDataRecv: possible replay attack");
        }
    } else if (type == PAYLOAD_TYPE_REQ && len >= 5) {
        uint32_t sender_timestamp;
        memcpy(&sender_timestamp, data, 4);
        if (sender_timestamp < client->last_timestamp) {
            LOG_DBG("onPeerDataRecv: possible replay attack");
        } else {
            client->last_timestamp = sender_timestamp;
            client->last_activity = getRTCClock()->getCurrentTime();
            client->extra.room.push_failures = 0;

            if (data[4] == REQ_TYPE_KEEP_ALIVE && packet->isRouteDirect()) {
                uint32_t forceSince = 0;
                if (len >= 9) {
                    memcpy(&forceSince, &data[5], 4);  // optional: client's last-seen post ts
                } else {
                    memcpy(&data[5], &forceSince, 4);  // zero-fill for the ack hash below
                }
                if (forceSince > 0) {
                    client->extra.room.sync_since = forceSince;
                }
                client->extra.room.pending_ack = 0;

                /* Keep-alive is only answered DIRECT, with the unsynced count
                 * appended to the ACK so the client knows posts are waiting. */
                if (client->out_path_len != OUT_PATH_UNKNOWN) {
                    uint32_t ack_hash;
                    mesh::Utils::sha256((uint8_t*)&ack_hash, 4, data, 9, client->id.pub_key, PUB_KEY_SIZE);
                    mesh::Packet* reply = createAck(ack_hash);
                    if (reply) {
                        reply->payload[reply->payload_len++] = getUnsyncedCount(client);
                        sendDirect(reply, client->out_path, client->out_path_len, SERVER_RESPONSE_DELAY);
                    }
                }
            } else {
                int reply_len = handleRequest(client, sender_timestamp, &data[4], len - 4);
                if (reply_len > 0) {
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
                }
            }
        }
    }
}

bool RoomServerMesh::onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret,
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

void RoomServerMesh::onControlDataRecv(mesh::Packet* packet) {
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

void RoomServerMesh::sendNodeDiscoverReq() {
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

RoomServerMesh::RoomServerMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
                           mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
    : mesh::Mesh(radio, ms, rng, rtc, *new mesh::StaticPoolPacketManager(), tables),
      _board(board),
      _cli(board, rtc, acl, &_prefs, this),
      region_map(key_store), temp_map(key_store),
      discover_limiter(4, 120),
      /* Failed-login rate limit: 4 wrong-password attempts per 180s.  Global
       * rate (not per-sender) — trade-off documented in CRYPTO_AUDIT_INDEX.md
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
    strcpy(_prefs.node_name, "Room");
    _prefs.advert_loc_policy = ADVERT_LOC_PREFS;  // advertise prefs coordinates
    _prefs.loop_detect = LOOP_DETECT_MODERATE;
    _prefs.path_hash_mode = 1;
    _prefs.disable_fwd = 1;  // a room server does not repeat other traffic by default

    /* Room server: circular post buffer + round-robin push state */
    next_post_idx = 0;
    next_client_idx = 0;
    next_push = 0;
    _num_posted = _num_post_pushes = 0;
    for (int i = 0; i < MAX_UNSYNCED_POSTS; i++) {
        posts[i].clear();
    }
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

void RoomServerMesh::begin(RepeaterDataStore* store) {
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

    LOG_INF("RoomServerMesh started: %s (freq=%.2f bw=%.0f sf=%d cr=%d)",
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

double RoomServerMesh::getNodeLat() const {
    struct gps_position pos;
    if (gps_get_last_known_position(&pos)) {
        return pos.latitude_ndeg / 1e9;
    }
    return _prefs.node_lat;
}

double RoomServerMesh::getNodeLon() const {
    struct gps_position pos;
    if (gps_get_last_known_position(&pos)) {
        return pos.longitude_ndeg / 1e9;
    }
    return _prefs.node_lon;
}

bool RoomServerMesh::setGpsEnabled(bool enabled) {
    if (!gps_is_available()) return false;
    gps_enable(enabled);
    return true;
}

bool RoomServerMesh::isGpsEnabled() const {
    return gps_is_enabled();
}

void RoomServerMesh::formatGpsStatsReply(char* reply) {
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

void RoomServerMesh::savePrefs() {
    if (_store) {
        _store->savePrefs(_prefs);
    }
}

void RoomServerMesh::applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) {
    set_radio_at = futureMillis(2000);
    pending_freq = freq;
    pending_bw = bw;
    pending_sf = sf;
    pending_cr = cr;
    revert_radio_at = futureMillis(2000 + timeout_mins * 60 * 1000);
}

void RoomServerMesh::freezeRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) {
    auto& radio = getRadioDriver(_radio);
    if (!radio.hasRadioOverride()) {
        radio.setRadioOverride(freq, bw, sf, cr);
    }
}

bool RoomServerMesh::formatFileSystem() {
    if (_store) {
        return _store->formatFileSystem();
    }
    return false;
}

void RoomServerMesh::sendSelfAdvertisement(int delay_millis, bool flood) {
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

void RoomServerMesh::updateAdvertTimer() {
    if (_prefs.advert_interval > 0) {
        next_local_advert = futureMillis(((uint32_t)_prefs.advert_interval) * 2 * 60 * 1000);
    } else {
        next_local_advert = 0;
    }
}

void RoomServerMesh::updateFloodAdvertTimer() {
    if (_prefs.flood_advert_interval > 0) {
        next_flood_advert = futureMillis(((uint32_t)_prefs.flood_advert_interval) * 60 * 60 * 1000);
    } else {
        next_flood_advert = 0;
    }
}

void RoomServerMesh::eraseLogFile() {
    // Logging to file not implemented in Zephyr version
    LOG_INF("Log erased");
}

void RoomServerMesh::dumpLogFile() {
    // Logging to file not implemented in Zephyr version
    LOG_INF("Log dump not implemented");
}

void RoomServerMesh::setTxPower(int8_t power_dbm) {
    radio_set_tx_power(power_dbm);
}

void RoomServerMesh::formatNeighborsReply(char* reply) {
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

void RoomServerMesh::removeNeighbor(const uint8_t* pubkey, int key_len) {
#if MAX_NEIGHBOURS > 0
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        if (memcmp(neighbours[i].id.pub_key, pubkey, key_len) == 0) {
            neighbours[i].clear();
        }
    }
#endif
}

void RoomServerMesh::formatStatsReply(char* reply) {
    StatsFormatHelper::formatCoreStats(reply, _board, *_ms, _err_flags, _mgr);
}

void RoomServerMesh::formatRadioStatsReply(char* reply) {
    auto& radio_driver = getRadioDriver(_radio);
    StatsFormatHelper::formatRadioStats(reply, _radio, radio_driver, getTotalAirTime(), getReceiveAirTime());
}

void RoomServerMesh::formatPacketStatsReply(char* reply) {
    auto& radio_driver = getRadioDriver(_radio);
    StatsFormatHelper::formatPacketStats(reply, radio_driver, getNumSentFlood(), getNumSentDirect(),
                                         getNumRecvFlood(), getNumRecvDirect());
}

void RoomServerMesh::saveIdentity(const mesh::LocalIdentity& new_id) {
    if (_store) {
        _store->saveIdentity(new_id);
    }
}

void RoomServerMesh::clearStats() {
    auto& radio_driver = getRadioDriver(_radio);
    radio_driver.resetStats();
    radio_driver.resetDutyCycleTimeoutRestarts();
    resetStats();
    ((mesh::SimpleMeshTables *)getTables())->resetStats();
}

uint32_t RoomServerMesh::getDutyCycleTimeoutRestarts() const {
    return getRadioDriver(_radio).getDutyCycleTimeoutRestarts();
}

void RoomServerMesh::resetDutyCycleTimeoutRestarts() {
    getRadioDriver(_radio).resetDutyCycleTimeoutRestarts();
}

/* Region-def CLI (handleRegionLoadLine / handleRegionCommand) and its static
 * parser helpers live in app/RepeaterRegionCLI.cpp. */

void RoomServerMesh::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
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

void RoomServerMesh::loop() {
    mesh::Mesh::loop();

    /* Room server: round-robin push of unsynced posts to logged-in clients. */
    if (millisHasNowPassed(next_push) && acl.getNumClients() > 0) {
        /* Expire any in-flight pushes that never got ACKed. */
        for (int i = 0; i < acl.getNumClients(); i++) {
            ClientInfo* c = acl.getClientByIdx(i);
            if (c->extra.room.pending_ack && millisHasNowPassed(c->extra.room.ack_timeout)) {
                c->extra.room.push_failures++;
                c->extra.room.pending_ack = 0;
            }
        }
        /* Service one client per tick (round robin). */
        ClientInfo* client = acl.getClientByIdx(next_client_idx);
        bool did_push = false;
        if (client->extra.room.pending_ack == 0 && client->last_activity != 0 &&
            client->extra.room.push_failures < 3) {
            uint32_t now = getRTCClock()->getCurrentTime();
            for (int k = 0, idx = next_post_idx; k < MAX_UNSYNCED_POSTS; k++) {
                PostInfo* p = &posts[idx];
                if (p->post_timestamp != 0 &&
                    now >= p->post_timestamp + POST_SYNC_DELAY_SECS &&
                    p->post_timestamp > client->extra.room.sync_since &&
                    !p->author.matches(client->id)) {
                    pushPostToClient(client, *p);
                    did_push = true;
                    break;
                }
                idx = (idx + 1) % MAX_UNSYNCED_POSTS;
            }
        }
        next_client_idx = (next_client_idx + 1) % acl.getNumClients();
        next_push = did_push ? futureMillis(SYNC_PUSH_INTERVAL) : futureMillis(SYNC_PUSH_INTERVAL / 8);
    }

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
        acl.save(_store->getAclPath(), RoomServerMesh::saveFilter);
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

bool RoomServerMesh::hasPendingWork() const {
    return _mgr->getOutboundTotal() > 0;
}
