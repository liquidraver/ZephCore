/*
 * SPDX-License-Identifier: Apache-2.0
 * RepeaterMesh - LoRa mesh repeater implementation
 *
 * Extends mesh::Mesh with:
 * - ACL-based client authentication
 * - Region-based flood filtering
 * - Neighbor tracking
 * - CLI commands (via USB serial)
 * - Protocol handlers (login, status, telemetry, etc.)
 */

#pragma once

#include <mesh/Mesh.h>
#include <mesh/StaticPoolPacketManager.h>
#include <mesh/SimpleMeshTables.h>
#include <helpers/ClientACL.h>
#include <helpers/CommonCLI.h>
#include <helpers/RegionMap.h>
#include <helpers/TransportKeyStore.h>
#include <helpers/RateLimiter.h>
#include <helpers/StatsFormatHelper.h>
#include <helpers/NodePrefs.h>
#include "RepeaterDataStore.h"
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK)
#include "observer_creds.h"
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.15.1-zephyr"
#endif

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   __DATE__
#endif

#define FIRMWARE_ROLE "repeater"

#ifndef MAX_NEIGHBOURS
  #ifdef CONFIG_ZEPHCORE_MAX_NEIGHBOURS
    #define MAX_NEIGHBOURS  CONFIG_ZEPHCORE_MAX_NEIGHBOURS
  #else
    #define MAX_NEIGHBOURS  16
  #endif
#endif

struct NeighbourInfo {
    mesh::Identity id;
    uint32_t advert_timestamp;
    uint32_t heard_timestamp;
    int8_t snr;  // multiplied by 4

    void clear() {
        id = mesh::Identity();
        advert_timestamp = 0;
        heard_timestamp = 0;
        snr = 0;
    }
};

struct RepeaterStats {
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
    int16_t last_snr;  // x 4
    uint16_t n_direct_dups, n_flood_dups;
    uint32_t total_rx_air_time_secs;
    uint32_t n_recv_errors;
};

class RepeaterMesh : public mesh::Mesh, public CommonCLICallbacks {
    mesh::MainBoard& _board;
    RepeaterDataStore* _store;
    uint32_t last_millis;
    uint64_t uptime_millis;
    unsigned long next_local_advert, next_flood_advert;
    bool _logging;
    NodePrefs _prefs;
    ClientACL acl;
    CommonCLI _cli;
    uint8_t reply_data[MAX_PACKET_PAYLOAD];
    uint8_t reply_path[MAX_PATH_SIZE];
    uint8_t reply_path_len;
    TransportKeyStore key_store;
    RegionMap region_map, temp_map;
    RegionEntry* load_stack[8];
    RegionEntry* recv_pkt_region;
    TransportKey default_scope;
    RateLimiter discover_limiter, anon_limiter;
    uint32_t pending_discover_tag;
    unsigned long pending_discover_until;
    bool region_load_active;
    unsigned long dirty_contacts_expiry;
#if MAX_NEIGHBOURS > 0
    NeighbourInfo neighbours[MAX_NEIGHBOURS];
#endif
    unsigned long set_radio_at, revert_radio_at;
    float pending_freq;
    float pending_bw;
    uint8_t pending_sf;
    uint8_t pending_cr;
    int matching_peer_indexes[MAX_CLIENTS];
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK)
    ObserverCreds _uplink_creds;
    bool _uplink_reboot_required;
    char _uplink_pubkey_hex[PUB_KEY_SIZE * 2 + 1];
    char _uplink_packets_topic[160];
    char _uplink_status_topic[160];
    float _uplink_last_score;
    float _uplink_last_rssi;
    uint8_t _uplink_last_raw[MAX_TRANS_UNIT];
    int _uplink_last_raw_len;
    unsigned long _uplink_next_status_at;
#endif

    void putNeighbour(const mesh::Identity& id, uint32_t timestamp, float snr);
    uint8_t handleLoginReq(const mesh::Identity& sender, const uint8_t* secret, uint32_t sender_timestamp, const uint8_t* data, bool is_flood);
    uint8_t handleAnonRegionsReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data);
    uint8_t handleAnonOwnerReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data);
    uint8_t handleAnonClockReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data);
    int handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len);
    mesh::Packet* createSelfAdvert();
    void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size);
    void sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size);

protected:
    uint8_t getDutyCyclePercent() const override {
        return (uint8_t)_prefs.airtime_factor;
    }

    bool allowPacketForward(const mesh::Packet* packet) override;
    bool isLooped(const mesh::Packet* packet, const uint8_t max_counters[]);
    const char* getLogDateTime() override;

    void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
    void logRx(mesh::Packet* pkt, int len, float score) override;
    void logTx(mesh::Packet* pkt, int len) override;
    void logTxFail(mesh::Packet* pkt, int len) override;
    uint32_t getRetransmitDelay(const mesh::Packet* packet) override;
    uint32_t getDirectRetransmitDelay(const mesh::Packet* packet) override;

    int getInterferenceThreshold() const override {
        return _prefs.interference_threshold;
    }
    int getAGCResetInterval() const override {
        if (_prefs.rx_duty_cycle) {
            return 0;
        }
        return ((int)_prefs.agc_reset_interval) * 4000;
    }
    uint8_t getExtraAckTransmitCount() const override {
        return _prefs.multi_acks;
    }

    bool filterRecvFloodPacket(mesh::Packet* pkt) override;

    void onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) override;
    int searchPeersByHash(const uint8_t* hash) override;
    void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
    void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len);
    void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
    bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
    void onControlDataRecv(mesh::Packet* packet) override;
#if IS_ENABLED(CONFIG_ZEPHCORE_REPEATER_UPLINK)
    bool handleUplinkCommand(const char *command, char *reply);
    void markUplinkRebootRequired() { _uplink_reboot_required = true; }
    bool isUplinkEnabled() const { return (_uplink_creds._reserved[0] & 0x01) != 0; }
    void setUplinkEnabled(bool en) {
        if (en) {
            _uplink_creds._reserved[0] |= 0x01;
        } else {
            _uplink_creds._reserved[0] &= (uint8_t)~0x01;
        }
    }
    bool saveUplinkCreds();
    void publishUplinkPacket(mesh::Packet *pkt);
    void publishUplinkStatus(const char *status);
#endif

public:
    RepeaterMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
                 mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables);

    void begin(RepeaterDataStore* store);

    void sendNodeDiscoverReq();

    /* CommonCLICallbacks */
    const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
    const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
    const char* getRole() override { return FIRMWARE_ROLE; }
    double getNodeLat() const override;
    double getNodeLon() const override;
    bool setGpsEnabled(bool enabled) override;
    bool isGpsEnabled() const override;
    void formatGpsStatsReply(char* reply) override;
    const char* getNodeName() { return _prefs.node_name; }
    NodePrefs* getNodePrefs() { return &_prefs; }

    void savePrefs() override;
    void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) override;
    void freezeRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) override;
    bool formatFileSystem() override;
    void sendSelfAdvertisement(int delay_millis, bool flood) override;
    void updateAdvertTimer() override;
    void updateFloodAdvertTimer() override;
    void setLoggingOn(bool enable) override { _logging = enable; }
    void eraseLogFile() override;
    void dumpLogFile() override;
    void setTxPower(int8_t power_dbm) override;
    void formatNeighborsReply(char* reply) override;
    void removeNeighbor(const uint8_t* pubkey, int key_len) override;
    void formatStatsReply(char* reply) override;
    void formatRadioStatsReply(char* reply) override;
    void formatPacketStatsReply(char* reply) override;

    mesh::LocalIdentity& getSelfId() override { return self_id; }
    void saveIdentity(const mesh::LocalIdentity& new_id) override;
    void clearStats() override;

    /* Adaptive contention window callbacks */
    float getContentionEstimate() const override {
        return getContentionTracker().getContentionEstimate();
    }
    float getFloodDelayFactor() const override {
        return getContentionTracker().getFloodDelayFactor();
    }
    void setBackoffMultiplier(float m) override {
        getContentionTracker().setBackoffMultiplier(m);
    }

    /* Duty-cycle preamble false-positive stats (SX126x only;
     * other radios return 0 from the base class). */
    uint32_t getDutyCycleTimeoutRestarts() const override;
    void resetDutyCycleTimeoutRestarts() override;

#ifdef CONFIG_ZEPHCORE_APC
    /* Adaptive Power Control callbacks */
    int8_t getAPCReduction() const override {
        return getPowerController().getPowerReduction();
    }
    float getAPCMargin() const override {
        return getPowerController().getMarginEstimate();
    }
    bool isAPCEnabled() const override {
        return getPowerController().isEnabled();
    }
    void setAPCEnabled(bool en) override {
        getPowerController().setEnabled(en);
        if (!en) {
            _radio->setTxPowerReduction(0);
        }
    }
    uint8_t getAPCTargetMargin() const override {
        return getPowerController().getTargetMargin();
    }
    void setAPCTargetMargin(uint8_t margin_db) override {
        getPowerController().setTargetMargin(margin_db);
    }
#endif

    void handleCommand(uint32_t sender_timestamp, char* command, char* reply);
    void loop();

    bool hasPendingWork() const;
};
