/*
 * SPDX-License-Identifier: MIT
 * RoomServerMesh - LoRa mesh shared-room (BBS) server
 *
 * Extends mesh::Mesh with:
 * - ACL-based client authentication (admin / room / guest)
 * - Region-based flood scoping
 * - Shared-post buffer + round-robin push/sync to logged-in clients
 * - CLI commands (via USB serial)
 *
 * A room server is an endpoint, not a repeater: it does not forward other
 * nodes' traffic and keeps no neighbour / node-discovery state.
 */

#pragma once

#include <mesh/Mesh.h>
#include <mesh/StaticPoolPacketManager.h>
#include <mesh/SimpleMeshTables.h>
#include <helpers/ClientACL.h>
#include <helpers/CommonCLI.h>
#include <helpers/MeshTimeSync.h>
#include <helpers/RegionMap.h>
#include <helpers/TransportKeyStore.h>
#include <helpers/RateLimiter.h>
#include <helpers/StatsFormatHelper.h>
#include <helpers/NodePrefs.h>
#include "RepeaterDataStore.h"

#ifndef FIRMWARE_VERSION
  // Real version injected by CMakeLists.txt (-DFIRMWARE_VERSION); this fallback
  // only applies to builds that bypass that injection and should never surface.
  #define FIRMWARE_VERSION   "v0.0.0-dev"
#endif

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   __DATE__
#endif

#define FIRMWARE_ROLE "room_server"

#ifndef MAX_UNSYNCED_POSTS
  #ifdef CONFIG_ZEPHCORE_MAX_UNSYNCED_POSTS
    #define MAX_UNSYNCED_POSTS  CONFIG_ZEPHCORE_MAX_UNSYNCED_POSTS
  #else
    #define MAX_UNSYNCED_POSTS  32
  #endif
#endif

/* Post text budget: matches upstream MeshCore (160-byte payload minus a
 * 9-byte header = 4-byte timestamp + 1-byte type + 4-byte author prefix). */
#define MAX_POST_TEXT_LEN  (160 - 9)

/* A single shared-room post held in the server's circular buffer. */
struct PostInfo {
    mesh::Identity author;
    uint32_t post_timestamp;   // by OUR clock
    char text[MAX_POST_TEXT_LEN + 1];

    void clear() {
        author = mesh::Identity();
        post_timestamp = 0;
        memset(text, 0, sizeof(text));
    }
};

class RoomServerMesh : public mesh::Mesh, public CommonCLICallbacks {
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
    TransportKeyStore key_store;
    RegionMap region_map, temp_map;
    RegionEntry* load_stack[8];
    RegionEntry* recv_pkt_region;
    TransportKey default_scope;
    RateLimiter login_fail_limiter;
    bool region_load_active;
    unsigned long dirty_contacts_expiry;
    /* Room server: circular post buffer + round-robin push state */
    unsigned long next_push;
    uint16_t _num_posted, _num_post_pushes;
    int next_client_idx;
    int next_post_idx;
    PostInfo posts[MAX_UNSYNCED_POSTS];
    unsigned long set_radio_at, revert_radio_at;
    float pending_freq;
    float pending_bw;
    uint8_t pending_sf;
    uint8_t pending_cr;
    int matching_peer_indexes[MAX_CLIENTS];
    /* Forward-only: post timestamps feed client sync_since ordering, so a
     * backward step would corrupt message sync. */
    MeshTimeSync _timesync{FIRMWARE_BUILD_EPOCH, true};

    int handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len);
    mesh::Packet* createSelfAdvert();
    void timeSyncTick();

    /* Room server: shared-post buffer + push-to-client sync */
    void addPost(ClientInfo* client, const char* postData);
    void storePost(const mesh::Identity& author, const char* postData);
    void pushPostToClient(ClientInfo* client, PostInfo& post);
    uint8_t getUnsyncedCount(ClientInfo* client);
    bool processAck(const uint8_t* data);
    static bool saveFilter(ClientInfo* client);

    void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size);
    void sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size);

    /* Region-definition CLI (defined in app/RepeaterRegionCLI.cpp).
     * handleRegionLoadLine: a continuation line during `region load`.
     * handleRegionCommand:  a `region ...` command. */
    void handleRegionLoadLine(char* command, char* reply);
    void handleRegionCommand(char* command, char* reply);

protected:
    uint8_t getDutyCyclePercent() const override {
        /* Arduino formula: duty% = 100 / (af + 1). af=0 → 100%, af=9 → 10%. */
        return (uint8_t)(100.0f / (_prefs.airtime_factor + 1.0f) + 0.5f);
    }

    bool allowPacketForward(const mesh::Packet* packet) override;
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
    uint8_t getExtraAckTransmitCount() const override {
        return _prefs.multi_acks;
    }

    /* Adaptive CAD */
    int formatCadStatus(char* buf, int cap) override {
        return _radio->formatCadStatus(buf, cap);
    }
    void applyCadPrefs() override {
        _radio->setCadParams(_prefs.cad_auto != 0, _prefs.cad_offset,
                             _prefs.probe_interval, _prefs.cad_busycap);
    }
    void resetCadStats() override {
        _radio->resetCadStats();
    }
    void onCadOffsetChanged(int8_t offset) override {
        _prefs.cad_offset = offset;
        savePrefs();
    }

    mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt) override;

    void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override;
    void onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) override;
    int searchPeersByHash(const uint8_t* hash) override;
    void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
    void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
    bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
    void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override;

public:
    RoomServerMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
                 mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables);

    void begin(RepeaterDataStore* store);

    /* Post authored by the server itself (admin "room.post <msg>" command). */
    void addSystemPost(const char* postData);

    /* CommonCLICallbacks */
    const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
    const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
    const char* getRole() override { return FIRMWARE_ROLE; }
    double getNodeLat() const override;
    double getNodeLon() const override;
    bool setGpsEnabled(bool enabled) override;
    bool isGpsEnabled() const override;
    void formatGpsStatsReply(char* reply) override;
    uint32_t getDefaultGpsIntervalSec() const override { return CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC; }
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
    bool setRxBoostedGain(bool enable) override;
    void formatNeighborsReply(char* reply) override;
    void formatStatsReply(char* reply) override;
    void formatRadioStatsReply(char* reply) override;
    void formatPacketStatsReply(char* reply) override;

    mesh::LocalIdentity& getSelfId() override { return self_id; }
    void saveIdentity(const mesh::LocalIdentity& new_id) override;
    void clearStats() override;

    /* Mesh time sync */
    MeshTimeSync* getMeshTimeSync() override { return &_timesync; }
    void noteGPSTimeSync() { _timesync.noteGPSSync((uint32_t)(k_uptime_get() / 1000)); }

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

    void handleCommand(uint32_t sender_timestamp, char* command, char* reply);
    void loop();

    bool hasPendingWork() const;
};
