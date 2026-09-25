#pragma once

//#include <Arduino.h>
#include <mesh/Mesh.h>
// ZEPHCORE: Zephyr platform headers instead of RTClib, target.h and the
// per-platform file systems; storage goes through RepeaterDataStore.
#include <mesh/StaticPoolPacketManager.h>
#include <mesh/SimpleMeshTables.h>
#include <helpers/MeshTimeSync.h>
#include <helpers/NodePrefs.h>
#include "RepeaterDataStore.h"

#include <helpers/AdvertDataHelpers.h>
#include <helpers/ClientACL.h>
#include <helpers/CommonCLI.h>
#include <helpers/IdentityStore.h>
#include <helpers/StatsFormatHelper.h>
#include <helpers/TxtDataHelpers.h>
#include <helpers/RegionMap.h>
#include <helpers/RoutingPolicy.h>
#include <helpers/TransportKeyStore.h>
#include <helpers/RateLimiter.h>

// ZEPHCORE: the version is injected by CMakeLists.txt; these fallbacks only
// apply to builds that bypass it.
#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   __DATE__
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v0.0.0-dev"
#endif

// ZEPHCORE: post buffer size from Kconfig.
#if !defined(MAX_UNSYNCED_POSTS) && defined(CONFIG_ZEPHCORE_MAX_UNSYNCED_POSTS)
  #define MAX_UNSYNCED_POSTS    CONFIG_ZEPHCORE_MAX_UNSYNCED_POSTS
#endif

#ifndef MAX_UNSYNCED_POSTS
  #define MAX_UNSYNCED_POSTS    32
#endif

#ifndef SERVER_RESPONSE_DELAY
  #define SERVER_RESPONSE_DELAY   300
#endif

#ifndef TXT_ACK_DELAY
  #define TXT_ACK_DELAY     200
#endif

#define FIRMWARE_ROLE "room_server"

#define PACKET_LOG_FILE  "/packet_log"

#define MAX_POST_TEXT_LEN    (160-9)

struct PostInfo {
  mesh::Identity author;
  uint32_t post_timestamp;   // by OUR clock
  char text[MAX_POST_TEXT_LEN+1];

  // ZEPHCORE: in place of upstream's memset (Identity has a ctor).
  void clear() {
    author = mesh::Identity();
    post_timestamp = 0;
    memset(text, 0, sizeof(text));
  }
};

class RoomServerMesh : public mesh::Mesh, public CommonCLICallbacks {
  // ZEPHCORE: our board handle and datastore instead of FILESYSTEM* _fs.
  mesh::MainBoard& _board;
  RepeaterDataStore* _store;
  uint32_t last_millis;
  uint64_t uptime_millis;
  unsigned long next_local_advert, next_flood_advert;
  bool _logging;
  // ZEPHCORE: members from here on are in our order, not upstream's (_prefs
  // before key_store, the post buffer after the region maps), so the object
  // layout, and the proof that this file's code is unchanged, stay intact.
  NodePrefs _prefs;
  ClientACL acl;
  CommonCLI _cli;
  uint8_t reply_data[MAX_PACKET_PAYLOAD];
  TransportKeyStore key_store;
  RegionMap region_map, temp_map;
  RegionEntry* load_stack[8];
  RegionEntry* recv_pkt_region;
  // A null recv_pkt_region means either a DIRECT request or an un-scoped flood
  // our wildcard denies, and sendFloodReply() treats them differently, so the
  // route type is recorded (UPSTREAM_TRACKER fad11c90).
  bool recv_pkt_unscoped_flood;
  TransportKey default_scope;
  RateLimiter login_fail_limiter;  // wrong-password attempts (MeshCore#2556)
  bool region_load_active;
  unsigned long dirty_contacts_expiry;
  unsigned long next_push;
  uint16_t _num_posted, _num_post_pushes;
  int next_client_idx;  // for round-robin polling
  int next_post_idx;
  PostInfo posts[MAX_UNSYNCED_POSTS];   // cyclic queue
  CayenneLPP telemetry;
  unsigned long set_radio_at, revert_radio_at;
  float pending_freq;
  float pending_bw;
  uint8_t pending_sf;
  uint8_t pending_cr;
  int  matching_peer_indexes[MAX_CLIENTS];
  // Forward-only mesh time sync; post timestamps order client sync.
  MeshTimeSync _timesync{FIRMWARE_BUILD_EPOCH, true};

  void addPost(ClientInfo* client, const char* postData);
  void storePost(const mesh::Identity& author, const char* postData);
  void pushPostToClient(ClientInfo* client, PostInfo& post);
  uint8_t getUnsyncedCount(ClientInfo* client);
  bool processAck(const uint8_t *data);
  mesh::Packet* createSelfAdvert();
  int handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len);

  void timeSyncTick();  // ZEPHCORE
  uint32_t msUntilNextPush();  // ZEPHCORE
  void handleRegionLoadLine(uint32_t sender_timestamp, char* command, char* reply);  // ZEPHCORE

protected:
  // ZEPHCORE: our Dispatcher asks for a duty-cycle percentage.
  uint8_t getDutyCyclePercent() const override {
    // Arduino formula: duty% = 100 / (af + 1). af=0 -> 100%, af=9 -> 10%.
    return (uint8_t)(100.0f / (_prefs.airtime_factor + 1.0f) + 0.5f);
  }

  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
  void logRx(mesh::Packet* pkt, int len, float score) override;
  void logTx(mesh::Packet* pkt, int len) override;
  void logTxFail(mesh::Packet* pkt, int len) override;

  const char* getLogDateTime() override;
  uint32_t getRetransmitDelay(const mesh::Packet* packet) override;
  uint32_t getDirectRetransmitDelay(const mesh::Packet* packet) override;

  int getInterferenceThreshold() const override {
    return _prefs.interference_threshold;
  }
  uint8_t getExtraAckTransmitCount() const override {
    return _prefs.multi_acks;
  }

  // ZEPHCORE: adaptive CAD.
  int formatFreqErrorStatus(char* buf, int cap) override {
    return _radio->formatFreqErrorStatus(buf, cap);
  }
  int formatCadStatus(char* buf, int cap) override {
    return _radio->formatCadStatus(buf, cap);
  }
  void applyCadPrefs() override {
    _radio->setCadParams(_prefs.cad_auto != 0, _prefs.cad_offset,
                         _prefs.probe_interval, _prefs.cad_busycap,
                         _prefs.cad_base);
    _prefs.cad_offset = _radio->getCadOffset();
    _prefs.cad_base = _radio->cadBasePeak();
  }
  void resetCadStats() override {
    _radio->resetCadStats();
  }
  void onCadOffsetChanged(int8_t offset) override {
    _prefs.cad_offset = offset;
    savePrefs();
  }

  mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt) override;

  bool allowPacketForward(const mesh::Packet* packet) override;
  void onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) override;
  int searchPeersByHash(const uint8_t* hash) override ;
  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
  void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override;
  // ZEPHCORE: feeds the mesh time sync.
  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override;

  void sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size);

public:
  RoomServerMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables);

  void begin(RepeaterDataStore* store);  // ZEPHCORE: was begin(FILESYSTEM* fs)
  void addSystemPost(const char* postData);

  const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
  const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
  const char* getRole() override { return FIRMWARE_ROLE; }
  const char* getNodeName() { return _prefs.node_name; }
  NodePrefs* getNodePrefs() {
    return &_prefs;
  }

  void savePrefs() override;  // ZEPHCORE: through RepeaterDataStore

  void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size);

  // CommonCLICallbacks
  void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) override;
  bool formatFileSystem() override;
  void sendSelfAdvertisement(int delay_millis, bool flood) override;
  void updateAdvertTimer() override;
  void updateFloodAdvertTimer() override;

  void setLoggingOn(bool enable) override { _logging = enable; }

  void eraseLogFile() override;  // ZEPHCORE: no log file

  void dumpLogFile() override;
  void setTxPower(int8_t power_dbm) override;
  bool setRxBoostedGain(bool enable) override;

  void formatNeighborsReply(char *reply) override;  // ZEPHCORE: defined in the .cpp
  void formatStatsReply(char *reply) override;
  void formatRadioStatsReply(char *reply) override;
  void formatPacketStatsReply(char *reply) override;
  void startRegionsLoad() override;
  bool saveRegions() override;
  void onDefaultRegionChanged(const RegionEntry* r) override;

  mesh::LocalIdentity& getSelfId() override { return self_id; }

  static bool saveFilter(ClientInfo* client);

  void saveIdentity(const mesh::LocalIdentity& new_id) override;
  void clearStats() override;
  void handleCommand(uint32_t sender_timestamp, char* command, char* reply);
  void loop();

  // ZEPHCORE: CommonCLICallbacks additions.
  uint32_t getDefaultGpsIntervalSec() const override { return CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC; }
  void freezeRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) override;
  bool setFemRxGain(bool enable) override;
  bool configSideDetectors(const uint8_t* sfs, uint8_t num) override;
  MeshTimeSync* getMeshTimeSync() override { return &_timesync; }
  void noteGPSTimeSync() { _timesync.noteGPSSync((uint32_t)(k_uptime_get() / 1000)); }
  float getContentionEstimate() const override {
    return getContentionTracker().getContentionEstimate();
  }
  float getFloodDelayFactor() const override {
    return getContentionTracker().getFloodDelayFactor();
  }
  void setBackoffMultiplier(float m) override {
    getContentionTracker().setBackoffMultiplier(m);
  }
  // Duty-cycle false-preamble re-arms (SX126x only; others return 0).
  uint32_t getDutyCycleTimeoutRestarts() const override;
  void resetDutyCycleTimeoutRestarts() override;

  // ZEPHCORE: folds this role's own deadlines (adverts, tempradio, ACL flush,
  // time sync, the post push engine) into the base maintenance deadline, so the
  // event loop arms one wake for loop() and maintenanceLoop().
  uint32_t msUntilNextMaintenance() override;
};
