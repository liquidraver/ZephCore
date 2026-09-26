#include "RoomServerMesh.h"

// ZEPHCORE: platform includes and helpers.
#include <mesh/Utils.h>
#include <Serial.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
#include <helpers/MeshcoreJson.h>
#include <adapters/radio/LoRaRadio.h>
#include <adapters/sensors/ZephyrSensorManager.h>
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
#include <helpers/PacketLog.h>

/* Helper to get radio driver for stats — uses LoRaRadio (works for SX126x and LR1110) */
static inline mesh::LoRaRadio& getRadioDriver(mesh::Radio* radio) {
  return *static_cast<mesh::LoRaRadio*>(radio);
}

LOG_MODULE_REGISTER(zephcore_room, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

// ZEPHCORE: our constants (protocol level 2 for the owner-info request;
// faster post push and sync than upstream's 2000 ms / 6 s).
/* Protocol constants */
#define FIRMWARE_VER_LEVEL       2

#define REQ_TYPE_GET_STATUS         0x01
#define REQ_TYPE_KEEP_ALIVE         0x02
#define REQ_TYPE_GET_TELEMETRY_DATA 0x03
#define REQ_TYPE_GET_ACCESS_LIST    0x05
#define REQ_TYPE_GET_OWNER_INFO     0x07

#define RESP_SERVER_LOGIN_OK        0

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

// ZEPHCORE: upstream's variant-global rtc_clock is the mesh's own clock here.
#define rtc_clock (*getRTCClock())

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
  uint16_t err_events; // was 'n_full_events'
  int16_t last_snr;    // x 4
  uint16_t n_direct_dups, n_flood_dups;
  uint16_t n_posted, n_post_push;
};

void RoomServerMesh::addPost(ClientInfo *client, const char *postData) {
  storePost(client->id, postData);
}

void RoomServerMesh::addSystemPost(const char *postData) {
  if (!postData || postData[0] == 0) return;

  MESH_DEBUG_PRINTLN("room.post: addSystemPost: %s", postData);

  storePost(self_id, postData);
}

// ZEPHCORE: ours. Keeps the full MAX_POST_TEXT_LEN characters (upstream's StrHelper copy keeps one fewer).
void RoomServerMesh::storePost(const mesh::Identity& author, const char* postData) {
  posts[next_post_idx].author = author;
  strncpy(posts[next_post_idx].text, postData, MAX_POST_TEXT_LEN);
  posts[next_post_idx].text[MAX_POST_TEXT_LEN] = '\0';
  posts[next_post_idx].post_timestamp = getRTCClock()->getCurrentTimeUnique();
  next_post_idx = (next_post_idx + 1) % MAX_UNSYNCED_POSTS;

  next_push = futureMillis(PUSH_NOTIFY_DELAY_MILLIS);
  _num_posted++;
}

void RoomServerMesh::pushPostToClient(ClientInfo *client, PostInfo &post) {
  MESH_DEBUG_PRINTLN("room.post: pushPostToClient text=%s", post.text);
  int len = 0;
  memcpy(&reply_data[len], &post.post_timestamp, 4);
  len += 4; // this is a PAST timestamp... but should be accepted by client

  uint8_t attempt;
  getRNG()->random(&attempt, 1); // need this for re-tries, so packet hash (and ACK) will be different
  reply_data[len++] = (TXT_TYPE_SIGNED_PLAIN << 2) | (attempt & 3); // 'signed' plain text

  // encode prefix of post.author.pub_key
  memcpy(&reply_data[len], post.author.pub_key, 4);
  len += 4; // just first 4 bytes

  int text_len = strlen(post.text);
  memcpy(&reply_data[len], post.text, text_len);
  len += text_len;

  // calc expected ACK reply
  mesh::Utils::sha256((uint8_t *)&client->extra.room.pending_ack, 4, reply_data, len, client->id.pub_key, PUB_KEY_SIZE);
  client->extra.room.push_post_timestamp = post.post_timestamp;

  auto reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, client->shared_secret, reply_data, len);
  if (reply) {
    if (client->out_path_len == OUT_PATH_UNKNOWN) {
      unsigned long delay_millis = 0;
      sendFloodScoped(default_scope, reply, delay_millis, _prefs.path_hash_mode + 1); // REVISIT
      client->extra.room.ack_timeout = futureMillis(PUSH_ACK_TIMEOUT_FLOOD);
    } else {
      sendDirect(reply, client->out_path, client->out_path_len);

      uint8_t path_hash_count = client->out_path_len & 63;
      client->extra.room.ack_timeout = futureMillis(PUSH_TIMEOUT_BASE + PUSH_ACK_TIMEOUT_FACTOR * (path_hash_count + 1));
    }
    _num_post_pushes++; // stats
  } else {
    client->extra.room.pending_ack = 0;
    MESH_DEBUG_PRINTLN("Unable to push post to client");
  }
}

uint8_t RoomServerMesh::getUnsyncedCount(ClientInfo *client) {
  uint8_t count = 0;
  for (int k = 0; k < MAX_UNSYNCED_POSTS; k++) {
    if (posts[k].post_timestamp > client->extra.room.sync_since // is new post for this Client?
        && !posts[k].author.matches(client->id)) {   // don't push posts to the author
      count++;
    }
  }
  return count;
}

bool RoomServerMesh::processAck(const uint8_t *data) {
  for (int i = 0; i < acl.getNumClients(); i++) {
    auto client = acl.getClientByIdx(i);
    if (client->extra.room.pending_ack && memcmp(data, &client->extra.room.pending_ack, 4) == 0) { // got an ACK from Client!
      client->extra.room.pending_ack = 0; // clear this, so next push can happen
      client->extra.room.push_failures = 0;
      client->extra.room.sync_since = client->extra.room.push_post_timestamp; // advance Client's SINCE timestamp, to sync next post
      return true;
    }
  }
  return false;
}

mesh::Packet *RoomServerMesh::createSelfAdvert() {
  uint8_t app_data[MAX_ADVERT_DATA_SIZE];
  uint8_t app_data_len = _cli.buildAdvertData(ADV_TYPE_ROOM, app_data);

  return createAdvert(self_id, app_data, app_data_len);
}

// ZEPHCORE: no openAppend() here: no log file on flash

// ZEPHCORE: ours. ZephCore telemetry, stats and ACL paging.
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
    stats.n_direct_dups = ((SimpleMeshTables *)getTables())->getNumDirectDups();
    stats.n_flood_dups = ((SimpleMeshTables *)getTables())->getNumFloodDups();
    stats.n_posted = _num_posted;
    stats.n_post_push = _num_post_pushes;
    memcpy(&reply_data[4], &stats, sizeof(stats));
    return 4 + sizeof(stats);
  }

  if (payload[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
    uint8_t perm_mask = ~(payload[1]); // NEW: first reserved byte (of 4), is now inverse mask to apply to permissions

    telemetry.reset();
    telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)_board.getBattMilliVolts() / 1000.0f);

    // query other sensors -- target specific
    if ((sender->permissions & PERM_ACL_ROLE_MASK) == PERM_ACL_GUEST) {
      perm_mask = 0x00;  // just base telemetry allowed
    }
    sensors.querySensors(perm_mask, telemetry);

    // This default temperature will be overridden by external sensors (if any)
    float temperature = _board.getMCUTemperature();
    if(!isnan(temperature)) { // Supported boards with built-in temperature sensor. ESP32-C3 may return NAN
      telemetry.addTemperature(TELEM_CHANNEL_SELF, temperature); // Built-in MCU Temperature
    }

    // ZEPHCORE: wake the GPS (or extend its acquire window) so the next poll
    // has a fresher fix. A server's GPS is normally asleep between its 48 h
    // time syncs. Gated like the position, so a guest cannot keep it awake.
    if ((perm_mask & TELEM_PERM_LOCATION) && gps_is_enabled()) {
      gps_request_fresh_fix();
    }

    uint8_t tlen = telemetry.getSize();
    memcpy(&reply_data[4], telemetry.getBuffer(), tlen);
    return 4 + tlen; // reply_len
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

  if (payload[0] == REQ_TYPE_GET_OWNER_INFO) {
    sprintf((char*)&reply_data[4], "%s\n%s\n%s", FIRMWARE_VERSION, _prefs.node_name, _prefs.owner_info);
    return 4 + strlen((char*)&reply_data[4]);
  }

  return 0;
}

// ZEPHCORE: ours. Packet logging through helpers/PacketLog.h.
void RoomServerMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
  /* Arduino-compatible RAW packet hex dump */
  static char hex_buf[MAX_TRANS_UNIT * 2 + 1];
  mesh::Utils::toHex(hex_buf, raw, len);
  printk("%s RAW: %s\n", getLogDateTime(), hex_buf);
#endif
  (void)snr;
  (void)rssi;
  (void)raw;
  (void)len;
}

// ZEPHCORE: ours. Packet logging through helpers/PacketLog.h.
void RoomServerMesh::logRx(mesh::Packet* pkt, int len, float score) {
  packet_log_rx(getLogDateTime(), pkt, _radio->getLastRSSI(), score, _radio->getEstAirtimeFor(len));
  if (_logging) {
    LOG_INF("RX len=%d type=%d route=%s payload_len=%d SNR=%d RSSI=%d",
      len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
      pkt->payload_len, (int)_radio->getLastSNR(), (int)_radio->getLastRSSI());
  }
  (void)score;
}
// ZEPHCORE: ours. Packet logging through helpers/PacketLog.h.
void RoomServerMesh::logTx(mesh::Packet* pkt, int len) {
  packet_log_tx(getLogDateTime(), pkt);
  if (_logging) {
    LOG_INF("TX len=%d type=%d route=%s payload_len=%d",
      len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
      pkt->payload_len);
  }
}
// ZEPHCORE: ours. Packet logging through helpers/PacketLog.h.
void RoomServerMesh::logTxFail(mesh::Packet* pkt, int len) {
  if (_logging) {
    LOG_WRN("TX FAIL len=%d type=%d route=%s payload_len=%d",
      len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F",
      pkt->payload_len);
  }
}

// ZEPHCORE: no calcRxDelay() here: rx delay is replaced by the adaptive contention window

// ZEPHCORE: ours. Zephyr clock formatting.
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

// ZEPHCORE: ours. Adaptive contention window.
uint32_t RoomServerMesh::getRetransmitDelay(const mesh::Packet* packet) {
  return computeAdaptiveFloodDelay(packet);
}
// ZEPHCORE: ours. Adaptive contention window.
uint32_t RoomServerMesh::getDirectRetransmitDelay(const mesh::Packet* packet) {
  return computeAdaptiveDirectDelay(packet);
}

// ZEPHCORE: ours. The room server never forwards (UPSTREAM_TRACKER fad11c90).
/* A room server is an endpoint, not a repeater: it never forwards other nodes'
 * transit traffic (no flood relaying, no neighbour/loop-detect machinery).  Its
 * own replies still go out via sendDirect()/sendFloodReply(); this only governs
 * relaying of pass-through packets. */
bool RoomServerMesh::allowPacketForward(const mesh::Packet* /*packet*/) {
  return false;
}

// ZEPHCORE: ours. Also records whether the packet was an unscoped flood.
mesh::DispatcherAction RoomServerMesh::onRecvPacket(mesh::Packet* pkt) {
  // Determine the request packet's region so sendFloodReply() can echo the same
  // scope. Runs for every packet (not just floods) so recv_pkt_region is cleared
  // for direct packets instead of inheriting the last flood's region.
  recv_pkt_unscoped_flood = (pkt->getRouteType() == ROUTE_TYPE_FLOOD);
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
  return Mesh::onRecvPacket(pkt);
}

// ZEPHCORE: ours. Constant-time password compare and a failed-login rate limit.
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

    /* An empty stored guest password disables guest access (as
     * CONFIG_ZEPHCORE_GUEST_PASSWORD documents) rather than matching an
     * empty submitted password and granting read+write to anyone. Both
     * compares above still run unconditionally, so timing is unchanged.
     * allow_read_only below remains the intended way to run an open room. */
    if (_prefs.guest_password[0] == 0) guest_match = false;

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
    /* What save() stores that a login can change: a new entry (permissions
     * 0 until set below), the role, sync_since, the secret. */
    uint8_t prev_perms = client->permissions;
    uint32_t prev_sync_since = client->extra.room.sync_since;
    bool secret_changed = memcmp(client->shared_secret, secret, PUB_KEY_SIZE) != 0;

    client->last_timestamp = sender_timestamp;
    client->extra.room.sync_since = sender_sync_since;
    client->extra.room.pending_ack = 0;
    client->extra.room.push_failures = 0;
    client->last_activity = getRTCClock()->getCurrentTime();
    client->permissions &= ~0x03;
    client->permissions |= perms;
    memcpy(client->shared_secret, secret, PUB_KEY_SIZE);

    /* Upstream schedules the write on every non-guest login; a member logging
     * in again with nothing new (same secret, role and sync point) would
     * rewrite an identical ACL. */
    if (perms != PERM_ACL_GUEST &&
        (client->permissions != prev_perms || client->extra.room.sync_since != prev_sync_since ||
         secret_changed)) {
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

int RoomServerMesh::searchPeersByHash(const uint8_t *hash) {
  int n = 0;
  for (int i = 0; i < acl.getNumClients(); i++) {
    if (acl.getClientByIdx(i)->id.isHashMatch(hash)) {
      matching_peer_indexes[n++] = i; // store the INDEXES of matching contacts (for subsequent 'peer' methods)
    }
  }
  return n;
}

void RoomServerMesh::getPeerSharedSecret(uint8_t *dest_secret, int peer_idx) {
  int i = matching_peer_indexes[peer_idx];
  if (i >= 0 && i < acl.getNumClients()) {
    // lookup pre-calculated shared_secret
    memcpy(dest_secret, acl.getClientByIdx(i)->shared_secret, PUB_KEY_SIZE);
  } else {
    MESH_DEBUG_PRINTLN("getPeerSharedSecret: Invalid peer idx: %d", i);
  }
}

// ZEPHCORE: ours. Activity clock and CLI reply delay.
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

    /* TXT_TYPE_CLI_COMMAND (v1.18+) is handled exactly like TXT_TYPE_CLI_DATA
     * here: both stay behind client->isAdmin() below, and both are covered
     * by the monotonic sender_timestamp / is_retry gates. */
    if (!(flags == TXT_TYPE_PLAIN || flags == TXT_TYPE_CLI_DATA ||
          flags == TXT_TYPE_CLI_COMMAND)) {
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

      uint8_t temp[5 + CLI_REMOTE_REPLY_SIZE];
      bool send_ack;
      if (flags == TXT_TYPE_CLI_DATA || flags == TXT_TYPE_CLI_COMMAND) {  // admin CLI over the air
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

bool RoomServerMesh::onPeerPathRecv(mesh::Packet *packet, int sender_idx, const uint8_t *secret, uint8_t *path,
                            uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len) {
  // TODO: prevent replay attacks
  int i = matching_peer_indexes[sender_idx];

  if (i >= 0 && i < acl.getNumClients()) { // get from our known_clients table (sender SHOULD already be known in this context)
    MESH_DEBUG_PRINTLN("PATH to client, path_len=%d", (uint32_t)path_len);
    auto client = acl.getClientByIdx(i);
    client->out_path_len = mesh::Packet::copyPath(client->out_path, path, path_len); // store a copy of path, for sendDirect()
    client->last_activity = getRTCClock()->getCurrentTime();
  } else {
    MESH_DEBUG_PRINTLN("onPeerPathRecv: invalid peer idx: %d", i);
  }

  if (extra_type == PAYLOAD_TYPE_ACK && extra_len >= 4) {
    // also got an encoded ACK!
    processAck(extra);
  }

  // NOTE: no reciprocal path send!!
  return false;
}

void RoomServerMesh::onAckRecv(mesh::Packet *packet, uint32_t ack_crc) {
  if (processAck((uint8_t *)&ack_crc)) {
    packet->markDoNotRetransmit(); // ACK was for this node, so don't retransmit
  }
}

// ZEPHCORE: ours. Zephyr datastore and ZephCore members.
RoomServerMesh::RoomServerMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
         mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
  : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(), tables),
    _board(board),
    _cli(board, rtc, sensors, &region_map, &acl, &_prefs, this),
    region_map(key_store), temp_map(key_store),
    /* Failed-login rate limit: 4 wrong-password attempts per 180s.  Global
     * rate (not per-sender) — trade-off documented in CRYPTO_AUDIT_INDEX.md
     * Phase 4 (mitigation for upstream MeshCore#2556). */
    login_fail_limiter(4, 180),
    telemetry(MAX_PACKET_PAYLOAD - 4) {

  _store = nullptr;
  last_millis = 0;
  uptime_millis = 0;
  next_local_advert = next_flood_advert = 0;
  dirty_contacts_expiry = 0;
  set_radio_at = revert_radio_at = 0;
  _logging = false;
  region_load_active = false;
  recv_pkt_region = nullptr;
  recv_pkt_unscoped_flood = false;
  memset(default_scope.key, 0, sizeof(default_scope.key));

  initNodePrefs(&_prefs);
  strcpy(_prefs.node_name, "Room");
  _prefs.advert_loc_policy = ADVERT_LOC_PREFS;  // advertise prefs coordinates
  /* path_hash_mode = 1 moved into initNodePrefs() -- see RepeaterMesh. */
  _prefs.disable_fwd = 1;  // a room server is an endpoint, never repeats
  _prefs.gps_interval = CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC;  // ZEPHCORE: as the repeater
  _prefs.gps_enabled = 1;      // ZEPHCORE: GPS on for time sync (upstream: 0)
  _prefs.gps_enabled_set = 1;

  /* Room server: circular post buffer + round-robin push state */
  next_post_idx = 0;
  next_client_idx = 0;
  next_push = 0;
  _num_posted = _num_post_pushes = 0;
  for (int i = 0; i < MAX_UNSYNCED_POSTS; i++) {
    posts[i].clear();
  }
}

// ZEPHCORE: ours. Zephyr storage and radio setup.
void RoomServerMesh::begin(RepeaterDataStore* store) {
  _store = store;

  /* Prefs and identity are loaded by the caller (main_room_server.cpp) before
   * begin() — the radio reads freq/bw/sf/cr through _prefs during
   * Mesh::begin() → Dispatcher::begin() → Radio::begin(). */
  mesh::Mesh::begin();
  _contention.setBackoffMultiplier(_prefs.backoff_multiplier);
  acl.load(_store->getFS(), self_id);
  region_map.load(_store->getFS());

  // establish default-scope from persisted default region (if any)
  {
    RegionEntry* r = region_map.getDefaultRegion();
    if (r) {
      region_map.getTransportKeysFor(*r, &default_scope, 1);
    }
  }

  updateAdvertTimer();
  updateFloodAdvertTimer();

  _board.setAdcMultiplier(_prefs.adc_multiplier);

  LOG_INF("RoomServerMesh started: %s (freq=%.2f bw=%.0f sf=%d cr=%d)",
    _prefs.node_name, (double)_prefs.freq, (double)_prefs.bw, _prefs.sf, _prefs.cr);
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

// ZEPHCORE: ours. An explicit unscoped-flood flag instead of isWildcard() (UPSTREAM_TRACKER fad11c90).
void RoomServerMesh::sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size) {
  TransportKey req_scope;
  bool req_scope_known = recv_pkt_region != nullptr && !recv_pkt_region->isWildcard()
            && region_map.getTransportKeysFor(*recv_pkt_region, &req_scope, 1) > 0;

  switch (mesh::chooseReplyScope(req_scope_known, recv_pkt_unscoped_flood, !default_scope.isNull())) {
  case mesh::REPLY_SCOPE_REQUEST:
    sendFloodScoped(req_scope, packet, delay_millis, path_hash_size);  // same scope as the request
    break;
  case mesh::REPLY_SCOPE_DEFAULT:
    // requester's scope is unknown: a DIRECT request (no transport codes), or a
    // code that matched no Region. Un-scoped would be dropped at hop 0 by every
    // repeater running flood.max.unscoped=0.
    sendFloodScoped(default_scope, packet, delay_millis, path_hash_size);
    break;
  case mesh::REPLY_SCOPE_NONE:
    sendFlood(packet, delay_millis, path_hash_size);  // send un-scoped
    break;
  }
}

void RoomServerMesh::applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) {
  set_radio_at = futureMillis(2000); // give CLI reply some time to be sent back, before applying temp radio params
  pending_freq = freq;
  pending_bw = bw;
  pending_sf = sf;
  pending_cr = cr;

  revert_radio_at = futureMillis(2000 + timeout_mins * 60 * 1000); // schedule when to revert radio params
}

// ZEPHCORE: ours. Zephyr storage.
bool RoomServerMesh::formatFileSystem() {
  if (_store) {
    return _store->formatFileSystem();
  }
  return false;
}

void RoomServerMesh::sendSelfAdvertisement(int delay_millis, bool flood) {
  mesh::Packet *pkt = createSelfAdvert();
  if (pkt) {
    if (flood) {
      sendFloodScoped(default_scope, pkt, delay_millis, _prefs.path_hash_mode + 1);
    } else {
      sendZeroHop(pkt, delay_millis);
    }
  } else {
    MESH_DEBUG_PRINTLN("ERROR: unable to create advertisement packet!");
  }
}

void RoomServerMesh::updateAdvertTimer() {
  if (_prefs.advert_interval > 0) { // schedule local advert timer
    next_local_advert = futureMillis((uint32_t)_prefs.advert_interval * 2 * 60 * 1000);
  } else {
    next_local_advert = 0; // stop the timer
  }
}
void RoomServerMesh::updateFloodAdvertTimer() {
  if (_prefs.flood_advert_interval > 0) { // schedule flood advert timer
    next_flood_advert = futureMillis(((uint32_t)_prefs.flood_advert_interval) * 60 * 60 * 1000);
  } else {
    next_flood_advert = 0; // stop the timer
  }
}

// ZEPHCORE: ours. No log file.
void RoomServerMesh::dumpLogFile() {
  // Logging to file not implemented in Zephyr version
  LOG_INF("Log dump not implemented");
}

// ZEPHCORE: ours. No runtime TX-power API.
void RoomServerMesh::setTxPower(int8_t power_dbm) {
  radio_set_tx_power(power_dbm);
}

// ZEPHCORE: ours. Through the LoRa adapter.
bool RoomServerMesh::setRxBoostedGain(bool enable) {
  return getRadioDriver(_radio).setRxBoost(enable);
}

// ZEPHCORE: ours. Through RepeaterDataStore.
void RoomServerMesh::saveIdentity(const mesh::LocalIdentity& new_id) {
  if (_store) {
    _store->saveIdentity(new_id);
  }
}

void RoomServerMesh::startRegionsLoad() {
  temp_map.resetFrom(region_map);   // rebuild regions in a temp instance
  memset(load_stack, 0, sizeof(load_stack));
  load_stack[0] = &temp_map.getWildcard();
  region_load_active = true;
}

// ZEPHCORE: ours. Through RepeaterDataStore.
bool RoomServerMesh::saveRegions() {
  return region_map.save(_store->getFS());
}

void RoomServerMesh::onDefaultRegionChanged(const RegionEntry* r) {
  if (r) {
    region_map.getTransportKeysFor(*r, &default_scope, 1);
  } else {
    memset(default_scope.key, 0, sizeof(default_scope.key));
  }
}

// ZEPHCORE: ours. Also clears the radio adapter counters.
void RoomServerMesh::clearStats() {
  auto& radio_driver = getRadioDriver(_radio);
  radio_driver.resetStats();
  radio_driver.resetDutyCycleTimeoutRestarts();
  resetStats();
  ((SimpleMeshTables *)getTables())->resetStats();
}

// ZEPHCORE: ours. Our board handle.
void RoomServerMesh::formatStatsReply(char* reply) {
  StatsFormatHelper::formatCoreStats(reply, _board, *_ms, _err_flags, _mgr);
}

// ZEPHCORE: ours. StatsFormatHelper over the LoRa adapter.
void RoomServerMesh::formatRadioStatsReply(char* reply) {
  auto& radio_driver = getRadioDriver(_radio);
  StatsFormatHelper::formatRadioStats(reply, _radio, radio_driver, getTotalAirTime(), getReceiveAirTime());
}

// ZEPHCORE: ours. StatsFormatHelper over the LoRa adapter.
void RoomServerMesh::formatPacketStatsReply(char* reply) {
  auto& radio_driver = getRadioDriver(_radio);
  StatsFormatHelper::formatPacketStats(reply, radio_driver, getNumSentFlood(), getNumSentDirect(),
               getNumRecvFlood(), getNumRecvDirect());
}

// ZEPHCORE: ours. Reply-header budget, blank lines, room.post.
void RoomServerMesh::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
  if (region_load_active) {
    handleRegionLoadLine(sender_timestamp, command, reply);
    return;
  }

  /* Blank line: nothing to run.  The USB reader forwards these (see
   * cli_rx_work_fn) because `region load` commits on one -- which is
   * handled above, before this returns. */
  if (StrHelper::isBlank(command)) { reply[0] = 0; return; }

  while (*command == ' ') command++;

  uint8_t hdr_used = 0;
  if (strlen(command) > 4 && command[2] == '|') {
    memcpy(reply, command, 3);
    reply += 3;
    command += 3;
    hdr_used = 3;
  }
  /* Tell the shared CLI how much of the caller's buffer is already spent, so
   * the self-limiting handlers do not write past temp[5 + CLI_REMOTE_REPLY_SIZE].
   * Set unconditionally: a stale value would shrink the next reply. */
  _cli.setReplyHeaderUsed(hdr_used);

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
    Serial.println("ACL:");
    for (int i = 0; i < acl.getNumClients(); i++) {
      auto c = acl.getClientByIdx(i);
      if (c->permissions == 0) continue;  // skip deleted (or guest) entries

      Serial.printf("%02X ", c->permissions);
      mesh::Utils::printHex(Serial, c->id.pub_key, PUB_KEY_SIZE);
      Serial.printf("\n");
    }
    reply[0] = 0;
  } else if (memcmp(command, "room.post", 9) == 0) {
    char* msg = command + 9;
    while (*msg == ' ') msg++;
    if (*msg == 0) {
      strcpy(reply, "Err - empty message");
    } else {
      addSystemPost(msg);
      strcpy(reply, "OK");
    }
  } else {
    _cli.handleCommand(sender_timestamp, command, reply);
  }
}

bool RoomServerMesh::saveFilter(ClientInfo* client) {
  return client->isAdmin();    // only save Admins
}

// ZEPHCORE: ours. Deadline-driven maintenance.
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
    acl.save(_store->getFS(), RoomServerMesh::saveFilter);
    dirty_contacts_expiry = 0;
  }

  timeSyncTick();

  uint32_t now = k_uptime_get();
  uptime_millis += now - last_millis;
  last_millis = now;
}


/* ---- ZEPHCORE: methods upstream does not have ---- */

void RoomServerMesh::savePrefs() {
  if (_store) {
    _store->savePrefs(_prefs);
  }
}

void RoomServerMesh::freezeRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) {
  auto& radio = getRadioDriver(_radio);
  if (!radio.hasRadioOverride()) {
    /* Holds the radio on the preset it is ALREADY running while _prefs
     * move ahead of it, so the learned CAD offset still applies — see
     * setRadioOverride(). */
    radio.setRadioOverride(freq, bw, sf, cr, /*visiting_new_preset=*/false);
  }
}

void RoomServerMesh::eraseLogFile() {
  // Logging to file not implemented in Zephyr version
  LOG_INF("Log erased");
}

bool RoomServerMesh::setFemRxGain(bool enable) {
  return getRadioDriver(_radio).setFemRxEnable(enable);
}

bool RoomServerMesh::configSideDetectors(const uint8_t* sfs, uint8_t num) {
  return getRadioDriver(_radio).configSideDetectors(sfs, num);
}

/* A room server keeps no neighbour table (it is not a repeater). */
void RoomServerMesh::formatNeighborsReply(char* reply) {
  strcpy(reply, "not supported");
}

uint32_t RoomServerMesh::getDutyCycleTimeoutRestarts() const {
  return getRadioDriver(_radio).getDutyCycleTimeoutRestarts();
}

void RoomServerMesh::resetDutyCycleTimeoutRestarts() {
  getRadioDriver(_radio).resetDutyCycleTimeoutRestarts();
}

/* A continuation line during `region load` (started by startRegionsLoad()). */
void RoomServerMesh::handleRegionLoadLine(uint32_t sender_timestamp, char* command, char* reply) {
  if (StrHelper::isBlank(command)) {
    region_map = temp_map;
    region_load_active = false;
    sprintf(reply, "OK - loaded %d regions", region_map.getCount());
  } else {
    char* np = command;
    while (*np == ' ') np++;
    int indent = np - command;

    /* An unindented, name-like line is a typed command, not a region row:
     * real rows are indent >= 1 (load_stack[0] is the wildcard), and the
     * one unindented line a client legitimately sends is the exported
     * wildcard header "*", whose '*' is not a name char.  Without this,
     * `region load` is only escapable by a blank line -- which the USB
     * reader discards (main_repeater.cpp) and a dead remote-admin client
     * never sends, stranding the CLI until a reboot.  Abort without
     * committing temp_map and run the command.  Must come BEFORE the
     * name-terminator write below, which would truncate `set foo 1` to
     * `set`. */
    if (indent == 0 && RegionMap::is_name_char((uint8_t)*np)) {
      region_load_active = false;
      handleCommand(sender_timestamp, command, reply);
      return;
    }

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
}

void RoomServerMesh::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id,
          uint32_t timestamp, const uint8_t* app_data,
          size_t app_data_len) {
  (void)app_data; (void)app_data_len;
  /* Signature already verified by mesh::Mesh before this hook fires.
   * Skip share rebroadcasts (transport codes {0,0}) — they replay stale
   * stored adverts and would churn the original sender's tenure. */
  if (packet->hasTransportCodes() &&
      packet->transport_codes[0] == 0 && packet->transport_codes[1] == 0) {
    return;
  }
  _timesync.onAdvertHeard(id.pub_key, timestamp, packet->getPathHashCount(),
        (uint32_t)(k_uptime_get() / 1000));
}

void RoomServerMesh::timeSyncTick() {
  if (!_prefs.meshtimesync) return;
  if (!_timesync.runTick(*getRTCClock())) return;

  /* Step applied (forward-only enforced in runTick) — shift wall-clock-
   * anchored bookkeeping with it. */
  int64_t delta = _timesync.lastStepDelta();
  for (int i = 0; i < acl.getNumClients(); i++) {
    ClientInfo* c = acl.getClientByIdx(i);
    if (c->last_activity == 0) continue;
    int64_t shifted = (int64_t)c->last_activity + delta;
    c->last_activity = (shifted > 0) ? (uint32_t)shifted : 1;
  }
  login_fail_limiter.reset();
}

/* When the push engine in loop() next has work. The engine acts only once
 * next_push has passed, so this is the later of next_push and the moment some
 * client has something for it: a push can go now, an in-flight push reaches
 * its ack_timeout, or a post clears POST_SYNC_DELAY_SECS (an RTC-seconds gate).
 * Idle when no client has anything, so a room with logged-in clients and no
 * new posts costs no wakes. Mirrors the eligibility test in loop(). */
uint32_t RoomServerMesh::msUntilNextPush() {
  uint32_t now = (uint32_t)k_uptime_get();
  uint32_t rtc_now = getRTCClock()->getCurrentTime();
  uint32_t work = mesh::MAINTENANCE_IDLE;

  for (int i = 0; i < acl.getNumClients() && work > 0; i++) {
    ClientInfo* c = acl.getClientByIdx(i);
    if (c->extra.room.pending_ack) {
      work = mesh::maintenanceSooner(work, mesh::maintenanceUntil(now, c->extra.room.ack_timeout));
      continue;
    }
    if (c->last_activity == 0 || c->extra.room.push_failures >= 3) continue;
    for (int k = 0; k < MAX_UNSYNCED_POSTS; k++) {
      PostInfo& p = posts[k];
      if (p.post_timestamp == 0 || p.post_timestamp <= c->extra.room.sync_since ||
          p.author.matches(c->id)) {
        continue;
      }
      uint32_t eligible_at = p.post_timestamp + POST_SYNC_DELAY_SECS;
      if (rtc_now >= eligible_at) {
        work = 0;
        break;
      }
      work = mesh::maintenanceSooner(work, (eligible_at - rtc_now) * 1000);
    }
  }
  if (work == mesh::MAINTENANCE_IDLE) return work;

  uint32_t gate = mesh::maintenanceUntil(now, next_push);
  return (work > gate) ? work : gate;
}

uint32_t RoomServerMesh::msUntilNextMaintenance() {
  uint32_t now = (uint32_t)_ms->getMillis();
  uint32_t next = mesh::Mesh::msUntilNextMaintenance();

  /* Advert timers.  0 means "disabled" for both. */
  if (next_flood_advert) {
    next = mesh::maintenanceSooner(next, mesh::maintenanceUntil(now, next_flood_advert));
  }
  if (next_local_advert) {
    next = mesh::maintenanceSooner(next, mesh::maintenanceUntil(now, next_local_advert));
  }

  /* Temporary radio params: apply, then revert. Both CLI-armed, both 0 when idle. */
  if (set_radio_at) {
    next = mesh::maintenanceSooner(next, mesh::maintenanceUntil(now, set_radio_at));
  }
  if (revert_radio_at) {
    next = mesh::maintenanceSooner(next, mesh::maintenanceUntil(now, revert_radio_at));
  }

  /* Deferred ACL write-back. */
  if (dirty_contacts_expiry) {
    next = mesh::maintenanceSooner(next, mesh::maintenanceUntil(now, dirty_contacts_expiry));
  }

  /* Mesh time sync evaluates at most every 15 min, and only when enabled. */
  if (_prefs.meshtimesync) {
    next = mesh::maintenanceSooner(
        next, _timesync.msUntilNextEval((uint32_t)(k_uptime_get() / 1000)));
  }

  return mesh::maintenanceSooner(next, msUntilNextPush());
}
