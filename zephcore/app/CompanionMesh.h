/*
 * SPDX-License-Identifier: MIT
 * CompanionMesh - the companion role (upstream: examples/companion_radio/MyMesh)
 */

#pragma once

#include <helpers/BaseChatMesh.h>
#include <helpers/BaseSerialInterface.h>
#include <helpers/MeshTimeSync.h>
#include <helpers/TransportKeyStore.h>
#include <ZephyrDataStore.h>
#include <NodePrefs.h>
#include <zephyr/kernel.h>

/* Push notification codes */
#define PUSH_CODE_ADVERT              0x80
#define PUSH_CODE_PATH_UPDATED        0x81
#define PUSH_CODE_SEND_CONFIRMED      0x82
#define PUSH_CODE_MSG_WAITING         0x83
#define PUSH_CODE_RAW_DATA            0x84
#define PUSH_CODE_LOGIN_SUCCESS       0x85
#define PUSH_CODE_LOGIN_FAIL          0x86
#define PUSH_CODE_STATUS_RESPONSE     0x87
#define PUSH_CODE_LOG_RX_DATA         0x88
#define PUSH_CODE_TRACE_DATA          0x89
#define PUSH_CODE_NEW_ADVERT          0x8A
#define PUSH_CODE_TELEMETRY_RESPONSE  0x8B
#define PUSH_CODE_BINARY_RESPONSE     0x8C
#define PUSH_CODE_PATH_DISCOVERY_RESP 0x8D
#define PUSH_CODE_CONTROL_DATA        0x8E
#define PUSH_CODE_CONTACT_DELETED     0x8F
#define PUSH_CODE_CONTACTS_FULL       0x90

#define REQ_TYPE_GET_TELEMETRY_DATA   0x03

/* Auto-add config bitmask */
#define AUTO_ADD_OVERWRITE_OLDEST  (1 << 0)
#define AUTO_ADD_CHAT              (1 << 1)
#define AUTO_ADD_REPEATER          (1 << 2)
#define AUTO_ADD_ROOM_SERVER       (1 << 3)
#define AUTO_ADD_SENSOR            (1 << 4)

/* 1 header + 32 pubkey + 1 type + 1 flags + 1 path_len + 64 path + 32 name + 4*4 fields */
#define CONTACT_FRAME_SIZE 148

#ifdef CONFIG_ZEPHCORE_OFFLINE_QUEUE_SIZE
#define OFFLINE_QUEUE_SIZE CONFIG_ZEPHCORE_OFFLINE_QUEUE_SIZE
#else
#define OFFLINE_QUEUE_SIZE 16
#endif

#ifdef CONFIG_ZEPHCORE_ACK_TABLE_SIZE
#define ACK_TABLE_SIZE CONFIG_ZEPHCORE_ACK_TABLE_SIZE
#else
#define ACK_TABLE_SIZE 16
#endif

#ifdef CONFIG_ZEPHCORE_ADVERT_PATH_TABLE_SIZE
#define ADVERT_PATH_TABLE_SIZE CONFIG_ZEPHCORE_ADVERT_PATH_TABLE_SIZE
#else
#define ADVERT_PATH_TABLE_SIZE 16
#endif

/* A recently heard advert's path */
struct AdvertPath {
	uint8_t pubkey_prefix[7];
	uint8_t path_len;
	char name[32];
	uint32_t recv_timestamp;
	uint8_t path[MAX_PATH_SIZE];
};

typedef uint16_t (*GetBatteryCallback)(void);

/* preset_changed: freq/bw/sf moved, not just TX power; the adaptive-CAD state
 * is tied to those and is reset with them. */
typedef void (*RadioReconfigureCallback)(bool preset_changed);

typedef void (*PinChangeCallback)(uint32_t new_pin);

/* Runs one CLI command for CompanionMesh::handleCommand (prefix already
 * stripped). main_companion owns the CommonCLI behind it. sender_timestamp 0
 * means local, where the buffer is COMPANION_CLI_PREFIX_ROOM +
 * COMPANION_CLI_REPLY_SIZE; a remote buffer is CLI_REMOTE_REPLY_SIZE.
 * reply_hdr_used is what the reflected prefix took of it. */
#define COMPANION_CLI_REPLY_SIZE 256
#define COMPANION_CLI_PREFIX_ROOM 3
typedef void (*CompanionCLICallback)(const char *command, uint32_t sender_timestamp,
	uint8_t reply_hdr_used, char *reply);

class CompanionMesh : public BaseChatMesh, public DataStoreHost {
public:
	CompanionMesh(mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
		mesh::RTCClock &rtc, mesh::PacketManager &mgr, mesh::MeshTables &tables,
		ZephyrDataStore &store);

	void begin();
	void loop();

	int getOfflineQueueCount() const { return _offline_queue_count; }

	/* The one self-advert path (app command and UI buttons alike), so every
	 * advert honours the location policy, path-hash mode and default scope.
	 * False if the packet pool was full. */
	bool sendSelfAdvert(bool flood) override;

	/* One companion-protocol frame from the app; true if handled. */
	bool handleCmdFrame(const uint8_t *data, size_t len);

	/* As upstream: every frame to and from the app goes through `serial` */
	void startInterface(BaseSerialInterface &serial);
	void setBatteryCallback(GetBatteryCallback cb) { _batt_cb = cb; }
	/* For the MCU temperature in telemetry (upstream: the global `board`). */
	void setBoard(mesh::MainBoard *board) { _board = board; }
	void setRadioReconfigureCallback(RadioReconfigureCallback cb) { _radio_reconfig_cb = cb; }
	void setPinChangeCallback(PinChangeCallback cb) { _pin_change_cb = cb; }
	void setCLICallback(CompanionCLICallback cb) { _cli_exec_cb = cb; }

	/* The companion CLI (upstream MyMesh::handleCommand). Front-ends: the app's
	 * CMD_RUN_CLI_COMMAND, the USB text console and the v-contact chat (all
	 * local, sender_timestamp 0), and TXT_TYPE_CLI_COMMAND from a contact with
	 * flag 0x10. False for an unknown command. */
	bool handleCommand(const char *command, uint32_t sender_timestamp, char *reply);

	/* ---- V-contact ("v<node_name>"), see app/VContact.cpp ----
	 * A chat contact that exists only toward the connected app; messages to it
	 * run the CLI and never reach the radio. */
	bool isVContactEnabled() const { return prefs.v_contact_enabled != 0; }
	/* Unsolicited notice (battery alert, restart reason), via the offline queue. */
	void vcontactNotify(const char *text);
	/* Push it as NEW_ADVERT after a runtime enable or rename. */
	void vcontactPushAdvert();
	/* Push CONTACT_DELETED after a runtime disable. */
	void vcontactPushDeleted();
	/* The clock may have become valid: activate a deferred v-contact and flush
	 * buffered notices. Self-gating, safe to call speculatively. */
	void vcontactClockSynced();
	/* A delivery ack is still waiting; reboot-class commands wait for it. */
	bool vcontactConfirmPending() const { return _vcontact_confirm_ack != 0; }

	/* Streams the contact dump; call each loop. True while contacts remain. */
	bool continueContactIteration();

	/* On disconnect: nobody to send CONTACT_END to. */
	void cancelContactIterator() { _contact_iter_active = false; }

	/* Dump state for the stall watchdog: it re-kicks only if the index stops. */
	bool isContactIterActive() const { return _contact_iter_active; }
	int getContactIterIdx() const { return _contact_iter_idx; }

	/* Housekeeping tick: re-prompt MSG_WAITING when the offline queue has gone
	 * quiet. A queued message reaches the app through one best-effort prompt;
	 * if that was dropped, held, or ignored, the queue sat until reconnect.
	 * MSG_WAITING is idempotent, so a duplicate costs nothing. */
	void msgWaitingWatchdog();

	/* On disconnect: the un-ACKed message stays queued and is re-sent. */
	void cancelSyncPending() { _sync_pending = false; }

	/* On disconnect: free the 8 KB sign buffer a session may have abandoned
	 * between CMD_SIGN_START and CMD_SIGN_FINISH. */
	void cleanupSignState() {
		if (_sign_data) {
			delete[] _sign_data;
			_sign_data = nullptr;
		}
		_sign_data_len = 0;
		_sign_data_capacity = 0;
	}

	const char *getDeviceName() const { return prefs.node_name[0] ? prefs.node_name : nullptr; }

	int getRecentlyHeard(AdvertPath dest[], int max_num);
	const AdvertPath *findAdvertPath(const uint8_t *pubkey_prefix, int prefix_len);

	/* A DM sent from the device UI, queued for the app with path_len
	 * OUT_PATH_SENT and a "(>>✓) "/"(>>✗) " delivery marker. */
	void queueLocalSentContactMessage(const ContactInfo &contact, uint32_t timestamp,
			const char *text, bool delivered);

	/* A channel message sent from the device UI, queued for the app as
	 * "<marker> <node_name>: <text>"; the marker says whether a repeat was heard. */
	void queueLocalSentChannelMessage(uint8_t channel_idx, uint32_t timestamp,
			const char *text, bool heard_repeat);

	/* DataStoreHost interface */
	bool onContactLoaded(const ContactInfo &c) override;
	bool getContactForSave(uint32_t idx, ContactInfo &c) override;
	bool onChannelLoaded(uint8_t idx, const ChannelDetails &ch) override;
	bool getChannelForSave(uint8_t idx, ChannelDetails &ch) override;

	/* Mesh time sync */
	MeshTimeSync *getMeshTimeSync() { return &_timesync; }
	void noteGPSTimeSync() { _timesync.noteGPSSync((uint32_t)(k_uptime_get() / 1000)); }
	/* From the housekeeping event: loop() only runs on packet events. */
	void timeSyncTick();

	/* Write out any lazily-deferred contacts/channels now. Main thread only;
	 * every reboot and power-off path reaches it through
	 * zephcore_persist_before_off(). */
	void flushPendingWrites() { flushDirtyContacts(); flushDirtyChannels(); }

	NodePrefs prefs;

protected:
	/* BaseChatMesh */
	void onAdvertRecv(mesh::Packet *packet, const mesh::Identity &id, uint32_t timestamp,
			  const uint8_t *app_data, size_t app_data_len) override;
	void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) override;
	ContactInfo *processAck(const uint8_t *data) override;
	void onContactPathUpdated(const ContactInfo &contact) override;
	void onMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override;
	void onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override;
	void onCLICommandRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp,
		const char *text, char *reply) override;
	void onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override;
	uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override;
	uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override;
	void onSendTimeout() override;
	void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp, const char *text) override;
	void onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
		const uint8_t *data, size_t data_len) override;
	uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data, uint8_t len, uint8_t *reply) override;
	void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override;

	/* Raw packet logging for the app's RX log */
	void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
	void logRx(mesh::Packet *pkt, int len, float score) override;
	void logTx(mesh::Packet *pkt, int len) override;

	void onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
		const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) override;
	void onControlDataRecv(mesh::Packet *packet) override;
	void onRawDataRecv(mesh::Packet *packet) override;

	/* Client repeat / off-grid forwarding */
	bool allowPacketForward(const mesh::Packet *packet) override;

	/* Path discovery: sees the path data before the base class strips it */
	bool onContactPathRecv(ContactInfo &from, uint8_t *in_path, uint8_t in_path_len,
		uint8_t *out_path, uint8_t out_path_len, uint8_t extra_type,
		uint8_t *extra, uint8_t extra_len) override;

	/* Region-scoped flooding */
	void sendFloodScoped(const TransportKey &scope, mesh::Packet *pkt, uint32_t delay_millis);
	void sendFloodScoped(const ContactInfo &recipient, mesh::Packet *pkt, uint32_t delay_millis = 0) override;
	void sendFloodScoped(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t delay_millis = 0) override;

	uint32_t getRetransmitDelay(const mesh::Packet *packet) override;
	uint32_t getDirectRetransmitDelay(const mesh::Packet *packet) override;

	/* Fixed-window initial jitter; passive flood tracking only matters when
	 * forwarding. */
	bool passivelyTrackFloods() const override { return false; }
	uint32_t getInitialFloodJitter(const mesh::Packet *packet) override;

	uint8_t getDutyCyclePercent() const override;
	uint8_t getExtraAckTransmitCount() const override;

	/* Adaptive CAD: persist the staircase's learned offset */
	void onCadOffsetChanged(int8_t offset) override {
		prefs.cad_offset = offset;
		if (_store) {
			_store->savePrefs(prefs);
		}
	}

	/* Auto-add filtering */
	bool isAutoAddEnabled() const override;
	bool shouldAutoAddContactType(uint8_t type) const override;
	bool shouldOverwriteWhenFull() const override;
	uint8_t getAutoAddMaxHops() const override;
	void onContactsFull() override;
	void onContactOverwrite(const uint8_t *pub_key) override;

	int getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) override;
	bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len) override;

private:
	ZephyrDataStore *_store;
	BaseSerialInterface *_serial;
	GetBatteryCallback _batt_cb;
	mesh::MainBoard *_board = nullptr;
	RadioReconfigureCallback _radio_reconfig_cb;
	PinChangeCallback _pin_change_cb;

	/* Contact dump. The bound and the v-contact's inclusion are snapshotted at
	 * PACKET_CONTACT_START, so the dump never streams more than the total it
	 * announced: the table grows mid-dump, and the v-contact appears the moment
	 * a cold-booted clock becomes valid. */
	bool _contact_iter_active;
	int _contact_iter_idx;
	int _contact_iter_num;
	bool _contact_iter_vc;
	uint32_t _contact_iter_lastmod;
	uint32_t _contact_iter_since;  /* only contacts with lastmod > this */

	/* Offline message queue */
	struct QueuedFrame {
		uint8_t len;
		uint8_t buf[172];
	};
	QueuedFrame _offline_queue[OFFLINE_QUEUE_SIZE];
	int _offline_queue_head;
	int _offline_queue_tail;
	int _offline_queue_count;
	bool _sync_pending;  /* the last peeked message is not yet ACKed by the app */

	struct AckEntry {
		uint32_t expected_ack;
		uint32_t sent_time;
		int contact_idx;
		bool active;
	};
	AckEntry _ack_table[ACK_TABLE_SIZE];

	AdvertPath _advert_paths[ADVERT_PATH_TABLE_SIZE];

	/* CMD_SIGN_* state */
	uint8_t *_sign_data;
	uint32_t _sign_data_len;
	uint32_t _sign_data_capacity;

	/* Pending requests, for matching responses */
	uint32_t _pending_login;
	uint32_t _pending_status;
	uint32_t _pending_telemetry;
	uint32_t _pending_discovery;
	uint32_t _pending_req;
#ifdef CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK
	uint32_t _pending_joystick_ping_tag;  /* matched by tag, before _pending_status */
	uint32_t _pending_joystick_admin_tag; /* same, for admin binary requests */
#endif

	/* Lazy contacts/channels writes */
	int64_t _dirty_contacts_expiry;
	int64_t _dirty_channels_expiry;
	static constexpr int64_t LAZY_WRITE_DELAY_MS = 5000;  /* as upstream */

	/* Deadline for a liveness-only change (a known contact re-advertised with
	 * nothing but a newer timestamp): only last_advert_timestamp and lastmod
	 * moved. saveContacts() rewrites the whole file (~40 KB, ten 4 KB blocks
	 * on a T1000-E's 128 KB /lfs); on a busy mesh something is always
	 * re-advertising, so this deadline IS the rewrite rate. 5 s (upstream)
	 * was a rewrite per advert, 10 min was 144 a day, and one hour is 24,
	 * below the adv_blobs write per advert. Every clean reboot and power-off
	 * flushes first (flushPendingWrites()), so only a crash or a pulled
	 * battery loses the last hour of "last heard" times. A substantive
	 * change still pulls the deadline in. */
	static constexpr int64_t LAZY_WRITE_LIVENESS_MS = 3600000;  /* 1 hour */

	void markContactsDirty(bool substantive = true);
	void markChannelsDirty();
	void flushDirtyContacts();
	void flushDirtyChannels();

	/* What the contact record held before the advert being processed, so
	 * onDiscoveredContact() can tell an addition or a real change (name,
	 * type, position) from a re-advert. Set by onAdvertRecv() around the
	 * base class call. */
	struct AdvertPrev {
		bool known;
		uint8_t type;
		int32_t gps_lat, gps_lon;
		char name[sizeof(ContactInfo::name)];
	} _advert_prev;

	/* Forward-only: our clock stamps outgoing DMs, and peers keep per-sender
	 * replay high-water marks. */
	MeshTimeSync _timesync{FIRMWARE_BUILD_EPOCH, true};
	void onAdvertTimeSample(const mesh::Identity &id, uint32_t timestamp,
		uint8_t hops) override;

	uint8_t _app_target_ver;

	/* Flood scope for transport filtering (all zeros = disabled) */
	TransportKey _send_scope;
	bool _send_scope_force_unscoped;

	/* The joystick ping tag is independent of app requests and not cleared. */
	void clearPendingReqs() {
		_pending_login = _pending_status = _pending_telemetry = _pending_discovery = _pending_req = 0;
	}

#ifdef CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK
public:
	void setJoystickPingTag(uint32_t tag)  { _pending_joystick_ping_tag = tag; }
	void clearJoystickPingTag()            { _pending_joystick_ping_tag = 0; }
	void setJoystickAdminTag(uint32_t tag) { _pending_joystick_admin_tag = tag; }
	void clearJoystickAdminTag()           { _pending_joystick_admin_tag = 0; }
	/* The joystick UI clears a stale out_path_len on its fallback flood. */
	void markContactsDirtyPublic() { markContactsDirty(); }
private:
#endif

	bool writeFrame(const uint8_t *data, size_t len);
	/* Companion protocol opcode handlers, in upstream handleCmdFrame order */
	bool handleCmdDeviceQuery(const uint8_t *data, size_t len);
	bool handleCmdAppStart(const uint8_t *data, size_t len);
	bool handleCmdRunCliCommand(const uint8_t *data, size_t len);
	bool handleCmdSendTxtMsg(const uint8_t *data, size_t len);
	bool handleCmdSendChannelTxtMsg(const uint8_t *data, size_t len);
	bool handleCmdSendChannelData(const uint8_t *data, size_t len);
	bool handleCmdGetContacts(const uint8_t *data, size_t len);
	bool handleCmdSetAdvertName(const uint8_t *data, size_t len);
	bool handleCmdSetAdvertLatlon(const uint8_t *data, size_t len);
	bool handleCmdGetDeviceTime(const uint8_t *data, size_t len);
	bool handleCmdSetDeviceTime(const uint8_t *data, size_t len);
	bool handleCmdSendSelfAdvert(const uint8_t *data, size_t len);
	bool handleCmdResetPath(const uint8_t *data, size_t len);
	bool handleCmdAddUpdateContact(const uint8_t *data, size_t len);
	bool handleCmdRemoveContact(const uint8_t *data, size_t len);
	bool handleCmdShareContact(const uint8_t *data, size_t len);
	bool handleCmdGetContactByKey(const uint8_t *data, size_t len);
	bool handleCmdExportContact(const uint8_t *data, size_t len);
	bool handleCmdImportContact(const uint8_t *data, size_t len);
	bool handleCmdSyncNextMessage(const uint8_t *data, size_t len);
	bool handleCmdSetRadioParams(const uint8_t *data, size_t len);
	bool handleCmdSetRadioTxPower(const uint8_t *data, size_t len);
	bool handleCmdSetTuningParams(const uint8_t *data, size_t len);
	bool handleCmdGetTuningParams(const uint8_t *data, size_t len);
	bool handleCmdSetOtherParams(const uint8_t *data, size_t len);
	bool handleCmdSetPathHashMode(const uint8_t *data, size_t len);
	bool handleCmdReboot(const uint8_t *data, size_t len);
	bool handleCmdGetBattAndStorage(const uint8_t *data, size_t len);
	bool handleCmdExportPrivateKey(const uint8_t *data, size_t len);
	bool handleCmdImportPrivateKey(const uint8_t *data, size_t len);
	bool handleCmdSendRawData(const uint8_t *data, size_t len);
	bool handleCmdSendLogin(const uint8_t *data, size_t len);
	bool handleCmdSendAnonReq(const uint8_t *data, size_t len);
	bool handleCmdSendStatusReq(const uint8_t *data, size_t len);
	bool handleCmdSendPathDiscoveryReq(const uint8_t *data, size_t len);
	bool handleCmdSendTelemetryReq(const uint8_t *data, size_t len);
	bool handleCmdSendBinaryReq(const uint8_t *data, size_t len);
	bool handleCmdHasConnection(const uint8_t *data, size_t len);
	bool handleCmdLogout(const uint8_t *data, size_t len);
	bool handleCmdGetChannel(const uint8_t *data, size_t len);
	bool handleCmdSetChannel(const uint8_t *data, size_t len);
	bool handleCmdSignStart(const uint8_t *data, size_t len);
	bool handleCmdSignData(const uint8_t *data, size_t len);
	bool handleCmdSignFinish(const uint8_t *data, size_t len);
	bool handleCmdSendTracePath(const uint8_t *data, size_t len);
	bool handleCmdSetDevicePin(const uint8_t *data, size_t len);
	bool handleCmdGetCustomVars(const uint8_t *data, size_t len);
	bool handleCmdSetCustomVar(const uint8_t *data, size_t len);
	bool handleCmdGetAdvertPath(const uint8_t *data, size_t len);
	bool handleCmdGetStats(const uint8_t *data, size_t len);
	bool handleCmdFactoryReset(const uint8_t *data, size_t len);
	bool handleCmdSetFloodScopeKey(const uint8_t *data, size_t len);
	bool handleCmdSetDefaultFloodScope(const uint8_t *data, size_t len);
	bool handleCmdGetDefaultFloodScope(const uint8_t *data, size_t len);
	bool handleCmdSendControlData(const uint8_t *data, size_t len);
	bool handleCmdSetAutoaddConfig(const uint8_t *data, size_t len);
	bool handleCmdGetAutoaddConfig(const uint8_t *data, size_t len);
	bool handleCmdGetAllowedRepeatFreq(const uint8_t *data, size_t len);
	bool handleCmdSendRawPacket(const uint8_t *data, size_t len);

	void writeOKFrame();
	void writeErrFrame(uint8_t code);
	/* [PACKET_SENT][is_flood][tag:4][est_timeout:4] */
	void sendPacketSent(uint8_t result, uint32_t tag, uint32_t est_timeout);
	void sendPush(uint8_t code, const uint8_t *data = nullptr, size_t len = 0);

	/* The default-scope body shared by the recipient and channel overloads */
	void sendFloodScopedDefault(mesh::Packet *pkt, uint32_t delay_millis);

	/* Self telemetry as Cayenne LPP; bytes written. Battery is always included,
	 * `permissions` gates location and environment. */
	int appendSelfTelemetry(uint8_t *out, uint8_t permissions);

	/* header != 0 is prepended; buf must hold CONTACT_FRAME_SIZE. */
	static size_t serializeContact(uint8_t *buf, const ContactInfo &c, uint8_t header = 0);

	void addToOfflineQueue(const uint8_t *data, size_t len);
	bool peekOfflineMessage(uint8_t *dest, size_t &len);
	void confirmOfflineMessage();
	bool enqueuePendingChannelInfo(uint8_t idx);
	bool sendChannelInfoFrame(uint8_t idx);
	void drainPendingChannelInfos();

	void queueMessage(const ContactInfo &contact, uint8_t txt_type, mesh::Packet *pkt,
		 uint32_t sender_timestamp, const uint8_t *extra, int extra_len, const char *text);

	CompanionCLICallback _cli_exec_cb;

	/* ---- V-contact state (app/VContact.cpp) ---- */
	/* _vcontact_lastmod == 0: not activated yet. The clock was invalid when it
	 * would have been stamped, so it is withheld rather than shown as 1970. */
	uint8_t _vcontact_pubkey[PUB_KEY_SIZE];
	uint32_t _vcontact_lastmod;
	/* App resends reuse the message timestamp and can land seconds later,
	 * behind other messages, hence a ring rather than one last-seen slot. */
	static const uint8_t VCONTACT_DEDUP_SLOTS = 16;
	uint32_t _vcontact_recent_ts[VCONTACT_DEDUP_SLOTS];
	uint8_t _vcontact_recent_head;
	char _vcontact_pending[2][128];   /* notices held while the clock is invalid */
	uint8_t _vcontact_pending_count;
	/* Holds a notice's MSG_WAITING from CMD_APP_START until CMD_GET_CONTACTS:
	 * a prompt there makes the app interleave a message sync into the contact
	 * stream. Bounded by _vcontact_hold_expiry, because the app need not ever
	 * sync to empty; the mid-dump case is checked directly in
	 * vcontactMsgWaitHeld(). See VCONTACT_HOLD_MAX_MS. */
	bool _vcontact_hold_msgwait;
	int64_t _vcontact_hold_expiry;   /* _ms->getMillis() deadline */
	/* Reads the latch, expiring it first; use instead of the flag. */
	bool vcontactMsgWaitHeld();
	/* Offline-queue watchdog: it re-prompts once the later of these is quiet. */
	int64_t _last_msgwait_ms;   /* last PUSH_CODE_MSG_WAITING we emitted */
	int64_t _last_sync_req_ms;  /* last CMD_SYNC_NEXT_MESSAGE from the app */
	/* Every MSG_WAITING goes through here, so the watchdog never fires over a
	 * live sync. */
	void pushMsgWaiting();
	/* An app-side delete hides the v-contact for the session only: a "purge
	 * all contacts" walks it like any other entry. `set v.contact off` is the
	 * durable disable. Cleared at CMD_APP_START. */
	bool _vcontact_app_hidden;
	/* Deferred SEND_CONFIRMED for a v-contact message (0 = none). Emitted
	 * inline it lands sub-millisecond after the response to the same write,
	 * before the app has committed the message, and the app drops it; the
	 * resend's ack then sticks, seconds late. See VCONTACT_CONFIRM_DELAY_MS. */
	uint32_t _vcontact_confirm_ack;
	/* CONTAINER_OF relies on offsetof, which is only conditionally supported on
	 * a non-standard-layout type like CompanionMesh; this POD carries its own
	 * back-pointer instead. */
	struct ConfirmWork {
		struct k_work_delayable work;
		CompanionMesh *self;
	};
	ConfirmWork _vcontact_confirm_work;
	static void vcontactConfirmWorkHandler(struct k_work *work);
	/* Emit a pending SEND_CONFIRMED now. Idempotent. */
	void vcontactFlushConfirm();
	bool vcontactClockValid();
	bool vcontactReady() {
		return isVContactEnabled() && !_vcontact_app_hidden && _vcontact_lastmod != 0;
	}
	void buildVContact(ContactInfo &c) const;
	bool isVContactKey(const uint8_t *key, int prefix_len) const;
	/* On boot and whenever the identity changes (CMD_IMPORT_PRIVATE_KEY). */
	void deriveVContactKey();
	/* True when a frame addressed to the v-contact was fully handled. */
	bool vcontactHandleFrame(const uint8_t *data, size_t len);
	/* Chunks `text` into v-contact messages on the offline queue. */
	void vcontactQueueText(const char *text);

	void addPendingAck(uint32_t expected, int contact_idx);
	int findAndRemoveAck(uint32_t ack, uint32_t *out_sent_time = nullptr);

	uint8_t _pending_channel_idx[MAX_GROUP_CHANNELS];
	uint8_t _pending_channel_head;
	uint8_t _pending_channel_tail;
	uint8_t _pending_channel_count;
};
