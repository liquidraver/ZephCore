/*
 * SPDX-License-Identifier: Apache-2.0
 * CompanionMesh - ZephCore Companion device application layer
 */

#pragma once

#include <helpers/BaseChatMesh.h>
#include <helpers/TransportKeyStore.h>
#include <ZephyrDataStore.h>
#include <NodePrefs.h>

/* BLE push notification codes */
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

/* Auto-add config bitmask */
#define AUTO_ADD_OVERWRITE_OLDEST  (1 << 0)
#define AUTO_ADD_CHAT              (1 << 1)
#define AUTO_ADD_REPEATER          (1 << 2)
#define AUTO_ADD_ROOM_SERVER       (1 << 3)
#define AUTO_ADD_SENSOR            (1 << 4)

/* Max BLE frame size — must match NUS MTU negotiated size */
#define MAX_FRAME_SIZE 172

/* 1 header + 32 pubkey + 1 type + 1 flags + 1 path_len + 64 path + 32 name + 4*4 fields = 148 */
#define CONTACT_FRAME_SIZE 148

/* Offline message queue depth */
#ifdef CONFIG_ZEPHCORE_OFFLINE_QUEUE_SIZE
#define OFFLINE_QUEUE_SIZE CONFIG_ZEPHCORE_OFFLINE_QUEUE_SIZE
#else
#define OFFLINE_QUEUE_SIZE 16
#endif

/* Pending ACK tracking slots */
#ifdef CONFIG_ZEPHCORE_ACK_TABLE_SIZE
#define ACK_TABLE_SIZE CONFIG_ZEPHCORE_ACK_TABLE_SIZE
#else
#define ACK_TABLE_SIZE 8
#endif

/* Recently-heard advert path slots */
#ifdef CONFIG_ZEPHCORE_ADVERT_PATH_TABLE_SIZE
#define ADVERT_PATH_TABLE_SIZE CONFIG_ZEPHCORE_ADVERT_PATH_TABLE_SIZE
#else
#define ADVERT_PATH_TABLE_SIZE 16
#endif

/* Cached advert path entry */
struct AdvertPath {
	uint8_t pubkey_prefix[7];
	uint8_t path_len;
	char name[32];
	uint32_t recv_timestamp;
	uint8_t path[MAX_PATH_SIZE];
};

/* BLE push notification callback */
typedef void (*PushCallback)(uint8_t code, const uint8_t *data, size_t len);

/* BLE write frame callback */
typedef size_t (*WriteFrameCallback)(const uint8_t *data, size_t len);

/* Battery millivolt read callback */
typedef uint16_t (*GetBatteryCallback)(void);

/* Radio reconfigure callback */
typedef void (*RadioReconfigureCallback)(void);

/* BLE PIN change callback */
typedef void (*PinChangeCallback)(uint32_t new_pin);

/**
 * CompanionMesh: Application layer for ZephCore Companion device
 *
 * Extends BaseChatMesh with:
 * - BLE protocol frame handling
 * - Offline message queue
 * - Push notifications for incoming messages/adverts
 * - ACK tracking for sent messages
 */
class CompanionMesh : public BaseChatMesh, public DataStoreHost {
public:
	CompanionMesh(mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
		mesh::RTCClock &rtc, mesh::PacketManager &mgr, mesh::MeshTables &tables,
		ZephyrDataStore &store);

	void begin();
	void loop();

	int getOfflineQueueCount() const { return _offline_queue_count; }

	/**
	 * Handle a protocol frame from BLE.
	 * Returns true if frame was handled.
	 */
	bool handleProtocolFrame(const uint8_t *data, size_t len);

	/**
	 * Set callback for BLE push notifications.
	 */
	void setPushCallback(PushCallback cb) { _push_cb = cb; }

	/**
	 * Set callback for writing response frames to BLE.
	 */
	void setWriteFrameCallback(WriteFrameCallback cb) { _write_cb = cb; }

	/**
	 * Set callback for getting battery voltage.
	 */
	void setBatteryCallback(GetBatteryCallback cb) { _batt_cb = cb; }

	/**
	 * Set callback for radio reconfigure.
	 */
	void setRadioReconfigureCallback(RadioReconfigureCallback cb) { _radio_reconfig_cb = cb; }

	/**
	 * Set callback for BLE PIN change.
	 */
	void setPinChangeCallback(PinChangeCallback cb) { _pin_change_cb = cb; }

	/**
	 * Continue contact iteration (call each main loop iteration).
	 * Returns true if contacts are still being sent.
	 */
	bool continueContactIteration();

	/**
	 * Reset contact iterator (call when new command received).
	 * Sends PACKET_CONTACT_END if iteration was in progress.
	 */
	void resetContactIterator();

	/**
	 * Cancel contact iteration silently (no frame sent).
	 * Call on BLE disconnect — there's nobody to send CONTACT_END to.
	 */
	void cancelContactIterator() { _contact_iter_active = false; }

	/**
	 * Cancel pending message sync. Un-ACKed message stays in queue.
	 * Call on BLE disconnect so the message is re-sent on reconnect.
	 */
	void cancelSyncPending() { _sync_pending = false; }

	/**
	 * Free the Ed25519 signing buffer if allocated.
	 * Call on BLE disconnect to prevent 8KB leak when disconnect
	 * occurs between CMD_SIGN_START and CMD_SIGN_FINISH.
	 */
	void cleanupSignState() {
		if (_sign_data) {
			delete[] _sign_data;
			_sign_data = nullptr;
		}
		_sign_data_len = 0;
		_sign_data_capacity = 0;
	}

	/**
	 * Get BLE device name for advertising.
	 */
	const char *getDeviceName() const { return prefs.node_name[0] ? prefs.node_name : nullptr; }

	/**
	 * Get recently heard advert paths.
	 */
	int getRecentlyHeard(AdvertPath dest[], int max_num);

	/**
	 * Find advert path by pubkey prefix.
	 */
	const AdvertPath *findAdvertPath(const uint8_t *pubkey_prefix, int prefix_len);

	/* DataStoreHost interface */
	bool onContactLoaded(const ContactInfo &c) override;
	bool getContactForSave(uint32_t idx, ContactInfo &c) override;
	bool onChannelLoaded(uint8_t idx, const ChannelDetails &ch) override;
	bool getChannelForSave(uint8_t idx, ChannelDetails &ch) override;

	/* Prefs (includes node_lat/lon) */
	NodePrefs prefs;

protected:
	/* BaseChatMesh virtual implementations */
	void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) override;
	ContactInfo *processAck(const uint8_t *data) override;
	void onContactPathUpdated(const ContactInfo &contact) override;
	void onMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override;
	void onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override;
	void onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override;
	uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override;
	uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override;
	void onSendTimeout() override;
	void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp, const char *text) override;
	void onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
		const uint8_t *data, size_t data_len) override;
	uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data, uint8_t len, uint8_t *reply) override;
	void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override;

	/* Raw packet logging for app RX log */
	void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;

	/* Trace path response */
	void onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
		const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) override;

	/* Control data response (repeater discovery, etc) */
	void onControlDataRecv(mesh::Packet *packet) override;

	/* Raw data response (custom packets) */
	void onRawDataRecv(mesh::Packet *packet) override;

	/* Packet forwarding (client repeat / offgrid mode) */
	bool allowPacketForward(const mesh::Packet *packet) override;

	/* Path discovery - intercept path data before base class strips it */
	bool onContactPathRecv(ContactInfo &from, uint8_t *in_path, uint8_t in_path_len,
		uint8_t *out_path, uint8_t out_path_len, uint8_t extra_type,
		uint8_t *extra, uint8_t extra_len) override;

	/* Flood scope - scoped sending for region filtering */
	void sendFloodScoped(const TransportKey &scope, mesh::Packet *pkt, uint32_t delay_millis);
	void sendFloodScoped(const ContactInfo &recipient, mesh::Packet *pkt, uint32_t delay_millis = 0) override;
	void sendFloodScoped(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t delay_millis = 0) override;

	/* Dispatcher tuning (uses prefs) */
	uint32_t getRetransmitDelay(const mesh::Packet *packet) override;
	uint32_t getDirectRetransmitDelay(const mesh::Packet *packet) override;

	/* Companion initial flood jitter is fixed-window; passive flood
	 * tracking is not needed unless forwarding is enabled. */
	bool passivelyTrackFloods() const override { return false; }
	uint32_t getInitialFloodJitter(const mesh::Packet *packet) override;

	uint8_t getDutyCyclePercent() const override;
	uint8_t getExtraAckTransmitCount() const override;

	/* Auto-add filtering overrides */
	bool isAutoAddEnabled() const override;
	bool shouldAutoAddContactType(uint8_t type) const override;
	bool shouldOverwriteWhenFull() const override;
	uint8_t getAutoAddMaxHops() const override;
	void onContactsFull() override;
	void onContactOverwrite(const uint8_t *pub_key) override;

	/* Storage overrides */
	int getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) override;
	bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len) override;

private:
	ZephyrDataStore *_store;
	PushCallback _push_cb;
	WriteFrameCallback _write_cb;
	GetBatteryCallback _batt_cb;
	RadioReconfigureCallback _radio_reconfig_cb;
	PinChangeCallback _pin_change_cb;

	/* Contact iteration state */
	bool _contact_iter_active;
	int _contact_iter_idx;
	uint32_t _contact_iter_lastmod;
	uint32_t _contact_iter_since;  /* Filter: only send contacts with lastmod > this */

	/* Offline message queue */
	struct QueuedFrame {
		uint8_t len;
		uint8_t buf[172];
	};
	QueuedFrame _offline_queue[OFFLINE_QUEUE_SIZE];
	int _offline_queue_head;
	int _offline_queue_tail;
	int _offline_queue_count;
	bool _sync_pending;  /* true = last peeked message not yet ACKed by phone */

	/* ACK tracking table */
	struct AckEntry {
		uint32_t expected_ack;
		uint32_t sent_time;
		int contact_idx;
		bool active;
	};
	AckEntry _ack_table[ACK_TABLE_SIZE];
	int _ack_next_overwrite;

	/* Advert path table for tracking recently heard nodes */
	AdvertPath _advert_paths[ADVERT_PATH_TABLE_SIZE];
	int _next_advert_path_idx;

	/* Signing state */
	uint8_t *_sign_data;
	uint32_t _sign_data_len;
	uint32_t _sign_data_capacity;

	/* Pending request tracking (for response matching) */
	uint32_t _pending_login;
	uint32_t _pending_status;
	uint32_t _pending_telemetry;
	uint32_t _pending_discovery;
	uint32_t _pending_req;

	/* Lazy contacts/channels write - batches rapid updates */
	int64_t _dirty_contacts_expiry;
	int64_t _dirty_channels_expiry;
	static constexpr int64_t LAZY_WRITE_DELAY_MS = 5000;  /* 5 seconds, matches Arduino */

	void markContactsDirty();
	void markChannelsDirty();
	void flushDirtyContacts();
	void flushDirtyChannels();

	/* Protocol version negotiation */
	uint8_t _app_target_ver;

	/* Flood scope for transport filtering (all zeros = disabled) */
	TransportKey _send_scope;
	bool _send_scope_force_unscoped;

	void clearPendingReqs() {
		_pending_login = _pending_status = _pending_telemetry = _pending_discovery = _pending_req = 0;
	}

	bool writeFrame(const uint8_t *data, size_t len);
	void sendPacketOk();
	void sendPacketError(uint8_t code);
	void sendPush(uint8_t code, const uint8_t *data = nullptr, size_t len = 0);

	/** Serialize a ContactInfo into buf. If header != 0, prepend it.
	 *  Returns total bytes written. buf must be >= CONTACT_FRAME_SIZE. */
	static size_t serializeContact(uint8_t *buf, const ContactInfo &c, uint8_t header = 0);

	void queueOfflineMessage(const uint8_t *data, size_t len);
	bool dequeueOfflineMessage(uint8_t *dest, size_t &len);
	bool peekOfflineMessage(uint8_t *dest, size_t &len);
	void confirmOfflineMessage();
	bool enqueuePendingChannelInfo(uint8_t idx);
	bool sendChannelInfoFrame(uint8_t idx);
	void drainPendingChannelInfos();

	void queueContactMessage(const ContactInfo &contact, mesh::Packet *pkt,
		uint8_t txt_type, uint32_t sender_timestamp, const uint8_t *extra, int extra_len, const char *text);

	void addPendingAck(uint32_t expected, int contact_idx);
	int findAndRemoveAck(uint32_t ack, uint32_t *out_sent_time = nullptr);

	uint8_t _pending_channel_idx[MAX_GROUP_CHANNELS];
	uint8_t _pending_channel_head;
	uint8_t _pending_channel_tail;
	uint8_t _pending_channel_count;
};
