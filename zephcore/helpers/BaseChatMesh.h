/*
 * SPDX-License-Identifier: MIT
 * ZephCore BaseChatMesh - base class for chat-style mesh clients
 */

#pragma once

#include <mesh/Mesh.h>
#include "AdvertDataHelpers.h"
#include "TxtDataHelpers.h"
#include "ContactInfo.h"
#include "ChannelDetails.h"

#define MAX_TEXT_LEN    (10 * CIPHER_BLOCK_SIZE)

#ifdef CONFIG_ZEPHCORE_MAX_CONTACTS
#define MAX_CONTACTS  CONFIG_ZEPHCORE_MAX_CONTACTS
#else
#define MAX_CONTACTS  32
#endif

/* Headroom for transient "anon" contacts (type ADV_TYPE_NONE) created to service
 * non-contact requests (CMD_SEND_ANON_REQ to a pubkey not in the contact list).
 * These are never persisted or synced to the app. Mirrors upstream. */
#define MAX_ANON_CONTACTS  8

#ifdef CONFIG_ZEPHCORE_MAX_CHANNELS
#define MAX_GROUP_CHANNELS  CONFIG_ZEPHCORE_MAX_CHANNELS
#else
#define MAX_GROUP_CHANNELS  8
#endif

#define MAX_SEARCH_RESULTS   8

#ifdef CONFIG_ZEPHCORE_MAX_CONNECTIONS
#define MAX_CONNECTIONS  CONFIG_ZEPHCORE_MAX_CONNECTIONS
#else
#define MAX_CONNECTIONS  16
#endif

#define MSG_SEND_FAILED       0
#define MSG_SEND_SENT_FLOOD   1
#define MSG_SEND_SENT_DIRECT  2

#define REQ_TYPE_GET_STATUS          0x01
#define REQ_TYPE_KEEP_ALIVE          0x02
#define REQ_TYPE_GET_TELEMETRY       0x03
#define REQ_TYPE_GET_TELEMETRY_DATA  0x03  // alias

#define RESP_SERVER_LOGIN_OK         0

/* Telemetry permissions */
#define TELEM_PERM_BASE         0x01
#define TELEM_PERM_LOCATION     0x02
#define TELEM_PERM_ENVIRONMENT  0x04

/* Connection info for room server keep-alive */
struct ConnectionInfo {
	mesh::Identity server_id;
	unsigned long next_ping;
	uint32_t last_activity;
	uint32_t keep_alive_millis;
	uint32_t expected_ack;
};

class BaseChatMesh;

class ContactVisitor {
public:
	virtual void onContactVisit(const ContactInfo &contact) = 0;
};

class ContactsIterator {
	int next_idx = 0;
public:
	bool hasNext(const BaseChatMesh *mesh, ContactInfo &dest);
};

/**
 * Abstract Mesh class for common 'chat' client functionality.
 * Handles contact management, message encryption/decryption, ACK handling.
 */
class BaseChatMesh : public mesh::Mesh {
	friend class ContactsIterator;

	ContactInfo contacts[MAX_CONTACTS + MAX_ANON_CONTACTS];
	int num_contacts;
	int sort_array[MAX_CONTACTS + MAX_ANON_CONTACTS];
	int matching_peer_indexes[MAX_SEARCH_RESULTS];
	unsigned long txt_send_timeout;

	ChannelDetails channels[MAX_GROUP_CHANNELS];
	int num_channels;

	ConnectionInfo connections[MAX_CONNECTIONS];

	mesh::Packet *_pendingLoopback;
	uint8_t temp_buf[MAX_TRANS_UNIT];

	mesh::Packet *composeMsgPacket(const ContactInfo &recipient, uint32_t timestamp, uint8_t attempt,
		const char *text, uint32_t &expected_ack);
	void sendAckTo(const ContactInfo &dest, const uint8_t *ack_hash, uint8_t ack_len = 4);

	/* Shared flood-vs-direct dispatch tail used by sendMessage/sendCommandData/
	 * sendLogin/sendAnonReq/sendRequest. Sets est_timeout and (when
	 * set_txt_timeout) txt_send_timeout; returns MSG_SEND_SENT_FLOOD/DIRECT. */
	int dispatchToRecipient(mesh::Packet *pkt, const ContactInfo &recipient,
		uint32_t &est_timeout, bool set_txt_timeout);

protected:
	BaseChatMesh(mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng, mesh::RTCClock &rtc,
		mesh::PacketManager &mgr, mesh::MeshTables &tables)
		: mesh::Mesh(radio, ms, rng, rtc, mgr, tables)
	{
		num_contacts = 0;
		memset(channels, 0, sizeof(channels));
		num_channels = 0;
		txt_send_timeout = 0;
		_pendingLoopback = nullptr;
	}

	void bootstrapRTCfromContacts();
	void resetContacts() { num_contacts = 0; }
	void populateContactFromAdvert(ContactInfo &ci, const mesh::Identity &id, const AdvertDataParser &parser, uint32_t timestamp);
	ContactInfo *allocateContactSlot(bool transient_only = false);

	// UI concepts for subclasses to implement
	virtual bool isAutoAddEnabled() const { return true; }
	virtual bool shouldAutoAddContactType(uint8_t type) const { return true; }
	virtual void onContactsFull() {}
	virtual bool shouldOverwriteWhenFull() const { return false; }
	virtual uint8_t getAutoAddMaxHops() const { return 0; }  // 0 = no limit, 1 = direct (0 hops), N = up to N-1 hops
	virtual void onContactOverwrite(const uint8_t *pub_key) {}
	virtual void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) = 0;
	virtual ContactInfo *processAck(const uint8_t *data) = 0;
	virtual void onContactPathUpdated(const ContactInfo &contact) = 0;
	virtual bool onContactPathRecv(ContactInfo &from, uint8_t *in_path, uint8_t in_path_len,
		uint8_t *out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len);
	virtual void onMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) = 0;
	virtual void onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) = 0;
	virtual void onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp,
		const uint8_t *sender_prefix, const char *text) = 0;
	virtual uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const = 0;
	virtual uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const = 0;
	virtual void onSendTimeout() = 0;
	virtual void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp, const char *text) = 0;
	virtual void onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
		const uint8_t *data, size_t data_len) {}
	virtual uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp,
		const uint8_t *data, uint8_t len, uint8_t *reply) = 0;
	virtual void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) = 0;
	virtual void handleReturnPathRetry(const ContactInfo &contact, const uint8_t *path, uint8_t path_len);

	virtual void sendFloodScoped(const ContactInfo &recipient, mesh::Packet *pkt, uint32_t delay_millis = 0);
	virtual void sendFloodScoped(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t delay_millis = 0);

	virtual void onLoginSent(const ContactInfo &contact) {}
	virtual void onChannelAdded(ChannelDetails *ch) {}

	/* Every signature-verified advert, before contact filtering/dedup —
	 * mesh time-sync harvesting hook. */
	virtual void onAdvertTimeSample(const mesh::Identity &id, uint32_t timestamp,
		uint8_t hops) { (void)id; (void)timestamp; (void)hops; }

	// Storage concepts for subclasses to override
	virtual int getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) { return 0; }
	virtual bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len) { return false; }

	// Mesh overrides
	void onAdvertRecv(mesh::Packet *packet, const mesh::Identity &id, uint32_t timestamp,
		const uint8_t *app_data, size_t app_data_len) override;
	int searchPeersByHash(const uint8_t *hash) override;
	void getPeerSharedSecret(uint8_t *dest_secret, int peer_idx) override;
	void onPeerDataRecv(mesh::Packet *packet, uint8_t type, int sender_idx, const uint8_t *secret,
		uint8_t *data, size_t len) override;
	bool onPeerPathRecv(mesh::Packet *packet, int sender_idx, const uint8_t *secret, uint8_t *path,
		uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len) override;
	void onAckRecv(mesh::Packet *packet, uint32_t ack_crc) override;
	int searchChannelsByHash(const uint8_t *hash, mesh::GroupChannel channels[], int max_matches) override;
	void onGroupDataRecv(mesh::Packet *packet, uint8_t type, const mesh::GroupChannel &channel,
		uint8_t *data, size_t len) override;

public:
	mesh::Packet *createSelfAdvert(const char *name);
	mesh::Packet *createSelfAdvert(const char *name, double lat, double lon);

	/* Build and send a self advert honoring the node's location policy,
	 * path-hash mode, and transport scope. Overridden by CompanionMesh
	 * (the only mesh with interactive advert UI triggers); the base default
	 * does nothing and returns false. Lets UI screens that only hold a
	 * BaseChatMesh* trigger adverts via the canonical scoped path instead
	 * of a bare sendFlood() that drops path_hash_mode and scope. */
	virtual bool sendSelfAdvert(bool flood) { (void)flood; return false; }
	int sendMessage(const ContactInfo &recipient, uint32_t timestamp, uint8_t attempt, const char *text,
		uint32_t &expected_ack, uint32_t &est_timeout);
	int sendCommandData(const ContactInfo &recipient, uint32_t timestamp, uint8_t attempt, const char *text,
		uint32_t &est_timeout);
	/* @param out_hash if non-null, filled with the FNV-1a packet hash so the
	 * caller can later query the contention tracker for "how many neighbors
	 * heard and retransmitted this?" — used by the joystick UI's send-feedback
	 * mechanism. When provided, the packet is also pre-registered with the
	 * contention tracker (so heard dupes match). */
	bool sendGroupMessage(uint32_t timestamp, mesh::GroupChannel &channel, const char *sender_name,
		const char *text, int text_len, uint32_t *out_hash = nullptr);
	bool sendGroupData(mesh::GroupChannel &channel, uint8_t *path, uint8_t path_len,
		uint16_t data_type, const uint8_t *data, int data_len);
	int sendLogin(const ContactInfo &recipient, const char *password, uint32_t &est_timeout);
	int sendAnonReq(const ContactInfo &recipient, const uint8_t *data, uint8_t len, uint32_t &tag, uint32_t &est_timeout);
	int sendRequest(const ContactInfo &recipient, uint8_t req_type, uint32_t &tag, uint32_t &est_timeout);
	int sendRequest(const ContactInfo &recipient, const uint8_t *req_data, uint8_t data_len, uint32_t &tag, uint32_t &est_timeout);
	bool shareContactZeroHop(const ContactInfo &contact);
	uint8_t exportContact(const ContactInfo &contact, uint8_t dest_buf[]);
	bool importContact(const uint8_t src_buf[], uint8_t len);
	void resetPathTo(ContactInfo &recipient);
	void scanRecentContacts(int last_n, ContactVisitor *visitor);
	ContactInfo *searchContactsByPrefix(const char *name_prefix);
	ContactInfo *lookupContactByPubKey(const uint8_t *pub_key, int prefix_len);
	bool removeContact(ContactInfo &contact);
	bool addContact(const ContactInfo &contact);
	int getNumContacts() const { return num_contacts; }
	bool getContactByIdx(uint32_t idx, ContactInfo &contact);
	ContactsIterator startContactsIterator();
	ChannelDetails *addChannel(const char *name, const uint8_t *psk, size_t psk_len);
	bool getChannel(int idx, ChannelDetails &dest);
	bool setChannel(int idx, const ChannelDetails &src);
	int findChannelIdx(const mesh::GroupChannel &ch);
	int getNumChannels() const { return num_channels; }

	/* Connection management for room servers */
	bool startConnection(const ContactInfo &contact, uint16_t keep_alive_secs);
	void stopConnection(const uint8_t *pub_key);
	bool hasConnectionTo(const uint8_t *pub_key);
	void markConnectionActive(const ContactInfo &contact);
	ContactInfo *checkConnectionsAck(const uint8_t *data);
	void checkConnections();

	void loop();
};
