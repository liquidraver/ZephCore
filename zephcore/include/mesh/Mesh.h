/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Mesh - routing protocol layer
 */

#pragma once

#include <mesh/Dispatcher.h>
#include <mesh/ContentionTracker.h>
#ifdef CONFIG_ZEPHCORE_APC
#include <mesh/PowerController.h>
#endif
#include <mesh/RTC.h>

namespace mesh {

struct GroupChannel {
	uint8_t hash[PATH_HASH_SIZE];
	uint8_t secret[PUB_KEY_SIZE];
};

class MeshTables {
public:
	virtual bool hasSeen(const Packet *packet) = 0;
	virtual void clear(const Packet *packet) = 0;
};

class Mesh : public Dispatcher {
	RNG *_rng;
	RTCClock *_rtc;
	MeshTables *_tables;

	void removeSelfFromPath(Packet *packet);
	void routeDirectRecvAcks(Packet *packet, uint32_t delay_millis);
	DispatcherAction forwardMultipartDirect(Packet *pkt);

protected:
	ContentionTracker _contention;
	ContentionTracker& getContentionTracker() { return _contention; }
	const ContentionTracker& getContentionTracker() const { return _contention; }
#ifdef CONFIG_ZEPHCORE_APC
	PowerController _power_ctrl;
	PowerController& getPowerController() { return _power_ctrl; }
	const PowerController& getPowerController() const { return _power_ctrl; }
#endif
	void extendPendingRetransmit(uint32_t hash32);

	DispatcherAction onRecvPacket(Packet *pkt) override;
	virtual uint32_t getCADFailRetryDelay() const override;
	virtual DispatcherAction routeRecvPacket(Packet *packet);
	virtual bool filterRecvFloodPacket(Packet *packet) { return false; }
	virtual bool allowPacketForward(const Packet *packet);
	virtual uint32_t getRetransmitDelay(const Packet *packet);
	virtual uint32_t getDirectRetransmitDelay(const Packet *packet) { return 0; }
	/* Passive contention tracking: if true, track heard floods we don't forward
	 * (warms the contention EMA on nodes that don't relay, e.g. companions). */
	virtual bool passivelyTrackFloods() const { return false; }
	/* Added to caller-supplied delay on every sendFlood.  Default 0 (repeater
	 * behavior).  Companion overrides to spread its initial TX adaptively. */
	virtual uint32_t getInitialFloodJitter(const Packet *packet) { (void)packet; return 0; }
	virtual uint8_t getExtraAckTransmitCount() const { return 0; }
	virtual int searchPeersByHash(const uint8_t *hash) { (void)hash; return 0; }
	virtual void getPeerSharedSecret(uint8_t *dest_secret, int peer_idx) { (void)dest_secret; (void)peer_idx; }
	virtual void onPeerDataRecv(Packet *packet, uint8_t type, int sender_idx, const uint8_t *secret, uint8_t *data, size_t len) { (void)packet; (void)type; (void)sender_idx; (void)secret; (void)data; (void)len; }
	virtual void onTraceRecv(Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags, const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) { (void)packet; (void)tag; (void)auth_code; (void)flags; (void)path_snrs; (void)path_hashes; (void)path_len; }
	virtual bool onPeerPathRecv(Packet *packet, int sender_idx, const uint8_t *secret, uint8_t *path, uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len) { (void)packet; (void)sender_idx; (void)secret; (void)path; (void)path_len; (void)extra_type; (void)extra; (void)extra_len; return false; }
	virtual void onAdvertRecv(Packet *packet, const Identity &id, uint32_t timestamp, const uint8_t *app_data, size_t app_data_len) { (void)packet; (void)id; (void)timestamp; (void)app_data; (void)app_data_len; }
	virtual void onAnonDataRecv(Packet *packet, const uint8_t *secret, const Identity &sender, uint8_t *data, size_t len) { (void)packet; (void)secret; (void)sender; (void)data; (void)len; }
	virtual void onPathRecv(Packet *packet, Identity &sender, uint8_t *path, uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len) { (void)packet; (void)sender; (void)path; (void)path_len; (void)extra_type; (void)extra; (void)extra_len; }
	virtual void onControlDataRecv(Packet *packet) { (void)packet; }
	virtual void onRawDataRecv(Packet *packet) { (void)packet; }
	virtual int searchChannelsByHash(const uint8_t *hash, GroupChannel channels[], int max_matches) { (void)hash; (void)channels; (void)max_matches; return 0; }
	virtual void onGroupDataRecv(Packet *packet, uint8_t type, const GroupChannel &channel, uint8_t *data, size_t len) { (void)packet; (void)type; (void)channel; (void)data; (void)len; }
	virtual void onAckRecv(Packet *packet, uint32_t ack_crc) { (void)packet; (void)ack_crc; }

public:
	Mesh(Radio &radio, MillisecondClock &ms, RNG &rng, RTCClock &rtc, PacketManager &mgr, MeshTables &tables);
	void begin();
	void loop();
	void maintenanceLoop();

	LocalIdentity self_id;

	RNG *getRNG() const { return _rng; }
	RTCClock *getRTCClock() const { return _rtc; }
	MeshTables *getTables() const { return _tables; }

	Packet *createAdvert(const LocalIdentity &id, const uint8_t *app_data = nullptr, size_t app_data_len = 0);
	Packet *createAck(uint32_t ack_crc);
	Packet *createMultiAck(uint32_t ack_crc, uint8_t remaining);
	Packet *createControlData(const uint8_t *data, size_t len);
	Packet *createDatagram(uint8_t type, const Identity &dest, const uint8_t *secret, const uint8_t *data, size_t len);
	Packet *createAnonDatagram(uint8_t type, const LocalIdentity &sender, const Identity &dest, const uint8_t *secret, const uint8_t *data, size_t data_len);
	Packet *createGroupDatagram(uint8_t type, const GroupChannel &channel, const uint8_t *data, size_t data_len);
	Packet *createPathReturn(const Identity &dest, const uint8_t *secret, const uint8_t *path, uint8_t path_len, uint8_t extra_type, const uint8_t *extra, size_t extra_len);
	Packet *createPathReturn(const uint8_t *dest_hash, const uint8_t *secret, const uint8_t *path, uint8_t path_len, uint8_t extra_type, const uint8_t *extra, size_t extra_len);
	Packet *createRawData(const uint8_t *data, size_t len);
	Packet *createTrace(uint32_t tag, uint32_t auth_code, uint8_t flags = 0);

	void sendFlood(Packet *packet, uint32_t delay_millis = 0, uint8_t path_hash_size = 1);
	void sendFlood(Packet *packet, uint16_t *transport_codes, uint32_t delay_millis = 0, uint8_t path_hash_size = 1);
	void sendDirect(Packet *packet, const uint8_t *path, uint8_t path_len, uint32_t delay_millis = 0);
	void sendZeroHop(Packet *packet, uint32_t delay_millis = 0);
	void sendZeroHop(Packet *packet, uint16_t *transport_codes, uint32_t delay_millis = 0);
};

} /* namespace mesh */
