/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Mesh - minimal port for Phase 5
 */

#include <mesh/Mesh.h>
#include <mesh/Utils.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_mesh, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

namespace mesh {

Mesh::Mesh(Radio &radio, MillisecondClock &ms, RNG &rng, RTCClock &rtc, PacketManager &mgr, MeshTables &tables)
	: Dispatcher(radio, ms, mgr), _rng(&rng), _rtc(&rtc), _tables(&tables)
{
}

void Mesh::begin()
{
	Dispatcher::begin();
}

void Mesh::loop()
{
	Dispatcher::loop();
}

void Mesh::maintenanceLoop()
{
	Dispatcher::maintenanceLoop();
	uint32_t now = (uint32_t)_ms->getMillis();
	_contention.tick(now);
#ifdef CONFIG_ZEPHCORE_APC
	_power_ctrl.tick(now);
	_radio->setTxPowerReduction(_power_ctrl.getPowerReduction());
#endif
}

void Mesh::extendPendingRetransmit(uint32_t hash32)
{
	uint32_t now = (uint32_t)_ms->getMillis();
	int total = _mgr->getOutboundTotal();
	for (int i = 0; i < total; i++) {
		Packet *pkt = _mgr->getOutboundByIdx(i);
		if (pkt && pkt->isRouteFlood()
		    && ContentionTracker::computePacketHash32(pkt) == hash32) {
			uint32_t airtime = _radio->getEstAirtimeFor(pkt->getRawLength());
			uint16_t delay = _contention.getReactiveHeadroom(hash32, airtime);
			if (delay == 0) break;
			/* Reschedule from NOW: heard a dupe, defer by one
			 * backoff_multiplier × airtime window per dupe. */
			_mgr->rescheduleOutbound(i, now + delay);
			_contention.addReactiveExtension(hash32, delay);
			notifyTxQueued(delay);
			break;
		}
	}
}

bool Mesh::allowPacketForward(const Packet *packet)
{
	(void)packet;
	return false;
}

uint32_t Mesh::getRetransmitDelay(const Packet *packet)
{
	uint32_t t = (_radio->getEstAirtimeFor(packet->getRawLength()) * 52 / 50) / 2;
	return _rng->nextInt(0, 5) * t;
}

uint32_t Mesh::getCADFailRetryDelay() const
{
	return _rng->nextInt(1, 4) * 120;
}

void Mesh::removeSelfFromPath(Packet *pkt)
{
	pkt->setPathHashCount(pkt->getPathHashCount() - 1);  // decrement the count

	uint8_t sz = pkt->getPathHashSize();
	for (int k = 0; k < pkt->getPathHashCount()*sz; k += sz) {  // shuffle path by 1 'entry'
		memcpy(&pkt->path[k], &pkt->path[k + sz], sz);
	}
}

DispatcherAction Mesh::routeRecvPacket(Packet *packet)
{
	uint8_t n = packet->getPathHashCount();
	if (packet->isRouteFlood() && !packet->isMarkedDoNotRetransmit()
		&& (n + 1)*packet->getPathHashSize() <= MAX_PATH_SIZE && allowPacketForward(packet)) {
		// append this node's hash to 'path'
		self_id.copyHashTo(&packet->path[n * packet->getPathHashSize()], packet->getPathHashSize());
		packet->setPathHashCount(n + 1);
		uint32_t h = ContentionTracker::computePacketHash32(packet);
		_contention.trackRetransmit(h, (uint32_t)_ms->getMillis());
#ifdef CONFIG_ZEPHCORE_APC
		_power_ctrl.trackTransmit(h, (uint32_t)_ms->getMillis());
#endif
		uint32_t d = getRetransmitDelay(packet);
		return ACTION_RETRANSMIT_DELAYED(packet->getPathHashCount(), d);  // give priority to closer sources
	}
	return ACTION_RELEASE;
}

DispatcherAction Mesh::forwardMultipartDirect(Packet *pkt)
{
	uint8_t remaining = pkt->payload[0] >> 4;
	uint8_t type = pkt->payload[0] & 0x0F;
	if (type == PAYLOAD_TYPE_ACK && pkt->payload_len >= 5) {
		Packet tmp;
		tmp.header = pkt->header;
		tmp.path_len = Packet::copyPath(tmp.path, pkt->path, pkt->path_len);
		tmp.payload_len = pkt->payload_len - 1;
		memcpy(tmp.payload, &pkt->payload[1], tmp.payload_len);
		if (!_tables->hasSeen(&tmp)) {
			removeSelfFromPath(&tmp);
			routeDirectRecvAcks(&tmp, ((uint32_t)remaining + 1) * 300);
		}
	}
	return ACTION_RELEASE;
}

void Mesh::routeDirectRecvAcks(Packet *packet, uint32_t delay_millis)
{
	if (!packet->isMarkedDoNotRetransmit()) {
		uint32_t crc;
		memcpy(&crc, packet->payload, 4);
		Packet *a2 = createAck(crc);
		if (a2) {
			a2->path_len = Packet::copyPath(a2->path, packet->path, packet->path_len);
			a2->header &= ~PH_ROUTE_MASK;
			a2->header |= ROUTE_TYPE_DIRECT;
			sendPacket(a2, 0, delay_millis);
		}
	}
}

DispatcherAction Mesh::onRecvPacket(Packet *pkt)
{
	// Handle direct TRACE packets
	if (pkt->isRouteDirect() && pkt->getPayloadType() == PAYLOAD_TYPE_TRACE) {
		if (pkt->path_len < MAX_PATH_SIZE) {
			int i = 0;
			uint32_t trace_tag;
			memcpy(&trace_tag, &pkt->payload[i], 4); i += 4;
			uint32_t auth_code;
			memcpy(&auth_code, &pkt->payload[i], 4); i += 4;
			uint8_t flags = pkt->payload[i++];
			uint8_t path_sz = flags & 0x03;

			uint8_t len = pkt->payload_len - i;
			uint8_t offset = pkt->path_len << path_sz;
			if (offset >= len) {
				onTraceRecv(pkt, trace_tag, auth_code, flags, pkt->path, &pkt->payload[i], len);
			} else if (self_id.isHashMatch(&pkt->payload[i + offset], 1 << path_sz) && allowPacketForward(pkt) && !_tables->hasSeen(pkt)) {
				pkt->path[pkt->path_len++] = (int8_t)(pkt->getSNR() * 4);
				uint32_t d = getDirectRetransmitDelay(pkt);
				return ACTION_RETRANSMIT_DELAYED(5, d);
			}
		}
		return ACTION_RELEASE;
	}

	// Handle direct CONTROL packets (zero-hop only)
	if (pkt->isRouteDirect() && pkt->getPayloadType() == PAYLOAD_TYPE_CONTROL && (pkt->payload[0] & 0x80) != 0) {
		if (pkt->getPathHashCount() == 0) {
			onControlDataRecv(pkt);
		}
		return ACTION_RELEASE;
	}

	// Handle direct zero-hop ACKs (path_len=0)
	if (pkt->isRouteDirect() && pkt->getPathHashCount() == 0 && pkt->getPayloadType() == PAYLOAD_TYPE_ACK) {
		uint32_t ack_crc;
		memcpy(&ack_crc, pkt->payload, 4);
		onAckRecv(pkt, ack_crc);
		return ACTION_RELEASE;
	}

	if (pkt->isRouteDirect() && pkt->getPathHashCount() > 0) {
		if (pkt->getPayloadType() == PAYLOAD_TYPE_ACK) {
			uint32_t ack_crc;
			memcpy(&ack_crc, pkt->payload, 4);
			onAckRecv(pkt, ack_crc);
		}
		if (self_id.isHashMatch(pkt->path, pkt->getPathHashSize()) && allowPacketForward(pkt)) {
			if (pkt->getPayloadType() == PAYLOAD_TYPE_MULTIPART) {
				return forwardMultipartDirect(pkt);
			}
			if (pkt->getPayloadType() == PAYLOAD_TYPE_ACK) {
				if (!_tables->hasSeen(pkt)) {
					removeSelfFromPath(pkt);
					routeDirectRecvAcks(pkt, 0);
				}
				return ACTION_RELEASE;
			}
			if (!_tables->hasSeen(pkt)) {
				removeSelfFromPath(pkt);
				return ACTION_RETRANSMIT_DELAYED(0, getDirectRetransmitDelay(pkt));
			}
		}
		return ACTION_RELEASE;
	}

	if (pkt->isRouteFlood() && filterRecvFloodPacket(pkt)) return ACTION_RELEASE;

	/* Record dupes for contention tracking + reactive backoff */
	if (pkt->isRouteFlood()) {
		uint32_t h = ContentionTracker::computePacketHash32(pkt);
#ifdef CONFIG_ZEPHCORE_APC
		uint8_t first_hop = (pkt->getPathHashCount() > 0) ? pkt->path[0] : 0;
		_power_ctrl.recordEcho(h, pkt->_snr, first_hop, (uint32_t)_ms->getMillis());
#endif
		if (_contention.recordDupeIfTracked(h, (uint32_t)_ms->getMillis())) {
			extendPendingRetransmit(h);
		} else if (passivelyTrackFloods()) {
			/* First hearing of a flood we won't forward — track it so the
			 * EMA reflects local contention (companion-side awareness). */
			_contention.trackRetransmit(h, (uint32_t)_ms->getMillis());
		}
	}

	DispatcherAction action = ACTION_RELEASE;

	switch (pkt->getPayloadType()) {
	case PAYLOAD_TYPE_ACK: {
		uint32_t ack_crc;
		memcpy(&ack_crc, pkt->payload, 4);
		if (!_tables->hasSeen(pkt)) {
			onAckRecv(pkt, ack_crc);
			action = routeRecvPacket(pkt);
		}
		break;
	}
	case PAYLOAD_TYPE_PATH:
	case PAYLOAD_TYPE_REQ:
	case PAYLOAD_TYPE_RESPONSE:
	case PAYLOAD_TYPE_TXT_MSG: {
		int i = 0;
		uint8_t dest_hash = pkt->payload[i++];
		uint8_t src_hash = pkt->payload[i++];

		uint8_t *macAndData = &pkt->payload[i];
		if (i + CIPHER_MAC_SIZE >= (int)pkt->payload_len) {
			LOG_WRN("onRecvPacket: incomplete packet (i=%d, payload_len=%d)", i, pkt->payload_len);
		} else if (!_tables->hasSeen(pkt)) {
			if (self_id.isHashMatch(&dest_hash)) {
				int num = searchPeersByHash(&src_hash);
				bool found = false;
				for (int j = 0; j < num; j++) {
					uint8_t secret[PUB_KEY_SIZE];
					getPeerSharedSecret(secret, j);

					uint8_t data[MAX_PACKET_PAYLOAD];
					int len = Utils::MACThenDecrypt(secret, data, macAndData, pkt->payload_len - i);
					if (len > 0) {
						if (pkt->getPayloadType() == PAYLOAD_TYPE_PATH) {
							int k = 0;
							uint8_t path_len = data[k++];
							if (!Packet::isValidPathLen(path_len)) {
								LOG_WRN("onRecvPacket: invalid inner path_len 0x%02x", path_len);
								break;
							}
							uint8_t hash_size = (path_len >> 6) + 1;
							uint8_t hash_count = path_len & 63;
							int path_bytes = hash_size * hash_count;
							if (k + path_bytes + 1 > len) {
								LOG_WRN("onRecvPacket: PATH payload truncated (k=%d path_bytes=%d len=%d)", k, path_bytes, len);
								break;
							}
							uint8_t *path = &data[k]; k += path_bytes;
							uint8_t extra_type = data[k++] & 0x0F;
							uint8_t *extra = &data[k];
							uint8_t extra_len = (uint8_t)(len - k);
							if (onPeerPathRecv(pkt, j, secret, path, path_len, extra_type, extra, extra_len)) {
								if (pkt->isRouteFlood()) {
									Packet *rpath = createPathReturn(&src_hash, secret, pkt->path, pkt->path_len, 0, nullptr, 0);
									if (rpath) sendDirect(rpath, path, path_len, 500);
								}
							}
						} else {
							onPeerDataRecv(pkt, pkt->getPayloadType(), j, secret, data, len);
						}
						found = true;
						break;
					}
				}
				if (found) {
					pkt->markDoNotRetransmit();
				} else {
					LOG_WRN("onRecvPacket: no peer could decrypt message");
				}
			}
			action = routeRecvPacket(pkt);
		}
		break;
	}
	case PAYLOAD_TYPE_ANON_REQ: {
		int i = 0;
		uint8_t dest_hash = pkt->payload[i++];
		uint8_t *sender_pub_key = &pkt->payload[i]; i += PUB_KEY_SIZE;

		uint8_t *macAndData = &pkt->payload[i];
		if (i + 2 >= (int)pkt->payload_len) {
			// incomplete packet
		} else if (!_tables->hasSeen(pkt)) {
			if (self_id.isHashMatch(&dest_hash)) {
				Identity sender(sender_pub_key);
				uint8_t secret[PUB_KEY_SIZE];
				self_id.calcSharedSecret(secret, sender);

				uint8_t data[MAX_PACKET_PAYLOAD];
				int len = Utils::MACThenDecrypt(secret, data, macAndData, pkt->payload_len - i);
				if (len > 0) {
					onAnonDataRecv(pkt, secret, sender, data, len);
					pkt->markDoNotRetransmit();
				}
			}
			action = routeRecvPacket(pkt);
		}
		break;
	}
	case PAYLOAD_TYPE_GRP_DATA:
	case PAYLOAD_TYPE_GRP_TXT: {
		int i = 0;
		uint8_t channel_hash = pkt->payload[i++];

		uint8_t *macAndData = &pkt->payload[i];
		if (i + 2 >= (int)pkt->payload_len) {
			// incomplete packet
		} else if (!_tables->hasSeen(pkt)) {
			GroupChannel channels[4];
			int num = searchChannelsByHash(&channel_hash, channels, 4);
			for (int j = 0; j < num; j++) {
				uint8_t data[MAX_PACKET_PAYLOAD];
				int len = Utils::MACThenDecrypt(channels[j].secret, data, macAndData, pkt->payload_len - i);
				if (len > 0) {
					onGroupDataRecv(pkt, pkt->getPayloadType(), channels[j], data, len);
					break;
				}
			}
			action = routeRecvPacket(pkt);
		}
		break;
	}
	case PAYLOAD_TYPE_ADVERT: {
		int i = 0;
		Identity id;
		memcpy(id.pub_key, &pkt->payload[i], PUB_KEY_SIZE);
		i += PUB_KEY_SIZE;
		uint32_t timestamp;
		memcpy(&timestamp, &pkt->payload[i], 4);
		i += 4;
		const uint8_t *signature = &pkt->payload[i];
		i += SIGNATURE_SIZE;
		if (i <= (int)pkt->payload_len && !self_id.matches(id.pub_key) && !_tables->hasSeen(pkt)) {
			uint8_t *app_data = (uint8_t *)&pkt->payload[i];
			size_t app_data_len = pkt->payload_len - (size_t)i;
			if (app_data_len > MAX_ADVERT_DATA_SIZE) app_data_len = MAX_ADVERT_DATA_SIZE;
			uint8_t message[PUB_KEY_SIZE + 4 + MAX_ADVERT_DATA_SIZE];
			int msg_len = 0;
			memcpy(&message[msg_len], id.pub_key, PUB_KEY_SIZE); msg_len += PUB_KEY_SIZE;
			memcpy(&message[msg_len], &timestamp, 4); msg_len += 4;
			memcpy(&message[msg_len], app_data, app_data_len); msg_len += app_data_len;
			if (id.verify(signature, message, msg_len)) {
				onAdvertRecv(pkt, id, timestamp, app_data, app_data_len);
				action = routeRecvPacket(pkt);
			}
		}
		break;
	}
	case PAYLOAD_TYPE_RAW_CUSTOM:
		if (pkt->isRouteDirect() && !_tables->hasSeen(pkt)) {
			onRawDataRecv(pkt);
		}
		break;
	case PAYLOAD_TYPE_MULTIPART:
		if (pkt->payload_len > 2) {
			/* uint8_t remaining = pkt->payload[0] >> 4; */ /* Reserved for future multipart support */
			uint8_t type = pkt->payload[0] & 0x0F;

			if (type == PAYLOAD_TYPE_ACK && pkt->payload_len >= 5) {
				Packet tmp;
				tmp.header = pkt->header;
				tmp.path_len = Packet::copyPath(tmp.path, pkt->path, pkt->path_len);
				tmp.payload_len = pkt->payload_len - 1;
				memcpy(tmp.payload, &pkt->payload[1], tmp.payload_len);

				if (!_tables->hasSeen(&tmp)) {
					uint32_t ack_crc;
					memcpy(&ack_crc, tmp.payload, 4);
					onAckRecv(&tmp, ack_crc);
				}
			}
		}
		break;
	default:
		break;
	}
	return action;
}

Packet *Mesh::createAdvert(const LocalIdentity &id, const uint8_t *app_data, size_t app_data_len)
{
	if (app_data_len > MAX_ADVERT_DATA_SIZE) return nullptr;

	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;

	packet->header = (PAYLOAD_TYPE_ADVERT << PH_TYPE_SHIFT);
	int len = 0;
	memcpy(&packet->payload[len], id.pub_key, PUB_KEY_SIZE);
	len += PUB_KEY_SIZE;
	uint32_t emitted_timestamp = _rtc->getCurrentTime();
	memcpy(&packet->payload[len], &emitted_timestamp, 4);
	len += 4;
	uint8_t *signature = &packet->payload[len];
	len += SIGNATURE_SIZE;
	if (app_data && app_data_len > 0) {
		memcpy(&packet->payload[len], app_data, app_data_len);
		len += (int)app_data_len;
	}
	packet->payload_len = len;

	uint8_t message[PUB_KEY_SIZE + 4 + MAX_ADVERT_DATA_SIZE];
	int msg_len = 0;
	memcpy(&message[msg_len], id.pub_key, PUB_KEY_SIZE); msg_len += PUB_KEY_SIZE;
	memcpy(&message[msg_len], &emitted_timestamp, 4); msg_len += 4;
	if (app_data && app_data_len > 0) {
		memcpy(&message[msg_len], app_data, app_data_len); msg_len += (int)app_data_len;
	}
	id.sign(signature, message, msg_len);
	return packet;
}

Packet *Mesh::createAck(uint32_t ack_crc)
{
	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;
	packet->header = (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
	memcpy(packet->payload, &ack_crc, 4);
	packet->payload_len = 4;
	return packet;
}

Packet *Mesh::createMultiAck(uint32_t ack_crc, uint8_t remaining)
{
	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;
	packet->header = (PAYLOAD_TYPE_MULTIPART << PH_TYPE_SHIFT);
	packet->payload[0] = (remaining << 4) | PAYLOAD_TYPE_ACK;
	memcpy(&packet->payload[1], &ack_crc, 4);
	packet->payload_len = 5;
	return packet;
}

Packet *Mesh::createControlData(const uint8_t *data, size_t len)
{
	if (len > sizeof(Packet::payload)) return nullptr;
	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;
	packet->header = (PAYLOAD_TYPE_CONTROL << PH_TYPE_SHIFT);
	memcpy(packet->payload, data, len);
	packet->payload_len = (uint16_t)len;
	return packet;
}

void Mesh::sendFlood(Packet *packet, uint32_t delay_millis, uint8_t path_hash_size)
{
	if (packet->getPayloadType() == PAYLOAD_TYPE_TRACE) {
		releasePacket(packet);
		return;
	}
	if (path_hash_size == 0 || path_hash_size > 3) {
		LOG_WRN("sendFlood: invalid path_hash_size");
		releasePacket(packet);
		return;
	}
	packet->header &= ~PH_ROUTE_MASK;
	packet->header |= ROUTE_TYPE_FLOOD;
	packet->setPathHashSizeAndCount(path_hash_size, 0);
	_tables->hasSeen(packet);
#ifdef CONFIG_ZEPHCORE_APC
	{
		uint32_t h = ContentionTracker::computePacketHash32(packet);
		_power_ctrl.trackTransmit(h, (uint32_t)_ms->getMillis());
	}
#endif

	uint8_t pri;
	if (packet->getPayloadType() == PAYLOAD_TYPE_PATH) {
		pri = 2;
	} else if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT) {
		pri = 3;
	} else {
		pri = 1;
	}
	sendPacket(packet, pri, delay_millis + getInitialFloodJitter(packet));
}

void Mesh::sendFlood(Packet *packet, uint16_t *transport_codes, uint32_t delay_millis, uint8_t path_hash_size)
{
	if (packet->getPayloadType() == PAYLOAD_TYPE_TRACE) {
		releasePacket(packet);
		return;
	}
	if (path_hash_size == 0 || path_hash_size > 3) {
		LOG_WRN("sendFlood: invalid path_hash_size");
		releasePacket(packet);
		return;
	}
	packet->header &= ~PH_ROUTE_MASK;
	packet->header |= ROUTE_TYPE_TRANSPORT_FLOOD;
	packet->transport_codes[0] = transport_codes[0];
	packet->transport_codes[1] = transport_codes[1];
	packet->setPathHashSizeAndCount(path_hash_size, 0);
	_tables->hasSeen(packet);
#ifdef CONFIG_ZEPHCORE_APC
	{
		uint32_t h = ContentionTracker::computePacketHash32(packet);
		_power_ctrl.trackTransmit(h, (uint32_t)_ms->getMillis());
	}
#endif

	uint8_t pri;
	if (packet->getPayloadType() == PAYLOAD_TYPE_PATH) {
		pri = 2;
	} else if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT) {
		pri = 3;
	} else {
		pri = 1;
	}
	sendPacket(packet, pri, delay_millis + getInitialFloodJitter(packet));
}

void Mesh::sendDirect(Packet *packet, const uint8_t *path, uint8_t path_len, uint32_t delay_millis)
{
	packet->header &= ~PH_ROUTE_MASK;
	packet->header |= ROUTE_TYPE_DIRECT;

	uint8_t pri;
	if (packet->getPayloadType() == PAYLOAD_TYPE_TRACE) {
		/* For TRACE packets, path is appended to end of PAYLOAD (used for SNRs) */
		memcpy(&packet->payload[packet->payload_len], path, path_len);
		packet->payload_len += path_len;
		packet->path_len = 0;
		pri = 5;
	} else {
		packet->path_len = Packet::copyPath(packet->path, path, path_len);
		if (packet->getPayloadType() == PAYLOAD_TYPE_PATH) {
			pri = 1;
		} else {
			pri = 0;
		}
	}

	_tables->hasSeen(packet);
	sendPacket(packet, pri, delay_millis);
}

void Mesh::sendZeroHop(Packet *packet, uint32_t delay_millis)
{
	packet->header &= ~PH_ROUTE_MASK;
	packet->header |= ROUTE_TYPE_DIRECT;
	packet->path_len = 0;
	_tables->hasSeen(packet);
	sendPacket(packet, 0, delay_millis);
}

void Mesh::sendZeroHop(Packet *packet, uint16_t *transport_codes, uint32_t delay_millis)
{
	packet->header &= ~PH_ROUTE_MASK;
	packet->header |= ROUTE_TYPE_TRANSPORT_DIRECT;
	packet->transport_codes[0] = transport_codes[0];
	packet->transport_codes[1] = transport_codes[1];
	packet->path_len = 0;
	_tables->hasSeen(packet);
	sendPacket(packet, 0, delay_millis);
}

#define MAX_COMBINED_PATH  (MAX_PACKET_PAYLOAD - 2 - CIPHER_BLOCK_SIZE)

Packet *Mesh::createPathReturn(const Identity &dest, const uint8_t *secret, const uint8_t *path, uint8_t path_len,
	uint8_t extra_type, const uint8_t *extra, size_t extra_len)
{
	uint8_t dest_hash[PATH_HASH_SIZE];
	dest.copyHashTo(dest_hash);
	return createPathReturn(dest_hash, secret, path, path_len, extra_type, extra, extra_len);
}

Packet *Mesh::createPathReturn(const uint8_t *dest_hash, const uint8_t *secret, const uint8_t *path, uint8_t path_len,
	uint8_t extra_type, const uint8_t *extra, size_t extra_len)
{
	uint8_t path_hash_size = (path_len >> 6) + 1;
	uint8_t path_hash_count = path_len & 63;

	if (path_hash_count*path_hash_size + extra_len + 5 > MAX_COMBINED_PATH) return nullptr;

	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;

	packet->header = (PAYLOAD_TYPE_PATH << PH_TYPE_SHIFT);

	int len = 0;
	memcpy(&packet->payload[len], dest_hash, PATH_HASH_SIZE); len += PATH_HASH_SIZE;
	len += self_id.copyHashTo(&packet->payload[len]);

	{
		int data_len = 0;
		uint8_t data[MAX_PACKET_PAYLOAD];

		data[data_len++] = path_len;
		memcpy(&data[data_len], path, path_hash_count*path_hash_size); data_len += path_hash_count*path_hash_size;
		if (extra_len > 0) {
			data[data_len++] = extra_type;
			memcpy(&data[data_len], extra, extra_len); data_len += extra_len;
		} else {
			data[data_len++] = 0xFF;  // dummy payload type
			_rng->random(&data[data_len], 4); data_len += 4;
		}

		len += Utils::encryptThenMAC(secret, &packet->payload[len], data, data_len);
	}

	packet->payload_len = len;
	return packet;
}

Packet *Mesh::createDatagram(uint8_t type, const Identity &dest, const uint8_t *secret, const uint8_t *data, size_t data_len)
{
	if (type == PAYLOAD_TYPE_TXT_MSG || type == PAYLOAD_TYPE_REQ || type == PAYLOAD_TYPE_RESPONSE) {
		if (data_len + CIPHER_MAC_SIZE + CIPHER_BLOCK_SIZE - 1 > MAX_PACKET_PAYLOAD) {
			LOG_WRN("createDatagram: data too large");
			return nullptr;
		}
	} else {
		LOG_WRN("createDatagram: unsupported type %d", type);
		return nullptr;
	}

	Packet *packet = obtainNewPacket();
	if (packet == nullptr) {
		LOG_ERR("createDatagram: packet alloc failed");
		return nullptr;
	}

	packet->header = (type << PH_TYPE_SHIFT);

	int len = 0;
	len += dest.copyHashTo(&packet->payload[len]);
	len += self_id.copyHashTo(&packet->payload[len]);
	len += Utils::encryptThenMAC(secret, &packet->payload[len], data, data_len);

	packet->payload_len = len;
	return packet;
}

Packet *Mesh::createAnonDatagram(uint8_t type, const LocalIdentity &sender, const Identity &dest,
	const uint8_t *secret, const uint8_t *data, size_t data_len)
{
	if (type == PAYLOAD_TYPE_ANON_REQ) {
		if (data_len + 1 + PUB_KEY_SIZE + CIPHER_BLOCK_SIZE - 1 > MAX_PACKET_PAYLOAD) return nullptr;
	} else {
		return nullptr;
	}

	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;

	packet->header = (type << PH_TYPE_SHIFT);

	int len = 0;
	if (type == PAYLOAD_TYPE_ANON_REQ) {
		len += dest.copyHashTo(&packet->payload[len]);
		memcpy(&packet->payload[len], sender.pub_key, PUB_KEY_SIZE); len += PUB_KEY_SIZE;
	}
	len += Utils::encryptThenMAC(secret, &packet->payload[len], data, data_len);

	packet->payload_len = len;
	return packet;
}

Packet *Mesh::createGroupDatagram(uint8_t type, const GroupChannel &channel, const uint8_t *data, size_t data_len)
{
	if (!(type == PAYLOAD_TYPE_GRP_TXT || type == PAYLOAD_TYPE_GRP_DATA)) return nullptr;
	if (data_len + 1 + CIPHER_BLOCK_SIZE - 1 > MAX_PACKET_PAYLOAD) return nullptr;

	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;

	packet->header = (type << PH_TYPE_SHIFT);

	int len = 0;
	memcpy(&packet->payload[len], channel.hash, PATH_HASH_SIZE); len += PATH_HASH_SIZE;
	len += Utils::encryptThenMAC(channel.secret, &packet->payload[len], data, data_len);

	packet->payload_len = len;
	return packet;
}

Packet *Mesh::createRawData(const uint8_t *data, size_t len)
{
	if (len > sizeof(Packet::payload)) return nullptr;

	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;

	packet->header = (PAYLOAD_TYPE_RAW_CUSTOM << PH_TYPE_SHIFT);
	memcpy(packet->payload, data, len);
	packet->payload_len = (uint16_t)len;

	return packet;
}

Packet *Mesh::createTrace(uint32_t tag, uint32_t auth_code, uint8_t flags)
{
	Packet *packet = obtainNewPacket();
	if (packet == nullptr) return nullptr;

	packet->header = (PAYLOAD_TYPE_TRACE << PH_TYPE_SHIFT);
	memcpy(packet->payload, &tag, 4);
	memcpy(&packet->payload[4], &auth_code, 4);
	packet->payload[8] = flags;
	packet->payload_len = 9;

	return packet;
}

} /* namespace mesh */
