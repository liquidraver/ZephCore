/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore BaseChatMesh implementation
 */

#include "BaseChatMesh.h"
#include <mesh/Utils.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_basechat, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#ifndef SERVER_RESPONSE_DELAY
#define SERVER_RESPONSE_DELAY   300
#endif

#ifndef TXT_ACK_DELAY
#define TXT_ACK_DELAY     200
#endif

void BaseChatMesh::sendFloodScoped(const ContactInfo &recipient, mesh::Packet *pkt, uint32_t delay_millis)
{
	sendFlood(pkt, delay_millis);
}

void BaseChatMesh::sendFloodScoped(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t delay_millis)
{
	sendFlood(pkt, delay_millis);
}

mesh::Packet *BaseChatMesh::createSelfAdvert(const char *name)
{
	uint8_t app_data[MAX_ADVERT_DATA_SIZE];
	uint8_t app_data_len;
	{
		AdvertDataBuilder builder(ADV_TYPE_CHAT, name);
		app_data_len = builder.encodeTo(app_data);
	}
	return createAdvert(self_id, app_data, app_data_len);
}

mesh::Packet *BaseChatMesh::createSelfAdvert(const char *name, double lat, double lon)
{
	uint8_t app_data[MAX_ADVERT_DATA_SIZE];
	uint8_t app_data_len;
	{
		AdvertDataBuilder builder(ADV_TYPE_CHAT, name, lat, lon);
		app_data_len = builder.encodeTo(app_data);
	}
	return createAdvert(self_id, app_data, app_data_len);
}

void BaseChatMesh::sendAckTo(const ContactInfo &dest, uint32_t ack_hash)
{
	if (dest.out_path_len == OUT_PATH_UNKNOWN) {
		mesh::Packet *ack = createAck(ack_hash);
		if (ack) sendFloodScoped(dest, ack, TXT_ACK_DELAY);
	} else {
		uint32_t d = TXT_ACK_DELAY;
		if (getExtraAckTransmitCount() > 0) {
			mesh::Packet *a1 = createMultiAck(ack_hash, 1);
			if (a1) sendDirect(a1, dest.out_path, dest.out_path_len, d);
			d += 300;
		}
		mesh::Packet *a2 = createAck(ack_hash);
		if (a2) sendDirect(a2, dest.out_path, dest.out_path_len, d);
	}
}

void BaseChatMesh::bootstrapRTCfromContacts()
{
	uint32_t latest = 0;
	for (int i = 0; i < num_contacts; i++) {
		if (contacts[i].lastmod > latest) {
			latest = contacts[i].lastmod;
		}
	}
	if (latest != 0) {
		getRTCClock()->setCurrentTime(latest + 1);
	}
}

ContactInfo *BaseChatMesh::allocateContactSlot()
{
	if (num_contacts < MAX_CONTACTS) {
		return &contacts[num_contacts++];
	} else if (shouldOverwriteWhenFull()) {
		int oldest_idx = -1;
		uint32_t oldest_lastmod = 0xFFFFFFFF;
		for (int i = 0; i < num_contacts; i++) {
			bool is_favourite = (contacts[i].flags & 0x01) != 0;
			if (!is_favourite && contacts[i].lastmod < oldest_lastmod) {
				oldest_lastmod = contacts[i].lastmod;
				oldest_idx = i;
			}
		}
		if (oldest_idx >= 0) {
			onContactOverwrite(contacts[oldest_idx].id.pub_key);
			return &contacts[oldest_idx];
		}
	}
	return nullptr;
}

void BaseChatMesh::populateContactFromAdvert(ContactInfo &ci, const mesh::Identity &id,
	const AdvertDataParser &parser, uint32_t timestamp)
{
	ci = ContactInfo{};
	ci.id = id;
	ci.out_path_len = OUT_PATH_UNKNOWN;
	StrHelper::strncpy(ci.name, parser.getName(), sizeof(ci.name));
	ci.type = parser.getType();
	if (parser.hasLatLon()) {
		ci.gps_lat = parser.getIntLat();
		ci.gps_lon = parser.getIntLon();
	}
	ci.last_advert_timestamp = timestamp;
	ci.lastmod = getRTCClock()->getCurrentTime();
}

void BaseChatMesh::onAdvertRecv(mesh::Packet *packet, const mesh::Identity &id, uint32_t timestamp,
	const uint8_t *app_data, size_t app_data_len)
{
	LOG_INF("onAdvertRecv: timestamp=%u app_data_len=%u", timestamp, (unsigned)app_data_len);

	AdvertDataParser parser(app_data, app_data_len);
	if (!(parser.isValid() && parser.hasName())) {
		LOG_WRN("onAdvertRecv: invalid parser or no name (valid=%d, hasName=%d)",
			parser.isValid(), parser.hasName());
		return;
	}

	LOG_INF("onAdvertRecv: valid advert from '%s' type=%d", parser.getName(), parser.getType());

	ContactInfo *from = nullptr;
	for (int i = 0; i < num_contacts; i++) {
		if (id.matches(contacts[i].id)) {
			from = &contacts[i];
			if (timestamp <= from->last_advert_timestamp) {
				return;  // replay attack protection
			}
			break;
		}
	}

	// Save raw advert packet for share function
	int plen;
	{
		uint8_t save = packet->header;
		packet->header &= ~PH_ROUTE_MASK;
		packet->header |= ROUTE_TYPE_FLOOD;
		plen = packet->writeTo(temp_buf);
		packet->header = save;
	}

	bool is_new = false;
	if (from == nullptr) {
		LOG_INF("onAdvertRecv: new contact, checking auto-add for type %d", parser.getType());
		if (!shouldAutoAddContactType(parser.getType())) {
			LOG_INF("onAdvertRecv: auto-add disabled for type %d, reporting only", parser.getType());
			ContactInfo ci;
			populateContactFromAdvert(ci, id, parser, timestamp);
			onDiscoveredContact(ci, true, packet->path_len, packet->path);
			return;
		}

		// check hop limit for new contacts (0 = no limit, 1 = direct (0 hops), N = up to N-1 hops)
		uint8_t max_hops = getAutoAddMaxHops();
		if (max_hops > 0 && packet->getPathHashCount() >= max_hops) {
			ContactInfo ci;
			populateContactFromAdvert(ci, id, parser, timestamp);
			onDiscoveredContact(ci, true, packet->path_len, packet->path);
			return;
		}

		from = allocateContactSlot();
		if (from == nullptr) {
			LOG_WRN("onAdvertRecv: contact table full, cannot allocate slot");
			ContactInfo ci;
			populateContactFromAdvert(ci, id, parser, timestamp);
			onDiscoveredContact(ci, true, packet->path_len, packet->path);
			onContactsFull();
			return;
		}

		LOG_INF("onAdvertRecv: allocated slot, num_contacts=%d", num_contacts);
		populateContactFromAdvert(*from, id, parser, timestamp);
		from->sync_since = 0;
		from->shared_secret_valid = false;
	} else {
		LOG_INF("onAdvertRecv: existing contact, updating");
	}

	// Update contact
	putBlobByKey(id.pub_key, PUB_KEY_SIZE, temp_buf, plen);
	StrHelper::strncpy(from->name, parser.getName(), sizeof(from->name));
	from->type = parser.getType();
	if (parser.hasLatLon()) {
		from->gps_lat = parser.getIntLat();
		from->gps_lon = parser.getIntLon();
	}
	from->last_advert_timestamp = timestamp;
	from->lastmod = getRTCClock()->getCurrentTime();

	LOG_INF("onAdvertRecv: calling onDiscoveredContact is_new=%d", is_new);
	onDiscoveredContact(*from, is_new, packet->path_len, packet->path);
}

int BaseChatMesh::searchPeersByHash(const uint8_t *hash)
{
	int n = 0;
	for (int i = 0; i < num_contacts && n < MAX_SEARCH_RESULTS; i++) {
		if (contacts[i].id.isHashMatch(hash)) {
			matching_peer_indexes[n++] = i;
		}
	}
	return n;
}

void BaseChatMesh::getPeerSharedSecret(uint8_t *dest_secret, int peer_idx)
{
	int i = matching_peer_indexes[peer_idx];
	if (i >= 0 && i < num_contacts) {
		memcpy(dest_secret, contacts[i].getSharedSecret(self_id), PUB_KEY_SIZE);
	}
}

void BaseChatMesh::onPeerDataRecv(mesh::Packet *packet, uint8_t type, int sender_idx,
	const uint8_t *secret, uint8_t *data, size_t len)
{
	LOG_INF("onPeerDataRecv: type=%d sender_idx=%d len=%u", type, sender_idx, (unsigned)len);
	int i = matching_peer_indexes[sender_idx];
	if (i < 0 || i >= num_contacts) {
		LOG_WRN("onPeerDataRecv: invalid peer index %d (num_contacts=%d)", i, num_contacts);
		return;
	}

	ContactInfo &from = contacts[i];
	LOG_INF("onPeerDataRecv: from '%s'", from.name);

	if (type == PAYLOAD_TYPE_TXT_MSG && len > 5) {
		uint32_t timestamp;
		memcpy(&timestamp, data, 4);
		uint8_t flags = data[4] >> 2;
		LOG_INF("onPeerDataRecv TXT_MSG: timestamp=%u flags=%d (data[4]=0x%02x) text='%s'",
			timestamp, flags, data[4], (const char*)&data[5]);

		data[len] = 0;  // null terminate

		if (flags == TXT_TYPE_PLAIN) {
			LOG_INF("onPeerDataRecv: flags match TXT_TYPE_PLAIN, calling onMessageRecv");
			from.lastmod = getRTCClock()->getCurrentTime();
			onMessageRecv(from, packet, timestamp, (const char *)&data[5]);

			uint32_t ack_hash;
			mesh::Utils::sha256((uint8_t *)&ack_hash, 4, data, 5 + strlen((char *)&data[5]),
				from.id.pub_key, PUB_KEY_SIZE);

			if (packet->isRouteFlood()) {
				mesh::Packet *path = createPathReturn(from.id, secret, packet->path, packet->path_len,
					PAYLOAD_TYPE_ACK, (uint8_t *)&ack_hash, 4);
				if (path) sendFloodScoped(from, path, TXT_ACK_DELAY);
			} else {
				sendAckTo(from, ack_hash);
			}
		} else if (flags == TXT_TYPE_CLI_DATA) {
			onCommandDataRecv(from, packet, timestamp, (const char *)&data[5]);

			if (packet->isRouteFlood()) {
				mesh::Packet *path = createPathReturn(from.id, secret, packet->path, packet->path_len, 0, nullptr, 0);
				if (path) sendFloodScoped(from, path);
			}
		} else if (flags == TXT_TYPE_SIGNED_PLAIN) {
			if (timestamp > from.sync_since) {
				from.sync_since = timestamp;
			}
			from.lastmod = getRTCClock()->getCurrentTime();
			onSignedMessageRecv(from, packet, timestamp, &data[5], (const char *)&data[9]);

			uint32_t ack_hash;
			mesh::Utils::sha256((uint8_t *)&ack_hash, 4, data, 9 + strlen((char *)&data[9]),
				self_id.pub_key, PUB_KEY_SIZE);

			if (packet->isRouteFlood()) {
				mesh::Packet *path = createPathReturn(from.id, secret, packet->path, packet->path_len,
					PAYLOAD_TYPE_ACK, (uint8_t *)&ack_hash, 4);
				if (path) sendFloodScoped(from, path, TXT_ACK_DELAY);
			} else {
				sendAckTo(from, ack_hash);
			}
		}
	} else if (type == PAYLOAD_TYPE_REQ && len > 4) {
		uint32_t sender_timestamp;
		memcpy(&sender_timestamp, data, 4);
		uint8_t reply_len = onContactRequest(from, sender_timestamp, &data[4], len - 4, temp_buf);
		if (reply_len > 0) {
			if (packet->isRouteFlood()) {
				mesh::Packet *path = createPathReturn(from.id, secret, packet->path, packet->path_len,
					PAYLOAD_TYPE_RESPONSE, temp_buf, reply_len);
				if (path) sendFloodScoped(from, path, SERVER_RESPONSE_DELAY);
			} else {
				mesh::Packet *reply = createDatagram(PAYLOAD_TYPE_RESPONSE, from.id, secret, temp_buf, reply_len);
				if (reply) {
					if (from.out_path_len != OUT_PATH_UNKNOWN) {
						sendDirect(reply, from.out_path, from.out_path_len, SERVER_RESPONSE_DELAY);
					} else {
						sendFloodScoped(from, reply, SERVER_RESPONSE_DELAY);
					}
				}
			}
		}
	} else if (type == PAYLOAD_TYPE_RESPONSE && len > 0) {
		LOG_INF("onPeerDataRecv: RESPONSE received, len=%d, calling onContactResponse", len);
		onContactResponse(from, data, len);
		if (packet->isRouteFlood() && from.out_path_len != OUT_PATH_UNKNOWN) {
			handleReturnPathRetry(from, packet->path, packet->path_len);
		}
	}
}

bool BaseChatMesh::onPeerPathRecv(mesh::Packet *packet, int sender_idx, const uint8_t *secret,
	uint8_t *path, uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len)
{
	int i = matching_peer_indexes[sender_idx];
	if (i < 0 || i >= num_contacts) {
		return false;
	}

	ContactInfo &from = contacts[i];
	return onContactPathRecv(from, packet->path, packet->path_len, path, path_len, extra_type, extra, extra_len);
}

bool BaseChatMesh::onContactPathRecv(ContactInfo &from, uint8_t *in_path, uint8_t in_path_len,
	uint8_t *out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len)
{
	from.out_path_len = mesh::Packet::copyPath(from.out_path, out_path, out_path_len);
	from.lastmod = getRTCClock()->getCurrentTime();

	onContactPathUpdated(from);

	if (extra_type == PAYLOAD_TYPE_ACK && extra_len >= 4) {
		if (processAck(extra) != nullptr) {
			txt_send_timeout = 0;
		}
	} else if (extra_type == PAYLOAD_TYPE_RESPONSE && extra_len > 0) {
		onContactResponse(from, extra, extra_len);
	}
	return true;
}

void BaseChatMesh::onAckRecv(mesh::Packet *packet, uint32_t ack_crc)
{
	LOG_INF("onAckRecv: ack_crc=0x%08x route=%s", ack_crc,
		packet->isRouteFlood() ? "flood" : "direct");
	ContactInfo *from;
	if ((from = processAck((uint8_t *)&ack_crc)) != nullptr) {
		LOG_INF("onAckRecv: ACK processed successfully for '%s'", from->name);
		txt_send_timeout = 0;
		packet->markDoNotRetransmit();

		if (packet->isRouteFlood() && from->out_path_len != OUT_PATH_UNKNOWN) {
			handleReturnPathRetry(*from, packet->path, packet->path_len);
		}
	} else {
		LOG_INF("onAckRecv: ACK not matched (no pending or wrong crc)");
	}
}

void BaseChatMesh::handleReturnPathRetry(const ContactInfo &contact, const uint8_t *path, uint8_t path_len)
{
	mesh::Packet *rpath = createPathReturn(contact.id, contact.getSharedSecret(self_id), path, path_len, 0, nullptr, 0);
	if (rpath) sendDirect(rpath, contact.out_path, contact.out_path_len, 3000);
}

int BaseChatMesh::searchChannelsByHash(const uint8_t *hash, mesh::GroupChannel dest[], int max_matches)
{
	int n = 0;
	for (int i = 0; i < MAX_GROUP_CHANNELS && n < max_matches; i++) {
		if (channels[i].channel.hash[0] == hash[0]) {
			dest[n++] = channels[i].channel;
		}
	}
	return n;
}

void BaseChatMesh::onGroupDataRecv(mesh::Packet *packet, uint8_t type, const mesh::GroupChannel &channel,
	uint8_t *data, size_t len)
{
	uint8_t txt_type = data[4];
	if (type == PAYLOAD_TYPE_GRP_TXT && len > 5 && (txt_type >> 2) == 0) {
		uint32_t timestamp;
		memcpy(&timestamp, data, 4);
		data[len] = 0;
		onChannelMessageRecv(channel, packet, timestamp, (const char *)&data[5]);
	}
}

mesh::Packet *BaseChatMesh::composeMsgPacket(const ContactInfo &recipient, uint32_t timestamp,
	uint8_t attempt, const char *text, uint32_t &expected_ack)
{
	int text_len = strlen(text);
	if (text_len > MAX_TEXT_LEN) return nullptr;
	if (attempt > 3 && text_len > MAX_TEXT_LEN - 2) return nullptr;

	uint8_t temp[5 + MAX_TEXT_LEN + 1];
	memcpy(temp, &timestamp, 4);
	temp[4] = (attempt & 3);
	memcpy(&temp[5], text, text_len + 1);

	mesh::Utils::sha256((uint8_t *)&expected_ack, 4, temp, 5 + text_len, self_id.pub_key, PUB_KEY_SIZE);

	int len = 5 + text_len;
	if (attempt > 3) {
		temp[len++] = 0;
		temp[len++] = attempt;
	}

	return createDatagram(PAYLOAD_TYPE_TXT_MSG, recipient.id, recipient.getSharedSecret(self_id), temp, len);
}

int BaseChatMesh::sendMessage(const ContactInfo &recipient, uint32_t timestamp, uint8_t attempt,
	const char *text, uint32_t &expected_ack, uint32_t &est_timeout)
{
	LOG_INF("sendMessage: to '%s' text='%s' attempt=%d path_len=%d",
		recipient.name, text, attempt, recipient.out_path_len);

	mesh::Packet *pkt = composeMsgPacket(recipient, timestamp, attempt, text, expected_ack);
	if (pkt == nullptr) {
		LOG_WRN("sendMessage: composeMsgPacket failed");
		return MSG_SEND_FAILED;
	}

	LOG_INF("sendMessage: packet created, expected_ack=0x%08x raw_len=%d",
		expected_ack, pkt->getRawLength());

	uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());

	int rc;
	if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
		LOG_INF("sendMessage: sending flood");
		sendFloodScoped(recipient, pkt);
		txt_send_timeout = futureMillis(est_timeout = calcFloodTimeoutMillisFor(t));
		rc = MSG_SEND_SENT_FLOOD;
	} else {
		LOG_INF("sendMessage: sending direct path_len=%d", recipient.out_path_len);
		sendDirect(pkt, recipient.out_path, recipient.out_path_len);
		txt_send_timeout = futureMillis(est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len));
		rc = MSG_SEND_SENT_DIRECT;
	}
	LOG_INF("sendMessage: result=%d est_timeout=%u", rc, est_timeout);
	return rc;
}

int BaseChatMesh::sendCommandData(const ContactInfo &recipient, uint32_t timestamp, uint8_t attempt,
	const char *text, uint32_t &est_timeout)
{
	int text_len = strlen(text);
	if (text_len > MAX_TEXT_LEN) return MSG_SEND_FAILED;

	uint8_t temp[5 + MAX_TEXT_LEN + 1];
	memcpy(temp, &timestamp, 4);
	temp[4] = (attempt & 3) | (TXT_TYPE_CLI_DATA << 2);
	memcpy(&temp[5], text, text_len + 1);

	mesh::Packet *pkt = createDatagram(PAYLOAD_TYPE_TXT_MSG, recipient.id,
		recipient.getSharedSecret(self_id), temp, 5 + text_len);
	if (pkt == nullptr) return MSG_SEND_FAILED;

	uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
	int rc;
	if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
		sendFloodScoped(recipient, pkt);
		txt_send_timeout = futureMillis(est_timeout = calcFloodTimeoutMillisFor(t));
		rc = MSG_SEND_SENT_FLOOD;
	} else {
		sendDirect(pkt, recipient.out_path, recipient.out_path_len);
		txt_send_timeout = futureMillis(est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len));
		rc = MSG_SEND_SENT_DIRECT;
	}
	return rc;
}

bool BaseChatMesh::sendGroupMessage(uint32_t timestamp, mesh::GroupChannel &channel,
	const char *sender_name, const char *text, int text_len)
{
	uint8_t temp[5 + MAX_TEXT_LEN + 32];
	memcpy(temp, &timestamp, 4);
	temp[4] = 0;  // TXT_TYPE_PLAIN

	snprintf((char *)&temp[5], sizeof(temp) - 5, "%s: ", sender_name);
	char *ep = strchr((char *)&temp[5], 0);
	int prefix_len = ep - (char *)&temp[5];

	if (text_len + prefix_len > MAX_TEXT_LEN) text_len = MAX_TEXT_LEN - prefix_len;
	memcpy(ep, text, text_len);
	ep[text_len] = 0;

	mesh::Packet *pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, temp, 5 + prefix_len + text_len);
	if (pkt) {
		sendFloodScoped(channel, pkt);
		return true;
	}
	return false;
}

bool BaseChatMesh::shareContactZeroHop(const ContactInfo &contact)
{
	int plen = getBlobByKey(contact.id.pub_key, PUB_KEY_SIZE, temp_buf);
	if (plen == 0) return false;

	mesh::Packet *packet = obtainNewPacket();
	if (packet == nullptr) return false;

	packet->readFrom(temp_buf, plen);
	uint16_t codes[2];
	codes[0] = codes[1] = 0;
	sendZeroHop(packet, codes);
	return true;
}

uint8_t BaseChatMesh::exportContact(const ContactInfo &contact, uint8_t dest_buf[])
{
	return getBlobByKey(contact.id.pub_key, PUB_KEY_SIZE, dest_buf);
}

bool BaseChatMesh::importContact(const uint8_t src_buf[], uint8_t len)
{
	mesh::Packet *pkt = obtainNewPacket();
	if (pkt) {
		if (pkt->readFrom(src_buf, len) && pkt->getPayloadType() == PAYLOAD_TYPE_ADVERT) {
			pkt->header |= ROUTE_TYPE_FLOOD;
			getTables()->clear(pkt);
			_pendingLoopback = pkt;
			return true;
		} else {
			releasePacket(pkt);
		}
	}
	return false;
}

int BaseChatMesh::sendLogin(const ContactInfo &recipient, const char *password, uint32_t &est_timeout)
{
	mesh::Packet *pkt;
	{
		int tlen;
		uint8_t temp[24];
		uint32_t now = getRTCClock()->getCurrentTimeUnique();
		memcpy(temp, &now, 4);
		if (recipient.type == ADV_TYPE_ROOM) {
			memcpy(&temp[4], &recipient.sync_since, 4);
			int len = strlen(password); if (len > 15) len = 15;
			memcpy(&temp[8], password, len);
			tlen = 8 + len;
		} else {
			int len = strlen(password); if (len > 15) len = 15;
			memcpy(&temp[4], password, len);
			tlen = 4 + len;
		}

		pkt = createAnonDatagram(PAYLOAD_TYPE_ANON_REQ, self_id, recipient.id,
			recipient.getSharedSecret(self_id), temp, tlen);
	}
	if (pkt) {
		uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
		if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
			sendFloodScoped(recipient, pkt);
			est_timeout = calcFloodTimeoutMillisFor(t);
			return MSG_SEND_SENT_FLOOD;
		} else {
			sendDirect(pkt, recipient.out_path, recipient.out_path_len);
			est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
			return MSG_SEND_SENT_DIRECT;
		}
	}
	return MSG_SEND_FAILED;
}

int BaseChatMesh::sendAnonReq(const ContactInfo &recipient, const uint8_t *data, uint8_t len,
	uint32_t &tag, uint32_t &est_timeout)
{
	mesh::Packet *pkt;
	{
		uint8_t temp[MAX_PACKET_PAYLOAD];
		tag = getRTCClock()->getCurrentTimeUnique();
		memcpy(temp, &tag, 4);
		memcpy(&temp[4], data, len);

		pkt = createAnonDatagram(PAYLOAD_TYPE_ANON_REQ, self_id, recipient.id,
			recipient.getSharedSecret(self_id), temp, 4 + len);
	}
	if (pkt) {
		uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
		if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
			sendFloodScoped(recipient, pkt);
			est_timeout = calcFloodTimeoutMillisFor(t);
			return MSG_SEND_SENT_FLOOD;
		} else {
			sendDirect(pkt, recipient.out_path, recipient.out_path_len);
			est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
			return MSG_SEND_SENT_DIRECT;
		}
	}
	return MSG_SEND_FAILED;
}

int BaseChatMesh::sendRequest(const ContactInfo &recipient, const uint8_t *req_data, uint8_t data_len,
	uint32_t &tag, uint32_t &est_timeout)
{
	if (data_len > MAX_PACKET_PAYLOAD - 16) return MSG_SEND_FAILED;

	mesh::Packet *pkt;
	{
		uint8_t temp[MAX_PACKET_PAYLOAD];
		tag = getRTCClock()->getCurrentTimeUnique();
		memcpy(temp, &tag, 4);
		memcpy(&temp[4], req_data, data_len);

		pkt = createDatagram(PAYLOAD_TYPE_REQ, recipient.id, recipient.getSharedSecret(self_id), temp, 4 + data_len);
	}
	if (pkt) {
		uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
		if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
			sendFloodScoped(recipient, pkt);
			est_timeout = calcFloodTimeoutMillisFor(t);
			return MSG_SEND_SENT_FLOOD;
		} else {
			sendDirect(pkt, recipient.out_path, recipient.out_path_len);
			est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
			return MSG_SEND_SENT_DIRECT;
		}
	}
	return MSG_SEND_FAILED;
}

int BaseChatMesh::sendRequest(const ContactInfo &recipient, uint8_t req_type, uint32_t &tag, uint32_t &est_timeout)
{
	mesh::Packet *pkt;
	{
		uint8_t temp[13];
		tag = getRTCClock()->getCurrentTimeUnique();
		memcpy(temp, &tag, 4);
		temp[4] = req_type;
		memset(&temp[5], 0, 4);
		getRNG()->random(&temp[9], 4);

		pkt = createDatagram(PAYLOAD_TYPE_REQ, recipient.id, recipient.getSharedSecret(self_id), temp, sizeof(temp));
	}
	if (pkt) {
		uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
		if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
			sendFloodScoped(recipient, pkt);
			est_timeout = calcFloodTimeoutMillisFor(t);
			return MSG_SEND_SENT_FLOOD;
		} else {
			sendDirect(pkt, recipient.out_path, recipient.out_path_len);
			est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
			return MSG_SEND_SENT_DIRECT;
		}
	}
	return MSG_SEND_FAILED;
}

void BaseChatMesh::resetPathTo(ContactInfo &recipient)
{
	recipient.out_path_len = OUT_PATH_UNKNOWN;
}

static ContactInfo *table_sort;  // pass via global for qsort

static int cmp_adv_timestamp(const void *a, const void *b)
{
	int a_idx = *((int *)a);
	int b_idx = *((int *)b);
	if (table_sort[b_idx].last_advert_timestamp > table_sort[a_idx].last_advert_timestamp) return 1;
	if (table_sort[b_idx].last_advert_timestamp < table_sort[a_idx].last_advert_timestamp) return -1;
	return 0;
}

void BaseChatMesh::scanRecentContacts(int last_n, ContactVisitor *visitor)
{
	for (int i = 0; i < num_contacts; i++) {
		sort_array[i] = i;
	}
	table_sort = contacts;
	qsort(sort_array, num_contacts, sizeof(sort_array[0]), cmp_adv_timestamp);

	if (last_n == 0) {
		last_n = num_contacts;
	} else {
		if (last_n > num_contacts) last_n = num_contacts;
	}
	for (int i = 0; i < last_n; i++) {
		visitor->onContactVisit(contacts[sort_array[i]]);
	}
}

ContactInfo *BaseChatMesh::searchContactsByPrefix(const char *name_prefix)
{
	int len = strlen(name_prefix);
	for (int i = 0; i < num_contacts; i++) {
		ContactInfo *c = &contacts[i];
		if (memcmp(c->name, name_prefix, len) == 0) return c;
	}
	return nullptr;
}

ContactInfo *BaseChatMesh::lookupContactByPubKey(const uint8_t *pub_key, int prefix_len)
{
	for (int i = 0; i < num_contacts; i++) {
		ContactInfo *c = &contacts[i];
		if (memcmp(c->id.pub_key, pub_key, prefix_len) == 0) return c;
	}
	return nullptr;
}

bool BaseChatMesh::addContact(const ContactInfo &contact)
{
	ContactInfo *dest = allocateContactSlot();
	if (dest) {
		*dest = contact;
		dest->shared_secret_valid = false;
		return true;
	}
	return false;
}

bool BaseChatMesh::removeContact(ContactInfo &contact)
{
	int idx = 0;
	while (idx < num_contacts && !contacts[idx].id.matches(contact.id)) {
		idx++;
	}
	if (idx >= num_contacts) return false;

	num_contacts--;
	while (idx < num_contacts) {
		contacts[idx] = contacts[idx + 1];
		idx++;
	}
	return true;
}

ChannelDetails *BaseChatMesh::addChannel(const char *name, const uint8_t *psk, size_t psk_len)
{
	if (num_channels < MAX_GROUP_CHANNELS) {
		ChannelDetails *dest = &channels[num_channels];

		memset(dest->channel.secret, 0, sizeof(dest->channel.secret));
		if (psk_len == 32 || psk_len == 16) {
			memcpy(dest->channel.secret, psk, psk_len);
			mesh::Utils::sha256(dest->channel.hash, sizeof(dest->channel.hash), dest->channel.secret, psk_len);
			StrHelper::strncpy(dest->name, name, sizeof(dest->name));
			num_channels++;
			return dest;
		}
	}
	return nullptr;
}

bool BaseChatMesh::getChannel(int idx, ChannelDetails &dest)
{
	if (idx >= 0 && idx < MAX_GROUP_CHANNELS) {
		dest = channels[idx];
		return true;
	}
	return false;
}

bool BaseChatMesh::setChannel(int idx, const ChannelDetails &src)
{
	static uint8_t zeroes[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	if (idx >= 0 && idx < MAX_GROUP_CHANNELS) {
		channels[idx] = src;
		if (memcmp(&src.channel.secret[16], zeroes, 16) == 0) {
			mesh::Utils::sha256(channels[idx].channel.hash, sizeof(channels[idx].channel.hash),
				src.channel.secret, 16);
		} else {
			mesh::Utils::sha256(channels[idx].channel.hash, sizeof(channels[idx].channel.hash),
				src.channel.secret, 32);
		}
		return true;
	}
	return false;
}

int BaseChatMesh::findChannelIdx(const mesh::GroupChannel &ch)
{
	for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
		if (memcmp(ch.secret, channels[i].channel.secret, sizeof(ch.secret)) == 0) return i;
	}
	return -1;
}

bool BaseChatMesh::getContactByIdx(uint32_t idx, ContactInfo &contact)
{
	if (idx >= (uint32_t)num_contacts) return false;
	contact = contacts[idx];
	return true;
}

ContactsIterator BaseChatMesh::startContactsIterator()
{
	return ContactsIterator();
}

bool ContactsIterator::hasNext(const BaseChatMesh *mesh, ContactInfo &dest)
{
	if (next_idx >= mesh->getNumContacts()) return false;
	dest = mesh->contacts[next_idx++];
	return true;
}

/* Connection management for room servers */
bool BaseChatMesh::startConnection(const ContactInfo &contact, uint16_t keep_alive_secs)
{
	int use_idx = -1;
	for (int i = 0; i < MAX_CONNECTIONS; i++) {
		if (connections[i].keep_alive_millis == 0) {  // free slot
			use_idx = i;
		} else if (connections[i].server_id.matches(contact.id)) {  // already in table
			use_idx = i;
			break;
		}
	}
	if (use_idx < 0) return false;  // table is full

	connections[use_idx].server_id = contact.id;
	uint32_t interval = connections[use_idx].keep_alive_millis = (uint32_t)keep_alive_secs * 1000;
	connections[use_idx].next_ping = futureMillis(interval);
	connections[use_idx].expected_ack = 0;
	connections[use_idx].last_activity = getRTCClock()->getCurrentTime();
	return true;
}

void BaseChatMesh::stopConnection(const uint8_t *pub_key)
{
	for (int i = 0; i < MAX_CONNECTIONS; i++) {
		if (connections[i].server_id.matches(pub_key)) {
			connections[i].keep_alive_millis = 0;  // mark slot as free
			connections[i].next_ping = 0;
			connections[i].expected_ack = 0;
			connections[i].last_activity = 0;
			break;
		}
	}
}

bool BaseChatMesh::hasConnectionTo(const uint8_t *pub_key)
{
	for (int i = 0; i < MAX_CONNECTIONS; i++) {
		if (connections[i].keep_alive_millis > 0 && connections[i].server_id.matches(pub_key)) return true;
	}
	return false;
}

void BaseChatMesh::markConnectionActive(const ContactInfo &contact)
{
	for (int i = 0; i < MAX_CONNECTIONS; i++) {
		if (connections[i].keep_alive_millis > 0 && connections[i].server_id.matches(contact.id)) {
			connections[i].last_activity = getRTCClock()->getCurrentTime();
			// schedule next keep-alive
			connections[i].next_ping = futureMillis(connections[i].keep_alive_millis);
			return;
		}
	}
}

ContactInfo *BaseChatMesh::checkConnectionsAck(const uint8_t *data)
{
	for (int i = 0; i < MAX_CONNECTIONS; i++) {
		if (connections[i].keep_alive_millis > 0 && memcmp(&connections[i].expected_ack, data, 4) == 0) {
			// got an ack for our keep_alive request
			connections[i].expected_ack = 0;
			connections[i].last_activity = getRTCClock()->getCurrentTime();
			connections[i].next_ping = futureMillis(connections[i].keep_alive_millis);
			return lookupContactByPubKey(connections[i].server_id.pub_key, PUB_KEY_SIZE);
		}
	}
	return nullptr;
}

void BaseChatMesh::checkConnections()
{
	// scan connections[] table, send KEEP_ALIVE requests
	for (int i = 0; i < MAX_CONNECTIONS; i++) {
		if (connections[i].keep_alive_millis == 0) continue;  // unused slot

		uint32_t now = getRTCClock()->getCurrentTime();
		uint32_t expire_secs = (connections[i].keep_alive_millis / 1000) * 5 / 2;  // 2.5x keep_alive
		if (now >= connections[i].last_activity + expire_secs) {
			// connection timed out
			connections[i].keep_alive_millis = 0;
			connections[i].next_ping = 0;
			connections[i].expected_ack = 0;
			connections[i].last_activity = 0;
			continue;
		}

		if (millisHasNowPassed(connections[i].next_ping)) {
			ContactInfo *contact = lookupContactByPubKey(connections[i].server_id.pub_key, PUB_KEY_SIZE);
			if (contact == nullptr) {
				LOG_WRN("checkConnections: keep_alive contact not found");
				continue;
			}
			if (contact->out_path_len == OUT_PATH_UNKNOWN) {
				LOG_WRN("checkConnections: keep_alive contact has no out_path");
				continue;
			}

			// Build keepalive request manually (9 bytes, matching Arduino wire format)
			uint8_t data[9];
			uint32_t ts = getRTCClock()->getCurrentTimeUnique();
			memcpy(data, &ts, 4);
			data[4] = REQ_TYPE_KEEP_ALIVE;
			memcpy(&data[5], &contact->sync_since, 4);

			// calc expected ACK reply (SHA256 of data + self pubkey, first 4 bytes)
			mesh::Utils::sha256((uint8_t *)&connections[i].expected_ack, 4,
				data, 9, self_id.pub_key, PUB_KEY_SIZE);

			auto pkt = createDatagram(PAYLOAD_TYPE_REQ, contact->id,
				contact->getSharedSecret(self_id), data, 9);
			if (pkt) {
				sendDirect(pkt, contact->out_path, contact->out_path_len);
			}

			// schedule next KEEP_ALIVE
			connections[i].next_ping = futureMillis(connections[i].keep_alive_millis);
		}
	}
}

void BaseChatMesh::loop()
{
	Mesh::loop();

	if (txt_send_timeout && millisHasNowPassed(txt_send_timeout)) {
		onSendTimeout();
		txt_send_timeout = 0;
	}

	if (_pendingLoopback) {
		onRecvPacket(_pendingLoopback);
		releasePacket(_pendingLoopback);
		_pendingLoopback = nullptr;
	}

	checkConnections();
}
