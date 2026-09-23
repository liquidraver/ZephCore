/*
 * SPDX-License-Identifier: MIT
 * Arduino-compatible packet log lines (CONFIG_ZEPHCORE_PACKET_LOGGING).
 *
 * Same format as Arduino MeshCore's MESH_PACKET_LOGGING, and like upstream it is
 * emitted from the roles' logRx()/logTx() hooks rather than from the core
 * Dispatcher. printk() bypasses log-level filtering so packet_logging.conf can
 * silence everything else. Compiles to nothing without the Kconfig option.
 */

#pragma once

#include <zephyr/kernel.h>
#include <mesh/Packet.h>
#include <mesh/Utils.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)

static inline bool packet_log_has_addrs(uint8_t ptype)
{
	return ptype == PAYLOAD_TYPE_PATH || ptype == PAYLOAD_TYPE_REQ ||
	       ptype == PAYLOAD_TYPE_RESPONSE || ptype == PAYLOAD_TYPE_TXT_MSG;
}

static inline void packet_log_rx(const char *datetime, const mesh::Packet *pkt,
				 float rssi, float score, uint32_t air_time)
{
	static uint8_t packet_hash[MAX_HASH_SIZE];
	static char hash_hex[MAX_HASH_SIZE * 2 + 1];
	uint8_t ptype = pkt->getPayloadType();

	pkt->calculatePacketHash(packet_hash);
	mesh::Utils::toHex(hash_hex, packet_hash, MAX_HASH_SIZE);
	if (packet_log_has_addrs(ptype)) {
		printk("%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%u hash=%s [%02X -> %02X]\n",
			datetime, pkt->getRawLength(), ptype,
			pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
			(int)pkt->getSNR(), (int)rssi, (int)(score * 1000), air_time,
			hash_hex, (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
	} else {
		printk("%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%u hash=%s\n",
			datetime, pkt->getRawLength(), ptype,
			pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
			(int)pkt->getSNR(), (int)rssi, (int)(score * 1000), air_time,
			hash_hex);
	}
}

/* Called on TX completion, as upstream does (before 2026-09-23 the core printed
 * this line just before startSendRaw()). */
static inline void packet_log_tx(const char *datetime, const mesh::Packet *pkt)
{
	uint8_t ptype = pkt->getPayloadType();

	if (packet_log_has_addrs(ptype)) {
		printk("%s: TX, len=%d (type=%d, route=%s, payload_len=%d) [%02X -> %02X]\n",
			datetime, pkt->getRawLength(), ptype,
			pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
			(uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
	} else {
		printk("%s: TX, len=%d (type=%d, route=%s, payload_len=%d)\n",
			datetime, pkt->getRawLength(), ptype,
			pkt->isRouteDirect() ? "D" : "F", pkt->payload_len);
	}
}

#else

static inline void packet_log_rx(const char *, const mesh::Packet *, float, float, uint32_t) {}
static inline void packet_log_tx(const char *, const mesh::Packet *) {}

#endif
