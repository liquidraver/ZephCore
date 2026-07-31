/*
 * SPDX-License-Identifier: MIT
 * ZephCore Packet - fundamental transmission unit
 */

#pragma once

#include <mesh/MeshCore.h>
#include <stdint.h>
#include <string.h>

namespace mesh {

#define PH_ROUTE_MASK     0x03
#define PH_TYPE_SHIFT     2
#define PH_TYPE_MASK      0x0F
#define PH_VER_SHIFT      6
#define PH_VER_MASK       0x03

#define ROUTE_TYPE_TRANSPORT_FLOOD   0x00
#define ROUTE_TYPE_FLOOD             0x01
#define ROUTE_TYPE_DIRECT            0x02
#define ROUTE_TYPE_TRANSPORT_DIRECT  0x03

#define PAYLOAD_TYPE_REQ         0x00
#define PAYLOAD_TYPE_RESPONSE    0x01
#define PAYLOAD_TYPE_TXT_MSG     0x02
#define PAYLOAD_TYPE_ACK         0x03
#define PAYLOAD_TYPE_ADVERT      0x04
#define PAYLOAD_TYPE_GRP_TXT     0x05
#define PAYLOAD_TYPE_GRP_DATA    0x06
#define PAYLOAD_TYPE_ANON_REQ    0x07
#define PAYLOAD_TYPE_PATH        0x08
#define PAYLOAD_TYPE_TRACE       0x09
#define PAYLOAD_TYPE_MULTIPART   0x0A
#define PAYLOAD_TYPE_CONTROL     0x0B
#define PAYLOAD_TYPE_RAW_CUSTOM  0x0F

#define PAYLOAD_VER_1       0x00
#define PAYLOAD_VER_2       0x01
#define PAYLOAD_VER_3       0x02
#define PAYLOAD_VER_4       0x03

class Packet {
public:
	Packet();

	uint8_t header;
	uint16_t payload_len;
	uint8_t path_len;
	uint16_t transport_codes[2];
	uint8_t path[MAX_PATH_SIZE];
	uint8_t payload[MAX_PACKET_PAYLOAD];
	int8_t _snr;  /* SNR * 4 (quarter-dB fixed point) */

	void calculatePacketHash(uint8_t *dest_hash) const;
	uint8_t getRouteType() const { return header & PH_ROUTE_MASK; }
	bool isRouteFlood() const { return getRouteType() == ROUTE_TYPE_FLOOD || getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD; }
	bool isRouteDirect() const { return getRouteType() == ROUTE_TYPE_DIRECT || getRouteType() == ROUTE_TYPE_TRANSPORT_DIRECT; }
	bool hasTransportCodes() const { return getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD || getRouteType() == ROUTE_TYPE_TRANSPORT_DIRECT; }
	uint8_t getPayloadType() const { return (header >> PH_TYPE_SHIFT) & PH_TYPE_MASK; }
	uint8_t getPayloadVer() const { return (header >> PH_VER_SHIFT) & PH_VER_MASK; }

	/* path_len layout: bits[7:6] = hash_size - 1, bits[5:0] = hop count */
	uint8_t getPathHashSize() const { return (path_len >> 6) + 1; }
	uint8_t getPathHashCount() const { return path_len & 0x3F; }
	uint8_t getPathByteLen() const { return getPathHashCount() * getPathHashSize(); }
	void setPathHashCount(uint8_t n) { path_len &= ~0x3F; path_len |= n; }
	void setPathHashSizeAndCount(uint8_t sz, uint8_t n) { path_len = ((sz - 1) << 6) | (n & 0x3F); }

	/* writePath / copyPath decode the 6-bit hash_count + 2-bit hash_size
	 * encoding in path_len, then copy hash_count*hash_size bytes from src
	 * to dest. src_len bounds the source: if the decoded byte count would
	 * read past src_len, the call returns 0 (no copy).
	 *
	 * Trusted callers writing from internal MAX_PATH_SIZE buffers must
	 * pass MAX_PATH_SIZE. Callers handling untrusted input (BLE phone
	 * frames, LoRa anon-request payloads) must pass the real remaining
	 * length so a malformed path_len cannot trigger an OOB read. */
	static size_t writePath(uint8_t *dest, const uint8_t *src, size_t src_len, uint8_t path_len);
	static uint8_t copyPath(uint8_t *dest, const uint8_t *src, size_t src_len, uint8_t path_len);
	static bool isValidPathLen(uint8_t path_len);

	void markDoNotRetransmit() { header = 0xFF; }
	bool isMarkedDoNotRetransmit() const { return header == 0xFF; }
	float getSNR() const { return ((float)_snr) / 4.0f; }
	int getRawLength() const;
	uint8_t writeTo(uint8_t dest[]) const;
	bool readFrom(const uint8_t src[], uint8_t len);
};

} /* namespace mesh */
