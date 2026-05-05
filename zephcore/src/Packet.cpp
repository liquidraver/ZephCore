/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Packet implementation
 */

#include <mesh/Packet.h>
#include <mesh/Utils.h>
#include <string.h>

namespace mesh {

Packet::Packet()
{
	header = 0;
	path_len = 0;
	payload_len = 0;
}

bool Packet::isValidPathLen(uint8_t path_len)
{
	uint8_t hash_count = path_len & 63;
	uint8_t hash_size = (path_len >> 6) + 1;
	if (hash_size == 4) return false;  // Reserved for future
	return hash_count * hash_size <= MAX_PATH_SIZE;
}

size_t Packet::writePath(uint8_t *dest, const uint8_t *src, uint8_t path_len)
{
	uint8_t hash_count = path_len & 63;
	uint8_t hash_size = (path_len >> 6) + 1;
	size_t len = hash_count * hash_size;
	if (len > MAX_PATH_SIZE) {
		return 0;   // Error
	}
	memcpy(dest, src, len);
	return len;
}

uint8_t Packet::copyPath(uint8_t *dest, const uint8_t *src, uint8_t path_len)
{
	size_t written = writePath(dest, src, path_len);
	return written > 0 ? path_len : 0;
}

int Packet::getRawLength() const
{
	return 2 + getPathByteLen() + payload_len + (hasTransportCodes() ? 4 : 0);
}

void Packet::calculatePacketHash(uint8_t *hash) const
{
	uint8_t t = getPayloadType();
	if (t == PAYLOAD_TYPE_TRACE) {
		uint8_t buf[2 + MAX_PACKET_PAYLOAD];
		buf[0] = t;
		memcpy(buf + 1, &path_len, sizeof(path_len));
		memcpy(buf + 2, payload, payload_len);
		Utils::sha256(hash, MAX_HASH_SIZE, buf, 2 + payload_len);
	} else {
		uint8_t buf[1 + MAX_PACKET_PAYLOAD];
		buf[0] = t;
		memcpy(buf + 1, payload, payload_len);
		Utils::sha256(hash, MAX_HASH_SIZE, buf, 1 + payload_len);
	}
}

uint8_t Packet::writeTo(uint8_t dest[]) const
{
	uint8_t i = 0;
	dest[i++] = header;
	if (hasTransportCodes()) {
		memcpy(&dest[i], &transport_codes[0], 2); i += 2;
		memcpy(&dest[i], &transport_codes[1], 2); i += 2;
	}
	dest[i++] = path_len;
	i += writePath(&dest[i], path, path_len);
	memcpy(&dest[i], payload, payload_len); i += payload_len;
	return i;
}

bool Packet::readFrom(const uint8_t src[], uint8_t len)
{
	uint8_t i = 0;
	if (len < 2) return false;
	header = src[i++];
	if (hasTransportCodes()) {
		if (len < 6) return false;
		memcpy(&transport_codes[0], &src[i], 2); i += 2;
		memcpy(&transport_codes[1], &src[i], 2); i += 2;
	} else {
		transport_codes[0] = transport_codes[1] = 0;
	}
	path_len = src[i++];
	if (!isValidPathLen(path_len)) return false;

	uint8_t bl = getPathByteLen();
	if ((uint16_t)i + bl > len) return false;
	memcpy(path, &src[i], bl); i += bl;
	if (i >= len) return false;
	payload_len = len - i;
	if (payload_len > sizeof(payload)) return false;
	memcpy(payload, &src[i], payload_len);
	return true;
}

} /* namespace mesh */
