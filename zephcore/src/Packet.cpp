#include <mesh/Packet.h>
#include <mesh/Utils.h>  // ZEPHCORE: SHA-256 via Utils (no Arduino SHA256 class)
#include <string.h>

// ZEPHCORE: log module backing MESH_DEBUG_PRINTLN (see MeshCore.h).
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zephcore_mesh, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

namespace mesh {

Packet::Packet() {
  header = 0;
  path_len = 0;
  payload_len = 0;
}

bool Packet::isValidPathLen(uint8_t path_len) {
  uint8_t hash_count = path_len & 63;
  uint8_t hash_size = (path_len >> 6) + 1;
  if (hash_size == 4) return false;  // Reserved for future
  return hash_count*hash_size <= MAX_PATH_SIZE;
}

size_t Packet::writePath(uint8_t* dest, const uint8_t* src, size_t src_len, uint8_t path_len) {
  uint8_t hash_count = path_len & 63;
  uint8_t hash_size = (path_len >> 6) + 1;
  size_t len = hash_count*hash_size;
  if (len > MAX_PATH_SIZE) {
    MESH_DEBUG_PRINTLN("Packet::copyPath, invalid path_len=%d", (uint32_t)path_len);
    return 0;   // Error
  }
  if (len > src_len) return 0;  // ZEPHCORE: would read past the caller-supplied source bound
  memcpy(dest, src, len);
  return len;
}

uint8_t Packet::copyPath(uint8_t* dest, const uint8_t* src, size_t src_len, uint8_t path_len) {
  // ZEPHCORE: 0 means "rejected" (upstream always returns path_len). A zero-hop
  // path copies nothing, so validate it instead of copying: a directly-heard
  // 2-byte-hash advert (0x40) must keep its hash-size bits.
  if ((path_len & 0x3F) == 0) {
    return isValidPathLen(path_len) ? path_len : 0;
  }
  return writePath(dest, src, src_len, path_len) > 0 ? path_len : 0;
}

int Packet::getRawLength() const {
  return 2 + getPathByteLen() + payload_len + (hasTransportCodes() ? 4 : 0);
}

void Packet::calculatePacketHash(uint8_t* hash) const {
  // ZEPHCORE: one-shot Utils::sha256 over (type [, path_len], payload), same digest
  // as upstream's incremental SHA256.
  uint8_t t = getPayloadType();
  if (t == PAYLOAD_TYPE_TRACE) {   // CAVEAT: TRACE packets can revisit same node on return path
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

uint8_t Packet::writeTo(uint8_t dest[]) const {
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

bool Packet::readFrom(const uint8_t src[], uint8_t len) {
  uint8_t i = 0;
  if (len < 2) return false;  // ZEPHCORE: bounds
  header = src[i++];
  if (hasTransportCodes()) {
    if (len < 6) return false;  // ZEPHCORE
    memcpy(&transport_codes[0], &src[i], 2); i += 2;
    memcpy(&transport_codes[1], &src[i], 2); i += 2;
  } else {
    transport_codes[0] = transport_codes[1] = 0;
  }
  path_len = src[i++];
  if (!isValidPathLen(path_len)) return false;   // bad encoding

  uint8_t bl = getPathByteLen();
  if ((uint16_t)i + bl > len) return false;  // ZEPHCORE
  memcpy(path, &src[i], bl); i += bl;

  if (i >= len) return false;   // bad encoding
  payload_len = len - i;
  if (payload_len > sizeof(payload)) return false;  // bad encoding
  memcpy(payload, &src[i], payload_len); //i += payload_len;
  return true;   // success
}

}