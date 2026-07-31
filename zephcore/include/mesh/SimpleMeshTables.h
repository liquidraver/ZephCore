/*
 * SPDX-License-Identifier: MIT
 * SimpleMeshTables - hash-based packet deduplication
 */

#pragma once

#include <mesh/Mesh.h>
#include <string.h>

namespace mesh {

#define MAX_PACKET_HASHES  (128+32)

class SimpleMeshTables : public MeshTables {
	uint8_t _hashes[MAX_PACKET_HASHES * MAX_HASH_SIZE];
	int _next_idx;
	uint32_t _direct_dups, _flood_dups;

public:
	SimpleMeshTables() {
		memset(_hashes, 0, sizeof(_hashes));
		_next_idx = 0;
		_direct_dups = _flood_dups = 0;
	}

	bool wasSeen(const Packet *packet) override {
		uint8_t hash[MAX_HASH_SIZE];
		packet->calculatePacketHash(hash);
		const uint8_t *sp = _hashes;
		for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
			if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) {
				if (packet->isRouteDirect()) {
					_direct_dups++;
				} else {
					_flood_dups++;
				}
				return true;
			}
		}
		return false;
	}

	void markSeen(const Packet *packet) override {
		uint8_t hash[MAX_HASH_SIZE];
		packet->calculatePacketHash(hash);
		memcpy(&_hashes[_next_idx * MAX_HASH_SIZE], hash, MAX_HASH_SIZE);
		_next_idx = (_next_idx + 1) % MAX_PACKET_HASHES;
	}

	void clear(const Packet *packet) override {
		uint8_t hash[MAX_HASH_SIZE];
		packet->calculatePacketHash(hash);
		uint8_t *sp = _hashes;
		for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
			if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) {
				memset(sp, 0, MAX_HASH_SIZE);
				break;
			}
		}
	}

	uint32_t getNumDirectDups() const { return _direct_dups; }
	uint32_t getNumFloodDups() const { return _flood_dups; }

	void resetStats() { _direct_dups = _flood_dups = 0; }
};

} /* namespace mesh */
