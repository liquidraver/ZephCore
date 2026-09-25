/*
 * SPDX-License-Identifier: MIT
 * The Arduino Crypto library's SHA256 interface (reset / update / finalize,
 * and the HMAC variants) over PSA Crypto, for code ported verbatim from
 * upstream MeshCore.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <psa/crypto.h>

class SHA256 {
	psa_hash_operation_t _hash;
	psa_mac_operation_t _mac;
	psa_key_id_t _key;
	bool _hmac;

public:
	SHA256();
	~SHA256() { clear(); }

	void reset();
	void update(const void *data, size_t len);
	void finalize(void *hash, size_t len);

	void resetHMAC(const void *key, size_t keyLen);
	void finalizeHMAC(const void *key, size_t keyLen, void *hash, size_t hashLen);

	void clear();
};
