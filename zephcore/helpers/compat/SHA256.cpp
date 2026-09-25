/*
 * SPDX-License-Identifier: MIT
 */

#include "SHA256.h"
#include <string.h>

SHA256::SHA256() : _hash(PSA_HASH_OPERATION_INIT), _mac(PSA_MAC_OPERATION_INIT),
		   _key(PSA_KEY_ID_NULL), _hmac(false)
{
	reset();
}

void SHA256::clear()
{
	psa_hash_abort(&_hash);
	psa_mac_abort(&_mac);
	if (_key != PSA_KEY_ID_NULL) {
		psa_destroy_key(_key);
		_key = PSA_KEY_ID_NULL;
	}
	_hmac = false;
}

void SHA256::reset()
{
	clear();
	psa_hash_setup(&_hash, PSA_ALG_SHA_256);
}

void SHA256::update(const void *data, size_t len)
{
	if (_hmac) {
		psa_mac_update(&_mac, (const uint8_t *)data, len);
	} else {
		psa_hash_update(&_hash, (const uint8_t *)data, len);
	}
}

void SHA256::finalize(void *hash, size_t len)
{
	uint8_t out[32] = {0};
	size_t out_len;
	psa_hash_finish(&_hash, out, sizeof(out), &out_len);
	memcpy(hash, out, len < sizeof(out) ? len : sizeof(out));
	clear();
}

void SHA256::resetHMAC(const void *key, size_t keyLen)
{
	clear();
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
	psa_set_key_bits(&attr, keyLen * 8);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	if (psa_import_key(&attr, (const uint8_t *)key, keyLen, &_key) == PSA_SUCCESS) {
		psa_mac_sign_setup(&_mac, _key, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	}
	_hmac = true;
}

void SHA256::finalizeHMAC(const void *key, size_t keyLen, void *hash, size_t hashLen)
{
	(void)key;
	(void)keyLen;
	uint8_t out[32] = {0};
	size_t out_len;
	psa_mac_sign_finish(&_mac, out, sizeof(out), &out_len);
	memcpy(hash, out, hashLen < sizeof(out) ? hashLen : sizeof(out));
	clear();
}
