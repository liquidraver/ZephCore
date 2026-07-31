/*
 * SPDX-License-Identifier: MIT
 * TransportKeyStore - Transport key cache for region filtering
 */

#include "TransportKeyStore.h"
#include <string.h>
#include <psa/crypto.h>

uint16_t TransportKey::calcTransportCode(const mesh::Packet* packet) const {
    /* HMAC-SHA256 using PSA Crypto */
    uint8_t hmac[32];
    uint8_t type = packet->getPayloadType();

    /* Build message: type + payload */
    uint8_t msg[MAX_PACKET_PAYLOAD + 1];
    msg[0] = type;
    memcpy(&msg[1], packet->payload, packet->payload_len);
    size_t msg_len = 1 + packet->payload_len;

    /* Import HMAC key */
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attr, sizeof(key) * 8);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));

    psa_key_id_t key_id;
    psa_status_t status = psa_import_key(&attr, key, sizeof(key), &key_id);
    if (status != PSA_SUCCESS) {
        return 0;
    }

    size_t out_len;
    status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256),
                             msg, msg_len, hmac, sizeof(hmac), &out_len);
    psa_destroy_key(key_id);

    if (status != PSA_SUCCESS) {
        return 0;
    }

    /* Extract first 2 bytes as transport code (little-endian, matching Arduino SHA256 finalizeHMAC behavior) */
    uint16_t code = (uint16_t)hmac[0] | ((uint16_t)hmac[1] << 8);
    if (code == 0) {       // reserve codes 0000 and FFFF
        code++;
    } else if (code == 0xFFFF) {
        code--;
    }
    return code;
}

bool TransportKey::isNull() const {
    for (size_t i = 0; i < sizeof(key); i++) {
        if (key[i]) return false;
    }
    return true;  // key is all zeroes
}

void TransportKeyStore::putCache(uint16_t id, const TransportKey& key) {
    if (num_cache < MAX_TKS_ENTRIES) {
        cache_ids[num_cache] = id;
        cache_keys[num_cache] = key;
        num_cache++;
    } else {
        // TODO: evict oldest cache entry
    }
}

void TransportKeyStore::getAutoKeyFor(uint16_t id, const char* name, TransportKey& dest) {
    // first, check cache
    for (int i = 0; i < num_cache; i++) {
        if (cache_ids[i] == id) {  // cache hit!
            dest = cache_keys[i];
            return;
        }
    }

    // calc key for publicly-known hashtag region name (SHA256 hash, first 16 bytes)
    uint8_t hash[32];
    size_t out_len;
    psa_hash_compute(PSA_ALG_SHA_256, (const uint8_t*)name, strlen(name),
                     hash, sizeof(hash), &out_len);
    memcpy(dest.key, hash, sizeof(dest.key));

    putCache(id, dest);
}

int TransportKeyStore::loadKeysFor(uint16_t id, TransportKey keys[], int max_num) {
    int n = 0;
    // first, check cache
    for (int i = 0; i < num_cache && n < max_num; i++) {
        if (cache_ids[i] == id) {
            keys[n++] = cache_keys[i];
        }
    }
    if (n > 0) return n;  // cache hit!

    // TODO: retrieve from hardware keystore

    // store in cache (if room)
    for (int i = 0; i < n; i++) {
        putCache(id, keys[i]);
    }
    return n;
}

bool TransportKeyStore::saveKeysFor(uint16_t id, const TransportKey keys[], int num) {
    invalidateCache();
    // TODO: update hardware keystore
    return false;  // failed
}

bool TransportKeyStore::removeKeys(uint16_t id) {
    invalidateCache();
    // TODO: remove from hardware keystore
    return false;  // failed
}

bool TransportKeyStore::clear() {
    invalidateCache();
    // TODO: clear hardware keystore
    return false;  // failed
}
