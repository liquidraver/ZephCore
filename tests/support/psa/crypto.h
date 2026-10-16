#pragma once
// Host stand-in for the PSA Crypto API, only the calls zephcore/src/Utils.cpp
// makes (SHA-256, AES-128-ECB without padding, HMAC-SHA256), over OpenSSL's
// libcrypto. It lets the mesh-level zcpfs tests run MeshCore's legacy
// encryption exactly as the firmware's Utils.cpp does. Not a PSA implementation.
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <cstdint>
#include <cstring>

typedef int32_t psa_status_t;
typedef uint32_t psa_algorithm_t;
typedef uint32_t psa_key_id_t;
typedef uint32_t psa_key_type_t;
typedef uint32_t psa_key_usage_t;

#define PSA_SUCCESS ((psa_status_t)0)
#define PSA_ERROR_GENERIC_ERROR ((psa_status_t)-132)
#define PSA_ERROR_INVALID_ARGUMENT ((psa_status_t)-135)
#define PSA_ERROR_INSUFFICIENT_MEMORY ((psa_status_t)-141)

#define PSA_ALG_SHA_256 ((psa_algorithm_t)0x02000009)
#define PSA_ALG_ECB_NO_PADDING ((psa_algorithm_t)0x04404400)
#define PSA_ALG_HMAC(hash) ((psa_algorithm_t)(0x03800000 | ((hash) & 0xff)))

#define PSA_KEY_TYPE_AES ((psa_key_type_t)0x2400)
#define PSA_KEY_TYPE_HMAC ((psa_key_type_t)0x1100)
#define PSA_KEY_USAGE_ENCRYPT ((psa_key_usage_t)0x100)
#define PSA_KEY_USAGE_DECRYPT ((psa_key_usage_t)0x200)
#define PSA_KEY_USAGE_SIGN_MESSAGE ((psa_key_usage_t)0x400)

struct psa_key_attributes_t {
    psa_key_type_t type;
    size_t bits;
    psa_key_usage_t usage;
    psa_algorithm_t alg;
};
#define PSA_KEY_ATTRIBUTES_INIT {0, 0, 0, 0}

inline void psa_set_key_type(psa_key_attributes_t *a, psa_key_type_t t) { a->type = t; }
inline void psa_set_key_bits(psa_key_attributes_t *a, size_t b) { a->bits = b; }
inline void psa_set_key_usage_flags(psa_key_attributes_t *a, psa_key_usage_t u) { a->usage = u; }
inline void psa_set_key_algorithm(psa_key_attributes_t *a, psa_algorithm_t alg) { a->alg = alg; }

namespace psa_host {
struct Key {
    bool used;
    psa_key_type_t type;
    uint8_t bytes[64];
    size_t len;
};
inline Key *keys() { static Key k[8]; return k; }
}  // namespace psa_host

inline psa_status_t psa_import_key(const psa_key_attributes_t *a, const uint8_t *data, size_t len,
                                   psa_key_id_t *id) {
    if (len > sizeof(psa_host::Key::bytes)) return PSA_ERROR_INVALID_ARGUMENT;
    for (uint32_t i = 0; i < 8; i++) {
        psa_host::Key &k = psa_host::keys()[i];
        if (!k.used) {
            k.used = true;
            k.type = a->type;
            memcpy(k.bytes, data, len);
            k.len = len;
            *id = i + 1;
            return PSA_SUCCESS;
        }
    }
    return PSA_ERROR_INSUFFICIENT_MEMORY;
}

inline psa_status_t psa_destroy_key(psa_key_id_t id) {
    if (id < 1 || id > 8) return PSA_ERROR_INVALID_ARGUMENT;
    psa_host::keys()[id - 1] = psa_host::Key{};
    return PSA_SUCCESS;
}

inline psa_status_t psa_hash_compute(psa_algorithm_t alg, const uint8_t *in, size_t in_len, uint8_t *out,
                                     size_t out_size, size_t *out_len) {
    unsigned int n = 0;
    if (alg != PSA_ALG_SHA_256 || out_size < 32) return PSA_ERROR_INVALID_ARGUMENT;
    if (!EVP_Digest(in, in_len, out, &n, EVP_sha256(), nullptr)) return PSA_ERROR_GENERIC_ERROR;
    *out_len = n;
    return PSA_SUCCESS;
}

struct psa_hash_operation_t {
    EVP_MD_CTX *ctx;
};
#define PSA_HASH_OPERATION_INIT {nullptr}

inline psa_status_t psa_hash_setup(psa_hash_operation_t *op, psa_algorithm_t alg) {
    if (alg != PSA_ALG_SHA_256) return PSA_ERROR_INVALID_ARGUMENT;
    op->ctx = EVP_MD_CTX_new();
    return op->ctx && EVP_DigestInit_ex(op->ctx, EVP_sha256(), nullptr) ? PSA_SUCCESS : PSA_ERROR_GENERIC_ERROR;
}
inline psa_status_t psa_hash_update(psa_hash_operation_t *op, const uint8_t *in, size_t len) {
    return EVP_DigestUpdate(op->ctx, in, len) ? PSA_SUCCESS : PSA_ERROR_GENERIC_ERROR;
}
inline psa_status_t psa_hash_abort(psa_hash_operation_t *op) {
    EVP_MD_CTX_free(op->ctx);
    op->ctx = nullptr;
    return PSA_SUCCESS;
}
inline psa_status_t psa_hash_finish(psa_hash_operation_t *op, uint8_t *out, size_t out_size, size_t *out_len) {
    unsigned int n = 0;
    if (out_size < 32) return PSA_ERROR_INVALID_ARGUMENT;
    int ok = EVP_DigestFinal_ex(op->ctx, out, &n);
    psa_hash_abort(op);
    *out_len = n;
    return ok ? PSA_SUCCESS : PSA_ERROR_GENERIC_ERROR;
}

inline psa_status_t psa_host_aes_ecb(psa_key_id_t id, const uint8_t *in, size_t len, uint8_t *out, bool enc) {
    if (id < 1 || id > 8 || len % 16 != 0) return PSA_ERROR_INVALID_ARGUMENT;
    psa_host::Key &k = psa_host::keys()[id - 1];
    if (!k.used || k.type != PSA_KEY_TYPE_AES || k.len != 16) return PSA_ERROR_INVALID_ARGUMENT;
    EVP_CIPHER_CTX *c = EVP_CIPHER_CTX_new();
    int n = 0, f = 0;
    bool ok = c && EVP_CipherInit_ex(c, EVP_aes_128_ecb(), nullptr, k.bytes, nullptr, enc ? 1 : 0) &&
              EVP_CIPHER_CTX_set_padding(c, 0) && EVP_CipherUpdate(c, out, &n, in, (int)len) &&
              EVP_CipherFinal_ex(c, out + n, &f);
    EVP_CIPHER_CTX_free(c);
    return ok && (size_t)(n + f) == len ? PSA_SUCCESS : PSA_ERROR_GENERIC_ERROR;
}

inline psa_status_t psa_cipher_encrypt(psa_key_id_t id, psa_algorithm_t alg, const uint8_t *in, size_t in_len,
                                       uint8_t *out, size_t out_size, size_t *out_len) {
    if (alg != PSA_ALG_ECB_NO_PADDING || out_size < in_len) return PSA_ERROR_INVALID_ARGUMENT;
    *out_len = in_len;
    return psa_host_aes_ecb(id, in, in_len, out, true);
}
inline psa_status_t psa_cipher_decrypt(psa_key_id_t id, psa_algorithm_t alg, const uint8_t *in, size_t in_len,
                                       uint8_t *out, size_t out_size, size_t *out_len) {
    if (alg != PSA_ALG_ECB_NO_PADDING || out_size < in_len) return PSA_ERROR_INVALID_ARGUMENT;
    *out_len = in_len;
    return psa_host_aes_ecb(id, in, in_len, out, false);
}

inline psa_status_t psa_mac_compute(psa_key_id_t id, psa_algorithm_t alg, const uint8_t *in, size_t in_len,
                                    uint8_t *mac, size_t mac_size, size_t *mac_len) {
    if (id < 1 || id > 8 || alg != PSA_ALG_HMAC(PSA_ALG_SHA_256) || mac_size < 32) return PSA_ERROR_INVALID_ARGUMENT;
    psa_host::Key &k = psa_host::keys()[id - 1];
    if (!k.used || k.type != PSA_KEY_TYPE_HMAC) return PSA_ERROR_INVALID_ARGUMENT;
    unsigned int n = 0;
    if (!HMAC(EVP_sha256(), k.bytes, (int)k.len, in, in_len, mac, &n)) return PSA_ERROR_GENERIC_ERROR;
    *mac_len = n;
    return PSA_SUCCESS;
}
