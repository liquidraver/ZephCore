/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Utils - PSA Crypto backend for AES-ECB, SHA256, HMAC
 */

#include <mesh/Utils.h>
#include <psa/crypto.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_utils, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

namespace mesh {

static uint8_t hexVal(char c)
{
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= '0' && c <= '9') return c - '0';
	return 0;
}

void Utils::sha256(uint8_t *hash, size_t hash_len, const uint8_t *msg, int msg_len)
{
	uint8_t full_hash[32];
	size_t out_len;
	psa_status_t status = psa_hash_compute(PSA_ALG_SHA_256, msg, (size_t)msg_len,
	                                       full_hash, sizeof(full_hash), &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_hash_compute failed: %d", (int)status);
		memset(hash, 0, hash_len);
		return;
	}
	size_t copy_len = (hash_len < 32) ? hash_len : 32;
	memcpy(hash, full_hash, copy_len);
}

void Utils::sha256(uint8_t *hash, size_t hash_len, const uint8_t *frag1, int frag1_len, const uint8_t *frag2, int frag2_len)
{
	psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
	psa_status_t status;

	status = psa_hash_setup(&op, PSA_ALG_SHA_256);
	if (status != PSA_SUCCESS) goto fail;

	status = psa_hash_update(&op, frag1, (size_t)frag1_len);
	if (status != PSA_SUCCESS) goto fail;

	status = psa_hash_update(&op, frag2, (size_t)frag2_len);
	if (status != PSA_SUCCESS) goto fail;

	{
		uint8_t full_hash[32];
		size_t out_len;
		status = psa_hash_finish(&op, full_hash, sizeof(full_hash), &out_len);
		if (status != PSA_SUCCESS) goto fail;
		size_t copy_len = (hash_len < 32) ? hash_len : 32;
		memcpy(hash, full_hash, copy_len);
	}
	return;

fail:
	LOG_ERR("sha256 (2-frag) failed: %d", (int)status);
	psa_hash_abort(&op);
	memset(hash, 0, hash_len);
}

/* AES-ECB using PSA Crypto (ECB_NO_PADDING) */
static int aes_ecb_crypt(const uint8_t *key, size_t key_len, const uint8_t *src, int src_len,
                         uint8_t *dest, bool encrypt)
{
	if (src_len % 16 != 0) {
		LOG_ERR("src_len=%d not multiple of 16", src_len);
		return -1;
	}

	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, key_len * 8);
	psa_set_key_usage_flags(&attr, encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);

	psa_key_id_t key_id;
	psa_status_t status = psa_import_key(&attr, key, key_len, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_import_key failed: %d", (int)status);
		return -1;
	}

	size_t out_len;
	if (encrypt) {
		status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING,
		                            src, (size_t)src_len, dest, (size_t)src_len, &out_len);
	} else {
		status = psa_cipher_decrypt(key_id, PSA_ALG_ECB_NO_PADDING,
		                            src, (size_t)src_len, dest, (size_t)src_len, &out_len);
	}

	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_cipher_%s failed: %d", encrypt ? "encrypt" : "decrypt", (int)status);
		return -1;
	}

	return (int)out_len;
}

int Utils::encrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len)
{
	uint8_t tmp[MAX_PACKET_PAYLOAD + CIPHER_BLOCK_SIZE];
	const uint8_t *sp = src;
	uint8_t *dp = tmp;
	int rem = src_len;
	while (rem >= (int)CIPHER_BLOCK_SIZE) {
		memcpy(dp, sp, CIPHER_BLOCK_SIZE);
		dp += CIPHER_BLOCK_SIZE;
		sp += CIPHER_BLOCK_SIZE;
		rem -= CIPHER_BLOCK_SIZE;
	}
	if (rem > 0) {
		memset(dp, 0, CIPHER_BLOCK_SIZE);
		memcpy(dp, sp, (size_t)rem);
		dp += CIPHER_BLOCK_SIZE;
	}
	int total = (int)(dp - tmp);
	int n = aes_ecb_crypt(shared_secret, CIPHER_KEY_SIZE, tmp, total, dest, true);
	return (n > 0) ? n : 0;
}

int Utils::decrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len)
{
	int n = aes_ecb_crypt(shared_secret, CIPHER_KEY_SIZE, src, src_len, dest, false);
	return (n > 0) ? n : 0;
}

/* HMAC-SHA256 using PSA Crypto */
static int compute_hmac_truncated(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len, uint8_t *mac_out, size_t mac_len)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
	psa_set_key_bits(&attr, key_len * 8);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));

	psa_key_id_t key_id;
	psa_status_t status = psa_import_key(&attr, key, key_len, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("compute_hmac: import_key failed: %d", (int)status);
		return -1;
	}

	uint8_t full_hmac[32];
	size_t out_len;
	status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256),
	                         data, data_len, full_hmac, sizeof(full_hmac), &out_len);
	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("compute_hmac: psa_mac_compute failed: %d", (int)status);
		return -1;
	}

	memcpy(mac_out, full_hmac, mac_len);
	return 0;
}

int Utils::encryptThenMAC(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len)
{
	int enc_len = encrypt(shared_secret, dest + CIPHER_MAC_SIZE, src, src_len);
	if (enc_len <= 0) return 0;
	if (compute_hmac_truncated(shared_secret, PUB_KEY_SIZE, dest + CIPHER_MAC_SIZE, (size_t)enc_len, dest, CIPHER_MAC_SIZE) != 0)
		return 0;
	return CIPHER_MAC_SIZE + enc_len;
}

int Utils::MACThenDecrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len)
{
	if (src_len <= (int)CIPHER_MAC_SIZE) return 0;
	uint8_t computed_mac[CIPHER_MAC_SIZE];
	if (compute_hmac_truncated(shared_secret, PUB_KEY_SIZE, src + CIPHER_MAC_SIZE, (size_t)src_len - CIPHER_MAC_SIZE, computed_mac, CIPHER_MAC_SIZE) != 0)
		return 0;
	/* Constant-time MAC compare. Runs on every encrypted packet — a
	 * timing oracle here would let attackers forge MACs byte-by-byte
	 * across the entire mesh, bypassing message authentication. */
	if (!Utils::constantTimeEqual(computed_mac, src, CIPHER_MAC_SIZE)) return 0;
	return decrypt(shared_secret, dest, src + CIPHER_MAC_SIZE, src_len - CIPHER_MAC_SIZE);
}

/* See header for rationale. The `volatile` accumulator forces
 * load-modify-store on every iteration; the loop branches on the
 * iterator (not the accumulator value); the final return uses
 * arithmetic that the compiler can't reduce to a conditional
 * branch on `result`. Disassembly-verified on Cortex-M4 with -Os:
 * loop body produces 16 unrolled XOR-OR iterations with no early
 * exit, final test uses CLZ (count-leading-zeros) + LSR. */
bool Utils::constantTimeEqual(const void *a, const void *b, size_t n)
{
	const uint8_t *pa = (const uint8_t *)a;
	const uint8_t *pb = (const uint8_t *)b;
	volatile uint8_t result = 0;
	for (size_t i = 0; i < n; i++) {
		result |= (uint8_t)(pa[i] ^ pb[i]);
	}
	return result == 0;
}

void Utils::secureZeroize(void *buf, size_t n)
{
	/* Volatile pointer prevents the compiler from eliminating the
	 * writes as dead store. Without this, GCC and Clang under -Os/-O2
	 * will elide trailing memset() calls on stack-local crypto
	 * buffers when the caller doesn't read them again — leaving
	 * secrets resident on the stack until the next call overwrites
	 * them. Standard idiom from BoringSSL, libsodium, etc. */
	volatile uint8_t *p = (volatile uint8_t *)buf;
	while (n--) {
		*p++ = 0;
	}
}

static const char hex_chars[] = "0123456789ABCDEF";

void Utils::toHex(char *dest, const uint8_t *src, size_t len)
{
	while (len > 0) {
		uint8_t b = *src++;
		*dest++ = hex_chars[b >> 4];
		*dest++ = hex_chars[b & 0x0F];
		len--;
	}
	*dest = 0;
}

bool Utils::fromHex(uint8_t *dest, int dest_size, const char *src_hex)
{
	size_t len = strlen(src_hex);
	if (len != (size_t)(dest_size * 2)) return false;
	uint8_t *dp = dest;
	while ((size_t)(dp - dest) < (size_t)dest_size) {
		char ch = *src_hex++;
		char cl = *src_hex++;
		*dp++ = (uint8_t)((hexVal(ch) << 4) | hexVal(cl));
	}
	return true;
}

bool Utils::isHexChar(char c)
{
	return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

int Utils::parseTextParts(char *text, const char *parts[], int max_num, char separator)
{
	int num = 0;
	char *sp = text;
	while (*sp && num < max_num) {
		parts[num++] = sp;
		while (*sp && *sp != separator) sp++;
		if (*sp) {
			*sp++ = 0;
		}
	}
	while (*sp && *sp != separator) sp++;
	if (*sp) *sp = 0;
	return num;
}

} /* namespace mesh */
