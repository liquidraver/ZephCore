/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Utils - mbedTLS backend for AES-ECB, SHA256, HMAC
 */

#include <mesh/Utils.h>
#include <mbedtls/aes.h>
#include <mbedtls/sha256.h>
#include <mbedtls/md.h>
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
	mbedtls_sha256(msg, (size_t)msg_len, full_hash, 0);
	size_t copy_len = (hash_len < 32) ? hash_len : 32;
	memcpy(hash, full_hash, copy_len);
}

void Utils::sha256(uint8_t *hash, size_t hash_len, const uint8_t *frag1, int frag1_len, const uint8_t *frag2, int frag2_len)
{
	mbedtls_sha256_context ctx;
	mbedtls_sha256_init(&ctx);
	mbedtls_sha256_starts(&ctx, 0);
	mbedtls_sha256_update(&ctx, frag1, (size_t)frag1_len);
	mbedtls_sha256_update(&ctx, frag2, (size_t)frag2_len);
	uint8_t full_hash[32];
	mbedtls_sha256_finish(&ctx, full_hash);
	mbedtls_sha256_free(&ctx);
	size_t copy_len = (hash_len < 32) ? hash_len : 32;
	memcpy(hash, full_hash, copy_len);
}

/* AES-ECB using mbedTLS low-level API (PSA doesn't support ECB mode) */
static int aes_ecb_encrypt(const uint8_t *key, size_t key_len, const uint8_t *src, int src_len, uint8_t *dest)
{
	if (src_len % 16 != 0) {
		LOG_ERR("src_len=%d not multiple of 16", src_len);
		return -1;
	}

	mbedtls_aes_context ctx;
	mbedtls_aes_init(&ctx);

	int ret = mbedtls_aes_setkey_enc(&ctx, key, key_len * 8);
	if (ret != 0) {
		LOG_ERR("setkey failed: %d", ret);
		mbedtls_aes_free(&ctx);
		return -1;
	}

	/* Encrypt each 16-byte block */
	for (int i = 0; i < src_len; i += 16) {
		ret = mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, src + i, dest + i);
		if (ret != 0) {
			LOG_ERR("crypt_ecb failed at block %d: %d", i/16, ret);
			mbedtls_aes_free(&ctx);
			return -1;
		}
	}

	mbedtls_aes_free(&ctx);
	return src_len;
}

static int aes_ecb_decrypt(const uint8_t *key, size_t key_len, const uint8_t *src, int src_len, uint8_t *dest)
{
	if (src_len % 16 != 0) {
		LOG_ERR("src_len=%d not multiple of 16", src_len);
		return -1;
	}

	mbedtls_aes_context ctx;
	mbedtls_aes_init(&ctx);

	int ret = mbedtls_aes_setkey_dec(&ctx, key, key_len * 8);
	if (ret != 0) {
		LOG_ERR("setkey failed: %d", ret);
		mbedtls_aes_free(&ctx);
		return -1;
	}

	/* Decrypt each 16-byte block */
	for (int i = 0; i < src_len; i += 16) {
		ret = mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_DECRYPT, src + i, dest + i);
		if (ret != 0) {
			LOG_ERR("crypt_ecb failed at block %d: %d", i/16, ret);
			mbedtls_aes_free(&ctx);
			return -1;
		}
	}

	mbedtls_aes_free(&ctx);
	return src_len;
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
	int n = aes_ecb_encrypt(shared_secret, CIPHER_KEY_SIZE, tmp, total, dest);
	return (n > 0) ? n : 0;
}

int Utils::decrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len)
{
	int n = aes_ecb_decrypt(shared_secret, CIPHER_KEY_SIZE, src, src_len, dest);
	return (n > 0) ? n : 0;
}

/* HMAC-SHA256 using mbedTLS */
static int compute_hmac_truncated(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len, uint8_t *mac_out, size_t mac_len)
{
	uint8_t full_hmac[32];
	const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
	if (md_info == nullptr) {
		LOG_ERR("compute_hmac: SHA256 not available");
		return -1;
	}

	int ret = mbedtls_md_hmac(md_info, key, key_len, data, data_len, full_hmac);
	if (ret != 0) {
		LOG_ERR("compute_hmac: mbedtls_md_hmac failed: %d", ret);
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
	if (memcmp(computed_mac, src, CIPHER_MAC_SIZE) != 0) return 0;
	return decrypt(shared_secret, dest, src + CIPHER_MAC_SIZE, src_len - CIPHER_MAC_SIZE);
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
