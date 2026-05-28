/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Utils - crypto and helpers (Zephyr port)
 */

#pragma once

#include <mesh/MeshCore.h>
#include <string.h>
#include <stddef.h>

namespace mesh {

class Utils {
public:
	static void sha256(uint8_t *hash, size_t hash_len, const uint8_t *msg, int msg_len);
	static void sha256(uint8_t *hash, size_t hash_len, const uint8_t *frag1, int frag1_len, const uint8_t *frag2, int frag2_len);
	static int encrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static int decrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static int encryptThenMAC(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static int MACThenDecrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);

	/* Constant-time byte-equality. Returns true iff every byte of `a`
	 * matches `b`. No early exit — timing is independent of input,
	 * defeating timing-leak attacks on MAC/password/secret compares.
	 * `volatile` accumulator survives `-Os` LTO — disassembly-verified
	 * on Cortex-M4 (rak3401_1watt). Pattern from rweather/arduinolibs. */
	static bool constantTimeEqual(const void *a, const void *b, size_t n);

	/* Securely zero a buffer such that the compiler cannot elide the
	 * writes as dead-store optimization. Uses volatile pointer writes —
	 * standard idiom for clearing crypto secrets before stack unwind.
	 * Use this for any buffer holding key material, seeds, or shared
	 * secrets after their last use. */
	static void secureZeroize(void *buf, size_t n);

	static void toHex(char *dest, const uint8_t *src, size_t len);
	static bool fromHex(uint8_t *dest, int dest_size, const char *src_hex);
	static bool isHexChar(char c);
	static int parseTextParts(char *text, const char *parts[], int max_num, char separator = ',');
};

} /* namespace mesh */
