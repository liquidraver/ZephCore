/*
 * SPDX-License-Identifier: MIT
 * ZephCore Identity - Ed25519 sign/verify (Monocypher backend)
 *
 * Backed by Monocypher (lib/monocypher). The persisted private key keeps the
 * historical 64-byte *expanded* layout — prv[0..31] = clamped SHA-512(seed)
 * scalar `a`, prv[32..63] = nonce prefix — so identities written by older
 * (orlp/ed25519) firmware load, sign, and key-exchange byte-for-byte unchanged.
 * Signing is built from Monocypher's low-level EdDSA primitives because the
 * stored expanded key carries no seed to feed the high-level API.
 */

#include <mesh/Identity.h>
#include <string.h>
#include <monocypher.h>
#include <monocypher-ed25519.h>

namespace mesh {

/* Expand a 32-byte seed into the stored 64-byte private key (orlp-compatible
 * layout): clamped SHA-512(seed)[0..31] || SHA-512(seed)[32..63]. */
static void expandSeed(uint8_t prv[PRV_KEY_SIZE], const uint8_t seed[SEED_SIZE])
{
	crypto_sha512(prv, seed, SEED_SIZE);
	prv[0]  &= 248;
	prv[31] &= 63;
	prv[31] |= 64;
}

/* Sign from the expanded key (no seed). RFC 8032 Ed25519 assembled from
 * Monocypher scalar/point primitives — byte-identical to the prior orlp
 * implementation (verified by known-answer tests). */
static void signExpanded(uint8_t sig[SIGNATURE_SIZE],
			 const uint8_t prv[PRV_KEY_SIZE],
			 const uint8_t pub[PUB_KEY_SIZE],
			 const uint8_t *msg, size_t msg_len)
{
	uint8_t r64[64], hram64[64], r[32], hram[32];
	crypto_sha512_ctx h;

	/* r = SHA-512(prefix || msg) mod L ; prefix = prv[32..63] */
	crypto_sha512_init(&h);
	crypto_sha512_update(&h, prv + 32, 32);
	crypto_sha512_update(&h, msg, msg_len);
	crypto_sha512_final(&h, r64);
	crypto_eddsa_reduce(r, r64);

	/* R = r·B -> sig[0..31] */
	crypto_eddsa_scalarbase(sig, r);

	/* hram = SHA-512(R || A || msg) mod L */
	crypto_sha512_init(&h);
	crypto_sha512_update(&h, sig, 32);
	crypto_sha512_update(&h, pub, 32);
	crypto_sha512_update(&h, msg, msg_len);
	crypto_sha512_final(&h, hram64);
	crypto_eddsa_reduce(hram, hram64);

	/* S = (hram·a + r) mod L ; a = prv[0..31] -> sig[32..63] */
	crypto_eddsa_mul_add(sig + 32, hram, prv, r);

	/* r / r64 are nonce material — leaking them leaks the private key. */
	crypto_wipe(r64, sizeof(r64));
	crypto_wipe(r, sizeof(r));
}

/* X25519 over Ed25519 keys: convert the peer's Edwards public key to its
 * Montgomery form, then scalar-multiply by our scalar (prv[0..31]). Raw
 * shared secret, no output hashing — matches the prior orlp key_exchange. */
static void calcECDH(uint8_t secret[CIPHER_KEY_SIZE * 2],
		     const uint8_t prv[PRV_KEY_SIZE],
		     const uint8_t other_pub[PUB_KEY_SIZE])
{
	uint8_t other_x[32];
	crypto_eddsa_to_x25519(other_x, other_pub);
	crypto_x25519(secret, prv, other_x);
}

Identity::Identity()
{
	memset(pub_key, 0, sizeof(pub_key));
}

Identity::Identity(const char *pub_hex)
{
	Utils::fromHex(pub_key, PUB_KEY_SIZE, pub_hex);
}

bool Identity::verify(const uint8_t *sig, const uint8_t *message, int msg_len) const
{
	return crypto_ed25519_check(sig, pub_key, message, (size_t)msg_len) == 0;
}

bool Identity::readFrom(const uint8_t *src, size_t len)
{
	if (len < PUB_KEY_SIZE) return false;
	memcpy(pub_key, src, PUB_KEY_SIZE);
	return true;
}

bool Identity::writeTo(uint8_t *dest, size_t max_len) const
{
	if (max_len < PUB_KEY_SIZE) return false;
	memcpy(dest, pub_key, PUB_KEY_SIZE);
	return true;
}

LocalIdentity::LocalIdentity()
{
	memset(prv_key, 0, sizeof(prv_key));
}

LocalIdentity::LocalIdentity(const char *prv_hex, const char *pub_hex) : Identity(pub_hex)
{
	Utils::fromHex(prv_key, PRV_KEY_SIZE, prv_hex);
}

LocalIdentity::LocalIdentity(RNG *rng)
{
	uint8_t seed[SEED_SIZE];
	rng->random(seed, SEED_SIZE);
	fromSeed(seed);
	Utils::secureZeroize(seed, sizeof(seed));
}

void LocalIdentity::fromSeed(const uint8_t seed[SEED_SIZE])
{
	expandSeed(prv_key, seed);
	crypto_eddsa_scalarbase(pub_key, prv_key);  /* pub = a·B */
}

bool LocalIdentity::validatePrivateKey(const uint8_t prv[64])
{
	uint8_t pub[32];
	crypto_eddsa_scalarbase(pub, prv);
	if (pub[0] == 0x00 || pub[0] == 0xFF) return false;

	const uint8_t test_client_prv[64] = {
		0x70, 0x65, 0xe1, 0x8f, 0xd9, 0xfa, 0xbb, 0x70,
		0xc1, 0xed, 0x90, 0xdc, 0xa1, 0x99, 0x07, 0xde,
		0x69, 0x8c, 0x88, 0xb7, 0x09, 0xea, 0x14, 0x6e,
		0xaf, 0xd9, 0x3d, 0x9b, 0x83, 0x0c, 0x7b, 0x60,
		0xc4, 0x68, 0x11, 0x93, 0xc7, 0x9b, 0xbc, 0x39,
		0x94, 0x5b, 0xa8, 0x06, 0x41, 0x04, 0xbb, 0x61,
		0x8f, 0x8f, 0xd7, 0xa8, 0x4a, 0x0a, 0xf6, 0xf5,
		0x70, 0x33, 0xd6, 0xe8, 0xdd, 0xcd, 0x64, 0x71
	};
	const uint8_t test_client_pub[32] = {
		0x1e, 0xc7, 0x71, 0x75, 0xb0, 0x91, 0x8e, 0xd2,
		0x06, 0xf9, 0xae, 0x04, 0xec, 0x13, 0x6d, 0x6d,
		0x5d, 0x43, 0x15, 0xbb, 0x26, 0x30, 0x54, 0x27,
		0xf6, 0x45, 0xb4, 0x92, 0xe9, 0x35, 0x0c, 0x10
	};

	uint8_t ss1[32], ss2[32];
	calcECDH(ss1, prv, test_client_pub);
	calcECDH(ss2, test_client_prv, pub);
	/* Constant-time even though this self-test runs at boot before
	 * any networking is up — hygiene + no attacker observation. */
	if (!Utils::constantTimeEqual(ss1, ss2, 32)) {
		Utils::secureZeroize(ss1, sizeof(ss1));
		Utils::secureZeroize(ss2, sizeof(ss2));
		return false;
	}

	bool nonzero = false;
	for (int i = 0; i < 32; i++) {
		if (ss1[i] != 0) { nonzero = true; break; }
	}
	Utils::secureZeroize(ss1, sizeof(ss1));
	Utils::secureZeroize(ss2, sizeof(ss2));
	return nonzero;
}

bool LocalIdentity::readFrom(const uint8_t *src, size_t len)
{
	if (len == PRV_KEY_SIZE + PUB_KEY_SIZE) {
		memcpy(prv_key, src, PRV_KEY_SIZE);
		memcpy(pub_key, src + PRV_KEY_SIZE, PUB_KEY_SIZE);
		return true;
	}
	if (len == PRV_KEY_SIZE) {
		memcpy(prv_key, src, PRV_KEY_SIZE);
		crypto_eddsa_scalarbase(pub_key, prv_key);  /* derive pub from a */
		return true;
	}
	return false;
}

size_t LocalIdentity::writeTo(uint8_t *dest, size_t max_len) const
{
	if (max_len < PRV_KEY_SIZE) return 0;
	if (max_len < PRV_KEY_SIZE + PUB_KEY_SIZE) {
		memcpy(dest, prv_key, PRV_KEY_SIZE);
		return PRV_KEY_SIZE;
	}
	memcpy(dest, prv_key, PRV_KEY_SIZE);
	memcpy(dest + PRV_KEY_SIZE, pub_key, PUB_KEY_SIZE);
	return PRV_KEY_SIZE + PUB_KEY_SIZE;
}

void LocalIdentity::sign(uint8_t *sig, const uint8_t *message, int msg_len) const
{
	signExpanded(sig, prv_key, pub_key, message, (size_t)msg_len);
}

void LocalIdentity::calcSharedSecret(uint8_t *secret, const uint8_t *other_pub_key) const
{
	calcECDH(secret, prv_key, other_pub_key);
}

} /* namespace mesh */
