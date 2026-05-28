/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Identity - Ed25519 sign/verify
 */

#include <mesh/Identity.h>
#include <string.h>
#define ED25519_NO_SEED 1
#include <ed_25519.h>

namespace mesh {

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
	return ed25519_verify(sig, message, (size_t)msg_len, pub_key) == 1;
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
	ed25519_create_keypair(pub_key, prv_key, seed);
}

bool LocalIdentity::validatePrivateKey(const uint8_t prv[64])
{
	uint8_t pub[32];
	ed25519_derive_pub(pub, prv);
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
	ed25519_key_exchange(ss1, test_client_pub, prv);
	ed25519_key_exchange(ss2, pub, test_client_prv);
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
		ed25519_derive_pub(pub_key, prv_key);
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
	ed25519_sign(sig, message, (size_t)msg_len, pub_key, prv_key);
}

void LocalIdentity::calcSharedSecret(uint8_t *secret, const uint8_t *other_pub_key) const
{
	ed25519_key_exchange(secret, other_pub_key, prv_key);
}

} /* namespace mesh */
