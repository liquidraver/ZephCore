/*
 * SPDX-License-Identifier: MIT
 * Section 19: login as a password-authenticated key exchange. Both sides
 * multiply a one-time scalar onto a curve point derived from the password;
 * the password itself is never sent.
 *
 *   login   dst ‖ S_c ‖ Y_c ‖ C ‖ T8        C: a flag byte, the legacy login   (ANON_REQ)
 *   reply   dst ‖ src ‖ count ‖ branches ‖ zeros                              (RESPONSE)
 *   branch  Y_s ‖ C ‖ T4, one per password the server would accept; C: 6 bytes
 *
 * The flag says whether the typed password is empty, which keeps a reply to
 * two branches. A branch's tag is 4 bytes: it only tells the client which
 * branch is its own, and the server trusts nothing but the 8-byte tag of the
 * first packet under the session (19.5).
 */

#include "zcpfs_internal.h"

#include <string.h>

#define LOGIN_HDR	64	/* S_c, Y_c */
#define LOGIN_AROUND	(ZCPFS_LOGIN_OVERHEAD - 1)	/* dst, S_c, Y_c, tag */
#define BRANCH_HDR	32	/* Y_s */
#define FLAG_EMPTY	0x01	/* the typed password is empty */

/* G(pw) = ELL2(KDF("pake.gen", pw, S_c, S_s; 32)), section 19.1 */
static void generator(const uint8_t *password, size_t password_len, const uint8_t S_c[32],
		      const uint8_t S_s[32], uint8_t G[32])
{
	const struct zcpfs_part parts[] = {
		{ password, password_len }, { S_c, 32 }, { S_s, 32 },
	};
	uint8_t h[32];

	zcpfs_kdf("pake.gen", parts, 3, h, sizeof(h));
	zcpfs_ell2(G, h);
	zcpfs_wipe(h, sizeof(h));
}

/* k1 = KDF("pake1", ss, S_c, S_s, Y_c; 32);  AD = dst ‖ 0x07 ‖ S_c ‖ Y_c */
static void login_key(const uint8_t ss[32], const uint8_t S_c[32], const uint8_t S_s[32],
		      const uint8_t Y_c[32], uint8_t k1[32], uint8_t ad[66])
{
	const struct zcpfs_part parts[] = {
		{ ss, 32 }, { S_c, 32 }, { S_s, 32 }, { Y_c, 32 },
	};

	zcpfs_kdf("pake1", parts, 4, k1, 32);
	ad[0] = S_s[0];
	ad[1] = ZCPFS_PT_ANON;
	memcpy(ad + 2, S_c, 32);
	memcpy(ad + 34, Y_c, 32);
}

/* The key of a branch and the session keys; AD = dst ‖ src ‖ 0x01 ‖ Y_s */
static void branch_key(const uint8_t ss[32], const uint8_t K[32], const uint8_t S_c[32],
		       const uint8_t S_s[32], const uint8_t Y_c[32], const uint8_t Y_s[32],
		       uint8_t k2[32], uint8_t ad[35], struct zcpfs_keys *keys)
{
	const struct zcpfs_part parts[] = {
		{ ss, 32 }, { K, 32 }, { S_c, 32 }, { S_s, 32 }, { Y_c, 32 }, { Y_s, 32 },
	};
	uint8_t master[32];
	const struct zcpfs_part m[] = { { master, 32 } };

	zcpfs_kdf("pake.master", parts, 6, master, sizeof(master));
	zcpfs_kdf("pake2", m, 1, k2, 32);
	zcpfs_keys_from_master(master, keys);
	zcpfs_wipe(master, sizeof(master));
	ad[0] = S_c[0];
	ad[1] = S_s[0];
	ad[2] = ZCPFS_PT_RESPONSE;
	memcpy(ad + 3, Y_s, 32);
}

int zcpfs_login_make(struct zcpfs_drbg *g, const uint8_t ss[32], const uint8_t S_c[32],
		     const uint8_t S_s[32], const uint8_t *password, size_t password_len,
		     const uint8_t *login_plain, size_t plain_len,
		     struct zcpfs_login_out *out, uint8_t *body, size_t *body_len)
{
	static const uint8_t zero[32];
	uint8_t G[32], k1[32], ad[66];
	/* what is sealed: the flag byte, the login plaintext, perhaps a zero byte */
	size_t len = zcpfs_fit(ZCPFS_PT_ANON, 1 + plain_len, LOGIN_AROUND);

	if (len + LOGIN_AROUND > ZCPFS_MAX_PAYLOAD) {
		return ZCPFS_ERR_ARG;
	}
	if (zcpfs_ct_equal(ss, zero, 32)) {
		return ZCPFS_ERR_WEAK;
	}
	zcpfs_drbg_generate(g, out->y, 32);
	generator(password, password_len, S_c, S_s, G);
	/* an all-zero Y_c gives no branch on the server (19.3) */
	(void)zcpfs_dh(out->Y, out->y, G);

	memcpy(body, S_c, 32);
	memcpy(body + 32, out->Y, 32);
	body[LOGIN_HDR] = password_len == 0 ? FLAG_EMPTY : 0;
	if (plain_len > 0) {
		memcpy(body + LOGIN_HDR + 1, login_plain, plain_len);
	}
	if (len > 1 + plain_len) {
		body[LOGIN_HDR + 1 + plain_len] = 0;
	}
	login_key(ss, S_c, S_s, out->Y, k1, ad);
	zcpfs_aead_seal(k1, k1 + 16, ad, sizeof(ad), body + LOGIN_HDR, len, body + LOGIN_HDR);
	zcpfs_wipe(k1, sizeof(k1));
	zcpfs_wipe(G, sizeof(G));
	*body_len = LOGIN_HDR + len + ZCPFS_TAG_LEN;
	return ZCPFS_OK;
}

int zcpfs_login_open(const uint8_t a_s[32], const uint8_t S_s[32], const uint8_t *body,
		     size_t body_len, uint8_t S_c[32], uint8_t Y_c[32], uint8_t ss[32],
		     uint8_t *plain, size_t *plain_len, int *empty_password)
{
	uint8_t A_c[32], k1[32], ad[66];
	size_t len;
	int rc;

	if (body_len < LOGIN_HDR + 1 + ZCPFS_TAG_LEN || body_len + 1 > ZCPFS_MAX_PAYLOAD) {
		return ZCPFS_ERR_ARG;
	}
	len = body_len - LOGIN_HDR - ZCPFS_TAG_LEN;	/* the flag byte and what follows */
	memcpy(S_c, body, 32);
	memcpy(Y_c, body + 32, 32);
	zcpfs_ed_to_x(A_c, S_c);
	if (zcpfs_dh(ss, a_s, A_c) != ZCPFS_OK) {
		return ZCPFS_ERR_WEAK;
	}
	login_key(ss, S_c, S_s, Y_c, k1, ad);
	rc = zcpfs_aead_open(k1, k1 + 16, ad, sizeof(ad), body + LOGIN_HDR,
			     body_len - LOGIN_HDR, plain);
	zcpfs_wipe(k1, sizeof(k1));
	if (rc != ZCPFS_OK) {
		zcpfs_wipe(ss, 32);
		*plain_len = 0;
		return rc;
	}
	*empty_password = (plain[0] & FLAG_EMPTY) != 0;
	memmove(plain, plain + 1, len - 1);
	plain[len - 1] = 0;
	*plain_len = len - 1;
	return ZCPFS_OK;
}

int zcpfs_login_branch_make(struct zcpfs_drbg *g, const uint8_t ss[32],
			    const uint8_t S_c[32], const uint8_t S_s[32],
			    const uint8_t Y_c[32], const uint8_t *password,
			    size_t password_len,
			    const uint8_t response[ZCPFS_LOGIN_RESPONSE_LEN],
			    uint8_t branch[ZCPFS_BRANCH_LEN], struct zcpfs_keys *keys)
{
	uint8_t y[32], G[32], K[32], k2[32], ad[35];
	int rc = ZCPFS_ERR_WEAK;

	zcpfs_drbg_generate(g, y, sizeof(y));
	generator(password, password_len, S_c, S_s, G);
	(void)zcpfs_dh(branch, y, G);			/* Y_s */
	if (zcpfs_dh(K, y, Y_c) == ZCPFS_OK) {
		branch_key(ss, K, S_c, S_s, Y_c, branch, k2, ad, keys);
		zcpfs_aead_seal_tag(k2, k2 + 16, ad, sizeof(ad), response,
				    ZCPFS_LOGIN_RESPONSE_LEN, branch + BRANCH_HDR,
				    ZCPFS_BRANCH_TAG_LEN);
		zcpfs_wipe(k2, sizeof(k2));
		rc = ZCPFS_OK;
	}
	zcpfs_wipe(y, sizeof(y));
	zcpfs_wipe(G, sizeof(G));
	zcpfs_wipe(K, sizeof(K));
	return rc;
}

int zcpfs_login_branch_open(const uint8_t ss[32], const struct zcpfs_login_out *out,
			    const uint8_t S_c[32], const uint8_t S_s[32],
			    const uint8_t branch[ZCPFS_BRANCH_LEN],
			    uint8_t response[ZCPFS_LOGIN_RESPONSE_LEN],
			    struct zcpfs_keys *keys)
{
	const uint8_t *Y_s = branch;
	uint8_t K[32], k2[32], ad[35];
	int rc = ZCPFS_ERR_WEAK;

	if (zcpfs_dh(K, out->y, Y_s) == ZCPFS_OK) {
		branch_key(ss, K, S_c, S_s, out->Y, Y_s, k2, ad, keys);
		rc = zcpfs_aead_open_tag(k2, k2 + 16, ad, sizeof(ad), branch + BRANCH_HDR,
					 ZCPFS_BRANCH_LEN - BRANCH_HDR, response,
					 ZCPFS_BRANCH_TAG_LEN);
		zcpfs_wipe(k2, sizeof(k2));
	}
	if (rc != ZCPFS_OK) {
		zcpfs_wipe(keys, sizeof(*keys));
	}
	zcpfs_wipe(K, sizeof(K));
	return rc;
}

int zcpfs_login_reply_pack(uint8_t count, const uint8_t Y_c[32], const uint8_t *branches, uint8_t *reply,
			   size_t *reply_len)
{
	size_t len = 1 + ZCPFS_LOGIN_ECHO_LEN + (size_t)count * ZCPFS_BRANCH_LEN;
	/* dst and src precede the reply in the RESPONSE payload */
	size_t padded = zcpfs_fit(ZCPFS_PT_RESPONSE, len, 2);

	if (count < 1 || count > ZCPFS_LOGIN_MAX_BRANCHES) {
		return ZCPFS_ERR_ARG;
	}
	reply[0] = count;
	memcpy(reply + 1, Y_c, ZCPFS_LOGIN_ECHO_LEN);	/* names the login it answers (19.3) */
	memcpy(reply + 1 + ZCPFS_LOGIN_ECHO_LEN, branches, (size_t)count * ZCPFS_BRANCH_LEN);
	if (padded > len) {
		reply[len] = 0;
	}
	*reply_len = padded;
	return ZCPFS_OK;
}

int zcpfs_login_reply_unpack(const uint8_t *reply, size_t reply_len, const uint8_t **echo,
			     const uint8_t **branches)
{
	const uint8_t *b = reply + 1 + ZCPFS_LOGIN_ECHO_LEN;

	if (reply_len < 1 || reply[0] < 1 || reply[0] > ZCPFS_LOGIN_MAX_BRANCHES ||
	    reply_len < 1 + ZCPFS_LOGIN_ECHO_LEN + (size_t)reply[0] * ZCPFS_BRANCH_LEN) {
		return ZCPFS_ERR_ARG;
	}
	/* 19.3: two branches with one Y_s would be two tag tries under one key */
	for (int i = 1; i < reply[0]; i++) {
		for (int j = 0; j < i; j++) {
			if (memcmp(b + (size_t)i * ZCPFS_BRANCH_LEN, b + (size_t)j * ZCPFS_BRANCH_LEN, 32) == 0) {
				return ZCPFS_ERR_ARG;
			}
		}
	}
	*echo = reply + 1;
	*branches = b;
	return reply[0];
}
