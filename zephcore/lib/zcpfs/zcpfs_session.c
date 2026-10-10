/*
 * SPDX-License-Identifier: MIT
 * Section 9: one session. Each direction is a hash chain; every packet uses
 * the key of the next chain position and the position is never sent. The
 * receiver finds it by trying: the next position, then positions it stepped
 * over earlier, then up to ZCPFS_W - 1 positions further on.
 */

#include "zcpfs_internal.h"

#include <string.h>

/* STEP(ck) = KDF("step", ck; 80), cut into these fields. */
enum {
	O_CK = 0,	/* the next chain key */
	O_KEY = 32,
	O_NONCE = 48,
	O_ACKK = 64,
	STEP_LEN = 80,
};

static void step(const uint8_t ck[32], uint8_t o[STEP_LEN])
{
	uint8_t in[32];
	const struct zcpfs_part parts[] = { { in, 32 } };

	memcpy(in, ck, sizeof(in));		/* `ck` may point into `o` */
	zcpfs_kdf("step", parts, 1, o, STEP_LEN);
	zcpfs_wipe(in, sizeof(in));
}

/* ack4 = KDF("ack", ackk, C ‖ T8; 4), section 9.5 */
static void ack_value(const uint8_t ackk[16], const uint8_t *body, size_t body_len,
		      uint8_t ack4[4])
{
	const struct zcpfs_part parts[] = { { ackk, 16 }, { body, body_len } };

	zcpfs_kdf("ack", parts, 2, ack4, 4);
}

/* Append to a list of at most ZCPFS_S entries; the oldest makes room. */
static void push(struct zcpfs_skipped *list, uint8_t *n, const struct zcpfs_skipped *e)
{
	if (*n == ZCPFS_S) {
		memmove(&list[0], &list[1], (ZCPFS_S - 1) * sizeof(list[0]));
		(*n)--;
	}
	list[*n] = *e;
	(*n)++;
}

static void remove_at(struct zcpfs_session *s, uint8_t i)
{
	memmove(&s->skipped[i], &s->skipped[i + 1],
		(size_t)(s->n_skipped - i - 1) * sizeof(s->skipped[0]));
	s->n_skipped--;
	zcpfs_wipe(&s->skipped[s->n_skipped], sizeof(s->skipped[0]));
}

void zcpfs_session_tidy(struct zcpfs_session *s, uint32_t now)
{
	uint8_t i = 0;

	while (i < s->n_skipped) {
		if ((int32_t)(s->skipped[i].expiry - now) > 0) {
			i++;
		} else {
			remove_at(s, i);
		}
	}
}

void zcpfs_session_init(struct zcpfs_session *s, const uint8_t ck_s[32],
			const uint8_t ck_r[32], uint32_t now)
{
	memset(s, 0, sizeof(*s));
	memcpy(s->ck_s, ck_s, 32);
	memcpy(s->ck_r, ck_r, 32);
	s->t_start = now;
	s->t_last = now;
}

void zcpfs_seal(struct zcpfs_session *s, const uint8_t *ad, size_t ad_len,
		const uint8_t *plain, size_t plain_len, uint8_t *body, uint8_t ack4[4],
		uint32_t now)
{
	uint8_t o[STEP_LEN];

	step(s->ck_s, o);
	memcpy(s->ck_s, o + O_CK, 32);		/* before any ciphertext exists */
	zcpfs_aead_seal(o + O_KEY, o + O_NONCE, ad, ad_len, plain, plain_len, body);
	ack_value(o + O_ACKK, body, plain_len + ZCPFS_TAG_LEN, ack4);
	s->t_last = now;
	zcpfs_wipe(o, sizeof(o));
}

int zcpfs_open(struct zcpfs_session *s, const uint8_t *ad, size_t ad_len,
	       const uint8_t *body, size_t body_len, uint8_t *plain, uint8_t ack4[4],
	       uint32_t now)
{
	uint8_t o[STEP_LEN];
	struct zcpfs_skipped passed[ZCPFS_S];	/* the last positions stepped over */
	uint8_t n_passed = 0;
	const uint8_t *ackk = NULL;

	if (body_len < ZCPFS_TAG_LEN) {
		return ZCPFS_ERR_ARG;
	}
	zcpfs_session_tidy(s, now);

	/* the next position */
	step(s->ck_r, o);
	if (zcpfs_aead_open(o + O_KEY, o + O_NONCE, ad, ad_len, body, body_len, plain) ==
	    ZCPFS_OK) {
		memcpy(s->ck_r, o + O_CK, 32);
		ackk = o + O_ACKK;
		goto opened;
	}
	s->fails++;

	/* a position stepped over earlier; its key is used once */
	for (uint8_t i = 0; i < s->n_skipped; i++) {
		const struct zcpfs_skipped *k = &s->skipped[i];

		if (zcpfs_aead_open(k->key, k->nonce, ad, ad_len, body, body_len, plain) ==
		    ZCPFS_OK) {
			ack_value(k->ackk, body, body_len, ack4);
			remove_at(s, i);
			goto done;
		}
	}

	/* further on: nothing is kept unless a position opens */
	for (unsigned pos = 1; pos < ZCPFS_W; pos++) {
		struct zcpfs_skipped e;

		memcpy(e.key, o + O_KEY, 16);
		memcpy(e.nonce, o + O_NONCE, 16);
		memcpy(e.ackk, o + O_ACKK, 16);
		e.expiry = now + ZCPFS_T_SKIP;
		push(passed, &n_passed, &e);
		zcpfs_wipe(&e, sizeof(e));

		step(o + O_CK, o);
		if (zcpfs_aead_open(o + O_KEY, o + O_NONCE, ad, ad_len, body, body_len,
				    plain) == ZCPFS_OK) {
			for (uint8_t i = 0; i < n_passed; i++) {
				push(s->skipped, &s->n_skipped, &passed[i]);
			}
			memcpy(s->ck_r, o + O_CK, 32);
			ackk = o + O_ACKK;
			goto opened;
		}
	}
	zcpfs_wipe(o, sizeof(o));
	zcpfs_wipe(passed, sizeof(passed));
	return ZCPFS_ERR_AUTH;

opened:
	ack_value(ackk, body, body_len, ack4);
done:
	s->opened = 1;
	s->t_last = now;
	zcpfs_wipe(o, sizeof(o));
	zcpfs_wipe(passed, sizeof(passed));
	return ZCPFS_OK;
}
