/*
 * SPDX-License-Identifier: MIT
 * Sections 7.2 and 8.5 to 11: what a node keeps for a contact and what a send,
 * a received packet, an ACK and the passing of time do to it.
 *
 * Per contact there are three sessions: `current` seals and opens, `next` is a
 * responder's session the initiator has not confirmed yet, `previous` is what
 * `current` replaced. Only this file moves sessions between them.
 *
 * Every packet built here leaves through ops->send. It is flooded in exactly
 * one place, send_text(), and at most once per call (section 11).
 */

#include "zcpfs_internal.h"

#include <string.h>

#define HDR	2	/* dst, src */

enum {			/* indices of gap_until[] */
	GAP_START,	/* handshake 1, or probe started by a send: HS_RETRY steps */
	GAP_RESYNC,	/* T_RESYNC */
	GAP_HINT,	/* T_HINT */
	GAP_PROBE,	/* T_PROBE */
	GAP_CONFIRM,	/* T_CONFIRM */
};

/* How easily a record is given to another contact, lowest first. */
enum {
	RANK_FREE,
	RANK_IDLE,	/* no session, nothing held, no handshake under way */
	RANK_UNCONFIRMED, /* only a `next` nobody confirmed */
	RANK_LIVE,
};

/* ---- time ---------------------------------------------------------------- */

/* `t` is now or in the past. Holds across a wrap of the 32-bit clock. */
static int reached(uint32_t now, uint32_t t)
{
	return (int32_t)(now - t) >= 0;
}

/* The step of 10.4 for a send timeout, in seconds. */
static uint32_t step_of(uint32_t timeout_ms)
{
	uint32_t s = timeout_ms / 1000 + (timeout_ms % 1000 != 0);

	s += (s + 1) / 2;
	return s < ZCPFS_T_STEP_MIN ? ZCPFS_T_STEP_MIN : s;
}

static uint32_t step(const struct zcpfs_contact *c)
{
	return step_of(c->timeout_ms);
}

/* At most one event per `gap` seconds: 1 if this one may happen. */
static int gap_take(struct zcpfs_peer *p, unsigned which, uint32_t gap, uint32_t now)
{
	uint8_t bit = (uint8_t)(1u << which);

	if ((p->gap_armed & bit) != 0 && !reached(now, p->gap_until[which])) {
		return 0;
	}
	p->gap_armed |= bit;
	p->gap_until[which] = now + gap;
	return 1;
}

/* Token bucket: one token back every `period` seconds, `burst` at most. */
static int bucket_take(struct zcpfs_bucket *b, uint32_t per_period, uint32_t period,
		       uint32_t burst, uint32_t now)
{
	uint32_t periods = (now - b->at) / period;

	if (b->spent == 0 || periods >= burst || periods * per_period >= b->spent) {
		b->spent = 0;
		b->at = now;
	} else {
		b->spent = (uint8_t)(b->spent - periods * per_period);
		b->at += periods * period;
	}
	if (b->spent >= burst) {
		return 0;
	}
	b->spent++;
	return 1;
}

/* ---- records ------------------------------------------------------------- */

/* Section 4: a peer whose shared secret is all zero is never spoken zcpfs to. */
static int usable(const struct zcpfs_contact *c)
{
	static const uint8_t zero[32];

	return !zcpfs_ct_equal(c->ss, zero, 32);
}

static struct zcpfs_peer *find(struct zcpfs_node *n, const uint8_t S[32])
{
	for (size_t i = 0; i < n->n_peers; i++) {
		if (n->peers[i].used && memcmp(n->peers[i].S, S, 32) == 0) {
			return &n->peers[i];
		}
	}
	return NULL;
}

static int rank(const struct zcpfs_peer *p)
{
	if (!p->used) {
		return RANK_FREE;
	}
	if (p->current.used || p->previous.used || p->n_msgs > 0 || p->n_out > 0) {
		return RANK_LIVE;
	}
	return p->next_used ? RANK_UNCONFIRMED : RANK_IDLE;
}

/* The contact's record, or a new one that replaces the record of the lowest
 * rank not above `max_rank`, the least recently used among equals. NULL if no
 * record is that low. */
static struct zcpfs_peer *take(struct zcpfs_node *n, const uint8_t S[32], int max_rank,
			       uint32_t now)
{
	struct zcpfs_peer *best = find(n, S);
	int best_rank = 0;

	if (best != NULL) {
		return best;
	}
	for (size_t i = 0; i < n->n_peers; i++) {
		struct zcpfs_peer *q = &n->peers[i];
		int r = rank(q);

		if (r > max_rank) {
			continue;
		}
		if (best == NULL || r < best_rank ||
		    (r == best_rank && (int32_t)(q->t_used - best->t_used) < 0)) {
			best = q;
			best_rank = r;
		}
	}
	if (best != NULL) {
		zcpfs_wipe(best, sizeof(*best));
		memcpy(best->S, S, 32);
		best->used = 1;
		best->t_used = now;
	}
	return best;
}

/* ---- held and resent messages (10.1, 10.3) ------------------------------- */

static void msgs_clear(struct zcpfs_peer *p)
{
	zcpfs_wipe(p->msgs, sizeof(p->msgs));
	p->n_msgs = 0;
	p->msgs_sent = 0;
}

static void msg_remove(struct zcpfs_peer *p, uint8_t i)
{
	memmove(&p->msgs[i], &p->msgs[i + 1],
		(size_t)(p->n_msgs - i - 1) * sizeof(p->msgs[0]));
	p->n_msgs--;
	zcpfs_wipe(&p->msgs[p->n_msgs], sizeof(p->msgs[0]));
	if (p->n_msgs == 0) {
		p->msgs_sent = 0;
	}
}

/* Where the text of a plaintext of 9.3 ends: at its first zero byte, or at
 * the end. The text starts at byte 5. */
static size_t text_end(const uint8_t *plain, size_t len)
{
	size_t i = 5;

	while (i < len && plain[i] != 0) {
		i++;
	}
	return i;
}

/* The same message, whatever the attempt: same timestamp and text. */
static int same_message(const struct zcpfs_msg *m, const uint8_t *plain, size_t len)
{
	size_t end = text_end(plain, len);

	return memcmp(m->plain, plain, 4) == 0 && text_end(m->plain, m->len) == end &&
	       memcmp(m->plain + 5, plain + 5, end - 5) == 0;
}

/* Keep a message until a session exists, or until `due`. Only called without
 * `current`. */
static void hold(struct zcpfs_peer *p, const uint8_t *plain, size_t len, uint32_t token,
		 uint32_t due)
{
	struct zcpfs_msg *m;

	if (p->msgs_sent) {
		msgs_clear(p);		/* they belonged to a session that is gone */
	}
	for (uint8_t i = 0; i < p->n_msgs; i++) {
		if (same_message(&p->msgs[i], plain, len)) {
			msg_remove(p, i);	/* a retry replaces the earlier attempt */
			break;
		}
	}
	if (p->n_msgs == ZCPFS_N_HELD) {
		msg_remove(p, 0);
	}
	m = &p->msgs[p->n_msgs++];
	memcpy(m->plain, plain, len);
	m->len = (uint8_t)len;
	m->sends = 0;
	m->token = token;
	m->due = due;
}

/* ---- slots (9.1, 9.4) ---------------------------------------------------- */

static void drop_current(struct zcpfs_peer *p)
{
	zcpfs_wipe(&p->current, sizeof(p->current));
	if (p->msgs_sent) {
		msgs_clear(p);		/* 10.3: resends end with their session */
	}
}

static void drop_next(struct zcpfs_peer *p)
{
	zcpfs_wipe(&p->next, sizeof(p->next));
	p->next_used = 0;
}

static void drop_previous(struct zcpfs_peer *p)
{
	zcpfs_wipe(&p->previous, sizeof(p->previous));
	p->prev_timed = 0;
}

static void drop_outstanding(struct zcpfs_peer *p)
{
	zcpfs_wipe(p->out, sizeof(p->out));
	p->n_out = 0;
}

/* Rotate: `previous` becomes what `current` was, `current` the new session. */
static void make_current(struct zcpfs_peer *p, const struct zcpfs_session *s)
{
	if (p->msgs_sent) {
		msgs_clear(p);
	}
	p->previous = p->current;
	p->prev_timed = 0;
	zcpfs_wipe(&p->current, sizeof(p->current));
	p->current.s = *s;
	p->current.used = 1;
}

static int session_over(const struct zcpfs_session *s, uint32_t now)
{
	return now - s->t_last > ZCPFS_T_IDLE || now - s->t_start > ZCPFS_T_MAX ||
	       s->fails >= ZCPFS_F_MAX;
}

/* Wipe everything whose time has passed. */
static void expire(struct zcpfs_peer *p, uint32_t now)
{
	if (p->current.used && session_over(&p->current.s, now)) {
		drop_current(p);
	}
	if (p->next_used && session_over(&p->next, now)) {
		drop_next(p);
	}
	if (p->previous.used && (session_over(&p->previous.s, now) ||
				 (p->prev_timed && reached(now, p->prev_deadline)))) {
		drop_previous(p);
	}
	if (p->current.used) {
		zcpfs_session_tidy(&p->current.s, now);
	}
	if (p->next_used) {
		zcpfs_session_tidy(&p->next, now);
	}
	if (p->previous.used) {
		zcpfs_session_tidy(&p->previous.s, now);
	}
	if (p->n_out > 0 && reached(now, p->hs_until)) {
		drop_outstanding(p);	/* 8.2 */
	}
	if (!p->msgs_sent) {
		uint8_t i = 0;

		while (i < p->n_msgs) {
			if (reached(now, p->msgs[i].due)) {
				msg_remove(p, i);	/* 10.1: held long enough */
			} else {
				i++;
			}
		}
	}
}

/* ---- sending ------------------------------------------------------------- */

static void emit(struct zcpfs_node *n, const struct zcpfs_peer *p, uint8_t ptype,
		 uint8_t *payload, size_t len, int flood)
{
	payload[0] = p->S[0];
	payload[1] = n->S[0];
	n->ops->send(n->user, p->S, ptype, payload, len, flood);
}

/* Seal a plaintext under `current` and send it (9.2, 9.5). An empty plaintext
 * is a confirmation. Without a path the packet goes out only from the
 * application's send point, and then flooded (section 11). */
static void send_data(struct zcpfs_node *n, struct zcpfs_peer *p,
		      const struct zcpfs_contact *c, const uint8_t *plain, size_t len,
		      uint32_t token, int at_send_point, uint32_t now)
{
	uint8_t payload[ZCPFS_MAX_PAYLOAD] = { 0 };
	const uint8_t ad[3] = { p->S[0], n->S[0], ZCPFS_PT_TXT };
	size_t padded = zcpfs_fit(ZCPFS_PT_TXT, len, ZCPFS_DATA_OVERHEAD);	/* 9.3 */
	struct zcpfs_slot *cur = &p->current;
	uint8_t ack4[4];

	if (!c->has_path && !at_send_point) {
		return;
	}
	if (len > 0) {
		memcpy(payload + HDR, plain, len);
	}
	zcpfs_seal(&cur->s, ad, sizeof(ad), payload + HDR, padded, payload + HDR, ack4, now);
	if (len > 0) {
		if (cur->n_expect == ZCPFS_N_ACK) {
			memmove(&cur->expect[0], &cur->expect[1],
				(ZCPFS_N_ACK - 1) * sizeof(cur->expect[0]));
			cur->n_expect--;
		}
		memcpy(cur->expect[cur->n_expect].ack4, ack4, 4);
		cur->expect[cur->n_expect].token = token;
		cur->n_expect++;
		if (cur->unacked < UINT8_MAX) {
			cur->unacked++;
		}
	}
	emit(n, p, ZCPFS_PT_TXT, payload, padded + ZCPFS_DATA_OVERHEAD, !c->has_path);
}

/* `current` has just come up: send what was held, in order (8.4 step 4, 8.5).
 * Returns how many messages that was. */
static unsigned send_held(struct zcpfs_node *n, struct zcpfs_peer *p,
			  const struct zcpfs_contact *c, uint32_t now)
{
	for (uint8_t i = 0; i < p->n_msgs; i++) {
		struct zcpfs_msg *m = &p->msgs[i];

		send_data(n, p, c, m->plain, m->len, m->token, 0, now);
		m->sends = 0;
		m->due = now + ZCPFS_DATA_RETRY_STEPS * step(c);
	}
	p->msgs_sent = p->n_msgs > 0;
	p->hs_tries = 0;
	return p->n_msgs;
}

/* 10.3: send the messages that waited for the handshake again while they stay
 * unacknowledged. */
static void resend(struct zcpfs_node *n, struct zcpfs_peer *p, const struct zcpfs_contact *c,
		   uint32_t now)
{
	uint8_t i = 0;

	while (p->msgs_sent && i < p->n_msgs) {
		struct zcpfs_msg *m = &p->msgs[i];

		if (!reached(now, m->due)) {
			i++;
		} else if (m->sends >= ZCPFS_N_DATA_TRIES || !c->has_path) {
			msg_remove(p, i);
		} else {
			send_data(n, p, c, m->plain, m->len, m->token, 0, now);
			m->sends++;
			m->due = now + ZCPFS_DATA_RETRY_STEPS * step(c);
			i++;
		}
	}
}

/* 8.2. Direct only: without a path nothing is sent. */
static void send_hs1(struct zcpfs_node *n, struct zcpfs_peer *p,
		     const struct zcpfs_contact *c, uint32_t now)
{
	uint8_t payload[HDR + ZCPFS_HS_BODY_LEN];
	struct zcpfs_hs_out out;

	if (!c->has_path) {
		return;
	}
	if (zcpfs_hs1_make(n->drbg, c->ss, n->S, c->S, &out, payload + HDR) != ZCPFS_OK) {
		return;
	}
	if (p->path_unconfirmed) {
		n->ops->reciprocal_path(n->user, p->S);		/* section 11 */
	}
	if (p->n_out == ZCPFS_N_HS_TRIES) {
		memmove(&p->out[0], &p->out[1], (ZCPFS_N_HS_TRIES - 1) * sizeof(p->out[0]));
		p->n_out--;
	}
	p->out[p->n_out++] = out;
	zcpfs_wipe(&out, sizeof(out));
	p->hs_until = now + ZCPFS_HS_WAIT_STEPS * step(c);
	p->gap_armed |= 1u << GAP_START;	/* the retry interval counts from the newest */
	p->gap_until[GAP_START] = now + ZCPFS_HS_RETRY_STEPS * step(c);
	if (p->hs_tries < UINT8_MAX) {
		p->hs_tries++;
	}
	emit(n, p, ZCPFS_PT_REQ, payload, sizeof(payload), 0);
}

/* Section 11: the probe that is flooded in place of the message. */
static void send_probe(struct zcpfs_node *n, struct zcpfs_peer *p,
		       const struct zcpfs_contact *c)
{
	uint8_t payload[HDR + ZCPFS_PROBE_BODY_LEN];
	uint8_t nonce4[4];

	zcpfs_drbg_generate(n->drbg, nonce4, sizeof(nonce4));
	zcpfs_probe_make(c->ss, n->S, c->S, nonce4, payload + HDR);
	emit(n, p, ZCPFS_PT_REQ, payload, sizeof(payload), 1);
}

int zcpfs_send_text(struct zcpfs_node *n, const struct zcpfs_contact *c,
		    const uint8_t *plain, size_t len, uint32_t token, uint32_t now,
		    uint32_t *held_for)
{
	struct zcpfs_peer *p;

	*held_for = 0;
	if (!c->pinned || !usable(c)) {
		return ZCPFS_LEGACY;
	}
	if (len < 5 || zcpfs_fit(ZCPFS_PT_TXT, len, ZCPFS_DATA_OVERHEAD) > ZCPFS_TEXT_MAX) {
		return ZCPFS_ERR_ARG;
	}
	p = take(n, c->S, RANK_LIVE, now);
	if (p == NULL) {
		return ZCPFS_ERR_FULL;
	}
	p->t_used = now;
	expire(p, now);
	if (p->current.used && p->current.unacked >= ZCPFS_K_UNACKED) {
		drop_current(p);	/* 9.4: handshake before this send */
	}
	if (p->current.used) {
		send_data(n, p, c, plain, len, token, 1, now);
		return ZCPFS_SENT;
	}

	/* 10.1: held exactly as long as the application is told to wait */
	*held_for = ZCPFS_HELD_STEPS * step(c);
	hold(p, plain, len, token, now + *held_for);
	p->hs_tries = 0;		/* a new attempt, a new budget */
	if (!gap_take(p, GAP_START, ZCPFS_HS_RETRY_STEPS * step(c), now)) {
		p->hs_tries = 1;	/* one is under way; the timer repeats it */
	} else if (c->has_path) {
		send_hs1(n, p, c, now);
	} else {
		send_probe(n, p, c);
	}
	return ZCPFS_HELD;
}

/* ---- receiving ----------------------------------------------------------- */

/* 9.2, duplicates: 1 if this plaintext was delivered from the contact within
 * T_DEDUP; otherwise remembers it. */
static int seen_before(struct zcpfs_peer *p, const uint8_t *plain, size_t len, uint32_t now)
{
	const struct zcpfs_part parts[] = { { plain, len } };
	uint8_t hash[8];
	uint8_t kept = 0;

	zcpfs_kdf("dedup", parts, 1, hash, sizeof(hash));
	for (uint8_t i = 0; i < p->n_seen; i++) {
		if (!reached(now, p->seen[i].expiry)) {
			p->seen[kept++] = p->seen[i];
		}
	}
	p->n_seen = kept;
	for (uint8_t i = 0; i < p->n_seen; i++) {
		if (memcmp(p->seen[i].hash, hash, sizeof(hash)) == 0) {
			return 1;
		}
	}
	if (p->n_seen == ZCPFS_N_DEDUP) {
		memmove(&p->seen[0], &p->seen[1], (ZCPFS_N_DEDUP - 1) * sizeof(p->seen[0]));
		p->n_seen--;
	}
	memcpy(p->seen[p->n_seen].hash, hash, sizeof(hash));
	p->seen[p->n_seen].expiry = now + ZCPFS_T_DEDUP;
	p->n_seen++;
	return 0;
}

/* A data packet has opened into `out`: what the caller is to do with it. */
static int delivered(struct zcpfs_peer *p, size_t body_len, uint32_t now,
		     struct zcpfs_rx_out *out)
{
	p->t_used = now;
	out->len = body_len - ZCPFS_TAG_LEN;
	if (out->len == 0) {
		return ZCPFS_RX_CONSUMED;	/* a confirmation */
	}
	return seen_before(p, out->plain, out->len, now) ? ZCPFS_RX_DUPLICATE : ZCPFS_RX_TEXT;
}

/* A TXT_MSG against the three slots (9.2). ZCPFS_RX_NEXT if none opens it. */
static int rx_data(struct zcpfs_node *n, struct zcpfs_peer *p, const struct zcpfs_contact *c,
		   const uint8_t *payload, size_t len, uint32_t now, struct zcpfs_rx_out *out)
{
	const uint8_t ad[3] = { payload[0], payload[1], ZCPFS_PT_TXT };
	const uint8_t *body = payload + HDR;
	size_t body_len = len - HDR;

	if (p->current.used && zcpfs_open(&p->current.s, ad, sizeof(ad), body, body_len,
					  out->plain, out->ack4, now) == ZCPFS_OK) {
		p->current.unacked = 0;		/* 9.5: the peer still holds the session */
		if (p->previous.used && !p->prev_timed) {
			p->prev_timed = 1;	/* the peer has switched */
			p->prev_deadline = now + ZCPFS_T_PREV;
		}
	} else if (p->next_used && zcpfs_open(&p->next, ad, sizeof(ad), body, body_len,
					      out->plain, out->ack4, now) == ZCPFS_OK) {
		/* 8.5: the initiator confirmed */
		make_current(p, &p->next);
		drop_next(p);
		p->prev_timed = p->previous.used;
		p->prev_deadline = now + ZCPFS_T_PREV;
		if (!c->pinned) {
			n->ops->pin(n->user, p->S);
		}
		send_held(n, p, c, now);
	} else if (p->previous.used && zcpfs_open(&p->previous.s, ad, sizeof(ad), body, body_len,
						  out->plain, out->ack4, now) == ZCPFS_OK) {
		/* the peer has not switched yet: tell it again */
		if (p->current.used && !p->current.s.opened &&
		    gap_take(p, GAP_CONFIRM, ZCPFS_T_CONFIRM, now)) {
			send_data(n, p, c, NULL, 0, 0, 0, now);
		}
	} else {
		return ZCPFS_RX_NEXT;
	}

	return delivered(p, body_len, now, out);
}

/* 10.2: a text from a pinned contact that nothing opened. */
static void resync(struct zcpfs_node *n, struct zcpfs_peer *p, const struct zcpfs_contact *c,
		   uint32_t now)
{
	if (!c->has_path || (p != NULL && p->n_out > 0)) {
		return;
	}
	if (p == NULL) {
		p = take(n, c->S, RANK_IDLE, now);	/* nobody vouched for this packet */
	}
	if (p != NULL && gap_take(p, GAP_RESYNC, ZCPFS_T_RESYNC, now)) {
		send_hs1(n, p, c, now);
	}
}

/* 8.3 */
static int rx_hs1(struct zcpfs_node *n, const struct zcpfs_contact *c, const uint8_t *body,
		  int flood, uint32_t now)
{
	uint8_t payload[HDR + ZCPFS_HS_BODY_LEN];
	struct zcpfs_keys keys;
	struct zcpfs_peer *p;
	uint8_t es[32];

	if (flood) {
		return ZCPFS_RX_NEXT;		/* handshakes are direct only */
	}
	if (!bucket_take(&n->pre, ZCPFS_R_PRE_PER_SECOND, 1, ZCPFS_R_PRE_BURST, now)) {
		return ZCPFS_RX_CONSUMED;	/* step 2 */
	}
	if (zcpfs_hs1_verify(c->ss, n->a, c->S, n->S, body, es) != ZCPFS_OK) {
		return ZCPFS_RX_NEXT;		/* step 3; another contact's, perhaps */
	}
	if (!c->has_path) {
		goto done;			/* the reply would have to be flooded */
	}
	p = take(n, c->S, RANK_LIVE, now);
	if (p == NULL ||
	    !bucket_take(&p->hs_limit, 1, ZCPFS_R_HS_SECONDS, ZCPFS_R_HS_BURST, now)) {
		goto done;			/* step 4 */
	}
	if (p->n_out > 0) {			/* step 5: simultaneous open */
		if (memcmp(n->S, c->S, 32) < 0) {
			goto done;		/* the smaller key stays initiator */
		}
		drop_outstanding(p);
	}
	if (zcpfs_hs2_make(n->drbg, c->ss, es, c->S, n->S, body, payload + HDR, &keys) !=
	    ZCPFS_OK) {
		goto done;
	}
	zcpfs_drbg_mix(n->drbg, keys.feed, sizeof(keys.feed));
	zcpfs_session_init(&p->next, keys.ck_r2i, keys.ck_i2r, now);	/* step 7 */
	p->next_used = 1;
	p->t_used = now;
	emit(n, p, ZCPFS_PT_RESPONSE, payload, sizeof(payload), 0);
done:
	zcpfs_wipe(es, sizeof(es));
	zcpfs_wipe(&keys, sizeof(keys));
	return ZCPFS_RX_CONSUMED;
}

/* 8.4 steps 2 to 4: the node holds the keys as initiator, of a handshake or
 * of a login (19.4). */
static void become_initiator(struct zcpfs_node *n, struct zcpfs_peer *p,
			     const struct zcpfs_contact *c, const struct zcpfs_keys *keys,
			     int flood, uint32_t now)
{
	struct zcpfs_session s;

	zcpfs_drbg_mix(n->drbg, keys->feed, sizeof(keys->feed));
	zcpfs_session_init(&s, keys->ck_i2r, keys->ck_r2i, now);
	make_current(p, &s);			/* step 3 */
	drop_next(p);
	drop_outstanding(p);
	zcpfs_wipe(&s, sizeof(s));
	p->t_used = now;
	if (!flood) {
		p->path_unconfirmed = 0;	/* the peer answered over the path */
	}
	if (!c->pinned) {
		n->ops->pin(n->user, p->S);
	}
	if (send_held(n, p, c, now) == 0) {	/* step 4: the initiator sends first */
		send_data(n, p, c, NULL, 0, 0, 0, now);
	}
}

/* 8.4 */
static int rx_hs2(struct zcpfs_node *n, struct zcpfs_peer *p, const struct zcpfs_contact *c,
		  const uint8_t *body, int flood, uint32_t now)
{
	struct zcpfs_keys keys;
	uint8_t i;

	if (p == NULL) {
		return ZCPFS_RX_NEXT;
	}
	for (i = p->n_out; i > 0; i--) {	/* a reply to any attempt still waiting */
		if (zcpfs_hs2_process(c->ss, &p->out[i - 1], n->a, n->S, c->S, body, &keys) ==
		    ZCPFS_OK) {
			break;
		}
	}
	if (i == 0) {
		return ZCPFS_RX_NEXT;
	}

	become_initiator(n, p, c, &keys, flood, now);
	zcpfs_wipe(&keys, sizeof(keys));
	return ZCPFS_RX_CONSUMED;
}

/* Section 11 */
static int rx_probe(struct zcpfs_node *n, struct zcpfs_peer *p, const struct zcpfs_contact *c,
		    const uint8_t *body, uint32_t now)
{
	if (zcpfs_probe_verify(c->ss, c->S, n->S, body) != ZCPFS_OK) {
		return ZCPFS_RX_NEXT;
	}
	if (p == NULL) {
		p = take(n, c->S, RANK_IDLE, now);	/* a probe can be replayed */
	}
	if (p != NULL && !gap_take(p, GAP_PROBE, ZCPFS_T_PROBE, now)) {
		return ZCPFS_RX_CONSUMED;
	}
	return ZCPFS_RX_PROBE;
}

static int rx_offers(struct zcpfs_node *n, const uint8_t *S, const struct zcpfs_contact *c,
		     const uint8_t *payload, size_t len, uint32_t now, uint8_t who[32],
		     struct zcpfs_rx_out *out);

int zcpfs_rx(struct zcpfs_node *n, const struct zcpfs_contact *c, uint8_t ptype,
	     const uint8_t *payload, size_t len, int flood, uint32_t now,
	     struct zcpfs_rx_out *out)
{
	struct zcpfs_peer *p;

	memset(out, 0, sizeof(*out));
	if (len < 4 || payload[0] != n->S[0] || payload[1] != c->S[0] || !usable(c)) {
		return ZCPFS_RX_LEGACY;
	}
	p = find(n, c->S);
	if (p != NULL) {
		expire(p, now);
	}

	switch (ptype) {
	case ZCPFS_PT_TXT:
		if (p != NULL && len >= ZCPFS_DATA_OVERHEAD && len <= ZCPFS_MAX_PAYLOAD) {
			int r = rx_data(n, p, c, payload, len, now, out);

			if (r != ZCPFS_RX_NEXT) {
				return r;
			}
		}
		if (n->n_offers > 0 && len >= ZCPFS_DATA_OVERHEAD && len <= ZCPFS_MAX_PAYLOAD) {
			int r = rx_offers(n, c->S, c, payload, len, now, NULL, out);

			if (r != ZCPFS_RX_NEXT) {
				return r;	/* 19.5 */
			}
		}
		if (!c->pinned) {
			return ZCPFS_RX_LEGACY;
		}
		resync(n, p, c, now);
		return ZCPFS_RX_NEXT;
	case ZCPFS_PT_REQ:
		if (len == HDR + ZCPFS_HS_BODY_LEN) {
			return rx_hs1(n, c, payload + HDR, flood, now);
		}
		if (len == HDR + ZCPFS_PROBE_BODY_LEN) {
			return rx_probe(n, p, c, payload + HDR, now);
		}
		break;
	case ZCPFS_PT_RESPONSE:
		if (len == HDR + ZCPFS_HS_BODY_LEN) {
			return rx_hs2(n, p, c, payload + HDR, flood, now);
		}
		break;
	case ZCPFS_PT_PATH:
		break;
	default:
		return ZCPFS_RX_LEGACY;
	}

	/* a request, response or path return */
	if ((len - 4) % 16 != 0) {
		if (ptype == ZCPFS_PT_RESPONSE && n->login.used &&
		    zcpfs_rx_login_reply(n, c, payload + HDR, len - HDR, flood, now, out) ==
			    ZCPFS_RX_LOGIN) {
			return ZCPFS_RX_LOGIN;		/* 19.4 */
		}
		if (zcpfs_pair_open(c->ss, c->S, n->S, ptype, payload, len, out->plain,
				    &out->len) == ZCPFS_OK) {
			return ZCPFS_RX_PAIR;		/* section 18 */
		}
		return ZCPFS_RX_NEXT;
	}
	return c->pinned ? ZCPFS_RX_NEXT : ZCPFS_RX_LEGACY;
}

/* ---- 19: login ----------------------------------------------------------- */

static void offers_expire(struct zcpfs_node *n, uint32_t now)
{
	for (size_t i = 0; i < n->n_offers; i++) {
		if (n->offers[i].used && reached(now, n->offers[i].until)) {
			zcpfs_wipe(&n->offers[i], sizeof(n->offers[i]));
		}
	}
	if (n->login.used && reached(now, n->login.until)) {
		zcpfs_wipe(&n->login, sizeof(n->login));
	}
}

/* Where the offer of a login from this client goes: over its own waiting
 * offer, else into a free place. NULL means the login is not answered
 * (19.3 step 2): the client's own offer is less than a step old, or the
 * table is full, or the client is a stranger and the only place left
 * is the one kept for clients the server knows. An offer never makes room
 * for another; it leaves by confirmation or by time. */
static struct zcpfs_offer *offer_slot(struct zcpfs_node *n, const uint8_t S[32], int known,
				      uint32_t now)
{
	struct zcpfs_offer *free_one = NULL;
	size_t strangers = 0;

	for (size_t i = 0; i < n->n_offers; i++) {
		struct zcpfs_offer *o = &n->offers[i];

		if (!o->used) {
			if (free_one == NULL) {
				free_one = o;
			}
		} else if (memcmp(o->S, S, 32) == 0) {
			return reached(now, o->renew) ? o : NULL;
		} else if (!o->known) {
			strangers++;
		}
	}
	if (!known && n->n_offers > 1 && strangers >= n->n_offers - 1) {
		return NULL;
	}
	return free_one;
}

int zcpfs_login_send(struct zcpfs_node *n, const struct zcpfs_contact *c,
		     const uint8_t *password, size_t password_len,
		     const uint8_t *login_plain, size_t plain_len, uint32_t now,
		     uint32_t *wait_for)
{
	uint8_t payload[ZCPFS_MAX_PAYLOAD];
	size_t body_len;
	int rc;

	*wait_for = 0;
	if (!c->pinned || !usable(c)) {
		return ZCPFS_LEGACY;
	}
	zcpfs_wipe(&n->login, sizeof(n->login));
	rc = zcpfs_login_make(n->drbg, c->ss, n->S, c->S, password, password_len, login_plain,
			      plain_len, &n->login.out, payload + 1, &body_len);
	if (rc != ZCPFS_OK) {
		zcpfs_wipe(&n->login, sizeof(n->login));
		return rc;
	}
	*wait_for = ZCPFS_LOGIN_STEPS * step(c);
	memcpy(n->login.S, c->S, 32);
	n->login.until = now + *wait_for;
	n->login.used = 1;
	payload[0] = c->S[0];
	n->ops->send(n->user, c->S, ZCPFS_PT_ANON, payload, 1 + body_len, !c->has_path);
	return ZCPFS_SENT;
}

int zcpfs_rx_login_reply(struct zcpfs_node *n, const struct zcpfs_contact *c,
			 const uint8_t *reply, size_t reply_len, int flood, uint32_t now,
			 struct zcpfs_rx_out *out)
{
	const uint8_t *branches;
	struct zcpfs_keys keys;
	struct zcpfs_peer *p;
	int count;

	memset(out, 0, sizeof(*out));
	offers_expire(n, now);
	if (!n->login.used || memcmp(n->login.S, c->S, 32) != 0 || !usable(c) ||
	    reply_len > ZCPFS_MAX_PAYLOAD) {
		return ZCPFS_RX_NEXT;
	}
	count = zcpfs_login_reply_unpack(reply, reply_len, &branches);
	for (int j = 0; j < count; j++) {
		/* 19.4: a branch opens only if the server used the typed password */
		if (zcpfs_login_branch_open(c->ss, &n->login.out, n->S, c->S,
					    branches + (size_t)j * ZCPFS_BRANCH_LEN, out->plain,
					    &keys) != ZCPFS_OK) {
			continue;
		}
		p = take(n, c->S, RANK_LIVE, now);
		if (p == NULL) {
			break;
		}
		out->len = ZCPFS_LOGIN_RESPONSE_LEN;
		zcpfs_wipe(&n->login, sizeof(n->login));
		become_initiator(n, p, c, &keys, flood, now);
		zcpfs_wipe(&keys, sizeof(keys));
		return ZCPFS_RX_LOGIN;
	}
	zcpfs_wipe(&keys, sizeof(keys));
	memset(out, 0, sizeof(*out));
	return ZCPFS_RX_NEXT;
}

void zcpfs_node_serve(struct zcpfs_node *n, struct zcpfs_offer *offers, size_t count)
{
	n->offers = offers;
	n->n_offers = count;
	for (size_t i = 0; i < count; i++) {
		zcpfs_wipe(&offers[i], sizeof(offers[i]));
	}
}

int zcpfs_login_rx(struct zcpfs_node *n, const uint8_t *payload, size_t len, int known,
		   uint32_t now, struct zcpfs_login_req *req)
{
	int empty = 0;

	memset(req, 0, sizeof(*req));
	if (len < 35 || payload[0] != n->S[0]) {
		return ZCPFS_ERR_AUTH;
	}
	if ((len - 35) % 16 == 0) {
		return ZCPFS_LEGACY;		/* 19.3: the length tells the two apart */
	}
	if (len < ZCPFS_LOGIN_OVERHEAD) {
		return ZCPFS_ERR_AUTH;
	}
	offers_expire(n, now);
	/* The key in the packet is not yet proven, but it is the one the offer
	 * would be kept under, so a full table is known before any work. */
	if (offer_slot(n, payload + 1, known, now) == NULL) {
		return ZCPFS_ERR_FULL;
	}
	if (!bucket_take(&n->pre, ZCPFS_R_PRE_PER_SECOND, 1, ZCPFS_R_PRE_BURST, now)) {
		return ZCPFS_ERR_FULL;		/* step 1 */
	}
	if (zcpfs_login_open(n->a, n->S, payload + 1, len - 1, req->S_c, req->Y_c, req->ss,
			     req->plain, &req->plain_len, &empty) != ZCPFS_OK) {
		zcpfs_wipe(req, sizeof(*req));
		return ZCPFS_ERR_AUTH;
	}
	req->empty_password = empty != 0;
	req->known = known != 0;
	return ZCPFS_OK;
}

int zcpfs_login_answer(struct zcpfs_node *n, const struct zcpfs_login_req *req,
		       const struct zcpfs_branch *branches, size_t count,
		       uint32_t timeout_ms, uint32_t now, uint8_t *reply, size_t *reply_len)
{
	uint8_t packed[ZCPFS_LOGIN_MAX_BRANCHES * ZCPFS_BRANCH_LEN];
	struct zcpfs_offer *o;

	if (count < 1 || count > ZCPFS_LOGIN_MAX_BRANCHES) {
		return ZCPFS_ERR_ARG;
	}
	offers_expire(n, now);
	o = offer_slot(n, req->S_c, req->known, now);
	if (o == NULL) {
		return ZCPFS_ERR_FULL;		/* step 2 */
	}
	zcpfs_wipe(o, sizeof(*o));		/* a new login replaces the client's last */
	for (size_t j = 0; j < count; j++) {	/* step 3 */
		if (zcpfs_login_branch_make(n->drbg, req->ss, req->S_c, n->S, req->Y_c,
					    branches[j].password, branches[j].password_len,
					    branches[j].response,
					    packed + o->count * ZCPFS_BRANCH_LEN,
					    &o->keys[o->count]) == ZCPFS_OK) {
			o->ids[o->count] = branches[j].id;
			o->count++;
		}
	}
	if (o->count == 0) {
		zcpfs_wipe(o, sizeof(*o));
		return ZCPFS_ERR_WEAK;
	}
	memcpy(o->S, req->S_c, 32);		/* step 5 */
	o->until = now + ZCPFS_LOGIN_STEPS * step_of(timeout_ms);
	o->renew = now + step_of(timeout_ms);
	o->known = req->known;
	o->used = 1;
	return zcpfs_login_reply_pack(o->count, packed, reply, reply_len);
}

/* 19.5: a TXT_MSG against the offers made to the clients whose hash is its
 * `src`, or to the one client `S` if that is given. The first branch it opens
 * under becomes that client's `current` and the role is granted.
 * ZCPFS_RX_NEXT if none opens it. */
static int rx_offers(struct zcpfs_node *n, const uint8_t *S, const struct zcpfs_contact *c,
		     const uint8_t *payload, size_t len, uint32_t now, uint8_t who[32],
		     struct zcpfs_rx_out *out)
{
	const uint8_t ad[3] = { payload[0], payload[1], ZCPFS_PT_TXT };
	struct zcpfs_contact view;
	struct zcpfs_session s;
	struct zcpfs_peer *p;
	uint8_t id;

	offers_expire(n, now);
	for (size_t i = 0; i < n->n_offers; i++) {
		struct zcpfs_offer *o = &n->offers[i];

		if (!o->used || o->S[0] != payload[1] || (S != NULL && memcmp(o->S, S, 32) != 0)) {
			continue;
		}
		for (uint8_t j = 0; j < o->count; j++) {
			/* the client is the initiator of a login's session (19.3) */
			zcpfs_session_init(&s, o->keys[j].ck_r2i, o->keys[j].ck_i2r, now);
			if (zcpfs_open(&s, ad, sizeof(ad), payload + HDR, len - HDR, out->plain,
				       out->ack4, now) != ZCPFS_OK) {
				continue;
			}
			p = take(n, o->S, RANK_LIVE, now);
			if (p == NULL) {
				zcpfs_wipe(out, sizeof(*out));	/* nowhere to keep a session */
				break;
			}
			zcpfs_drbg_mix(n->drbg, o->keys[j].feed, sizeof(o->keys[j].feed));
			id = o->ids[j];
			if (who != NULL) {
				memcpy(who, o->S, 32);
			}
			zcpfs_wipe(o, sizeof(*o));	/* this and every other branch */
			make_current(p, &s);
			zcpfs_wipe(&s, sizeof(s));
			drop_next(p);
			p->prev_timed = p->previous.used;
			p->prev_deadline = now + ZCPFS_T_PREV;
			if (n->ops->granted != NULL) {
				n->ops->granted(n->user, p->S, id);
			}
			if (c == NULL && n->ops->contact(n->user, p->S, &view)) {
				c = &view;	/* the client `granted` has just made known */
			}
			if (c != NULL) {
				send_held(n, p, c, now);
			}
			return delivered(p, len - HDR, now, out);
		}
	}
	zcpfs_wipe(&s, sizeof(s));
	return ZCPFS_RX_NEXT;
}

int zcpfs_rx_offer(struct zcpfs_node *n, const uint8_t *payload, size_t len, uint32_t now,
		   uint8_t S[32], struct zcpfs_rx_out *out)
{
	memset(out, 0, sizeof(*out));
	if (len < ZCPFS_DATA_OVERHEAD || len > ZCPFS_MAX_PAYLOAD || payload[0] != n->S[0]) {
		return ZCPFS_RX_NEXT;
	}
	return rx_offers(n, NULL, NULL, payload, len, now, S, out);
}

/* ---- the other events ---------------------------------------------------- */

int zcpfs_on_ack(struct zcpfs_node *n, const uint8_t ack4[4], uint32_t *token, uint8_t S[32])
{
	/* 9.5: every `current` first, then every `previous` */
	for (int pass = 0; pass < 2; pass++) {
		for (size_t i = 0; i < n->n_peers; i++) {
			struct zcpfs_peer *p = &n->peers[i];
			struct zcpfs_slot *slot = pass == 0 ? &p->current : &p->previous;

			if (!p->used || !slot->used) {
				continue;
			}
			for (uint8_t k = 0; k < slot->n_expect; k++) {
				if (!zcpfs_ct_equal(slot->expect[k].ack4, ack4, 4)) {
					continue;
				}
				*token = slot->expect[k].token;
				memcpy(S, p->S, 32);
				memmove(&slot->expect[k], &slot->expect[k + 1],
					(size_t)(slot->n_expect - k - 1) * sizeof(slot->expect[0]));
				slot->n_expect--;
				if (pass == 0) {
					slot->unacked = 0;
					/* 10.3: delivered, so no more resends of it */
					for (uint8_t m = 0; p->msgs_sent && m < p->n_msgs; m++) {
						if (p->msgs[m].token == *token) {
							msg_remove(p, m);
							break;
						}
					}
				}
				return 1;
			}
		}
	}
	return 0;
}

void zcpfs_on_path(struct zcpfs_node *n, const struct zcpfs_contact *c, int flood,
		   uint32_t now)
{
	struct zcpfs_peer *p = find(n, c->S);

	if (p == NULL || !usable(c)) {
		return;
	}
	expire(p, now);
	p->path_unconfirmed = flood != 0;
	if (p->current.used || p->n_out > 0) {
		return;
	}
	/* the handshake that waited for a path */
	if ((c->pinned && p->n_msgs > 0) || p->hint_pending) {
		p->hint_pending = 0;
		send_hs1(n, p, c, now);
	}
}

void zcpfs_on_hint(struct zcpfs_node *n, const struct zcpfs_contact *c, uint32_t now)
{
	struct zcpfs_peer *p = find(n, c->S);

	if (c->pinned || !usable(c) || (p != NULL && (p->current.used || p->n_out > 0))) {
		return;
	}
	if (p == NULL) {
		p = take(n, c->S, RANK_IDLE, now);	/* a hint is weakly authenticated */
	}
	if (p == NULL || !gap_take(p, GAP_HINT, ZCPFS_T_HINT, now)) {
		return;
	}
	if (c->has_path) {
		send_hs1(n, p, c, now);
	} else {
		p->hint_pending = 1;
	}
}

/* ---- timers -------------------------------------------------------------- */

static void sooner(uint32_t *wait, uint32_t now, uint32_t t)
{
	uint32_t d = reached(now, t) ? 0 : t - now;

	if (d < *wait) {
		*wait = d;
	}
}

static void sooner_session(uint32_t *wait, uint32_t now, const struct zcpfs_session *s)
{
	sooner(wait, now, s->t_last + ZCPFS_T_IDLE + 1);
	sooner(wait, now, s->t_start + ZCPFS_T_MAX + 1);
	for (uint8_t i = 0; i < s->n_skipped; i++) {
		sooner(wait, now, s->skipped[i].expiry);
	}
}

/* 10.1: while messages are held and a path exists, a new handshake 1 every
 * HS_RETRY steps, N_HS_TRIES in all per send. */
static int hs_repeat_wanted(const struct zcpfs_peer *p, const struct zcpfs_contact *c)
{
	return p->n_msgs > 0 && !p->msgs_sent && !p->current.used && c->has_path &&
	       p->hs_tries > 0 && p->hs_tries < ZCPFS_N_HS_TRIES;
}

uint32_t zcpfs_tick(struct zcpfs_node *n, uint32_t now)
{
	uint32_t wait = UINT32_MAX;

	offers_expire(n, now);
	if (n->login.used) {
		sooner(&wait, now, n->login.until);
	}
	for (size_t i = 0; i < n->n_offers; i++) {
		if (n->offers[i].used) {
			sooner(&wait, now, n->offers[i].until);
		}
	}
	for (size_t i = 0; i < n->n_peers; i++) {
		struct zcpfs_peer *p = &n->peers[i];
		struct zcpfs_contact c;

		if (!p->used) {
			continue;
		}
		if (!n->ops->contact(n->user, p->S, &c)) {
			zcpfs_wipe(p, sizeof(*p));
			continue;
		}
		expire(p, now);
		if (hs_repeat_wanted(p, &c) &&
		    gap_take(p, GAP_START, ZCPFS_HS_RETRY_STEPS * step(&c), now)) {
			send_hs1(n, p, &c, now);
		}
		resend(n, p, &c, now);

		/* when this record next needs the timer */
		if (p->current.used) {
			sooner_session(&wait, now, &p->current.s);
		}
		if (p->next_used) {
			sooner_session(&wait, now, &p->next);
		}
		if (p->previous.used) {
			sooner_session(&wait, now, &p->previous.s);
			if (p->prev_timed) {
				sooner(&wait, now, p->prev_deadline);
			}
		}
		if (p->n_out > 0) {
			sooner(&wait, now, p->hs_until);
		}
		for (uint8_t k = 0; k < p->n_msgs; k++) {
			sooner(&wait, now, p->msgs[k].due);
		}
		if (hs_repeat_wanted(p, &c)) {
			sooner(&wait, now, p->gap_until[GAP_START]);
		}
	}
	return wait;
}

void zcpfs_forget(struct zcpfs_node *n, const uint8_t S[32])
{
	struct zcpfs_peer *p = find(n, S);

	if (p != NULL) {
		zcpfs_wipe(p, sizeof(*p));
	}
	if (n->login.used && memcmp(n->login.S, S, 32) == 0) {
		zcpfs_wipe(&n->login, sizeof(n->login));
	}
	for (size_t i = 0; i < n->n_offers; i++) {
		if (n->offers[i].used && memcmp(n->offers[i].S, S, 32) == 0) {
			zcpfs_wipe(&n->offers[i], sizeof(n->offers[i]));
		}
	}
}

void zcpfs_node_init(struct zcpfs_node *n, const uint8_t S[32], const uint8_t a[32],
		     struct zcpfs_drbg *drbg, struct zcpfs_peer *peers, size_t n_peers,
		     const struct zcpfs_ops *ops, void *user)
{
	memset(n, 0, sizeof(*n));
	n->S = S;
	n->a = a;
	n->drbg = drbg;
	n->peers = peers;
	n->n_peers = n_peers;
	n->ops = ops;
	n->user = user;
	for (size_t i = 0; i < n_peers; i++) {
		zcpfs_wipe(&peers[i], sizeof(peers[i]));
	}
}
