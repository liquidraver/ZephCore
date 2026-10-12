/*
 * SPDX-License-Identifier: MIT
 * zcpfs: the cryptographic core of ../SPEC.md.
 *
 * Portable C99. No heap, no I/O, no clock, no logging: every function works on
 * buffers and state the caller owns, and the caller passes the time. Section
 * numbers in comments refer to SPEC.md.
 */

#ifndef ZCPFS_H
#define ZCPFS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZCPFS_TAG_LEN		8
#define ZCPFS_MAX_PAYLOAD	176

/* Protocol parameters (section 12). */
#ifndef ZCPFS_W
#define ZCPFS_W			16	/* receive window, chain positions tried */
#endif
#ifndef ZCPFS_S
#define ZCPFS_S			4	/* skipped keys kept per session */
#endif
#ifndef ZCPFS_T_SKIP
#define ZCPFS_T_SKIP		120	/* seconds a skipped key is kept */
#endif

/* MeshCore payload type codes the envelopes are bound to. */
#define ZCPFS_PT_REQ		0x00
#define ZCPFS_PT_RESPONSE	0x01
#define ZCPFS_PT_TXT		0x02
#define ZCPFS_PT_GRP_TXT		0x05
#define ZCPFS_PT_GRP_DATA	0x06
#define ZCPFS_PT_ANON		0x07
#define ZCPFS_PT_PATH		0x08

/* Envelope sizes: what each format adds around the plaintext. */
#define ZCPFS_DATA_OVERHEAD	10	/* dst, src, tag */
#define ZCPFS_PAIR_OVERHEAD	16	/* dst, src, n6, tag */
#define ZCPFS_CHAN_OVERHEAD	15	/* channel hash, n6, tag */
#define ZCPFS_CHAN_MAX_PLAIN	(ZCPFS_MAX_PAYLOAD - ZCPFS_CHAN_OVERHEAD)
#define ZCPFS_LOGIN_OVERHEAD	74	/* dst, S_c, Y_c, a flag byte, tag */
#define ZCPFS_LOGIN_RESPONSE_LEN 6	/* the login response inside a branch */
#define ZCPFS_BRANCH_TAG_LEN	4
#define ZCPFS_BRANCH_LEN	42	/* Y_s, response, tag */
#define ZCPFS_LOGIN_MAX_BRANCHES 2

#define ZCPFS_OK		0
#define ZCPFS_ERR_AUTH		(-1)	/* tag did not verify, or nothing opened */
#define ZCPFS_ERR_ARG		(-2)	/* a length is out of range */
#define ZCPFS_ERR_WEAK		(-3)	/* a Diffie-Hellman result was all zero */
#define ZCPFS_ERR_FULL		(-4)	/* no contact record could be had */

/* ---- 3: helpers ---------------------------------------------------------- */

struct zcpfs_part {
	const uint8_t *data;
	size_t len;
};

/* KDF(label, parts; out_len), section 3.1. `label` is one of the constant
 * labels of section 12, at most 24 characters. */
void zcpfs_kdf(const char *label, const struct zcpfs_part *parts, size_t n_parts,
	       uint8_t *out, size_t out_len);

/* AUTH(k, ad), section 3.2. */
void zcpfs_auth(const uint8_t k[32], const uint8_t *ad, size_t ad_len,
		uint8_t tag[ZCPFS_TAG_LEN]);

/* 1 if the buffers are equal, 0 if not; time does not depend on the content. */
int zcpfs_ct_equal(const uint8_t *a, const uint8_t *b, size_t len);

void zcpfs_wipe(void *p, size_t len);

/* Length of a plaintext of `plain_len` bytes after the zero byte section 7
 * may require, for a packet of type `ptype` whose envelope adds `overhead`
 * bytes. At most one byte longer than plain_len. */
size_t zcpfs_fit(uint8_t ptype, size_t plain_len, size_t overhead);

/* ---- 5: random numbers --------------------------------------------------- */

struct zcpfs_drbg {
	uint8_t k[32];
};

void zcpfs_drbg_seed(struct zcpfs_drbg *g, const uint8_t hw[32], const uint8_t saved[32]);
void zcpfs_drbg_generate(struct zcpfs_drbg *g, uint8_t *out, size_t len);
void zcpfs_drbg_mix(struct zcpfs_drbg *g, const uint8_t *x, size_t len);

/* ---- 8: handshake -------------------------------------------------------- */

#define ZCPFS_HS_BODY_LEN	40	/* ephemeral key and tag; follows dst, src */

struct zcpfs_hs_out {			/* an outstanding handshake 1 */
	uint8_t e[32];
	uint8_t E[32];
	uint8_t es[32];
};

struct zcpfs_keys {			/* what a completed handshake or login yields */
	uint8_t ck_i2r[32];
	uint8_t ck_r2i[32];
	uint8_t feed[32];		/* for zcpfs_drbg_mix */
};

/* Initiator, 8.2. `ss` is the MeshCore shared secret of the pair; S_i and S_r
 * are the Ed25519 identity keys of initiator and responder. */
int zcpfs_hs1_make(struct zcpfs_drbg *g, const uint8_t ss[32], const uint8_t S_i[32],
		   const uint8_t S_r[32], struct zcpfs_hs_out *out,
		   uint8_t body[ZCPFS_HS_BODY_LEN]);

/* Responder, 8.3 step 3. `a_r` is the responder's X25519 scalar. */
int zcpfs_hs1_verify(const uint8_t ss[32], const uint8_t a_r[32], const uint8_t S_i[32],
		     const uint8_t S_r[32], const uint8_t body[ZCPFS_HS_BODY_LEN],
		     uint8_t es[32]);

/* Responder, 8.3 step 6. */
int zcpfs_hs2_make(struct zcpfs_drbg *g, const uint8_t ss[32], const uint8_t es[32],
		   const uint8_t S_i[32], const uint8_t S_r[32], const uint8_t E_i[32],
		   uint8_t body[ZCPFS_HS_BODY_LEN], struct zcpfs_keys *keys);

/* Initiator, 8.4. `a_i` is the initiator's X25519 scalar. */
int zcpfs_hs2_process(const uint8_t ss[32], const struct zcpfs_hs_out *out,
		      const uint8_t a_i[32], const uint8_t S_i[32], const uint8_t S_r[32],
		      const uint8_t body[ZCPFS_HS_BODY_LEN], struct zcpfs_keys *keys);

/* ---- 9: session ---------------------------------------------------------- */

struct zcpfs_skipped {
	uint8_t key[16];
	uint8_t nonce[16];
	uint8_t ackk[16];
	uint32_t expiry;
};

struct zcpfs_session {
	uint8_t ck_s[32];
	uint8_t ck_r[32];
	struct zcpfs_skipped skipped[ZCPFS_S];	/* oldest first */
	uint8_t n_skipped;
	uint8_t opened;				/* a packet has opened under it */
	uint32_t fails;
	uint32_t t_start;
	uint32_t t_last;
};

void zcpfs_session_init(struct zcpfs_session *s, const uint8_t ck_s[32],
			const uint8_t ck_r[32], uint32_t now);

/* Seal `plain` (9.2). `body` receives plain_len + ZCPFS_TAG_LEN bytes: the
 * ciphertext and the tag, to follow dst and src; it may start where `plain`
 * does. `ack4` receives the ACK value the sender expects. The chain is stepped
 * before anything else. */
void zcpfs_seal(struct zcpfs_session *s, const uint8_t *ad, size_t ad_len,
		const uint8_t *plain, size_t plain_len, uint8_t *body, uint8_t ack4[4],
		uint32_t now);

/* Open a received body (9.2). `plain` must hold body_len - ZCPFS_TAG_LEN
 * bytes. Returns ZCPFS_OK and commits the new chain position, or
 * ZCPFS_ERR_AUTH and changes nothing but `fails`. */
int zcpfs_open(struct zcpfs_session *s, const uint8_t *ad, size_t ad_len,
	       const uint8_t *body, size_t body_len, uint8_t *plain, uint8_t ack4[4],
	       uint32_t now);

/* Wipe the skipped keys whose time has passed. zcpfs_open does this itself;
 * a timer calls it so that none outlives ZCPFS_T_SKIP. */
void zcpfs_session_tidy(struct zcpfs_session *s, uint32_t now);

/* ---- 11, 17, 18: stateless packets --------------------------------------- */

#define ZCPFS_PROBE_BODY_LEN	12	/* nonce4 and tag; follows dst, src */

void zcpfs_probe_make(const uint8_t ss[32], const uint8_t S_sender[32],
		      const uint8_t S_receiver[32], const uint8_t nonce4[4],
		      uint8_t body[ZCPFS_PROBE_BODY_LEN]);
int zcpfs_probe_verify(const uint8_t ss[32], const uint8_t S_sender[32],
		       const uint8_t S_receiver[32], const uint8_t body[ZCPFS_PROBE_BODY_LEN]);

/* Channel packet (17). `secret` is the 32-byte channel secret field,
 * `chan_hash` MeshCore's 1-byte channel hash. `payload` must hold
 * plain_len + ZCPFS_CHAN_OVERHEAD + 1 bytes; its length is returned in
 * *payload_len. plain_len may not exceed ZCPFS_CHAN_MAX_PLAIN. */
int zcpfs_chan_seal(const uint8_t secret[32], uint8_t chan_hash, uint8_t ptype,
		    const uint8_t n6[6], const uint8_t *plain, size_t plain_len,
		    uint8_t *payload, size_t *payload_len);

/* `plain` must hold payload_len - ZCPFS_CHAN_OVERHEAD bytes. */
int zcpfs_chan_open(const uint8_t secret[32], uint8_t chan_hash, uint8_t ptype,
		    const uint8_t *payload, size_t payload_len, uint8_t *plain,
		    size_t *plain_len);

/* Request, response or path return (18). `payload` must hold
 * plain_len + ZCPFS_PAIR_OVERHEAD + 1 bytes. */
int zcpfs_pair_seal(const uint8_t ss[32], const uint8_t S_sender[32],
		    const uint8_t S_receiver[32], uint8_t ptype, const uint8_t n6[6],
		    const uint8_t *plain, size_t plain_len, uint8_t *payload,
		    size_t *payload_len);

/* `plain` must hold payload_len - ZCPFS_PAIR_OVERHEAD bytes. */
int zcpfs_pair_open(const uint8_t ss[32], const uint8_t S_sender[32],
		    const uint8_t S_receiver[32], uint8_t ptype, const uint8_t *payload,
		    size_t payload_len, uint8_t *plain, size_t *plain_len);

/* ---- 19: login ----------------------------------------------------------- */

struct zcpfs_login_out {		/* an outstanding login */
	uint8_t y[32];
	uint8_t Y[32];
};

/* Client, 19.2. `login_plain` is the legacy login plaintext without the
 * password. `body` (S_c, Y_c, ciphertext, tag; follows dst) must hold
 * plain_len + ZCPFS_LOGIN_OVERHEAD bytes. The packet says of the password
 * only whether it is empty. */
int zcpfs_login_make(struct zcpfs_drbg *g, const uint8_t ss[32], const uint8_t S_c[32],
		     const uint8_t S_s[32], const uint8_t *password, size_t password_len,
		     const uint8_t *login_plain, size_t plain_len,
		     struct zcpfs_login_out *out, uint8_t *body, size_t *body_len);

/* Server, 19.3 step 1. Outputs the client's key, its point, the shared secret,
 * the login plaintext (`plain` must hold body_len - 72 bytes) and whether the
 * client typed an empty password. */
int zcpfs_login_open(const uint8_t a_s[32], const uint8_t S_s[32], const uint8_t *body,
		     size_t body_len, uint8_t S_c[32], uint8_t Y_c[32], uint8_t ss[32],
		     uint8_t *plain, size_t *plain_len, int *empty_password);

/* Server, 19.3 step 3: one branch for one password it would accept. */
int zcpfs_login_branch_make(struct zcpfs_drbg *g, const uint8_t ss[32],
			    const uint8_t S_c[32], const uint8_t S_s[32],
			    const uint8_t Y_c[32], const uint8_t *password,
			    size_t password_len,
			    const uint8_t response[ZCPFS_LOGIN_RESPONSE_LEN],
			    uint8_t branch[ZCPFS_BRANCH_LEN], struct zcpfs_keys *keys);

/* Client, 19.4: opens only if the server used the same password, or by a
 * chance of 2^-32, which costs a login that the server then does not confirm. */
int zcpfs_login_branch_open(const uint8_t ss[32], const struct zcpfs_login_out *out,
			    const uint8_t S_c[32], const uint8_t S_s[32],
			    const uint8_t branch[ZCPFS_BRANCH_LEN],
			    uint8_t response[ZCPFS_LOGIN_RESPONSE_LEN],
			    struct zcpfs_keys *keys);

/* The reply of 19.3 step 4, after dst and src: count, the branches back to
 * back, zero bytes. `reply` must hold 1 + count * ZCPFS_BRANCH_LEN + 1 bytes. */
int zcpfs_login_reply_pack(uint8_t count, const uint8_t *branches, uint8_t *reply,
			   size_t *reply_len);

/* Returns the number of branches or ZCPFS_ERR_ARG; *branches points into
 * `reply`. */
int zcpfs_login_reply_unpack(const uint8_t *reply, size_t reply_len, const uint8_t **branches);

/* ---- 7.2, 8.5, 9 to 11: the sessions of each contact ---------------------
 *
 * zcpfs_peer.c. Per active contact the engine keeps the three session slots
 * and what sections 9 to 11 attach to them, and decides what a send, a
 * received packet, an ACK and the passing of time do. The caller keeps the
 * contacts, builds and routes packets, answers with ACKs and path returns, and
 * runs legacy processing.
 *
 * All times are uptime seconds. Nothing here is reentrant: a callback must not
 * call back into the engine.
 */

/* Parameters that size a record (section 12). */
#ifndef ZCPFS_N_ACK
#define ZCPFS_N_ACK		16	/* expected ACK values kept per session */
#endif
#ifndef ZCPFS_N_HS_TRIES
#define ZCPFS_N_HS_TRIES	3	/* handshake 1 attempts kept, and sent per send */
#endif
#ifndef ZCPFS_N_HELD
#define ZCPFS_N_HELD		4	/* messages held while a session comes up */
#endif
#ifndef ZCPFS_N_DEDUP
#define ZCPFS_N_DEDUP		16	/* delivered plaintexts remembered */
#endif

/* The other parameters of section 12. */
#define ZCPFS_T_IDLE		86400U	/* idle time that ends a session */
#define ZCPFS_T_MAX		604800U	/* age that ends a session */
#define ZCPFS_T_PREV		60	/* `previous` kept after the peer switched */
#define ZCPFS_T_CONFIRM		5	/* between repeated confirmations */
#define ZCPFS_K_UNACKED		8	/* unacknowledged sends before a new handshake */
#define ZCPFS_F_MAX		1048576U	/* 2^20 failed decryptions end a session */
#define ZCPFS_N_DATA_TRIES	2	/* resends of a message that was held */
#define ZCPFS_T_DEDUP		120	/* a delivered plaintext is remembered */
#define ZCPFS_T_RESYNC		600	/* between receiver-started handshakes */
#define ZCPFS_T_HINT		86400U	/* between hint-started handshakes */
#define ZCPFS_T_PROBE		15	/* between answered probes */
#define ZCPFS_R_PRE_PER_SECOND	4	/* handshake 1 verifications, node-wide */
#define ZCPFS_R_PRE_BURST	16
#define ZCPFS_R_HS_SECONDS	10	/* one verified handshake 1 acted on per contact */
#define ZCPFS_R_HS_BURST	3	/* per this many seconds: 3 per 30 s */

/* The intervals of 10.4 are counted in steps. A step is one and a half times
 * the caller's own send timeout towards the contact (zcpfs_contact.timeout_ms),
 * in whole seconds, and never less than ZCPFS_T_STEP_MIN. */
#define ZCPFS_T_STEP_MIN	5
#define ZCPFS_HS_RETRY_STEPS	1	/* between handshake 1 messages (or probes) */
#define ZCPFS_HS_WAIT_STEPS	2	/* own handshake 1 attempts wait for a reply */
#define ZCPFS_HELD_STEPS	(ZCPFS_N_HS_TRIES + 1) /* a message waits for a session */
#define ZCPFS_DATA_RETRY_STEPS	1	/* between resends of a message that was held */
#define ZCPFS_LOGIN_STEPS	2	/* a login waits for its reply, an offer to be confirmed */

/* Longest text plaintext of 9.3, with the alignment byte. */
#define ZCPFS_TEXT_MAX		(ZCPFS_MAX_PAYLOAD - ZCPFS_DATA_OVERHEAD)

/* A contact as the caller knows it, valid for the duration of a call. */
struct zcpfs_contact {
	const uint8_t *S;		/* Ed25519 identity key, 32 bytes */
	const uint8_t *ss;		/* MeshCore shared secret, 32 bytes (section 4) */
	uint8_t pinned;			/* 6.2 */
	uint8_t has_path;		/* a direct path to it is known */
	/* How long the caller would wait for the answer to a 42-byte packet sent
	 * to this contact now: over its path, or flooded if there is none. */
	uint32_t timeout_ms;
};

struct zcpfs_ops {
	/* Transmit a packet the engine built. `flood` is nonzero only inside
	 * zcpfs_send_text, at most once per call (section 11). */
	void (*send)(void *user, const uint8_t S[32], uint8_t ptype, const uint8_t *payload,
		     size_t len, int flood);
	/* Send our path to the contact again, direct (section 11). */
	void (*reciprocal_path)(void *user, const uint8_t S[32]);
	/* A handshake with the contact completed: store the pin (8.6). */
	void (*pin)(void *user, const uint8_t S[32]);
	/* Fill `c` for the contact with this key; 0 if there is none any more. */
	int (*contact)(void *user, const uint8_t S[32], struct zcpfs_contact *c);
	/* Server: the client proved the password of the branch the caller named
	 * `id` (19.5). Grant that role and store the pin; from now on `contact`
	 * has to know this client. May be NULL on a node that serves no logins. */
	void (*granted)(void *user, const uint8_t S[32], uint8_t id);
};

struct zcpfs_bucket {			/* a rate limit; all zero is a full bucket */
	uint32_t at;			/* refill is counted up to here */
	uint8_t spent;
};

struct zcpfs_expect {			/* 9.5 */
	uint8_t ack4[4];
	uint32_t token;
};

struct zcpfs_slot {			/* a session that has been `current` */
	struct zcpfs_session s;
	struct zcpfs_expect expect[ZCPFS_N_ACK];	/* oldest first */
	uint8_t n_expect;
	uint8_t unacked;
	uint8_t used;
};

struct zcpfs_msg {			/* 10.1, 10.3 */
	uint8_t plain[ZCPFS_TEXT_MAX];
	uint8_t len;
	uint8_t sends;			/* resends made */
	uint32_t token;
	uint32_t due;			/* held: when it is dropped; sent: the next resend */
};

struct zcpfs_seen {			/* 9.2, duplicates */
	uint8_t hash[8];
	uint32_t expiry;
};

#define ZCPFS_N_GAPS		5

/* What the node keeps for one contact it has sessions or business with. RAM
 * only; found by the contact's key. */
struct zcpfs_peer {
	uint8_t S[32];
	uint8_t used;
	uint8_t next_used;
	uint8_t path_unconfirmed;	/* the peer may lack our reciprocal path (11) */
	uint8_t hint_pending;		/* 6.3: handshake once a path exists */
	uint8_t prev_timed;		/* prev_deadline is running */
	uint8_t hs_tries;		/* handshake 1 messages for the current send */
	uint8_t n_out;
	uint8_t n_msgs;
	uint8_t msgs_sent;		/* msgs[] were sent under `current`; else held */
	uint8_t n_seen;
	uint8_t gap_armed;		/* one bit per gap_until[] */
	uint32_t gap_until[ZCPFS_N_GAPS];
	uint32_t prev_deadline;
	uint32_t hs_until;		/* when out[] is wiped (8.2) */
	uint32_t t_used;		/* for eviction */
	struct zcpfs_bucket hs_limit;	/* R_HS */
	struct zcpfs_slot current;
	struct zcpfs_slot previous;
	struct zcpfs_session next;	/* responder: not yet confirmed (8.3) */
	struct zcpfs_hs_out out[ZCPFS_N_HS_TRIES];	/* newest last (8.2) */
	struct zcpfs_msg msgs[ZCPFS_N_HELD];		/* oldest first */
	struct zcpfs_seen seen[ZCPFS_N_DEDUP];		/* oldest first */
};

struct zcpfs_login_wait {		/* client: the one login awaiting its reply (19.2) */
	uint8_t S[32];			/* the server */
	struct zcpfs_login_out out;
	uint32_t until;
	uint8_t used;
};

struct zcpfs_offer {			/* server: a login answered, not yet confirmed (19.3) */
	uint8_t S[32];			/* the client */
	struct zcpfs_keys keys[ZCPFS_LOGIN_MAX_BRANCHES];
	uint8_t ids[ZCPFS_LOGIN_MAX_BRANCHES];	/* the caller's name for each branch */
	uint8_t count;
	uint8_t used;
	uint8_t known;			/* the server knew the client before this login */
	uint8_t repeated;		/* it has replaced an earlier offer to the client */
	uint32_t until;
};

struct zcpfs_node {
	const uint8_t *S;		/* own Ed25519 identity key */
	const uint8_t *a;		/* own X25519 scalar (section 4) */
	struct zcpfs_drbg *drbg;
	struct zcpfs_peer *peers;	/* n_peers records, the caller's memory */
	size_t n_peers;
	const struct zcpfs_ops *ops;
	void *user;
	struct zcpfs_bucket pre;	/* R_PRE */
	struct zcpfs_login_wait login;
	struct zcpfs_offer *offers;	/* n_offers entries, the caller's memory; a server's */
	size_t n_offers;
};

/* The pointers must stay valid as long as the node is used. Wipes the records. */
void zcpfs_node_init(struct zcpfs_node *n, const uint8_t S[32], const uint8_t a[32],
		     struct zcpfs_drbg *drbg, struct zcpfs_peer *peers, size_t n_peers,
		     const struct zcpfs_ops *ops, void *user);

#define ZCPFS_SENT		1	/* sealed and sent under the session */
#define ZCPFS_HELD		2	/* kept until a session exists, for *held_for seconds */
#define ZCPFS_LEGACY		3	/* not a zcpfs contact: send it the legacy way */

/* One send attempt of a text by the application. `plain` is the plaintext of
 * 9.3 without the alignment byte, 5 to ZCPFS_TEXT_MAX bytes. `token` comes
 * back from zcpfs_on_ack when the message is acknowledged. Returns one of the
 * three values above, ZCPFS_ERR_ARG or ZCPFS_ERR_FULL. With ZCPFS_HELD,
 * *held_for is the number of seconds the message is kept: the timeout to give
 * the application for this attempt (10.4). The only call that can flood. */
int zcpfs_send_text(struct zcpfs_node *n, const struct zcpfs_contact *c,
		    const uint8_t *plain, size_t len, uint32_t token, uint32_t now,
		    uint32_t *held_for);

/* What zcpfs_rx found (7.2). */
#define ZCPFS_RX_LEGACY		0	/* not sealed: run legacy processing for this contact */
#define ZCPFS_RX_NEXT		1	/* not from this contact; no legacy for it either */
#define ZCPFS_RX_CONSUMED	2	/* handled; stop */
#define ZCPFS_RX_TEXT		3	/* out: a text to deliver and to acknowledge with ack4 */
#define ZCPFS_RX_DUPLICATE	4	/* out: a text delivered before; acknowledge only */
#define ZCPFS_RX_PROBE		5	/* answer with a path return without extra (11) */
#define ZCPFS_RX_PAIR		6	/* out: an opened request, response or path return */
#define ZCPFS_RX_LOGIN		7	/* out: the login response; the login succeeded (19.4) */

struct zcpfs_rx_out {
	/* The plaintext, then zero bytes to the end: the legacy view of 7.1. */
	uint8_t plain[ZCPFS_MAX_PAYLOAD + 8];
	size_t len;
	uint8_t ack4[4];
};

/* A received TXT_MSG, REQ, RESPONSE or PATH whose `dst` is this node, for one
 * contact whose hash matches `src`; called before any legacy decryption.
 * `flood` says how the packet was routed. For every result but LEGACY and
 * NEXT the packet is not to be retransmitted. */
int zcpfs_rx(struct zcpfs_node *n, const struct zcpfs_contact *c, uint8_t ptype,
	     const uint8_t *payload, size_t len, int flood, uint32_t now,
	     struct zcpfs_rx_out *out);

/* A received ACK value (9.5). Returns 1 with the token of the acknowledged
 * send and the contact's key, or 0: hand the value to legacy ACK handling. */
int zcpfs_on_ack(struct zcpfs_node *n, const uint8_t ack4[4], uint32_t *token,
		 uint8_t S[32]);

/* The caller processed a path return from the contact and now has a path.
 * `flood` says how that packet was routed. */
void zcpfs_on_path(struct zcpfs_node *n, const struct zcpfs_contact *c, int flood,
		   uint32_t now);

/* A legacy text from the contact carried the padding hint (6.3). */
void zcpfs_on_hint(struct zcpfs_node *n, const struct zcpfs_contact *c, uint32_t now);

/* Timers: handshake repeats, resends, expiry. Returns the seconds until it
 * needs to run again, UINT32_MAX if nothing is waiting. */
uint32_t zcpfs_tick(struct zcpfs_node *n, uint32_t now);

/* The contact was deleted or lost its pin (6.2): wipe what is kept for it. */
void zcpfs_forget(struct zcpfs_node *n, const uint8_t S[32]);

/* ---- 19: where a login meets the sessions --------------------------------
 *
 * The client keeps one login waiting for its reply. The server keeps, per
 * login it answered, the session keys of every branch until one is confirmed
 * by the first packet that opens under it. A login is answered only while
 * there is room for that: the size of the offer table is the limit on
 * password guesses and on reply airtime (19.3). One place of the table is
 * kept for clients the server already knows, and a client gets one new
 * login per offer, not more.
 */

#define ZCPFS_LOGIN_PLAIN_MAX	(ZCPFS_MAX_PAYLOAD - ZCPFS_LOGIN_OVERHEAD)

/* Client: the application logs in to a server (19.2). `login_plain` is the
 * legacy login plaintext without the password. The ANON_REQ leaves through
 * ops->send, flooded if there is no path: the second and last call that can
 * flood. Returns ZCPFS_SENT with *wait_for, the seconds the reply is waited
 * for and the timeout to give the application; ZCPFS_LEGACY if the server is
 * not pinned (19.6 decides whether a legacy login may go out instead); or an
 * error. A new login replaces one that is still waiting. For this call
 * c->timeout_ms is the caller's send timeout for the login packet itself. */
int zcpfs_login_send(struct zcpfs_node *n, const struct zcpfs_contact *c,
		     const uint8_t *password, size_t password_len,
		     const uint8_t *login_plain, size_t plain_len, uint32_t now,
		     uint32_t *wait_for);

/* Client: a login reply that did not come as a RESPONSE packet but as the
 * extra of a path return (19.4); zcpfs_rx handles the packet form itself.
 * ZCPFS_RX_LOGIN with the login response in `out`, or ZCPFS_RX_NEXT. */
int zcpfs_rx_login_reply(struct zcpfs_node *n, const struct zcpfs_contact *c,
			 const uint8_t *reply, size_t reply_len, int flood, uint32_t now,
			 struct zcpfs_rx_out *out);

/* Server: give the node room for `count` answered logins. Without this call
 * it answers none. */
void zcpfs_node_serve(struct zcpfs_node *n, struct zcpfs_offer *offers, size_t count);

struct zcpfs_login_req {		/* an authenticated login, between the two calls below */
	uint8_t S_c[32];
	uint8_t Y_c[32];
	uint8_t ss[32];
	uint8_t plain[ZCPFS_LOGIN_PLAIN_MAX + 1]; /* the login plaintext, then zeros */
	size_t plain_len;
	uint8_t empty_password;		/* the client typed none: which branches to offer */
	uint8_t known;
};

/* Server, 19.3 step 1: an ANON_REQ addressed to this node. `known` says
 * whether the key in the packet (payload + 1) is a client the server already
 * has on its list. ZCPFS_OK with `req` filled; ZCPFS_LEGACY if the length is
 * a legacy login's; ZCPFS_ERR_FULL if this login cannot be answered now;
 * ZCPFS_ERR_AUTH otherwise. For a known client the caller then drops a login
 * whose timestamp (in req->plain) is not newer than the last one it answered. */
int zcpfs_login_rx(struct zcpfs_node *n, const uint8_t *payload, size_t len, int known,
		   uint32_t now, struct zcpfs_login_req *req);

struct zcpfs_branch {			/* one password the server would accept from this client */
	const uint8_t *password;
	size_t password_len;
	const uint8_t *response;	/* ZCPFS_LOGIN_RESPONSE_LEN bytes for that role */
	uint8_t id;			/* comes back through ops->granted */
};

/* Server, 19.3 steps 2 to 5: build the reply and keep the offer. `reply` (what
 * follows dst and src, or goes into the path return) must hold
 * ZCPFS_MAX_PAYLOAD bytes. One or two branches: for an empty password the
 * one role it would get, otherwise the passwords that are not empty.
 * `timeout_ms` is the caller's send timeout for this reply; the offer is kept
 * ZCPFS_LOGIN_STEPS steps of it (10.4). */
int zcpfs_login_answer(struct zcpfs_node *n, const struct zcpfs_login_req *req,
		       const struct zcpfs_branch *branches, size_t count,
		       uint32_t timeout_ms, uint32_t now, uint8_t *reply, size_t *reply_len);

/* Server: a TXT_MSG for this node that no contact claimed. It may be the
 * first packet of a client that is not a contact yet (19.5). Results as
 * zcpfs_rx; anything but ZCPFS_RX_NEXT means ops->granted was called, and `S`
 * is that client. */
int zcpfs_rx_offer(struct zcpfs_node *n, const uint8_t *payload, size_t len,
		   uint32_t now, uint8_t S[32], struct zcpfs_rx_out *out);

#ifdef __cplusplus
}
#endif

#endif /* ZCPFS_H */
