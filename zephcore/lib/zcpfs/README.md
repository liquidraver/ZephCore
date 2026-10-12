# zcpfs

Forward-secret encryption between ZephCore nodes. `SPEC.md` is the protocol; this directory holds it as a small
C library, the Python reference model (`ref/`) and the test vectors both must reproduce (`vectors.json`).

**Status, 2026-10-10:** the library reproduces every vector, and its per-contact state machine passes the
model's whole-node scenarios in host tests. It is not part of any firmware build, nothing calls it yet, and
neither the design nor the code has had a cryptographic review.

## The library

Portable C99. No heap, no I/O, no clock, no logging, no Zephyr and no mesh code: every function works on buffers
and state the caller owns, and the caller passes the time.

It has two layers. The lower one is the cryptography, one packet or one session at a time. The upper one,
`zcpfs_peer.c`, is the protocol's behaviour: which session of a contact is used, when to handshake, what is
held and resent, the limits, and the rule that nothing but an application's send may flood. What stays outside
is what the firmware already has: the contact list with keys, paths and the stored pin, legacy encryption,
building and routing packets, ACK packets and path returns. The upper layer reaches it through four callbacks.

| File | What it is | Spec |
|---|---|---|
| `include/zcpfs.h` | The whole public API. | |
| `zcpfs_prim.c` | `KDF`, `AUTH`, sealing with the 8-byte tag, constant-time compare, wipe, the allowed-length rule. The only file that calls Ascon. | 3, 7 |
| `zcpfs_drbg.c` | The random generator. | 5 |
| `zcpfs_handshake.c` | The two handshake packets, both sides. | 8 |
| `zcpfs_session.c` | Seal and open under a session: the chains, the trial window, skipped keys, ACK values. | 9 |
| `zcpfs_stateless.c` | Path probe, channel packets, requests, responses and path returns. | 11, 17, 18 |
| `zcpfs_login.c` | The password login, both sides. | 19 |
| `zcpfs_peer.c` | The state machine per contact: the three session slots and their rotation, receive dispatch, held messages and resends, handshake retries, expected ACKs, duplicates, rate limits, timers, the bounded contact table. Also where a login meets the sessions: the client's one waiting login, the server's table of unconfirmed offers. | 7.2, 8.5, 9, 10, 11, 19 |
| `zcpfs_curve.c` | X25519, the Ed25519-to-X25519 key map and Elligator 2, from the vendored Monocypher. The only file that calls it. | 4, 19.1 |
| `zcpfs_internal.h` | What the files above share. | |

Ascon itself is not part of this library: it is `../ascon`, a standalone two-file implementation with its own
tests and no knowledge of zcpfs.

Not in the library: the advert tag and the padding hint, which are firmware work.

## Using the state machine

```c
static struct zcpfs_peer records[8];            /* about 2.4 kB each */
static struct zcpfs_node node;

zcpfs_node_init(&node, my_public_key, my_x25519_scalar, &drbg, records, 8, &ops, user);

/* the application sends a text */
switch (zcpfs_send_text(&node, &contact, plain, len, token, now, &held_for)) {
case ZCPFS_SENT:   /* on its way under the session */
case ZCPFS_HELD:   /* waits for a handshake: the application's timeout is held_for seconds */
case ZCPFS_LEGACY: /* not a zcpfs contact: encrypt and send it the old way */
}

/* a packet for this node arrived, for each contact whose hash matches, before legacy decryption */
switch (zcpfs_rx(&node, &contact, type, payload, len, was_flood, now, &out)) {
case ZCPFS_RX_LEGACY:    /* run legacy processing for this contact */
case ZCPFS_RX_NEXT:      /* try the next contact */
case ZCPFS_RX_CONSUMED:  /* done */
case ZCPFS_RX_TEXT:      /* deliver out.plain, acknowledge with out.ack4 */
case ZCPFS_RX_DUPLICATE: /* acknowledge with out.ack4 only */
case ZCPFS_RX_PROBE:     /* answer with a path return */
case ZCPFS_RX_PAIR:      /* out.plain is a request, response or path return */
}

if (zcpfs_on_ack(&node, ack4, &token, peer_key)) { /* tell the application `token` was delivered */ }
zcpfs_on_path(&node, &contact, was_flood, now);    /* after a path return was processed */
next_wake = zcpfs_tick(&node, now);                /* seconds until it wants to run again */
```

A server (repeater, room) also gives the node a small table for logins it has answered and not yet seen
confirmed (`zcpfs_node_serve`), passes an `ANON_REQ` through `zcpfs_login_rx` and `zcpfs_login_answer`, and
is told through the `granted` callback when a client has proved a password. A client logs in with
`zcpfs_login_send`. Section 19 of the header has the calls.

`contact` is a view the caller fills for the call: identity key, shared secret, pinned, has a path, and the
caller's own send timeout towards it. The engine has no fixed retry intervals: it counts them in steps of one
and a half times that timeout (spec 10.4), so they follow the radio setting and the path length. The four
callbacks in `struct zcpfs_ops` that every node needs transmit a packet, send the reciprocal path again,
store a pin, and look a contact up by key.

## Checking it

The host tests are part of the repository's quick profile. They build the library with
`-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror`.

- `tests/unit/zcpfs_tests.cpp`, cases `UNIT-ZCPFS-001` to `-010`: the lower layer against `vectors.json`.
- `tests/unit/zcpfs_peer_tests.cpp`, cases `UNIT-ZCPFS-011` to `-023`: whole nodes around the state machine,
  with a stand-in for the firmware, over a simulated link with loss, delay, duplication and replay; from
  `-021` on, logins to a server.

```bash
python3 tests/run.py run --profile quick --sanitize
```

The expected values are `tests/unit/zcpfs_vectors.h`, generated from `vectors.json`:

```bash
python zephcore/lib/zcpfs/ref/gen_c_vectors.py --check
```
