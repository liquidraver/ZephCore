# zcpfs: forward-secret direct messages for ZephCore

**Draft 0.19, 2026-10-17. A Python reference model and test vectors exist (`ref/`, `vectors.json`), and a C
library (this directory, see `README.md`) with the cryptographic core, which reproduces the vectors, and the
per-contact state machine of sections 7.2, 8.5 to 11 and 19. ZephCore companions, repeaters and room servers
can be built with it (`CONFIG_ZEPHCORE_ZCPFS`, on by default on the `zcpfs` branch, experimental). No cryptographic audit.**

Draft 0.19 follows a third review. A server clears a client's pin on a legacy login only if that login's
timestamp is newer than the client's last one, so a recorded empty-password login, which legacy accepts
without a replay check, cannot be replayed to unpin (6.2). The advert mark is capped at the receiver's own
clock, so a sender whose clock runs ahead cannot freeze its later adverts (6.2). A new setting,
`zcpfs.enforce`, makes pins permanent: nothing received clears one, and a server drops a pinned client's
legacy login (6.2, 19.6).

Draft 0.18 answers the second review of draft 0.17. A login reply whose branches repeat a `Y_s` is not a
reply (19.3): it would give a forger two tries at one branch key's 4-byte tag. The identity mixer runs
only at boot, before Bluetooth or WiFi start; a generator first seeded later uses the hardware RNG in its
place (5.1).

Draft 0.17 answers the first outside review (`devdocs/zcpfs-crypto-review-findings.md`). A client tries one
reply per login: a reply names the login it answers by two bytes of `Y_c`, one for another login is ignored,
and the first that names this one ends it, so a server gets the branches of one reply, at most two guesses,
and a forger one try at each branch tag (19.3, 19.4). A sealed `RESPONSE` is opened as one before it is tried
as a login reply (7.2). A seed that cannot be stored is replaced by mixing in fresh entropy (5.1). A sealed text or request raises
the sender's advert timestamp, so a recorded untagged advert from before it can no longer unpin (6.2).

Draft 0.16 settles the legacy-login setting as built (19.6): `zcpfs.legacylogin`, effective while the mode
is on.

Draft 0.15 settles the channel setting as built (17.3): its CLI name, where it is stored, and how it combines
with the run-time mode.

Draft 0.14 adds a run-time mode (6.4): on, off, and passive, which answers what arrives sealed but starts
nothing, so that a node can leave the protocol without its peers losing messages.

Draft 0.13 follows the first server built on it. A server keeps the pin of a client with the client's entry
and clears it when that client logs in the legacy way (6.2), since it keeps no adverts of its clients.
`R_LOGIN_FLOOD` is counted over a sliding window, so it holds for any 180 s, not per period (19.3). A room
server that allows read-only access gives it to an empty password only (19.3 step 3).

Draft 0.12 settles how a login reply travels (19.3): as legacy's does, flood for flood and direct for
direct, with a budget on flooded replies to clients the server does not know and a direct answer over the
reverse path when the budget is empty. The budget does not look at whether earlier replies were followed
up, because a lost reply and a guess look the same. For the same reason a client's new login replaces its
offer whenever that is a step old, instead of once.

Draft 0.11 makes the login reply as small as the construction allows and closes two ways to abuse a server
(section 19). The login says whether a password was typed at all, so a reply never has more than two
branches; a branch carries 6 bytes of response and a 4-byte tag; one place of the offer table is kept for
clients the server knows; a client gets one new login per offer. The login packets and their vectors
changed; nothing else did. How a reply is routed is still open (19.8, 16).

Draft 0.10 gives the login its limits in the same terms as everything else since 10.4: the client's wait and
the server's offer last two steps of the sender's own timeout (was a fixed 30 s), and a login is answered
only while the server has room to keep its offer, which replaces the separate rate `R_LOGIN` (19.3). It also
corrects the airtime behind the table of 10.4 and records two things the login does to the mesh that legacy
does not (19.8, 16). No packet format, key or test vector changes.

Draft 0.9 follows the C state machine. Writing it for a fixed amount of memory showed four things the model
had left open: own handshake attempts and held messages had no end (8.2, 10.1, new `T_HS_WAIT` and `T_HELD`),
the contact table is bounded and needs a rule for who may take a record (new 9.6), and `T_HS_RETRY` now counts
from the newest handshake 1 whatever started it (10.1). The four intervals of the handshake and the held
messages are no longer fixed seconds: they follow the node's own send timeout towards the contact (new
10.4), and a held message lives exactly as long as the application is told to wait for it. No packet format,
key or test vector changes.

Draft 0.8 replaces the login of draft 0.7 with a password-authenticated key exchange (section 19): the
password is no longer sent at all. It adds the setting that refuses legacy logins, without which that
protection can be bypassed (19.6).

Draft 0.7 covers the remaining encrypted packet types: requests, responses and path returns (section 18,
stateless) and the login to repeaters and room servers (section 19, which also sets up a session).

Draft 0.6 shrinks the ACK of a sealed message to 4 bytes (7, 9.5). Signed channel messages were considered
and rejected: 68 more bytes on every flooded message, to stop only members posing as each other.

Draft 0.5 makes the channel mode part of version 1, gives the channel tag a strict setting (17.3) and
names the firmware's entropy sources (5.1).

Draft 0.4 adds a stateless mode for channels (section 17), without forward secrecy, switched on per
channel by a CLI command.

Draft 0.3 follows the reference model. Running it over a lossy link changed the rules for holding messages and
retrying the handshake (10.1), kept several handshake attempts alive at once (8.2), repeated the reciprocal
path until the peer answers (11), and corrected three parameters (`N_ACK`, `T_PROBE`, new `T_HS_RETRY`,
`N_HS_TRIES`, `N_HELD`). Later the same day: node-level resend of messages that waited for a handshake (10.3)
and the receiver's duplicate rule (9.2), added to make up for the attempts a handshake uses.

Draft 0.2 answers a first outside review of 0.1 (seven findings, all accepted): the MeshCore binding is
specified (7.1), retries never repeat bytes (7, 8.2), skipped keys belong to the session in any slot (9.1),
the handshake rate limit is charged after authentication (8.3), ACK handling is normative (9.5), "newer advert"
is defined (6.2), and the failed-decryption limit is enforced (9.4). That review was of the text, not a
cryptographic audit.

This document is the byte-exact description of the protocol. Rationale, alternatives and history are in
`devdocs/PFS_ASCON_PLAN.md`. Where the two disagree, this document is the one the code must follow, and the
disagreement is a bug in one of them.

The key words MUST, MUST NOT, SHOULD and MAY are used as in RFC 2119.

## 1. Scope

zcpfs protects direct text messages between two ZephCore nodes so that traffic recorded off the air cannot be
decrypted with keys taken from either device later.

Version 1 gives every packet type MeshCore encrypts a sealed form:

| Traffic | Protection | Section |
|---|---|---|
| Text between two nodes: direct messages, admin text to and from a repeater, room posts | session, forward-secret | 4 to 16 |
| Login to a repeater or room server | a password-authenticated key exchange: the password is never sent; it also sets up a session | 19 |
| Requests, responses, path returns | stateless, a key per packet, no forward secrecy | 18 |
| Channels | stateless, a key per packet, no forward secrecy, per channel by CLI | 17 |

Adverts, ACKs, traces and control packets are not encrypted by MeshCore and are not changed. Forward secrecy
does not cover the stateless rows, or plaintext a device or phone still stores. It does not hide who talks to whom. It is not secure against an
attacker with a quantum computer who recorded the handshake.

A node that does not implement zcpfs is never affected: it forwards every zcpfs packet as an ordinary MeshCore
packet and is only ever sent legacy packets itself.

## 2. Notation

- `a ‖ b` is concatenation. `x[i:j]` is bytes `i` up to but not including `j`.
- `LE16(n)` is `n` as two bytes, least significant first.
- `0^n` is `n` zero bytes. `""` is the empty byte string.
- Lengths are in bytes unless bits are stated.

## 3. Primitives

| Name | Definition |
|---|---|
| `X25519(k, u)` | RFC 7748, section 5. |
| `AEAD.enc(K, N, AD, P)` | Ascon-AEAD128, NIST SP 800-232 section 4. 16-byte key, 16-byte nonce. Returns `C` (same length as `P`) and a 16-byte tag `T`. |
| `AEAD.dec(K, N, AD, C, T8)` | The matching decryption with the tag truncated to its leftmost 8 bytes (SP 800-232 section 4.2.1). |
| `CXOF(M, Z, L)` | Ascon-CXOF128, SP 800-232 section 5, message `M`, customization string `Z`, `L` output bytes. |

The tag length is 8 bytes and never varies for a key (SP 800-232 requirement R4). One kind of key uses 4
bytes instead, also never varying: the key of a login reply's branch (section 19.3), whose tag tells the
client which branch is its own and is trusted for nothing else.

Tag comparison MUST be constant-time. Every comparison of a DH output against `0^32` MUST be constant-time.

### 3.1 Key derivation

```
KDF(label, p1, ..., pn; L) = CXOF(M, Z, L)
    Z = "zcpfs1/" ‖ label                          (ASCII, no terminator)
    M = LE16(len(p1)) ‖ p1 ‖ ... ‖ LE16(len(pn)) ‖ pn
```

Every use has its own label (section 12). No derived value is used for two purposes.

### 3.2 Empty-message authentication

Handshake and probe bodies are authenticated with the AEAD over an empty plaintext:

```
AUTH(k, AD) = T[0:8]   where (_, T) = AEAD.enc(k[0:16], k[16:32], AD, "")
```

`k` is 32 bytes from `KDF` and is used for exactly one `AUTH`.

## 4. Identities

A node's identity is its MeshCore Ed25519 key pair. In this document:

- `S_X` is node X's 32-byte Ed25519 public key as it appears in adverts.
- `A_X` is its Montgomery form (the birational map of RFC 7748 section 4.1; monocypher
  `crypto_eddsa_to_x25519`).
- `a_X` is X's X25519 scalar: the first 32 bytes of the 64-byte expanded MeshCore private key.
- `ss = X25519(a_I, A_R) = X25519(a_R, A_I)` is the 32-byte MeshCore shared secret, unhashed. Legacy
  encryption keys AES and HMAC with it directly; zcpfs only ever feeds it to `KDF`.

If `ss = 0^32` the peer's key is of small order. Such a peer MUST NOT be tagged, pinned, probed or handshaken
with.

## 5. Random numbers

All secret random values (ephemeral scalars, probe nonces) come from one generator per node.

```
state:    K (32 bytes)

seed at boot:
    K = KDF("drbg.seed", hw, saved; 32)       hw    = 32 bytes from the platform entropy source
                                              saved = 32-byte seed file, or 0^32 if absent
    write generate(32) to the seed file before any other use

generate(n):
    out = KDF("drbg.gen", K; 32 + n)
    K   = out[0:32]                           the old K is wiped
    return out[32:32+n]

mix(x):
    K = KDF("drbg.mix", K, x; 32)
```

Each call hashes from a fresh initial state, so an earlier `K`, and with it earlier output, cannot be computed
from a later `K`. A generator that keeps one sponge state and only squeezes MUST NOT be used: the permutation
is invertible.

### 5.1 Where the entropy comes from in ZephCore

This generator is a layer on top of the firmware's existing random number code (`adapters/rng/ZephyrRNG`),
not a replacement for it and not a second entropy source.

- `hw` is 32 bytes from `ZephyrRNG::random()`, the Zephyr CSPRNG that the rest of the firmware uses.
- When no seed file exists (first boot, or after the storage was erased), `saved` is 32 bytes from
  `ZephyrRNG::mixIdentitySeed()`, the slower layered mixer that also produces the node's identity key. The
  first seed is therefore as good as the identity key itself.
- On every later boot `saved` is the seed file, so the state chains from boot to boot and is fed after
  every handshake (8.3, 8.4).
- If the next seed cannot be written, the next boot will read this boot's seed again and, with a repeated
  hardware output, rebuild this boot's state. The generator then mixes in 32 bytes of fresh entropy (`mix`),
  the identity mixer at boot, before it gives anything else, so the stream of this boot is not the one the
  next boot rebuilds.
- The file is `/lfs/zcpfs_seed`, one per node whatever its role, replaced atomically. The generator is
  seeded at boot when the mode is on or passive, after the identity and the settings are loaded and before
  Bluetooth or WiFi start: the identity mixer blocks about half a second, and on ESP32 it switches the
  hardware RNG to a source that is not safe once the radio or the ADC is in use. If zcpfs is first used
  without a generator seeded at boot (the mode was off at boot and set later), it is seeded then, with the
  hardware RNG in place of the identity mixer in both uses above; `get zcpfs.status` shows it (seed bit 8).
  By then the radios run, so on ESP32 the hardware RNG has its RF entropy; what is lost is the mixer's own
  timing entropy, for a hardware RNG that is broken.
- The generator is seeded once per boot; a new identity (an imported key) keeps it.

Every random value in this document comes from `generate`: ephemeral scalars, the probe nonce, the channel
`n6` and the random byte of an ACK. None is drawn from the hardware directly.

## 6. Capability signalling

### 6.1 Advert tag

A zcpfs node appends one byte `0x00` to the name field of its advert `app_data`. The advert signature covers
it. The whole `app_data` MUST still fit in 32 bytes; the name is shortened if necessary. A zcpfs node MUST
always emit the name field, even with an empty name, because an advert without the tag clears the pin
(section 6.2).

A receiver that finds a `0x00` inside the name bytes of a validly signed advert reads the capability:

- nothing after the `0x00`: version 1 (this document);
- one byte after it: that version number. Unknown versions are treated as "not capable".

The displayed name is the bytes before the `0x00`.

### 6.2 Pinning

A node keeps, per contact and across reboots, one flag: **pinned**.

- A contact becomes pinned when a validly signed advert with a capability tag is processed for it, by any route
  that runs the normal advert verification (radio, import, share).
- A contact becomes pinned when a handshake with it completes (section 8.6).
- A contact stops being pinned only when an **accepted** advert for it carries no tag. Its sessions are wiped
  at the same moment.
- A server keeps no adverts of its clients (18.2). It pins a client when the client's login is confirmed
  (19.5) or a handshake with it completes, keeps the flag with the client's entry, and clears it, with the
  sessions, when the client logs in with a legacy login: that login is authenticated by the client's key and,
  like an untagged advert, says that its owner no longer speaks this protocol. Only a legacy login whose
  timestamp is newer than the client's last stored timestamp clears the pin. Legacy checks that for a login
  with a password but not for a known client's empty one, which it accepts as it is; such a login is still
  accepted, and clears the pin only if it passes the same test. The stored timestamp is kept in RAM, as
  legacy keeps it, so after a server reboot a recorded legacy login can still unpin; `zcpfs.enforce`
  closes that.

"Accepted" is MeshCore's existing rule, unchanged: the signature verifies and the advert's signed 32-bit
timestamp is strictly greater, as an unsigned integer, than the timestamp of the last advert accepted for that
contact. An advert that fails either test is not processed at all and can neither pin nor unpin. This holds for
every route, including import and share, which pass through the same check. Consequences:

- A replayed or equal-timestamp advert changes nothing.
- A peer whose clock went backwards cannot change its pin state in either direction until its clock passes
  the stored value. Timestamp wraparound is not handled, as in MeshCore.
- An accepted advert without the tag **is** intended to authorise legacy sending again. It is signed by the
  identity key, so only the owner of that identity can issue it (for example by moving to firmware without
  zcpfs).
- Contact fields written by the application, including a stored advert timestamp, do not affect the pin.

A handshake pins without an advert, so the stored timestamp may still be one from before the peer spoke this
protocol, and a recorded untagged advert signed later than it but before the peer turned zcpfs on would be
accepted and unpin. To close that, a node raises the stored advert timestamp of a contact when a sealed text
(9) or sealed request (18) from it opens: both start with the sender's 32-bit timestamp, authenticated by the
session or pair secret. The new value is that timestamp less `ADVERT_MARK_SLACK` (60 s), if that is
greater than the stored one. The slack is there because a text's timestamp comes from the sender's
application (a phone) and an advert's from its node: without it the peer's own next advert, for example the
untagged one of a move back to legacy, could be refused when it is signed in the same second or the node's
clock is behind. The sender's time is first capped at the receiver's own clock: a sender whose clock runs
ahead would otherwise raise the mark past every advert it will sign until its clock is right, and the
receiver would refuse them all, the untagged one included. The application sets the node's clock when it connects, so what is left is one second of
rounding and the node's drift since; 60 s is more than a day at 500 ppm, the worst sleep clock Bluetooth LE
allows. A node without an application stamps its texts with its own clock. What remains: an untagged advert
signed less than the slack before the peer's latest sealed text or request, or any one before its first, can
still unpin until a later sealed message passes it or a tagged advert is accepted.

Towards a pinned contact a node MUST NOT send a legacy text message. If no session can be established the
message fails.

**Enforcing.** A node can be set to keep its pins (`set zcpfs.enforce on|off`, default `off`, every role,
effective while the mode is `on`). While enforcing, nothing received clears a pin: an accepted advert
without the tag is processed (name, position, path) but leaves the pin and the sessions as they are, and a
server drops a legacy login from a pinned client unread (19.6). A pin then goes only when the contact is
deleted, or a server's client is removed from its list. This closes every replay that could unpin, whatever
the timestamps, at the price of the graceful exit of 6.4: a peer that leaves this protocol stays unreachable
from an enforcing node until it is deleted and added again there.

### 6.3 Padding hint

A legacy text message to a contact that is not pinned carries a hint when it fits.

```
legacy plaintext before AES padding:   timestamp(4) ‖ flags(1) ‖ text ‖ [0x00 ‖ attempt]     (attempt > 3 only)
with hint:                             timestamp(4) ‖ flags(1) ‖ text ‖ 0x00 ‖ a ‖ 0x5A
                                       a = attempt if attempt > 3, else 0x00
```

- The hint is added only if it does not increase the number of AES blocks.
- A receiver recognises it as byte `0x5A` at offset `strlen(text) + 2` after the text start.
- On a hint from a contact that is not pinned, a zcpfs node SHOULD start a handshake towards it: direct only,
  only once it has a path, at most once per contact per `T_HINT`.
- A hint MUST NOT pin a contact by itself.
- In ZephCore the hint is added and acted on only while the node's mode (6.4) is `on`, by companions, on plain
  text messages. A legacy receiver is not affected: the text ends at the first zero byte, and the byte after
  it, which legacy reads as the extended attempt number for its ACK value, is the same as without the hint.

### 6.4 Run-time mode

A node has one setting, changed by a CLI command (`set zcpfs on|passive|off`, default `off`): locally on a
companion, and on a repeater or room server also by an admin over the mesh, as its other settings are:

| Mode | Advert tag | Starts anything sealed | Reads and answers sealed | Legacy from a pinned contact |
|---|---|---|---|---|
| `on` | yes | yes | yes | dropped (6.2, 7.2) |
| `passive` | no | no | yes | read |
| `off` | no | no | no | read |

- `off` is legacy on the air. A sealed packet is not understood, and the state of every session is wiped when
  the node switches to it.
- `passive` is the way out. Peers that pinned the node keep sealing to it until its untagged advert reaches
  them (6.2); it answers their handshakes, logins and requests and acknowledges their texts, but it treats no
  contact as pinned: it reads legacy from everyone and an unopened text never makes it start a handshake
  (10.2). It seals a text only to a contact that has a session with it and has sent it something sealed since
  its own last advert, because a contact that heard that advert has wiped the session and reads legacy. A
  contact that still pins it and gets a legacy text starts a handshake, which the node answers, and the
  application's next attempt goes sealed.
- A node keeps tracking pins from adverts in every mode, so switching back on needs nothing more.
- `zcpfs.enforce` (6.2) acts only in `on`; a `passive` node unpins as usual, which is what leaving needs.
- Switching off directly from `on` costs messages: until the untagged advert reaches them, peers drop the
  node's legacy texts, and its logins and handshakes go unanswered. `passive` first, `off` once
  `get zcpfs.status` shows no sealed traffic, avoids that.

## 7. Packets

All packets are ordinary MeshCore packets with payload version 1. `dst` and `src` are the 1-byte hashes MeshCore
already uses (first byte of the public key).

| Packet | MeshCore type | Payload | Length |
|---|---|---|---|
| Data | `TXT_MSG` (0x02) | `dst ‖ src ‖ C ‖ T8` | `len(P) + 10` |
| Handshake 1 | `REQ` (0x00) | `dst ‖ src ‖ E_I ‖ tag1` | 42 |
| Handshake 2 | `RESPONSE` (0x01) | `dst ‖ src ‖ E_R ‖ tag2` | 42 |
| Probe | `REQ` (0x00) | `dst ‖ src ‖ nonce4 ‖ tagp` | 14 |
| Request, response, path return | `REQ`, `RESPONSE`, `PATH` (0x08) | `dst ‖ src ‖ n6 ‖ C ‖ T8` (section 18) | `len(P) + 16` |
| Login | `ANON_REQ` (0x07) | `dst ‖ S_c ‖ Y_c ‖ C ‖ T8` (section 19) | `len(L) + 74`: 78 to a repeater, 82 to a room |
| Login reply | `RESPONSE` (0x01) | `dst ‖ src ‖ count ‖ e ‖ branches` (section 19) | 47 with one branch, 89 with two |
| ACK | `ACK` (0x03) | `ack4` | 4 (legacy sends 6) |

Rules for every zcpfs packet:

- **Never block-aligned:** `(payload length - 4) mod 16` MUST NOT be 0; for `ANON_REQ`, whose legacy layout
  carries a 32-byte key, the test is `(payload length - 35) mod 16`. Handshake (38) and probe (10) satisfy
  it by construction.
- **Reserved lengths:** a `REQ` of 42 or 14 bytes is always a handshake 1 or a probe, and a `RESPONSE` of 42
  bytes is always a handshake 2. No other packet of those types may have those lengths.
- A sender meets both rules by appending zero bytes to the plaintext before sealing (at most two). This is
  harmless because legacy plaintexts are zero-padded to the block anyway, and every handler already
  tolerates trailing zeros.
- **Maximum payload:** a zcpfs payload MUST NOT exceed 176 bytes. MeshCore itself allows 184. 176 is what the
  longest legacy plaintext needs here (165, plus one alignment byte, plus 10), and it stays below 181, the
  size from which a non-aligned payload overflows a buffer in Arduino MeshCore releases that predate upstream
  PR 1809.
- **Every transmission is unique on the wire.** MeshCore suppresses any packet whose type and payload it has
  seen recently (a table of 160 hashes), at the destination and at every repeater on the way. A zcpfs node
  MUST NOT retransmit the bytes of an earlier packet; every retry is built anew (fresh ephemeral, fresh nonce,
  or the next chain key).

### 7.1 MeshCore binding

- **Receive.** MeshCore decrypts `PATH`, `REQ`, `RESPONSE` and `TXT_MSG` as legacy before any callback sees
  them, and a zcpfs payload fails that decryption. The zcpfs dispatch of section 7.2 therefore runs inside the
  mesh receive layer: after the duplicate check, for each candidate contact, **before** the legacy
  MAC-then-decrypt. If it consumes the packet, the legacy path is skipped for that packet and the packet is
  marked as not to be retransmitted, as a legacy packet for this node is. Flood routing of the packet is
  otherwise unchanged.
- **Send.** A zcpfs payload is placed in the packet as it is, with the type code of section 7. It MUST NOT pass
  through the legacy AES and HMAC wrapping.
- **Legacy view.** An opened plaintext of any type is zero-extended to a whole number of 16-byte blocks before
  it enters legacy handling, so the code above sees exactly what legacy decryption would have produced.
- **Delivery.** An opened data plaintext enters the same message handling as a decrypted legacy text, with one
  difference: the ACK value is supplied by zcpfs instead of being computed from the plaintext.
- **The application's expected ACK.** The phone app is told an "expected ACK" value the moment it sends, and
  later a confirmation carrying that value. `ack4` does not exist yet at that moment when the message is held
  for a handshake. The node therefore hands the application a local token, and reports that token when the
  matching `ack4` arrives. On-air ACK values never reach the application.
- **The application's timeout.** When a send has to wait for a handshake, the estimated timeout returned to
  the application is `T_HELD` (section 10.4), the time the message is held.

### 7.2 Receiver dispatch

For a packet whose `dst` matches the node, for each contact whose hash matches `src`:

| Type | Condition | Action |
|---|---|---|
| `TXT_MSG` | contact has a session in any slot | try section 9.2; on success stop |
| `TXT_MSG` | the node keeps a login offer for the sender (19.3), nothing opened it | try the offer's branches, section 19.5; on success stop |
| `TXT_MSG` | contact pinned, nothing opened it | section 10.2 |
| `TXT_MSG` | contact not pinned | legacy processing, then the hint check of 6.3 |
| `REQ` | length 42 | handshake 1, section 8.3 |
| `REQ` | length 14 | probe, section 11 |
| `RESPONSE` | length 42, a handshake 1 is outstanding for the contact | handshake 2, section 8.4 |
| `REQ`, `RESPONSE`, `PATH` | not block-aligned | stateless open, section 18; if it does not verify, the next row |
| `RESPONSE` | not block-aligned, a login is outstanding for the contact | login reply, section 19.4 |
| `REQ`, `RESPONSE`, `PATH` | block-aligned, contact pinned | dropped |
| anything else | | legacy processing |

The tests are made in this order. Legacy payloads are always block-aligned, so none of the length tests can
misfire on them. An `ANON_REQ` is not addressed by a contact; it is dispatched by section 19.2. On a server a
`TXT_MSG` whose sender is no contact at all is still tried against the login offers (19.5): a client becomes
known to the server only when its login is confirmed.

## 8. Handshake

Two messages, mutually authenticated by the static keys, with one ephemeral each. The Diffie-Hellman terms are
those of the Noise KK pattern (`-> e, es, ss` / `<- e, ee, se`). This is **not** an implementation of the Noise
framework: the key derivation and the authentication tags are this document's own construction. I is the
initiator, R the responder.

### 8.1 Values

```
e_I, e_R   ephemeral scalars, generate(32) each
E_I        = X25519(e_I, 9)          E_R = X25519(e_R, 9)
es         = X25519(e_I, A_R)        = X25519(a_R, E_I)
ee         = X25519(e_I, E_R)        = X25519(e_R, E_I)
se         = X25519(a_I, E_R)        = X25519(e_R, A_I)
```

Each of `es`, `ee`, `se` MUST be compared with `0^32`; on a match the handshake is abandoned silently.

### 8.2 Initiator sends handshake 1

```
k1   = KDF("hs1", ss, es, S_I, S_R, E_I; 32)
tag1 = AUTH(k1, 0x01 ‖ E_I)
send   dst ‖ src ‖ E_I ‖ tag1
keep   e_I, E_I, es as the outstanding handshake
```

A retry is a new handshake 1 with a new `e_I`; the same bytes are never sent twice (section 7). The initiator
keeps its last `N_HS_TRIES` outstanding attempts, and a handshake 2 that verifies against any of them completes
the handshake. Otherwise a reply that is merely slower than `T_HS_RETRY` could never be used.

Outstanding attempts are wiped, all of them, `T_HS_WAIT` after the newest was sent. An ephemeral key does not
wait for a reply indefinitely.

### 8.3 Responder receives handshake 1

1. If the contact is of small order (section 4), drop.
2. Pre-authentication work limit: if the node-wide budget `R_PRE` is exhausted, drop. Each `tag1` verification
   costs one X25519 and consumes one unit, whether or not it succeeds.
3. Compute `es`, `k1`; verify `tag1`. On failure drop silently.
4. Per-contact limit: if `R_HS` is exceeded for this contact, drop. Only a message that passed step 3 counts
   against `R_HS`, so traffic that merely carries the contact's address byte cannot consume it.
5. Simultaneous open: if the responder itself has an outstanding handshake 1 towards this contact, compare
   `S_local` and `S_peer` as byte strings. The node with the smaller key stays initiator and drops the received
   message; the other abandons its own outstanding handshake and continues here.
6. Generate `e_R`; compute `ee`, `se`, then:

```
master = KDF("master", ss, es, ee, se, S_I, S_R, E_I, E_R; 32)
k2     = KDF("hs2", master; 32)
tag2   = AUTH(k2, 0x02 ‖ E_R)
ck_i2r = KDF("chain.i2r", master; 32)
ck_r2i = KDF("chain.r2i", master; 32)
mix(KDF("drbg.feed", master; 32))
send   dst ‖ src ‖ E_R ‖ tag2
```

7. Store the new session in the `next` slot (section 9.1), replacing whatever was there: send chain `ck_r2i`,
   receive chain `ck_i2r`. Wipe `e_R`, `es`, `ee`, `se`, `master`, `k1`, `k2`. Nothing of the handshake itself
   is kept.
8. The `current` slot is not touched.

### 8.4 Initiator receives handshake 2

1. Compute `ee`, `se`, `master`, `k2`; verify `tag2`. On failure drop silently and keep waiting.
2. Derive `ck_i2r` (send) and `ck_r2i` (receive); `mix(KDF("drbg.feed", master; 32))`.
3. Rotate: `previous = current`, `current = ` the new session, `next` is wiped. Wipe `e_I`, `es`, `ee`, `se`,
   `master`, `k1`, `k2` and all outstanding attempts.
4. The initiator MUST send first. Messages held for this contact (section 10.1) are sealed and sent now, in
   the order they were given; if there are none, a data packet with empty plaintext (a **confirmation**,
   10-byte payload) is sent, direct only.

### 8.5 Responder confirmation

When any data packet opens under the responder's `next` session (section 9.2), the responder rotates:
`previous = current`, `current = next`, `next` empty. If it holds messages for the contact (it lost a
simultaneous open), it sends them now.

Until then the responder MUST NOT seal anything under `next`. A replayed handshake 1 can create a `next` session
that nobody else holds; sending under it would lose the message.

### 8.6 Result

A contact is pinned (section 6.2) by the initiator at step 3 of 8.4 and by the responder at the rotation of
8.5.

## 9. Session

### 9.1 State

Per contact there are three session slots, the model WireGuard uses for the same problem:

| Slot | Holds | Used for |
|---|---|---|
| `current` | the session in use | sealing and opening |
| `next` | responder only: a session whose handshake the initiator has not yet confirmed | opening only |
| `previous` | the session `current` replaced | opening only |

`previous` is wiped `T_PREV` after a packet first opens under `current`: the peer has demonstrably switched,
and the delay lets packets it sealed just before switching still arrive.

Per session, in whichever slot it is; all of it moves with the session when slots rotate:

```
ck_s          32   send chain key
ck_r          32   receive chain key
skipped[S]         up to S entries of (key 16, nonce 16, ackk 16, expiry)
expect[N_ACK]      ack4 values of the most recent non-empty sends still unacknowledged
unacked        1   consecutive non-empty sends without a matching ACK
fails          4   received packets that caused at least one failed decryption under this session
t_start, t_last    uptime seconds: establishment, last successful seal or open
```

An implementation MAY bound the number of skipped keys node-wide instead of per session. Dropping a skipped key
early only loses the late packet it was kept for.

Sessions live in RAM only. All timers use the uptime clock; the real-time clock MUST NOT be used.

Chain step:

```
STEP(ck) = KDF("step", ck; 80)
    ck'   = out[0:32]
    key   = out[32:48]
    nonce = out[48:64]
    ackk  = out[64:80]
```

### 9.2 Sending and receiving

Associated data for data packets: `AD = dst ‖ src ‖ 0x02`.

**Seal** plaintext `P`, always under `current`. If `current` is empty the sender has no session (section 10.1),
whatever `next` holds.

1. `(ck', key, nonce, ackk) = STEP(ck_s)`; set `ck_s = ck'` and wipe the old chain key. This happens before any
   ciphertext is produced and on every path, including errors.
2. `(C, T) = AEAD.enc(key, nonce, AD, P)`; `T8 = T[0:8]`.
3. `ack4 = KDF("ack", ackk, C ‖ T8; 4)` is the ACK value the sender expects (section 9.5).
4. Wipe `key`, `nonce`, `ackk`. Emit `dst ‖ src ‖ C ‖ T8`.

**Open** a received `C ‖ T8`. The slots are tried in the order `current`, `next`, `previous`; within a slot:

1. Candidates, in this order: position 0 of the receive chain; the entries of `skipped`; positions 1 to `W-1`
   of the receive chain. Position `j` is obtained by applying `STEP` `j+1` times to a **copy** of `ck_r`.
2. Try `AEAD.dec` with each candidate until one verifies. If none does, nothing in the session changes except
   `fails`. `fails` is incremented once for every received packet during whose processing at least one
   `AEAD.dec` under this session failed, whether or not a later candidate opened it. No key of the session can
   therefore have seen more failed decryptions than `fails`.
3. If a `skipped` entry verified, remove it.
4. If chain position `j` verified: store the keys of positions `0..j-1` in `skipped` (keeping the `S` most
   recent, each expiring after `T_SKIP`), set `ck_r` to the chain key after position `j`, and wipe the rest.

Steps 1 to 4 are the same for all three slots, skipped keys included.

Then, by the slot that opened it:

- `current`: if this is the first packet opened under it, start the `T_PREV` timer that wipes `previous`.
- `next`: rotate as in section 8.5.
- `previous`: if nothing has opened under `current` yet, the peer has not switched. The node sends a
  confirmation (empty plaintext under `current`), direct only, at most once per `T_CONFIRM`.

If the plaintext is empty the packet was a confirmation: nothing is delivered and no ACK is sent. Otherwise the
plaintext is delivered and `ack4 = KDF("ack", ackk, C ‖ T8; 4)` is returned the way legacy returns its ACK (an
ACK packet, or the extra of a path return).

**Duplicates.** A plaintext byte-identical to one already delivered from the same contact within `T_DEDUP` is
acknowledged but not delivered again. The node keeps a short hash of the last `N_DEDUP` delivered plaintexts
per contact for this (`KDF("dedup", P; 8)` in the model and the library; it never leaves the node, so any
hash will do). An application retry is not affected: it carries a different attempt number, so its
plaintext differs.

A packet that verified once can never verify again, because its key is gone; no replay window is kept.

### 9.3 Plaintext

`P` is exactly what legacy would encrypt before AES padding:

```
timestamp(4) ‖ flags(1) ‖ text ‖ [0x00 ‖ attempt]        (the bracket only when attempt > 3)
```

If `(len(P) + 6) mod 16 = 0`, one byte `0x00` is appended to `P` (the never-block-aligned rule).

A confirmation (section 8.4) is the one data packet whose `P` is empty.

The receiver treats the opened plaintext as followed by two zero bytes, then processes it exactly as a legacy
text message.

### 9.4 Lifetime

A session in any slot is wiped, on either side independently, when any of these holds:

- `now - t_last > T_IDLE`;
- `now - t_start > T_MAX`;
- the node reboots or the contact table gives its record away (section 9.6);
- `fails` reaches `F_MAX`;
- for `current`: a send finds `unacked` at `K_UNACKED` or more. The check is made when the next message is to
  be sealed, not when the count is reached, so ACKs still in flight can reset it; the sender then handshakes
  before that send.

### 9.5 Acknowledgements

`ack4` is a 32-bit keyed token. It tells the sender that the holder of the session keys received that exact
ciphertext. It is not packet authentication: a guessed value matches with probability 2^-32 per expected
entry, and the worst a match can do is mark one message delivered and reset `unacked`.

Sender side:

- Sealing a non-empty plaintext appends its `ack4` to the session's `expect` list (the oldest entry is dropped
  when the list holds `N_ACK`) and increments `unacked`, saturating.
- Sealing a confirmation changes neither `expect` nor `unacked`. No ACK is expected for it.
- An ACK value arrives without addresses, as in legacy. It is compared with the `expect` lists of every
  contact's `current` session, then of every `previous` session.
  - Match in `current`: remove the entry, report the message delivered, set `unacked` to 0.
  - Match in `previous`: remove the entry and report the message delivered. `unacked` of `current` is not
    changed.
  - No match: the value is handed to the legacy ACK handling.
- `expect` is wiped with its session. A message whose session was wiped before its ACK arrived is reported
  through the application's ordinary timeout.

`unacked` is also set to 0 whenever a packet opens under `current`. The count exists to notice a peer that
has lost the session, and a peer whose packets open under it has not. Without this, text that legacy does not
acknowledge (CLI commands and the replies to them) would have to be acknowledged here, or an admin session
would be torn down every `K_UNACKED` commands.

Receiver side: one ACK per opened non-empty packet of a kind legacy acknowledges (plain and signed text; not
CLI commands or their replies), returned the way legacy returns it (an ACK packet, or the
extra of a path return). Its payload is `ack4` alone, 4 bytes, the shortest ACK MeshCore accepts. Legacy
appends an attempt byte and a random byte so that two ACK packets are never identical and suppressed as
duplicates; `ack4` already differs for every sealed packet, and a packet is opened and acknowledged once.
A confirmation is never acknowledged.

### 9.6 A bounded contact table

Everything sections 8 to 11 keep for a contact (the three slots, outstanding handshakes, held messages, the
duplicate list, the timers and the per-contact limits) is one record in RAM. A node has room for a fixed
number of records, usually far fewer than it has contacts. A contact without a record has no session; a
record is made when one is needed, and when none is free another contact's record is wiped and reused.

Which record may be taken depends on what asks for it:

| What needs a record | May take |
|---|---|
| a text that opens under no slot (10.2), a probe (11), a padding hint (6.3) | a free record, or an **idle** one: no session in any slot, nothing held, no handshake outstanding |
| a handshake 1 that verified (8.3), a send by the local application | any record |

Among the records it may take, a node takes in this order: free; idle; holding only a `next` nobody
confirmed; any other. Within one class, the least recently used.

The first row is the point of the rule. Those three events can be caused by anyone who can transmit two
address bytes, or replay a recorded probe; if they could take any record, a stream of forged packets would
wipe every live session. If no record may be taken the event is dropped, except that a probe is still
answered: without a record it is answered every time, which is what legacy does with every flooded message.

A replayed handshake 1 verifies, so it can take a record. It takes an idle or an unconfirmed one if there is
one, and it leaves an unconfirmed one behind, which the next replay takes. A series of replays therefore costs
one live session, not one each, for as long as that unconfirmed record is not reused by something else.

## 10. Recovery

### 10.1 Sender has no session

The `current` slot is empty. The message is held and a handshake is run with this node as initiator.

- **Held messages.** Up to `N_HELD` messages per contact wait for the session, oldest first; when the list is
  full the oldest is dropped. A retry of a message already held (same timestamp and text) replaces it. All of
  them are sealed, in order, the moment the session becomes `current` (8.4 step 4, 8.5). A message that has
  waited `T_HELD` is dropped. `T_HELD` is also the timeout the application was given for that attempt (7.1),
  so the message goes at the moment the application gives up on it; a session that comes up an hour later
  must not send it.
- **Starting.** A send starts a handshake 1 (or, without a path, the probe of section 11) only if no probe was
  started by a send and no handshake 1 was sent, for whatever reason, to this contact within the last
  `T_HS_RETRY`. Otherwise the message just joins the held list. Without this rule a burst of sends restarts
  the handshake on every message and none completes.
- **Repeating.** While messages are held, a path exists and no session has come up, the node sends a new
  handshake 1 every `T_HS_RETRY`, at most `N_HS_TRIES` in total per application send attempt. Direct only. A
  probe is never repeated by the node: it is a flood and stays bound to an application attempt.
- A session that never comes up is reported to the application through its ordinary timeout.

### 10.2 Receiver cannot open

A node that receives a `TXT_MSG` from a pinned contact which no slot opens, and that has no outstanding
handshake towards that contact, SHOULD send a handshake 1 to it: direct only, at most once per contact per
`T_RESYNC`. Existing slots are left as they are; the handshake replaces them only through the rotations of
section 8.

This also fires, rarely, on a foreign packet whose two address bytes collide. The cost is one handshake.

### 10.3 Resending a held message

A message that waited for a handshake has used up part of the application's attempt before it was first sent.
While it stays unacknowledged the node therefore sends it again itself: every `T_DATA_RETRY`, at most
`N_DATA_TRIES` more times, direct only, and only while the session it was sent under is still `current`. Each
copy is sealed anew under the next chain key (section 7); every copy's `ack4` maps to the same application
token. Messages that did not wait for a handshake are not resent by the node; the application retries those.

The receiver side of this is the duplicate rule of section 9.2.

### 10.4 The step: intervals follow the link

`T_HS_RETRY`, `T_HS_WAIT`, `T_HELD` and `T_DATA_RETRY` are not fixed numbers of seconds. Airtime differs by a
factor of fifty between the radio settings a mesh may run, and a path has one hop or many. A fixed interval is
too short on a slow link, where a second handshake would leave before the first answer can be back, and on a
fast one it keeps ephemeral keys and held plaintext long after any answer could still come.

A node already estimates how long an answer takes: the timeout its legacy code applies when it sends a packet
to that contact. Let `T_SEND` be that timeout for a packet of handshake size (42 bytes) sent to the contact
now: over the known path, or flooded if there is none.

```
s      = T_SEND in seconds, rounded up
T_STEP = max(5, s + ceil(s / 2))                  one and a half times the node's own timeout

T_HS_RETRY   = 1 x T_STEP
T_HS_WAIT    = 2 x T_STEP
T_HELD       = (N_HS_TRIES + 1) x T_STEP          every attempt, and one step for the last answer
T_DATA_RETRY = 1 x T_STEP
```

The extra half is for what the legacy timeout does not count, above all a transmitter on the way that holds
a packet back while the channel is busy.

`T_STEP` is evaluated when an interval starts; a path that changes afterwards does not move an interval that
is already running. `T_HELD` is the timeout handed to the application for that attempt (7.1), so nothing is
held for an application that has stopped waiting.

In ZephCore `T_SEND` is what `calcDirectTimeoutMillisFor` and `calcFloodTimeoutMillisFor` return:
`500 + (6 x airtime + 250) x (hops + 1)` ms with a path, `500 + 16 x airtime` ms without. Airtime is the
firmware's own estimate, with its 32-symbol preamble at SF 8 and below. For a 42-byte payload (47 bytes on
the air with a three-byte path) on the two settings in use on the mesh this was written for, at 62.5 kHz:

| Setting | Airtime | Path | `T_SEND` | `T_STEP` | `T_HS_WAIT` | `T_HELD` |
|---|---|---|---|---|---|---|
| SF7, CR 4/5 | 234 ms | 1 hop | 3.8 s | 6 s | 12 s | 24 s |
| | | 3 hops | 7.1 s | 12 s | 24 s | 48 s |
| | | 5 hops | 10.4 s | 17 s | 34 s | 68 s |
| | | none (flood) | 4.2 s | 8 s | 16 s | 32 s |
| SF8, CR 4/8 | 607 ms | 1 hop | 8.3 s | 14 s | 28 s | 56 s |
| | | 3 hops | 16.1 s | 26 s | 52 s | 104 s |
| | | 5 hops | 23.9 s | 36 s | 72 s | 144 s |
| | | none (flood) | 10.2 s | 17 s | 34 s | 68 s |

These are computed, not measured. `N_HS_TRIES` is 3 because the simulator says so: one message at 25 %
packet loss with a path is delivered in 98 % of runs with three attempts per send, 90 % with two and 72 %
with one (legacy: 88 %); at 10 % loss all three reach 100 %.

## 11. Path probe and the flood rule

**The flood rule.** A zcpfs implementation MUST NOT originate a flooded packet except at the one point where
the legacy code floods a text message because the contact has no path, and at most once per application send
attempt. Handshake messages, confirmations, resynchronisation and retransmissions are direct only; without a
path they are not sent.

At that one point, towards a pinned contact:

- with a `current` session: the sealed data packet is flooded, as legacy would flood the message;
- without one: a probe is flooded instead of the message, through the same scoped-flood path legacy uses.

```
nonce4 = generate(4)
kp     = KDF("probe", ss, S_I, S_R, nonce4; 32)        S_I = sender, S_R = receiver
tagp   = AUTH(kp, 0x03 ‖ nonce4)
send     dst ‖ src ‖ nonce4 ‖ tagp
```

A receiver verifies `tagp`. On success, and at most once per contact per `T_PROBE`, it answers exactly as it
would answer a flooded legacy message without an ACK: a legacy path return with no extra data. It learns
nothing else from a probe and changes no session state.

The sender then has a path, sends its reciprocal path as legacy does, and runs the handshake direct.

The responder can only answer once that reciprocal path has reached it, and the sender cannot see whether it
did. So until the sender has received a direct packet from the peer over the new path, every handshake 1 it
sends is preceded by the reciprocal path packet again. As in legacy, an ACK that arrives flooded while a path
to its sender is known also causes the reciprocal path to be sent again.

## 12. Labels and parameters

Labels (each prefixed with `zcpfs1/` by `KDF`):

| Label | Used for |
|---|---|
| `drbg.seed`, `drbg.gen`, `drbg.mix`, `drbg.feed` | section 5 and the feed after a handshake |
| `hs1`, `hs2` | handshake authentication keys |
| `master` | handshake master secret |
| `chain.i2r`, `chain.r2i` | initial chain keys |
| `step` | chain step |
| `ack` | ACK value |
| `probe` | probe authentication key |
| `chan` | key and nonce of a channel packet (section 17) |
| `pair` | key and nonce of a request, response or path return (section 18) |
| `pake.gen` | input of the password generator (section 19) |
| `pake1`, `pake2`, `pake.master` | keys of the login, of a reply branch, and the login's master secret (section 19) |
| `dedup` | hash of a delivered plaintext for the duplicate list (section 9.2); local, never sent |

`AUTH` associated-data prefixes: `0x01` handshake 1, `0x02` handshake 2, `0x03` probe.

Parameters (defaults; all provisional until measured):

| Name | Default | Meaning |
|---|---|---|
| `W` | 16 | receive window, chain positions tried |
| `S` | 4 | skipped keys kept per session |
| `T_SKIP` | 120 s | lifetime of a skipped key |
| `T_IDLE` | 24 h | idle time after which a session is wiped |
| `T_MAX` | 7 d | maximum session age |
| `T_PREV` | 60 s | how long `previous` is kept after the first packet opens under `current` |
| `ADVERT_MARK_SLACK` | 60 s | a sealed text or request raises the sender's stored advert timestamp to its own less this (6.2) |
| `T_CONFIRM` | 5 s | minimum interval between repeated confirmations per contact |
| `K_UNACKED` | 8 | consecutive unacknowledged sends before a new handshake |
| `N_ACK` | 16 | expected ACK values kept per session; the companion's existing ACK table has 16 entries |
| `T_STEP` | at least 5 s | section 10.4: 1.5 times the node's own send timeout towards the contact |
| `T_HS_RETRY` | 1 step | minimum interval between handshake 1 messages (or probes) started for a contact |
| `N_HS_TRIES` | 3 | handshake 1 messages per application send attempt, and outstanding attempts kept |
| `T_HS_WAIT` | 2 steps | how long outstanding handshake 1 attempts are kept after the newest was sent |
| `N_HELD` | 4 | messages held per contact while a session comes up |
| `T_HELD` | `N_HS_TRIES` + 1 steps | how long a message is held, and the application's timeout for that attempt |
| `T_DATA_RETRY` | 1 step | interval between node-level resends of a held message |
| `N_DATA_TRIES` | 2 | node-level resends of a held message |
| `T_DEDUP` | 120 s | how long a delivered plaintext is remembered |
| `N_DEDUP` | 16 | delivered plaintexts remembered per contact |
| `F_MAX` | 2^20 | failed-decryption count at which a session is wiped |
| `T_LOGIN` | 2 steps | how long a client waits for a login reply, and a server keeps an unconfirmed offer (19.2, 19.3) |
| `N_LOGIN` | 2 | unconfirmed offers a server keeps at once; no login is answered without room for its offer (19.3) |
| `R_LOGIN_FLOOD` | 4 in any 180 s, node-wide | flooded login replies to clients the server does not know (19.3) |
| `R_PRE` | 4 per second, burst 16, node-wide | handshake 1 verifications attempted |
| `R_HS` | 3 per 30 s per contact | authenticated handshake 1 messages acted on |
| `T_RESYNC` | 10 min | interval between receiver-initiated handshakes per contact |
| `T_HINT` | 24 h | interval between hint-triggered handshakes per contact |
| `T_PROBE` | 15 s | interval between answered probes per contact; MUST be shorter than the application's retry interval, or a lost answer costs a whole attempt |

`K_UNACKED` MUST be smaller than `W`. `F_MAX` MUST NOT exceed 2^32. `R_PRE` MUST be generous enough that only a
sender occupying most of the channel can exhaust it; its purpose is to bound CPU, and the radio already limits
how many 42-byte packets can arrive.

## 13. Security notes

- **Forward secrecy.** `ee` depends on two ephemeral scalars that are wiped when the handshake finishes, and
  each chain key is wiped when it is stepped. A device captured later holds nothing that yields an earlier
  message key. Exception: a message that was sent but never opened stays decryptable from the receiver's chain
  until a later message is opened or the session is wiped (`T_IDLE` at most).
- **Authentication.** `tag1` requires `ss` and `es`; the session keys additionally require `se`. An attacker
  holding one side's static key cannot complete a session as the other side.
- **Weak ephemerals.** `ss`, `es` and `se` are in `master`, so a predictable ephemeral loses forward secrecy
  for that session but does not make the keys predictable.
- **Tag length.** With an 8-byte tag a forgery attempt succeeds with probability 2^-64 (SP 800-232 section
  4.4.1). Trial decryption multiplies the attempts per received packet by at most `3 (W + S)`. SP 800-232
  requirement R5 allows 2^32 failed decryptions per key at this tag length. A key encrypts once but can be
  tried many times, so the limit is enforced: the session is wiped when `fails` reaches `F_MAX` (section 9.4).
- **Many keys.** Every message has its own key. The nonce is derived with the key and differs per key, which
  avoids the same-nonce multi-key case of SP 800-232 section 4.4.2.
- **Nonce reuse.** There is no misuse resistance. Safety rests on rule 1 of Seal (step and wipe before
  encrypting).
- **Replay.** A replayed data packet fails. A replayed handshake 1 yields one rate-limited handshake 2 and a
  `next` session nobody can confirm; `current` is untouched and nothing is ever sealed under `next`. A replayed
  probe yields at most one path return per `T_PROBE`. On a node whose contact table is full, replayed
  handshakes of several contacts can cost one other contact its session, and a replayed probe is answered
  every time (section 9.6).
- **Replay against a handshake in progress.** Handshake 1 carries no freshness, so an old recorded one,
  replayed between a genuine handshake 1 and the initiator's first packet, displaces the genuine `next`. The
  initiator's packet then fails to open and section 10.2 repairs it. This is denial of service only, by an
  attacker who must transmit at the right moment. WireGuard closes the same hole with a monotonic timestamp in
  the initiation; the clock-free equivalent would be a persisted handshake counter (section 16).
- **ACK.** `ack4` is a 32-bit token, not authentication (section 9.5).
- **Length.** There is no padding, so the packet length reveals the plaintext length to the byte (legacy
  reveals it to the 16-byte block). This is accepted for the airtime it saves.
- **Downgrade.** A pinned contact is never sent legacy text. The pin is set from signed data or a completed
  handshake and cleared only by a newer signed advert. The padding hint cannot pin.
- **Denial of service.** Whoever can block the handshake blocks messages to that contact; they are never sent
  under older keys or as legacy.
- **Legacy ACK.** The legacy ACK value is a hash of the plaintext and reveals whether a guessed message is
  right. `ack4` is keyed and depends on the ciphertext.

## 14. Conformance statement

The AEAD, the XOF and their parameters are those of NIST SP 800-232. zcpfs is not a claim of conformance to
it: the keys are derived with Ascon-CXOF128 used as a key derivation function, and SP 800-232 section 6 leaves
the approved uses of the XOFs to other publications. Of its AEAD requirements: R3 (unique nonce) holds because
every key and nonce pair is derived once and the chain key is wiped before use; R4 (tag length) holds because
every kind of key has one tag length, 8 bytes, or 4 for the branch key of a login reply (section 3); R5
(failure limit) is enforced by `F_MAX`, and a branch key is tried once.

## 15. Test vectors

`vectors.json`, next to this file, is written by `ref/gen_vectors.py` from the reference model. A firmware
implementation must reproduce every value in it. The handshake values are computed twice there, once step by
step from the primitives and once through the model's functions. The primitives under it are checked against
published vectors: the official Ascon known-answer tests, RFC 7748 and RFC 8032.

The C library in this directory reproduces the packets, keys and ACK values of items 1 to 10 below, with its
own Ascon and with Monocypher for the curve where the model has its own Python (`tests/unit/zcpfs_tests.cpp`).
Its state machine has no vectors: it is tested as whole nodes over a simulated link, with the scenarios of
the model's `ref/tests/test_sim.py` and of the login in `ref/tests/test_requests_login.py`
(`tests/unit/zcpfs_peer_tests.cpp`). It does not implement the flooded-handshake variant of section 16. The bounded table of 9.6
exists only in the library; the model keeps a record for every contact.
The advert tag and the padding hint are firmware work and not in the library. Model and
library were written by the same author from the same text: their agreement shows that the text can be
implemented twice with the same result, not that it is right. An independent implementation agreeing with
the vectors is still outstanding.

Contents:

1. `KDF` for each label with fixed inputs.
2. The generator: seed, three `generate` calls, one `mix`.
3. A complete handshake with fixed static and ephemeral keys: every intermediate value and both packets.
4. A session: ten sealed packets in each direction, with `ack4`.
5. Open with loss and reordering: positions 0, 3, 1, 2 (skipped keys), and a gap of `W`.
6. A probe.
7. Negative cases: flipped bit in tag, ciphertext, `dst`; all-zero DH output; replayed packet; a block-aligned
   plaintext length (the padding byte).
8. Channel packets of section 17: key, nonce and three sealed packets for a fixed secret and `n6`.
9. Stateless pairwise packets of section 18, including plaintexts that need the length padding.
10. A complete login of section 19: the generator, the login with a typed and with an empty password, the
    reply with a matching and a non-matching branch, every intermediate value, the resulting chain keys;
    and the Elligator map alone.

## 16. Open points

- Parameter values in section 12 are tested only in the reference simulator (one hop, independent random
  loss), not on radios.
- First contact without a path is less robust than legacy under heavy loss, because six packets must arrive
  before the message is confirmed instead of two. Simulator, one message to a new contact, 40 runs per figure:
  delivered in 100 % of runs at 10 % packet loss (legacy 100 %) and in 78 % at 25 % loss (legacy 88 %). The
  simulator's link is one hop with 0.3 to 1.2 s of delay; these runs use a step of 8 s.
- **Open decision: what is flooded at first contact.** The reference model also implements the earlier design
  in which handshake 1 itself is flooded and handshake 2 rides the flooded path return
  (`Params.FIRST_CONTACT = "handshake"`). It needs four packets instead of six: 100 % at 10 % loss, 85 % at
  25 %. The price is flooded bytes: about 115 per first contact at 10 % loss against 39 for the probe and 76
  for legacy. Sections 7 and 11 describe the probe, and the C library implements only the probe: it ignores a
  handshake 1 that arrives flood-routed.
- A handshake 2 is verified against every outstanding attempt, two X25519 each, and that work is not under
  `R_PRE`. It only happens while a handshake is outstanding (`T_HS_WAIT`), and a 42-byte packet takes longer
  on the air than the six X25519 it can cost; whether it should be budgeted is open.
- `R_LOGIN_FLOOD` (four per 180 s) is the shape ZephCore already uses for its other limits on requests from
  unknown senders, not a derived number. A server that sees more first logins by flood than that in normal
  use would answer some of them over the reverse path.
- Legacy logins have no such budget. On a repeater with open guest access they are the larger exposure
  (19.8), and the same budget would cover them.
- Going below one curve point per accepted password in the reply (a point shared by all branches) needs
  another key exchange than this one, and a cryptographer.
- The rule of 9.6 lets replayed probes and handshakes do more on a node whose table is full. How large the
  table has to be on a companion is not measured.
- With a known path a single message is delivered as often as legacy at 10 % loss and more often at 25 %
  (98 % against 88 %), through the resend of 10.3. Longer conversations measured within a few points of legacy
  before 10.3 existed (87 % against 91 % for 30 messages at 25 % loss).
- Airtime and delivery latency have to be measured across the supported LoRa settings, for each multi-packet
  sequence: handshake plus confirmation, and probe, path return, reciprocal path, handshake. The timers and
  the application timeout handed to the phone depend on it.
- A persisted handshake counter in handshake 1 (4 more bytes) would remove the replay of section 13. It needs a
  recovery path for an identity moved to a device with a lower counter, which is why it is not in this draft.
- RAM for three slots per contact against the companion budget; whether `previous` and `next` come from a
  small shared pool.
- Behaviour of third-party advert decoders with the tag byte of 6.1.
- The magic byte of 6.3 against the padding signal of nextgens' unpublished implementation.
- The login of section 19 is this document's own instantiation of the CPace construction (its own key
  derivation, several password branches in one reply). It follows the published design but is not the
  CFRG draft byte for byte and has had no review.
- CPU cost of a login on the smallest boards: up to three Elligator maps and seven X25519 on the server.
- No independent review has taken place.

## 17. Stateless mode for channels

Channels do not get forward secrecy: that needs pairwise key exchanges and therefore a member list, which a
MeshCore channel does not have. This mode replaces the legacy channel encryption (AES-ECB and a 2-byte MAC
under one static key) with a key per message and an 8-byte tag. Anyone who holds the channel secret, now or
later, can read every message sealed under it.

### 17.1 Packet

For the MeshCore types `GRP_TXT` (0x05) and `GRP_DATA` (0x06):

```
payload = channel_hash(1) ‖ n6 ‖ C ‖ T8                         length: len(P) + 15

n6           = generate(6)                 fresh for every packet
k            = KDF("chan", secret32, n6; 32)
(C, T)       = AEAD.enc(k[0:16], k[16:32], channel_hash ‖ type ‖ n6, P);   T8 = T[0:8]
```

- `secret32` is the 32-byte channel secret field as MeshCore stores it (a 16-byte secret followed by zeros).
  `channel_hash` is MeshCore's existing 1-byte hash.
- `P` is what legacy would encrypt before padding. If `(len(P) + 12) mod 16 = 0`, one byte `0x00` is appended,
  so that `(payload length - 3) mod 16` is never 0 (the never-block-aligned rule for channel packets).
- `P` MUST NOT exceed 161 bytes, so the payload stays within 176. That is 4 bytes less text than legacy
  allows; the sender cuts the text as legacy does.
- `n6` is random, not a counter, on purpose: a counter or a per-boot value in clear would let an observer
  group a channel's messages by sender, which today's channel packets do not allow.

### 17.2 Receiving

A node that implements this mode reads both formats on every channel it holds, unless the channel is set to
strict (17.3). For a channel
packet whose `channel_hash` matches a held channel:

- `(payload length - 3) mod 16 ≠ 0`: sealed. Derive `k` from `n6`, verify and decrypt.
- otherwise: legacy.

The length decides exactly; nothing is tried twice. An opened plaintext enters the normal channel handling.

### 17.3 The channel tag

Each channel has a setting, changed only by a local CLI command on the companion
(`set zcpfs.channel <name|index> off|on|strict`, `get zcpfs.channel [<name|index>]`). The phone app is not
involved and cannot see it.

| Setting | Sends | Reads |
|---|---|---|
| `off` (default) | legacy | both formats |
| `on` | sealed | both formats |
| `strict` | sealed | sealed only; a legacy packet on this channel is dropped |

- `on` is for the transition: members who have not switched yet are still heard. As long as legacy packets
  are accepted, an outsider can still forge one with the legacy odds of 2^-16 per attempt.
- `strict` closes that. A strict node no longer hears members who send legacy, whether they run old
  firmware or have not tagged the channel, and nothing tells either side.
- The setting is stored with the node, keyed by a digest of the channel secret, not by the channel's index,
  so it follows the channel when the app reorders the list and disappears with the secret. In ZephCore the
  digest is the first 4 bytes of SHA-256 of the 32-byte secret field, and up to 12 channels can be `on` or
  `strict` at once (the entries fit one prefs value); entries of channels the node no longer holds are
  reused first.
- The setting applies only while the node's mode (6.4) is `on`. A `passive` node reads both formats on every
  channel and sends legacy; an `off` node is legacy and does not understand sealed channel packets.
- A sealed text longer than 161 bytes of plaintext is cut, never inside a UTF-8 character; channel data that
  does not fit sealed is not sent.
- It does not travel with a shared channel. Each member sets it at their own node.
- Members on firmware without this mode cannot read sealed messages and are not told that they missed any.
  Tagging is a decision of whoever runs the channel.
- Public and hashtag channels may be tagged but gain nothing: their secret is public.

### 17.4 Security notes

- No forward secrecy, no protection against anyone who has or later gets the channel secret.
- No sender authentication: any member can write as any other, as in legacy.
- Integrity against outsiders: a sealed packet is forged with probability 2^-64 per attempt. A channel that
  still reads legacy (`off`, `on`) remains forgeable at 2^-16 through the legacy format; only `strict`
  removes that.
- Identical texts give unrelated packets; the header bytes are bound to the message.
- Two packets sharing an `n6` under one channel secret would reveal the XOR of their plaintexts. With 48
  random bits that is expected after about 2^24 (16 million) messages on one channel secret.
- SP 800-232 requirement R5 (at most 2^32 failed decryptions per key) is not enforced here: there is no state
  to count in. An attacker would have to deliver 2^32 forgeries carrying one `n6` value.
- Replay: a recorded packet verifies again. MeshCore's duplicate table and the timestamp in the message are
  the only protection, as in legacy.
- Length: the packet length reveals the plaintext length to the byte.

## 18. Requests, responses and path returns

These three types get the stateless treatment of section 17, with the pair's static shared secret in place of
a channel secret. They are never sealed under a session, on purpose: either side can lose a session at any
time, text has ACKs and section 10 to notice and repair that, and these packets do not. A request sealed under
a session the other side no longer has would simply vanish.

### 18.1 Packet

For the MeshCore types `REQ`, `RESPONSE` and `PATH`:

```
payload = dst ‖ src ‖ n6 ‖ C ‖ T8                               length: len(P) + 16

n6       = generate(6)                 fresh for every packet
k        = KDF("pair", ss, S_sender, S_receiver, n6; 32)
(C, T)   = AEAD.enc(k[0:16], k[16:32], dst ‖ src ‖ type ‖ n6, P);   T8 = T[0:8]
```

- `P` is what legacy would encrypt before padding, with zero bytes appended until the length is allowed
  (section 7).
- Both identities are in the derivation, sender first, so the two directions never share a key and a packet
  cannot be reflected back to its sender.
- A path return's `extra` has no length field in legacy. Where the exact length of an extra matters (the login
  reply, 19.2), the extra starts with one length byte.

### 18.2 When it is used

- **Sending.** Sealed towards a pinned contact. Sealed also when answering, with a response or a path return,
  something that itself arrived sealed, whether or not that contact is pinned ("in kind"): a server keeps no
  adverts of its clients and learns what they understand from what they send. Legacy otherwise.
- **Reading.** A node reads the sealed form from any contact. From a pinned contact a legacy `REQ`, `RESPONSE`
  or `PATH` is dropped.
- Receiving a sealed packet does not pin its sender (section 6.2 is unchanged).

### 18.3 Security notes

- No forward secrecy: whoever later holds either static key can read recorded requests and responses.
- A forged path return could redirect a contact's route. It now needs 2^-64 per attempt instead of 2^-16.
- Replay: a recorded packet verifies again. The tags and timestamps legacy already uses in requests are the
  only protection, as before.
- SP 800-232 requirement R5 is not enforced, as in section 17: there is no state to count in.

## 19. Login

A legacy login is an `ANON_REQ` carrying the client's public key and, under the static shared secret with ECB
and a 2-byte MAC, a timestamp and the password. Whoever holds the addressed key reads the password, including
a repeater that merely copied a familiar name.

Here the login is a password-authenticated key exchange in the manner of CPace. The password is never sent,
not even encrypted. Each side multiplies a secret scalar onto a curve point derived from the password; only if
both started from the same password do they arrive at the same key. A server that does not know the password
learns nothing from a login except whether one of its guesses, at most two, was right.

### 19.1 The password generator

```
G(pw) = ELL2( KDF("pake.gen", pw, S_c, S_s; 32) )
```

`pw` is the password as typed, without terminator, possibly empty. `S_c` and `S_s` are the client's and the
server's identity keys, so the same password gives unrelated points for different pairs.

`ELL2` is the Elligator 2 map onto Curve25519, exactly as monocypher's `crypto_elligator_map`: the input is
read as a little-endian integer `r` with its two top bits cleared; `w = -A / (1 + 2 r^2)` with `A = 486662`;
the result is `w` if `w^3 + A w^2 + w` is a square, and `-w - A` otherwise; output as 32 bytes little-endian.

### 19.2 The client sends

Used only towards a pinned server. Otherwise the legacy login is sent, unless legacy logins are switched off
(19.6).

```
y_c = generate(32);   Y_c = X25519(y_c, G(pw))
k1  = KDF("pake1", ss, S_c, S_s, Y_c; 32)
(C, T) = AEAD.enc(k1[0:16], k1[16:32], dst ‖ 0x07 ‖ S_c ‖ Y_c, f ‖ L)
send, type ANON_REQ:   dst ‖ S_c ‖ Y_c ‖ C ‖ T8
keep   y_c, Y_c as the outstanding login
```

`f` is one byte of flags: bit 0 is set if the typed password is empty; the other bits are zero and ignored.
With it the server knows which of its passwords can be meant without knowing anything about a typed one
(19.3). `L` is the legacy login plaintext **without the password**: the timestamp, and for a room the sync
marker, with zero bytes appended until `(payload length - 35) mod 16 ≠ 0`. The packet travels as a legacy login does:
direct, or flooded when the server has no known path.

A node keeps one outstanding login, as the firmware keeps one pending login; a new login replaces it. It is
wiped `T_LOGIN` after it was sent, with `T_STEP` taken from the node's send timeout for the login packet
itself, and `T_LOGIN` is the timeout the application is given for the login (section 10.4).

`k1` depends only on the static keys. It shows the server that the sender owns `S_c`; it says nothing about
the password.

### 19.3 The server

An `ANON_REQ` addressed to the server is this format if `(payload length - 35) mod 16 ≠ 0`, and a legacy
login otherwise.

1. Pre-authentication limit `R_PRE`. `ss = X25519(a_s, A_c)`; drop if it is `0^32`. Derive `k1`, open. On
   failure drop silently.
2. Login limit: the server answers only if it has room to keep the offer of step 5, that is, fewer than
   `N_LOGIN` offers to other clients are waiting. One place is kept for clients the server already knows
   (they are on its list): a client it does not know is answered only while fewer than `N_LOGIN` - 1
   offers to other unknown clients are waiting, or `N_LOGIN` is 1. Nobody can pose as a known client,
   because step 1 has verified the key. A known client's login is dropped if its timestamp (in `L`) is not
   newer than that of the last login the server answered for it, as legacy does, so that an old login
   cannot be replayed into the kept place. Every answered login gives its sender one password guess
   and costs the server one reply, so this one limit bounds both. A full table is known from the key in the
   packet before any work is done; the check is repeated after step 1 has authenticated that key.
3. The server cannot yet tell which password the client typed. It prepares one **branch** for each password
   it would accept from this client that fits the flag `f`, at most two. If none was typed: the one role an
   empty password gets this client (it is on the access list, or the guest password is empty); if there is
   none, the login is not answered. If one was typed: the admin password, and the guest password if one is
   set and not empty. A room server that allows read-only access gives it to an empty password from a client
   it does not know. Legacy gives it to any password that matches neither; here a typed password can only be
   answered by a branch built from that same password, so a wrong one gets nothing and the client has to log
   in again without one. For each, with a fresh scalar:

```
y_j  = generate(32)
Y_j  = X25519(y_j, G(pw_j))
K_j  = X25519(y_j, Y_c)                              skip this branch if K_j = 0^32
master_j = KDF("pake.master", ss, K_j, S_c, S_s, Y_c, Y_j; 32)
k2_j     = KDF("pake2", master_j; 32)
(C_j, T_j) = AEAD.enc(k2_j[0:16], k2_j[16:32], dst ‖ src ‖ 0x01 ‖ Y_j, R_j)
branch_j   = Y_j ‖ C_j ‖ T_j[0:4]                    42 bytes
chains: ck_i2r = KDF("chain.i2r", master_j; 32), ck_r2i = KDF("chain.r2i", master_j; 32)     client = initiator
```

`R_j` is 6 bytes: the server's time (4), the permissions of that role (1), the firmware level (1). That is
what a legacy login response carries that the client cannot know; the client's firmware rebuilds the
13-byte legacy response from it for the application.

The branch tag is 4 bytes, the shortest SP 800-232 allows. It lets the client find the branch of its own
password and nothing rests on it: a tag that matches by chance, once in 2^32 wrong branches, gives the
client keys the server does not have, its first packet does not open under any offer, and nothing is
granted (19.5). What the server trusts is the 8-byte tag of that first packet.

4. Reply, type `RESPONSE`:

```
dst ‖ src ‖ count ‖ e ‖ branch_1 ‖ ... ‖ branch_count ‖ zero bytes      47 bytes with one branch, 89 with two
e = Y_c[0:2]
```

`e` names the login the reply answers. It is public, as `Y_c` is, and authenticates nothing; it lets the
client tell a late reply to an earlier login from a reply to the one it waits for (19.4). Zero bytes are
appended until the length is allowed (section 7). The `Y_s` of the branches are distinct: a reply with one
`Y_s` twice is not a login reply, and the client goes on waiting. Two such branches would have one key
(`K`, the transcript and `Y_s` are the same), so a forger could try two tags under it, where SP 800-232
allows one failure for a 4-byte tag. The bytes are compared as sent, as they enter the transcript.

**How the reply travels.** As legacy's login response does, with one limit:

- The login arrived **direct**: a `RESPONSE` packet, sent direct along the reply path the request supplied
  or the path the server has stored for the client. With neither it would have to be flooded.
- The login arrived **by flood**: the extra of a path return (with the length byte of 18.1), flooded in the
  scope the request arrived in, so that the client learns its path to the server.
- A flooded reply to a client the server does not know (step 2) takes one unit of the budget
  `R_LOGIN_FLOOD`, whatever becomes of it: at most four such replies in any 180 s, a sliding window, so that
  no burst at the edge of a period doubles it. When the budget is empty, the reply to a flooded login goes
  direct instead, as a path return over the reverse of the path the login arrived on, and a direct login
  without a known way back is not answered.
- Replies to clients the server knows are not counted.

The budget counts replies, not outcomes. A reply the mesh lost and a reply to a wrong guess are the same
thing to the server, an answer nobody follows up; a limit that tightened on those would slow the users of
a lossy mesh down exactly when they have to retry. The reverse path is not proven in that direction, which
is why it is the fallback and not the rule.

5. The server keeps the candidate sessions, one per branch, as the **offer** to that client, for `T_LOGIN`,
   with `T_STEP` taken from its send timeout for the reply it has just built (section 10.4). A new login of
   the same client replaces its offer and starts its time again if the offer is at least one `T_STEP` old;
   sooner, the login is not answered. An application's retry comes after `T_LOGIN`, two steps, and is
   always answered, however many it takes. A sender guessing passwords gets one answer per step and place. An offer leaves the table when it is confirmed (19.5) or when its
   time is up, never to make room. **Nothing is granted yet**: no role, no access-list entry, and the
   client is not yet a contact.

A wrong password is not answered with silence as in legacy: the server answers every well-formed login,
because it cannot know. What a wrong password gets is a reply none of whose branches it can open.

### 19.4 The client receives the reply

A non-block-aligned `RESPONSE` from a server with an outstanding login that does not open as a sealed
response (18), or the extra of its path return, is parsed as a login reply. A reply whose `e` is not the start
of the outstanding login's `Y_c` answers another login and is ignored. The first reply whose `e` matches ends
the login, whatever it holds: for each of its branches `K = X25519(y_c, Y_j)` (skip if `0^32`), `master`,
`k2`, try to open, and then the outstanding login is wiped whether a branch opened or not. A server therefore
tests at most the passwords of one reply's branches per login, two, and someone who forges a reply gets one
try at each 4-byte branch tag, as SP 800-232 allows for that tag length. A forged branch that opens by chance
(2^-32) shows the application a login that succeeded, with keys nobody else has; the server grants nothing.

- A branch opens: the server knows the typed password. Its `R` is delivered to the application as the login
  response, the session of that branch becomes `current` (section 8.4 steps 3 and 4), and the client sends
  first, normally a confirmation.
- No branch opens: the server does not know that password. Either the password is wrong or this is not the
  server the user meant. Nothing was revealed. The application sees a failed login through its timeout.

### 19.5 The server confirms

The first packet that opens under one of the candidate sessions proves that the client derived that `master`,
so it holds that password. The server then, and only then, grants the role of that branch, makes that session
`current` for the client, pins the client, feeds the generator and wipes the other candidates.

A `TXT_MSG` is tried against the offers after the three slots and before section 10.2 (table of 7.2), so a
client the server already knows can log in again without the server starting a handshake of its own. That
first packet is normally the client's confirmation, but it need not be: if the confirmation is lost, the
client's next text confirms the login just as well and is delivered. `previous` then holds the session the
login replaced, and what the server held for the client is sent (8.5).

### 19.6 Legacy logins and the downgrade

Old and new firmware keep working together: a client sends this login only to a pinned server and the legacy
login to any other; a server accepts both.

That compatibility is also the way around the protection. A server set up to steal a password simply does not
advertise the capability, and the client sends it a legacy login with the password in it. No protocol change
can prevent this while legacy logins are sent at all.

So there is a setting, changed by a CLI command (`set zcpfs.legacylogin on|off`, default `on`; locally on a
companion, also by an admin over the mesh on a server):

- on a **client**, `off` means a legacy login is never sent. A login to a server that is not pinned fails
  locally, at once, and the password does not leave the node. Only with this setting off does section 19
  protect against the wrong server. The client can then log in only to servers that run this protocol and
  have it on.
- on a **server**, `off` means legacy logins are dropped before they are read, so its password is never
  received in readable form. Clients without this protocol, and clients in `passive` or `off` mode (6.4),
  can then not log in.
- The setting acts only while the node's mode is `on`. A node in `passive` or `off` is leaving the protocol
  and logs in, or answers logins, the legacy way.
- With `zcpfs.legacylogin on`, a server with `zcpfs.enforce on` (6.2) still drops a legacy login from a client
  it has pinned: that is a downgrade, and the client has the zcpfs login. Clients it has not pinned log in
  the legacy way as before.

### 19.7 After the login

- Text in both directions (CLI commands and replies, room posts) uses the session.
- Requests and responses use section 18.
- If the server loses the session, section 10.2 applies in both directions: the server knows the client's key.
- Logging in again is always allowed and replaces the session.

### 19.8 Security notes

- **What the wrong server learns.** From the login packet, nothing: `Y_c` is a point with no checkable
  relation to the password. By answering with branches for guesses of its own it can test those guesses, at
  most two (one reply per login, 19.4), and sees the outcome only in whether the client goes on.
- **What a wrong client learns.** One guess per login, tested against every password the server would accept
  from it, and it must wait for the reply. A client the server does not know has `N_LOGIN` - 1 places, each
  good for one answer per `T_STEP` (an offer, and a replacement when that is a step old). With `N_LOGIN` = 2 and a
  one-branch reply in a flooded path return that is two per 18 s on SF7 at 62.5 kHz, about 9,600 guesses a
  day, and two per 40 s on SF8, about 4,300. Legacy has no limit. A password from a word list does not
  survive either.
- **Lockout.** Whoever keeps the places for unknown clients filled keeps other unknown clients from logging
  in, at the price of one login packet per `T_LOGIN` and place. He cannot keep out a client the server
  knows: that place cannot be taken without the client's key or a login of its that is newer than its last.
  A device the server has never seen can be kept out for as long as the sender keeps it up. The
  alternative, making room by dropping the oldest offer, would let him break logins that are under way.
- **Answers to wrong passwords.** Legacy is silent on a wrong password. Here the server answers every
  well-formed login, because it cannot tell. What that can cost the mesh is bounded by `R_LOGIN_FLOOD`: at
  most four flooded replies per 180 s from one server to clients it does not know, about 1,900 a day, 0.7 %
  of its airtime on SF7 and 1.7 % on SF8 with one-branch replies. For comparison: a legacy repeater with
  open guest access, the default, floods one reply for every flooded login with an empty password, without
  any limit.
- **A stranger during an attack.** With the budget spent by somebody else, a client the server does not
  know is answered over the reverse path only. If that path does not work in that direction, it cannot log
  in until the budget is back or its login arrives another way.
- **A recording** of a login does not allow guessing the password offline, whatever keys are stolen later:
  `y_c` and the `y_j` are gone, and without one of them `K` cannot be computed from a guess.
- **Forward secrecy** of the resulting session comes from `K`, which rests on two one-time scalars.
- **Static keys.** `ss` in `master` ties the session to both identities: knowing the password is not enough to
  log in as someone else's key.
- **The number of branches** in a reply to a typed password reveals whether a guest password is set, and
  whether an empty password is answered at all reveals whether the client is on the access list.
- **The flag** tells a server, a false one included, that the user typed nothing. An empty password is not a
  secret. Outsiders do not see the flag; it is sealed with the login.
- The server stores its passwords in readable form, as today. Whoever extracts the server's storage has them.
- A replayed login packet makes the server answer with fresh branches that nobody can open.
- Unconfirmed logins are bounded in number and time and live outside the server's client table, which they
  cannot fill: a client enters it when its login is confirmed.
- This is the CPace construction with this document's own key derivation and with several branches in one
  reply. It is not the CFRG draft byte for byte and it has not been reviewed.
