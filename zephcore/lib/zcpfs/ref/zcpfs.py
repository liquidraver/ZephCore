# SPDX-License-Identifier: MIT
"""Reference model of zcpfs, following ../SPEC.md section by section.

Two layers:

* the cryptographic core (KDF, DRBG, handshake, Session): pure functions and
  small classes, no I/O, no clock of their own;
* `Node`: the behaviour around it (slots, pinning, dispatch, recovery, the
  flood rule) for one device, driven by a caller that supplies time and
  carries packets. `sim.py` is such a caller.

Legacy MeshCore encryption is only imitated here (`legacy_seal`): it has the
right shape (2-byte MAC, block-aligned, zero-padded) but is not AES. The model
needs legacy packets to exist, not to be byte-exact.
"""

import hashlib
import hmac
from dataclasses import dataclass, field

import ascon
from curve import ed25519_public, edwards_to_montgomery, elligator2, x25519, x25519_public

TAG = 8
ZERO32 = bytes(32)

# MeshCore payload type codes used by the model.
REQ, RESPONSE, TXT, ACK, PATH = 0x00, 0x01, 0x02, 0x03, 0x08
GRP_TXT, GRP_DATA = 0x05, 0x06
ANON = 0x07

HINT_BYTE = 0x5A
MAX_PAYLOAD = 176


@dataclass
class Params:
    W: int = 16
    S: int = 4
    T_SKIP: float = 120
    T_IDLE: float = 24 * 3600
    T_MAX: float = 7 * 24 * 3600
    T_PREV: float = 60
    T_CONFIRM: float = 5
    K_UNACKED: int = 8
    N_ACK: int = 16
    F_MAX: int = 2 ** 20
    R_PRE_RATE: float = 4.0
    R_PRE_BURST: int = 16
    R_HS_COUNT: int = 3
    R_HS_WINDOW: float = 30
    T_RESYNC: float = 600
    T_HINT: float = 24 * 3600
    T_PROBE: float = 15
    # The step of section 10.4: 1.5 times the node's own send timeout towards the
    # contact, at least 5 s. The simulator's link is the same for everybody, so
    # one value stands for all contacts.
    T_STEP: float = 8
    N_HS_TRIES: int = 3
    N_HELD: int = 4
    N_DATA_TRIES: int = 2          # extra sends of a held message that stays unacknowledged
    T_DEDUP: float = 120
    N_LOGIN: int = 2               # unconfirmed logins a server keeps, and so answers, at once
    N_DEDUP: int = 16
    # What is flooded for a pinned contact with neither path nor session:
    # "probe" (14 bytes, handshake follows direct) or "handshake" (handshake 1
    # itself, the reply riding the flooded path return).
    FIRST_CONTACT: str = "probe"


# The four intervals section 10.4 derives from the step.
Params.T_HS_RETRY = property(lambda p: p.T_STEP)
Params.T_HS_WAIT = property(lambda p: 2 * p.T_STEP)
Params.T_HELD = property(lambda p: (p.N_HS_TRIES + 1) * p.T_STEP)
Params.T_DATA_RETRY = property(lambda p: p.T_STEP)
Params.T_LOGIN = property(lambda p: 2 * p.T_STEP)      # both the client's wait and the server's offer


# ---------------------------------------------------------------------------
# Section 3: key derivation and empty-message authentication
# ---------------------------------------------------------------------------

def kdf(label, *parts, n):
    message = b"".join(len(p).to_bytes(2, "little") + p for p in parts)
    return ascon.cxof128(message, b"zcpfs1/" + label.encode("ascii"), n)


def auth(k, ad):
    _, tag = ascon.aead_encrypt(k[:16], k[16:32], ad, b"")
    return tag[:TAG]


def ct_equal(a, b):
    return hmac.compare_digest(a, b)


# ---------------------------------------------------------------------------
# Section 5: random numbers (fast key erasure)
# ---------------------------------------------------------------------------

class Drbg:
    def __init__(self, hw, saved=ZERO32):
        self.k = kdf("drbg.seed", hw, saved, n=32)

    def generate(self, n):
        out = kdf("drbg.gen", self.k, n=32 + n)
        self.k = out[:32]
        return out[32:]

    def mix(self, x):
        self.k = kdf("drbg.mix", self.k, x, n=32)


# ---------------------------------------------------------------------------
# Section 4: identities
# ---------------------------------------------------------------------------

class Identity:
    def __init__(self, scalar):
        self.a = scalar[:32]
        self.S = ed25519_public(self.a)
        self.A = edwards_to_montgomery(self.S)

    def shared_secret(self, peer_S):
        return x25519(self.a, edwards_to_montgomery(peer_S))


# ---------------------------------------------------------------------------
# Section 8: handshake
# ---------------------------------------------------------------------------

@dataclass
class Outstanding:
    e: bytes
    E: bytes
    es: bytes


def hs1_make(drbg, ss, S_i, S_r):
    """Initiator, 8.2. Returns (Outstanding, 40-byte body) or None."""
    e = drbg.generate(32)
    E = x25519_public(e)
    es = x25519(e, edwards_to_montgomery(S_r))
    if ct_equal(es, ZERO32):
        return None
    k1 = kdf("hs1", ss, es, S_i, S_r, E, n=32)
    return Outstanding(e, E, es), E + auth(k1, b"\x01" + E)


def hs1_verify(ss, a_r, S_i, S_r, body):
    """Responder, 8.3 step 3. Returns es, or None."""
    E, tag = body[:32], body[32:40]
    es = x25519(a_r, E)
    if ct_equal(es, ZERO32):
        return None
    k1 = kdf("hs1", ss, es, S_i, S_r, E, n=32)
    return es if ct_equal(auth(k1, b"\x01" + E), tag) else None


def _session_keys(ss, es, ee, se, S_i, S_r, E_i, E_r):
    master = kdf("master", ss, es, ee, se, S_i, S_r, E_i, E_r, n=32)
    k2 = kdf("hs2", master, n=32)
    return (auth(k2, b"\x02" + E_r),
            kdf("chain.i2r", master, n=32),
            kdf("chain.r2i", master, n=32),
            kdf("drbg.feed", master, n=32))


def hs2_make(drbg, ss, es, S_i, S_r, E_i):
    """Responder, 8.3 step 6. Returns (body, ck_i2r, ck_r2i, feed) or None."""
    e = drbg.generate(32)
    E_r = x25519_public(e)
    ee = x25519(e, E_i)
    se = x25519(e, edwards_to_montgomery(S_i))
    if ct_equal(ee, ZERO32) or ct_equal(se, ZERO32):
        return None
    tag2, ck_i2r, ck_r2i, feed = _session_keys(ss, es, ee, se, S_i, S_r, E_i, E_r)
    return E_r + tag2, ck_i2r, ck_r2i, feed


def hs2_process(ss, out, a_i, S_i, S_r, body):
    """Initiator, 8.4. Returns (ck_i2r, ck_r2i, feed) or None."""
    E_r, tag = body[:32], body[32:40]
    ee = x25519(out.e, E_r)
    se = x25519(a_i, E_r)
    if ct_equal(ee, ZERO32) or ct_equal(se, ZERO32):
        return None
    tag2, ck_i2r, ck_r2i, feed = _session_keys(ss, out.es, ee, se, S_i, S_r, out.E, E_r)
    return (ck_i2r, ck_r2i, feed) if ct_equal(tag2, tag) else None


# ---------------------------------------------------------------------------
# Section 9: session
# ---------------------------------------------------------------------------

def step(ck):
    out = kdf("step", ck, n=80)
    return out[:32], out[32:48], out[48:64], out[64:80]


@dataclass
class Session:
    ck_s: bytes
    ck_r: bytes
    t_start: float
    t_last: float
    skipped: list = field(default_factory=list)   # (key, nonce, ackk, expiry)
    expect: list = field(default_factory=list)    # (ack4, token)
    unacked: int = 0
    fails: int = 0
    opened: bool = False                          # a packet has opened under it

    def seal(self, ad, plaintext, now, on_key=None):
        ck, key, nonce, ackk = step(self.ck_s)
        self.ck_s = ck                           # before any ciphertext exists
        if on_key:
            on_key(key, nonce)
        c, tag = ascon.aead_encrypt(key, nonce, ad, plaintext)
        body = c + tag[:TAG]
        self.t_last = now
        return body, kdf("ack", ackk, body, n=4)

    def open(self, ad, body, now, p):
        """Returns (plaintext, ack4), or None. Commits only on success."""
        if len(body) < TAG:
            return None
        c, tag = body[:-TAG], body[-TAG:]
        self.skipped = [s for s in self.skipped if s[3] > now]

        def done(plaintext, ackk, failed):
            self.fails += failed
            self.opened = True
            self.t_last = now
            return plaintext, kdf("ack", ackk, body, n=4)

        ck, key, nonce, ackk = step(self.ck_r)
        plaintext = ascon.aead_decrypt(key, nonce, ad, c, tag)
        if plaintext is not None:
            self.ck_r = ck
            return done(plaintext, ackk, 0)

        for i, (skey, snonce, sackk, _) in enumerate(self.skipped):
            plaintext = ascon.aead_decrypt(skey, snonce, ad, c, tag)
            if plaintext is not None:
                del self.skipped[i]
                return done(plaintext, sackk, 1)

        passed = [(key, nonce, ackk, now + p.T_SKIP)]
        for _ in range(1, p.W):
            ck, key, nonce, ackk = step(ck)
            plaintext = ascon.aead_decrypt(key, nonce, ad, c, tag)
            if plaintext is not None:
                self.skipped = (self.skipped + passed)[-p.S:]
                self.ck_r = ck
                return done(plaintext, ackk, 1)
            passed.append((key, nonce, ackk, now + p.T_SKIP))

        self.fails += 1
        return None


# ---------------------------------------------------------------------------
# Sections 6.1 and 6.3: advert tag and padding hint
# ---------------------------------------------------------------------------

def advert_name_field(name, budget):
    """Name bytes plus the version-1 tag, within `budget` bytes."""
    return name[:budget - 1] + b"\x00"


def parse_name_field(data):
    """Returns (name, capability version or 0)."""
    i = data.find(b"\x00")
    if i < 0:
        return data, 0
    return data[:i], (data[i + 1] if i + 1 < len(data) else 1)


def text_plaintext(timestamp, attempt, text):
    p = timestamp.to_bytes(4, "little") + bytes([attempt & 3]) + text
    if attempt > 3:
        p += b"\x00" + bytes([attempt])
    return p


def with_hint(timestamp, attempt, text):
    """Legacy plaintext carrying the padding hint, or without it if it would
    cost another AES block (6.3)."""
    plain = text_plaintext(timestamp, attempt, text)
    hinted = (timestamp.to_bytes(4, "little") + bytes([attempt & 3]) + text
              + b"\x00" + bytes([attempt if attempt > 3 else 0, HINT_BYTE]))
    blocks = lambda n: max(1, -(-n // 16))
    return hinted if blocks(len(hinted)) <= blocks(len(plain)) else plain


def parse_text(data):
    """Returns (timestamp, flags, text, extended attempt, hint present)."""
    data = bytes(data) + b"\x00\x00\x00"
    end = data.index(b"\x00", 5)
    return (int.from_bytes(data[:4], "little"), data[4], data[5:end],
            data[end + 1], data[end + 2] == HINT_BYTE)


# ---------------------------------------------------------------------------
# Imitation of the legacy envelope (shape only, see module docstring)
# ---------------------------------------------------------------------------

def legacy_seal(ss, plaintext):
    padded = plaintext + bytes(-len(plaintext) % 16 or (16 if not plaintext else 0))
    stream = ascon.xof128(b"legacy-standin" + ss, len(padded))
    ct = bytes(a ^ b for a, b in zip(padded, stream))
    return hmac.new(ss, ct, hashlib.sha256).digest()[:2] + ct


def legacy_open(ss, body):
    mac, ct = body[:2], body[2:]
    if not ct or len(ct) % 16:
        return None                               # upstream PR 1809 check
    if not ct_equal(hmac.new(ss, ct, hashlib.sha256).digest()[:2], mac):
        return None
    stream = ascon.xof128(b"legacy-standin" + ss, len(ct))
    return bytes(a ^ b for a, b in zip(ct, stream))


def legacy_ack(plaintext, sender_S):
    _, _, text, _, _ = parse_text(plaintext)
    return hashlib.sha256(plaintext[:5 + len(text)] + sender_S).digest()[:4]


# ---------------------------------------------------------------------------
# Section 17: stateless mode for channels (no forward secrecy)
# ---------------------------------------------------------------------------

CHAN_MAX_PLAIN = MAX_PAYLOAD - 15      # hash 1 + n6 6 + tag 8


def chan_hash(secret):
    """MeshCore's 1-byte channel hash: SHA-256 of the 16-byte secret."""
    return hashlib.sha256(secret[:16]).digest()[:1]


def chan_field(secret):
    """The 32-byte secret field as MeshCore stores it: the secret, zero-filled."""
    return secret[:32] + bytes(32 - len(secret[:32]))


def chan_seal(secret, ptype, plaintext, n6):
    if len(plaintext) > CHAN_MAX_PLAIN:
        raise ValueError("channel plaintext too long")
    if (len(plaintext) + 12) % 16 == 0:
        plaintext += b"\x00"                      # never block-aligned
    h = chan_hash(secret)
    k = kdf("chan", chan_field(secret), n6, n=32)
    c, tag = ascon.aead_encrypt(k[:16], k[16:], h + bytes([ptype]) + n6, plaintext)
    return h + n6 + c + tag[:TAG]


def chan_open(secret, ptype, payload):
    """Returns the plaintext, or None. Only non-block-aligned payloads are
    sealed ones; block-aligned payloads are legacy and not tried here."""
    if len(payload) < 15 or (len(payload) - 3) % 16 == 0:
        return None
    h, n6, c, tag = payload[:1], payload[1:7], payload[7:-TAG], payload[-TAG:]
    if h != chan_hash(secret):
        return None
    k = kdf("chan", chan_field(secret), n6, n=32)
    return ascon.aead_decrypt(k[:16], k[16:], h + bytes([ptype]) + n6, c, tag)


def legacy_chan_seal(secret, plaintext):
    return chan_hash(secret) + legacy_seal(chan_field(secret), plaintext)


def legacy_chan_open(secret, payload):
    if payload[:1] != chan_hash(secret):
        return None
    return legacy_open(chan_field(secret), payload[1:])


# ---------------------------------------------------------------------------
# Section 7: allowed payload lengths
# ---------------------------------------------------------------------------

def fit(ptype, plain, overhead):
    """Append zero bytes to a plaintext until the payload it gives has an
    allowed length: never block-aligned, and never one of the lengths that
    mean a handshake or a probe. `overhead` is what the envelope adds."""
    while True:
        n = len(plain) + overhead
        if ptype == ANON:
            bad = (n - 35) % 16 == 0
        else:
            bad = ((n - 4) % 16 == 0 or (ptype == REQ and n in (14, 42))
                   or (ptype == RESPONSE and n == 42))
        if not bad:
            return plain
        plain += b"\x00"


def legacy_view(plain):
    """An opened plaintext as legacy handling expects it: zero-extended to a
    whole number of 16-byte blocks."""
    return plain + bytes(-len(plain) % 16 or (0 if plain else 16))


# ---------------------------------------------------------------------------
# Section 18: stateless pairwise mode (REQ, RESPONSE, PATH)
# ---------------------------------------------------------------------------

def pair_seal(ss, S_sender, S_receiver, ptype, plaintext, n6):
    hdr = bytes([S_receiver[0], S_sender[0]])
    plaintext = fit(ptype, plaintext, 16)
    k = kdf("pair", ss, S_sender, S_receiver, n6, n=32)
    c, tag = ascon.aead_encrypt(k[:16], k[16:], hdr + bytes([ptype]) + n6, plaintext)
    return hdr + n6 + c + tag[:TAG]


def pair_open(ss, S_sender, S_receiver, ptype, payload):
    if len(payload) < 16:
        return None
    hdr, n6, c, tag = payload[:2], payload[2:8], payload[8:-TAG], payload[-TAG:]
    k = kdf("pair", ss, S_sender, S_receiver, n6, n=32)
    return ascon.aead_decrypt(k[:16], k[16:], hdr + bytes([ptype]) + n6, c, tag)


# ---------------------------------------------------------------------------
# Section 19: login, a password-authenticated key exchange
# ---------------------------------------------------------------------------

LOGIN_EMPTY = 0x01        # flag in the login: the typed password is empty
LOGIN_RESPONSE = 6        # bytes of login response in a branch
BRANCH_TAG = 4            # a branch's tag only tells the client which branch is its own
BRANCH = 32 + LOGIN_RESPONSE + BRANCH_TAG
MAX_BRANCHES = 2


@dataclass
class LoginOut:
    y: bytes
    Y: bytes


def pake_generator(password, S_c, S_s):
    """The curve point both sides derive from the password and the two
    identities. Nothing about the password can be read from a multiple of it."""
    return elligator2(kdf("pake.gen", password, S_c, S_s, n=32))


def _pake_keys(ss, K, S_c, S_s, Y_c, Y_s):
    master = kdf("pake.master", ss, K, S_c, S_s, Y_c, Y_s, n=32)
    return (kdf("pake2", master, n=32), kdf("chain.i2r", master, n=32),
            kdf("chain.r2i", master, n=32), kdf("drbg.feed", master, n=32))


def pake1_make(drbg, ss, S_c, S_s, password, login_plain):
    """Client. Returns (LoginOut, body), body = S_c + Y_c + C + T8. The
    password is not in the packet, only whether it is empty; `login_plain`
    is the rest of the legacy login (timestamp, a room's sync marker)."""
    if ct_equal(ss, ZERO32):
        return None
    y = drbg.generate(32)
    Y = x25519(y, pake_generator(password, S_c, S_s))
    k1 = kdf("pake1", ss, S_c, S_s, Y, n=32)
    flags = bytes([0 if password else LOGIN_EMPTY])
    plain = fit(ANON, flags + login_plain, 1 + 32 + 32 + TAG)
    c, tag = ascon.aead_encrypt(k1[:16], k1[16:], bytes([S_s[0], ANON]) + S_c + Y, plain)
    return LoginOut(y, Y), S_c + Y + c + tag[:TAG]


def pake1_open(a_s, S_s, body):
    """Server. Returns (S_c, Y_c, ss, login plaintext, password is empty) or
    None. This proves the sender holds the static key of S_c; of the password
    it says only whether there is one."""
    if len(body) < 64 + TAG:
        return None
    S_c, Y, c, tag = body[:32], body[32:64], body[64:-TAG], body[-TAG:]
    ss = x25519(a_s, edwards_to_montgomery(S_c))
    if ct_equal(ss, ZERO32):
        return None
    k1 = kdf("pake1", ss, S_c, S_s, Y, n=32)
    plain = ascon.aead_decrypt(k1[:16], k1[16:], bytes([S_s[0], ANON]) + S_c + Y, c, tag)
    if plain is None or not plain:
        return None
    return S_c, Y, ss, plain[1:], bool(plain[0] & LOGIN_EMPTY)


def pake_branch_make(drbg, ss, S_c, S_s, Y_c, password, response_plain):
    """Server, for one password it would accept. Returns
    (branch, ck_i2r, ck_r2i, feed) or None; branch = Y_s + C + T4."""
    assert len(response_plain) == LOGIN_RESPONSE
    y = drbg.generate(32)
    Y_s = x25519(y, pake_generator(password, S_c, S_s))
    K = x25519(y, Y_c)
    if ct_equal(K, ZERO32):
        return None
    k2, ck_i2r, ck_r2i, feed = _pake_keys(ss, K, S_c, S_s, Y_c, Y_s)
    ad = bytes([S_c[0], S_s[0], RESPONSE]) + Y_s
    c, tag = ascon.aead_encrypt(k2[:16], k2[16:], ad, response_plain)
    return Y_s + c + tag[:BRANCH_TAG], ck_i2r, ck_r2i, feed


def pake_branch_open(ss, out, S_c, S_s, branch):
    """Client, one branch. Returns (response, ck_i2r, ck_r2i, feed) or None.
    It opens only if the server used the same password."""
    if len(branch) != BRANCH:
        return None
    Y_s, c, tag = branch[:32], branch[32:-BRANCH_TAG], branch[-BRANCH_TAG:]
    K = x25519(out.y, Y_s)
    if ct_equal(K, ZERO32):
        return None
    k2, ck_i2r, ck_r2i, feed = _pake_keys(ss, K, S_c, S_s, out.Y, Y_s)
    ad = bytes([S_c[0], S_s[0], RESPONSE]) + Y_s
    plain = ascon.aead_decrypt(k2[:16], k2[16:], ad, c, tag)
    return None if plain is None else (plain, ck_i2r, ck_r2i, feed)


LOGIN_ECHO = 2            # a login reply names its login: the first bytes of Y_c
ADVERT_MARK_SLACK = 60    # 6.2: a sealed text or request raises the advert mark to its time less this


def pake_reply_pack(branches, Y_c):
    """count, the login's echo, the branches, and zero bytes until the RESPONSE
    payload (two address bytes more) has an allowed length."""
    body = bytes([len(branches)]) + Y_c[:LOGIN_ECHO] + b"".join(branches)
    while (len(body) - 2) % 16 == 0 or len(body) + 2 == 42:
        body += b"\x00"
    return body


def pake_reply_unpack(body):
    """(echo, branches), or None if it is not a login reply."""
    if len(body) < 1 or not 1 <= body[0] <= MAX_BRANCHES:
        return None
    count = body[0]
    if len(body) < 1 + LOGIN_ECHO + count * BRANCH:
        return None
    at = 1 + LOGIN_ECHO
    branches = [body[at + i * BRANCH:at + (i + 1) * BRANCH] for i in range(count)]
    if len({bytes(b[:32]) for b in branches}) != count:
        return None                       # 19.3: one Y_s twice would be two tag tries under one key
    return body[1:at], branches


# ---------------------------------------------------------------------------
# Node: one device
# ---------------------------------------------------------------------------

@dataclass
class Packet:
    ptype: int
    payload: bytes
    flood: bool


class _Bucket:
    def __init__(self, rate, burst):
        self.rate, self.burst, self.level, self.at = rate, burst, float(burst), None

    def take(self, now):
        if self.at is not None:
            self.level = min(self.burst, self.level + (now - self.at) * self.rate)
        self.at = now
        if self.level < 1:
            return False
        self.level -= 1
        return True


class _Gap:
    """At most one event per `gap` seconds."""
    def __init__(self, gap):
        self.gap, self.at = gap, None

    def take(self, now):
        if self.at is not None and now - self.at < self.gap:
            return False
        self.at = now
        return True


class Peer:
    def __init__(self, S, ss, p):
        self.S, self.ss = S, ss
        self.has_path = False
        self.path_confirmed = True     # False while the peer may not have our reciprocal path
        self.pinned = False            # persisted
        self.advert_ts = None          # persisted
        self.login_stamp = 0           # server: timestamp of the last login answered for a known client
        self.reset_ram(p)

    def reset_ram(self, p):
        self.current = self.next = self.previous = None
        self.prev_deadline = None
        self.outstanding = []          # own handshake 1 attempts awaiting a reply, newest last
        self.outstanding_at = 0        # when the newest of them was sent
        self.login_out = None          # client: own login awaiting its reply (section 19)
        self.login_until = 0
        self.login_renew = 0           # server: from when a new login replaces the offer
        self.login_known = False       # server: the offer is to a client that was known
        self.login_candidates = []     # server: (Session, role, feed) awaiting confirmation
        self.login_deadline = 0
        self.held = []                 # (plaintext, token, expiry), oldest first, at most N_HELD
        self.pending = []              # [plaintext, token, extra sends so far, due time, session]
        self.recent = []               # (hash of plaintext, expiry) of delivered messages
        self.start_gap = _Gap(p.T_HS_RETRY)   # handshake 1 or probe started by a send
        self.hs_tries = 0              # handshake 1 messages since the last app attempt
        self.hint_pending = False
        self.legacy_expect = []        # (ack4, token)
        self.hs_limit = _Bucket(p.R_HS_COUNT / p.R_HS_WINDOW, p.R_HS_COUNT)
        self.resync_gap = _Gap(p.T_RESYNC)
        self.hint_gap = _Gap(p.T_HINT)
        self.probe_gap = _Gap(p.T_PROBE)
        self.confirm_gap = _Gap(p.T_CONFIRM)

    def slots(self):
        return [("current", self.current), ("next", self.next), ("previous", self.previous)]


class Node:
    """One device. `zcpfs=False` models firmware without the feature."""

    def __init__(self, scalar, hw_entropy, clock, params=None, zcpfs=True, name=b"node",
                 password=None, guest_password=None):
        self.id = Identity(scalar)
        self.p = params or Params()
        self.clock = clock
        self.zcpfs = zcpfs
        self.name = name
        self.peers = {}
        self.seed_file = ZERO32
        self._boot(hw_entropy)
        self.password = password       # set on a server (repeater, room): it accepts logins
        self.guest_password = guest_password
        self.acl = set()               # server: client keys that may log in with a blank password
        self.allow_legacy_login = True  # CLI `set login.legacy`: both as client and as server
        self.refused_legacy_logins = 0
        self.inbox = []                # (peer S, text, attempt)
        self.responses = []            # (peer S, response plaintext)
        self.requests_seen = []        # (peer S, request plaintext)
        self.logins = []               # client: (server S, login response)
        self.logged_in = []            # server: (client key, role) of confirmed logins
        self.channels = []             # {"secret": bytes, "mode": "off" | "on" | "strict"}
        self.channel_inbox = []        # (channel index, text, arrived sealed)
        self.confirmed = []            # tokens of acknowledged sends
        self.stats = {k: 0 for k in (
            "app_attempts", "floods_at_send_point", "floods_in_reply", "hs1_sent",
            "hs2_sent", "probes_sent", "confirmations_sent", "legacy_sent",
            "sealed_sent", "no_path_suppressed", "resync", "hint_handshakes",
            "stateless_sent")}
        self.key_log = []              # (key, nonce) of every seal, for the uniqueness test
        self.wakeups = []              # times at which the caller should call tick()
        self.held_for = None           # the app's timeout for the last send, if it was held
        self._token = 0

    # -- boot and contacts ---------------------------------------------------

    def _boot(self, hw_entropy):
        self.drbg = Drbg(hw_entropy, self.seed_file)
        self.seed_file = self.drbg.generate(32)
        self.pre_auth = _Bucket(self.p.R_PRE_RATE, self.p.R_PRE_BURST)

    def reboot(self, hw_entropy):
        """Sessions and timers are RAM; pins, paths and the seed file survive."""
        for peer in self.peers.values():
            peer.reset_ram(self.p)
        self._boot(hw_entropy)

    def add_contact(self, S, has_path=False):
        peer = Peer(S, self.id.shared_secret(S), self.p)
        peer.has_path = has_path
        self.peers[S] = peer
        return peer

    def advert(self, timestamp, budget=23):
        field_ = (advert_name_field(self.name, budget) if self.zcpfs
                  else self.name[:budget])
        return self.id.S, timestamp, field_

    def on_advert(self, S, timestamp, name_field):
        """6.2. The signature check is outside the model."""
        peer = self.peers.get(S) or self.add_contact(S)
        if peer.advert_ts is not None and timestamp <= peer.advert_ts:
            return False                          # not accepted: no effect at all
        peer.advert_ts = timestamp
        _, version = parse_name_field(name_field)
        if not self.zcpfs or ct_equal(peer.ss, ZERO32):
            return True
        if version == 1:
            peer.pinned = True
        elif peer.pinned:
            peer.pinned = False
            peer.reset_ram(self.p)
        return True

    # -- helpers -------------------------------------------------------------

    def _hdr(self, peer):
        return bytes([peer.S[0], self.id.S[0]])

    def _ad(self, dst, src):
        return bytes([dst, src, TXT])

    def _send_typed(self, peer, ptype, plain, flood, out, in_kind=False):
        """A REQ, RESPONSE or PATH in the right envelope (section 18): stateless
        sealing towards a pinned contact, or when answering something that
        arrived sealed; legacy otherwise."""
        if self.zcpfs and (peer.pinned or in_kind) and not ct_equal(peer.ss, ZERO32):
            self.stats["stateless_sent"] += 1
            payload = pair_seal(peer.ss, self.id.S, peer.S, ptype, plain, self.drbg.generate(6))
            self._emit(out, ptype, payload, flood, zc=True)
        else:
            self._emit(out, ptype, self._hdr(peer) + legacy_seal(peer.ss, plain), flood)

    def _emit(self, out, ptype, payload, flood, zc=False):
        if zc:                                    # section 7: rules for every zcpfs packet
            assert (len(payload) - 4) % 16 != 0, "zcpfs payload is block-aligned"
            assert len(payload) <= MAX_PAYLOAD, "zcpfs payload too long"
        out.append(Packet(ptype, payload, flood))

    def _expire(self, peer, now):
        for name, s in peer.slots():
            if s and (now - s.t_last > self.p.T_IDLE or now - s.t_start > self.p.T_MAX
                      or s.fails >= self.p.F_MAX):
                setattr(peer, name, None)
        if peer.previous and peer.prev_deadline is not None and now >= peer.prev_deadline:
            peer.previous, peer.prev_deadline = None, None
        if peer.login_candidates and now >= peer.login_deadline:
            peer.login_candidates = []
        if peer.login_out and now >= peer.login_until:
            peer.login_out = None                 # 19.2: the application has stopped waiting
        if peer.outstanding and now - peer.outstanding_at >= self.p.T_HS_WAIT:
            peer.outstanding = []                 # 8.2: the ephemeral keys do not wait for ever
        peer.held = [h for h in peer.held if h[2] > now]   # 10.1

    def tick(self):
        """Timers. Returns packets: the only thing sent from here is a repeated
        handshake 1 for messages that are still held (10.1), direct only."""
        now = self.clock()
        out = []
        for peer in self.peers.values():
            self._expire(peer, now)
            if (peer.held and not peer.current and peer.has_path
                    and 0 < peer.hs_tries < self.p.N_HS_TRIES and peer.start_gap.take(now)):
                self._send_hs1(peer, out)
            self._resend_pending(peer, now, out)
        return out

    def _resend_pending(self, peer, now, out):
        """10.3: a message that waited for a handshake is sealed and sent again
        while it stays unacknowledged. Direct only, bounded."""
        keep = []
        for entry in peer.pending:
            plain, token, sends, due, session = entry
            if token in self.confirmed or session is not peer.current:
                continue                          # only under the session it was first sent in
            if due <= now:
                if sends >= self.p.N_DATA_TRIES or not peer.has_path:
                    continue
                self._send_data(peer, plain, token, out)
                entry[2], entry[3] = sends + 1, now + self.p.T_DATA_RETRY
                self.wakeups.append(entry[3])
            keep.append(entry)
        peer.pending = keep

    def _new_token(self):
        self._token += 1
        return self._token

    # -- sending -------------------------------------------------------------

    def app_send(self, S, text, timestamp, attempt=0):
        """One send attempt by the application. Returns (token, packets).

        The token is what the phone app is given as its 'expected ACK'; the
        on-air ACK value is mapped to it inside the node.
        """
        now = self.clock()
        peer = self.peers[S]
        out = []
        token = self._new_token()
        self.stats["app_attempts"] += 1
        self.held_for = None

        if not (self.zcpfs and peer.pinned):
            plain = (with_hint(timestamp, attempt, text) if self.zcpfs
                     else text_plaintext(timestamp, attempt, text))
            peer.legacy_expect = (peer.legacy_expect
                                  + [(legacy_ack(plain, self.id.S), token)])[-self.p.N_ACK:]
            flood = not peer.has_path
            self.stats["legacy_sent"] += 1
            self.stats["floods_at_send_point"] += flood
            self._emit(out, TXT, self._hdr(peer) + legacy_seal(peer.ss, plain), flood)
            return token, out

        self._expire(peer, now)
        if peer.current and peer.current.unacked >= self.p.K_UNACKED:
            peer.current = None                   # 9.4: handshake before this send
        plain = text_plaintext(timestamp, attempt, text)
        if peer.current:
            self._send_data(peer, plain, token, out, at_send_point=True)
            return token, out

        self._hold(peer, plain, token)            # 10.1
        self.held_for = self.p.T_HELD             # 10.4: the app gives up when the hold ends
        peer.hs_tries = 0                         # a new attempt, a new budget
        if not peer.start_gap.take(now):
            peer.hs_tries = 1                     # one is already under way; keep retrying it
            self.wakeups.append(now + self.p.T_HS_RETRY)
            return token, out
        if peer.has_path:
            self._send_hs1(peer, out)
        elif self.p.FIRST_CONTACT == "handshake":
            self._send_hs1(peer, out, flood=True)
        else:                                     # section 11: the one flood point
            nonce4 = self.drbg.generate(4)
            kp = kdf("probe", peer.ss, self.id.S, peer.S, nonce4, n=32)
            self.stats["probes_sent"] += 1
            self.stats["floods_at_send_point"] += 1
            self._emit(out, REQ, self._hdr(peer) + nonce4 + auth(kp, b"\x03" + nonce4), True,
                       zc=True)
        return token, out

    def request(self, S, data):
        """An application request (telemetry, status ...). Returns packets."""
        peer = self.peers[S]
        out = []
        flood = not peer.has_path
        self.stats["app_attempts"] += 1
        self.stats["floods_at_send_point"] += flood
        self._send_typed(peer, REQ, data, flood, out)
        return out

    def login(self, S, password, timestamp):
        """Log in to a server (section 19). Returns packets."""
        peer = self.peers[S]
        out = []
        flood = not peer.has_path
        stamp = timestamp.to_bytes(4, "little")
        if self.zcpfs and peer.pinned:
            made = pake1_make(self.drbg, peer.ss, self.id.S, peer.S, password, stamp)
            if made is None:
                return out
            peer.login_out, body = made
            peer.login_until = self.clock() + self.p.T_LOGIN
            self.held_for = self.p.T_LOGIN        # the application's timeout for this login
            payload = bytes([peer.S[0]]) + body
            assert (len(payload) - 35) % 16 != 0 and len(payload) <= MAX_PAYLOAD
            self._emit(out, ANON, payload, flood)
        elif not (self.zcpfs and not self.allow_legacy_login):
            self._emit(out, ANON, bytes([peer.S[0]]) + self.id.S
                       + legacy_seal(peer.ss, stamp + password), flood)
        else:
            self.refused_legacy_logins += 1       # `set login.legacy off`: the password stays here
            return out
        self.stats["app_attempts"] += 1
        self.stats["floods_at_send_point"] += flood
        return out

    def _hold(self, peer, plain, token):
        """Keep a message until a session exists. A retry of the same message
        (same timestamp and text) replaces the earlier attempt."""
        same = lambda other: other[:4] == plain[:4] and parse_text(other)[2] == parse_text(plain)[2]
        peer.held = ([h for h in peer.held if not same(h[0])]
                     + [(plain, token, self.clock() + self.p.T_HELD)])
        peer.held = peer.held[-self.p.N_HELD:]

    def _send_held(self, peer, out):
        """The side whose session just became current sends what it held, in
        order, or a confirmation if it is the initiator and holds nothing."""
        held, peer.held = peer.held, []
        peer.hs_tries = 0
        due = self.clock() + self.p.T_DATA_RETRY
        for plain, token, _ in held:
            self._send_data(peer, plain, token, out)
            peer.pending.append([plain, token, 0, due, peer.current])
        if held:
            self.wakeups.append(due)
        return bool(held)

    def _send_data(self, peer, plain, token, out, at_send_point=False):
        now = self.clock()
        flood = not peer.has_path
        if flood and not at_send_point:
            self.stats["no_path_suppressed"] += 1  # flood rule
            return
        if plain and (len(plain) + 6) % 16 == 0:
            plain += b"\x00"                      # 9.3: never block-aligned
        s = peer.current
        body, ack4 = s.seal(self._ad(peer.S[0], self.id.S[0]), plain, now,
                            on_key=lambda k, n: self.key_log.append((k, n)))
        if plain:
            s.expect = (s.expect + [(ack4, token)])[-self.p.N_ACK:]
            s.unacked += 1
            self.stats["sealed_sent"] += 1
            self.stats["floods_at_send_point"] += flood
        else:
            self.stats["confirmations_sent"] += 1
        self._emit(out, TXT, self._hdr(peer) + body, flood, zc=True)

    def _send_hs1(self, peer, out, flood=False):
        """`flood` is only ever True at the application's send point."""
        if not flood and not peer.has_path:
            self.stats["no_path_suppressed"] += 1
            return
        made = hs1_make(self.drbg, peer.ss, self.id.S, peer.S)
        if made is None:
            return
        entry, body = made
        if not flood and not peer.path_confirmed:  # the peer may have missed our reciprocal path
            self._send_typed(peer, PATH, b"\x01", False, out)
        peer.outstanding = (peer.outstanding + [entry])[-self.p.N_HS_TRIES:]
        peer.outstanding_at = self.clock()
        peer.start_gap.at = self.clock()          # 10.1: T_HS_RETRY counts from the newest
        peer.hs_tries += 1
        self.wakeups.append(self.clock() + self.p.T_HS_RETRY)
        self.stats["hs1_sent"] += 1
        self.stats["floods_at_send_point"] += flood
        self._emit(out, REQ, self._hdr(peer) + body, flood, zc=True)

    def _send_ack(self, peer, ack4, attempt, was_flood, out, sealed=False):
        """A sealed message is acknowledged with the 4 keyed bytes alone (9.5).
        Legacy appends the attempt and a random byte to keep ACK packets apart;
        a sealed message's value is already unique."""
        ack = ack4 if sealed else ack4 + bytes([attempt]) + self.drbg.generate(1)
        if was_flood:                             # legacy: path return carrying the ACK
            self.stats["floods_in_reply"] += 1
            self._send_typed(peer, PATH, b"\x01" + ack, True, out, in_kind=sealed)
        else:
            flood = not peer.has_path             # legacy sendAckTo floods without a path
            self.stats["floods_in_reply"] += flood
            self._emit(out, ACK, ack, flood)

    # -- channels (section 17) ----------------------------------------------

    def add_channel(self, secret, mode="off"):
        self.channels.append({"secret": secret, "mode": "off"})
        self.set_channel_mode(len(self.channels) - 1, mode)
        return len(self.channels) - 1

    def set_channel_mode(self, index, mode):
        """The CLI command (17.3). off: send legacy, read both. on: send sealed,
        read both. strict: send sealed, read sealed only."""
        if mode not in ("off", "on", "strict"):
            raise ValueError("channel mode must be off, on or strict")
        self.channels[index]["mode"] = mode

    def channel_send(self, index, timestamp, text):
        ch = self.channels[index]
        sealed = self.zcpfs and ch["mode"] != "off"
        limit = CHAN_MAX_PLAIN if sealed else 165
        plain = (timestamp.to_bytes(4, "little") + b"\x00" + self.name + b": " + text)[:limit]
        if sealed:
            payload = chan_seal(ch["secret"], GRP_TXT, plain, self.drbg.generate(6))
        else:
            payload = legacy_chan_seal(ch["secret"], plain)
        return [Packet(GRP_TXT, payload, True)]

    def _on_channel(self, ptype, payload):
        for index, ch in enumerate(self.channels):
            plain = chan_open(ch["secret"], ptype, payload) if self.zcpfs else None
            was_sealed = plain is not None
            if plain is None and not (self.zcpfs and ch["mode"] == "strict"):
                plain = legacy_chan_open(ch["secret"], payload)
            if plain is not None:
                text = plain[5:].split(b"\x00")[0]
                self.channel_inbox.append((index, text, was_sealed))
                return

    # -- receiving -----------------------------------------------------------

    def receive(self, ptype, payload, flood):
        """A packet that passed MeshCore's duplicate check. Returns packets."""
        out = []
        if ptype in (GRP_TXT, GRP_DATA):
            self._on_channel(ptype, payload)
            return out
        if ptype == ACK:
            if len(payload) >= 4:
                self._on_ack(payload[:4], flood, out)
            return out
        if ptype == ANON:
            if len(payload) > 35 and payload[0] == self.id.S[0]:
                self._on_login(payload, flood, out)
            return out
        if ptype not in (REQ, RESPONSE, TXT, PATH) or len(payload) <= 4:
            return out
        if payload[0] != self.id.S[0]:
            return out
        for peer in [q for q in self.peers.values() if q.S[0] == payload[1]]:
            if self._dispatch(peer, ptype, payload, flood, out):
                break
        return out

    def _dispatch(self, peer, ptype, payload, flood, out):
        """7.2, for one candidate contact. True if the packet was consumed."""
        now = self.clock()
        body = payload[2:]
        aligned = (len(payload) - 4) % 16 == 0
        if self.zcpfs and not ct_equal(peer.ss, ZERO32):
            self._expire(peer, now)
            if ptype == TXT:
                if any(s for _, s in peer.slots()) or peer.login_candidates:
                    ad = self._ad(payload[0], payload[1])
                    for name, s in peer.slots():
                        got = s.open(ad, body, now, self.p) if s else None
                        if got:
                            self._on_opened(peer, name, got[0], got[1], flood, out)
                            return True
                    for session, role, feed in peer.login_candidates:
                        got = session.open(ad, body, now, self.p)
                        if got:                   # 19.4: this proves the client's password
                            self._login_confirmed(peer, session, role, feed)
                            self._on_opened(peer, "current", got[0], got[1], flood, out)
                            return True
                if peer.pinned:
                    self._resync(peer, out)       # 10.2
                    return False
            elif ptype == REQ and len(payload) == 42:
                return self._on_hs1(peer, body, flood, out)
            elif ptype == REQ and len(payload) == 14:
                return self._on_probe(peer, body, out)
            elif ptype == RESPONSE and len(payload) == 42:
                return bool(peer.outstanding) and self._on_hs2(peer, body, out)
            elif not aligned:
                plain = pair_open(peer.ss, peer.S, self.id.S, ptype, payload)   # section 18, first
                if plain is not None:
                    return self._handle(peer, ptype, legacy_view(plain), plain, flood, out, True)
                if ptype == RESPONSE and peer.login_out:
                    return self._on_login_reply(peer, body, out)
                return False
            elif peer.pinned:
                return False                      # nothing legacy from a pinned contact
        plain = legacy_open(peer.ss, body)
        if plain is None:
            return False
        return self._handle(peer, ptype, plain, plain, flood, out, False)

    def _handle(self, peer, ptype, plain, exact, flood, out, sealed):
        """A decrypted REQ, RESPONSE, PATH or legacy TXT. `plain` is the legacy
        view (zero-padded); `exact` is what was sealed, where that is known."""
        if ptype == PATH:
            return self._on_path(peer, exact, flood, out, sealed)
        if ptype == REQ:
            if sealed:
                self._raise_mark(peer, exact)
            self.requests_seen.append((peer.S, plain.rstrip(b"\x00")))
            reply = b"re:" + plain.rstrip(b"\x00")
            if flood:                             # legacy: a path return carrying the response
                self.stats["floods_in_reply"] += 1
                self._send_typed(peer, PATH, b"\x03" + reply, True, out, in_kind=sealed)
            else:
                flood_reply = not peer.has_path
                self.stats["floods_in_reply"] += flood_reply
                self._send_typed(peer, RESPONSE, reply, flood_reply, out, in_kind=sealed)
            return True
        if ptype == RESPONSE:
            self.responses.append((peer.S, plain.rstrip(b"\x00")))
            return True
        return self._on_legacy_text(peer, plain, flood, out)

    def _raise_mark(self, peer, plain):
        """6.2: the sender's time, authenticated, less the slack, raises the
        advert mark, so an advert signed before it is no longer accepted."""
        t = int.from_bytes(bytes(plain[:4]).ljust(4, b"\x00"), "little")
        if t > ADVERT_MARK_SLACK and (peer.advert_ts is None or t - ADVERT_MARK_SLACK > peer.advert_ts):
            peer.advert_ts = t - ADVERT_MARK_SLACK

    def _on_opened(self, peer, slot, plain, ack4, flood, out):
        now = self.clock()
        if slot == "current":
            peer.current.unacked = 0              # 9.5: the peer still holds the session
            if peer.previous and peer.prev_deadline is None:
                peer.prev_deadline = now + self.p.T_PREV
        elif slot == "next":                      # 8.5
            peer.previous, peer.current, peer.next = peer.current, peer.next, None
            peer.prev_deadline = now + self.p.T_PREV if peer.previous else None
            peer.pinned = True
            self._send_held(peer, out)            # e.g. after losing a simultaneous open
        elif peer.current and not peer.current.opened and peer.confirm_gap.take(now):
            self._send_data(peer, b"", None, out)  # peer has not switched yet
        if not plain:
            return                                # a confirmation
        self._raise_mark(peer, plain)
        _, flags, text, ext, _ = parse_text(plain)
        attempt = ext if ext > 3 else flags & 3
        peer.recent = [r for r in peer.recent if r[1] > now]
        digest = kdf("dedup", plain, n=8)
        if all(r[0] != digest for r in peer.recent):  # a resend of the same message is
            self.inbox.append((peer.S, text, attempt))  # acknowledged but delivered once
            peer.recent = (peer.recent + [(digest, now + self.p.T_DEDUP)])[-self.p.N_DEDUP:]
        self._send_ack(peer, ack4, ext, flood, out, sealed=True)

    def _on_hs1(self, peer, body, flood, out):
        now = self.clock()
        if not self.pre_auth.take(now):
            return True
        es = hs1_verify(peer.ss, self.id.a, peer.S, self.id.S, body)
        if es is None:
            return False                          # maybe another contact's
        if not peer.hs_limit.take(now):
            return True
        if peer.outstanding:                      # simultaneous open
            if self.id.S < peer.S:
                return True
            peer.outstanding = []
        if flood:
            if not peer.probe_gap.take(now):      # one flooded answer per T_PROBE
                return True
        elif not peer.has_path:
            self.stats["no_path_suppressed"] += 1
            return True
        made = hs2_make(self.drbg, peer.ss, es, peer.S, self.id.S, body[:32])
        if made is None:
            return True
        body2, ck_i2r, ck_r2i, feed = made
        self.drbg.mix(feed)
        peer.next = Session(ck_s=ck_r2i, ck_r=ck_i2r, t_start=now, t_last=now)
        self.stats["hs2_sent"] += 1
        if flood:                                 # as legacy answers a flooded request:
            self.stats["floods_in_reply"] += 1    # a path return with the response as its extra
            self._send_typed(peer, PATH, b"\x02" + body2, True, out, in_kind=True)
        else:
            self._emit(out, RESPONSE, self._hdr(peer) + body2, False, zc=True)
        return True

    def _become_initiator_side(self, peer, ck_i2r, ck_r2i, feed, direct, out):
        """8.4 steps 2 to 4, shared by the handshake and the login."""
        now = self.clock()
        self.drbg.mix(feed)
        if direct:
            peer.path_confirmed = True
        peer.previous, peer.prev_deadline = peer.current, None
        peer.current = Session(ck_s=ck_i2r, ck_r=ck_r2i, t_start=now, t_last=now)
        peer.next, peer.outstanding, peer.login_out = None, [], None
        peer.pinned = True
        if not self._send_held(peer, out):        # the initiator sends first
            self._send_data(peer, b"", None, out)

    def _on_hs2(self, peer, body, out, direct=True):
        got = None
        for entry in reversed(peer.outstanding):  # a reply to any attempt still in flight
            got = hs2_process(peer.ss, entry, self.id.a, self.id.S, peer.S, body)
            if got:
                break
        if got is None:
            return False
        self._become_initiator_side(peer, got[0], got[1], got[2], direct, out)
        return True

    def _on_probe(self, peer, body, out):
        nonce4, tag = body[:4], body[4:12]
        kp = kdf("probe", peer.ss, peer.S, self.id.S, nonce4, n=32)
        if not ct_equal(auth(kp, b"\x03" + nonce4), tag):
            return False
        if peer.probe_gap.take(self.clock()):
            self.stats["floods_in_reply"] += 1
            self._send_typed(peer, PATH, b"\x01", True, out, in_kind=True)
        return True

    def _resync(self, peer, out):
        if peer.outstanding or not peer.has_path:
            return
        if peer.resync_gap.take(self.clock()):
            self.stats["resync"] += 1
            self._send_hs1(peer, out)

    def _on_path(self, peer, plain, flood, out, sealed):
        """A path return. The model's stand-in for its content is one kind byte
        and an extra: 1 = nothing or an ACK, 2 = handshake 2, 3 = a response,
        4 = a login reply (length byte, then the body)."""
        peer.has_path = True
        peer.path_confirmed = not flood
        kind = plain[0] if plain else 1
        if kind == 1 and any(plain[1:5]):
            self._on_ack(plain[1:5], False, out)
        if flood:                                 # reciprocal path, direct, in kind
            self._send_typed(peer, PATH, b"\x01", False, out, in_kind=sealed)
        if kind == 2 and self.zcpfs and peer.outstanding:
            self._on_hs2(peer, plain[1:41], out, direct=not flood)
            return True
        if kind == 3:
            self.responses.append((peer.S, plain[1:].rstrip(b"\x00")))
            return True
        if kind == 4 and self.zcpfs and peer.login_out:
            self._on_login_reply(peer, plain[2:2 + plain[1]], out, direct=not flood)
            return True
        if self.zcpfs and not peer.current and not peer.outstanding:
            if (peer.pinned and peer.held) or peer.hint_pending:
                self.stats["hint_handshakes"] += peer.hint_pending
                peer.hint_pending = False
                self._send_hs1(peer, out)
        return True

    def _on_legacy_text(self, peer, plain, flood, out):
        now = self.clock()
        if self.zcpfs and peer.pinned:
            return True                           # never accept legacy text from a pinned contact
        _, flags, text, ext, hint = parse_text(plain)
        self.inbox.append((peer.S, text, ext if ext > 3 else flags & 3))
        self._send_ack(peer, legacy_ack(plain, peer.S), ext, flood, out)
        if (self.zcpfs and hint and not peer.outstanding and not peer.current
                and peer.hint_gap.take(now)):
            if peer.has_path:
                self.stats["hint_handshakes"] += 1
                self._send_hs1(peer, out)
            else:
                peer.hint_pending = True
        return True

    # -- login (section 19) --------------------------------------------------

    def _on_login(self, payload, flood, out):
        """Server side."""
        if self.password is None:
            return
        now = self.clock()
        body = payload[1:]
        stamp = int(now).to_bytes(4, "little")
        if not (self.zcpfs and (len(payload) - 35) % 16 != 0):
            self._on_legacy_login(body, stamp, flood, out)
            return
        if not self.pre_auth.take(now):
            return
        got = pake1_open(self.id.a, self.id.S, body)
        if got is None:
            return
        S_c, Y_c, ss, login_plain, empty = got
        peer = self.peers.get(S_c)
        known = S_c in self.acl or (peer is not None and peer.pinned)
        # A client the server knows cannot have an old login of its replayed.
        sent = int.from_bytes(login_plain[:4], "little")
        if known and peer is not None and sent <= peer.login_stamp:
            return
        # 19.3 step 2: a login is answered only while there is room to keep its
        # offer; that is the limit on password guesses and on reply airtime.
        for other in self.peers.values():
            self._expire(other, now)
        if peer is not None and peer.login_candidates and now < peer.login_renew:
            return                                # its offer is not yet a step old
        waiting = [q for q in self.peers.values() if q.login_candidates and q.S != S_c]
        strangers = [q for q in waiting if not q.login_known]
        if len(waiting) >= self.p.N_LOGIN or (
                not known and len(strangers) >= max(1, self.p.N_LOGIN - 1)):
            return                                # the last place is for a known client
        # Every password this client could be using. The server cannot know
        # which one, or whether any, before the client confirms. It knows
        # whether one was typed at all, which keeps this to two.
        accepted = []
        if empty:
            if S_c in self.acl:
                accepted.append((b"known", b""))
            elif self.guest_password == b"":
                accepted.append((b"guest", b""))
        else:
            accepted.append((b"admin", self.password))
            if self.guest_password:
                accepted.append((b"guest", self.guest_password))
        if not accepted:
            return
        peer = peer or self.add_contact(S_c)
        peer.login_renew = now + self.p.T_STEP
        peer.login_known, peer.login_stamp = known, sent
        peer.login_candidates = []
        branches = []
        for role, password in accepted:
            made = pake_branch_make(self.drbg, ss, S_c, self.id.S, Y_c, password, stamp + role[:2])
            if made is None:
                continue
            branches.append(made[0])
            peer.login_candidates.append(
                (Session(ck_s=made[2], ck_r=made[1], t_start=now, t_last=now), role, made[3]))
        if not branches:
            return
        peer.login_deadline = now + self.p.T_LOGIN
        reply = pake_reply_pack(branches, Y_c)
        if flood:                                 # legacy: a path return carrying the response
            self.stats["floods_in_reply"] += 1
            self._send_typed(peer, PATH, b"\x04" + bytes([len(reply)]) + reply, True, out,
                             in_kind=True)
        else:
            flood_reply = not peer.has_path
            self.stats["floods_in_reply"] += flood_reply
            self._emit(out, RESPONSE, self._hdr(peer) + reply, flood_reply, zc=True)

    def _on_legacy_login(self, body, stamp, flood, out):
        """The password arrives readable. A wrong one is answered with silence."""
        if self.zcpfs and not self.allow_legacy_login:
            self.refused_legacy_logins += 1
            return
        S_c = body[:32]
        ss = self.id.shared_secret(S_c)
        plain = legacy_open(ss, body[32:])
        if plain is None:
            return
        password = plain[4:].rstrip(b"\x00")
        if ct_equal(password, self.password):
            role = b"admin"
        elif self.guest_password is not None and ct_equal(password, self.guest_password):
            role = b"guest"
        elif password == b"" and S_c in self.acl:
            role = b"known"
        else:
            return
        peer = self.peers.get(S_c) or self.add_contact(S_c)
        self.logged_in.append((S_c, role))
        if flood:
            self.stats["floods_in_reply"] += 1
            self._emit(out, PATH, self._hdr(peer) + legacy_seal(ss, b"\x03" + stamp + role), True)
        else:
            flood_reply = not peer.has_path
            self.stats["floods_in_reply"] += flood_reply
            self._emit(out, RESPONSE, self._hdr(peer) + legacy_seal(ss, stamp + role), flood_reply)

    def _login_confirmed(self, peer, session, role, feed):
        """Server: a packet opened under one candidate, so the client holds the
        password of that role. Only now is anything granted."""
        now = self.clock()
        self.drbg.mix(feed)
        peer.previous = peer.current
        peer.prev_deadline = now + self.p.T_PREV if peer.previous else None
        peer.current, peer.next, peer.login_candidates = session, None, []
        peer.pinned = True
        self.logged_in.append((peer.S, role))

    def _on_login_reply(self, peer, body, out, direct=True):
        """Client: one branch opens if, and only if, the server knows the
        password that was typed. One reply per login (19.4): a reply names
        its login, one for another login is ignored, and the first that names
        this one ends it, so a server gets no more guesses than its branches."""
        got = pake_reply_unpack(body)
        if got is None or got[0] != peer.login_out.Y[:LOGIN_ECHO]:
            return False
        branches = got[1]
        login, peer.login_out = peer.login_out, None
        for branch in branches:
            got = pake_branch_open(peer.ss, login, self.id.S, peer.S, branch)
            if got:
                plain, ck_i2r, ck_r2i, feed = got
                self.logins.append((peer.S, plain))
                self._become_initiator_side(peer, ck_i2r, ck_r2i, feed, direct, out)
                return True
        return False

    # -- acknowledgements ----------------------------------------------------

    def _path_retry(self, peer, flood, out):
        """Legacy handleReturnPathRetry: an ACK that had to be flooded means the
        peer has no path to us, so the reciprocal path is sent again."""
        if flood and peer.has_path:
            self._send_typed(peer, PATH, b"\x01", False, out)

    def _on_ack(self, ack4, flood, out):
        """9.5: current sessions first, then previous, then legacy."""
        for slot in ("current", "previous"):
            for peer in self.peers.values():
                s = getattr(peer, slot)
                for i, (value, token) in enumerate(s.expect if s else []):
                    if ct_equal(value, ack4):
                        del s.expect[i]
                        self.confirmed.append(token)
                        if slot == "current":
                            s.unacked = 0
                        self._path_retry(peer, flood, out)
                        return
        for peer in self.peers.values():
            for i, (value, token) in enumerate(peer.legacy_expect):
                if ct_equal(value, ack4):
                    del peer.legacy_expect[i]
                    self.confirmed.append(token)
                    self._path_retry(peer, flood, out)
                    return
