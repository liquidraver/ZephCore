# SPDX-License-Identifier: MIT
"""The cryptographic core of the model, one spec rule per test."""

import hashlib
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zcpfs  # noqa: E402
from zcpfs import (Drbg, Identity, Params, Session, advert_name_field, auth,  # noqa: E402
                   hs1_make, hs1_verify, hs2_make, hs2_process, kdf,
                   parse_name_field, parse_text, text_plaintext, with_hint)

P = Params()
AD = bytes([0x11, 0x22, zcpfs.TXT])


def identity(label):
    return Identity(hashlib.sha256(b"test-identity:" + label).digest())


def handshake(seed=b"s"):
    """A complete handshake. Returns (initiator session, responder session)."""
    i, r = identity(b"i"), identity(b"r")
    ss = i.shared_secret(r.S)
    rng_i, rng_r = Drbg(seed + b"i"), Drbg(seed + b"r")
    out, body1 = hs1_make(rng_i, ss, i.S, r.S)
    es = hs1_verify(ss, r.a, i.S, r.S, body1)
    body2, ck_i2r, ck_r2i, _ = hs2_make(rng_r, ss, es, i.S, r.S, body1[:32])
    got = hs2_process(ss, out, i.a, i.S, r.S, body2)
    assert got[:2] == (ck_i2r, ck_r2i)
    return (Session(ck_s=ck_i2r, ck_r=ck_r2i, t_start=0, t_last=0),
            Session(ck_s=ck_r2i, ck_r=ck_i2r, t_start=0, t_last=0))


class Kdf(unittest.TestCase):
    def test_labels_separate(self):
        self.assertNotEqual(kdf("step", b"k", n=32), kdf("ack", b"k", n=32))

    def test_parts_are_length_framed(self):
        self.assertNotEqual(kdf("x", b"ab", b"c", n=32), kdf("x", b"a", b"bc", n=32))
        self.assertNotEqual(kdf("x", b"abc", n=32), kdf("x", b"abc", b"", n=32))

    def test_auth_depends_on_key_and_data(self):
        k = kdf("hs1", b"secret", n=32)
        self.assertEqual(len(auth(k, b"\x01abc")), 8)
        self.assertNotEqual(auth(k, b"\x01abc"), auth(k, b"\x02abc"))
        self.assertNotEqual(auth(k, b"\x01abc"), auth(kdf("hs2", b"secret", n=32), b"\x01abc"))


class Generator(unittest.TestCase):
    def test_deterministic_and_advancing(self):
        a, b = Drbg(b"hw", b"seed"), Drbg(b"hw", b"seed")
        first = a.generate(32)
        self.assertEqual(first, b.generate(32))
        self.assertNotEqual(first, a.generate(32))

    def test_state_is_replaced_on_every_call(self):
        g = Drbg(b"hw")
        before = g.k
        g.generate(1)
        self.assertNotEqual(before, g.k)

    def test_seed_file_and_mix_change_the_stream(self):
        self.assertNotEqual(Drbg(b"hw", b"one").generate(16), Drbg(b"hw", b"two").generate(16))
        a, b = Drbg(b"hw"), Drbg(b"hw")
        a.mix(b"peer entropy")
        self.assertNotEqual(a.generate(16), b.generate(16))


class Handshake(unittest.TestCase):
    def setUp(self):
        self.i, self.r, self.x = identity(b"i"), identity(b"r"), identity(b"x")
        self.ss = self.i.shared_secret(self.r.S)

    def test_shared_secret_is_symmetric(self):
        self.assertEqual(self.ss, self.r.shared_secret(self.i.S))

    def test_both_sides_derive_the_same_chains(self):
        a, b = handshake()
        self.assertEqual((a.ck_s, a.ck_r), (b.ck_r, b.ck_s))
        self.assertNotEqual(a.ck_s, a.ck_r)

    def test_fresh_ephemerals_give_fresh_keys(self):
        self.assertNotEqual(handshake(b"one")[0].ck_s, handshake(b"two")[0].ck_s)

    def test_tampered_handshake_1_is_rejected(self):
        _, body = hs1_make(Drbg(b"a"), self.ss, self.i.S, self.r.S)
        for i in (0, 31, 32, 39):
            bad = bytearray(body)
            bad[i] ^= 1
            self.assertIsNone(hs1_verify(self.ss, self.r.a, self.i.S, self.r.S, bytes(bad)))

    def test_handshake_1_is_bound_to_both_identities(self):
        _, body = hs1_make(Drbg(b"a"), self.ss, self.i.S, self.r.S)
        # verified as if it came from another contact
        self.assertIsNone(hs1_verify(self.x.shared_secret(self.r.S), self.r.a,
                                     self.x.S, self.r.S, body))
        # verified by a different responder
        self.assertIsNone(hs1_verify(self.i.shared_secret(self.x.S), self.x.a,
                                     self.i.S, self.x.S, body))

    def test_low_order_ephemeral_is_rejected(self):
        body = bytes(32) + bytes(8)
        self.assertIsNone(hs1_verify(self.ss, self.r.a, self.i.S, self.r.S, body))

    def test_tampered_handshake_2_is_rejected(self):
        out, body1 = hs1_make(Drbg(b"a"), self.ss, self.i.S, self.r.S)
        es = hs1_verify(self.ss, self.r.a, self.i.S, self.r.S, body1)
        body2 = hs2_make(Drbg(b"b"), self.ss, es, self.i.S, self.r.S, body1[:32])[0]
        for i in (0, 32, 39):
            bad = bytearray(body2)
            bad[i] ^= 1
            self.assertIsNone(hs2_process(self.ss, out, self.i.a, self.i.S, self.r.S, bytes(bad)))

    def test_reply_to_an_abandoned_ephemeral_fails(self):
        rng = Drbg(b"a")
        _, body1 = hs1_make(rng, self.ss, self.i.S, self.r.S)
        retry, _ = hs1_make(rng, self.ss, self.i.S, self.r.S)
        es = hs1_verify(self.ss, self.r.a, self.i.S, self.r.S, body1)
        body2 = hs2_make(Drbg(b"b"), self.ss, es, self.i.S, self.r.S, body1[:32])[0]
        self.assertIsNone(hs2_process(self.ss, retry, self.i.a, self.i.S, self.r.S, body2))

    def test_one_stolen_static_key_cannot_complete_a_session(self):
        # The attacker holds the responder's static key and forges handshake 1
        # as the initiator. It can make tag1, but not the session keys.
        e = hashlib.sha256(b"attacker ephemeral").digest()
        E = zcpfs.x25519_public(e)
        es = zcpfs.x25519(self.r.a, E)
        k1 = kdf("hs1", self.ss, es, self.i.S, self.r.S, E, n=32)
        forged = E + auth(k1, b"\x01" + E)
        self.assertIsNotNone(hs1_verify(self.ss, self.r.a, self.i.S, self.r.S, forged))
        body2, ck_i2r, _, _ = hs2_make(Drbg(b"b"), self.ss, es, self.i.S, self.r.S, E)
        E_r = body2[:32]
        # The attacker can compute ss, es and ee. `se` needs the initiator's
        # static scalar or the responder's ephemeral one; it has neither, so
        # every value within its reach gives other session keys.
        ee = zcpfs.x25519(e, E_r)
        A_i = zcpfs.edwards_to_montgomery(self.i.S)
        for guess in (zcpfs.x25519(e, A_i), zcpfs.x25519(self.r.a, E_r), ee, es):
            tag2, forged_chain, _, _ = zcpfs._session_keys(
                self.ss, es, ee, guess, self.i.S, self.r.S, E, E_r)
            self.assertNotEqual(forged_chain, ck_i2r)
            self.assertNotEqual(tag2, body2[32:])


class SessionRules(unittest.TestCase):
    def test_in_order(self):
        a, b = handshake()
        for i in range(5):
            body, ack = a.seal(AD, b"message %d" % i, 0)
            self.assertEqual(b.open(AD, body, 0, P), (b"message %d" % i, ack))
        self.assertEqual(b.fails, 0)

    def test_both_directions_are_independent(self):
        a, b = handshake()
        x, _ = a.seal(AD, b"to b", 0)
        y, _ = b.seal(AD, b"to a", 0)
        self.assertEqual(a.open(AD, y, 0, P)[0], b"to a")
        self.assertEqual(b.open(AD, x, 0, P)[0], b"to b")
        self.assertIsNone(a.open(AD, x, 0, P))   # own packet does not open

    def test_reordering_uses_skipped_keys(self):
        a, b = handshake()
        packets = [a.seal(AD, b"m%d" % i, 0)[0] for i in range(4)]
        for i in (0, 3, 1, 2):
            self.assertEqual(b.open(AD, packets[i], 0, P)[0], b"m%d" % i)
        self.assertEqual(b.skipped, [])

    def test_skipped_keys_expire(self):
        a, b = handshake()
        packets = [a.seal(AD, b"m%d" % i, 0)[0] for i in range(3)]
        self.assertIsNotNone(b.open(AD, packets[2], 0, P))
        self.assertIsNone(b.open(AD, packets[0], P.T_SKIP + 1, P))

    def test_only_the_most_recent_skipped_keys_are_kept(self):
        a, b = handshake()
        packets = [a.seal(AD, b"m%d" % i, 0)[0] for i in range(8)]
        self.assertIsNotNone(b.open(AD, packets[7], 0, P))
        self.assertEqual(len(b.skipped), P.S)
        self.assertIsNone(b.open(AD, packets[0], 0, P))      # dropped
        self.assertIsNotNone(b.open(AD, packets[6], 0, P))   # kept

    def test_window(self):
        a, b = handshake()
        packets = [a.seal(AD, b"m%d" % i, 0)[0] for i in range(P.W + 1)]
        self.assertIsNone(b.open(AD, packets[P.W], 0, P))            # position W: outside
        self.assertIsNotNone(b.open(AD, packets[P.W - 1], 0, P))     # position W-1: inside

    def test_replay_fails(self):
        a, b = handshake()
        body, _ = a.seal(AD, b"once", 0)
        self.assertIsNotNone(b.open(AD, body, 0, P))
        self.assertIsNone(b.open(AD, body, 0, P))

    def test_tampering_fails_and_changes_nothing_but_the_counter(self):
        a, b = handshake()
        body, _ = a.seal(AD, b"intact", 0)
        before = (b.ck_r, list(b.skipped))
        for i in (0, len(body) - 1):
            bad = bytearray(body)
            bad[i] ^= 1
            self.assertIsNone(b.open(AD, bytes(bad), 0, P))
        self.assertIsNone(b.open(bytes([0x11, 0x23, zcpfs.TXT]), body, 0, P))  # other address
        self.assertEqual((b.ck_r, b.skipped), before)
        self.assertEqual(b.fails, 3)
        self.assertEqual(b.open(AD, body, 0, P)[0], b"intact")

    def test_fails_counts_packets_that_opened_late(self):
        a, b = handshake()
        packets = [a.seal(AD, b"m%d" % i, 0)[0] for i in range(2)]
        b.open(AD, packets[1], 0, P)
        self.assertEqual(b.fails, 1)   # position 0 failed before position 1 opened

    def test_ack_is_bound_to_the_ciphertext(self):
        a, b = handshake()
        body, ack = a.seal(AD, b"hello", 0)
        self.assertEqual(b.open(AD, body, 0, P)[1], ack)
        a2, _ = handshake()
        other, ack2 = a2.seal(AD, b"hellp", 0)
        self.assertNotEqual(ack, ack2)

    def test_confirmation_is_an_empty_plaintext(self):
        a, b = handshake()
        body, _ = a.seal(AD, b"", 0)
        self.assertEqual(len(body), 8)
        self.assertEqual(b.open(AD, body, 0, P)[0], b"")

    def test_every_key_is_used_once(self):
        a, _ = handshake()
        seen = []
        for i in range(200):
            a.seal(AD, b"x", 0, on_key=lambda k, n: seen.append((k, n)))
        self.assertEqual(len(set(seen)), 200)

    def test_chain_key_is_replaced_before_use(self):
        a, _ = handshake()
        before = a.ck_s
        a.seal(AD, b"x", 0, on_key=lambda k, n: self.assertNotEqual(a.ck_s, before))


class Signalling(unittest.TestCase):
    def test_advert_tag(self):
        self.assertEqual(advert_name_field(b"alice", 23), b"alice\x00")
        self.assertEqual(parse_name_field(b"alice\x00"), (b"alice", 1))
        self.assertEqual(parse_name_field(b"alice\x00\x02"), (b"alice", 2))
        self.assertEqual(parse_name_field(b"alice"), (b"alice", 0))
        self.assertEqual(parse_name_field(b"\x00"), (b"", 1))        # empty name, still tagged

    def test_advert_tag_fits_the_budget(self):
        field = advert_name_field(b"n" * 40, 23)
        self.assertEqual(len(field), 23)
        self.assertEqual(parse_name_field(field), (b"n" * 22, 1))

    def test_hint_never_costs_a_block_and_sits_two_past_the_nul(self):
        with_it = 0
        for n in range(0, 161):
            text = b"t" * n
            for attempt in (0, 2, 4):
                if attempt > 3 and n > 158:
                    continue
                plain = text_plaintext(7, attempt, text)
                hinted = with_hint(7, attempt, text)
                self.assertEqual(-(-len(hinted) // 16), -(-len(plain) // 16))
                ts, flags, got, ext, hint = parse_text(hinted)
                self.assertEqual((ts, got), (7, text))
                self.assertEqual(ext, attempt if attempt > 3 else 0)   # byte at NUL + 1 untouched
                with_it += hint
                self.assertFalse(parse_text(plain)[4])
        self.assertGreater(with_it, 300)

    def test_data_payload_is_never_block_aligned_and_fits(self):
        a, _ = handshake()
        for n in range(0, 161):
            for attempt in (0, 4):
                if attempt > 3 and n > 158:
                    continue
                plain = text_plaintext(7, attempt, b"t" * n)
                if (len(plain) + 6) % 16 == 0:
                    plain += b"\x00"
                payload = 2 + len(a.seal(AD, plain, 0)[0])
                self.assertNotEqual((payload - 4) % 16, 0)
                self.assertLessEqual(payload, zcpfs.MAX_PAYLOAD)
                self.assertEqual(parse_text(plain)[2], b"t" * n)


if __name__ == "__main__":
    unittest.main()
