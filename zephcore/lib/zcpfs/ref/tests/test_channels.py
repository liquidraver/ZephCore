# SPDX-License-Identifier: MIT
"""SPEC section 17: the stateless mode for channels."""

import hashlib
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zcpfs  # noqa: E402
from sim import Sim, scalar  # noqa: E402
from zcpfs import GRP_DATA, GRP_TXT, chan_open, chan_seal  # noqa: E402

SECRET = hashlib.sha256(b"a private channel").digest()[:16]
OTHER = hashlib.sha256(b"another channel").digest()[:16]
N6 = bytes(range(6))


class ChannelSealing(unittest.TestCase):
    def test_round_trip(self):
        payload = chan_seal(SECRET, GRP_TXT, b"\x01\x02\x03\x04\x00alice: hi", N6)
        self.assertEqual(chan_open(SECRET, GRP_TXT, payload), b"\x01\x02\x03\x04\x00alice: hi")
        self.assertEqual(len(payload), len(b"\x01\x02\x03\x04\x00alice: hi") + 15)

    def test_wrong_secret_type_or_bits_fail(self):
        payload = chan_seal(SECRET, GRP_TXT, b"\x00" * 5 + b"hello", N6)
        self.assertIsNone(chan_open(OTHER, GRP_TXT, payload))
        self.assertIsNone(chan_open(SECRET, GRP_DATA, payload))        # the type is bound
        for i in (1, 6, 7, len(payload) - 1):                          # n6, ciphertext, tag
            bad = bytearray(payload)
            bad[i] ^= 1
            self.assertIsNone(chan_open(SECRET, GRP_TXT, bytes(bad)))

    def test_same_text_never_gives_the_same_packet(self):
        plain = b"\x00" * 5 + b"same text"
        a = chan_seal(SECRET, GRP_TXT, plain, bytes(6))
        b = chan_seal(SECRET, GRP_TXT, plain, b"\x01" + bytes(5))
        self.assertNotEqual(a[7:], b[7:])

    def test_never_block_aligned_and_within_the_limit(self):
        for n in range(0, zcpfs.CHAN_MAX_PLAIN + 1):
            plain = bytes([65 + n % 26]) * n
            payload = chan_seal(SECRET, GRP_TXT, plain, N6)
            self.assertNotEqual((len(payload) - 3) % 16, 0, n)       # legacy would see enc_len % 16 != 0
            self.assertLessEqual(len(payload), zcpfs.MAX_PAYLOAD + 1, n)
            self.assertEqual(chan_open(SECRET, GRP_TXT, payload)[:n], plain)
        with self.assertRaises(ValueError):
            chan_seal(SECRET, GRP_TXT, bytes(zcpfs.CHAN_MAX_PLAIN + 1), N6)

    def test_legacy_packets_are_not_tried_as_sealed_and_the_reverse(self):
        legacy = zcpfs.legacy_chan_seal(SECRET, b"\x00" * 5 + b"old style")
        self.assertEqual((len(legacy) - 3) % 16, 0)
        self.assertIsNone(chan_open(SECRET, GRP_TXT, legacy))
        sealed = chan_seal(SECRET, GRP_TXT, b"\x00" * 5 + b"new style", N6)
        self.assertIsNone(zcpfs.legacy_chan_open(SECRET, sealed))     # upstream's alignment check


class ChannelNodes(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(seed=30)
        self.tagged = self.sim.node(scalar(b"tagged"), True, b"tagged")
        self.fresh = self.sim.node(scalar(b"fresh"), True, b"fresh")      # new firmware, channel not tagged
        self.old = self.sim.node(scalar(b"old"), False, b"old")           # firmware without zcpfs
        for n in (self.tagged, self.fresh, self.old):
            n.add_channel(OTHER)
            n.add_channel(SECRET)
        self.tagged.set_channel_mode(1, "on")

    def say(self, node, text):
        self.sim.transmit(node, node.channel_send(1, 1_700_000_000, text))
        self.sim.run(self.sim.t + 10)

    def test_tag_controls_sending_only(self):
        self.say(self.tagged, b"sealed hello")
        self.assertEqual(self.fresh.channel_inbox, [(1, b"tagged: sealed hello", True)])
        self.assertEqual(self.old.channel_inbox, [])                  # old firmware cannot read it
        self.say(self.fresh, b"legacy reply")
        self.assertIn((1, b"fresh: legacy reply", False), self.tagged.channel_inbox)
        self.assertIn((1, b"fresh: legacy reply", False), self.old.channel_inbox)

    def test_every_sealed_message_is_a_flood_with_a_fresh_value(self):
        for i in range(5):
            self.say(self.tagged, b"again")
        packets = [p for p in self.sim.tap if p.ptype == GRP_TXT]
        self.assertEqual(len(packets), 5)
        self.assertTrue(all(p.flood for p in packets))
        self.assertEqual(len({p.payload[1:7] for p in packets}), 5)
        self.assertEqual(len([m for m in self.fresh.channel_inbox if m[1] == b"tagged: again"]), 5)

    def test_untagging_goes_back_to_legacy(self):
        self.tagged.set_channel_mode(1, "off")
        self.say(self.tagged, b"plain again")
        self.assertEqual(self.old.channel_inbox, [(1, b"tagged: plain again", False)])

    def test_strict_reads_sealed_only(self):
        self.fresh.set_channel_mode(1, "strict")
        self.say(self.tagged, b"sealed")                              # sealed: read
        self.say(self.old, b"legacy from old firmware")               # legacy: refused
        self.assertEqual(self.fresh.channel_inbox, [(1, b"tagged: sealed", True)])
        self.assertIn((1, b"old: legacy from old firmware", False), self.tagged.channel_inbox)
        self.say(self.fresh, b"from strict")                          # and it sends sealed
        self.assertIn((1, b"fresh: from strict", True), self.tagged.channel_inbox)
        self.assertNotIn(b"fresh: from strict", [m[1] for m in self.old.channel_inbox])

    def test_strict_is_per_channel(self):
        self.fresh.set_channel_mode(1, "strict")
        self.sim.transmit(self.old, self.old.channel_send(0, 1_700_000_000, b"other channel"))
        self.sim.run(self.sim.t + 10)
        self.assertEqual(self.fresh.channel_inbox, [(0, b"old: other channel", False)])

    def test_mode_is_validated(self):
        with self.assertRaises(ValueError):
            self.tagged.set_channel_mode(1, "maybe")

    def test_long_text_is_cut_to_fit(self):
        self.say(self.tagged, b"x" * 200)
        packet = [p for p in self.sim.tap if p.ptype == GRP_TXT][-1]
        self.assertLessEqual(len(packet.payload), zcpfs.MAX_PAYLOAD + 1)
        self.assertTrue(self.fresh.channel_inbox[-1][1].startswith(b"tagged: xxxx"))


if __name__ == "__main__":
    unittest.main()
