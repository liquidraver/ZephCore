# SPDX-License-Identifier: MIT
"""Whole-node behaviour over a lossy, reordering link. Every scenario also
checks the standing rules in `check_rules`."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim import App, Sim, delivered, pair, scalar  # noqa: E402
from zcpfs import ACK, ADVERT_MARK_SLACK, PATH, REQ, RESPONSE, TXT, Packet  # noqa: E402


def check_rules(test, sim, handshake_may_flood=False):
    """Rules that must hold in every run."""
    for t, src, ptype, length, flood in sim.air:
        if flood:
            # The flood rule: no handshake, confirmation or resync is ever flooded.
            # (In the experimental first-contact variant handshake 1 is the one
            # packet flooded at the send point; the count check below covers it.)
            test.assertFalse(ptype == REQ and length == 42 and not handshake_may_flood,
                             "handshake 1 flooded")
            test.assertFalse(ptype == RESPONSE, "handshake 2 flooded")
            test.assertFalse(ptype == TXT and length == 10, "confirmation flooded")
    for node in sim.nodes:
        test.assertLessEqual(node.stats["floods_at_send_point"], node.stats["app_attempts"])
        test.assertEqual(len(set(node.key_log)), len(node.key_log), "a key was used twice")
    keys = [k for node in sim.nodes for k in node.key_log]
    test.assertEqual(len(set(keys)), len(keys), "two nodes produced the same key")


def confirmed_were_delivered(test, app, receiver, sender):
    got = delivered(receiver, sender.id.S)
    for text, result in app.results.items():
        if result == "ok":
            test.assertIn(text, got)


class Scenarios(unittest.TestCase):
    def test_capable_pair_with_paths(self):
        sim = Sim(seed=1)
        a, b = pair(sim)
        app_a, app_b = App(sim, a), App(sim, b)
        for i in range(10):
            app_a.send(b.id.S, b"a to b %d" % i, when=5 + 20 * i)
            app_b.send(a.id.S, b"b to a %d" % i, when=15 + 20 * i)
        sim.run(600)
        self.assertEqual(set(app_a.results.values()) | set(app_b.results.values()), {"ok"})
        self.assertEqual(len(delivered(b, a.id.S)), 10)
        self.assertEqual(len(delivered(a, b.id.S)), 10)
        self.assertEqual(sim.count(RESPONSE, 42), 1)             # one handshake in all
        self.assertEqual(sim.count(ACK), 20)
        self.assertEqual(sim.count(ACK, 4), 20)                  # sealed messages: 4-byte ACKs
        self.assertEqual(sim.bytes_on_air(flood=True), 0)
        self.assertEqual(a.stats["legacy_sent"] + b.stats["legacy_sent"], 0)
        check_rules(self, sim)

    def test_first_contact_without_a_path_floods_only_a_probe(self):
        sim = Sim(seed=2)
        a, b = pair(sim, paths=False)
        app = App(sim, a, flood_last=False)
        app.send(b.id.S, b"first message, long enough to matter" * 3, when=5)
        sim.run(120)
        self.assertEqual(app.results, {b"first message, long enough to matter" * 3: "ok"})
        floods = [(ptype, n) for _, _, ptype, n, flood in sim.air if flood]
        self.assertIn((REQ, 14), floods)                         # the probe
        self.assertTrue(all(ptype != TXT for ptype, _ in floods))  # the message never floods
        self.assertLessEqual(sum(1 for f in floods if f == (REQ, 14)), app.tries[next(iter(app.tries))])
        check_rules(self, sim)

    def test_lossy_link(self):
        sim = Sim(seed=3, loss=0.3)
        a, b = pair(sim)
        app_a, app_b = App(sim, a), App(sim, b)
        for i in range(12):
            app_a.send(b.id.S, b"lossy a %d" % i, when=5 + 40 * i)
            app_b.send(a.id.S, b"lossy b %d" % i, when=25 + 40 * i)
        sim.run(1200)
        confirmed_were_delivered(self, app_a, b, a)
        confirmed_were_delivered(self, app_b, a, b)
        ok = sum(r == "ok" for app in (app_a, app_b) for r in app.results.values())
        self.assertGreaterEqual(ok, 16)                          # of 24, at 30 % loss
        self.assertEqual(a.stats["legacy_sent"] + b.stats["legacy_sent"], 0)
        check_rules(self, sim)

    def test_receiver_reboot_recovers_on_the_next_attempt(self):
        sim = Sim(seed=4)
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"before", when=5)
        sim.run(60)
        b.reboot(b"new entropy")
        app.send(b.id.S, b"after", when=70)
        sim.run(200)
        self.assertEqual(app.results, {b"before": "ok", b"after": "ok"})
        self.assertEqual(app.tries[b"after"], 2)                 # one wasted attempt
        self.assertEqual(b.stats["resync"], 1)
        check_rules(self, sim)

    def test_sender_reboot_handshakes_again_without_a_wasted_attempt(self):
        sim = Sim(seed=5)
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"before", when=5)
        sim.run(60)
        a.reboot(b"new entropy")
        app.send(b.id.S, b"after", when=70)
        sim.run(200)
        self.assertEqual(app.results, {b"before": "ok", b"after": "ok"})
        self.assertEqual(app.tries[b"after"], 1)
        self.assertEqual(sim.count(RESPONSE, 42), 2)
        check_rules(self, sim)

    def test_simultaneous_open(self):
        sim = Sim(seed=6)
        a, b = pair(sim)
        app_a, app_b = App(sim, a), App(sim, b)
        app_a.send(b.id.S, b"from a", when=5)
        app_b.send(a.id.S, b"from b", when=5)
        sim.run(120)
        self.assertEqual(app_a.results, {b"from a": "ok"})
        self.assertEqual(app_b.results, {b"from b": "ok"})
        self.assertEqual(app_a.tries[b"from a"] + app_b.tries[b"from b"], 2)   # no retry needed
        check_rules(self, sim)

    def test_replayed_handshake_1_does_not_disturb_a_session(self):
        sim = Sim(seed=7)
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"one", when=5)
        sim.run(60)
        old_hs1 = next(p for p in sim.tap if p.ptype == REQ and len(p.payload) == 42)
        sim.seen[sim.nodes.index(b)].clear()                     # past the duplicate horizon
        sim.inject(b, old_hs1)
        self.assertIsNotNone(b.peers[a.id.S].next)               # an unconfirmable session
        app.send(b.id.S, b"two", when=70)
        sim.run(200)
        self.assertEqual(app.results, {b"one": "ok", b"two": "ok"})
        self.assertEqual(app.tries[b"two"], 1)
        check_rules(self, sim)

    def test_replay_during_a_handshake_is_repaired(self):
        """SPEC section 13: denial of service only, repaired by 10.2."""
        sim = Sim(seed=8)
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"one", when=5)
        sim.run(60)
        old_hs1 = next(p for p in sim.tap if p.ptype == REQ and len(p.payload) == 42)
        a.reboot(b"entropy 2")                                   # a needs a new handshake
        token, packets = a.app_send(b.id.S, b"two", 1_700_000_100, 0)
        hs1 = packets[0]
        hs2 = b.receive(hs1.ptype, hs1.payload, False)[0]
        b.receive(old_hs1.ptype, old_hs1.payload, False)         # displaces the genuine `next`
        data = a.receive(hs2.ptype, hs2.payload, False)[0]
        back = b.receive(data.ptype, data.payload, False)
        self.assertNotIn(b"two", delivered(b, a.id.S))           # the attack worked once
        self.assertEqual(b.stats["resync"], 1)
        for pkt in back:                                         # b's own handshake repairs it
            for reply in a.receive(pkt.ptype, pkt.payload, False):
                for more in b.receive(reply.ptype, reply.payload, False):
                    a.receive(more.ptype, more.payload, False)
        token, packets = a.app_send(b.id.S, b"two", 1_700_000_100, 1)
        for pkt in packets:
            for reply in b.receive(pkt.ptype, pkt.payload, False):
                a.receive(reply.ptype, reply.payload, False)
        self.assertIn(b"two", delivered(b, a.id.S))
        self.assertIn(token, a.confirmed)

    def test_long_outage_ends_in_a_new_handshake(self):
        sim = Sim(seed=9)
        a, b = pair(sim)
        app = App(sim, a, attempts=1)
        app.send(b.id.S, b"start", when=5)
        sim.run(30)
        sim.loss = 1.0
        for i in range(12):                                      # more than K_UNACKED
            app.send(b.id.S, b"lost %d" % i, when=40 + 12 * i)
        sim.run(200)
        sim.loss = 0.0
        retry = App(sim, a)
        retry.send(b.id.S, b"back", when=210)
        sim.run(400)
        self.assertEqual(retry.results, {b"back": "ok"})
        self.assertGreaterEqual(a.stats["hs1_sent"], 2)
        check_rules(self, sim)

    def test_reordering_and_duplicates(self):
        sim = Sim(seed=10, delay=(0.1, 6.0), duplicate=0.3)
        a, b = pair(sim)
        app = App(sim, a, timeout=30)
        for i in range(12):
            app.send(b.id.S, b"burst %d" % i, when=20 + i)
        sim.run(300)
        self.assertEqual(set(app.results.values()), {"ok"})
        self.assertEqual(len(delivered(b, a.id.S)), 12)
        self.assertEqual(len([1 for s, _, _ in b.inbox if s == a.id.S]), 12)   # none twice
        check_rules(self, sim)

    def test_legacy_peer_gets_plain_legacy(self):
        sim = Sim(seed=11)
        a, b = pair(sim, b_zcpfs=False)
        app_a, app_b = App(sim, a), App(sim, b)
        app_a.send(b.id.S, b"to old", when=5)
        app_b.send(a.id.S, b"from old", when=20)
        sim.run(120)
        self.assertEqual(app_a.results, {b"to old": "ok"})
        self.assertEqual(app_b.results, {b"from old": "ok"})
        self.assertFalse(a.peers[b.id.S].pinned)
        self.assertEqual(sim.count(ACK, 6), sim.count(ACK))      # legacy ACKs keep 6 bytes
        self.assertTrue(all((n - 4) % 16 == 0 for _, _, t, n, _ in sim.air if t == TXT))
        self.assertEqual(sim.count(REQ) + sim.count(RESPONSE), 0)
        check_rules(self, sim)

    def test_hint_upgrades_two_hand_added_contacts(self):
        for paths in (True, False):
            sim = Sim(seed=12)
            a, b = pair(sim, adverts=False, paths=paths)
            app = App(sim, a, flood_last=False)
            app.send(b.id.S, b"first goes legacy", when=5)
            sim.run(60)
            self.assertEqual(app.results, {b"first goes legacy": "ok"})
            self.assertEqual(a.stats["legacy_sent"], 1)
            self.assertTrue(a.peers[b.id.S].pinned and b.peers[a.id.S].pinned, paths)
            app.send(b.id.S, b"second is sealed", when=70)
            sim.run(140)
            self.assertEqual(app.results[b"second is sealed"], "ok")
            self.assertEqual(a.stats["legacy_sent"], 1)
            self.assertEqual(a.stats["sealed_sent"], 1)
            check_rules(self, sim)

    def test_hint_cannot_pin_by_itself(self):
        sim = Sim(seed=13)
        a, b = pair(sim, adverts=False, b_zcpfs=False)
        app = App(sim, a)
        app.send(b.id.S, b"hello old node", when=5)
        sim.run(60)
        self.assertEqual(app.results, {b"hello old node": "ok"})
        self.assertFalse(a.peers[b.id.S].pinned)

    def test_pin_follows_accepted_adverts_only(self):
        sim = Sim(seed=14)
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"sealed", when=5)
        sim.run(60)
        self.assertFalse(a.on_advert(b.id.S, 900, b"bob"))       # older: not accepted
        self.assertFalse(a.on_advert(b.id.S, 1000, b"bob"))      # equal: not accepted
        self.assertTrue(a.peers[b.id.S].pinned)
        self.assertIsNotNone(a.peers[b.id.S].current)
        self.assertTrue(a.on_advert(b.id.S, 2000, b"bob"))       # newer, untagged
        self.assertFalse(a.peers[b.id.S].pinned)
        self.assertIsNone(a.peers[b.id.S].current)
        b.zcpfs = False                                          # the peer really moved to old firmware
        b.peers[a.id.S].reset_ram(b.p)
        app.send(b.id.S, b"legacy again", when=70)
        sim.run(140)
        self.assertEqual(app.results[b"legacy again"], "ok")
        self.assertEqual(a.stats["legacy_sent"], 1)

    def test_sealed_text_raises_the_advert_mark(self):
        sim = Sim(seed=14)
        a, b = pair(sim)
        app = App(sim, b)
        app.send(a.id.S, b"sealed", when=5)                      # timestamp 1_700_000_005
        sim.run(60)
        self.assertEqual(app.results[b"sealed"], "ok")
        mark = 1_700_000_005 - ADVERT_MARK_SLACK
        self.assertEqual(a.peers[b.id.S].advert_ts, mark)
        self.assertFalse(a.on_advert(b.id.S, mark, b"bob"))      # signed before: a replay
        self.assertTrue(a.peers[b.id.S].pinned)
        self.assertTrue(a.on_advert(b.id.S, mark + 1, b"bob"))   # within the slack: accepted
        self.assertFalse(a.peers[b.id.S].pinned)

    def test_pinned_contact_is_never_sent_legacy(self):
        sim = Sim(seed=15, loss=1.0)                             # nothing ever arrives
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"into the void", when=5)
        sim.run(120)
        self.assertEqual(app.results, {b"into the void": "failed"})
        self.assertEqual(a.stats["legacy_sent"], 0)
        self.assertTrue(all(t != TXT or (n - 4) % 16 for _, _, t, n, _ in sim.air))
        check_rules(self, sim)

    def test_third_node_with_a_colliding_address_byte(self):
        sim = Sim(seed=16)
        a, b = pair(sim)
        i = 0
        while True:                                              # a contact whose hash byte equals a's
            s = scalar(b"mallory %d" % i)
            c = sim.node(s, zcpfs=True, name=b"carol")
            if c.id.S[0] == a.id.S[0]:
                break
            sim.nodes.pop()
            sim.seen.pop()
            i += 1
        b.add_contact(c.id.S, has_path=True)
        c.add_contact(b.id.S, has_path=True)
        b.on_advert(*c.advert(1000))
        c.on_advert(*b.advert(1000))
        app_a, app_c = App(sim, a), App(sim, c)
        for k in range(4):
            app_a.send(b.id.S, b"from a %d" % k, when=5 + 30 * k)
            app_c.send(b.id.S, b"from c %d" % k, when=20 + 30 * k)
        sim.run(400)
        self.assertEqual(set(app_a.results.values()) | set(app_c.results.values()), {"ok"})
        self.assertEqual(len(delivered(b, a.id.S)), 4)
        self.assertEqual(len(delivered(b, c.id.S)), 4)
        check_rules(self, sim)

    def test_session_expires_when_idle(self):
        sim = Sim(seed=17)
        a, b = pair(sim)
        app = App(sim, a)
        app.send(b.id.S, b"day one", when=5)
        app.send(b.id.S, b"day three", when=5 + 2 * 86400)
        sim.run(3 * 86400)
        self.assertEqual(set(app.results.values()), {"ok"})
        self.assertEqual(sim.count(RESPONSE, 42), 2)             # a new handshake after the gap
        self.assertEqual(app.tries[b"day three"], 1)
        check_rules(self, sim)

    def test_held_message_is_resent_and_delivered_once(self):
        """SPEC 10.3, stepped by hand so that exactly one packet is lost."""
        sim = Sim(seed=18)
        a, b = pair(sim)
        token, packets = a.app_send(b.id.S, b"resend me", 1_700_000_000, 0)
        hs2 = b.receive(packets[0].ptype, packets[0].payload, False)[0]
        first = a.receive(hs2.ptype, hs2.payload, False)[0]       # the held message; "lost"
        self.assertEqual(a.tick(), [])                            # not due yet
        sim.t += a.p.T_DATA_RETRY
        again = a.tick()
        self.assertEqual(len(again), 1)
        self.assertNotEqual(again[0].payload, first.payload)      # sealed anew, never the same bytes
        for pkt in (again[0], first):                             # both copies reach b after all
            for reply in b.receive(pkt.ptype, pkt.payload, False):
                a.receive(reply.ptype, reply.payload, False)
        self.assertEqual([t for _, t, _ in b.inbox], [b"resend me"])   # delivered once
        self.assertEqual(a.confirmed.count(token), 2)             # each copy was acknowledged
        sim.t += 10 * a.p.T_DATA_RETRY
        self.assertEqual(a.tick(), [])                            # nothing more once confirmed

    def test_resend_is_bounded(self):
        sim = Sim(seed=19)
        a, b = pair(sim)
        _, packets = a.app_send(b.id.S, b"into silence", 1_700_000_000, 0)
        hs2 = b.receive(packets[0].ptype, packets[0].payload, False)[0]
        a.receive(hs2.ptype, hs2.payload, False)
        sent = 0
        for _ in range(10):
            sim.t += a.p.T_DATA_RETRY
            sent += len(a.tick())
        self.assertEqual(sent, a.p.N_DATA_TRIES)

    def test_first_contact_variant_with_the_handshake_in_the_flood(self):
        """Experimental alternative to the probe (Params.FIRST_CONTACT)."""
        sim = Sim(seed=20)
        a, b = pair(sim, paths=False, first_contact="handshake")
        app = App(sim, a, flood_last=False)
        app.send(b.id.S, b"first message to a new contact", when=5)
        sim.run(120)
        self.assertEqual(app.results, {b"first message to a new contact": "ok"})
        floods = [(ptype, n) for _, _, ptype, n, flood in sim.air if flood]
        self.assertEqual(floods.count((REQ, 42)), 1)
        self.assertNotIn((REQ, 14), floods)
        self.assertTrue(all(ptype != TXT for ptype, _ in floods))
        check_rules(self, sim, handshake_may_flood=True)


if __name__ == "__main__":
    unittest.main()
