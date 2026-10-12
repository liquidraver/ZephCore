# SPDX-License-Identifier: MIT
"""SPEC sections 18 and 19: requests, responses and path returns in the
stateless pairwise mode, and the login handshake."""

import hashlib
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zcpfs  # noqa: E402
from sim import App, Sim, delivered, pair, scalar  # noqa: E402
from zcpfs import (ACK, ANON, PATH, REQ, RESPONSE, TXT, Drbg, Identity, fit,  # noqa: E402
                   legacy_seal, pair_open, pair_seal, pake1_make, pake1_open,
                   pake_branch_make, pake_branch_open, pake_generator, pake_reply_pack,
                   pake_reply_unpack)


def identity(label):
    return Identity(hashlib.sha256(b"test-identity:" + label).digest())


def deliver(sim, sender, packets, until=None):
    sim.transmit(sender, packets)
    sim.run(sim.t + 30 if until is None else until)


class Lengths(unittest.TestCase):
    def test_fit_avoids_every_reserved_length(self):
        for ptype in (REQ, RESPONSE, PATH, TXT):
            for overhead in (10, 16, 42):
                for n in range(0, 150):
                    total = len(fit(ptype, bytes(n), overhead)) + overhead
                    self.assertNotEqual((total - 4) % 16, 0)
                    if ptype == REQ:
                        self.assertNotIn(total, (14, 42))
                    if ptype == RESPONSE:
                        self.assertNotEqual(total, 42)
        for n in range(0, 40):
            total = len(fit(ANON, bytes(n), 73)) + 73
            self.assertNotEqual((total - 35) % 16, 0)

    def test_fit_adds_at_most_two_bytes(self):
        for ptype in (REQ, RESPONSE, PATH):
            for n in range(0, 150):
                self.assertLessEqual(len(fit(ptype, bytes(n), 16)) - n, 2)


class PairwiseSealing(unittest.TestCase):
    def setUp(self):
        self.a, self.b = identity(b"a"), identity(b"b")
        self.ss = self.a.shared_secret(self.b.S)
        self.n6 = bytes(range(6))

    def test_round_trip_and_size(self):
        payload = pair_seal(self.ss, self.a.S, self.b.S, REQ, b"\x01\x02\x03\x04status?", self.n6)
        self.assertEqual(pair_open(self.ss, self.a.S, self.b.S, REQ, payload)[:11],
                         b"\x01\x02\x03\x04status?")
        self.assertLessEqual(len(payload), 11 + 16 + 2)
        self.assertEqual(payload[:2], bytes([self.b.S[0], self.a.S[0]]))

    def test_bound_to_direction_type_and_content(self):
        payload = pair_seal(self.ss, self.a.S, self.b.S, REQ, b"question", self.n6)
        self.assertIsNone(pair_open(self.ss, self.b.S, self.a.S, REQ, payload))     # other direction
        self.assertIsNone(pair_open(self.ss, self.a.S, self.b.S, RESPONSE, payload))  # other type
        for i in (0, 2, 8, len(payload) - 1):
            bad = bytearray(payload)
            bad[i] ^= 1
            self.assertIsNone(pair_open(self.ss, self.a.S, self.b.S, REQ, bytes(bad)))

    def test_a_reflected_packet_does_not_open(self):
        # a's own request, sent back to a with the address bytes swapped
        payload = pair_seal(self.ss, self.a.S, self.b.S, REQ, b"question", self.n6)
        swapped = bytes([payload[1], payload[0]]) + payload[2:]
        self.assertIsNone(pair_open(self.ss, self.b.S, self.a.S, REQ, swapped))


class Requests(unittest.TestCase):
    def test_request_and_response_between_capable_nodes(self):
        sim = Sim(seed=40)
        a, b = pair(sim)
        deliver(sim, a, a.request(b.id.S, b"\x11\x22\x33\x44telemetry"))
        self.assertEqual(b.requests_seen, [(a.id.S, b"\x11\x22\x33\x44telemetry")])
        self.assertEqual(a.responses, [(b.id.S, b"re:\x11\x22\x33\x44telemetry")])
        for _, _, ptype, n, _ in sim.air:
            self.assertNotEqual((n - 4) % 16, 0)                 # nothing legacy on the air
            self.assertFalse(ptype == REQ and n in (14, 42))
            self.assertFalse(ptype == RESPONSE and n == 42)
        self.assertEqual(sim.count(REQ) + sim.count(RESPONSE), 2)
        self.assertIsNone(a.peers[b.id.S].current)               # no handshake was needed

    def test_request_to_a_legacy_node_is_legacy(self):
        sim = Sim(seed=41)
        a, b = pair(sim, b_zcpfs=False)
        deliver(sim, a, a.request(b.id.S, b"\x11\x22\x33\x44status"))
        self.assertEqual(a.responses, [(b.id.S, b"re:\x11\x22\x33\x44status")])
        self.assertTrue(all((n - 4) % 16 == 0 for _, _, _, n, _ in sim.air))

    def test_reply_is_in_kind_even_if_the_requester_is_not_pinned(self):
        sim = Sim(seed=42)
        a, b = pair(sim, adverts=False)
        a.on_advert(*b.advert(1000))                             # only a knows b is capable
        deliver(sim, a, a.request(b.id.S, b"\x11\x22\x33\x44ping"))
        self.assertEqual(a.responses, [(b.id.S, b"re:\x11\x22\x33\x44ping")])
        self.assertTrue(all((n - 4) % 16 != 0 for _, _, _, n, _ in sim.air))
        self.assertFalse(b.peers[a.id.S].pinned)                 # a sealed request does not pin

    def test_legacy_request_from_a_pinned_contact_is_refused(self):
        sim = Sim(seed=43)
        a, b = pair(sim)
        forged = bytes([b.id.S[0], a.id.S[0]]) + legacy_seal(a.peers[b.id.S].ss, b"\x00" * 4 + b"old")
        self.assertEqual(b.receive(REQ, forged, False), [])
        self.assertEqual(b.requests_seen, [])

    def test_flooded_request_is_answered_through_a_sealed_path_return(self):
        sim = Sim(seed=44)
        a, b = pair(sim, paths=False)
        deliver(sim, a, a.request(b.id.S, b"\x11\x22\x33\x44where"))
        self.assertEqual(a.responses, [(b.id.S, b"re:\x11\x22\x33\x44where")])
        self.assertTrue(a.peers[b.id.S].has_path and b.peers[a.id.S].has_path)
        paths = [(n, flood) for _, _, ptype, n, flood in sim.air if ptype == PATH]
        self.assertEqual([flood for _, flood in paths], [True, False])   # the return, then the reciprocal
        self.assertTrue(all((n - 4) % 16 != 0 for n, _ in paths))

    def test_path_returns_of_a_first_contact_are_sealed(self):
        sim = Sim(seed=45)
        a, b = pair(sim, paths=False)
        app = App(sim, a, flood_last=False)
        app.send(b.id.S, b"hello over a new path", when=5)
        sim.run(120)
        self.assertEqual(app.results, {b"hello over a new path": "ok"})
        paths = [n for _, _, ptype, n, _ in sim.air if ptype == PATH]
        self.assertGreaterEqual(len(paths), 2)
        self.assertTrue(all((n - 4) % 16 != 0 for n in paths))

    def test_a_contact_that_only_one_side_has_pinned(self):
        sim = Sim(seed=46)
        a, b = pair(sim, adverts=False)
        a.on_advert(*b.advert(1000))                             # a pins b; b knows nothing of a
        app = App(sim, b)
        app.send(a.id.S, b"b speaks first", when=5)
        sim.run(120)
        self.assertEqual(app.results, {b"b speaks first": "ok"})
        self.assertEqual(app.tries[b"b speaks first"], 2)         # the legacy copy was refused
        self.assertEqual(b.stats["legacy_sent"], 1)
        self.assertTrue(b.peers[a.id.S].pinned)                   # the handshake pinned b as well


class Elligator(unittest.TestCase):
    def test_matches_the_firmware_library(self):
        """Vectors produced by the vendored monocypher (see the file header)."""
        import curve
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(here, "kat", "elligator_monocypher.txt"), encoding="ascii") as f:
            rows = [line.split() for line in f if line.strip() and not line.startswith("#")]
        self.assertEqual(len(rows), 18)
        for t in range(6):
            given, mapped, dh = (bytes.fromhex(rows[3 * t + k][1]) for k in range(3))
            scalar_ = bytes((0xA0 + j + t) & 0xFF for j in range(32))
            self.assertEqual(curve.elligator2(given), mapped)
            self.assertEqual(curve.x25519(scalar_, mapped), dh)

    def test_generator_depends_on_password_and_both_identities(self):
        a, b, c = identity(b"a"), identity(b"b"), identity(b"c")
        g = pake_generator(b"hunter2", a.S, b.S)
        self.assertNotEqual(g, pake_generator(b"hunter3", a.S, b.S))
        self.assertNotEqual(g, pake_generator(b"hunter2", a.S, c.S))
        self.assertNotEqual(g, pake_generator(b"hunter2", c.S, b.S))


class LoginCore(unittest.TestCase):
    def setUp(self):
        self.c, self.s = identity(b"client"), identity(b"server")
        self.ss = self.c.shared_secret(self.s.S)

    def client(self, password):
        return pake1_make(Drbg(b"c"), self.ss, self.c.S, self.s.S, password, b"\x01\x02\x03\x04")

    def server(self, body1, password, response=b"\x00\x00\x00\x00ad"):
        S_c, Y_c, ss, plain, _ = pake1_open(self.s.a, self.s.S, body1)
        return pake_branch_make(Drbg(b"s" + password), ss, S_c, self.s.S, Y_c, password, response)

    def test_same_password_gives_the_same_keys(self):
        out, body1 = self.client(b"hunter2")
        branch, ck_i2r, ck_r2i, _ = self.server(body1, b"hunter2")
        plain, a, b, _ = pake_branch_open(self.ss, out, self.c.S, self.s.S, branch)
        self.assertEqual(plain, b"\x00\x00\x00\x00ad")
        self.assertEqual(len(branch), 42)                         # a point, six bytes, a 4-byte tag
        self.assertEqual((a, b), (ck_i2r, ck_r2i))

    def test_different_password_gives_nothing(self):
        out, body1 = self.client(b"hunter2")
        branch, _, _, _ = self.server(body1, b"hunter3")
        self.assertIsNone(pake_branch_open(self.ss, out, self.c.S, self.s.S, branch))

    def test_the_login_packet_does_not_depend_on_the_password_in_any_checkable_way(self):
        # Opening the login packet needs only the static keys, and succeeds
        # whatever the password: the server learns nothing about it here.
        for password in (b"hunter2", b"", b"x" * 15):
            _, body1 = self.client(password)
            S_c, Y_c, ss, plain, empty = pake1_open(self.s.a, self.s.S, body1)
            self.assertEqual((S_c, plain.rstrip(b"\x00")), (self.c.S, b"\x01\x02\x03\x04"))
            self.assertEqual(empty, password == b"")           # all it says: whether one was typed
        self.assertNotIn(b"hunter2", self.client(b"hunter2")[1])

    def test_a_server_that_guesses_gets_exactly_one_guess(self):
        # The fake server answers with a branch for its guess. The client's
        # reaction (opens or not) is all it learns; with another guess it would
        # need another login from the client.
        out, body1 = self.client(b"hunter2")
        wrong, _, _, _ = self.server(body1, b"123456")
        right, _, _, _ = self.server(body1, b"hunter2")
        self.assertIsNone(pake_branch_open(self.ss, out, self.c.S, self.s.S, wrong))
        self.assertIsNotNone(pake_branch_open(self.ss, out, self.c.S, self.s.S, right))

    def test_tampering_and_wrong_identities_fail(self):
        out, body1 = self.client(b"hunter2")
        for i in (0, 32, 64, len(body1) - 1):
            bad = bytearray(body1)
            bad[i] ^= 1
            self.assertIsNone(pake1_open(self.s.a, self.s.S, bytes(bad)))
        other = identity(b"another server")
        self.assertIsNone(pake1_open(other.a, other.S, body1))
        branch, _, _, _ = self.server(body1, b"hunter2")
        for i in (0, 32, len(branch) - 1):
            bad = bytearray(branch)
            bad[i] ^= 1
            self.assertIsNone(pake_branch_open(self.ss, out, self.c.S, self.s.S, bytes(bad)))

    def test_low_order_point_from_the_client_is_refused(self):
        _, body1 = self.client(b"hunter2")
        S_c, _, ss, _, _ = pake1_open(self.s.a, self.s.S, body1)
        self.assertIsNone(pake_branch_make(Drbg(b"s"), ss, S_c, self.s.S, bytes(32), b"hunter2", b"rrrrrr"))

    def test_packet_lengths(self):
        _, body1 = self.client(b"hunter2")
        self.assertNotEqual((1 + len(body1) - 35) % 16, 0)
        self.assertEqual(1 + len(body1), 78)                      # 74 + a 4-byte timestamp
        out, body1 = self.client(b"hunter2")
        for count in (1, 2):
            branches = [self.server(body1, b"pw%d" % i)[0] for i in range(count)]
            reply = pake_reply_pack(branches)
            total = 2 + len(reply)
            self.assertNotEqual((total - 4) % 16, 0)
            self.assertNotEqual(total, 42)
            self.assertLessEqual(total, zcpfs.MAX_PAYLOAD)
            self.assertEqual(pake_reply_unpack(reply), branches)
            self.assertEqual(total, (45, 87)[count - 1])
        self.assertIsNone(pake_reply_unpack(b"\x03" + bytes(200)))        # too many branches
        self.assertIsNone(pake_reply_unpack(b"\x02" + bytes(83)))         # truncated
        self.assertIsNone(pake_reply_unpack(b"\x00" + bytes(83)))


class LoginNodes(unittest.TestCase):
    def setup(self, client_zcpfs=True, server_zcpfs=True, paths=True, seed=50):
        sim = Sim(seed=seed)
        client = sim.node(scalar(b"client"), client_zcpfs, b"client")
        server = sim.node(scalar(b"repeater"), server_zcpfs, b"repeater", password=b"hunter2")
        client.add_contact(server.id.S, has_path=paths)
        client.on_advert(*server.advert(1000))                    # the server does not know the client
        return sim, client, server

    def test_login_then_admin_text_both_ways(self):
        sim, client, server = self.setup()
        deliver(sim, client, client.login(server.id.S, b"hunter2", 1_700_000_000))
        self.assertEqual(len(client.logins), 1)
        self.assertTrue(client.logins[0][1].endswith(b"ad"))
        self.assertEqual(server.logged_in, [(client.id.S, b"admin")])
        self.assertEqual(sim.count(ANON), 1)
        self.assertEqual(sim.count(TXT, 10), 1)                   # the client's confirmation
        self.assertNotIn(b"hunter2", b"".join(p.payload for p in sim.tap))
        peer_at_server = server.peers[client.id.S]
        self.assertIsNotNone(peer_at_server.current)
        self.assertTrue(peer_at_server.pinned)
        app_c, app_s = App(sim, client), App(sim, server)
        app_c.send(server.id.S, b"get radio", when=sim.t + 1)
        app_s.send(client.id.S, b"> 869.5,62.5,8,8", when=sim.t + 20)
        sim.run(sim.t + 120)
        self.assertEqual(app_c.results, {b"get radio": "ok"})
        self.assertEqual(app_s.results, {b"> 869.5,62.5,8,8": "ok"})
        self.assertEqual(sim.count(REQ, 42) + sim.count(RESPONSE, 42), 0)   # no second handshake
        self.assertTrue(all((n - 4) % 16 != 0 for _, _, t, n, _ in sim.air if t == TXT))

    def test_wrong_password_grants_nothing_and_the_client_knows(self):
        sim, client, server = self.setup()
        deliver(sim, client, client.login(server.id.S, b"letmein", 1_700_000_000))
        self.assertEqual(client.logins, [])                       # no branch opened
        self.assertEqual(server.logged_in, [])
        self.assertIsNone(client.peers[server.id.S].current)
        self.assertEqual(sim.count(TXT), 0)                       # so no confirmation was sent
        sim.run(sim.t + 60)
        server.tick()
        self.assertEqual(server.peers[client.id.S].login_candidates, [])   # and the offer expired

    def test_guest_and_known_client_roles(self):
        sim, client, server = self.setup()
        server.guest_password = b"visitor"
        deliver(sim, client, client.login(server.id.S, b"visitor", 1_700_000_000))
        self.assertEqual(server.logged_in, [(client.id.S, b"guest")])
        self.assertTrue(client.logins[-1][1].endswith(b"gu"))
        server.acl.add(client.id.S)
        deliver(sim, client, client.login(server.id.S, b"", 1_700_000_100))
        self.assertEqual(server.logged_in[-1], (client.id.S, b"known"))
        replies = [n for _, _, t, n, _ in sim.air if t == RESPONSE]
        self.assertEqual(replies, [87, 45])                       # admin and guest; then the empty password alone

    def test_a_server_that_does_not_know_the_password_learns_nothing(self):
        """The repeater with a copied name: capable, but with another password."""
        sim, client, server = self.setup()
        server.password = b"something else"
        deliver(sim, client, client.login(server.id.S, b"hunter2", 1_700_000_000))
        self.assertEqual(client.logins, [])
        self.assertEqual(server.logged_in, [])
        self.assertNotIn(b"hunter2", b"".join(p.payload for p in sim.tap))

    def test_login_without_a_path_rides_the_flood_like_legacy(self):
        sim, client, server = self.setup(paths=False)
        deliver(sim, client, client.login(server.id.S, b"hunter2", 1_700_000_000))
        self.assertEqual(len(client.logins), 1)
        floods = [ptype for _, _, ptype, _, flood in sim.air if flood]
        self.assertEqual(floods[:2], [ANON, PATH])                # request out, path return back
        self.assertTrue(client.peers[server.id.S].has_path)
        self.assertEqual(server.logged_in, [(client.id.S, b"admin")])

    def test_legacy_client_and_legacy_server_still_log_in(self):
        for client_zcpfs, server_zcpfs in ((False, True), (True, False)):
            sim, client, server = self.setup(client_zcpfs, server_zcpfs, seed=51)
            deliver(sim, client, client.login(server.id.S, b"hunter2", 1_700_000_000))
            self.assertEqual(server.logged_in, [(client.id.S, b"admin")], (client_zcpfs, server_zcpfs))
            self.assertEqual(len(client.responses), 1)
            anon = [n for _, _, t, n, _ in sim.air if t == ANON]
            self.assertEqual((anon[0] - 35) % 16, 0)             # the legacy login format

    def test_legacy_login_off_keeps_the_password_in_the_node(self):
        sim, client, server = self.setup(server_zcpfs=False)       # the server looks old
        client.allow_legacy_login = False
        self.assertEqual(client.login(server.id.S, b"hunter2", 1_700_000_000), [])
        self.assertEqual(client.refused_legacy_logins, 1)
        sim2, client2, server2 = self.setup(client_zcpfs=False, seed=52)
        server2.allow_legacy_login = False                       # and a server can refuse old clients
        deliver(sim2, client2, client2.login(server2.id.S, b"hunter2", 1_700_000_000))
        self.assertEqual(server2.logged_in, [])

    def test_replayed_login_gives_the_attacker_nothing_usable(self):
        sim, client, server = self.setup()
        deliver(sim, client, client.login(server.id.S, b"hunter2", 1_700_000_000))
        recorded = next(p for p in sim.tap if p.ptype == ANON)
        before = server.peers[client.id.S].current
        sim.seen[sim.nodes.index(server)].clear()
        sim.inject(server, recorded)
        sim.run(sim.t + 60)
        self.assertIs(server.peers[client.id.S].current, before)  # the live session is untouched
        self.assertEqual(len(server.logged_in), 1)                # and nothing new was granted
        app = App(sim, client)
        app.send(server.id.S, b"still works", when=sim.t + 1)
        sim.run(sim.t + 60)
        self.assertEqual(app.results, {b"still works": "ok"})

    def test_requests_after_login_are_sealed(self):
        sim, client, server = self.setup()
        deliver(sim, client, client.login(server.id.S, b"hunter2", 1_700_000_000))
        deliver(sim, client, client.request(server.id.S, b"\x01\x02\x03\x04stats"))
        self.assertEqual(client.responses, [(server.id.S, b"re:\x01\x02\x03\x04stats")])
        self.assertTrue(all((n - 4) % 16 != 0 for _, _, t, n, _ in sim.air if t in (REQ, RESPONSE)))


if __name__ == "__main__":
    unittest.main()
