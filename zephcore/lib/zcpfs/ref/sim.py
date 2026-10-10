# SPDX-License-Identifier: MIT
"""A small radio and phone-app simulator around the zcpfs reference model.

All nodes hear each other (one hop). Every delivery is independently lost with
probability `loss`, delayed by a random time (which reorders), and passes a
copy of MeshCore's duplicate table (160 hashes of type + payload) before the
node sees it. A flooded packet is only counted differently; repeaters are not
modelled, so "flooded bytes" is the size of what a mesh would repeat.

Run `python sim.py` for a comparison of traffic profiles against legacy.
"""

import hashlib
import heapq
import random
from collections import deque

from zcpfs import ACK, PATH, REQ, RESPONSE, TXT, Node, Params

DEDUP_TABLE = 160


class Sim:
    def __init__(self, seed=1, loss=0.0, delay=(0.3, 1.2), duplicate=0.0):
        self.t = 0.0
        self.rng = random.Random(seed)
        self.loss, self.delay, self.duplicate = loss, delay, duplicate
        self.nodes = []
        self.seen = []
        self.queue = []
        self.seq = 0
        self.air = []                  # (time, sender index, ptype, length, flood)
        self.tap = []                  # every transmitted packet, for replay tests

    def clock(self):
        return self.t

    def node(self, scalar, zcpfs=True, name=b"node", first_contact="probe", password=None):
        n = Node(scalar, hashlib.sha256(b"hw" + scalar).digest(), self.clock,
                 params=Params(FIRST_CONTACT=first_contact), zcpfs=zcpfs, name=name,
                 password=password)
        self.nodes.append(n)
        self.seen.append(deque(maxlen=DEDUP_TABLE))
        return n

    def at(self, when, fn):
        self.seq += 1
        heapq.heappush(self.queue, (when, self.seq, fn))

    def transmit(self, sender, packets):
        src = self.nodes.index(sender)
        for pkt in packets:
            self.air.append((self.t, src, pkt.ptype, len(pkt.payload), pkt.flood))
            self.tap.append(pkt)
            for dst in range(len(self.nodes)):
                if dst == src:
                    continue
                copies = 2 if self.rng.random() < self.duplicate else 1
                for _ in range(copies):
                    if self.rng.random() < self.loss:
                        continue
                    when = self.t + self.rng.uniform(*self.delay)
                    self.at(when, lambda d=dst, p=pkt: self._deliver(d, p))
        self.arm(sender)

    def arm(self, node):
        """Schedule the timer calls a node asked for."""
        wakeups, node.wakeups = node.wakeups, []
        for when in wakeups:
            self.at(when, lambda n=node: self.transmit(n, n.tick()))

    def inject(self, dst_node, pkt):
        """Deliver a packet straight to one node (replay by an attacker)."""
        self._deliver(self.nodes.index(dst_node), pkt)

    def _deliver(self, dst, pkt):
        digest = hashlib.sha256(bytes([pkt.ptype]) + pkt.payload).digest()[:8]
        if digest in self.seen[dst]:
            return                     # MeshCore's duplicate suppression
        self.seen[dst].append(digest)
        node = self.nodes[dst]
        self.transmit(node, node.receive(pkt.ptype, pkt.payload, pkt.flood))

    def run(self, until):
        while self.queue and self.queue[0][0] <= until:
            when, _, fn = heapq.heappop(self.queue)
            self.t = when
            fn()
        self.t = until

    # -- measurements --------------------------------------------------------

    def bytes_on_air(self, flood=None):
        return sum(length for _, _, _, length, f in self.air if flood is None or f == flood)

    def count(self, ptype, length=None):
        return sum(1 for _, _, t, n, _ in self.air if t == ptype and (length is None or n == length))


class App:
    """The phone app's side of a send: wait for the confirmation, retry, and
    on the last attempt reset the path so the message floods."""

    def __init__(self, sim, node, timeout=10.0, attempts=3, flood_last=True):
        self.sim, self.node = sim, node
        self.timeout, self.attempts, self.flood_last = timeout, attempts, flood_last
        self.results = {}              # text -> "ok" | "failed"
        self.tries = {}                # text -> attempts used

    def send(self, peer_S, text, when=None):
        self.sim.at(self.sim.t if when is None else when,
                    lambda: self._attempt(peer_S, text, int(self.sim.t) + 1_700_000_000, 0))

    def _attempt(self, peer_S, text, timestamp, attempt):
        if attempt == self.attempts - 1 and attempt > 0 and self.flood_last:
            self.node.peers[peer_S].has_path = False
        token, packets = self.node.app_send(peer_S, text, timestamp, attempt)
        self.tries[text] = attempt + 1
        self.sim.transmit(self.node, packets)
        self.sim.at(self.sim.t + self.timeout * self.node.wait_factor,
                    lambda: self._check(peer_S, text, timestamp, attempt, token))

    def _check(self, peer_S, text, timestamp, attempt, token):
        if self.results.get(text) == "ok":
            return
        # Any attempt's confirmation counts; tokens of this text are consecutive
        # per call, so look for the ones this App issued.
        self._tokens = getattr(self, "_tokens", {})
        self._tokens.setdefault(text, []).append(token)
        if any(t in self.node.confirmed for t in self._tokens[text]):
            self.results[text] = "ok"
        elif attempt + 1 < self.attempts:
            self._attempt(peer_S, text, timestamp, attempt + 1)
        else:
            self.results[text] = "failed"


def scalar(label):
    return hashlib.sha256(b"zcpfs-sim-identity:" + label).digest()


def pair(sim, a_zcpfs=True, b_zcpfs=True, paths=True, adverts=True, first_contact="probe"):
    """Two nodes that know each other as contacts."""
    a = sim.node(scalar(b"alice"), a_zcpfs, b"alice", first_contact)
    b = sim.node(scalar(b"bob"), b_zcpfs, b"bob", first_contact)
    a.add_contact(b.id.S, has_path=paths)
    b.add_contact(a.id.S, has_path=paths)
    if adverts:
        a.on_advert(*b.advert(1000))
        b.on_advert(*a.advert(1000))
    return a, b


def delivered(node, sender_S):
    """Distinct texts the node received from that sender."""
    return {text for s, text, _ in node.inbox if s == sender_S}


# ---------------------------------------------------------------------------
# Profiles for `python sim.py`
# ---------------------------------------------------------------------------

def _profile(zc, loss, messages, gap, paths, seed, text_len=24):
    sim = Sim(seed=seed, loss=loss)
    a, b = pair(sim, zc, zc, paths=paths)
    app_a, app_b = App(sim, a), App(sim, b)
    for i in range(messages):
        sender, peer = (app_a, b) if i % 2 == 0 else (app_b, a)
        text = (b"%04d " % i) + b"x" * (text_len - 5)
        sender.send(peer.id.S, text, when=10 + i * gap)
    sim.run(10 + messages * gap + 600)
    ok = sum(1 for app in (app_a, app_b) for r in app.results.values() if r == "ok")
    return {
        "ok": ok,
        "air": sim.bytes_on_air(),
        "flood": sim.bytes_on_air(flood=True),
        "handshakes": sim.count(RESPONSE, 42),
        "packets": len(sim.air),
    }


def main():
    profiles = [
        ("chatty: 40 messages, 30 s apart", dict(messages=40, gap=30, paths=True)),
        ("sparse: 4 messages, 2 days apart", dict(messages=4, gap=2 * 86400, paths=True)),
        ("first contact: 6 messages, no path", dict(messages=6, gap=60, paths=False)),
    ]
    print("%-36s %5s | %-28s | %-28s" % ("profile", "loss", "legacy", "zcpfs"))
    print("%-36s %5s | %-28s | %-28s" % ("", "", "ok  bytes  flooded", "ok  bytes  flooded  handsh."))
    for label, kw in profiles:
        for loss in (0.0, 0.2):
            old = _profile(False, loss, seed=7, **kw)
            new = _profile(True, loss, seed=7, **kw)
            print("%-36s %4d%% | %2d %6d %8d        | %2d %6d %8d %7d" % (
                label, loss * 100, old["ok"], old["air"], old["flood"],
                new["ok"], new["air"], new["flood"], new["handshakes"]))


if __name__ == "__main__":
    main()
