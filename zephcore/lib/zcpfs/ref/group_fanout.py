# SPDX-License-Identifier: MIT
"""Experiment, not part of the spec: a small private group as fan-out over
zcpfs direct messages, against one legacy channel flood per message. Uses the
reference model as it is. See devdocs/PFS_ASCON_PLAN.md section 20.

    python group_fanout.py
"""

import sim
from zcpfs import RESPONSE

TEXT = 30          # characters of the message itself
NAME = 8           # legacy channel text is "name: text"


def legacy_flood_bytes():
    plain = 4 + 1 + NAME + 2 + TEXT
    return 1 + 2 + -(-plain // 16) * 16          # channel hash, MAC, padded blocks


def run(members, messages, loss, seed):
    s = sim.Sim(seed=seed, loss=loss)
    nodes = [s.node(sim.scalar(b"member %d" % i), True, b"m%d" % i) for i in range(members)]
    for a in nodes:
        for b in nodes:
            if a is not b:
                a.add_contact(b.id.S, has_path=True)
                a.on_advert(*b.advert(1000))
    apps = [sim.App(s, n) for n in nodes]
    expected = 0
    for m in range(messages):
        sender = m % members
        for r in range(members):
            if r != sender:
                # one byte of channel id would precede the text; counted in TEXT + 1
                text = (b"%03d>%d " % (m, r)) + b"x" * (TEXT + 1 - 7)
                apps[sender].send(nodes[r].id.S, text, when=10 + m * 30)
                expected += 1
    s.run(10 + messages * 30 + 300)
    ok = sum(r == "ok" for app in apps for r in app.results.values())
    return ok, expected, s.bytes_on_air(), s.count(RESPONSE, 42), len(s.air)


def main():
    seeds = 4
    messages = 24
    flood = legacy_flood_bytes()
    print("legacy channel message: one flood of %d bytes, no ACK, no retry" % flood)
    print("%7s %5s | %9s | %12s %10s %10s | %s" % (
        "members", "loss", "delivered", "bytes/message", "packets/msg", "handshakes", "legacy delivered"))
    for members in (3, 5, 8):
        for loss in (0.0, 0.1, 0.25):
            tot = [0, 0, 0, 0, 0]
            for seed in range(seeds):
                for i, v in enumerate(run(members, messages, loss, seed)):
                    tot[i] += v
            n = messages * seeds
            print("%7d %4d%% | %8.0f%% | %12.0f %10.1f %10.1f | %.0f%%" % (
                members, loss * 100, 100.0 * tot[0] / tot[1], tot[2] / n, tot[4] / n,
                tot[3] / seeds, 100 * (1 - loss)), flush=True)


if __name__ == "__main__":
    main()
