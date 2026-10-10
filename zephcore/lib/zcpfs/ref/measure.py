# SPDX-License-Identifier: MIT
"""Delivery rate and airtime of zcpfs against legacy, averaged over random
seeds. Slow (pure-Python curve arithmetic): about a minute per 100 runs.

    python measure.py [seeds]
"""

import sys

import sim
from zcpfs import RESPONSE


def run(zc, loss, paths, messages, gap, seed, first_contact="probe"):
    s = sim.Sim(seed=seed, loss=loss)
    a, b = sim.pair(s, zc, zc, paths=paths, first_contact=first_contact)
    app_a, app_b = sim.App(s, a), sim.App(s, b)
    for i in range(messages):
        app, peer = (app_a, b) if i % 2 == 0 else (app_b, a)
        text = (b"%03d " % i) + b"x" * (3 + (i * 7 + seed) % 40)   # mixed lengths
        app.send(peer.id.S, text, when=10 + i * gap)
    s.run(10 + messages * gap + 300)
    ok = sum(r == "ok" for app in (app_a, app_b) for r in app.results.values())
    return ok, s.bytes_on_air(), s.bytes_on_air(flood=True), s.count(RESPONSE, 42)


def average(zc, loss, paths, messages, gap, seeds, first_contact="probe"):
    tot = [0, 0, 0, 0]
    for seed in range(seeds):
        for i, v in enumerate(run(zc, loss, paths, messages, gap, seed, first_contact)):
            tot[i] += v
    sent = messages * seeds
    return 100.0 * tot[0] / sent, tot[1] / sent, tot[2] / sent, tot[3] / seeds


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    profiles = [
        ("one message, path known", dict(paths=True, messages=1, gap=60)),
        ("one message, no path", dict(paths=False, messages=1, gap=60)),
        ("10 messages, 60 s apart", dict(paths=True, messages=10, gap=60)),
        ("30 messages, 30 s apart", dict(paths=True, messages=30, gap=30)),
    ]
    print("%d seeds per row; bytes are per message sent" % seeds)
    print("%-26s %5s | %-22s | %-30s" % ("profile", "loss", "legacy", "zcpfs"))
    print("%-26s %5s | %-22s | %-30s" % ("", "", " ok%  bytes  flooded", " ok%  bytes  flooded  handsh."))
    for label, kw in profiles:
        for loss in (0.0, 0.1, 0.25):
            old = average(False, loss, seeds=seeds, **kw)
            new = average(True, loss, seeds=seeds, **kw)
            print("%-26s %4d%% | %4.0f %6.1f %8.1f | %4.0f %6.1f %8.1f %8.2f" % (
                label, loss * 100, old[0], old[1], old[2], new[0], new[1], new[2], new[3]),
                flush=True)


if __name__ == "__main__":
    main()
