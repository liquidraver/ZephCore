# zcpfs reference model

A plain-Python model of `../SPEC.md`. It exists to make the spec testable before any firmware is written, and to
produce the test vectors the firmware will later have to reproduce. It is slow, not constant-time, and never
runs on a device.

| File | What it is |
|---|---|
| `ascon.py` | Ascon-AEAD128, XOF128 and CXOF128 written from NIST SP 800-232. |
| `curve.py` | X25519, the Ed25519 public key of a scalar, the Edwards-to-Montgomery map. |
| `zcpfs.py` | The protocol: key derivation, generator, handshake, session, and `Node` (one device: slots, pinning, dispatch, recovery, the flood rule). |
| `sim.py` | A lossy, reordering, duplicating radio with MeshCore's duplicate table, and a phone app that retries. |
| `measure.py` | Delivery rate and bytes on air against legacy, averaged over random seeds. |
| `gen_vectors.py` | Writes `../vectors.json`. |
| `gen_c_vectors.py` | Writes `tests/unit/zcpfs_vectors.h`: the vectors as C tables for the host tests of the C library. |
| `kat/` | Elligator 2 outputs of the vendored Monocypher. The Ascon known-answer files the model is checked against are in `../../ascon/kat`. |
| `tests/` | `unittest` suites: primitives, protocol core, simulator scenarios, vectors. |

```bash
python -m unittest discover -s tests
```

```bash
python gen_vectors.py --check
```

```bash
python gen_c_vectors.py --check
```

```bash
python measure.py 24
```

What is real and what is not:

- The primitives are real and checked: all 3203 records of the official Ascon KAT files passed on 2026-10-10
  (a subset is kept in `lib/ascon/kat`), and the curve code reproduces the RFC 7748 and RFC 8032 vectors.
- Legacy MeshCore encryption is only imitated (`legacy_seal`): right shape, wrong cipher. Legacy packets in
  the model are not byte-exact and are not in the vectors.
- The radio is one hop with independent random loss. Repeaters, airtime and collisions are not modelled;
  "flooded bytes" is only the size of what a mesh would repeat.
- Advert signatures are assumed verified.

No external packages. Written for Python 3.10 or newer.
