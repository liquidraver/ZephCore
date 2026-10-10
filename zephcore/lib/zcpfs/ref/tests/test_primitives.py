# SPDX-License-Identifier: MIT
"""The primitives against published vectors: Ascon (official KAT subset in
lib/ascon/kat), X25519 (RFC 7748) and Ed25519 public keys (RFC 8032)."""

import hashlib
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import ascon  # noqa: E402
import curve  # noqa: E402

h = bytes.fromhex


def records(name):
    path = os.path.join(os.path.dirname(HERE), "..", "..", "ascon", "kat", name)
    cur = {}
    with open(path, encoding="ascii") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                continue
            if not line:
                if cur:
                    yield cur
                    cur = {}
                continue
            key, _, value = line.partition("=")
            cur[key.strip()] = value.strip()
    if cur:
        yield cur


class AsconKat(unittest.TestCase):
    def test_aead128(self):
        n = 0
        for r in records("aead.txt"):
            key, nonce, ad, pt = h(r["Key"]), h(r["Nonce"]), h(r["AD"]), h(r["PT"])
            expected = h(r["CT"])
            ct, tag = ascon.aead_encrypt(key, nonce, ad, pt)
            self.assertEqual(ct + tag, expected, r["Count"])
            self.assertEqual(ascon.aead_decrypt(key, nonce, ad, ct, tag), pt)
            self.assertEqual(ascon.aead_decrypt(key, nonce, ad, ct, tag[:8]), pt)
            bad = bytes([tag[0] ^ 1]) + tag[1:8]
            self.assertIsNone(ascon.aead_decrypt(key, nonce, ad, ct, bad))
            n += 1
        self.assertGreater(n, 50)

    def test_xof128(self):
        for r in records("xof.txt"):
            self.assertEqual(ascon.xof128(h(r["Msg"]), 64), h(r["MD"]), r["Count"])

    def test_cxof128(self):
        for r in records("cxof.txt"):
            self.assertEqual(ascon.cxof128(h(r["Msg"]), h(r["Z"]), 64), h(r["MD"]), r["Count"])

    def test_xof_output_is_a_prefix(self):
        self.assertEqual(ascon.xof128(b"abc", 80)[:13], ascon.xof128(b"abc", 13))


class Curve(unittest.TestCase):
    def test_rfc7748_single(self):
        out = curve.x25519(
            h("a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4"),
            h("e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c"))
        self.assertEqual(out.hex(),
                         "c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552")

    def test_rfc7748_diffie_hellman(self):
        a = h("77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a")
        b = h("5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb")
        pa, pb = curve.x25519_public(a), curve.x25519_public(b)
        self.assertEqual(pa.hex(),
                         "8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a")
        self.assertEqual(pb.hex(),
                         "de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f")
        self.assertEqual(curve.x25519(a, pb), curve.x25519(b, pa))
        self.assertEqual(curve.x25519(a, pb).hex(),
                         "4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742")

    def test_rfc8032_public_key_and_montgomery_map(self):
        seed = h("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60")
        scalar = hashlib.sha512(seed).digest()[:32]
        public = curve.ed25519_public(scalar)
        self.assertEqual(public.hex(),
                         "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a")
        self.assertEqual(curve.edwards_to_montgomery(public), curve.x25519_public(scalar))

    def test_small_order_point_gives_zero(self):
        scalar = hashlib.sha256(b"any").digest()
        for u in (0, 1):
            self.assertEqual(curve.x25519(scalar, u.to_bytes(32, "little")), bytes(32))


if __name__ == "__main__":
    unittest.main()
