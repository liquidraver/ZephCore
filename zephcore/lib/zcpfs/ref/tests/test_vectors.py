# SPDX-License-Identifier: MIT
"""../../vectors.json must be exactly what the model produces."""

import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import gen_vectors  # noqa: E402


class Vectors(unittest.TestCase):
    def test_checked_in_file_matches_the_model(self):
        with open(gen_vectors.OUT, encoding="utf-8") as f:
            self.assertEqual(json.load(f), json.loads(json.dumps(gen_vectors.build())))

    def test_packet_sizes(self):
        v = gen_vectors.build()
        self.assertEqual(len(v["handshake"]["packet_handshake_1"]) // 2, 42)
        self.assertEqual(len(v["handshake"]["packet_handshake_2"]) // 2, 42)
        self.assertEqual(len(v["probe"]["payload"]) // 2, 14)
        self.assertEqual(len(v["session"]["confirmation"]) // 2, 10)
        for direction in ("i_to_r", "r_to_i"):
            for p in v["session"][direction]:
                size = len(p["payload"]) // 2
                self.assertNotEqual((size - 4) % 16, 0)
                self.assertLessEqual(size, 176)


if __name__ == "__main__":
    unittest.main()
