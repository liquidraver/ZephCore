# SPDX-License-Identifier: MIT
"""Ascon-AEAD128, Ascon-XOF128 and Ascon-CXOF128 as specified in NIST SP 800-232.

Written from the standard for the zcpfs reference model. It is deliberately
plain and slow. It is checked against the official known-answer tests (see
tests/test_primitives.py); it is not for use in firmware.
"""

MASK = 0xFFFFFFFFFFFFFFFF

# Round constants of Ascon-p. A permutation of `rounds` rounds uses the last
# `rounds` entries (SP 800-232 section 3.2).
_RC = (0x3C, 0x2D, 0x1E, 0x0F, 0xF0, 0xE1, 0xD2, 0xC3,
       0xB4, 0xA5, 0x96, 0x87, 0x78, 0x69, 0x5A, 0x4B)

IV_AEAD128 = 0x00001000808C0001
IV_XOF128 = 0x0000080000CC0003
IV_CXOF128 = 0x0000080000CC0004

TAG_LEN = 16


def _rotr(x, n):
    return ((x >> n) | (x << (64 - n))) & MASK


def permute(s, rounds):
    """Ascon-p[rounds] on a list of five 64-bit words, in place."""
    x0, x1, x2, x3, x4 = s
    for c in _RC[16 - rounds:]:
        x2 ^= c
        # substitution layer
        x0 ^= x4
        x4 ^= x3
        x2 ^= x1
        t0 = ~x0 & x1 & MASK
        t1 = ~x1 & x2 & MASK
        t2 = ~x2 & x3 & MASK
        t3 = ~x3 & x4 & MASK
        t4 = ~x4 & x0 & MASK
        x0 ^= t1
        x1 ^= t2
        x2 ^= t3
        x3 ^= t4
        x4 ^= t0
        x1 ^= x0
        x0 ^= x4
        x3 ^= x2
        x2 = ~x2 & MASK
        # linear diffusion layer
        x0 ^= _rotr(x0, 19) ^ _rotr(x0, 28)
        x1 ^= _rotr(x1, 61) ^ _rotr(x1, 39)
        x2 ^= _rotr(x2, 1) ^ _rotr(x2, 6)
        x3 ^= _rotr(x3, 10) ^ _rotr(x3, 17)
        x4 ^= _rotr(x4, 7) ^ _rotr(x4, 41)
    s[0], s[1], s[2], s[3], s[4] = x0, x1, x2, x3, x4


def _le(b):
    return int.from_bytes(b, "little")


def _pad(b, size):
    """pad(X, r): the bit 1, then zeros, up to `size` bytes (little-endian)."""
    return bytes(b) + b"\x01" + bytes(size - len(b) - 1)


# --------------------------------------------------------------------------
# Ascon-AEAD128
# --------------------------------------------------------------------------

def _aead_start(key, nonce, ad):
    if len(key) != 16 or len(nonce) != 16:
        raise ValueError("Ascon-AEAD128 needs a 16-byte key and a 16-byte nonce")
    k0, k1 = _le(key[:8]), _le(key[8:])
    s = [IV_AEAD128, k0, k1, _le(nonce[:8]), _le(nonce[8:])]
    permute(s, 12)
    s[3] ^= k0
    s[4] ^= k1
    if ad:
        full = len(ad) // 16 * 16
        for i in range(0, full, 16):
            s[0] ^= _le(ad[i:i + 8])
            s[1] ^= _le(ad[i + 8:i + 16])
            permute(s, 8)
        last = _pad(ad[full:], 16)
        s[0] ^= _le(last[:8])
        s[1] ^= _le(last[8:])
        permute(s, 8)
    s[4] ^= 1 << 63  # domain separation
    return s, k0, k1


def _aead_finish(s, k0, k1):
    s[2] ^= k0
    s[3] ^= k1
    permute(s, 12)
    return ((s[3] ^ k0).to_bytes(8, "little")
            + (s[4] ^ k1).to_bytes(8, "little"))


def aead_encrypt(key, nonce, ad, plaintext):
    """Returns (ciphertext, 16-byte tag)."""
    s, k0, k1 = _aead_start(key, nonce, ad)
    out = bytearray()
    full = len(plaintext) // 16 * 16
    for i in range(0, full, 16):
        s[0] ^= _le(plaintext[i:i + 8])
        s[1] ^= _le(plaintext[i + 8:i + 16])
        out += s[0].to_bytes(8, "little") + s[1].to_bytes(8, "little")
        permute(s, 8)
    rest = plaintext[full:]
    last = _pad(rest, 16)
    s[0] ^= _le(last[:8])
    s[1] ^= _le(last[8:])
    out += (s[0].to_bytes(8, "little") + s[1].to_bytes(8, "little"))[:len(rest)]
    return bytes(out), _aead_finish(s, k0, k1)


def aead_decrypt(key, nonce, ad, ciphertext, tag):
    """Returns the plaintext, or None if the tag does not verify.

    `tag` may be truncated to its leftmost bytes (SP 800-232 section 4.2.1).
    """
    if not 4 <= len(tag) <= TAG_LEN:
        raise ValueError("tag length out of range")
    s, k0, k1 = _aead_start(key, nonce, ad)
    out = bytearray()
    full = len(ciphertext) // 16 * 16
    for i in range(0, full, 16):
        c0 = _le(ciphertext[i:i + 8])
        c1 = _le(ciphertext[i + 8:i + 16])
        out += (s[0] ^ c0).to_bytes(8, "little") + (s[1] ^ c1).to_bytes(8, "little")
        s[0], s[1] = c0, c1
        permute(s, 8)
    rest = ciphertext[full:]
    n = len(rest)
    stream = s[0].to_bytes(8, "little") + s[1].to_bytes(8, "little")
    plain_last = bytes(a ^ b for a, b in zip(rest, stream))
    out += plain_last
    # The rate becomes the ciphertext bytes followed by the untouched state,
    # with the padding bit applied at position n.
    block = bytearray(rest + stream[n:])
    block[n] ^= 0x01
    s[0] = _le(block[:8])
    s[1] = _le(block[8:])
    expected = _aead_finish(s, k0, k1)[:len(tag)]
    diff = 0
    for a, b in zip(expected, tag):
        diff |= a ^ b
    return bytes(out) if diff == 0 else None


# --------------------------------------------------------------------------
# Ascon-XOF128 and Ascon-CXOF128
# --------------------------------------------------------------------------

def _absorb(s, data):
    """Absorb a padded byte string into the 64-bit rate, one block at a time."""
    full = len(data) // 8 * 8
    for i in range(0, full, 8):
        s[0] ^= _le(data[i:i + 8])
        permute(s, 12)
    s[0] ^= _le(_pad(data[full:], 8))
    permute(s, 12)


def _squeeze(s, length):
    out = bytearray()
    while True:
        out += s[0].to_bytes(8, "little")
        if len(out) >= length:
            return bytes(out[:length])
        permute(s, 12)


def xof128(message, length):
    s = [IV_XOF128, 0, 0, 0, 0]
    permute(s, 12)
    _absorb(s, message)
    return _squeeze(s, length)


def cxof128(message, customization, length):
    if len(customization) > 256:
        raise ValueError("customization string longer than 2048 bits")
    s = [IV_CXOF128, 0, 0, 0, 0]
    permute(s, 12)
    s[0] ^= len(customization) * 8  # Z0 = int64(|Z|), in bits
    permute(s, 12)
    _absorb(s, customization)
    _absorb(s, message)
    return _squeeze(s, length)
