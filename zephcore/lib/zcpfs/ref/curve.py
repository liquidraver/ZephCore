# SPDX-License-Identifier: MIT
"""Curve25519 arithmetic for the zcpfs reference model: X25519 (RFC 7748), the
Ed25519 public key of a scalar, and the Edwards-to-Montgomery map.

Plain big-integer code. Not constant-time; not for use in firmware.
"""

P = 2 ** 255 - 19
_A24 = 121665
_D = (-121665 * pow(121666, P - 2, P)) % P
_BX = 15112221349535400772501151409588531511454012693041857206046113283949847762202
_BY = 46316835694926478169428394003475163141307993866256225615783033603165251855960

BASE_U = (9).to_bytes(32, "little")


def clamp(scalar):
    k = bytearray(scalar[:32])
    k[0] &= 248
    k[31] &= 127
    k[31] |= 64
    return bytes(k)


def x25519(scalar, u_bytes):
    """RFC 7748 section 5. The scalar is clamped here, as monocypher does."""
    k = int.from_bytes(clamp(scalar), "little")
    x1 = int.from_bytes(u_bytes, "little") & ((1 << 255) - 1)
    x2, z2, x3, z3, swap = 1, 0, x1, 1, 0
    for t in reversed(range(255)):
        bit = (k >> t) & 1
        swap ^= bit
        if swap:
            x2, x3, z2, z3 = x3, x2, z3, z2
        swap = bit
        a = (x2 + z2) % P
        aa = a * a % P
        b = (x2 - z2) % P
        bb = b * b % P
        e = (aa - bb) % P
        c = (x3 + z3) % P
        d = (x3 - z3) % P
        da = d * a % P
        cb = c * b % P
        x3 = (da + cb) % P
        x3 = x3 * x3 % P
        z3 = (da - cb) % P
        z3 = x1 * z3 * z3 % P
        x2 = aa * bb % P
        z2 = e * (aa + _A24 * e) % P
    if swap:
        x2, x3, z2, z3 = x3, x2, z3, z2
    return (x2 * pow(z2, P - 2, P) % P).to_bytes(32, "little")


def x25519_public(scalar):
    return x25519(scalar, BASE_U)


def _edwards_add(p, q):
    x1, y1 = p
    x2, y2 = q
    k = _D * x1 * x2 * y1 * y2 % P
    x3 = (x1 * y2 + x2 * y1) * pow(1 + k, P - 2, P) % P
    y3 = (y1 * y2 + x1 * x2) * pow(1 - k, P - 2, P) % P
    return x3, y3


def ed25519_public(scalar):
    """Encoded Ed25519 point scalar * B, for an already expanded scalar.

    MeshCore stores the expanded private key; its first 32 bytes are this
    scalar. Seed expansion (SHA-512) is not part of this function.
    """
    k = int.from_bytes(clamp(scalar), "little")
    result = (0, 1)
    addend = (_BX, _BY)
    while k:
        if k & 1:
            result = _edwards_add(result, addend)
        addend = _edwards_add(addend, addend)
        k >>= 1
    x, y = result
    return (y | ((x & 1) << 255)).to_bytes(32, "little")


def elligator2(r_bytes):
    """Elligator 2 on Curve25519, as monocypher's crypto_elligator_map: the two
    top bits of the input are ignored, the non-square is 2, the output is the
    Montgomery u-coordinate of a point on the curve."""
    a = 486662
    r = int.from_bytes(r_bytes, "little") & ((1 << 254) - 1)
    w = -a * pow(1 + 2 * r * r, P - 2, P) % P
    on_curve = pow((w * w * w + a * w * w + w) % P, (P - 1) // 2, P) == 1
    u = w if on_curve else (-w - a) % P
    return u.to_bytes(32, "little")


def edwards_to_montgomery(ed_public):
    """u = (1 + y) / (1 - y), RFC 7748 section 4.1."""
    y = int.from_bytes(ed_public, "little") & ((1 << 255) - 1)
    return ((1 + y) * pow(1 - y, P - 2, P) % P).to_bytes(32, "little")
