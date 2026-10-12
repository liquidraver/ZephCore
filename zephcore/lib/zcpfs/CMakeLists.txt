# SPDX-License-Identifier: MIT
# Zephyr build of the zcpfs library (README.md, SPEC.md). Portable C99 with no
# Zephyr calls; it needs lib/ascon and the vendored Monocypher.

zephyr_library_named(zcpfs)
zephyr_library_sources(
    zcpfs_prim.c
    zcpfs_curve.c
    zcpfs_drbg.c
    zcpfs_handshake.c
    zcpfs_session.c
    zcpfs_stateless.c
    zcpfs_login.c
    zcpfs_peer.c
)
zephyr_library_include_directories(include . ../ascon ../monocypher)
