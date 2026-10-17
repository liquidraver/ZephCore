# ADR 0010 — zcpfs: how forward-secret messaging sits in the firmware, and when it may default on

Status: accepted, experimental (recorded 2026-10-16; ZephCore-only)

## Context

zcpfs ([../ZCPFS.md](../ZCPFS.md)) seals direct messages with forward secrecy, and requests, logins and
(opt-in) channel messages with Ascon, between ZephCore nodes that both have it, inside ordinary MeshCore
packets. It touches the receive and send paths of every role, the stored contact and access-list data, the
prefs and the advert. It has run on a bench of three nodes beside official MeshCore, and has host tests down
to whole nodes over a simulated channel, but no cryptographic review and no field test on a live mesh.

## Decision

1. **Two switches.** `CONFIG_ZEPHCORE_ZCPFS` (Kconfig, default `n` on `dev` and `master`) compiles the
   library and the glue in; without it the firmware is unchanged. In a build with it, `set zcpfs on|passive|off`
   (prefs, default `off`) decides at run time; `off` is legacy on the air. A node leaves through `passive`:
   no advert tag, nothing started, but whatever arrives sealed is read and answered, so peers that still pin
   it lose nothing until its untagged advert reaches them. On the `zcpfs` branch itself the Kconfig default is
   `y` (2026-10-17), so a build from this branch needs nothing extra to get the code in — a development
   convenience, scoped to this branch only, that does not touch the run-time default and is not the decision
   the gate below is about: `dev` and `master` keep the Kconfig default `n`, and this branch's default is not
   carried across a merge without going through that gate.
2. **A portable library, a thin glue.** The protocol and its state machine are `lib/zcpfs` (C99, no Zephyr, no
   MeshCore), with Ascon in `lib/ascon`; the firmware side is `helpers/zcpfs/` (`ZcpfsChat` for companions,
   `ZcpfsServer` for repeaters and rooms, `ZcpfsCommon.h`, `ZcpfsSeed.cpp`). Roles only add fenced calls.
3. **Hooks in the Shared mesh files, fenced.** `mesh::Mesh` offers packets to zcpfs before legacy decryption
   (`onPeerSealedRecv`, `onAnonSealedRecv`, `onStrangerSealedRecv`, `onGroupSealedRecv`, `onPeerRecvDone`)
   and lets it build payloads (`sealOutgoing`, `sealGroupOutgoing`); `BaseChatMesh` and `ClientACL` get fenced
   calls. Every line is inside `#ifdef CONFIG_ZEPHCORE_ZCPFS` and marked `// ZEPHCORE:`, so a diff against
   upstream shows it as intentional divergence. An opened packet re-enters upstream's code as the plaintext
   legacy decryption would have produced, so message, request, CLI and login handling stays upstream's.
4. **Where state lives.** The pin of a contact is bit `0x80` of `ContactInfo.flags` (the companion protocol's
   flag writes keep it); the pin of a client is the first of the two bytes of each ACL file record that
   upstream writes as zero and never reads. Sessions are RAM only, on purpose. Settings are `zc{}` keys of
   `prefs.json` (`zcpfs`, `zcpfs_nolegl`, `zcpfs_ch`); the generator's seed is `/lfs/zcpfs_seed`. No storage
   format changes.
5. **Rules the code keeps.** zcpfs never floods on its own: only the application's own send or login may
   flood, and only where legacy would. Every interval and limit is derived from the LoRa preset and the
   firmware's own timeouts. No limit reacts to unanswered replies (on a lossy mesh a lost reply and a guess
   look the same).
6. **Legacy stays reachable.** Legacy nodes see the tagged advert's name unchanged and compute the same ACK for
   a hinted text; legacy logins work both ways unless a node opts out (`set zcpfs.legacylogin off`); channel
   sealing is per channel and opt-in.

## Consequences

- Upstream ports into `Mesh.cpp`, `Mesh.h`, `BaseChatMesh.*` and `ClientACL.*` must carry the fences along;
  a change to the receive path before legacy decryption must consider the hooks. The host test executable
  `zephcore_zcpfs_mesh` (built with the switch on) runs those files and fails if the integration breaks.
- If upstream starts using bit 7 of `ContactInfo.flags` or the ACL record's reserved bytes, the pin has to
  move. Upstream uses flag bits 0 to 3 today.
- The mesh packet pool is one static per process; host tests of several nodes use their own packet manager.
- Host tests need OpenSSL's libcrypto (the firmware's `Utils.cpp` runs over a PSA stand-in).

**Before either default changes:**

1. A cryptographic review of `lib/zcpfs/SPEC.md` and the library, by someone outside the project.
2. Field tests on a live mesh: loss, several hops, SF8 as well as SF7, at least two days with real traffic.
3. The phone apps over BLE: DMs, logins and channels on the official apps against zcpfs nodes.
4. One release with `CONFIG_ZEPHCORE_ZCPFS=y` and the mode `off`, to show that a build with the code in it
   behaves as one without; only then a decision on the mode's default, in a new ADR.
