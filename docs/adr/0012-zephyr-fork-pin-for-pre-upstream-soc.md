# ADR 0012 — Zephyr pinned to a fork commit while a SoC port is not upstream

Status: accepted (recorded 2026-11-22)

## Context

ADR 0005 pins Zephyr to a commit on upstream `main`. ESP32-S31 support exists only on Espressif's
`feature/esp32s31-initial-support` branch (12 commits on top of upstream `main` of 2026-11-13: SoC, DTS,
clock control, shared-driver enablement, the `esp32s31_function_coreboard` board). Carrying those commits
as `patches/zephyr/` would break the one-patch-per-file rule across dozens of files and would have to be
regenerated on every bump.

## Decision

`zephcore/west.yml` may point the `zephyr` project at a public fork (`github.com/marekmatej/zephyr`) at a
pinned commit whose history is upstream `main` plus the pre-upstream SoC branch, with a comment naming
the upstream base and the branch. `import: true` stays, so hal_espressif and the other modules come from
that commit's manifest (hal_espressif `main`, which already carries the S31 sources). Everything else in
ADR 0005 applies unchanged.

## Consequences

- CI and contributors build S31 without local checkouts; `west update` works from a clean clone.
- The fork is a mirror, not a place for ZephCore changes: `patches/zephyr/` remains the only carrier of
  ZephCore's own Zephyr modifications.
- Exit condition: when the S31 SoC lands upstream, the pin returns to `zephyrproject-rtos` on the next
  bump and this ADR is marked superseded.
