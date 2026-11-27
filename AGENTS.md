# AGENTS.md

Rules for AI-assisted contributions to ZephCore. They apply to human contributors too. Read the section of
`docs/DESIGN.md` (invariants, layer rules) and `docs/ARCHITECTURE.md` (component reference) for the area you
touch before writing code. Decisions are in `docs/adr/`.

## How a change is judged

1. **Find the root cause.** The PR states the cause and the evidence for it (datasheet section, register dump,
   log, trace). If the cause is not known, say so; do not ship a guess.
2. **No band-aids.** Do not add a retry, delay, timeout, re-init, read-back check or watchdog to make a symptom
   go away. Recovery code is accepted only for a named, understood hardware failure, in the layer that owns the
   state, with its bound (`docs/DESIGN.md` §8.7). There is no hardware watchdog and none will be added.
3. **The smallest change that fixes it.** Close what is reachable today, not what might be reachable later. No
   speculative guards, build asserts, new abstractions, options or refactors riding along. One concern per PR.
   If the diff is larger than the bug, expect to be asked to cut it.
4. **Do what upstream MeshCore does** when it already solves the problem, unless ours can be more accurate at no
   compatibility cost (`docs/DESIGN.md` §9).
5. **Comment diet.** A comment says what the code does or the invariant it keeps, in a few lines. Rationale,
   history, incidents, measurements and rejected alternatives go in the PR description and `docs/`, not in code.
   Do not comment the obvious.
6. **Fix it for every board.** A fix or feature that concerns behaviour goes in the common path, for all
   platforms and roles. Do not scope it to the board or SoC family where it was found or tested, and do not
   exclude a platform because the problem cannot happen there today; a later port or Zephyr bump can make it
   real. Untested boards still get the change; the PR says which hardware it ran on.
7. **Gate on the capability, never on the name.** The only reason to gate code is that the hardware is absent,
   and the test for that is a devicetree node or property, or a Kconfig symbol for the feature. Never
   `CONFIG_SOC_*`, `CONFIG_BOARD_*` or a board name as a stand-in for "has this part" or "has this bug". A board
   that lacks the hardware then carries no code for it, and a new board that has it gets the behaviour for free.
8. **A real platform difference is stated as one.** If something truly is specific to one SoC or chip (an
   erratum, a vendor blob, a peripheral that exists nowhere else), it lives in that platform's layer and the PR
   cites the source that makes it specific. "Only seen on X" is not that.
9. **Chip state belongs to the driver.** A register or mode is set inside the driver, while the chip is awake and
   under the driver's lock, and must survive every sleep the driver uses (SX126x: the warm-start retention
   list). A write from an adapter hook after `begin()` is lost at the next sleep. Bus errors are returned to the
   caller, not discarded with `(void)`.

## Invariants: do not change without an ADR

- Byte compatibility with Arduino MeshCore on the air and on the companion protocol; `prefs.json` format.
- Settings persist in `prefs.json` only. The legacy binary prefs files are frozen: no new fields in
  `PrefsCodec`, no change to its size constants or to the pinned offsets and lengths in its tests
  (`tests/support/prefs_layout.h`).
- The wall clock only moves forward.
- Mesh, GPS state and UI run on the main thread; loops are event-driven, no polling. A button, UI or work-queue
  thread that needs the mesh or the Bluetooth host sets a flag in `pending_ui_actions` and lets
  `mesh_handle_ui_actions()` make the call on the main thread; it never calls in directly.
- No heap in the packet path. Companion builds are RAM-bound.
- Never write to a peripheral that was not cleanly identified first.
- BLE: no proactive SMP Security Request, keep `BT_LE_ADV_OPT_USE_IDENTITY` (ADR 0004), leave connection
  parameters alone.
- Files ported from MeshCore (listed in `.editorconfig`) keep upstream's text verbatim; every change in them is
  fenced `// ZEPHCORE:`. Do not restyle or de-duplicate them.
- `zephcore/patches/zephyr`: one patch owns each Zephyr file.
- `board.conf` does not repeat what a parent conf or the devicetree already provides.

## Mechanics

- Branch from `dev`, target `dev`, rebase instead of merging.
- After a rebase, re-read the whole diff against `dev`: a branch older than the code around it can undo newer
  work without a conflict.
- Tabs for indentation, except the upstream-ported files.
- Build with `--pristine` when the board or role changes; ESP32 S3/C-series repeaters need `--sysbuild`.
- Host tests: `python3 tests/run.py run --sanitize` (Linux or WSL). A new case is a `TEST()` in `tests/unit/`.
  A failing pinned test is a finding: do not edit the pinned values to make it pass.
- Logic that runs without hardware (a protocol parser, a state machine, a timing rule, a `// ZEPHCORE:` change
  to upstream behaviour) comes with host tests that pin it. Driver code in a Zephyr patch counts too: extract the
  functions from the patch at configure time, as `tests/support/extract_sx126x_helpers.py` does, and fake the bus.
- A changed `zephcore/patches/zephyr` patch must apply to the pinned Zephyr; check it before pushing.
- Remove what the change orphans: a hook, API or Kconfig symbol left with no user goes in the same PR. Grep
  docs, comments and patch headers for every name and number you changed.
- PR CI runs the host tests only. Build every board you touched and one you did not, and run each role you
  changed. Releases build from `zephcore.yml` alone, so nothing may depend on a hand-passed fragment. A role or
  fragment no `zephcore.yml` row builds goes into the "compile-only roles" step of `.github/workflows/build.yml`.
- A new role or composition root puts every part the board has into a defined state, used or not: GPS parked,
  LEDs off, display blanked, as `src/server_main_common.cpp` does.
- New board: `zephcore.yml` without `release:`, then `python zephcore/scripts/board_manifest.py check` and
  `... docs`.
- Docs in the same PR: a clause in `docs/ARCHITECTURE.md` when behaviour, an interface or a limit changes;
  `docs/CLI_commands.md` for any CLI change; an ADR when a decision is taken or reversed; the next release's
  `releasenotes/RELEASE_NOTES_<version>-zephcore.md` for anything a user sees.

## PR description

- Cause, fix, and what was deliberately left out.
- Which boards it was built for, and which hardware it actually ran on. Never claim hardware verification that
  was not done.
- Flash and RAM delta when code is added to a common path.
