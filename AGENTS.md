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

## Invariants: do not change without an ADR

- Byte compatibility with Arduino MeshCore on the air and on the companion protocol; `prefs.json` format.
- Settings persist in `prefs.json` only. The legacy binary prefs files are frozen: no new fields in
  `PrefsCodec`, no change to its size constants or to the pinned offsets and lengths in its tests.
- The wall clock only moves forward.
- Mesh, GPS state and UI run on the main thread; loops are event-driven, no polling.
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
- Host tests: `python3 tests/run.py run --profile quick` (Linux or WSL). A new case needs a
  `tests/catalog.json` entry. A failing pinned test is a finding: do not edit the pinned values to make it pass.
- PR CI runs the host tests only. Build every board you touched and one you did not, and run each role you
  changed. Releases build from `zephcore.yml` alone, so nothing may depend on a hand-passed fragment.
- New board: `zephcore.yml` without `release:`, then `python zephcore/scripts/board_manifest.py check` and
  `... docs`.
- Docs in the same PR: a clause in `docs/ARCHITECTURE.md` when behaviour, an interface or a limit changes;
  `docs/CLI_commands.md` for any CLI change; an ADR when a decision is taken or reversed.

## PR description

- Cause, fix, and what was deliberately left out.
- Which boards it was built for, and which hardware it actually ran on. Never claim hardware verification that
  was not done.
- Flash and RAM delta when code is added to a common path.
