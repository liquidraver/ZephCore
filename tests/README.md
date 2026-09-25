# Hardware-free regression tests

Run from the repository root:

```sh
python tests/run.py run --profile quick
```

This first implementation builds **actual production C++ sources** and executes
the named native cases listed in `catalog.json` (114 on 2026-09-25) plus the Python
harness self-tests. No boards, Zephyr workspace,
west, network access or third-party Python packages are required. This is the first
subset of the [regression plan](../docs/REGRESSION_TEST_PLAN.md), not completion of
that plan or a release qualification suite.

## Prerequisites

- Python 3.10 or newer.
- CMake 3.20 or newer and Ninja on PATH.
- Host GCC or Clang C and C++ compilers, with C++17 support. C is used for the
  bundled production Monocypher library.

On Windows, the runner also looks for GCC in the standard MSYS2 `ucrt64` and
`mingw64` locations, and Clang in `clang64`. It adds the selected compiler's directory
to the child PATH so compiler tools and runtime DLLs can be found. Override discovery
with `CXX` set to an executable name or full path (not a shell command with flags).
The runner currently supports GCC/Clang discovery, not automatic Visual Studio setup.

```powershell
$env:CXX = 'C:/msys64/mingw64/bin/g++.exe'
python tests/run.py run --profile quick
```

On Linux, run the instrumented profile:

```sh
python3 tests/run.py run --profile quick --sanitize
```

This enables AddressSanitizer and UndefinedBehaviorSanitizer with failing diagnostics
by default. Sanitizer execution is not supported by this target on Windows. The Linux
test executable is non-PIE to avoid sanitizer address-space placement conflicts.

## Commands

```sh
# Enumerate the implemented native cases without compiling.
python tests/run.py list --profile quick

# Reproduce one native case; harness self-tests still run first.
python tests/run.py run --case UNIT-PACKET-002

# Run only the Python runner's self-tests.
python -m unittest discover -s tests/harness_tests -v
```

`quick` is the only implemented profile. Unknown cases/profiles and empty collections
fail explicitly. The catalog is JSON to keep the first runner dependency-free; it
serves the catalog role proposed as YAML in the plan. Do not regenerate the catalog
on ordinary runs: the runner compares it to the compiled executable's full case list
and rejects missing, unexpected, renamed or duplicate cases.

## What is covered

| Production code | Cases |
|---|---|
| `src/Packet.cpp` | All path encodings and header bytes, payload boundaries, truncated headers/paths, independent wire vector, zero-hop width regression, 2,000 valid generated round trips, 8,192 arbitrary parser inputs, hash input rules |
| `src/StaticPoolPacketManager.cpp` | Capacity/reuse, due-time and priority ordering, reschedule/remove, time wrap, 5,000 generated ownership transitions |
| `src/ContentionTracker.cpp` | Observation boundaries/wrap, duplicate saturation, exactly-once finalization, reactive caps, warmup/eviction and time-based decay |
| `helpers/AdvertDataHelpers.cpp`, `UTF8Helpers.h` | Independent advertisement bytes, optional-field truncation, bounded names, UTF-8 invalid sequences and codepoint boundaries |
| `helpers/RoutingPolicy.h` | Reply route/scope truth tables and scoped/unscoped/advert hop limits |
| `helpers/RateLimiter.h` | Normal window limit, expiry and reset only |
| `include/mesh/Maintenance.h` | Due/past/future deadlines, wraparound and idle composition |
| `src/Dispatcher.cpp` | Immediate wakes, scheduled TX, completion/timeout ownership, priority-preserving refusal, LBT starvation across retry gaps, final RX gate, busy recovery, budget refill/admin exception, RX draining/hold/retransmit, maintenance, truncated headers, stats reset, clock wrap and 4,096 generated parser inputs |
| `src/Identity.cpp` and bundled Monocypher | Two RFC8032 Ed25519 known-answer vectors, tampered signatures/messages, wrong signer, canonical/legacy/private-only storage, failed-read state preservation and bidirectional ECDH |

The catalog maps stable case IDs to their test source files. Production paths above
are relative to `zephcore/`. Generated-case seeds are fixed and documented in the
report; this version has no `--seed` override. Assertion output includes source and
line. Native cases run in separate processes because the production pool has global
state; tests never patch/reset that private state. Pool tests exercise repeated use
within each case and return owned slots through public APIs.

External boundaries are deliberately limited:

- Zephyr logging: a test-local header discards messages. It does not emulate a kernel.
- SHA-256: a test-local definition captures the hash input and returns marker bytes.
  This verifies **Packet's choice of bytes to hash, not cryptographic correctness**.
  `Utils.cpp` is not linked; the fake cannot support real crypto or mesh integration.
- Dispatcher uses a scripted `Radio`, fake millisecond clock and the **real** static
  packet pool. Tests override CAD retry delay to 100ms; the default jitter's random
  call has a deterministic host substitute. This exercises scheduler logic, not IRQs
  or kernel concurrency. Packet logging is disabled explicitly in this target.
- Identity's hex constructors, RNG constructor, private-key validation and recovery
  depend on Utils integration that is not implemented here. Those utility boundaries
  throw if reached, rather than silently producing fake cryptographic results.
  `fromSeed`, signing/verification, ECDH and storage serialization use the actual
  production Identity and Monocypher code, including Monocypher's real SHA-512.

The Ed25519 seeds, public keys and signatures are public test vectors from
[RFC 8032 section 7.1, tests 1 and 2](https://www.rfc-editor.org/rfc/rfc8032.txt).
They are fixed in the test source and never fetched during a run. ECDH currently has
a bidirectional agreement/nonzero property, not an independent known-answer vector.
Storage compatibility tests exercise all three layouts and verify restored signatures;
an independently archived historical storage corpus is still planned.

Golden byte vectors are authored independently of the serializer. Generated round
trips supplement them. Native little-endian wire expectations match the current
supported MCU targets; this suite does not establish big-endian compatibility.

Not yet covered: Mesh/application roles, PSA AES/HMAC/SHA-256, the remaining identity
entry points described above, filesystem,
Zephyr kernel behavior, driver/hardware behavior, coverage percentages, exhaustive
feature configurations, and RateLimiter overflow/wrap. The broader source inventory,
fuzz engine, fixture corpus and mutation automation remain planned. There are no
passing placeholders for these areas.

## Results and failure behavior

Each invocation creates unique directories under ignored `build-tests/` and
`test-results/`. Builds are isolated between concurrent invocations; the initial
small suite rebuilds on each run rather than sharing a mutable build cache.

The result directory contains:

- `results.json`: statuses, selected IDs, source hashes including untracked test
  files, Git revision/status/diff hash, compiler version, host/Python metadata,
  binary hash, fixed seeds, sanitizer options and executed build commands.
- `junit.xml`: native and harness results for CI. Missing prerequisites are BLOCKED
  in JSON and errors in JUnit, never successful skips.
- `configure.log`, `build.log`, `harness.log` and one log per executed native case.

Exit code 0 means every selected native case and harness test passed. Selection or
argument errors return 2. Build errors, crashes, timeouts, assertion failures or
incomplete execution return nonzero and retain available diagnostics. No automatic
retries. A child must both exit successfully and emit its exact case success marker.
The runner captures subprocess output directly; it never treats old logs as results.

Per-native-case deadline is 10 seconds, discovery 30 seconds, and each build phase
180 seconds. Hard termination or disk failure can prevent report writing; absence of
a complete report is not a pass. Build/result directories can be removed when no run
is active; they are disposable. No personal lab configuration is needed for `quick`.

## Adding a test

1. Add a `TEST(function_name, "UNIT-AREA-NNN", "description")` to a relevant
   `unit/*_tests.cpp` file; add a new source to CMake if needed.
2. Add its exact ID, description and source path to `catalog.json`.
3. Test a behavioral contract with independent expected values. Use `CHECK`, which
   remains active in release builds; do not use C `assert` for test assertions.
4. Keep hardware calls behind explicit boundaries. Never copy a production algorithm
   into a test substitute or weaken an expected result to match an observed defect.
5. Run the case and full quick profile; run Linux sanitizers for memory/parser changes.

Harness self-tests cover empty selection, unknown IDs, catalog disagreement,
subprocess failures/timeouts, missing executables, false success markers and reports.
They are part of every quick run. The dedicated CI workflow runs on PRs and pushes
to master; it uploads reports even when checks fail and has no release permissions.
