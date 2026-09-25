# ZephCore repeatable regression suite

Status: implementation roadmap, based on source at `1aa629c` (2026-09-21).
A first hardware-free subset now exists in [tests/README.md](../tests/README.md):
`quick`, exact `--case` selection, 45 native C++ cases, 10 harness self-tests,
JSON/JUnit reporting and Linux sanitizer CI. It uses a JSON catalog and standalone
host CMake target; Zephyr-dependent layers still require separate integration targets.
Only the commands and coverage documented in that README are implemented. The broader
profiles, options, fixtures and applications below remain planned. No devices were
flashed or exercised during this implementation.

Implemented follow-up: Dispatcher scheduling, RX parsing and recovery cases plus
real Monocypher-backed identity/signature/storage tests. The new truncated-header
case reproduced an out-of-bounds read under AddressSanitizer; Dispatcher now checks
the received extent before reading route header fields. Full milestone completion,
kernel integration, application roles and hardware profiles remain outstanding.

## Objective

After a change, run the same version-controlled cases against an identified build,
produce comparable evidence, and identify precisely what passed, failed, or was not
tested. Start with radio RX/TX and packet handling; grow into all firmware roles and
subsystems. No finite suite proves that every future change is safe.

The deliverable is a regression package with unit, integration, hardware-in-the-loop
(HIL), interoperability, and endurance tests. Most edge cases should run without
boards. Hardware tests verify the driver, timing, and RF behavior that mocks cannot.

## Existing foundations and gaps

- `zephcore/src/Packet.cpp`: serialization, bounds, path encoding, packet hashes.
- `Dispatcher.cpp`, `Mesh.cpp`, `StaticPoolPacketManager.cpp`, `ContentionTracker.cpp`:
  scheduling, routing, ownership, deduplication, contention and airtime accounting.
- `zephcore/include/mesh/Radio.h`, `Clock.h`, `RNG.h`, `Dispatcher.h`: existing seams
  for fake radio, time, randomness, and packet management.
- `zephcore/adapters/radio/LoRaRadioBase.*`: shared radio state; chip-specific
  `hw*()` methods provide a useful seam, but the base still needs a Zephyr kernel.
- `zephcore/patches/`: radio behavior also depends on patched Zephyr/module drivers.
- `zephcore/app/` and `helpers/`: companion, repeater, room, observer, storage,
  transport, routing policies and time synchronization all need their own coverage.
- `tests/` holds the host suite (published 2026-09-25); `.github/workflows/tests.yml`
  runs it with ASan and UBSan on pull requests and pushes to master, separate from
  the firmware build and release workflow.
- The Linux native application uses real SPI/GPIO hardware. It is not automatically
  a simulated radio network; a fake-radio test target must be added separately.

## Test layers and profiles

| Layer | Runs against | Main purpose | Proposed profile |
|---|---|---|---|
| Unit | Actual production C/C++ with controlled dependencies | Exhaustive boundaries, exact state transitions | `quick` |
| Zephyr integration | Test applications on `native_sim` | Queues, timers, events, transport, shared radio base | `integration` |
| Build | Board/role/feature configurations | Compile, link, memory and packaging regressions | `build` |
| HIL smoke | Four physical boards | Boot, pairwise RX/TX, messages, recovery | `bench-smoke` |
| HIL extended | Boards with role/preset rotation | Radio matrix, routing, failures, persistence | `bench-full` |
| Endurance | Fixed bench scenario | Leaks, stalls, drift, recovery over hours | `soak` |
| Release | Above plus reference firmware and release images | Compatibility and packaging qualification | `release` |

Target budgets after build caching: quick under 2 minutes, integration under 10,
smoke 10–20 minutes, full 1–3 hours, soak 8–24 hours. These are design budgets,
not measurements; slow modem presets and duty-cycle limits may require longer.
Estimate airtime and duration before each hardware run.

Use Ztest/Twister for C/C++ and Zephyr tests, and pytest for bench orchestration,
parameterization and reports. Initially keep the multi-board bench orchestration
in pytest so it can control mixed firmware roles and flashing methods explicitly.
Validate all tool options against the Zephyr revision pinned in `west.yml` before
implementation; current online documentation can describe newer features.

Use Linux CI/devcontainer for native simulation and sanitizers. Run bench Python
on Windows where the serial ports and debug probes are connected; exchange builds
through an artifact manifest. Avoid requiring Windows USB passthrough into Linux.

## Hardware-free coverage is the main suite

Most behavioral regression cases should execute without connected boards, including
cases inside large application `.cpp` files. Physical hardware is needed to validate
the boundary, not every decision behind it. A Zephyr dependency does not itself
imply a hardware dependency: kernel-backed tests can run in native simulation.

The table is a source-level implementation backlog, not a claim that these files
already compile standalone. Include chains, global symbols and configured features
must be resolved as each target is brought up. Paths are relative to `zephcore/`.

| Production source / group | Hardware-free behaviors to test | Required environment or seam | Order |
|---|---|---|---|
| `src/Packet.cpp` | Decode/encode, lengths, path copies, headers, hash semantics | Real hash backend for golden hash cases; no radio | 1 |
| `src/StaticPoolPacketManager.cpp` | Capacity, ownership, priority, scheduling, rescheduling | Controlled timestamps; real pool | 1 |
| `src/ContentionTracker.cpp` | Samples, warmup, eviction, decay, caps and overflow boundaries | Supplied timestamps and packet fixtures | 1 |
| `helpers/AdvertDataHelpers.cpp` | Flags, coordinates, names, UTF-8 truncation, malformed input | Byte fixtures; no device | 1 |
| `src/Identity.cpp` | Seed expansion, persisted expanded keys, signing, verification, shared secrets | Bundled real Monocypher; deterministic test RNG where needed | 1 |
| `src/Utils.cpp` | Hex/conversion helpers and crypto wrappers, failure handling | Software PSA backend; injected PSA errors/reboot capture in separate fault tests | 1 |
| `src/Dispatcher.cpp` | Queue dispatch, CAD/LBT timing, TX failure, budgets, counters, maintenance deadlines | Fake Radio/clock and actual packet manager; small platform shims or native_sim | 1 |
| `src/Mesh.cpp` | Routing, dispatch, dedup, packet construction, decrypt/verify, response decisions | Deterministic Radio/clock/RNG; real packet/crypto code; capture callbacks | 2 |
| `helpers/BaseChatMesh.cpp` | Contacts, advert updates, route selection, pending ACKs and retry state | Mesh fixture, fake time and captured delivery callbacks | 2 |
| `helpers/ClientACL.cpp` | Client lookup/update, capacity, permissions, save/load validation | Real identity code; filesystem fake or native filesystem fixture | 2 |
| `helpers/TransportKeyStore.cpp` | Derivation, transport codes, cache replacement and key lookup | Real PSA crypto; controlled key storage | 2 |
| `helpers/RegionMap.cpp` | Region names/hierarchy, scope decisions, defaults, removal, persistence | Real key-store behavior; filesystem fixture | 2 |
| `helpers/MeshTimeSync.cpp` | Quorum, tenure, stale samples, inconsistent voters, bootstrap, suppression, forward-only steps | Controlled uptime/RTC and clock-set hooks; no real waiting or GPS | 2 |
| `app/CompanionMesh.cpp` | Command parsing, response bytes, contacts/channels, offline queue, message send/receive, authorization | Native integration fixture with storage, board, BLE-output, GPS/sensor and reboot boundaries substituted | 3 |
| `app/RepeaterMesh.cpp` | Forward/drop decisions, ACL, neighbor state, requests, rate limits, deadlines | Real Mesh plus fake external services and captured transmissions | 3 |
| `app/RoomServerMesh.cpp` | Login, permissions, posts, history limits, fanout, retry and slow-client behavior | Multiple simulated clients, fake time/storage, captured packets | 3 |
| `app/ObserverMesh.cpp` | Receive processing, publication content, filtering, no LoRa TX | Recording publisher and fake radio | 3 |
| `helpers/CommonCLI.cpp`, `app/*RegionCLI.cpp` | Parsing, numeric limits, invalid arguments, preference changes, replies, deferred effects | Capture board/radio/settings side effects; native kernel for work items | 3 |
| `app/RepeaterUplink.cpp` | Command validation, JSON content, status and publish decisions | Recording WiFi/MQTT boundary; no network account | 3 |
| `app/RepeaterDataStore.cpp`, `adapters/datastore/ZephyrDataStore.cpp`, `app/observer_creds.cpp` | Serialization, migration, atomic replacement, truncation, short writes, full filesystem | Native filesystem/flash simulation plus explicit error injection | 3 |
| `adapters/radio/LoRaRadioBase.cpp` | State transitions, config/clamps, counters, CAD adaptation, recovery and maintenance | Real Zephyr kernel with fake `hw*()` primitives; event scheduling control | 3 |
| `adapters/radio/{SX126x,LR1110,LR2021,SX127x}Radio.cpp` | Driver argument mapping, feature support, error propagation | Fake driver API/SPI boundary; hardware still validates actual chip behavior | 4 |
| `adapters/usb/*.cpp`, `adapters/ble/ZephyrBLE.cpp` | Framing/queues, congestion, lifecycle and callback ordering | Native kernel, simulated connection and transport callbacks | 4 |
| `adapters/gps/ZephyrGPSManager.cpp` | Fix acquisition, timeout, duty policy, invalid/stale readings | Fake GNSS events, uptime, power-control calls | 4 |
| `adapters/clock/*.cpp`, `adapters/rng/*.cpp`, `adapters/sensors/ZephyrEnvSensors.cpp` | Conversion, fallback, missing-device and error behavior | Fake device API; actual software algorithms retained | 4 |
| `helpers/ui-joystick/`, `helpers/ui/ui_mesh_actions.cpp` | Input sequences, state transitions, selected actions and text bounds | Recorded input/render/action interfaces; visual and electrical checks remain separate | 4 |
| `src/main_*.cpp`, `app/main_observer.cpp`, `adapters/board/ZephyrBoard.cpp` | Initialization failure, event dispatch, sleep/wake decisions, reboot requests | Selected native integration scenarios; substantial board glue remains build/HIL tested | 4 |

Header-only and C code must be included too: `RoutingPolicy.h`, `RateLimiter.h`,
`SimpleMeshTables.h`, `UTF8Helpers.h`, `TxtDataHelpers.h`, `MeshcoreJson.h`,
`compat/CayenneLPP.h` (covered: UNIT-LPP-001..004), battery conversion, shared companion framing and transport C files.
For example, exhaust every reply-route Boolean combination and flood-limit boundary;
exercise the rate limiter after prolonged denials, counter saturation, timestamp
wrap and reset. These cases need no physical devices and can catch real defects.

### How to test the large classes

Keep the production class as the system under test. Construct a fixture around it:

```text
scripted incoming bytes/commands → actual CompanionMesh / RepeaterMesh / RoomServerMesh
                                      ↕ real Mesh, Packet, queues, crypto
fake clock + seeded RNG + storage → fixture → recorded outgoing bytes and state
```

Assertions describe observable behavior: exact response bytes, intended recipient,
number of transmissions, stored settings, delivered messages and ownership after
completion. Avoid a test that merely asserts each internal helper was called.
Use existing virtual interfaces first. For direct global/device calls, introduce
small service boundaries or link-time fakes with documented contracts; prefer
native Zephyr integration when emulating the kernel would become a second project.
Do not fake all of storage and then claim to test its real persistence implementation.

Examples of executable hardware-free scenarios:

1. Feed a channel-message command into CompanionMesh; inspect its actual emitted
   packet; deliver it to a second real mesh instance with the same channel; assert
   one matching app event. Repeat with a wrong key and a truncated body.
2. Let RepeaterMesh receive a flood at each configured hop boundary; assert the
   correct forwarding decision and path bytes, including region policy.
3. Let RoomServerMesh accept clients, post a message, lose an ACK and advance fake
   time; verify fanout/retry state without sleeping or using a radio.
4. Feed MeshTimeSync days of synthetic signed-advert metadata in milliseconds of
   test runtime; verify quorum and suppression boundaries. Separately test that
   callers never feed unverified adverts into the estimator.
5. Drive real datastore code through short writes and failed rename operations;
   reopen the same simulated volume and verify the documented recovery contract.
6. Feed fragmented companion frames through the real parser, interleave unrelated
   push events, and verify command responses survive queue pressure.

### Coverage accounting and implementation order

Maintain a tracked inventory mapping every owned production `.cpp`, relevant `.c`
and logic-bearing header to test targets, covered behaviors, missing behaviors and
remaining hardware-only checks. New unclassified files fail an inventory check.
An intentional exclusion needs a reason; vendor libraries are tracked separately
from owned code and tested through integration where used. A file merely linked
into a test binary does not count as behavior covered.

Deliver orders 1 and 2 as the first software track. Build the board smoke harness
as a separate track, not a prerequisite for application-logic tests. Then add order
3 application/service fixtures and order 4 boundary integrations. Measure function
and branch coverage per module alongside requirement coverage; the first goal is
to expose missing behaviors, not to meet an arbitrary aggregate percentage.

## Standard unit-test checklist for each module

Apply this checklist to each owned module in the coverage inventory. Mark a category
covered, planned or non-applicable with a reason; do not manufacture meaningless
tests simply to fill every category. Test public contracts and observable effects,
including documented internal invariants where resource safety depends on them.

| Category | Required questions and useful cases |
|---|---|
| Initialization | Are defaults valid? What happens before initialization, after failed initialization and after repeated initialization where supported? Are all readable fields initialized? |
| Normal behavior | At least one independently specified example for each meaningful operation and decision outcome; verify outputs and side effects together. |
| Boundaries | Empty, one element, typical, capacity minus one, capacity and one over; just before/at/after every threshold; minimum/maximum supported configuration. |
| Invalid input | Truncation, unsupported enum/opcode, inconsistent length/count, invalid flags, malformed text and invalid numeric input; no unauthorized or unintended state change. |
| Failure behavior | Each external operation fails in turn; errors propagate according to contract; partial work is cleaned up; subsequent valid operations succeed. |
| State transitions | Legal transitions, forbidden transitions, duplicate operations, cancellation, timeout, reset and reuse; stale events from an earlier operation. |
| Ownership | Every acquired resource has exactly one terminal owner/release; no premature release, leaks, double completion or overwrite of queued data. |
| Persistence | Supported old fixtures load; updates preserve unrelated fields; interrupted operations leave a documented recoverable state. |
| Isolation | One instance/client/channel does not corrupt another; each test begins with known state; test order does not change results. |
| Configuration | Relevant role, feature and capacity variants execute the behavior, rather than merely compiling. |

Null pointers, overlapping buffers and invalid object states should be tested only
where the API promises to handle them or an untrusted entry point can produce them.
Do not call undefined C/C++ operations and demand graceful recovery. Document trusted
caller preconditions and test that production callers enforce them. Invalid external
input must be rejected before reaching a helper with stricter preconditions.

### Numeric, memory and representation boundaries

- NUM-001: signed/unsigned comparisons, narrowing lengths, negative intermediate
  values, multiplication before bounds checks, integer rounding and saturation.
  Test time/counter arithmetic around zero and the largest representable values.
- NUM-002: millisecond wraparound and intervals at the supported scheduling horizon;
  distinguish wrapping monotonic counters from adjustable wall-clock timestamps.
  State the maximum valid interval for any wrap-safe comparison algorithm.
- NUM-003: where floating-point input is accepted, test NaN, infinity, negative zero,
  out-of-range values and rounding near valid boundaries. Specify tolerances from
  the physical/protocol requirement; never choose them after seeing a failure.
- MEM-001: exact-size input/output buffers and guard bytes; truncated variable-size
  fields; no out-of-range access. Verify required output termination and capacity
  handling without imposing string rules on binary buffers.
- MEM-002: allocate/free/reuse packet and contact slots with distinct byte patterns;
  ensure stale lengths, metadata, paths or flags cannot affect the next operation.
  Check sensitive-data clearing only where the lifecycle contract requires it.
- REP-001: explicit wire byte order, unaligned input addresses, serialization field
  widths and signed fixed-point conversion. Do not serialize native struct padding.
  Native simulation may use different `long`, pointer and alignment sizes from MCUs;
  retain cross-target builds and selected on-target tests for architecture differences.

Use address/undefined-behavior sanitizers on supported host test targets, with bounds
and guard assertions providing additional evidence. Sanitizer availability varies
by target; record which instrumentation actually ran. Sanitizers do not establish
correct scheduling, physical timing or complete absence of memory defects.

### Failure atomicity and recovery

FAIL-001: snapshot observable state before a malformed or rejected command. Verify
that only documented effects, such as an error counter, changed; contacts, settings,
identity and queued traffic remain consistent. Parsing failures need not preserve
every temporary parser field unless its API promises that behavior.

FAIL-002: inject a failure at each meaningful stage of an operation: allocation,
read/open/write/sync/rename, crypto setup/update/finalization, enqueue and driver
submission/completion. Include short successful reads/writes, not just negative
return codes. Assert cleanup and recovery using the actual production error path.

FAIL-003: after each failure, run a valid operation and verify useful progress. A
correct error response followed by permanent deafness or a leaked pool is a failure.
For multi-step persistence, specify allowed old/new states and recovery behavior;
do not demand atomicity that the actual storage contract does not provide.

FAIL-004: distinguish accepted, queued, transmitted, acknowledged and delivered.
A send-call success is not an application delivery assertion; a driver rejection
must not become a successful-transmit statistic. Verify counters by independent
event accounting at stable checkpoints, not by reading the same counter twice.

### Ordering, lifecycle and concurrency

SEQ-001: exercise timeout/completion/cancellation at the same logical deadline,
completion after reset, duplicate completion, disconnect with pending output, and
reconnect with stale buffered input. Check terminal outcomes and resource ownership.

SEQ-002: run bounded deterministic sequences against real production state machines:
receive → enqueue → busy → retry → completion; enqueue → reset → late completion;
receive → overflow → drain → receive. Maintain a small independent state model and
compare externally observable results after every step.

SEQ-003: for code that actually runs across threads/ISRs, use controlled barriers
and event injection in kernel integration tests to force relevant interleavings.
Check missed wakeups, publication before initialization, queue draining races and
shutdown with pending work. Test ISR constraints through appropriate driver/kernel
fixtures. A single-threaded fake cannot prove thread safety, and a random stress run
cannot replace explicit interleaving cases.

SEQ-004: multiple instances, contacts and channels with similar identifiers; duplicate
requests; repeated start/stop/reset where supported; repeated errors beyond counter
capacity. Assert isolation and documented idempotency rather than assuming every
operation is idempotent.

## Properties, generated cases and independent oracles

Example-based cases remain the readable specification. Supplement them with bounded,
seeded input generation against the same production code. Save the seed, exact input
or operation sequence and minimized failing case; a seed alone may cease reproducing
after a generator version changes. Keep generator versions in the run metadata.

| Property family | Examples and limits |
|---|---|
| Serialization | Valid representable packets preserve semantic fields through encode/decode; pair this with independent golden bytes to avoid a shared encoder/decoder bug. |
| Parser safety | Arbitrary bounded bytes do not crash, access invalid memory or trigger unintended actions; accepted inputs satisfy the parser's documented invariants. |
| Resource conservation | At stable checkpoints, free + queued + held + in-flight equals pool capacity; no packet belongs to two ownership states. |
| Bounded policy | Jitter, delay, output power, queue depth and path size stay within their specified bounds; test special cases where a cap or override intentionally changes behavior. |
| Stream partitioning | A valid frame produces the same command when split at any byte boundary or coalesced with other frames, provided timeouts are not crossed. |
| Isolation | Adding an unrelated contact/channel or processing unrelated traffic does not alter the operation's result except for explicitly shared capacity or scheduling effects. |
| Persistence | Save/reload preserves supported semantic fields; migrations match independently authored old-version fixtures. |
| Routing truth tables | All Boolean policy combinations and threshold-adjacent hop counts agree with a simple independently reviewed policy table. |

Use exhaustive enumeration for small spaces, table-driven cases for known boundaries,
and bounded generated sequences for large spaces. Quick CI has fixed seeds and small
limits; extended runs rotate recorded seeds and larger limits. Shrinking must retain
the relevant preconditions so the reduced failure still represents a valid finding.

Golden fixtures include provenance, protocol/storage version, expected fields and
why the vector is valid or invalid. Never auto-update expected results in a normal
test run. Differential tests may compare a pinned reference implementation, but
document intentional differences and use independent vectors: agreement alone is
not proof that either implementation is correct.

## Configuration and cross-feature tests

CONFIG-001: run logical capacity tests at the smallest supported setting and a
representative production setting; zero is included only when supported. Ensure
compile-time limits and runtime bounds agree for contacts, channels, queues and pools.

CONFIG-002: test feature-off paths, unavailable optional devices and role differences.
An unsupported request gets its specified response without accessing disabled
services. Build-only checks and executed behavioral checks have separate statuses.

CONFIG-003: include these high-value interactions in native integration fixtures:

- Clear statistics while TX is pending; subsequent completion is accounted for
  according to the reset contract without corrupting control state.
- Save preferences during a temporary radio override; stored and active settings
  retain their intended meanings, including restore after reboot.
- Remove a contact/channel while related work is queued; no dangling reference or
  unintended delivery, with cancellation behavior explicitly specified.
- Adjust wall time while retries, rate limits and maintenance are scheduled.
- Disconnect a companion while lossless responses and lossy pushes compete for space.
- Exhaust a shared packet pool while a recovery/admin operation needs resources.

Start with explicit risk-driven combinations. A versioned pairwise matrix broadens
coverage but does not replace known three-way or higher-order interactions.

## Test reliability and harness self-tests

These are tests of the regression package, distinct from firmware tests. Use tiny
dummy executables, recorded transport streams and fake discovery/flashing services
so harness failures can themselves be reproduced without attached boards.

| ID | Injected harness condition | Required outcome |
|---|---|---|
| HARNESS-001 | Empty collection, unmatched case selector, missing mandatory implementation | Nonzero incomplete/error result; never a successful zero-test run |
| HARNESS-002 | Child failure, crash, hang or malformed/truncated report | Nonzero result, bounded timeout, retained stdout/stderr and failure phase |
| HARNESS-003 | Wrong firmware hash/board, duplicate device identity or absent capability | Reject assignment before dependent tests; record exact mismatch |
| HARNESS-004 | Old success text in a log; packet from a prior session | Ignore as evidence for the current run using session/correlation IDs |
| HARNESS-005 | Port disappears/re-enumerates; probe/port already locked | Bounded recovery or infrastructure error; no accidental reassignment |
| HARNESS-006 | Setup or teardown fails; runner interrupted | Original failure retained, cleanup status reported, locks released where possible |
| HARNESS-007 | Full output disk, invalid profile or conflicting artifact paths | Explicit failure; no overwritten baseline or incomplete report presented as complete |
| HARNESS-008 | Duplicate case IDs, stale catalog mapping or missing expected case results | Inventory/report validation fails; every selected case accounted for |
| HARNESS-009 | Diagnostic retry succeeds after initial failure | Both attempts preserved; original qualification result not silently changed |

Fixtures reset mutable globals, fake time/RNG, queues and filesystem contents between
tests. Run selected suites in reversed/shuffled order with a recorded seed to expose
dependencies; repeat a case in the same process to expose leaked state. Ordinary unit
tests must not depend on Internet access, wall-clock sleeps, local timezone, personal
files or connected devices. Bound loops, generated inputs, waits and subprocesses.

Fakes should fail on unexpected external calls, record inputs and allow explicit
failure scripting. Keep fake service contracts small and test them against the real
boundary where practical. Avoid asserting incidental call order unless the order is
part of correctness. Prefer value/result assertions over large opaque snapshots.

Measure assertions as well as coverage: selectively mutate boundary comparisons,
remove a validation check, suppress a required wakeup or alter a routing decision in
an isolated checkout and verify the relevant tests fail. Surviving mutations require
review; equivalent mutations and unreachable configurations need an explanation,
not artificial tests. Run targeted mutation checks periodically rather than making
every edit wait for the full mutation workload.

Flaky cases remain visible with their first failure evidence. Temporary quarantine
requires an owner, issue and expiry and reduces the reported qualified scope; it
does not turn a mandatory failing case into a green release gate.

## Proposed package and command contract

```text
tests/
  README.md
  run.py                         # one entry point; delegates to existing tools
  requirements.txt               # pinned test dependencies
  catalog.yaml                   # stable case IDs, requirements and implementation mapping
  profiles/                      # explicit case selections and acceptance limits
  lab.example.yaml               # identities/capabilities, no personal keys
  unit/                          # production sources linked into small test targets
  integration/                   # Zephyr native_sim applications and fakes
  hil/                           # pytest test cases
  harness/                       # discovery, transport, flashing, capture, reports
  fixtures/                      # golden packets, corrupt inputs, saved-state versions
  fuzz/                          # parser and state-machine fuzz targets/corpora
  regressions/                   # minimized reproductions of specific fixed bugs
  harness_tests/                 # runner/report/discovery failures without boards
```

Do not put this in root `tools/`: that directory is ignored as part of the west
workspace. Keep personal lab maps and generated results ignored separately.

Proposed interface, to implement in milestone 1:

```text
python tests/run.py list --profile quick
python tests/run.py run --profile quick
python tests/run.py doctor --lab tests/lab.local.yaml
python tests/run.py run --profile bench-smoke --lab tests/lab.local.yaml --firmware artifacts/build.json
python tests/run.py run --profile bench-full --lab tests/lab.local.yaml --firmware artifacts/build.json
python tests/run.py run --profile soak --lab tests/lab.local.yaml --duration 8h
python tests/run.py run --case RADIO-004 --lab tests/lab.local.yaml --seed 12345
```

`list` expands the exact cases and parameters without side effects. `doctor` checks
device identities, probes, transports, firmware capabilities and required equipment.
`run` owns setup, execution, teardown and reporting. Planned/unimplemented cases
must never count as executed passes. A requested profile missing mandatory cases
or equipment exits nonzero and reports incomplete coverage.

## Repeatability contract

Every implemented case has: stable ID, requirement, layer, applicable roles/chips,
fixture version, setup, exact stimulus, independent oracle, deadline, cleanup,
required observability, and implementation path. Preserve IDs when refactoring.

Each run records:

- Git commit plus dirty diff/hash; west manifest and module revisions; patch hashes.
- Toolchain/test dependency versions; firmware SHA-256; resolved `.config` and DTS;
  build command, board, role, bootloader layout and test-hook status.
- Physical device identities and radio firmware revisions where queryable; actual
  control/log transport; RF preset, power, antenna/attenuation arrangement.
- Case/catalog/profile versions, seed, selected parameters, monotonic event times,
  boot/session IDs and explicit packet correlation identifiers.
- Raw serial/BLE/RTT logs, transmitted/received bytes, counter snapshots, reset
  reasons, JUnit XML, JSON measurements and a short human-readable summary.

Statuses: PASS, FAIL, ERROR (infrastructure), SKIP (declared non-applicable),
BLOCKED (missing prerequisite), and NOT_IMPLEMENTED. Report selected, executed,
passed, failed and uncovered cases separately. A required SKIP is not a green gate.
An optional expected failure needs a linked issue and expiry; an unexpected pass
must prompt review. Never silently retry failures until green. Diagnostic retries
retain the first result and all attempts.

Pin baseline firmware and retain its artifacts. Use identical cases, fixtures and
RF arrangements for comparisons. Test candidate-to-candidate, candidate-to-baseline
and baseline-to-candidate; two candidates can share a bug and still communicate.
Freeze acceptance limits before a run; changing a limit is a reviewed test change.

## Four-board laboratory

Starting assumptions, to verify against the actual connected hardware:

| Logical ID | Board target | MCU / radio | Purpose |
|---|---|---|---|
| wio | `wio_tracker_l1` | nRF52840 / SX1262 | Established SX1262 peer |
| t1000 | `t1000_e` | nRF52840 / LR1110 | Cross-radio peer |
| nrf54 | `xiao_nrf54l15/nrf54l15/cpuapp` | nRF54L15 / external Wio SX1262 | Different MCU/timing |
| esp32 | `xiao_esp32s3/esp32s3/procpu` | ESP32-S3 / SX1262 assumed | Different scheduler/USB/BLE |

The exact Wio variant and ESP32 radio attachment must be captured in the lab map.
Use USB serial numbers/probe IDs/BLE identity, not hardcoded COM numbers. Re-resolve
ports after reboot and bootloader entry. nRF54 has no native USB: describe its SWD
probe and UART/RTT/BLE control path explicitly. A USB cable is not proof of an
available companion-protocol endpoint.

Each lab entry declares flash method, recovery procedure, available transports,
power-control capability, supported presets/features, and stored-state policy.
Acquire exclusive device locks. Restore settings in `finally` cleanup, verify them,
and record recovery instructions if interrupted. Use dedicated test identities and
channels; never log personal private keys or reset personal data as implicit setup.

Exercise all 12 directed links between four devices; keep other nodes passive so
forwarding cannot masquerade as direct RX. Rotate sender, receiver and relay roles.
The four-board rig does not physically cover SX127x, LR2021, STM32WL or MG24.
Their build/software checks remain useful, but hardware coverage must say unavailable.

Use a fixed RF arrangement and low suitable power. High packet delivery on a desk
does not measure sensitivity, calibrated output power, or range. Those require
attenuation/shielding and appropriate RF equipment. Do not directly connect radio
ports without a designed attenuation/power budget. Keep test traffic within the
lab's configured permitted band and airtime budget.

## Radio RX/TX case families

All cases below start as planned. Each family expands into individually named,
parameterized cases; HIL and simulated cases have separate results.

| ID | Stimulus | Required observations / assertions | Layer |
|---|---|---|---|
| RADIO-001 | Boot, idle, first RX and first TX | Correct preset; receiver armed; no unexplained reboot | HIL |
| RADIO-002 | All 12 directed links, fixed numbered payloads | Byte-exact receive; missing/duplicate sequences; latency; counter deltas | HIL |
| RADIO-003 | Raw lengths 1, 2, boundary lengths through 255; invalid sizes through API seam | Supported raw frames intact; invalid lengths rejected without buffer damage | Both |
| RADIO-004 | Repeated RX → TX → RX; immediate inbound packet after TX | Exactly one TX completion; RX rearmed within a specified preset-dependent bound | Both |
| RADIO-005 | Back-to-back RX and bursts around ring capacity | Defined drop policy; overflow accounting; later packet succeeds; no stale metadata | Both |
| RADIO-006 | Async send failure, missing/late/duplicate completion, timeout | Bounded recovery; no double free/completion; subsequent TX/RX works | Integration, HIL subset |
| RADIO-007 | Preamble without payload, CRC/header error, aborted reception | Busy state clears; no phantom valid payload; next valid RX succeeds | Integration, RF fixture subset |
| RADIO-008 | CAD clear/busy, repeated driver LBT refusals, persistent busy | Correct deferral, independent streak timing, bounded escalation, no false TX count | Both |
| RADIO-009 | Supported frequency/BW/SF/CR/preamble/power combinations | Readback where available, peer interoperability, airtime within defined tolerance | Both |
| RADIO-010 | Change preset while idle/busy; temporary override and restore | Active/saved settings correct; no stale completion applied to new session | Both |
| RADIO-011 | SX126x RX duty cycle on/off across supported presets | Reception and recovery; unsupported radios reject/report unsupported | Both |
| RADIO-012 | Noise/CAD adaptation, starvation recovery, idle maintenance | Bounds, freeze/restore, deadlines and hysteresis; no permanent deaf state | Both |
| RADIO-013 | Reconfigure/send/receive/maintenance event order permutations | No deadlock, missed wake, use-after-free or stuck TX/RX state | Integration |
| RADIO-014 | Long simultaneous bidirectional load | Delivery distribution, queue high-water, memory trend and bounded recovery | HIL |
| RADIO-015 | Reset/disconnect one node during traffic | Remaining nodes live; reconnect/rejoin works; stale events ignored | HIL |
| RADIO-016 | Driver BUSY/SPI error/IRQ timeout injection | Error propagation, bounded recovery and counter attribution | Integration; fixture-dependent HIL |

Separate raw radio diagnostics from application packet tests: a 255-byte radio
frame is not necessarily a valid MeshCore packet. Invalid physical CRC/header tests
need driver injection or a capable RF generator; a normal companion send command
cannot be assumed to generate them.

Raw diagnostics should link the real adapter and patched driver in a small test
application. Also test ordinary production firmware end to end, since a diagnostic
image bypasses mesh scheduling and transport behavior.

Start with one fixed preset on every directed pair. Extend with a versioned pairwise
configuration matrix, explicit minimum/maximum supported settings, and mandatory
chip-specific interactions. Exhaustive combinations are appropriate for small
software inputs; avoid an unbounded Cartesian product of every HIL dimension.

## Packet and protocol coverage

For every type below, specify behavior per role and route: decode, dispatch, forward,
respond, or intentionally ignore. Presence of a constant does not imply that every
role implements it. Test parser acceptance separately from semantic acceptance.

| Value | Type | Semantic cases |
|---|---|---|
| 0x00 | REQ | Authorized/unauthorized request, destination mismatch, malformed body |
| 0x01 | RESPONSE | Matching, unsolicited, stale and malformed response |
| 0x02 | TXT_MSG | Direct/flood messaging, encryption, ACK/retry, duplicate delivery |
| 0x03 | ACK | Match/mismatch, short body, late/duplicate ACK, queue ownership |
| 0x04 | ADVERT | Valid/invalid signature, name/location boundaries, contact/path updates |
| 0x05 | GRP_TXT | Correct/wrong channel key, UTF-8 and message size boundaries |
| 0x06 | GRP_DATA | Binary data, maximum length, unknown channel, malformed body |
| 0x07 | ANON_REQ | Valid/invalid crypto, truncated identity/body, access control |
| 0x08 | PATH | Hash widths, empty/full paths, destination mismatch, path updates |
| 0x09 | TRACE | Hop processing, growing response, bounds and special dedup hash |
| 0x0A | MULTIPART | Implemented subtype behavior including ACK; unsupported subtype handling |
| 0x0B | CONTROL | Supported subtypes, flags, short bodies and unsupported commands |
| 0x0C–0x0E | Reserved | Explicit unsupported behavior; no unintended side effects |
| 0x0F | RAW_CUSTOM | Delivery/forward policy, empty/max payload, unsupported consumer |

PACKET-001: enumerate all 256 header bytes and all 256 encoded path-length bytes in
parser tests. Cover all four routes and version-bit combinations, documenting the
current protocol's allowed/unsupported behavior rather than inventing restrictions.

PACKET-002: lengths 0/1, minimal headers, truncated transport codes, absent payload,
path truncation at every byte, payload 1/183/184/185 and total unit boundary.
Current constants: payload 184, path buffer 64, transmission unit 255. Boundary
expectations are independently specified; flag constant changes for review.

PACKET-003: hash widths 1/2/3, reserved width 4, zero hops, maximum valid count for
each width, one-over-limit and source buffers one byte too short. Verify zero-hop
`0x40`/`0x80` path encodings survive copies. Test helper and untrusted-call-site
contracts separately: a serializer's internal trusted-input contract is not a parser.

PACKET-004: independently reviewed golden wire bytes and expected decoded fields,
including transport-code byte order. Round trips supplement golden vectors; they
cannot catch encoder and decoder sharing the same mistake.

PACKET-005: golden packet hashes; ordinary path changes versus TRACE path-length
changes; payload/type changes; dedup collisions and expiry. Do not infer integrity
from the truncated dedup hash.

PACKET-006: fuzz parser, advertisement, path, companion framing and CLI entry points
with sanitizers and bounded resources. Fixed seeds/corpora in quick CI; time-bounded
extended fuzzing nightly. Save minimized crashes as permanent deterministic tests.
Do not pass values outside a C++ API's representable input type and claim coverage.

## Scheduling, routing and mesh behavior

- MESH-001: real packet pool exhaustion, enqueue ordering, priority and due-time
  ties, rescheduling, manual hold/release and every error-path ownership transfer.
  Free + queued + in-flight + held counts must reconcile at quiescent checkpoints.
- MESH-002: fake clock/RNG through Dispatcher; timer expiry one tick before/at/after
  deadlines, 32-bit wrap, airtime budget refill/debit/cap, failed-send accounting,
  RX accounting, stats reset while active and maintenance-only wakeups.
- MESH-003: direct, flood and transport routes; matching/nonmatching next hop;
  append/consume paths; loop prevention; duplicate suppression and cache eviction;
  region filters, transport keys and admin exceptions.
- MESH-004: ContentionTracker warmup/EMA/decay, ring eviction, duplicate-window
  boundaries, reactive backoff caps, multiplier limits and timer wrap.
- MESH-005: ACK loss, delayed/duplicate/out-of-order ACK, retry exhaustion, path
  discovery/fallback, no duplicate user-visible delivery where required by protocol.
- MESH-006: A → relay → B, two relays, competing relays, relay loss/rejoin and
  asymmetric links. Model topology deterministically in software first.
- MESH-007: role-specific forwarding and delivery; observer never transmits LoRa;
  room login/post/fanout/history, offline clients and slow clients; repeater ACL,
  rate limiting, neighbors and region configuration.

Four nearby boards do not enforce a multi-hop RF topology. Prove forwarding with
path/capture evidence and, when testing lack of a direct link, use RF isolation or
attenuation. Simulated topology results must not be labeled physical range tests.

## Remaining subsystem coverage

| Family | Required cases |
|---|---|
| CRYPTO | Known-answer signatures, verification, key agreement, encryption/MAC; tamper/wrong key/truncation; accelerated vs software results; RNG error propagation and boot uniqueness smoke (not proof of entropy quality) |
| TRANSPORT | USB/UART/TCP split and coalesced frames, endian length, oversize/zero frames, garbage resync, frame timeout, disconnect midway, reconnect, saturation and response/push loss policy |
| BLE | Real GATT exchange, pairing/bonding, MTU fragmentation, notification backpressure, reconnect, bond removal, concurrent LoRa load; requires host BLE adapter |
| COMMAND | Inventory every companion opcode and CLI command; valid/min/max/truncated/unknown input, authorization, response bytes, persistence and side effects; fail catalog audit on newly uncategorized command |
| STORAGE | Identity/prefs/contacts/channels/ACL/keys; save/reboot/load, migration fixtures, corrupt/truncated files, full storage, failure during write and recovery without silent identity loss |
| TIME | RTC unset, forward/backward changes, time sync, expired/future timestamps, reboot, monotonic scheduling independent of wall-clock jumps |
| POWER | Sleep/wake with pending traffic, RX duty operation, GPS power gating, USB/BLE interactions; current measurement only with instrumented equipment |
| GPS/SENSORS | No device/no fix, partial/malformed data, timeout/recovery, valid position, battery conversion boundaries and sensor errors; fixture injection plus real-device smoke |
| UI | Buttons/joystick, menu actions, display sleep/wake, buzzer/haptic gates, long/invalid UTF-8; manual visual checks explicitly reported |
| NETWORK | Observer MQTT reconnect/backpressure, WiFi loss, uplink loops, credentials persistence, no inadvertent LoRa transmit |
| OTA/BOOT | Release image boot, board/partition compatibility, update/reboot/state preservation, interrupted/rejected update and recovery according to supported bootloader behavior |
| BUILD | All supported boards/roles; feature configurations, patched-driver presence, memory budgets, signed/merged/UF2/DFU artifacts as applicable; inspect actual release outputs |

For each family record unsupported features as non-applicable, not passing tests.
Hardware power loss requires a controllable supply/USB relay; soft reboot does not
prove power-failure safety. Test destructive storage/OTA scenarios only on designated
test devices with known recovery paths.

## Observability and fault injection

First use existing production APIs, CLI, counters and captures. Add only the seams
needed to make otherwise invisible transitions testable. Prefer structured test
events over assertions that depend on human-readable debug text.

Proposed test-only build option (name to be chosen during implementation): expose
capabilities, boot ID, TX accepted/completed/failed, RX accepted/dropped, queue/pool
counts, active preset and driver state. Fault injection can suppress a completion,
return a driver error or alter an event order. It must be off in release configs
and unreachable through production remote commands. CI checks that exclusion.

Compile real production algorithms; fakes replace external inputs, not the logic
being tested. Use fake time to advance hours/wraparound instantly, and deterministic
RNG for repeatable jitter. For kernel behavior, run real Zephyr timers/queues in
native integration tests. Include production builds in HIL to catch instrumentation
changing timing, USB routing, logging volume or memory pressure.

## Concrete smoke acceptance

Version 1 smoke, after implementation:

1. Preflight all four identities and firmware hashes; reserve devices; save settings;
   apply a fixed test configuration and capture fresh boot/session markers.
2. Run all 12 directed links, initially 20 uniquely identified valid frames per link
   at one fixed preset, with other nodes passive. Record bytes and RSSI/SNR per frame.
3. Require all expected frames byte-exact and exactly once in the isolated raw-link
   case. A missing frame fails this strict smoke criterion; use control runs to
   diagnose interference rather than silently weakening the result. This small
   sample is a connectivity gate, not a high-reliability statistical claim.
4. Exercise representative direct/channel messages and advertisements; verify
   application receipt and expected ACK behavior, not just sender API success.
5. Run at least 100 TX/RX reversals per board within the airtime budget. Verify RX
   returns, queues drain, and no unexplained reset or leaked packet remains.
6. Reboot each board in turn and verify it rejoins and completes another exchange.
7. Restore settings, release ports/locks and emit report even on failure.

Longer RF profiles specify sample count, offered load, loss limit, latency bound and
confidence method before execution. Report packet delivery with a binomial confidence
interval and latency percentiles; compare like-for-like baseline sessions and include
a control link. Do not set a universal RSSI value or a universal millisecond deadline:
radio airtime varies with modem configuration. Calibration data and reviewed limits
belong to the lab/profile version. Infrastructure failures leave qualification incomplete.

## Implementation milestones and completion gates

### 1. Establish the permanent software foundation

- Add catalog/profile/report schemas and runner `list`/`quick`; no device dependency.
- Link Packet, real pool, ContentionTracker and Dispatcher into tests with fake
  clock/radio/RNG. Resolve minimal logging/crypto dependencies without broad refactors.
- Implement PACKET-001–005 and MESH-001/002/004 high-risk boundaries.
- Establish the source coverage inventory above; include advertisement/UTF-8,
  routing policy, rate limiting and identity/crypto vectors in the initial software
  track. Continue with Mesh, BaseChatMesh, ACL, region/key and time-sync fixtures
  without waiting for hardware-runner completion.
- Add golden fixtures and the zero-hop path and LBT streak regressions already
  described in source comments. Demonstrate each targeted regression fails when
  its known defect is deliberately reintroduced in a disposable worktree.
- Add PR CI, sanitizer runs where supported, JUnit/artifact output and coverage.
- Apply the per-module checklist and implement HARNESS-001/002/008 first. Add
  bounded property tests for packet inputs and pool ownership, plus fixture isolation
  checks. Expand harness self-tests as discovery/flashing services are introduced.

Done: a documented command from a clean checkout runs actual production code,
reports failures with stable IDs, requires no boards and cannot pass an empty suite.

### 2. Make the four-board smoke repeatable

- Implement lab map/discovery/locking, transport adapters, firmware manifest checks,
  role-specific build/flash recipes and guaranteed report/cleanup paths.
- Build minimal raw radio test application plus production-firmware packet scenarios.
- Implement RADIO-001–004/015 and the smoke procedure above.
- Prove recovery after an intentionally unplugged board and an interrupted run.
- Respect nRF54 `--no-sysbuild` and actual ESP32 role/release boot layouts; derive
  flash recipes from resolved artifacts instead of one generic flash command.

Done: reconnect the same boards next week and run the same cases without rewriting
scripts; failures have raw evidence and a one-case reproduction command.

### 3. Expand deterministic failure and protocol coverage

- Complete packet-type/role matrix, fake network routing, transport/opcode tests,
  radio-base kernel integration and driver failure seams.
- Add parser fuzzing, saved-state fixtures and permanent cases for historical fixes.
- Extend HIL presets, payloads, CAD/LBT, overflow, duty cycle and role rotations.

Done: every packet type and command is accounted for; unsupported, unimplemented
and unobservable cases remain visible, with owners and prerequisites.

### 4. Qualify releases and long-run behavior

- Add reference Arduino MeshCore fixtures/images pinned to a specific revision;
  both directions for supported wire behavior, keeping ZephCore-only behavior separate.
- Run candidate/baseline mixes and production release binaries, persistence/OTA,
  BLE/network scenarios and soak profiles.
- Add missing radio boards or instrumented fixtures as coverage needs justify them.

Done: release report names its precise supported board/role/feature coverage,
has no missing mandatory cases and retains comparable baseline artifacts.

## Ongoing change policy

Every bug fix adds a minimized permanent reproduction at the lowest useful layer;
driver/timing bugs also get HIL coverage when physically reproducible. New packet
types, commands and features must update the catalog. Do not regenerate expected
golden bytes from the implementation under test during ordinary runs.

Run quick on every change, integration/build checks on PRs, hardware smoke after
radio/driver/transport changes, full bench before release, and scheduled fuzz/soak
when a dedicated rig is available. Change-based selection supplements a fixed core
suite; a shared driver patch must not evade checks because only application paths
were mapped. Serialize bench jobs and isolate west trees used by builds that patch
dependencies at configure time.

Track branch coverage for owned code, covered requirements, supported configurations,
fault scenarios and hardware execution separately. Initially measure coverage rather
than inventing an overall percentage target; then ratchet thresholds for modules
with meaningful tests. Review missing error branches and use targeted mutation checks
to establish that assertions detect the failures they claim to cover.

## Framework references

- [Zephyr Ztest](https://docs.zephyrproject.org/latest/develop/test/ztest.html)
- [Zephyr Twister](https://docs.zephyrproject.org/latest/develop/twister/index.html)
- [Zephyr pytest harness](https://docs.zephyrproject.org/latest/develop/twister/harness/pytest.html)

These explain the recommended framework capabilities; the repository's pinned
Zephyr source remains authoritative for the implementation's available APIs.
