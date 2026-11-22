# ZephCore — Zephyr Standards Alignment Audit

Date: 2026-10-16
Scope: `zephcore/` application tree plus root `build.sh`, `tests/`, `docs/`, `.github/`.
Upstream `zephyr/` and `modules/` checkouts were excluded (they are dependencies, not project code).
Method: read-only source inspection; every finding cites file evidence.

Overall assessment: ZephCore is a mature, well-documented application with strong release
engineering, but it deliberately diverges from standard Zephyr project structure in several
load-bearing places — configure-time patching of the upstream tree, conf-filename-driven role
selection, boards that are not Zephyr boards, and no Twister/HIL testing. Most divergences are
documented decisions (ADRs, AGENTS.md invariants), not accidents, but they carry real
maintenance cost.

Alignment score per area (0 = fully divergent, 10 = fully aligned):

| # | Area | Score | Severity of worst finding |
|---|------|:-----:|---------------------------|
| 1 | Code organization | 5 | High |
| 2 | API compliance | 7 | Medium |
| 3 | Build system | 3 | High |
| 4 | Kconfig integration | 6 | High |
| 5 | Devicetree integration | 7 | High (via build system) |
| 6 | Driver architecture | 4 | High |
| 7 | Dependency management | 8 | Medium |
| 8 | Coding conventions | 7 | Low |
| 9 | Testing infrastructure | 4 | High |
| 10 | CI integration | 5 | Medium-high |
| 11 | Documentation | 9 | Low |
| 12 | Error handling & diagnostics | 7 | Low |
| 13 | Version compatibility | 6 | Medium |
| 14 | Portability | 5 | Medium |
| 15 | Lifecycle management | 6 | Medium |

---

## 1. Code organization — score 5/10

Misalignments:

- **Boards are not real Zephyr boards (HIGH).** `zephcore/boards/esp32/heltec_wifi_lora32_v3/`
  contains only `board.conf`, `board.overlay`, `partitions.overlay`, `zephcore.yml` — no
  `board.yaml`, `Kconfig.board`, `Kconfig.defconfig`, or `<board>_defconfig`. These fragments are
  layered onto upstream base boards by custom CMake (`cmake/zephcore_board.cmake:20-42` globs
  `boards/*/<board>` and name-matches against the upstream `BOARD`). Hardware description that
  belongs in defconfig/Kconfig.defconfig lives in name-matched conf fragments.
- **Board metadata duplicated in a private manifest (MEDIUM).** `zephcore.yml` (target,
  capabilities, release, catalog) overlaps `board.yaml`; `zephcore_board.cmake:86-110` parses YAML
  with `file(STRINGS ... REGEX)` because Python is unavailable pre-Zephyr-init.
- **Application/driver/library code mixed across five parallel trees** (`src/`, `app/`,
  `helpers/`, `adapters/`, `lib/`) with 13 hand-maintained `target_include_directories` entries
  (`CMakeLists.txt:338-353`) instead of per-module `zephyr_library()` or a `zephyr/module.yml`.
  Everything links into the monolithic `app` target.
- **Porting-history layout (LOW).** `app/main_observer.cpp` sits in `app/` while
  `src/main_repeater.cpp` sits in `src/`; Arduino-port shims (`helpers/compat/SHA256.cpp`,
  `helpers/compat/ZephyrFS.cpp`) are first-class sources.

Done well: clean adapter-per-peripheral split (`adapters/radio|gps|ble|wifi|mqtt|sensors|usb|
datastore`); explicit source lists with per-line rationale; role sources gated on Kconfig;
vendored crypto isolated in `lib/monocypher`.

## 2. API compliance — score 7/10

Misalignments:

- **Raw register access (MEDIUM).** `helpers/power_diag.c:309+` reads raw nRF52 peripheral
  addresses (`0x50000514`, `0x40001FFC`) via a `REG()` macro. Gated on
  `CONFIG_ZEPHCORE_POWER_DIAG` + `CONFIG_SOC_NRF52840` and opt-in, but should use `sys_read32()`
  with DT-derived base addresses.
- **Vendor HAL/ROM calls (LOW-MEDIUM).** `adapters/board/ZephyrBoard.cpp:491`
  (`RTC_CNTL_FORCE_DOWNLOAD_BOOT`), `helpers/pm_esp32_console.c:59` (`esp_rom_output_tx_wait_idle`).
  No Zephyr equivalents exist; acceptable per AGENTS.md rule 8, already isolated in ESP32-only
  files.
- **`#include <soc.h>` (LOW)** in `include/mesh/stm32_cmsis_fixup.h:32` — semi-internal header in
  an STM32 shim.
- **11 carried patches on upstream Zephyr (MEDIUM).** `zephcore/patches/zephyr/` (0001–0019):
  new lr11xx/lr20xx drivers, full replacement of the sx126x driver, sx127x 62.5 kHz BW, GNSS
  query, esp32 WiFi RX fix (0017), cdc_acm poll-out flush (0019), etc. Each is well-motivated
  with proper commit bodies and "one patch owns each Zephyr file" is enforced — but 0017, 0019,
  0004, 0009 read like upstreamable commits and their continued carrying is drift risk.

Done well: zero includes of internal Zephyr paths; all subsystem usage through public headers;
devicetree used for hardware description throughout.

## 3. Build system — score 3/10

The most divergent area.

- **CMake auto-patches and mutates the Zephyr/module trees at configure time (HIGH).**
  `cmake/zephyr_patches.cmake` + `cmake/zephyr_new_files.cmake` run `git apply` and copy files
  into `zephyr/`, `modules/loramac-node`, `modules/hal_espressif`. Non-hermetic; breaks on
  read-only checkouts; requires patches to land before MCUboot's devicetree pass.
  Remediation: forked module in `west.yml` or a proper Zephyr module with `zephyr/module.yml`.
- **Hardcoded workspace layout (HIGH).** `CMakeLists.txt:16-17` resolves
  `${CMAKE_CURRENT_SOURCE_DIR}/../zephyr` and `../modules` — assumes this exact west layout.
  Convention is `$ENV{ZEPHYR_BASE}` / west resolution.
- **Reimplements Zephyr's config layering (HIGH).** `CMakeLists.txt:71-316` manually parses
  `EXTRA_CONF_FILE` names, scrapes `SYSBUILD_CACHE` for BOARD, infers "role" and "debug" from
  conf *file names*, auto-appends conf/overlay chains, generates `zephcore_version.conf`, then
  FORCEs cache vars and FATAL_ERRORs on disagreement. Duplicates what Kconfig defaults,
  `depends on`, `FILE_SUFFIX` and defconfigs exist for; role selection keyed on filenames is
  fragile.
- **Overlay auto-pairing by filename (MEDIUM).** `zephcore_add_conf` (`CMakeLists.txt:54-64`)
  silently pairs `foo.conf`→`foo.overlay`; a missed pairing once dropped the SX1262 node from a
  build.
- **Compiles a source out of hal_espressif by absolute path (MEDIUM).**
  `CMakeLists.txt:467-479` targets
  `${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/.../bootloader_random_${CONFIG_SOC_SERIES}.c`, bypassing
  the module's build system.
- **DFU packaging in app CMake (LOW).** `CMakeLists.txt:789-808` `find_program(adafruit-nrfutil)`
  + `dfu_zip` target; belongs in scripts.

Done well: `build.sh` keeps release firmware identical to a local build (no release-only `-D`
flags); `board_manifest.py` schema + validator; `sysbuild/CMakeLists.txt` correctly shares
`partitions.overlay` between app and MCUboot with the contract documented.

## 4. Kconfig integration — score 6/10

Misalignments:

- **Role selection via conf fragments + CMake filename sniffing instead of Kconfig (HIGH).**
  `choice ZEPHCORE_ROLE` exists (`Kconfig:603`) but per-board defaults can't be set because the
  boards have no `Kconfig.defconfig`.
- **Some inverted `select` dependencies (MEDIUM).** `Kconfig:622-623`
  `ZEPHCORE_RADIO_LR1110` selects LORA and SPI (should be `depends on`); `Kconfig:854-858` a debug
  symbol selects six kernel introspection options. Overall `select` usage is restrained (18
  occurrences).
- **Overriding upstream defaults by ordering tricks (MEDIUM).** `Kconfig:5-53` redefines
  `choice ESP_WIFI_HEAP`, `MAIN_STACK_SIZE`, `GNSS_LUATOS_AIR530Z_NAV_MODE` defaults ahead of
  `Kconfig.zephyr` — brittle against upstream Kconfig reordering.
- **Duplication across the fragment chain (MEDIUM).** 5+ platform fragments
  (`boards/common/*_common.conf`, 88-line `zephcore_common.conf`, role/debug confs) keep heap and
  stack sizes in sync manually; several settings exist both as Kconfig defaults and in conf
  files.
- **Compile-time default credentials (LOW).** `ZEPHCORE_ADMIN_PASSWORD default "password"`,
  `ZEPHCORE_BLE_PASSKEY default 123456` — documented, but flag-worthy.

Done well: extensive help texts with measured rationale on every option; `range`/`depends on`
used properly; DT-based defaulting via `$(dt_chosen_enabled,...)`; consistent `ZEPHCORE_`
namespacing.

## 5. Devicetree integration — score 7/10

Misalignments:

- **Board overlays depend on patched upstream bindings (HIGH, consequence of area 3).** Overlays
  use `lna-bypass-gpios`, which exists only after patch 0003 modifies upstream
  `semtech,sx126x-base.yaml`; fresh trees fail MCUboot's DTS pass (documented in
  `sysbuild/CMakeLists.txt`). Binding changes should live in local `dts/bindings/` or a forked
  module.
- **Some custom bindings bypass the driver model (MEDIUM).** `zephcore,rtc-i2c` drives a raw-I2C
  RTC auto-discovery instead of a `zephyr,rtc`-API driver; `zephcore,poweroff-gpios` and
  `zephcore,input-multi-tap` are board glue consumed by app code, not drivers; `zephcore,mono-tft`
  partially reimplements upstream display bindings.
- **Board-specific binding with vendor prefix (LOW).** `seeed,t1000e-analog` should be a generic
  ADC-based pattern.

Done well: all pins/GPIOs uniformly in overlays, no hardcoded IRQ/GPIO numbers in C code;
flash-partition `reg` values are the legitimate kind; `/delete-node/` and chosen nodes used
properly; local bindings namespaced `zephcore,*`; partition overlay shared with MCUboot under a
documented contract.

## 6. Driver architecture — score 4/10

Misalignments:

- **Almost nothing uses the Zephyr driver model (HIGH).** Zero `DEVICE_DT_DEFINE`/`DEVICE_DEFINE`
  in project code; only ~10 `SYS_INIT` registrations for app glue. The stc8h keypad, RTC
  discovery, buzzer, oled_power etc. are plain app modules — no PM-device integration, no
  `device_get_binding` discipline, no init-priority dependency expressions.
- **The exception proves the gap.** `adapters/sensors/spa06.c` is a textbook out-of-tree Zephyr
  sensor driver (`DT_DRV_COMPAT`, config/data structs, local binding); the RTC-i2c and keypad
  should follow it.
- **Radio is a hybrid (MEDIUM).** SX126x/SX127x silicon is driven through the real Zephyr LoRa
  API (`lora_config`, `lora_recv_duty_cycle_async` in `LoRaRadio.cpp`) — good. But LR1110/LR2021
  support exists only via build-time patches injecting `drivers/lora/lr11xx/` plus private
  extension headers (`zephyr/drivers/lora/lr11xx_lora.h`), with ZephCore's own `LoRaRadioOps`
  vtable on top. Boards cannot move to a stock upstream tree.
- **Hand-ordered init (LOW-MEDIUM).** Initialization relies on SYS_INIT priority numbers and
  constructor ordering in `main_*.cpp` rather than `DEVICE_DT_GET` + `device_is_ready`.

Done well: radio genuinely consumes the Zephyr LoRa API rather than bit-banging SPI; GPS uses
upstream `gnss-nmea-generic` + GNSS API with PM; USB CDC via `USBD_DEVICE_DEFINE`; entropy via
`sys_csrand_get`.

## 7. Dependency management — score 8/10

One of the strongest areas.

- Zephyr pinned to a full SHA in `zephcore/west.yml:9` (not a branch/tag); `import: true`
  transitively pins all modules.
- `lib/monocypher` vendored with exemplary provenance header (version 4.0.2, source URLs,
  CC0-1.0 OR BSD-2-Clause, exact local modification).

Misalignments:

- **Pin has no human-readable version comment (LOW).** Which Zephyr release the SHA corresponds
  to is not stated.
- **Vendored Semtech lr20xx/lr11xx driver code not in any top-level dependency/licensing doc
  (LOW-MEDIUM)** — only discoverable by reading patch 0001's body. Add a NOTICE/DEPENDENCIES
  entry.
- **Configure-time mutation of the upstream tree (LOW as documented design; see area 3).**

## 8. Coding conventions — score 7/10

Done well: thorough `.editorconfig` matching Zephyr conventions (ported MeshCore files explicitly
exempted with rationale); consistent `LOG_MODULE_REGISTER` (59 files) with Kconfig-namespaced
log levels; SPDX headers; Zephyr-style C in `adapters/`.

Misalignments:

- **Mixed C/C++ conventions (LOW, deliberate).** MeshCore-style camelCase in `app/` vs Zephyr
  snake_case in `adapters/` — accepted debt of byte-compatible porting.
- **printk bypassing the log subsystem (LOW-MEDIUM).** `app/RoomServerMesh.cpp:284`,
  `app/RepeaterMesh.cpp:620` (RAW packet hex dumps), `adapters/rng/ZephyrRNG.cpp:58`. Route
  through `LOG_DBG`/`LOG_HEXDUMP_DBG`.
- **Unbounded `sprintf` into fixed buffers (LOW-MEDIUM, latent overflow).**
  `app/RoomServerMesh.cpp:270`, `app/RepeaterMesh.cpp:514` — switch to `snprintf`.
- **No `.clang-format`** (LOW) — `.editorconfig` alone leaves continuation style to taste.

## 9. Testing infrastructure — score 4/10

- **No Twister integration at all (MEDIUM).** Zero `testcase.yaml`/`sample.yaml` files.
  `tests/run.py` is a fully custom host runner ("No board discovery or flashing"). Twister appears
  only as a future plan in `docs/REGRESSION_TEST_PLAN.md:64`.
- **No on-target/HIL testing; release qualification is manual (HIGH).** bench-smoke/bench-full/
  soak profiles are unimplemented designs; README: "changes are tested on real hardware before
  release (mostly nRF boards on our own bench)". For firmware shipped to ~47 boards this is the
  largest testing gap.
- **Zephyr boundaries faked with shim headers (LOW-MEDIUM, documented).**
  `tests/support/zephyr/{kernel.h,logging/log.h,random/random.h,sys/util.h}`; SHA-256 is an
  input-capture stub. native_sim ztest would replace these with the real kernel.
- **Stale case count in docs (LOW).** tests/README.md says 114 cases; `tests/catalog.json` has
  104.

Done well: the custom runner is unusually rigorous — harness self-tests, catalog cross-validation
(missing/renamed/duplicate cases rejected), per-case success markers, fixed seeds, SHA-256 input
hashing, JUnit/JSON reports, ASan/UBSan, RFC 8032 Ed25519 known-answer vectors against real
Monocypher. Stronger than many Twister suites as a host unit suite; it is just not the standard
mechanism and has no integration or HIL layers.

## 10. CI integration — score 5/10

Misalignments:

- **Firmware build matrix does not run on PRs (MEDIUM-HIGH).** `build.yml` triggers only on push
  to master; AGENTS.md confirms "PR CI runs the host tests only". A PR can break a board/role
  build and only fail after merge.
- **No Twister in CI (MEDIUM)** — `tests.yml` runs only `tests/run.py --profile quick
  --sanitize`.
- **No static analysis or compliance checks (LOW-MEDIUM).** No checkpatch, clang-tidy,
  Kconfig/devicetree-binding lint, yamllint, or DCO check. Only custom validation is
  `board_manifest.py check`.
- **Action version inconsistency (LOW).** `actions/upload-artifact@v4` in tests.yml vs v7 in
  build.yml.

Done well: third-party actions pinned to full SHAs; runner and SDK (1.0.1) pinned with rationale;
modules cache keyed on `hashFiles('zephcore/west.yml')`; per-job ccache; artifact-based handoffs;
release-atomicity concurrency control; standard `zephyrproject-rtos/action-zephyr-setup`; broad
matrix (nRF52, ESP32, nRF54L, MG24, STM32WL, cross-compiled Linux ARMv7/aarch64).

## 11. Documentation — score 9/10

The strongest area. `docs/` contains DESIGN.md, ARCHITECTURE.md, 9 ADRs with an index,
BUILDING.md (setup/variants/flashing/Kconfig reference), full CLI references, a generated
supported-boards matrix (`supported_boards.md` from board manifests), and a regression test plan
that honestly separates implemented from planned. README links everything and includes a
per-platform hardware matrix (22 nRF52840, 16 ESP32, 4 nRF54L, 1 MG24, 1 STM32WL, 3 Linux
presets) and a "For Zephyr people" section.

Misalignments (all LOW): no generated API docs (Doxygen); no CI check that generated docs stay in
sync (`board_manifest.py docs` + `git diff --exit-code` would close this); minor doc drift
(test case count). The apparent `doc/` vs `docs/` duplication is a non-issue — `doc/` is a
git-ignored local checkout of upstream's reqmgmt repo.

## 12. Error handling & diagnostics — score 7/10

Done well: Zephyr negative-errno style throughout native code; no `__ASSERT`/assert() anywhere —
failures are handled, not asserted, appropriate for firmware that must keep meshing; fatal crypto
errors reboot loudly; the 70 `(void)` casts are overwhelmingly intentional unused-parameter
markers in ported callbacks.

Misalignments (all LOW to LOW-MEDIUM): real return values silently discarded in
`helpers/boot_info.c:68`, `src/main_companion.cpp:918`, `helpers/ui-button/ui_task.c:783`
(undocumented); `boot_info_init` returns 0 on hwinfo failure by design (broken driver invisible
except via accessor); printk-based reporting can't be silenced via Kconfig log levels.

## 13. Version compatibility — score 6/10

- Baseline declared in three places: `zephcore/west.yml:9` (full SHA), `.west/config`,
  `docs/adr/0005-zephyr-main-pin.md`. SDK pinned in CI.
- **Pinned to a floating `main` commit, not a release tag (MEDIUM, deliberate).** ADR 0005
  records the decision and the exact API breakage each bump entails (LoRa API 0.9.0,
  uart_irq_update signature, ring-buffer API, nRF LFCLK to devicetree...). Every bump is a manual
  porting exercise. Consider pinning releases to Zephyr release tags, tracking main only on dev.
- **Patch set must be rebased every bump (LOW-MEDIUM).** Mechanism is well engineered (fails
  loudly, idempotent stamps, "one patch owns each file" invariant); the burden is inherent.
  Several patches (0017, 0019, 0004) are upstreamable and upstreaming them shrinks the set.

## 14. Portability — score 5/10

AGENTS.md rule 7 states "Never `CONFIG_SOC_*`, `CONFIG_BOARD_*`... as a stand-in for 'has this
part'". **The policy is broadly violated in practice: 25+ `CONFIG_SOC_*` gates in native
(non-ported) code (MEDIUM).**

- `helpers/CommonCLI.cpp:343-352, 664, 1068-1084` — DFU/powersaving replies gated on
  `CONFIG_SOC_SERIES_ESP32S3` / `NRF52` / `FAMILY_NORDIC_NRF`. A fourth SoC port silently gets
  "not supported". Capability Kconfigs would satisfy the project's own rule.
- `adapters/gps/gps_internal.h:17,45` keys features to `CONFIG_SOC_NRF52840` — a specific part,
  not even a family; exactly the pattern rule 7 forbids.
- Also: `adapters/board/ZephyrBoard.cpp`, `adapters/rng/ZephyrRNG.cpp`, `helpers/boot_info.c`,
  `helpers/pm_sleep_guard.h:46`, `helpers/power_diag.c:20`, `adapters/board/zephyr_poweroff.c:35`.
  Some are legitimately chip-specific (RNG peripherals, ROM download boot — rule 8), but most
  should be capability-gated.
- **Hard memory-layout assumption in common code (MEDIUM).** `src/main_companion.cpp:549`
  `__attribute__((section(".ext_ram.bss.companion_mesh")))` — ESP32/PSRAM layout in a common main
  file; should be gated on the existing `Kconfig.psram` capability.
- **Arch-keyed behavior with a hole (LOW-MEDIUM).** `helpers/fatal_reboot.c:67-70` handles
  ARM/RISCV; Xtensa (ESP32-S3) falls through — verify intent.
- LOW: `stm32_cmsis_fixup.h` in the shared mesh include tree; `__attribute__((weak))` instead of
  `__weak` from `zephyr/toolchain.h`; implicit little-endian assumption in the protocol layer
  (documented byte-compat invariant).

## 15. Lifecycle management — score 6/10

Done well: mature automated release process — version single-sourced in
`zephcore/CMakeLists.txt:36`, becomes git tag + catalog version + BLE firmware-revision string;
release notes per version (12 releases); data-driven release matrix; MIT LICENSE enumerates
vendored components; thorough AGENTS.md contribution rules.

Misalignments:

- **No MAINTAINERS/CODEOWNERS (LOW-MEDIUM).** Undocumented single-maintainer bus factor.
- **No SECURITY.md (MEDIUM).** Notable for firmware with BLE pairing, OTA/DFU update paths and an
  RF attack surface; no vulnerability reporting channel or supported-versions statement.
- **No deprecation/EOL policy (LOW)** for boards, roles, or CLI commands.
- **Contribution policy not where GitHub surfaces it (LOW).** AGENTS.md is thorough but there is
  no CONTRIBUTING.md, no issue/PR templates.
- **Legacy tag noise (COSMETIC).** ~44 old timestamp tags alongside the current scheme.

---

## Priority remediations

1. **Replace configure-time patching with a forked west module** (or a Zephyr module providing
   the lr11xx/lr20xx drivers and binding additions). Removes the largest non-standard mechanism
   and the fresh-checkout MCUboot failure class. (Areas 3, 5, 13)
2. **Make boards real Zephyr boards** (board.yaml, Kconfig.defconfig, defconfig) so role/feature
   defaults live in Kconfig; collapses most of `CMakeLists.txt:66-316` filename sniffing.
   (Areas 1, 3, 4)
3. **Stand up on-target bench testing** (the planned bench-smoke tier, twister
   `--device-testing` or pytest) on 2-4 always-connected boards; gate releases on it. (Area 9)
4. **Run the firmware build matrix on PRs**, at least a representative subset. (Area 10)
5. **Upstream patches 0017, 0019, 0004** (they already read like upstream commits). (Areas 2, 13)
6. **Replace the 25+ `CONFIG_SOC_*` gates** in common CLI/GPS/RNG paths with capability Kconfig
   symbols or DT properties — enforce the project's own rule 7. (Area 14)
7. **Convert RTC-i2c, stc8h keypad, buzzer/poweroff glue into DEVICE_DT_DEFINE drivers** modeled
   on `adapters/sensors/spa06.c`. (Areas 5, 6)
8. **Add SECURITY.md and CODEOWNERS.** (Area 15)
9. Small fixes: `snprintf` for the `sprintf` reply paths in `app/*`; route printk hex dumps
   through `LOG_HEXDUMP_DBG`; add a version comment to the west.yml pin; align
   upload-artifact action versions; add CONTRIBUTING.md pointing at AGENTS.md.
