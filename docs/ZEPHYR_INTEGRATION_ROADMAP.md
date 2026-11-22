# ZephCore — Zephyr Integration Roadmap

Status: proposal (2026-11-22). A task list for making ZephCore integrate with Zephyr the standard way.
It builds on [ZEPHYR_ALIGNMENT_AUDIT.md](ZEPHYR_ALIGNMENT_AUDIT.md): audit remediations 1, 2, 4 and 5
are expanded here into concrete tasks; 3, 6, 7, 8 and 9 are not repeated.

Three goals:

1. **ZephCore consumable as a Zephyr module** from any west workspace: boards, bindings, LoRa drivers
   and snippets available to a third-party application, not only to `zephcore/` itself.
2. **Configuration through Zephyr mechanisms** (`zephyr/module.yml`, snippets, board `Kconfig.defconfig`,
   sysbuild Kconfig, `VERSION`) instead of the hand-coded conf/overlay chain and filename sniffing in
   `zephcore/CMakeLists.txt` (`zephcore_add_conf`, role detection from `_user_conf_names`, the
   auto-included feature block, the role cross-check).
3. **`build.sh` stays** as the batch/release driver, but consumes a declarative row schema instead of
   conf-file paths and board-name regexes.

Carried through every phase (AGENTS.md): byte compatibility on air and on the companion protocol;
releases build from `zephcore.yml` alone; `board.conf` never repeats a parent; one owner per upstream
file; every current board keeps building, and each phase ends with the fingerprint checkpoint of §E,
identical or with a documented deliberate diff.

## A. Module-ization

Target layout, new files under `zephcore/`:

```
zephyr/module.yml
  name: zephcore
  build:
    cmake: drivers            # drivers/CMakeLists.txt: zephyr_library() per driver
    kconfig: drivers/Kconfig  # sourced for every image, MCUboot included
    settings:
      board_root: .           # boards/<platform>/<board>/board.yml
      dts_root: .             # dts/bindings/ — visible to MCUboot's DTS pass too
      snippet_root: .         # snippets/<name>/snippet.yml
drivers/lora/{lr11xx,lr20xx}/  drivers/gpio/native_linux/  drivers/spi/native_linux/
drivers/entropy/fake_entropy_native_sim.c  include/zephyr/drivers/lora/*.h
```

What it replaces: `cmake/zephyr_new_files.cmake` (whole file), patches `0001` and `0007` (both only
register drivers in upstream CMake/Kconfig), the hardcoded `../zephyr` / `../modules` / `BOARD_ROOT`
lines at the top of `CMakeLists.txt`, and the `BOARD_ROOT` + re-patching prologue of
`sysbuild/CMakeLists.txt`. The "binding must exist before MCUboot's DTS pass" failure class disappears:
a module `dts_root` is a workspace fact, not an application one.

Consumption. ZephCore itself: the manifest repo is already a west project, so `zephyr_module.py` picks
up `zephcore/zephyr/module.yml` with no manifest change. Third party: `west.yml` gets
`- name: zephcore, url: …, revision: v1.x` (or `import:` with a `name-allowlist` to inherit our pins) and
builds `west build -b <board> their_app -S zephcore-repeater`. The mesh application stays an application;
exposing it as a `zephyr_library` is the Phase 3 stretch.

Carried patches, disposition:

| Patch | Path out | Mechanism |
|---|---|---|
| 0001, 0007 (registration only) | drop | module `cmake:` / `kconfig:` |
| `patches/zephyr-new/` drivers, bindings, headers | move | module drivers; `zephyr_include_directories(include)` keeps the `<zephyr/drivers/lora/…>` include paths |
| 0003 sx126x (owns `drivers/lora/native/sx126x/*` + `semtech,sx126x-base.yaml`) | module driver | add `LORA_MODULE_BACKEND_ZEPHCORE` to upstream's named `choice LORA_MODULE_BACKEND` (`drivers/lora/Kconfig`), replacing `CONFIG_LORA_MODULE_BACKEND_NATIVE=y` in `zephcore_common.conf`; `lna-bypass-gpios` moves to a module binding `zephcore,sx126x.yaml` (`include: semtech,sx126x-base.yaml`), the 10 boards using it add `compatible = "zephcore,sx1262", "semtech,sx1262"`. Upstream main has since grown async RX, duty cycle and CAD: re-diff against it to shrink the fork (ADR 0006 is the behaviour contract) |
| 0004 sx127x 62.5 kHz + `loramac-node/0001` | upstream 0004; fork `loramac-node` | the one forked *module* that is justified: a Semtech-vendored table with no hook |
| 0005, 0006, 0008, 0009, 0010, 0019 | fork `zephyr` as the interim carrier, then upstream | `west.yml` `zephyr` → fork branch = upstream pin + `git am patches/zephyr/*` (ADR 0012 already allows a fork pin); deletes `cmake/zephyr_patches.cmake`. Keep a "fork budget": patches still on the branch, target 0 |

Risk (M): upstream now ships `drivers/lora/native/lr11xx` (`LORA_LR11XX_NATIVE`, `semtech,lr1121`).
ZephCore's `LORA_LR11XX` / `semtech,lr1110` do not clash today; gate ours on
`CONFIG_ZEPHCORE_LORA_LR11XX` and decide at the next bump whether upstream's covers LR1110.

## B. Roles and features as snippets

Principle: snippets are the user-facing selection axes only (role, variant, debug, opt-in features).
Everything the CMake block derives today from role × capability becomes a Kconfig default; its DT half
moves into the board DTS (role-independent) or into the role snippet (role-dependent). No board names
anywhere except per-board variant snippets.

Snippet set, `zephcore/snippets/<name>/snippet.yml`:

| Snippet | Replaces | Notes |
|---|---|---|
| `zephcore-repeater`, `-room-server`, `-observer`, `-kiss-modem` | `boards/common/{repeater,room_server,observer,kiss_modem}.conf` + role sniffing and cross-check in `CMakeLists.txt` | companion = no snippet (choice default). The repeater snippet's `boards:` section for ESP32 S3/C/S31 appends `SB_EXTRA_CONF_FILE` so `--sysbuild` is not hand-passed |
| `zephcore-serial-companion`, `-repeater-uplink`, `-packet-logging`, `-debug`, `-hangdebug`, `-gps-debug`, `-power-diag`, `-healthcheck` | the same-named confs | the debug snippet appends `debug_esp32.conf` under `boards: /.*\/esp32.*/` — a platform regex is allowed here: "no RTT, UART console" is a stated platform fact (AGENTS rule 8) |
| `zephcore-no-display` (variant) | `boards/<board>/no_display.conf` referenced from `zephcore.yml release.variants` | board-keyed by nature |

Derived features → Kconfig (`zephcore/Kconfig`, keyed on role × the capability symbols of §C):

| Today (CMake auto-include) | Tomorrow |
|---|---|
| `wifi_ota.conf` on ESP32 S3/C repeaters unless `wifi_ota: false` | `ZEPHCORE_WIFI_OTA default y if ZEPHCORE_ROLE_REPEATER && ZEPHCORE_BOARD_WIFI_OTA && BOOTLOADER_MCUBOOT`; the `NET_*`/`HTTP_*` values of `wifi_ota.conf` become `default … if ZEPHCORE_WIFI_OTA` in a `Kconfig.features` (app Kconfig is sourced before `Kconfig.zephyr`) |
| `ble_ota.conf` unless `ble_ota: false` | `ZEPHCORE_BLE_OTA default y if ZEPHCORE_ROLE_COMPANION && BOOTLOADER_MCUBOOT && !ZEPHCORE_BOARD_NO_BLE_OTA` (its `depends on` already exist) |
| `wifi_companion.conf` when `wifi: true` | `ZEPHCORE_COMPANION_WIFI default y if ZEPHCORE_ROLE_COMPANION && ZEPHCORE_BOARD_WIFI` |
| `pm_esp32.conf` + `.overlay` when the board declares light sleep | conf → `PM default y if (ROLE_REPEATER && ZEPHCORE_BOARD_LIGHT_SLEEP) \|\| (ROLE_COMPANION && ZEPHCORE_BOARD_LIGHT_SLEEP_COMPANION)`; the overlay (`&rtc_timer`, `wakeup-source`) → the DTS of the boards that declare it, inert with `PM=n`. Deliberate diff: `zephyr.dts` only |
| `esp32_cpu_80mhz.overlay`, `esp32_coretemp.overlay` | into each ESP32 board DTS / board-extension overlay (role-independent hardware facts) |
| `esp32s3_usb.conf` + console reroute, triggered by reading `board.overlay` text | conf → `ZEPHCORE_COMPANION_USB default y if ROLE_COMPANION && DT_HAS_ZEPHYR_CDC_ACM_UART_ENABLED && !ZEPHCORE_DEBUG`; the DT half (console → uart0) is role-dependent and cannot be Kconfig: it goes into a companion snippet `zephcore-companion-usb` that `build.sh` adds for companion rows of boards whose `Kconfig.defconfig` sets `ZEPHCORE_BOARD_USB_CDC`. Hardest case; gets ADR 0003's successor |
| generated `zephcore_version.conf` | app `VERSION` file → `BT_DIS_FW_REV_STR default "$(APP_VERSION_EXTENDED_STRING)"`, `FIRMWARE_VERSION` from `app_version.h`; `build.yml` reads `VERSION` for the tag |

What snippets cannot express, and the answer:

- *Conf-paired overlays* (`zephcore_add_conf` pairing `foo.conf` with `foo.overlay`): a snippet *is* the
  pairing, listed explicitly in one `snippet.yml`.
- *Ordering* (`partitions.overlay` last, `board.overlay` first, ADR 0003): snippets apply after board files
  and before `-D EXTRA_*`. Once partitions live in the board DTS (§C) there is nothing left to order, and
  the MCUboot sharing in `sysbuild/CMakeLists.txt` goes away because both images read the same board DTS.
- *Mutual exclusion* (serial vs USB companion, double `wifi_companion`): already `depends on` relations in
  Kconfig; the CMake guards are dropped.
- *Role vs conf cross-check*: no longer needed, the role has one input, the Kconfig choice.

## C. Boards

`zephcore.yml` keeps only what Zephyr has no slot for: `release:`, `catalog:`, `linux:`, and an explicit
`stem:` if target strings change. `capabilities:` become promptless Kconfig symbols in a
`zephcore/Kconfig.board` (`ZEPHCORE_BOARD_WIFI`, `_LIGHT_SLEEP`, `_LIGHT_SLEEP_COMPANION`, `_NO_BLE_OTA`,
`_WIFI_OTA`, `_USB_CDC`), each board's `Kconfig.defconfig` setting `default y` under `if BOARD_<NAME>`
(the pattern `boards/nrf54l/me25ls02/Kconfig.defconfig` already uses). Where the capability is a DT
fact (CDC node, PSRAM size, `Kconfig.psram` already does this) prefer `DT_HAS_*` defaults over a board
symbol. This deletes the regex YAML read in `cmake/zephcore_board.cmake` and the `CAPS`/`CAP_LINE`
enforcement in `board_manifest.py`, and retires the two `BOARD_WIO_TRACKER_L1` gates in `Kconfig` into
that board's `Kconfig.defconfig`.

Two board classes (45 board directories, 10 with a `board.yml` today):

1. **ZephCore-owned boards**: `board.conf` → `<board>_defconfig` + `Kconfig.defconfig`; `board.overlay`
   and `partitions.overlay` → the board `.dts`. No app-side files remain.
2. **Upstream Zephyr boards with ZephCore fragments** (about 35): Zephyr board *extension*
   (`board.yml: board: extend: <upstream>` plus a `zephcore` variant) so overlay, defconfig and
   `Kconfig.defconfig` ship in the module and MCUboot sees the partitions. Cost: the target becomes
   `rak4631/nrf52840/zephcore`; `stem:` in `zephcore.yml` keeps published artifact names stable.
   Fallback if extension is too intrusive on nRF: Zephyr's automatic `boards/<board>.conf|.overlay` in
   the app directory (standard, zero CMake), extending only the ESP32 sysbuild boards. Decide after a
   pilot on one board of each kind (Phase 2).

`board_manifest.py` then enumerates from `zephcore.yml` + `board.yml` (`socs:` replaces the
`esp32[^/]*` regex and `PLATFORM_DIRS`), reads sysbuild from the role row rather than guessing from the
chip name, and `check` validates snippet names against `snippets/*/snippet.yml`.

## D. `build.sh` row schema

Today (`board_manifest.py matrix`): `target|role|variant|confs|stem|sysbuild|host|cross` with conf paths.
Tomorrow:

```
target | role | snippets (;) | stem | sysbuild | host | cross
rak4631/nrf52840/zephcore   | repeater  | zephcore-repeater                        | rak4631      |          |     |
heltec_t114/…               | companion | zephcore-no-display                      | heltec_t114-noscreen |  |     |
xiao_esp32s3/esp32s3/procpu | companion | zephcore-companion-usb                   | …            | sysbuild |     |
native_sim                  | companion | zephcore-linux-femtofox                  | zephcore_linux_femtofox | | arm | /usr/bin/arm-linux-gnueabihf-
```

`build_row` becomes `west build -b $target zephcore --pristine [--$sysbuild] -S a -S b`; no
`-DEXTRA_CONF_FILE`. Linux presets become snippets `zephcore-linux-<preset>`.

Packaging class from the build, not the name: read `build/*/zephyr/.config` after the build —
`CONFIG_BUILD_OUTPUT_UF2` → uf2 (+ dfu zip), `CONFIG_BOOTLOADER_MCUBOOT && CONFIG_SOC_FAMILY_ESPRESSIF_ESP32`
→ merged bin, `CONFIG_ARCH_POSIX` → exe, else hex. Offsets from DT instead of the literal `0x0`,
`0x10000` and `0x1000`: grow `scripts/dts_flash_size.py` into `dts_query.py <zephyr.dts> flash-size |
partition-offset boot_partition | partition-offset slot0_partition`. The ESP32-S31 (MCUboot at 0x2000)
is the first board the literal offset breaks. Flash size already comes from `&flash0 reg` in the board
DTS for the app and MCUboot alike (upstream removed the Kconfig in 2026-11, and the `sysbuild/` fragment
plumbing went with it). `sysbuild.conf` and `sysbuild/mcuboot.conf` stay: they are already the native way.

## E. Migration order, checkpoints, CI

Fingerprint tool (Phase 0 prerequisite): `tools/matrix_fingerprint.py` builds every matrix row with
`SOURCE_DATE_EPOCH` fixed (CMake already honours it) and records per row the sha256 of
`zephyr.bin` / `zephyr.signed.bin`, the sorted `.config` and `zephyr.dts`. A phase is complete when the
diff against the committed baseline is empty or every delta is listed in this document under
"Deliberate diffs" (expected: `.dts` for pm/coretemp/80 MHz moving into boards; `BT_DIS_FW_REV_STR`
source; target strings if board extension is adopted).

CI: `build.yml` gains a `pull_request` job building one board per platform × every role from the new
rows (`west twister -T zephcore --build-only` with a `sample.yaml` whose scenarios set
`extra_args: SNIPPET=zephcore-<role>` and `sysbuild: true` where needed), replacing the two hand-written
"compile-only roles" steps; `tests.yml` adds `board_manifest.py check` and `zephyr_module.py`
validation; the modules cache key adds `zephcore/zephyr/module.yml`.

### Phased task list

**Phase 0 — quick wins (no behaviour change)**
- [ ] S — `tools/matrix_fingerprint.py` + committed baseline for the current matrix (prerequisite for all).
- [ ] S — `west.yml`: `import:` with a `name-blocklist` / `group-filter` dropping HALs ZephCore never builds (faster CI cache).
- [ ] S — `CMakeLists.txt`, `sysbuild/CMakeLists.txt`: derive the workspace from `west topdir` / `$ENV{ZEPHYR_BASE}` instead of `../`.
- [ ] S — `VERSION` file replaces `ZEPHCORE_FIRMWARE_VERSION` and the generated conf; `build.yml` tag derivation reads it. Binary identical expected.
- [ ] S — `zephyr/module.yml` with `settings:` only (`board_root`, `dts_root`, `snippet_root`); move `patches/zephyr-new/dts/bindings/*` to `dts/bindings/`; delete both `BOARD_ROOT` lines. Risk: duplicate binding paths, verify `DTS_ROOT` dedup. **Checkpoint 0.**
- [ ] S — PR CI build subset (`build.yml` `pull_request` job, representative rows). Audit #4.

**Phase 1 — patches out of configure time**
- [ ] M — Module drivers: lr11xx, lr20xx, native-linux spi/gpio, fake entropy and headers under `drivers/` + `include/`; `module.yml build: cmake/kconfig`; delete `zephyr_new_files.cmake`, patches 0001/0007. Depends: Phase 0 module.yml.
- [ ] M — Fork branch for `zephyr`: `git am` the remaining patches, point `west.yml` at it (ADR 0012 mechanism); delete `zephyr_patches.cmake` and both `include()` sites. Fork `loramac-node` for the sx1276 patch. **Checkpoint 1** (binary identical).
- [ ] M — Upstream 0004, 0009, 0010, 0019 (audit #5) and 0005's config half. Shrinks the fork branch each bump.
- [ ] L — sx126x as module driver: `LORA_MODULE_BACKEND_ZEPHCORE` choice entry, `zephcore,sx126x` binding, dual compat on the FEM boards; remove 0003 from the fork. Risk H (radio timing byte-compat: host tests + bench, ADR 0006).

**Phase 2 — Zephyr-native configuration**
- [ ] M — Snippets for roles, opt-ins, debug, variants, linux presets; `board_manifest.py matrix` emits the new row schema; `build.sh` consumes `-S`. Keep the CMake chain alive in parallel until **Checkpoint 2a** shows identical `.config`.
- [ ] M — Capability Kconfig symbols + `Kconfig.defconfig` for the owned boards; feature auto-enable moved to Kconfig defaults (`Kconfig.features`); delete the role sniffing, the auto-include block, the cross-check, and the YAML regex read in `zephcore_board.cmake`. Checkpoint 2a.
- [ ] L — Owned boards: `board.conf` / `board.overlay` / `partitions.overlay` folded into defconfig / `Kconfig.defconfig` / DTS; delete the partition sharing in `sysbuild/CMakeLists.txt`. Deliberate diff: `.dts` only.
- [ ] M — Pilot board extension on one upstream ESP32 board (`xiao_esp32s3`) and one nRF (`rak4631`); decide the class-2 strategy; roll out (L). **Checkpoint 2b.**
- [ ] S — `build.sh` packaging class from `.config`, offsets from `dts_query.py`; delete the chip regex and the literal offsets.
- [ ] S — Move `dfu_zip` and the hal_espressif absolute-path `bootloader_random_*.c` source out of the app CMake (module `zephyr_library_sources_ifdef`).

**Phase 3 — external consumption and tests**
- [ ] M — `sample.yaml` / twister matrix for compile coverage on PRs; `board_manifest.py check` validates snippet names and `board.yml` presence.
- [ ] M — Consumer proof: `samples/consumer/` with a minimal west manifest that imports ZephCore and builds a hello-world-sized app on `heltec_wifi_lora32_v4` with `-S zephcore-repeater`. Goal 1 is met when this passes in CI.
- [ ] S — Delete `zephcore_board.cmake` once `zephcore_board_file()` has no callers (`battery_curve.c` / `sd_quiesce.c` become `zephyr_library_sources` via the board's `board.cmake`).
- [ ] L (stretch) — `ZEPHCORE_LIB`: the mesh core as a `zephyr_library` so third-party apps link it rather than copying `src/`.
