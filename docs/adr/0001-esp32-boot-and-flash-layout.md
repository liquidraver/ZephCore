# ADR 0001 — ESP32 boot: simple boot for plain builds, MCUboot for OTA builds and releases

Status: accepted (recorded 2026-09-25)

## Context

**ESP32 flash** — Zephyr uses `CONFIG_ESP_SIMPLE_BOOT` by default (no MCUBoot). The standard build produces a
self-contained `zephyr.bin` that the ESP32 ROM bootloader loads directly from `0x0`. Just `west flash` — no
`--sysbuild`, no ceremony, works on a bare chip:

```bash
west build -b <board>/esp32s3/procpu zephcore --pristine
west flash --esp-device COMX
```

**Exception — WiFi AP OTA** (`wifi_ota.conf`): the HTTP OTA updater writes to MCUBoot slot1 and requires
MCUBoot. When building with `wifi_ota.conf`, add `--sysbuild` and flash once to seat MCUBoot:

```bash
west build -b <board>/esp32s3/procpu zephcore --pristine --sysbuild -- \
  -DEXTRA_CONF_FILE="boards/common/wifi_ota.conf"
west flash --esp-device COMX
```

Subsequent OTA-capable builds still need `--sysbuild`; plain builds without `wifi_ota.conf` do not.

**`repeater.conf` pulls `wifi_ota.conf` in by itself on S3/C-series boards, so ESP32 repeater builds always need
`--sysbuild`.** The ESP32 auto-include block in `CMakeLists.txt` appends it whenever the platform is `esp32`, the
build is classified as repeater (from the user conf *names* — `repeater.conf` — or
`-DCONFIG_ZEPHCORE_ROLE_REPEATER`; checked against Kconfig after `find_package`), and the board is not a classic
`/esp32/` one. Without `--sysbuild` the app still comes out built *for* the MCUboot slot
(`CONFIG_BOOTLOADER_MCUBOOT=y`, `CONFIG_FLASH_LOAD_OFFSET=0x10000`) but is never signed: `west flash` writes it to
`0x10000`, leaves `0x0` untouched, and whatever MCUboot is already there rejects the unsigned header. The symptom
is not a build or flash error — both report success and the hash verifies — but the board stops enumerating USB
entirely, and only the BOOT button gets it back. Companion builds on the same boards really are simple-boot
(`CONFIG_ESP_SIMPLE_BOOT=y`, load offset `0x0`), which is why this bites only when switching a board to the
repeater role.

**GitHub Release artifacts diverge from the plain-build default.** `build.sh` always builds S3/C-series boards
(`heltec_wifi_lora32_v3/v4/v43/v4_r8`, `station_g2`, `xiao_esp32s3`, `xiao_esp32c6`, `lilygo_tlora_c6`,
`heltec_wireless_tracker`, `thinknode_m9`) with `--sysbuild`, regardless of `wifi_ota.conf` — `sysbuild.conf` at
the repo root forces `SB_CONFIG_BOOTLOADER_MCUBOOT=y` whenever `--sysbuild` is passed. So the published release
for these boards is always an MCUboot layout (MCUboot @ `0x0`, signed app @ `0x10000`), never the simple-boot
`zephyr.bin`. (The app slot is at `0x10000` — right after the 64 KB MCUboot region, matching the
Arduino/ESP-IDF app offset; it was `0x20000` before 2026-07-09. The flash map lives in per-board
`boards/esp32/<board>/partitions.overlay`, fed to **both** the app and the MCUboot image via
`sysbuild/CMakeLists.txt` so their slots match. Before that split, `board.overlay` reached only the app, so
MCUboot used the upstream base table and looked for OTA images at a different `image-1` address than the app
wrote them to, silently reverting every WiFi-OTA update on the slot-customizing boards.) Only `-merged.bin`
(MCUboot + signed app, esptool `merge-bin`) is published for them — the signed app alone isn't bootable
standalone and isn't shipped (see [GH #42](https://github.com/liquidraver/ZephCore/issues/42): publishing it as a
"plain .bin" caused bricked boards when flashed like classic-ESP32's self-contained image). Classic ESP32 boards
(T-Beam, PICO-D4 — chip name `esp32`, not `esp32s3`/`esp32c*`) still use simple-boot and ship a genuinely
self-contained plain `.bin` at the `0x1000` ROM bootloader offset. Flashing `-merged.bin` via
`west flash`/`esptool write_flash 0x0 <file>` only touches MCUboot + the app slot — it does not reach
`storage_partition`/`lfs_partition` (identity, prefs, contacts, BLE bonds), so it's safe for routine updates as
well as first flash.

## Decision

Plain local builds of S3/C-series companions stay simple-boot; repeater builds and every release build use
MCUboot (`--sysbuild`), with one per-board `partitions.overlay` shared by app and MCUboot.

## Consequences

- A repeater build without `--sysbuild` bricks USB silently (see above): always pass it.
- Releases publish only `-merged.bin` for S3/C-series boards.
- `sysbuild.conf` sets `SB_CONFIG_BOOT_SIGNATURE_TYPE_NONE` (the RSA default overflows MCUboot's 64 KB slot).
