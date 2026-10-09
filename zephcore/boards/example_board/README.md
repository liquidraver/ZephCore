# ZephCore: Adding a Board

This directory is the template for a new board. It is not a buildable board.

| File | Copy it when | What it is |
|---|---|---|
| [`board.conf`](board.conf) | always | Kconfig that cannot be derived from devicetree: name, SoftDevice ID, radio type |
| [`board.overlay`](board.overlay) | always, unless a custom `.dts` carries everything | Devicetree: radio, flash layout, console, peripherals. Every example is an `#if 0` block taken from a real board |
| [`partitions.overlay`](partitions.overlay) | ESP32-S3, ESP32-C3, ESP32-C6 | The flash map, shared by the app and MCUboot |
| [`zephcore.yml`](zephcore.yml) | always | The board manifest: registers the board for releases, the catalog and the docs |

Looking for something else?

- **Which boards exist, and their build strings:** [docs/supported_boards.md](../../../docs/supported_boards.md)
  (generated from the manifests).
- **Building, roles, flashing, the config chain:** [docs/BUILDING.md](../../../docs/BUILDING.md).
- **Native Linux presets** (not boards): [docs/LINUX_NATIVE.md](../../../docs/LINUX_NATIVE.md).

## 1. Two patterns

**Pattern 1: the board already exists in Zephyr.** You add three or four files and no board definition.

```
zephcore/boards/<platform>/<board>/
  board.conf
  board.overlay
  partitions.overlay     ESP32-S3 / C3 / C6 only
  zephcore.yml
```

References: `nrf52840/rak4631`, `nrf52840/wio_tracker_l1`, `nrf54l/xiao_nrf54l15`, `mg24/xiao_mg24`,
`esp32/xiao_esp32c3`, `esp32/xiao_esp32s3`, `esp32/heltec_wifi_lora32_v3`, `esp32/ttgo_tbeam` (classic ESP32),
`stm32wl/lora_e5_mini`.

**Pattern 2: a custom board.** The same directory also holds a complete Zephyr board definition:

| File | Purpose |
|---|---|
| `board.yml` | Board name, vendor, SoC |
| `Kconfig.<board>` | Selects the SoC |
| `<board>[_<soc>[_<cluster>]].dts` | The devicetree. Named after the build target, as Zephyr expects; copy a sibling's naming |
| `<board>-pinctrl.dtsi` | Pin control |
| `<board>_defconfig` | Minimal defconfig |
| `board.cmake` | Flash runners (optional: a UF2-only board needs none) |
| `pre_dt_board.cmake`, `Kconfig.defconfig`, `Kconfig` | Only when needed: DTC warning flags, SoC defaults, extra heap |
| `<board>.yaml` | Twister metadata, optional |
| `battery_curve.c` | Optional per-board battery discharge curve, compiled automatically when present |

With a custom `.dts` the hardware can be described there entirely, and `board.overlay` is then optional.

References: `nrf52840/ikoka_nano_30dbm` (minimal, radio only), `nrf52840/thinknode_m1` (e-paper, GPS, QSPI,
buzzer, two buttons), `nrf52840/t1000_e` (LR1110), `esp32/lilygo_t3s3` (ESP32-S3), `nrf54l/me25ls02` (nRF54L15).

Two rules hold for both patterns:

- **The directory name is the board name**, the first component of the `west build -b` string.
- **The parent directory is the platform.** It selects `<platform>_common.conf`; there is nothing to register in
  `CMakeLists.txt`. A board name may exist under one platform directory only (the build stops otherwise).

| Directory | Platform conf | SoCs |
|---|---|---|
| `boards/nrf52840/` | `nrf52_common.conf` | nRF52840 with the Adafruit UF2 bootloader |
| `boards/nrf54l/` | `nrf54l_common.conf` | nRF54L15, nRF54LM20A |
| `boards/esp32/` | `esp32_common.conf` (+ `esp32c3s3_common.conf` on S3 and C3) | ESP32, ESP32-S3, ESP32-C3, ESP32-C6 |
| `boards/mg24/` | `mg24_common.conf` | EFR32MG24 |
| `boards/stm32wl/` | `stm32wl_common.conf` | STM32WL |

## 2. Steps

1. **Collect the facts first:** the schematic, and the board's Arduino MeshCore variant (`variants/<board>/`)
   if there is one. The variant gives the pin map, the SoftDevice version (nRF52) and the name MeshCore uses.
   Where a vendor annotation and the pin map disagree, trust the pin map.
2. **Create `boards/<platform>/<board>/`** and copy the template files into it.
3. **Edit `board.overlay`:** keep the examples for your platform, remove the `#if 0` / `#endif` lines around
   them, delete the rest, and change every pin. The minimum is a radio node, a flash layout and a console.
4. **Edit `board.conf`:** the two name strings, plus `CONFIG_ZEPHCORE_SD_FWID` on nRF52 and the radio type if
   the radio is not an SX126x.
5. **Edit `zephcore.yml`:** set `target`, and leave `release:` out for now.
6. **Build** the companion, then the repeater. Always `--pristine` when changing board or role.

   ```bash
   west build -b <board> zephcore --pristine
   ```

   ```bash
   west build -b <board> zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"
   ```

   nRF54L adds `--no-sysbuild`. An ESP32-S3 / C3 / C6 **repeater** adds `--sysbuild` (section 5). ESP32 and
   MG24 need their radio blobs once: `west blobs fetch hal_espressif` / `west blobs fetch hal_silabs`.
7. **Read back what was built.** CMake prints a `ZephCore config hierarchy` block with the board directory,
   platform, and the final conf and overlay lists: check that your files are in it. Then look at
   `build/zephyr/zephyr.dts` for the radio node, the `chosen` entries and the partition map.
8. **Validate on hardware**, then register the board for release (section 6).

## 3. `board.conf`

Required on every board:

| Symbol | Meaning |
|---|---|
| `CONFIG_ZEPHCORE_BOARD_NAME` | Name shown in the app and by the `board` command; 39 characters at most |
| `CONFIG_BT_DIS_MODEL_NUMBER_STR` | BLE Device Information model string |
| `CONFIG_ZEPHCORE_SD_FWID` | nRF52 only: `0x00B6` (s140 v6) or `0x0123` (s140 v7), matching the partition include |

Set only when the hardware calls for it (details and the values used in the tree are in the template):

| Symbol | When |
|---|---|
| `CONFIG_ZEPHCORE_RADIO_LR1110` / `_LR2021` | LR11xx or LR2021 radio. SX126x, LLCC68 and STM32WL need nothing |
| `CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM`, `_MAX_TX_POWER_DBM` | External PA: keep the chip below the PA's maximum input |
| `CONFIG_ZEPHCORE_MAX_CONTACTS`, `_MAX_CHANNELS`, `_OFFLINE_QUEUE_SIZE` | RAM-bound boards (ESP32 without PSRAM, classic ESP32, STM32WL) |
| `CONFIG_HEAP_MEM_POOL_SIZE` | nRF with a panel larger than 128x64 |
| `CONFIG_ESPTOOLPY_FLASHSIZE_8MB` / `_16MB` | ESP32 with more than 4 MB flash |
| `CONFIG_SPIRAM_MODE_OCT` | ESP32-S3 with 8 MB octal PSRAM (R8) |
| `CONFIG_ESPTOOLPY_FLASHMODE_DIO` (+ `_QIO=n`) | Classic ESP32 PICO-D4 |

Do not set what devicetree already decides (`ZEPHCORE_UI_DISPLAY`, `ZEPHCORE_UI_BUZZER`, `PWM`, `SPI`,
`NORDIC_QSPI_NOR`, `ESP_SPIRAM`), and do not put role or debug policy in a board: roles, logging and `BT=n`
come from `boards/common/*.conf`. `CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE` is off by default for every role and
is a user setting (`set rxduty`), not a board property.

The chain, later wins:

```
prj.conf → zephcore_common.conf → <platform>_common.conf → esp32c3s3_common.conf (S3/C3)
  → board.conf → auto-included feature confs → your EXTRA_CONF_FILE
```

Overlays follow the same order: `board.overlay` first, overlays paired with a conf after it, `partitions.overlay`
last ([ADR 0003](../../../docs/adr/0003-devicetree-overlay-precedence.md)). A `foo.overlay` next to a `foo.conf`
is paired automatically, which is how a board variant changes Kconfig and devicetree together
(`heltec_t114/no_display.conf` + `no_display.overlay`).

## 4. What the firmware reads from devicetree

Only `lora0` is required. Everything else enables its feature by being present.

| Devicetree | Effect |
|---|---|
| alias `lora0` | The radio, for example `semtech,sx1262`, `semtech,lr1110` or `semtech,lr2021` |
| `lfs_partition` + `filesystem.dtsi` | `/lfs`: identity, prefs, contacts, channels |
| chosen `zephyr,settings-partition` | NVS for BLE bonds (nRF52, ESP32). nRF54L and MG24 keep bonds in a file on `/lfs` |
| aliases `led0`, `led1`, `lora-tx-led` | Heartbeat, unread, TX blink |
| aliases `heartbeat-pwm-led`, `lora-tx-pwm-led` | Same heartbeat/TX blink, brightness-adjustable (`leds.brightness`). Point both at one `pwm-leds` node to share a single physical LED. On ESP32, leave `led0`/`led1` out of the board's `main()` GPIO setup when either alias is present: configuring the same pin as plain GPIO after the PWM driver has claimed it takes the pin back from the LEDC peripheral (`CONFIG_ZEPHCORE_LED_PWM` follows the aliases: it selects `CONFIG_PWM` and makes `src/server_main_common.cpp` and `app/main_observer.cpp` skip this for you) |
| alias `sw0` | User button; wake source from nRF System OFF and ESP32 light sleep |
| chosen `zephyr,display` | Display UI |
| node labelled `buzzer` + alias `buzzer` | PWM buzzer |
| node labelled `gnss` | GPS |
| aliases `gps-enable`, `gps-wakeup`, `gps-reset`, `gps-resetb`, `gps-vrtc-enable`, `gps-sleep-int`, `gps-rtc-int`; chosen `zephcore,gps-power` | GPS power control |
| `zephyr,user { io-channels; vbat-mv-multiplier; }` | Battery voltage; a regulator labelled `vbat_enable` is switched per read |
| `sensors-i2c.dtsi`, `rtc-i2c.dtsi` | I2C sensors (probed at runtime) and a battery-backed RTC (opt-in) |
| `nordic,qspi-nor` + `qspi-ext.dtsi` | `/ext` on external flash for contacts, channels and blobs |
| chosen `zephcore,companion-uart` | Companion protocol on a UART, for boards without Bluetooth |
| `zephcore,poweroff-gpios` | Pins released at shutdown on boards with a latched supply rail |

`board.overlay` has a worked example for each row.

## 5. Platform notes and traps

### nRF52840

- **SoftDevice version decides two things together:** the partition include (`nrf52_partitions_sdv6.dtsi` or
  `_sdv7.dtsi`) and `CONFIG_ZEPHCORE_SD_FWID`. The Arduino variant's linker script tells you which. The wrong
  pair puts the app at the wrong address and the board bootloops with no USB.
- **Include the shared partition file; do not write the map by hand.** It keeps `/lfs` and the bond store at
  the offsets every shipped ZephCore board uses, and it sets `zephyr,sram`, which the nRF52840 SoC file does
  not (without it the linker reports `region 'RAM' overflowed` on every build).
- **No 32.768 kHz crystal:** set `&lfclk { k32src = "rc"; k32src-accuracy-ppm = <250>; };` in devicetree. The
  old `CONFIG_CLOCK_CONTROL_NRF_K32SRC_*` choices are silently ignored
  ([ADR 0005](../../../docs/adr/0005-zephyr-main-pin.md)).
- **Wake from power-off** uses the `sw0` alias: `adapters/board/zephyr_poweroff.c` arms GPIO SENSE on that pin
  before System OFF. `nrf52_wakeup.dtsi` documents the intent in devicetree; include it only when a node
  labelled `buttons` exists, since a missing label is a build error.
- **QSPI flash:** declare it only if the chip is fitted, and disable an upstream `&qspi` the variant lacks.
- **SPIM0 / SPIM1 share their instance with TWIM0 / TWIM1.** Put the radio on `&spi2` or `&spi3`.
- **SSD1306 / SH1106 on a `nordic,nrf-twim` bus** needs `zephyr,concat-buf-size`, or the panel stays blank.

### nRF54L

- Build with `--no-sysbuild`. The SoC has no USB peripheral: no UF2 and no DFU, `zephyr.hex` over SWD.
- Enable `&xo` and `&lfclk` if the board DTS does not; the BLE controller does not build without them.
- **Port P2 has no GPIOTE.** No GPIO interrupt works there: the radio IRQ cannot be on P2, and a button on P2
  needs `polling-mode` on its `gpio-keys` node (`seeed_lr2021_evk`).

### ESP32

- **Boot and flash layout** ([ADR 0001](../../../docs/adr/0001-esp32-boot-and-flash-layout.md)): a plain
  companion build is simple boot at `0x0`. A repeater build on S3 / C3 / C6 pulls in WiFi OTA automatically
  and **must be built with `--sysbuild`**; without it the board stops enumerating USB. Releases are always the
  MCUboot `-merged.bin`. Classic ESP32 (`/esp32/` targets) is simple boot in every build and has no
  `partitions.overlay`.
- **`partitions.overlay` is shared with MCUboot.** Read the template before changing a layout, including the
  `_default_` versus `_amp_` base-table trap.
- **Flash size** defaults to 4 MB; state the real one in `board.conf` (it is copied to MCUboot).
- **ESP32-S3 native USB** ([ADR 0002](../../../docs/adr/0002-esp32s3-native-usb-companion.md)): including
  `esp32s3_usb_otg.dtsi` from `board.overlay` gives companion builds a USB transport. esptool then cannot
  auto-reset the board; use `start dfu` or a 1200-baud touch.
- **PSRAM** is enabled from devicetree when the board DTS uses an R-suffix module file. Only the mode needs
  stating, because the upstream default cannot be overridden conditionally:

  | Part | PSRAM | `board.conf` |
  |---|---|---|
  | `R2` | 2 MB quad | nothing |
  | `R8` | 8 MB octal | `CONFIG_SPIRAM_MODE_OCT=y` |

- **Manifest capabilities:** `wifi: true` gives the companion a WiFi transport and belongs only on boards with
  the RAM for WiFi and BLE together ([ADR 0009](../../../docs/adr/0009-wifi-companion-boards.md)).
  `light_sleep: true` makes repeaters light-sleep; the requirements are in `boards/common/pm_esp32.conf`, and
  it is declared only after it was validated on the hardware. Either way, give NSS, RESET and every MCU-driven
  RF switch or FEM line `ESP32_GPIO_SLEEP_HOLD_EN` from the start.
- **RAM, not flash, limits the companion.** Boards without PSRAM lower `MAX_CONTACTS`; the classic ESP32 also
  trims channels and the offline queue.
- **Classic ESP32 PICO-D4** bootloops in QIO flash mode: `CONFIG_ESPTOOLPY_FLASHMODE_DIO=y`. Console and CLI
  are on `uart0`.
- GPIO32 and up are on `&gpio1`: GPIO41 is `<&gpio1 9 ...>`.

### MG24

- Erase blocks are 8 KB: partition sizes are multiples of 8 KB.
- Enable `&se` (secure element: crypto and the TRNG).
- On the XIAO the default I2C pins are the radio's NSS / RXEN; disable `&i2c0`.

### STM32WL

- No Bluetooth and no USB device. The companion protocol and the CLI both run on USART1; the board selects it
  with the `zephcore,companion-uart` chosen node.
- 64 KB SRAM: the companion tables are cut hard in `board.conf`.
- `boards/stm32wl/lora_e5_mini/` is the only reference.

### SX127x

Source-only and unsupported: a separate path through Zephyr's loramac-node backend with no RX duty cycle and
no RX boost. `boards/esp32/ttgo_lora32/` is the one board on it, and its `board.conf` shows the overrides.

## 6. Registering the board

`zephcore.yml` is the only registry. `build.sh` (and so CI and the release), the Mesh America catalog,
[docs/supported_boards.md](../../../docs/supported_boards.md) and the board count in the root README are all
generated from it. The schema is documented at the top of `zephcore/scripts/board_manifest.py`.

- **Bring-up:** `target` only. The board builds locally and is listed as source-only.
- **Validated:** add `release:` with the roles to publish.
- **In the configurator:** add `catalog:`. `device` must be MeshCore's exact device name for the board to fold
  into their tile ([docs/PROVIDER_CATALOG.md](../../../docs/PROVIDER_CATALOG.md)).

After every manifest change:

```bash
python zephcore/scripts/board_manifest.py check
```

```bash
python zephcore/scripts/board_manifest.py docs
```

The notes under each list in `docs/supported_boards.md` are written by hand; add one if the board has
something a user must know before flashing.

## Quick reference: Wio-SX1262 on a XIAO

Every XIAO carries the Wio-SX1262 on the same D pins:

| Signal | XIAO pin | nRF52840 | nRF54L15 | MG24 | ESP32-C3 | ESP32-C6 | ESP32-S3 |
|---|---|---|---|---|---|---|---|
| DIO1 | D1 | P0.03 | P1.05 | PC01 | GPIO3 | GPIO1 | GPIO39 |
| RESET | D2 | P0.28 | P1.06 | PC02 | GPIO4 | GPIO2 | GPIO42 |
| BUSY | D3 | P0.05 | P1.07 | PC03 | GPIO5 | GPIO21 | GPIO40 |
| NSS | D4 | P0.04 | P1.10 | PC04 | GPIO6 | GPIO22 | GPIO41 |
| RXEN | D5 | P0.29 | P1.11 | PC05 | GPIO7 | GPIO23 | GPIO38 |
| SCK | D8 | P1.13 | P2.01 | PA03 | GPIO8 | GPIO19 | GPIO7 |
| MISO | D9 | P1.14 | P2.04 | PA04 | GPIO9 | GPIO20 | GPIO8 |
| MOSI | D10 | P1.15 | P2.02 | PA05 | GPIO10 | GPIO18 | GPIO9 |

RXEN and DIO2 together drive the module's RF switch, and DIO3 powers its TCXO at 1.8 V.
