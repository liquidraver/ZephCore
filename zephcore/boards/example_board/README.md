ZephCore — Adding a New Board
=============================


Supported Boards
----------------

### nRF52840

| Board               | Build string                              | Flash                         |
|----------------------|-------------------------------------------|-------------------------------|
| RAK4631              | `west build -b rak4631 zephcore`          | UF2 drag-drop or `west flash` |
| RAK3401 1W           | `west build -b rak3401_1watt zephcore`    | UF2 drag-drop or `west flash` |
| Wio Tracker L1       | `west build -b wio_tracker_l1 zephcore`   | UF2 drag-drop or `west flash` |
| SenseCAP Solar       | `west build -b sensecap_solar zephcore`    | UF2 drag-drop or `west flash` |
| XIAO nRF52840        | `west build -b xiao_nrf52840 zephcore`     | UF2 drag-drop or `west flash` |
| T1000-E              | `west build -b t1000_e zephcore`          | UF2 drag-drop or `west flash` |
| ThinkNode M1         | `west build -b thinknode_m1 zephcore`     | UF2 drag-drop or `west flash` |
| ThinkNode M3         | `west build -b thinknode_m3 zephcore`     | UF2 drag-drop or `west flash` |
| ThinkNode M6         | `west build -b thinknode_m6 zephcore`     | UF2 drag-drop or `west flash` |
| RAK WisMesh Tag      | `west build -b rak_wismesh_tag zephcore`  | UF2 drag-drop or `west flash` |
| Ikoka Nano 30dBm     | `west build -b ikoka_nano_30dbm zephcore` | UF2 drag-drop                 |
| LilyGo T-Echo        | `west build -b lilygo_techo zephcore`     | UF2 drag-drop or `west flash` |

UF2 flash: Double-tap reset button, drag `build/zephyr/zephyr.uf2` to the USB drive.
SWD flash: `west flash` (requires J-Link, pyocd, or nrfjprog connected).

### ESP32

| Board               | Build string                                              | Flash           |
|----------------------|-----------------------------------------------------------|-----------------|
| XIAO ESP32-C3        | `west build -b xiao_esp32c3 zephcore`                   | `west flash`    |
| XIAO ESP32-C6        | `west build -b xiao_esp32c6/esp32c6/hpcore zephcore`    | `west flash`    |
| LilyGo TLoRa C6      | `west build -b lilygo_tlora_c6/esp32c6/hpcore zephcore` | `west flash`    |
| XIAO ESP32-S3        | `west build -b xiao_esp32s3/esp32s3/procpu zephcore`     | `west flash`    |
| Station G2           | `west build -b station_g2/esp32s3/procpu zephcore`       | `west flash`    |
| Heltec V3            | `west build -b heltec_wifi_lora32_v3/esp32s3/procpu zephcore` | `west flash` |
| Heltec V4.2 (GC1109 PA)  | `west build -b heltec_wifi_lora32_v4/esp32s3/procpu zephcore`  | `west flash` |
| Heltec V4.3 (KCT8103L PA) | `west build -b heltec_wifi_lora32_v43/esp32s3/procpu zephcore` | `west flash` |

**Heltec V3 console:** ZephCore routes console/shell to `uart0` on V3. Use the UART serial port for boot logs and CLI.

**Heltec V4.2 vs V4.3:** The hardware revision is printed on the PCB silkscreen. If
unclear, check GPIO2's default pull: the V4.2 GC1109 PA has an internal pull-down
(GPIO2 reads LOW at boot), while the V4.3 KCT8103L PA has an internal pull-up
(GPIO2 reads HIGH). Only difference in firmware: TX control pin GPIO46→GPIO5.

ESP32 flash uses esptool over USB via `west flash`. Hold BOOT button if the device
doesn't enter download mode automatically.

Zephyr's `CONFIG_ESP_SIMPLE_BOOT` is active by default (no MCUBoot). The build
produces a self-contained `zephyr.bin` that the ESP32 ROM bootloader loads directly
from `0x0` — no special first-flash procedure, works on a bare chip.

**Exception — WiFi AP OTA** (`boards/common/wifi_ota.conf`): the HTTP OTA updater
writes firmware to MCUBoot slot1 and requires MCUBoot. Add `--sysbuild` when building
with `wifi_ota.conf`, and run `west flash` once to seat MCUBoot before OTA works:
```bash
west build -b <board>/esp32s3/procpu zephcore --pristine --sysbuild -- \
  -DEXTRA_CONF_FILE="boards/common/wifi_ota.conf"
west flash --esp-device COMX
```

**One-time setup required (all ESP32 boards):**

```
# Download Espressif BLE controller blobs (closed-source, required for BLE)
west blobs fetch hal_espressif
```

Run once after `west init`/`west update`. Without it, CMake aborts with a blob
validation error. Re-run after any `west update` that bumps the `hal_espressif`
revision.

### nRF54L15

| Board               | Build string                                                           | Flash           |
|----------------------|------------------------------------------------------------------------|-----------------|
| XIAO nRF54L15        | `west build -b xiao_nrf54l15/nrf54l15/cpuapp zephcore --no-sysbuild` | `west flash`    |

Requires J-Link or CMSIS-DAP (built into XIAO board via SAMD11 bridge).
The `--no-sysbuild` flag is required (no MCUboot support yet).

### MG24 (Silicon Labs)

| Board               | Build string                              | Flash           |
|----------------------|-------------------------------------------|-----------------|
| XIAO MG24            | `west build -b xiao_mg24 zephcore`       | `west flash`    |

**First-time setup required:**

```
# Download Silicon Labs BLE controller blob
west blobs fetch hal_silabs

# Install pyocd + Silicon Labs device pack (if using pyocd)
pip install pyocd
pyocd pack install EFR32MG24B220F1536IM48
```

Flash with: `west flash --runner pyocd`

### Building for Repeater Role

Append `-- -DEXTRA_CONF_FILE="boards/common/repeater.conf"` to any build command:

```
west build -b rak4631 zephcore -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"
```

### Production Build (logging disabled)

```
west build -b rak4631 zephcore -- -DEXTRA_CONF_FILE="boards/common/prod.conf"
```

### Repeater + Production

```
west build -b rak4631 zephcore -- -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/prod.conf"
```

All build commands should include `--pristine` when switching between roles or boards.


Adding a New Board
------------------

There are TWO ways to add a board, depending on whether Zephyr
already has a board definition for your hardware.


### PATTERN 1: Existing Zephyr Board (overlay only)

Use this when your board already exists in Zephyr's tree.
You only need TWO files: board.conf + board.overlay

Examples: XIAO nRF54L15, XIAO MG24, XIAO ESP32-C3, RAK4631

Directory structure:

    zephcore/boards/<platform>/<board_name>/
      board.conf       — Kconfig (name, radio type)
      board.overlay    — DT overlay (LoRa SPI, partitions, peripherals)

Steps:
  1. Create directory: `boards/<platform>/<board_name>/`
     Platform folders: nrf52840, nrf54l, mg24, esp32
  2. Copy board.conf and board.overlay from THIS directory
  3. Uncomment the sections matching your platform
  4. Fill in YOUR pin numbers and partition layout
  5. Add board detection to CMakeLists.txt (~line 60-75):
     Add `BOARD MATCHES "your_board"` to the correct platform line
  6. Build and iterate!


### PATTERN 2: Fully Custom Board (new DTS)

Use this when your board does NOT exist in Zephyr's tree.
You need a full board definition: .dts, pinctrl, Kconfig, etc.

Examples: Ikoka Nano 30dBm, ThinkNode M1 (custom nRF52840 designs)

Look at existing custom boards as templates:

    zephcore/boards/nrf52840/ikoka_nano_30dbm/   — minimal (LoRa only)
    zephcore/boards/nrf52840/thinknode_m1/       — full-featured (EPD, GPS, QSPI, buzzer)

A full custom board includes:

    board.conf                           — Kconfig (name, radio, overrides)
    board.overlay                        — DT overlay (usually empty if DTS is complete)
    <board>_<soc>.dts                    — Full device tree
    <board>_<soc>-pinctrl.dtsi           — Pin control definitions
    board.yml                            — Board metadata (name, arch, SoC)
    Kconfig.<board>                      — SoC selection
    <board>_<soc>_defconfig              — Minimal defconfig
    board.cmake                          — Flash runner config

**Critical: `zephyr,sram` in the chosen node (nRF52840 only)**

The nRF52840 SoC DTSI defines `sram0` but does NOT set `zephyr,sram` in
the chosen node. Without it, the linker gets `RAM size = 0` and every
build fails with "region 'RAM' overflowed" regardless of actual RAM use.

Boards that include `<nordic/nrf52840_partition.dtsi>` get this for free.
Boards that use `nrf52_partitions_sdv6.dtsi` or `nrf52_partitions_sdv7.dtsi`
directly (without the upstream include) also get it now — those DTSIs set
`zephyr,sram = &sram0` themselves.

If you write a fully custom DTS that includes neither, add it yourself:

    chosen {
        zephyr,sram = &sram0;
        zephyr,code-partition = &code_partition;
        ...
    };


**nRF52 boards with a user button: button wakeup from System OFF**

`sys_poweroff()` on nRF52840 enters System OFF (~1µA). To allow waking via
button press (instead of only via USB/charger), GPIO SENSE bits must be set
before poweroff. This is done via the `wakeup-source` property on the
`gpio-keys` node, which the nRF52 GPIO driver handles automatically.

Boards that define a `buttons:` gpio-keys node must add at the END of their DTS:

    #include "../../common/nrf52_wakeup.dtsi"

Boards WITHOUT a `buttons` label (ikoka_nano) must NOT include it —
referencing an undefined `&buttons` label is a hard build error.

**RAK4631 user button:** `gpio-keys` on **P0.09** (NFC1). Short → next page (after
tap window), double-tap → previous page, long press (≥1s) → enter / activate.


What Goes in board.conf
-----------------------

Most hardware features are auto-detected from devicetree. board.conf
should ONLY contain settings that can't be inferred from hardware:

  REQUIRED (all boards):
    CONFIG_ZEPHCORE_BOARD_NAME          Human-readable name
    CONFIG_BT_DIS_MODEL_NUMBER_STR      BLE Device Information model

  REQUIRED (nRF52 only):
    CONFIG_ZEPHCORE_SD_FWID             SoftDevice firmware ID (0x00B6 or 0x0123)

  OPTIONAL (only if needed):
    CONFIG_ZEPHCORE_RADIO_LR1110        LR1110 radio (auto-selects SPI)
    CONFIG_ZEPHCORE_MAX_CONTACTS        Override for RAM-limited boards
    CONFIG_HEAP_MEM_POOL_SIZE           Override for large displays (>128x64)
    CONFIG_SEGGER_RTT_BUFFER_SIZE_UP    Shrink RTT on RAM-tight boards
    CONFIG_ESPTOOLPY_FLASHSIZE_16MB     ESP32 boards with 16MB flash
    CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM  Boards with external PA
    CONFIG_ZEPHCORE_MAX_TX_POWER_DBM      Boards with external PA
    CONFIG_ZEPHCORE_APC                   Adaptive Power Control — OFF by default.
                                          Reduces TX power when echo SNR shows excess margin.
                                          See apc.md for details on target margin tuning.

  AUTO-DETECTED (do NOT set in board.conf):
    CONFIG_PWM                          Auto from DT buzzer nodelabel
    CONFIG_ZEPHCORE_UI_BUZZER           Auto from DT buzzer nodelabel
    CONFIG_ZEPHCORE_UI_DISPLAY          Auto from DT zephyr,display chosen
    CONFIG_SPI                          Auto from ZEPHCORE_RADIO_LR1110
    CONFIG_NORDIC_QSPI_NOR             Auto from DT nordic,qspi-nor node
    CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE  Auto: ON for companion+SX1262, OFF for repeater/LR1110


Config Inheritance
------------------

    prj.conf                           Always loaded first
      |
    zephcore_common.conf               BLE, storage, input, LoRa, crypto, sensors
      |
    <platform>_common.conf             Platform-specific overrides only
      |                                  nrf52_common.conf  — UF2, USB CDC, DLE, RTT
      |                                  nrf54l_common.conf — DLE, RTT
      |                                  mg24_common.conf   — SiLabs blob stacks, heap
      |                                  esp32_common.conf  — Espressif blob stacks, heap
      |
    board.conf                         Board name, radio type, board-specific

DO NOT duplicate settings from parent configs in board.conf!


Quick Reference: Wio-SX1262 XIAO Pin Mapping
---------------------------------------------

All XIAO boards use the same D-pin assignment for Wio-SX1262:

    Signal | XIAO Pin | nRF52840  | nRF54L15  | MG24      | ESP32-C3
    -------|----------|-----------|-----------|-----------|--------
    DIO1   | D1       | P0.03     | P1.05     | PC01      | GPIO3
    RESET  | D2       | P0.28     | P1.06     | PC02      | GPIO4
    BUSY   | D3       | P0.05     | P1.07     | PC03      | GPIO5
    NSS    | D4       | P0.04     | P1.10     | PC04      | GPIO6
    RXEN   | D5       | P0.29     | P1.11     | PC05      | GPIO7
    SCK    | D8       | P1.13     | P2.01     | PA03      | GPIO8
    MISO   | D9       | P1.14     | P2.04     | PA04      | GPIO9
    MOSI   | D10      | P1.15     | P2.02     | PA05      | GPIO10

Note: nRF52840 D-pin mapping varies by board (XIAO nRF52840 shown).
RAK4631 has SX1262 integrated — different pinout entirely.


RAK4631 LEDs and overlay (RAK4631)
-----------------------------------

- **Polarity:** Stock Zephyr RAK4631 DTS uses `GPIO_ACTIVE_LOW` for the two user
  LEDs (P1.3 / P1.4). On hardware here they are **active-high** — the overlay
  overrides `green_led` / `blue_led` to `GPIO_ACTIVE_HIGH`.
- **Aliases:** Overlay sets `led0`→green (heartbeat), `led1`→blue (unread +
  LoRa TX blink via `lora-tx-led`). On repeaters, firmware does not drive the
  second LED as “unread”; blue stays TX-only.
