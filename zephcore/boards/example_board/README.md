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
| ProMicro SX1262      | `west build -b promicro_sx1262 zephcore`   | UF2 drag-drop or `west flash` |
| T1000-E              | `west build -b t1000_e zephcore`          | UF2 drag-drop or `west flash` |
| ThinkNode M1         | `west build -b thinknode_m1 zephcore`     | UF2 drag-drop or `west flash` |
| ThinkNode M3         | `west build -b thinknode_m3 zephcore`     | UF2 drag-drop or `west flash` |
| ThinkNode M6         | `west build -b thinknode_m6 zephcore`     | UF2 drag-drop or `west flash` |
| RAK WisMesh Tag      | `west build -b rak_wismesh_tag zephcore`  | UF2 drag-drop or `west flash` |
| Ikoka Nano 30dBm     | `west build -b ikoka_nano_30dbm zephcore` | UF2 drag-drop                 |
| GAT562 30S Mesh Kit  | `west build -b gat562_30s zephcore`       | UF2 drag-drop or `west flash` |
| LilyGo T-Echo        | `west build -b lilygo_techo zephcore`     | UF2 drag-drop or `west flash` |
| Heltec T114          | `west build -b heltec_t114 zephcore`      | UF2 drag-drop or `west flash` |
| Heltec Mesh Node T096 | `west build -b heltec_t096 zephcore`     | UF2 drag-drop or `west flash` |

**Heltec T114 screenless:** append `boards/nrf52840/heltec_t114/no_display.conf` to `EXTRA_CONF_FILE` for units without the TFT module.

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
| Heltec Wireless Tracker | `west build -b heltec_wireless_tracker/esp32s3/procpu zephcore` | `west flash` |
| LilyGo T-Beam v1.2     | `west build -b ttgo_tbeam/esp32/procpu zephcore`               | `west flash` |

**Heltec V3 console:** ZephCore routes console/shell to `uart0` on V3. Use the UART serial port for boot logs and CLI.

**Heltec V4.2 vs V4.3:** The hardware revision is printed on the PCB silkscreen. If
unclear, check GPIO2's default pull: the V4.2 GC1109 PA has an internal pull-down
(GPIO2 reads LOW at boot), while the V4.3 KCT8103L PA has an internal pull-up
(GPIO2 reads HIGH). Only difference in firmware: TX control pin GPIO46→GPIO5.

**LilyGo T-Beam v1.2:** Classic ESP32 (PICO-D4) board — several caveats apply:
- Upstream Zephyr DTS models the SX1276 variant; `board.overlay` overrides the radio node to SX1262 on the same SPI3 wiring.
- PICO-D4 rev 1.0 bootloops when the bootloader tries to enable QIO flash mode. `board.conf` forces DIO (`CONFIG_ESPTOOLPY_FLASHMODE_DIO=y`). Any new classic ESP32 board with PICO-D4 needs this.
- Classic ESP32 DRAM is much smaller than ESP32-S3. `board.conf` shrinks contacts/channels/queue (`MAX_CONTACTS=160`, `MAX_CHANNELS=8`, `OFFLINE_QUEUE_SIZE=128`) to fit.
- AXP2101 PMU manages LoRa and GPS power rails via Zephyr regulator + fuel-gauge drivers (auto-enabled from the upstream DTS PMU nodes). Add `CONFIG_FUEL_GAUGE=y` to boards that expose battery state-of-charge over I2C.
- Console/CLI are on `uart0` (onboard USB-UART). No native USB on classic ESP32.

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

### SX127x Boards (loramac-node backend)

ZephCore supports SX1272/SX1276/SX1278 via the loramac-node backend — a separate radio path from the native SX126x driver used by all other boards. The TTGO LoRa32 is the reference implementation:

| Board          | Build string                                   | Flash        |
|----------------|------------------------------------------------|--------------|
| TTGO LoRa32    | `west build -b ttgo_lora32/esp32/procpu zephcore` | `west flash` |

SX127x boards require these `board.conf` overrides (the `zephcore_common.conf` default enables the native SX126x path):

```kconfig
CONFIG_LORA_MODULE_BACKEND_NATIVE=n
CONFIG_LORA_MODULE_BACKEND_LORAMAC_NODE=y
CONFIG_ZEPHCORE_RADIO_SX127X=y
CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE=n   # lora_recv_duty_cycle not implemented for SX127x
CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM=17 # PA_BOOST max without external PA
```

Classic ESP32 + PICO-D4 also need the DIO flash mode fix (see T-Beam note above).

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

### Building for Room Server Role

Append `-- -DEXTRA_CONF_FILE="boards/common/room_server.conf"` to any build command:

```
west build -b rak4631 zephcore -- -DEXTRA_CONF_FILE="boards/common/room_server.conf"
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
    CONFIG_ESPTOOLPY_FLASHMODE_DIO=y    Classic ESP32 PICO-D4 boards (bootloops otherwise)
    CONFIG_ESPTOOLPY_FLASHMODE_QIO=n    Companion to the DIO override above
    CONFIG_FUEL_GAUGE=y                 Boards with AXP2101 or other I2C fuel gauge
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
    CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE  OFF by default for all roles (boot default only;
                                        runtime toggle via CLI "set rxduty on/off")
    CONFIG_ESP_SPIRAM                   Auto: ON when DT psram0 size > 0 (ESP32-S/C5)
    CONFIG_ESP_SPIRAM_SIZE              Auto: read from DT psram0 size by upstream Kconfig

  NOT auto-detected (must set in board.conf for ESP32-S3 R8 boards):
    CONFIG_SPIRAM_MODE_OCT              R8 chips (8 MB OPI octal) need this set
                                        to "y" in board.conf. R2 chips (2 MB QSPI
                                        quad) need nothing — QUAD is the upstream
                                        default. See "ESP32 PSRAM" section below.


ESP32 PSRAM (auto-enable, per-board mode on S3 R8)
--------------------------------------------------

`CONFIG_ESP_SPIRAM=y` is enabled automatically from devicetree by
[zephcore/Kconfig.psram](../../Kconfig.psram). As long as a board's DTS
includes a Zephyr WROOM dtsi with an R-suffix (e.g. `esp32s3_wroom_n16r2.dtsi`,
`esp32s3_wroom_n8r8.dtsi`), PSRAM lights up — no `board.conf` entry needed
for the basics. Boards without PSRAM use a non-R-suffix dtsi (e.g.
`esp32s3_wroom_n8`), which leaves `psram0` at size = 0, so PSRAM stays off.
ESP32-C3/C6 are outside Espressif's PSRAM Kconfig gate entirely — no effect.

**Mode selection is silicon-strapped, but not auto-pickable here:** upstream
Kconfig.spiram sets an unconditional `default SPIRAM_MODE_QUAD` on the choice,
which wins over any conditional override we'd add downstream. So:

| Chip suffix | PSRAM size | Mode      | board.conf needs           |
|-------------|-----------|-----------|----------------------------|
| `R2`        | 2 MB      | QSPI quad | nothing — upstream default |
| `R8`        | 8 MB      | OPI octal | `CONFIG_SPIRAM_MODE_OCT=y` |

If you add a new ESP32-S3 **R8** board (e.g. `esp32s3_wroom_n*r8.dtsi`),
add a one-liner to its board.conf:

```kconfig
# PSRAM mode — ESP32-S3R8 is silicon-strapped OPI octal.
# Enable is auto-detected from DTS by Kconfig.psram; only the mode is set here.
CONFIG_SPIRAM_MODE_OCT=y
```

For aggressive DRAM relief beyond the heap rebalance, additional Kconfigs
`CONFIG_SPIRAM_FETCH_INSTRUCTIONS=y` and `CONFIG_SPIRAM_RODATA=y` move
text/rodata into PSRAM at boot, but introduce PSRAM-cache-miss timing
variability — only enable per-board after verifying radio and BLE paths
still meet their timing budgets.


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

    Signal | XIAO Pin | nRF52840  | nRF54L15  | MG24      | ESP32-C3  | ESP32-C6  | ESP32-S3
    -------|----------|-----------|-----------|-----------|-----------|-----------|----------
    DIO1   | D1       | P0.03     | P1.05     | PC01      | GPIO3     | GPIO1     | GPIO39
    RESET  | D2       | P0.28     | P1.06     | PC02      | GPIO4     | GPIO2     | GPIO42
    BUSY   | D3       | P0.05     | P1.07     | PC03      | GPIO5     | GPIO21    | GPIO40
    NSS    | D4       | P0.04     | P1.10     | PC04      | GPIO6     | GPIO22    | GPIO41
    RXEN   | D5       | P0.29     | P1.11     | PC05      | GPIO7     | GPIO23    | GPIO38
    SCK    | D8       | P1.13     | P2.01     | PA03      | GPIO8     | GPIO19    | GPIO7
    MISO   | D9       | P1.14     | P2.04     | PA04      | GPIO9     | GPIO20    | GPIO8
    MOSI   | D10      | P1.15     | P2.02     | PA05      | GPIO10    | GPIO18    | GPIO9

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
