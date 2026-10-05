# Building ZephCore

Everything about building from source: setup, every build variant, flashing, per-platform notes and the
Kconfig options worth knowing. The short version is in the [README](../README.md#build-it-yourself).

## Setup

**Dev container (easiest).** Open the repo in VS Code and run **Dev Containers: Reopen in Container**. It
carries the SDK, toolchains, west and the vendor blobs, and matches CI. See
[.devcontainer/README.md](../.devcontainer/README.md).

**By hand.** Install the
[Zephyr SDK >= 1.0.1 and west](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
(the SDK version matters: Zephyr is pinned to a `main` commit in `zephcore/west.yml`, see
[ADR 0005](adr/0005-zephyr-main-pin.md)). Then, in the cloned folder:

```bash
west init -l zephcore
west update
```

Optional: [adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil) on the path makes nRF52
builds also produce the DFU `.zip`. ESP32 and MG24 need their vendor blobs once:

```bash
west blobs fetch hal_espressif
west blobs fetch hal_silabs
```

## Build variants

All builds have the same shape; the role and the extras are config fragments from
`zephcore/boards/common/`, passed in `EXTRA_CONF_FILE` and separated with `;`.

```bash
west build -b <board> zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/<fragment>.conf"
```

| Fragment | Gives you | Notes |
|---|---|---|
| *(none)* | Companion | Production build: no logging, no asserts, reboot on fatal error |
| `repeater.conf` | Repeater | On ESP32-S3/C-series this pulls in `wifi_ota.conf`, so add `--sysbuild` |
| `room_server.conf` | Room server | |
| `observer.conf` | Observer | ESP32 only (WiFi + MQTT) |
| `debug.conf` | Logging and asserts | Combine with any role |
| `packet_logging.conf` | One RAW/RX/TX line per packet | Logging subsystem stays off, so the console carries only these lines and the CLI |
| `repeater_uplink.conf` | Repeater WiFi + MQTT uplink | ESP32, with `repeater.conf` |
| `wifi_ota.conf` | WiFi AP + HTTP firmware update | ESP32 repeaters; needs MCUboot, so `--sysbuild` |
| `wifi_companion.conf` | App over TCP on your WiFi | Added automatically for boards whose manifest declares `wifi: true` |

Board strings are in [supported_boards.md](supported_boards.md). Examples:

```bash
# Companion
west build -b wio_tracker_l1 zephcore --pristine

# Companion with debug logging
west build -b wio_tracker_l1 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/debug.conf"

# Repeater
west build -b rak4631 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Repeater with packet logging
west build -b rak4631 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/packet_logging.conf"

# ESP32-S3 repeater (WiFi OTA comes with it, hence sysbuild)
west build -b xiao_esp32s3/esp32s3/procpu zephcore --pristine --sysbuild -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Room server
west build -b rak4631 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/room_server.conf"

# Observer
west build -b xiao_esp32c3 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/observer.conf"

# BLE adapter at debug level (debug.conf turns logging on, the flag raises BLE to DBG)
west build -b rak4631 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/debug.conf" -DCONFIG_ZEPHCORE_BLE_LOG_LEVEL_DBG=y

# Formatter (factory-reset utility)
west build -b wio_tracker_l1 zephcore/tools/formatter --pristine
```

**Always use `--pristine` when you switch board or role.** Outputs land in `build/zephyr/`: `.uf2`, `.hex`,
`.bin` and the DFU `.zip`, as the platform allows.

The release matrix is `build.sh` (`./build.sh nrf`, `./build.sh esp32 companions`, ...). Which boards it
builds comes from the board manifests (`zephcore/boards/**/zephcore.yml`), and it adds no `-D` flags of its
own, so a local build of the same board and role is the same firmware.

## Flashing and platform notes

| Platform | Flash | Worth knowing |
|---|---|---|
| nRF52840 | Copy `zephyr.uf2` to the UF2 drive, or `west flash` | Keeps the Adafruit bootloader and SoftDevice layout of stock MeshCore |
| ESP32 companion | `west flash` (image at 0x0) | Simple boot, no MCUboot ([ADR 0001](adr/0001-esp32-boot-and-flash-layout.md)) |
| ESP32-S3 companion | esptool on the ROM's USB-Serial-JTAG port | Native USB: esptool auto-reset does not work, enter the bootloader with `start dfu` or a 1200-baud touch ([ADR 0002](adr/0002-esp32s3-native-usb-companion.md)) |
| ESP32 repeater (S3, C-series) | `west flash` after a `--sysbuild` build | MCUboot image; without `--sysbuild` the board silently stops enumerating USB |
| nRF54L | `zephyr.hex` over SWD | Build with `--no-sysbuild`; no USB peripheral, so no UF2 |
| EFR32MG24 | `west flash --runner pyocd` | Needs the `hal_silabs` blobs |
| STM32WL | `west flash` over SWD / ST-Link | No BLE or USB device: companion protocol and CLI run on USART1 |
| Native Linux | Copy the binary to the SBC | See [LINUX_NATIVE.md](LINUX_NATIVE.md) |

Released ESP32 images are always MCUboot `-merged.bin`, written at offset 0. Heltec V3 and the classic
ESP32 boards have their console and CLI on `uart0` (the USB-UART port).

## How the configuration is put together

`zephcore/CMakeLists.txt` derives the platform from the board string and layers the Kconfig fragments; a
later layer overrides an earlier one:

```
prj.conf → boards/common/zephcore_common.conf → boards/common/<platform>_common.conf
  → boards/common/esp32c3s3_common.conf (S3/C3 only) → boards/<platform>/<board>/board.conf
  → your EXTRA_CONF_FILE
```

Devicetree follows the same rule: `board.overlay` first, overlays paired with a conf fragment after it,
`partitions.overlay` last ([ADR 0003](adr/0003-devicetree-overlay-precedence.md)). Most hardware features
(display, buttons, buzzer, GPS, PSRAM) are detected from devicetree and need no Kconfig. Zephyr patches
from `zephcore/patches/` are applied at configure time and abort the build if they do not apply.

## Kconfig options worth knowing

Set them in a board conf or with `-DCONFIG_...=`. The full list with help texts is `zephcore/Kconfig`.

| Option | Default | Meaning |
|---|---|---|
| `ZEPHCORE_ROLE_*` | `COMPANION` | Role choice: `COMPANION`, `REPEATER`, `ROOM_SERVER`, `OBSERVER`. Normally set by the role fragment |
| `ZEPHCORE_RADIO_*` | `NATIVE` | Radio choice: `NATIVE` (Zephyr SX126x driver, LLCC68, STM32WL), `LR1110` (LR1110/1120/1121), `LR2021`, `SX127X`. Set by the board |
| `ZEPHCORE_DEFAULT_TX_POWER_DBM` | 22 | Initial TX power; lower on boards with an external PA |
| `ZEPHCORE_MAX_TX_POWER_DBM` | 22 | Hard cap, enforced by the radio adapter |
| `ZEPHCORE_LORA_RX_DUTY_CYCLE` | n | Boot default for the RX duty cycle; the runtime setting is `set rxduty` |
| `ZEPHCORE_MAX_CONTACTS` | 350 | Companion contact slots (boards short on RAM lower it) |
| `ZEPHCORE_MAX_CHANNELS` | 40 | Companion channel slots |
| `ZEPHCORE_OFFLINE_QUEUE_SIZE` | 256 | Companion offline message queue |
| `ZEPHCORE_BLE_PASSKEY` | 123456 | BLE pairing PIN |
| `ZEPHCORE_GPS_POLL_INTERVAL_SEC` | 300 | Companion GPS interval between fixes (10 to 86400); the runtime setting is `set gps duty` |
| `ZEPHCORE_GPS_FIRST_FIX_TIMEOUT_SEC` | 300 | Window for a fix after a full GPS power-off: boot, `gps on`, and every wake when the GPS interval is above `gps standby` |
| `ZEPHCORE_REPEATER_GPS_INTERVAL_SEC` | 172800 | Repeater and room server GPS interval (48 h); 0 = always on |
| `ZEPHCORE_PACKET_LOGGING` | n | Upstream-format packet log lines (`packet_logging.conf`) |
| `ZEPHCORE_WIFI_OTA` | n | WiFi AP + HTTP firmware update (ESP32 repeaters) |
| `ZEPHCORE_REPEATER_UPLINK` | n | Repeater WiFi + MQTT uplink (ESP32) |
| `ZEPHCORE_HOUSEKEEPING_INTERVAL_MS` | 5000 | Companion housekeeping tick |

## Tests

Host regression tests live in `tests/` and run in CI under ASan and UBSan. Locally (Linux or WSL):

```bash
python3 tests/run.py run --profile quick
```
