# ZephCore — MeshCore for Zephyr RTOS

A port of [MeshCore](https://github.com/meshcore-dev/MeshCore/) LoRa mesh firmware from Arduino to [Zephyr RTOS](https://zephyrproject.org/). Aiming for full protocol compatibility with the original Arduino firmware and the MeshCore mobile apps.

## Why Zephyr?

The Arduino version uses a `loop()`. This port replaces that with Zephyr's event-driven primitives (`k_event_wait`, `k_poll`, `k_msgq`), so the CPU sleeps in WFI (Wait For Interrupt) between events.

Other benefits:

- **Proper driver model** -- LoRa, GNSS, display, sensors, and BLE all use Zephyr subsystem drivers rather than Arduino libraries
- **Hierarchical build configuration** -- board-specific settings compose cleanly via Kconfig and devicetree overlays
- **DFU support** -- generates Arduino compatible zip packages for OTA updates and UF2 binaries for drag-and-drop flashing
- **Back and forth compatible** -- Adapted to softdevice and adafruit's bootloader, so no bootloader re-flashing required.

## Supported Boards

### nRF52840

| Board | Radio | Extras |
|-------|-------|--------|
| **Wio Tracker L1** | SX1262 | GPS (L76KB), OLED (SH1106), joystick, buzzer, QSPI flash |
| **Seeed T1000-E** | LR1110 | GPS (AG3335), LEDs, button |
| **RAK4631** / **RAK WisMesh Pocket** | SX1262 | Same `rak4631` build. GPS (u-blox MAX-7Q), optional WisBlock OLED (SSD1306), I2C sensors (SHTC3, LPS22HB, BME680) |
| **RAK3401 1W** | SX1262 + SKY66122 (30 dBm) | GPS (u-blox MAX-7Q, optional), I2C sensors |
| **RAK WisMesh Tag** | SX1262 | GPS (AT6558R), accelerometer, buzzer |
| **ThinkNode M1** | SX1262 | GPS, e-paper display (SSD1681), QSPI flash, buzzer, RGB LEDs |
| **ThinkNode M3** | LR1110 | GPS, buzzer, two buttons, RGB LEDs |
| **ThinkNode M6** | SX1262 | GPS (L76K), QSPI flash, RGB LEDs |
| **LilyGo T-Echo** | SX1262 (TCXO 1.8V) | GPS (L76K), 1.54" e-paper (SSD1681), BME280, QSPI flash, touch-button backlight |
| **Ikoka Nano 30dBm** | SX1262 (E22-900M30S, 30 dBm PA) | RGB LEDs |
| **SenseCAP Solar** | SX1262 | GPS (L76K), QSPI flash, battery monitor |
| **XIAO nRF52840 + Wio-SX1262** | SX1262 | Bare XIAO + Wio-SX1262 expansion |

### ESP32

| Board | MCU | Radio | Extras |
|-------|-----|-------|--------|
| **XIAO ESP32-C3** | ESP32-C3 | SX1262 | BLE 5.0 |
| **XIAO ESP32-C6** | ESP32-C6 | SX1262 | BLE 5.0, Wi-Fi 6 |
| **XIAO ESP32-S3** | ESP32-S3 | SX1262 | BLE 5.0, 8MB flash, 8MB PSRAM |
| **Station G2** | ESP32-S3 | SX1262 + PA (~20 dB gain) | OLED (SH1106), GPS, 16MB flash, 8MB PSRAM |
| **LilyGo TLoRa C6** | ESP32-C6 | SX1262 | BLE 5.0, Wi-Fi 6 |
| **Heltec V3** | ESP32-S3 | SX1262 | OLED (SSD1306), 8MB flash |
| **Heltec V4.2** | ESP32-S3 | SX1262 + GC1109 PA | OLED (SSD1306), 16MB flash, 2MB PSRAM |
| **Heltec V4.3** | ESP32-S3 | SX1262 + KCT8103L PA | OLED (SSD1306), 16MB flash, 2MB PSRAM |

### Other

| Board | MCU | Radio | Extras |
|-------|-----|-------|--------|
| **XIAO nRF54L15 + Wio-SX1262** | nRF54L15 | SX1262 | FLPR multicore, RRAM storage |
| **XIAO MG24 + Wio-SX1262** | EFR32MG24 | SX1262 | BLE (SiLabs blob) |

For exact `west build -b` board strings, flash methods, and special setup, see the [supported boards list](zephcore/boards/supported_boards.md) and the [Board Porting Guide](zephcore/boards/example_board/README.md).

## Device Roles

- **Companion** (default) -- connects to MeshCore mobile apps via BLE. Contacts, channels, offline message queue.
- **Repeater** -- forwards packets, configured via USB serial CLI. See the [Repeater CLI Command Reference](zephcore/Repeater_CLI_commands.md) for all available commands.
- **Observer** (ESP32 only) -- listen-only node that publishes received LoRa packets to MQTT over WiFi STA. Configured at runtime via serial CLI.

## Building

Prerequisites: [Zephyr SDK >=1.0.1 (!)](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) and `west` installed (required by Zephyr 4.4.0, which is pinned in `west.yml`).

Optional: [adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil) to allow DFU zip generation for OTA updates on nRF52.

```bash
# Initialize workspace (first time only)
cd <cloned folder>
west init -l zephcore
west update

# Companion (with logging)
west build -b wio_tracker_l1 zephcore --pristine

# Companion (production, no logging, no USB)
west build -b wio_tracker_l1 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/prod.conf"

# Repeater (with logging)
west build -b rak4631 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Repeater (production)
west build -b rak4631 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/prod.conf"

# Repeater with packet logging (clean RAW/RX/TX lines only, no debug spam)
west build -b rak4631 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/packet_logging.conf"

# ESP32 repeater + WiFi AP HTTP OTA (requires sysbuild for MCUboot)
west build -b xiao_esp32s3/esp32s3/procpu zephcore --pristine --sysbuild -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/wifi_ota.conf"

# Observer (ESP32, listen-only WiFi+MQTT)
west build -b xiao_esp32c3 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/observer.conf"

# Formatter (factory-reset utility)
west build -b wio_tracker_l1 zephcore/tools/formatter --pristine

# BLE debug logging overlay
west build -b rak4631 zephcore --pristine -- -DCONFIG_ZEPHCORE_BLE_LOG_LEVEL_DBG=y
```

Output binaries are in `build/zephyr/` -- `.hex`, `.uf2`, and DFU `.zip` as applicable.

**Platform notes:**
- ESP32 boards require `west blobs fetch hal_espressif` once before the first build.
- ESP32 default builds use `CONFIG_ESP_SIMPLE_BOOT` (no MCUboot) -- `west flash` writes a self-contained `zephyr.bin`. The `wifi_ota.conf` overlay is the exception: it requires MCUboot, so add `--sysbuild`.
- MG24 requires `west blobs fetch hal_silabs` and `pyocd` (`west flash --runner pyocd`).
- nRF54L15 requires `--no-sysbuild` (no MCUboot support yet).
- Heltec V3 routes console/shell to `uart0` -- use the UART serial port for logs/CLI.

Always use `--pristine` when switching boards or roles.

## Architecture Overview

```
Mobile App  <--BLE (NUS)--> [ Companion ]  <--LoRa-->  Mesh Network
                                  |
                            k_event_wait()
                           /      |       \
                    LORA_RX   LORA_TX_DONE  BLE_RX
```

All code paths are event-driven. The CPU sleeps in WFI between events.

- **LoRa RX**: Zephyr driver callback enqueues to a ring buffer and signals the mesh event loop
- **LoRa TX**: A dedicated thread blocks on `k_poll()`, restarts RX on completion, then notifies the mesh loop
- **BLE**: NUS write handler enqueues to `k_msgq` and signals the mesh loop; TX uses `bt_gatt_notify_cb()` chaining
- **USB**: CDC-ACM with V3 binary framing protocol, frame timeout recovery
- **Main loop**: `k_event_wait()` blocks until work arrives; housekeeping runs every 5s

### Key Differences from Arduino

| | Arduino | Zephyr |
|---|---------|--------|
| Idle behavior | Cooperative loop; CPU busy-waits unless `board.sleep()` called explicitly | `k_event_wait(K_FOREVER)` yields to idle thread → WFI between events |
| LoRa TX completion | ISR sets flag, polled in `loop()` via `isSendComplete()` | ISR signals `k_poll_signal`, dedicated thread blocks on `k_poll()` |
| BLE transport | Platform-specific (ESP-IDF BLE, Adafruit nRF52 lib) | Unified `bt_gatt` API across all SoCs |
| LoRa driver | RadioLib (userspace SPI bit-bang) | Zephyr subsystem driver (DTS-configured, kernel-managed SPI) |
| Configuration | `platformio.ini` + `variant.h` per board | Kconfig + devicetree overlays, hierarchical config inheritance |
| Threading | Single `loop()` + ISRs | Explicit threads (main mesh, TX wait) + system work queue |

### Adaptive Contention Window (ZephCore-only)

Arduino MeshCore uses three static delay knobs (`txdelay`, `rxdelay`, `direct.txdelay`) that add the same retransmit jitter regardless of local conditions. In a linear chain of repeaters where each only hears its neighbor, this adds latency for zero benefit. In dense areas with 50+ neighbors, the same value may be too low to avoid collisions.

ZephCore replaces all three with a self-tuning system based on **observed retransmit contention**:

1. **Dupe counting**: When a node retransmits a flood packet, it counts how many times it hears that same packet retransmitted by neighbors within a 10-second window. This is a direct measurement of local contention -- 0 dupes means a quiet linear chain, 15+ means a dense cluster.

2. **EMA-based delay sizing**: Dupe counts feed into a rolling exponential moving average. This drives a sqrt-curve delay factor for future retransmits: near-zero delay in sparse areas, scaling up in dense ones. At ~15 dupes (moderate density), the factor matches the old Arduino default of 0.5. Jitter is double-capped at `min(2000ms, 6·airtime)` for repeaters.

3. **Reactive per-packet backoff**: When a node is waiting to retransmit and hears a neighbor retransmit the same packet, it pushes its own TX back by a random amount (up to `backoff.multiplier × airtime`). This is real-time CSMA -- you hear the channel being used for your packet, so you defer. Capped at `min(2000ms, 12·airtime)` total reactive extension per packet.

**Companion-originated floods** use a smaller spread (up to `min(1000ms, 3·airtime)`) so user messages feel responsive while still avoiding collisions with nearby repeaters that may be in TX/RX.

**Direct packets** (routed, single next-hop) use minimal fixed jitter instead of adaptive delay, since only the next hop retransmits them.

The old `txdelay`, `rxdelay`, and `direct.txdelay` commands are still accepted for binary compatibility with Arduino prefs but are ignored -- the system is fully adaptive.

**CLI commands:**
- `get txdelay` -- shows adaptive status: contention estimate and current flood delay factor
- `get/set backoff.multiplier` -- reactive backoff cap (default 0.5, range 0.0-2.0). Set to 0 to disable reactive backoff (EMA window still works). Higher values allow more per-packet deferral in dense areas.

**Compatibility**: Purely local behavior, no wire protocol changes. Works alongside Arduino MeshCore repeaters -- their retransmits are counted as dupes just the same.

## Power Saving

- **LoRa RX duty cycle**: CAD-based receive windowing reduces LoRa RX current from ~10-15mA to ~3-5mA. Auto-enabled for SX1262 companion and repeater builds (toggleable at runtime via `set rxduty on/off`). Disabled for LR1110 (mid-preamble lock issue) and SX127x.
- **Adaptive Power Control (APC)**: compiled in by default but disabled at runtime. Enable per-node with `set tx apc` -- automatically reduces TX power when echo SNR shows excess margin, ramping back up if echoes drop. See [apc.md](zephcore/apc.md).
- **USB disabled in production**: Saves ~2-5mA and 62KB flash when logging is off
- **GPIO-gated GPS**: Powered on only during fix acquisition

## Configuration

Key Kconfig options (set in board configs or via `-D` flags):

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_ZEPHCORE_ROLE_COMPANION` | y | BLE companion mode |
| `CONFIG_ZEPHCORE_ROLE_REPEATER` | n | USB CLI repeater mode |
| `CONFIG_ZEPHCORE_ROLE_OBSERVER` | n | Listen-only WiFi+MQTT mode (ESP32) |
| `CONFIG_ZEPHCORE_RADIO_NATIVE` | y | SX1261/SX1262/SX1268, LLCC68, STM32WL (Zephyr native sx126x driver) |
| `CONFIG_ZEPHCORE_RADIO_LR1110` | n | LR1110/LR1120/LR1121 (custom driver) |
| `CONFIG_ZEPHCORE_RADIO_LR2021` | n | LR2021 (custom driver) |
| `CONFIG_ZEPHCORE_RADIO_SX127X` | n | SX1272/SX1276/SX1278 (loramac-node backend) |
| `CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE` | auto | CAD-based RX power saving (auto ON for companion+SX1262, OFF for LR1110/SX127x) |
| `CONFIG_ZEPHCORE_APC` | y (compiled in, runtime OFF) | Adaptive Power Control — enable at runtime via CLI |
| `CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM` | 22 | Initial TX power; lower for boards with external PA |
| `CONFIG_ZEPHCORE_MAX_TX_POWER_DBM` | 22 | Hard cap (radio adapter clamps above this) |
| `CONFIG_ZEPHCORE_MAX_CONTACTS` | 350 | Contact storage slots (companion) |
| `CONFIG_ZEPHCORE_MAX_CHANNELS` | 40 | Channel slots (companion) |
| `CONFIG_ZEPHCORE_BLE_PASSKEY` | 123456 | BLE pairing PIN |
| `CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC` | 300 | GPS fix interval (seconds) |
| `CONFIG_ZEPHCORE_WIFI_OTA` | n | WiFi AP + HTTP OTA updates (ESP32 repeaters, requires `--sysbuild`) |
| `CONFIG_ZEPHCORE_REPEATER_UPLINK` | n | Repeater WiFi+MQTT uplink (ESP32) |
| `CONFIG_ZEPHCORE_PACKET_LOGGING` | n | Arduino-compatible mesh packet logging |
| `CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS` | 5000 | Periodic maintenance interval |

## Project Structure

```
zephcore/
  src/              Core mesh engine (Mesh, Dispatcher, Packet, Identity, ContentionTracker)
  app/              Companion / Repeater / Observer role implementations
  adapters/
    ble/            BLE NUS transport
    board/          GPIO, LED, power management
    clock/          Millisecond and RTC clocks
    datastore/      LittleFS filesystem wrapper
    gps/            GPS/GNSS drivers
    mqtt/           MQTT publisher (observer / repeater uplink)
    ota/            WiFi AP + HTTP firmware update server
    radio/          LoRa radio drivers (SX126x, LR1110, LR2021, SX127x)
    rng/            Random number generator
    sensors/        I2C sensor auto-detection
    usb/            USB serial transport (CDC-ACM, V3 framing)
    wifi/           WiFi station client
  helpers/
    ui/             Display, buzzer, button input
  boards/
    nrf52840/       nRF52840 board overlays and configs
    esp32/          ESP32-C3/C6/S3 board overlays and configs
    nrf54l/         nRF54L15 board overlay and config
    mg24/           EFR32MG24 board overlay and config
    common/         Shared Kconfig fragments and devicetree includes
  lib/              ED25519 crypto library
  patches/          Auto-applied patches to the Zephyr tree
```

## License

Same license as the upstream MeshCore project.

![aXa0YNLq_700w_0](https://github.com/user-attachments/assets/ddce17fd-7b83-4dc7-999f-0519593fcc3d)

(FYI this is sarcasm, the whole project is 99,9% claude and cursor backed, relying heavily on the [official meshcore repo](https://github.com/meshcore-dev/MeshCore/) and the work they do in it)
