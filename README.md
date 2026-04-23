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
| **RAK WisMesh Tag** | SX1262 | GPS (AT6558R), accelerometer, RGB LEDs, buzzer |
| **ThinkNode M1** | SX1262 | GPS, e-paper display (SSD1681), QSPI flash, buzzer, RGB LEDs |
| **Ikoka Nano 30dBm** | SX1262 (E22-900M30S, 30dBm PA) | RGB LEDs |

### ESP32

| Board | MCU | Radio | Extras |
|-------|-----|-------|--------|
| **XIAO ESP32-C3** | ESP32-C3 | SX1262 | BLE 5.0 |
| **XIAO ESP32-C6** | ESP32-C6 | SX1262 | BLE 5.0, Wi-Fi 6 |
| **Station G2** | ESP32-S3 | SX1262 (+PA) | OLED (SH1106), GPS, 16MB flash, 8MB PSRAM |
| **LilyGo TLoRa C6** | ESP32-C6 | SX1262 | BLE 5.0, Wi-Fi 6 |
| **Heltec V3** | ESP32-S3 | SX1262 | OLED (SSD1306), 8MB flash |
| **Heltec V4.2** | ESP32-S3 | SX1262 (+PA) | OLED (SSD1306), 16MB flash |
| **Heltec V4.3** | ESP32-S3 | SX1262 (+PA) | OLED (SSD1306), 16MB flash |

### Other

| Board | MCU | Radio | Extras |
|-------|-----|-------|--------|
| **XIAO nRF54L15** | nRF54L15 | SX1262 | FLPR multicore, RRAM storage |
| **XIAO MG24** | EFR32MG24 | SX1262 | BLE (SiLabs blob) |

## Device Roles

- **Companion** (default) -- connects to MeshCore mobile apps via BLE
- **Repeater** -- forwards packets, configured via USB serial CLI. See the [Repeater CLI Command Reference](zephcore/Repeater_CLI_commands.md) for all available commands.

## Building

Prerequisites: [Zephyr SDK >=1.0.1 (!)](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) and `west` installed.

Optional: [adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil) to allow DFU zip generation for OTA updates on nRF52 

```bash
# Initialize workspace (first time only)
cd %cloned folder%
west init -l zephcore
west update

# Companion (with logging)
west build -b wio_tracker_l1 zephcore --pristine

# Companion (production, no logging)
west build -b wio_tracker_l1 zephcore --pristine -- \
 -DEXTRA_CONF_FILE="boards/common/prod.conf"

# Repeater (with logging)
west build -b rak4631/nrf52840 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Repeater (production)
west build -b rak4631/nrf52840 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/prod.conf"

# Repeater with packet logging (clean RAW/RX/TX lines only, no debug spam)
west build -b rak4631/nrf52840 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/packet_logging.conf"

# Formatter (with serial logging)
west build -b wio_tracker_l1 zephcore/tools/formatter --pristine

# Companion (BLE debug logging)
west build -b rak4631/nrf52840 zephcore --pristine -- -DCONFIG_ZEPHCORE_BLE_LOG_LEVEL_DBG=y
```

Output binaries are in `build/zephyr/` -- `.hex`, `.uf2`, and DFU `.zip` as applicable.

For exact `west build -b` board strings, flash methods, and special setup (MG24 pyocd, nRF54L15 `--no-sysbuild`), see the [Board Porting Guide](zephcore/boards/example_board/README.md).

Heltec V3 note: console and shell are routed to `uart0` in ZephCore. Use the UART serial port for logs/CLI.

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

2. **EMA-based delay sizing**: Dupe counts feed into a rolling exponential moving average. This drives a sqrt-curve delay factor for future retransmits: near-zero delay in sparse areas, scaling up in dense ones. At ~15 dupes (moderate density), the factor matches the old Arduino default of 0.5.

3. **Reactive per-packet backoff**: When a node is waiting to retransmit and hears a neighbor retransmit the same packet, it pushes its own TX back by a random amount (up to `backoff.multiplier` x airtime). This is real-time CSMA -- you hear the channel being used for your packet, so you defer.

**Direct packets** (routed, single next-hop) use minimal fixed jitter (~0-45ms) instead of adaptive delay, since only the next hop retransmits them.

The old `txdelay`, `rxdelay`, and `direct.txdelay` commands are still accepted for binary compatibility with Arduino prefs but are ignored -- the system is fully adaptive.

**CLI commands:**
- `get txdelay` -- shows adaptive status: contention estimate and current flood delay factor
- `get/set backoff.multiplier` -- reactive backoff cap (default 0.5, range 0.0-2.0). Set to 0 to disable reactive backoff (EMA window still works). Higher values allow more per-packet deferral in dense areas.

**Compatibility**: Purely local behavior, no wire protocol changes. Works alongside Arduino MeshCore repeaters -- their retransmits are counted as dupes just the same.

## Power Saving

- **LoRa RX duty cycle**: CAD-based receive windowing reduces LoRa RX current from ~10-15mA to ~3-5mA (configurable via `CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE`)
- **USB disabled in production**: Saves ~2-5mA and 62KB flash when logging is off
- **GPIO-gated GPS**: Powered on only during fix acquisition

## Configuration

Key Kconfig options (set in board configs or via `-D` flags):

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_ZEPHCORE_ROLE_COMPANION` | y | BLE companion mode |
| `CONFIG_ZEPHCORE_ROLE_REPEATER` | n | USB CLI repeater mode |
| `CONFIG_ZEPHCORE_RADIO_NATIVE` | y | SX126x, SX127x, LLCC68, STM32WL |
| `CONFIG_ZEPHCORE_RADIO_LR1110` | n | LR1110/LR1120/LR1121 (custom driver) |
| `CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE` | auto | CAD-based RX power saving (auto ON for companion+SX1262, OFF for LR1110/repeater) |
| `CONFIG_ZEPHCORE_MAX_CONTACTS` | 350 | Contact storage slots (companion) |
| `CONFIG_ZEPHCORE_MAX_CHANNELS` | 40 | Channel slots (companion) |
| `CONFIG_ZEPHCORE_BLE_PASSKEY` | 123456 | BLE pairing PIN |
| `CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC` | 300 | GPS fix interval (seconds) |
| `CONFIG_ZEPHCORE_WIFI_OTA` | n | WiFi AP + HTTP OTA updates (ESP32 repeaters) |
| `CONFIG_ZEPHCORE_PACKET_LOGGING` | n | Arduino-compatible mesh packet logging |
| `CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS` | 5000 | Periodic maintenance interval |

## Project Structure

```
zephcore/
  src/              Main entry points and core mesh protocol
  app/              Companion and repeater role implementations
  adapters/
    ble/            BLE NUS transport
    board/          GPIO, LED, power management
    clock/          Millisecond and RTC clocks
    datastore/      LittleFS filesystem wrapper
    gps/            GPS/GNSS drivers
    ota/            WiFi OTA firmware updates
    radio/          LoRa radio drivers (SX126x, LR1110)
    rng/            Random number generator
    sensors/        I2C sensor auto-detection
    usb/            USB serial transport (CDC-ACM, V3 framing)
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