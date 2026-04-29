# ZephCore Architecture Guide

> Comprehensive developer reference for the ZephCore codebase — a Zephyr RTOS port of the Arduino MeshCore LoRa mesh networking firmware.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Directory Structure](#2-directory-structure)
3. [Layer Architecture](#3-layer-architecture)
4. [Core Mesh Engine](#4-core-mesh-engine)
5. [Radio Subsystem](#5-radio-subsystem)
6. [Application Layer](#6-application-layer)
7. [Hardware Adapters](#7-hardware-adapters)
8. [UI Subsystem](#8-ui-subsystem)
9. [Build System](#9-build-system)
10. [Board Matrix](#10-board-matrix)
11. [Packet Format Reference](#11-packet-format-reference)
12. [BLE Protocol Reference](#12-ble-protocol-reference)
13. [Data Storage](#13-data-storage)
14. [Key Call Flows](#14-key-call-flows)

---

## 1. Project Overview

ZephCore is a LoRa mesh networking firmware running on Zephyr RTOS. It supports two device roles:

- **Companion**: BLE-connected device paired with a phone app. Full contact/channel/message management.
- **Repeater**: Autonomous headless relay node. CLI administration via authenticated mesh connections or serial UART.

Supported hardware: nRF52840, nRF54L15, ESP32-C3/C6/S3, EFR32MG24 — all with SX1262 or LR1110 LoRa radios.

### Upstream Relationship

ZephCore is a port of [Arduino MeshCore](https://github.com/rmendes76/MeshCore). The core mesh protocol (Mesh.cpp, Dispatcher.cpp, Packet.cpp, Identity.cpp, Utils.cpp) is shared code. Adapters (`adapters/`) bridge MeshCore's HAL interfaces to Zephyr APIs. Binary file formats (prefs, contacts, channels) are byte-compatible with Arduino MeshCore.

---

## 2. Directory Structure

```
zephcore/
├── src/                    # Core mesh engine (shared with Arduino MeshCore)
│   ├── Mesh.cpp            # Routing protocol: flood, direct, dedup, adverts
│   ├── Dispatcher.cpp      # Packet queue, radio scheduling, CAD, duty cycle
│   ├── Packet.cpp          # Packet serialization, hash, wire format
│   ├── Identity.cpp        # Ed25519 key management, ECDH shared secrets
│   ├── Utils.cpp           # AES-ECB encrypt, HMAC-SHA256, MAC
│   ├── StaticPoolPacketManager.cpp  # Fixed-size packet pool (32 slots)
│   ├── main_companion.cpp  # Companion mode entry point + event loop
│   └── main_repeater.cpp   # Repeater mode entry point + event loop
│
├── include/mesh/           # Core interfaces (shared with Arduino MeshCore)
│   ├── Mesh.h, Dispatcher.h, Packet.h, Identity.h, Utils.h
│   ├── MeshCore.h          # Constants: key sizes, packet limits
│   ├── Radio.h             # Abstract radio interface
│   ├── Board.h, Clock.h, RNG.h, RTC.h  # HAL interfaces
│   ├── LoRaConfig.h        # Default radio parameters
│   ├── RadioIncludes.h     # Compile-time radio driver selection
│   ├── SimpleMeshTables.h  # Hash-based packet deduplication
│   └── StaticPoolPacketManager.h  # Fixed pool allocator
│
├── adapters/               # Zephyr HAL implementations
│   ├── radio/              # LoRa radio drivers
│   │   ├── LoRaRadioBase.cpp/h    # Shared TX/RX state machine, noise floor, AGC
│   │   ├── SX126xRadio.cpp/h      # SX1262 adapter (native Zephyr driver)
│   │   ├── LR1110Radio.cpp/h      # LR1110 adapter (patched Zephyr driver)
│   │   ├── radio_common.h         # Shared radio types and constants
│   │   └── lr11xx/                # LR11xx low-level HAL (SPI, GPIO, Semtech SDK)
│   ├── ble/ZephyrBLE.cpp/h        # BLE NUS service, pairing, TX congestion
│   ├── board/ZephyrBoard.cpp/h    # Battery ADC, LEDs, reboot, bootloader
│   ├── clock/                     # Millisecond uptime + software RTC
│   ├── datastore/ZephyrDataStore.cpp/h  # LittleFS persistence
│   ├── gps/ZephyrGPSManager.cpp/h      # GNSS state machine, power mgmt
│   ├── ota/wifi_ota.c/h           # WiFi SoftAP + HTTP firmware upload
│   ├── rng/ZephyrRNG.cpp/h        # Hardware CSPRNG with PRNG fallback
│   ├── sensors/                   # I2C env sensors + power monitors
│   └── usb/                       # USB CDC for companion + repeater
│
├── app/                    # Application layer
│   ├── CompanionMesh.cpp/h       # Phone-connected companion logic
│   ├── RepeaterMesh.cpp/h        # Autonomous repeater logic
│   └── RepeaterDataStore.cpp/h   # Repeater-specific persistence paths
│
├── helpers/                # Shared utilities
│   ├── BaseChatMesh.cpp/h        # Contact/channel/message base class
│   ├── CommonCLI.cpp/h           # Serial/mesh CLI command processor
│   ├── AdvertDataHelpers.cpp/h   # Advertisement wire format encoder/decoder
│   ├── ClientACL.cpp/h           # Authenticated client management
│   ├── TransportKeyStore.cpp/h   # Region transport key cache
│   ├── RegionMap.cpp/h           # Region-based flood filtering
│   ├── ContactInfo.h, ChannelDetails.h, NodePrefs.h  # Data structures
│   ├── RateLimiter.h, IdentityStore.h, StatsFormatHelper.h
│   └── ui/                       # Display, buzzer, input, pages, Doom game
│
├── boards/                 # Board definitions
│   ├── common/             # Shared configs, DTS includes, partition layouts
│   ├── nrf52840/           # RAK4631, WisMesh Tag, T1000-E, ThinkNode M1, etc.
│   ├── nrf54l/             # XIAO nRF54L15
│   ├── esp32/              # LilyGo TLoRa C6, Station G2, XIAO ESP32-C3/C6
│   └── mg24/               # XIAO MG24
│
├── patches/                # Zephyr tree modifications
│   ├── zephyr/             # Unified diffs (SX126x extensions, GNSS, blobs)
│   └── zephyr-new/         # New files (LR11xx Zephyr driver, DTS bindings)
│
├── lib/ed25519/            # Vendored Ed25519 crypto library
├── tools/                  # Formatter (flash erase) + LR1110 firmware updater
├── CMakeLists.txt          # Build orchestration
├── Kconfig                 # All ZephCore configuration options
├── prj.conf                # Base project config
└── west.yml                # West manifest (Zephyr version pin)
```

---

## 3. Layer Architecture

```
┌─────────────────────────────────────────────────┐
│  Phone App (BLE NUS)  or  Serial CLI (USB CDC)  │  External
├─────────────────────────────────────────────────┤
│  CompanionMesh / RepeaterMesh                   │  App Layer
│    ├── BaseChatMesh (contacts, channels, msgs)  │
│    ├── CommonCLI (command processor)            │
│    ├── ClientACL, RegionMap, TransportKeyStore  │
│    └── UI (display, buzzer, buttons)            │
├─────────────────────────────────────────────────┤
│  mesh::Mesh                                     │  Routing
│    ├── Flood routing (path hash accumulation)   │
│    ├── Direct routing (source-routed paths)     │
│    ├── Packet dedup (SimpleMeshTables)          │
│    └── Advert / ACK / Trace / Group dispatch    │
├─────────────────────────────────────────────────┤
│  mesh::Dispatcher                               │  Scheduling
│    ├── TX/RX queue management                   │
│    ├── CAD (channel activity detection)         │
│    ├── Duty cycle enforcement (EU ETSI)         │
│    ├── RX delay (score-based prioritization)    │
│    └── Maintenance (noise floor, AGC reset)     │
├─────────────────────────────────────────────────┤
│  LoRaRadioBase                                  │  Radio HAL
│    ├── SX126xRadio  ──► Zephyr SX126x driver   │
│    └── LR1110Radio  ──► Custom LR11xx driver    │
├─────────────────────────────────────────────────┤
│  Zephyr RTOS (kernel, drivers, BLE, FS, USB)    │  Platform
└─────────────────────────────────────────────────┘
```

---

## 4. Core Mesh Engine

### 4.1 Packet Lifecycle

1. **Allocation**: `StaticPoolPacketManager::allocNew()` — fixed pool of 32 `Packet` objects (no heap)
2. **Creation**: `Mesh::createDatagram()`, `createAdvert()`, `createAck()`, etc.
3. **Queuing**: `Dispatcher::sendPacket()` → `PacketManager::queueOutbound()` with priority + scheduled time
4. **Transmission**: `Dispatcher::checkSend()` → CAD check → serialize → `radio->startSendRaw()`
5. **Release**: `PacketManager::free()` after TX complete or processing done

### 4.2 Packet Structure

```
Wire format:
  [header: 1B] [transport_codes: 0 or 4B] [path_len: 1B] [path: variable] [payload: variable]

Header byte:
  Bits 0-1: Route type (0=transport_flood, 1=flood, 2=direct, 3=transport_direct)
  Bits 2-5: Payload type (0=REQ .. 15=RAW_CUSTOM)
  Bits 6-7: Version (0=v1)

Path_len byte:
  Bits 0-5: Hash count (0-63 hops)
  Bits 6-7: Hash size mode (0=1B, 1=2B, 2=3B, 3=reserved)
```

### 4.3 Payload Types

| Type | Value | Description |
|------|-------|-------------|
| REQ | 0x00 | Encrypted request to peer |
| RESPONSE | 0x01 | Encrypted response from peer |
| TXT_MSG | 0x02 | Encrypted text message |
| ACK | 0x03 | 4-byte CRC acknowledgment |
| ADVERT | 0x04 | Signed identity advertisement |
| GRP_TXT | 0x05 | Group channel text message |
| GRP_DATA | 0x06 | Group channel data |
| ANON_REQ | 0x07 | Anonymous request (includes full pubkey) |
| PATH | 0x08 | Path return (source route exchange) |
| TRACE | 0x09 | Trace route |
| MULTIPART | 0x0A | Multi-ACK container |
| CONTROL | 0x0B | Control data (zero-hop) |
| RAW_CUSTOM | 0x0F | Raw custom data |

### 4.4 Routing

**Flood routing**: Packet has no destination path. Each relay node appends its identity hash to `path[]` and retransmits. Priority decreases with hop count. `allowPacketForward()` is the gatekeeper.

**Direct routing**: Packet carries a source-routed `path[]`. Each relay node checks if the first path hash matches its own identity, removes itself, and forwards. Path is built from previous flood packets' accumulated hashes.

**Deduplication**: `SimpleMeshTables` maintains a circular buffer of 128 packet hashes (8 bytes each, SHA-256 truncated) and 64 ACK CRCs. `hasSeen()` prevents duplicate processing and retransmission.

### 4.5 Dispatcher Scheduling

The Dispatcher runs a tight loop:

```
loop():
  1. Check if current TX is complete → release packet, record airtime
  2. Process next inbound packet from queue (if scheduled time has passed)
  3. checkRecv(): Drain radio RX ring buffer
     - Parse raw bytes into Packet
     - Flood packets: compute RX delay based on score → defer or process immediately
     - Direct packets: process immediately
  4. checkSend(): Check outbound queue
     - CAD: if channel busy, retry every 100-200ms (jittered) up to 4s total
     - Duty cycle: if exceeded, defer 5 seconds (admin packets exempt)
     - Final LBT check right before TX (closes timing gap)
     - Serialize and transmit
```

**RX Delay**: Flood packets are delayed based on signal quality. High-quality signals (high SNR, short packets) get shorter delays, allowing closer/better relays to retransmit first. Uses a lookup table approximation of `10^(0.85 - score*0.1) - 1` multiplied by airtime.

**Duty Cycle**: Fixed 1-hour sliding window. Default 10%. Admin packets (REQ, RESPONSE, ANON_REQ, CONTROL) are exempt.

### 4.6 Maintenance Loop

Called every ~5 seconds from the main event loop:

1. **Noise floor calibration**: EMA with alpha=1/8, jitter, threshold filtering, warmup
2. **RX mode watchdog**: Flags error if radio stuck outside RX for >8 seconds
3. **AGC reset**: Periodic warm sleep + recalibration (configurable interval, default off)

### 4.7 Adaptive Contention Window

Replaces Arduino MeshCore's static `txdelay`/`rxdelay` with three complementary mechanisms.

**EMA Delay Factor (proactive)**

`ContentionTracker` measures observed duplicates per retransmitted packet using a **24-entry ring buffer** (sized for ~50-neighbor hilltop topologies with multiple concurrent in-flight floods). Each entry tracks a packet (identified by FNV-1a hash) and records how many dupes arrive within a 10-second observation window. When the window closes, the entry is finalized and an EMA is updated with alpha = 1/8. The resulting estimate feeds the delay factor formula:

```
factor = 0.05 + 0.170 * sqrt(est)
```

Capped at 2.0. During warmup (fewer than 4 finalized entries), factor defaults to 0.5. Sparse nodes converge toward near-zero delay; dense nodes get proportionally higher delay.

The flood retransmit jitter window is `5·airtime·factor` clamped by **two ceilings**:
- Airtime-scaled: `6·airtime` — keeps SF7/narrow-BW configs from wasting time in oversized windows.
- Absolute: `2000ms` — bounds per-hop latency in dense areas even when airtime is large.

**Per-Dupe Reactive Backoff**

When a duplicate of a pending outbound packet is heard, TX is rescheduled to `now + backoff_multiplier * airtime`. Each dupe triggers a full delay (not diminishing). Cumulative reactive extension is capped at `min(2000ms, 12·airtime)` per packet; after the cap, CAD handles remaining channel activity. `backoff_multiplier` is configurable via `set backoff.multiplier X` (range 0.0–2.0).

**Initial-Flood Jitter (companion-only)**

Companions don't retransmit floods, but they observe mesh contention and need to spread their *originated* transmissions to avoid colliding with repeaters still busy in TX/RX. `Mesh::passivelyTrackFloods()` (overridden to `true` on `CompanionMesh`) registers every first-hearing of a flood with the ContentionTracker, so the EMA warms up even without forwarding. `Mesh::getInitialFloodJitter(packet)` is added to the caller-supplied delay in both `sendFlood` overloads; on companion this is `rand(0, min(1000ms, 3·airtime, 5·airtime·factor))` — half the repeater's ceilings. Repeaters keep the default 0 (no double-jitter on forwards).

**Direct Packets**

Direct (source-routed) packets bypass adaptive scaling entirely. They use minimal fixed jitter: `20 + rand(0, airtime / 10)` ms.

**CLI**

- `get txdelay` — shows current adaptive state (EMA estimate, delay factor, backoff multiplier).
- `set backoff.multiplier X` — controls per-dupe reactive delay (0.0–2.0).
- `txdelay`, `rxdelay`, `direct.txdelay` — accepted for prefs compatibility but ignored at runtime.

**ContentionTracker Resource Usage**

~260 bytes RAM (24-entry ring buffer × ~16B/entry + state). FNV-1a packet hash, 10-second observation window, EMA with alpha = 1/8.

### 4.8 Encryption

- **Peer-to-peer**: ECDH shared secret (Curve25519) → AES-128-ECB encrypt → 2-byte HMAC-SHA256 MAC
- **Group channels**: SHA-256 of channel name → AES key
- **Advertisements**: Ed25519 signature over (pubkey + timestamp + app_data)
- **ACKs**: SHA-256(shared_secret + packet_hash) truncated to 4 bytes

---

## 5. Radio Subsystem

### 5.1 Class Hierarchy

```
mesh::Radio (abstract interface)
  └── LoRaRadioBase (shared state machine, ring buffer, noise floor)
        ├── SX126xRadio → Zephyr native SX126x driver + sx126x_ext.h
        └── LR1110Radio → Custom lr11xx_lora.c driver + Semtech HAL
```

Compile-time selection via `CONFIG_ZEPHCORE_RADIO_LR1110` in `RadioIncludes.h`.

### 5.2 LoRaRadioBase State Machine

**TX Flow**:
1. `startSendRaw()` → cancel RX → configure TX → copy to buffer → async send → wake TX wait thread
2. TX wait thread blocks on semaphore, polls completion signal (5s timeout)
3. On DIO1 TX_DONE interrupt → signal raised → restart RX → update stats

**RX Flow**:
1. `lora_recv_async()` with callback
2. ISR writes to 8-slot SPSC ring buffer (drops NEW packet on overflow)
3. Main thread drains via `recvRaw()`

**Config Caching**: Avoids redundant `lora_config()` calls. Fast-path for TX↔RX transitions when only direction differs.

### 5.3 Noise Floor EMA

Algorithm in `triggerNoiseFloorCalibrate()`:
- 8 RSSI samples per tick, take median (insertion-sort midpoint)
- Threshold filter: reject samples ≥ floor + 14dB (after 8-tick warmup)
- Periodic bypass: every 16th tick accepts unconditionally
- EMA: `floor += round_nearest((sample - floor) / 8)`, clamped to [-120, -50] dBm

### 5.4 LR1110 Driver Errata Workarounds

The custom `lr11xx_lora.c` driver handles several LR1110 firmware bugs:
- **CMD_ERROR IRQ**: Benign error flag on several write commands — cleared silently
- **RX buffer drift**: Buffer base shifts 4 bytes per packet → `clear_rxbuffer()` after every RX
- **Header error**: Can shift buffer pointer → standby before RX restart
- **DIO1 stuck HIGH**: 5-cycle detection → full hardware reset + recovery
- **RX duty cycle broken**: 23-40% packet loss → disabled, always continuous RX

### 5.5 Default Radio Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| Frequency | 869.618 MHz | EU 869.4-869.65 MHz band (500mW ERP allowed) |
| Bandwidth | 62 kHz | |
| Spreading Factor | 8 | |
| Coding Rate | 4/8 | |
| Preamble | 16 symbols | |
| TX Power | 22 dBm | Clamped by `CONFIG_ZEPHCORE_MAX_TX_POWER_DBM` |

---

## 6. Application Layer

### 6.1 Class Hierarchy

```
mesh::Mesh
├── BaseChatMesh (contacts, channels, messages, connections)
│   └── CompanionMesh (BLE protocol, phone sync, offline queue, ACK tracking)
└── RepeaterMesh (ClientACL, RegionMap, CLI, rate limiting, neighbor tracking)
```

### 6.2 CompanionMesh

Handles the binary BLE protocol with ~60 command opcodes. Key features:
- **Offline queue**: 16-frame circular buffer with peek/confirm pattern (survives BLE drops)
- **ACK tracking**: 8-slot table, computes expected ACK = SHA256(secret + hash)[0:4]
- **Contact iteration**: Streaming protocol with `lastmod` filtering for incremental sync
- **Lazy write batching**: Dirty contacts/channels flush after 5-second delay
- **Protocol versioning**: V2/V3 frame format negotiation with phone app
- **Ed25519 signing**: 3-phase flow (start→data→finish) for signing up to 8KB
- **Flood scope**: Transport key filtering for region-scoped sends

### 6.3 RepeaterMesh

Autonomous operation features:
- **Authentication**: Password-based login with timestamp replay protection (120s window)
- **Permission levels**: GUEST(0), READ_ONLY(1), READ_WRITE(2), ADMIN(3)
- **Region filtering**: `RegionMap` with transport key matching per flood packet
- **Rate limiting**: 4 requests per 120s (discovery), 4 per 180s (anonymous)
- **Neighbor tracking**: 16-slot table with RSSI/SNR/name/timestamp
- **Temporary radio params**: `tempradio` command applies freq/bw/sf/cr via `LoRaRadioBase::setRadioOverride()` (does not mutate `_prefs`); auto-revert timer calls `clearRadioOverride()` to fall back to saved prefs

### 6.4 CommonCLI Commands

System: `ver`, `board`, `reboot`, `start dfu`, `start ota`, `erase`
Config: `set name/freq/radio/tx/flood.max/password/...`, corresponding getters
GPS: `gps on/off/setloc/advert`
Sensors: `sensor get/set/list`
Stats: `stats-core/stats-radio/stats-packets`, `clear stats`
Power: `powersaving on/off`

---

## 7. Hardware Adapters

### 7.1 BLE (`adapters/ble/`)

- Nordic UART Service (NUS) with AUTHEN permissions on CCC + RX (forces pairing)
- Passkey-based MITM pairing (SC + MITM + Bonding), runtime configurable PIN via `app_passkey` callback
- DisplayOnly IO capability — phone enters passkey displayed on device / known to user
- Advertising with public identity address (no RPA) — required for Android Flutter BLE app compatibility
- `CONFIG_BT_PRIVACY` disabled: Android's Flutter BLE plugin fails to `connectGatt()` to RPA-advertised devices from app context; iOS and Android system BT settings handle RPA fine but the MeshCore app doesn't. Arduino MeshCore also uses public addresses.
- Pairing triggered reactively: phone hits ATT error 0x05 on secured attribute → initiates SMP pairing (Apple Accessory Design Guidelines §55 compliant — no proactive Security Request)
- TX congestion control: queue (12 frames) + overflow buffer + retry + timeout watchdog
- Fast/slow advertising switching with post-disconnect flap prevention
- DLE (Data Length Extension) to 251 bytes
- Interface coexistence: BLE vs USB, one active at a time
- Debug: `boards/common/ble_debug.conf` overlay enables DBG on bt_smp/att/gatt/conn

### 7.2 DataStore (`adapters/datastore/`)

- **Internal**: LittleFS on flash (`/lfs`), 256-byte cache for reduced flash I/O
- **External**: Optional LittleFS on QSPI (`/ext`) with auto-migration
- **BLE bonds**: File-based settings on LittleFS (`/lfs/settings/`) — all platforms (no NVS)
- **Prefs**: 292-byte binary format, Arduino-compatible, field-by-field I/O (see §13)
- **Contacts**: 152-byte records, stored on external flash if available
- **Channels**: 68-byte records (4 pad + 32 name + 32 secret)
- **Blobs**: Fixed-size records with LRU eviction by timestamp

### 7.3 GPS (`adapters/gps/`)

- State machine: OFF → ACQUIRING → STANDBY (with warm standby on supported hardware)
- 3 consecutive good fixes (≥4 satellites) required before reporting
- Multi-constellation: GPS+GLONASS+Galileo+BeiDou with fallback
- T1000-E: Complex 6-GPIO power sequencing with VRTC preservation
- Repeater mode: 48-hour wake interval for time sync only
- GPS time blocks phone time sync for 2 hours after last fix

### 7.4 USB (`adapters/usb/`)

- **CompanionUSB**: V3-framed CDC (little-endian 16-bit length prefix + payload)
- **RepeaterUSB**: Minimal CDC with 1200-baud DFU touch detection
- Both share message queues with BLE adapter (transport-agnostic mesh layer)

### 7.5 Board (`adapters/board/`)

- Battery ADC with optional regulator-gated voltage divider, 8-sample average (boards with `zephyr,user` ADC node; MG24 has no battery divider, ADC disabled)
- UF2 bootloader entry via GPREGRET magic (0x57 = UF2, 0xA8 = BLE DFU)
- TX LED bracketing for LoRa transmissions
- Bootloader version detection via flash memory scan

---

## 8. UI Subsystem

### 8.1 Architecture

Event-driven, no dedicated thread. All UI work on Zephyr work queues.

```
Hardware buttons → Zephyr input subsystem → Longpress filter → Multi-tap filter
    → ui_input_cb() → page navigation / action dispatch → schedule_render()
        → render_work (50ms OLED / 200ms EPD debounce) → CFB framebuffer → display
```

### 8.2 Pages

**Companion** (11 pages): Messages, Recent, Radio, Bluetooth, Advert, GPS, Buzzer, Sensors, Offgrid, DFU, Shutdown

**Repeater** (3 pages): Status, Radio, Shutdown

### 8.3 Multi-Tap Input

Single button, up to 4 taps within 400ms window:
- 1 tap → Page next
- 2 taps → Flood advert
- 3 taps → Buzzer toggle
- 4 taps → GPS toggle (immediate, no delay)

### 8.4 Buzzer

Non-blocking RTTTL parser on dedicated work queue. Predefined melodies for startup, shutdown, messages, ACKs. 2-second safety watchdog auto-silences on work queue stall.

### 8.5 Doom Easter Egg

Wolf3D-style raycaster on OLED: textured walls, 2 enemy types, shooting, HUD. Bypasses CFB, writes directly to display. ~1.7KB RAM, ~5KB flash. Enabled via `CONFIG_ZEPHCORE_EASTER_EGG_DOOM`. Triple-press ENTER on Messages page to activate.

---

## 9. Build System

### 9.1 Config Layering

```
prj.conf (base: console, logging)
  → boards/common/zephcore_common.conf (ALL boards: BLE, crypto, FS, LoRa, sensors)
    → boards/common/<platform>_common.conf (nrf52/esp32/nrf54l/mg24 specifics)
      → boards/<mcu>/<board>/board.conf (board-specific pins, features)
        → [optional] repeater.conf, prod.conf (user extras, LAST = highest priority)
```

### 9.2 Key Kconfig Choices

- **Role**: `ZEPHCORE_ROLE_COMPANION` (default) vs `ZEPHCORE_ROLE_REPEATER`
- **Radio**: `ZEPHCORE_RADIO_NATIVE` (SX126x, default) vs `ZEPHCORE_RADIO_LR1110`
- **Features**: Display, buzzer, buttons, multi-tap, Doom (auto-enabled from DT)

### 9.3 Platform Notes

- **nRF52840**: Zephyr open-source BLE controller, UF2 bootloader, partial flash erase for BLE coexistence
- **nRF54L15**: Same BLE controller as nRF52, CMSIS-DAP via SAMD11 bridge, no native USB
- **ESP32**: Espressif proprietary BLE blob, 32KB heap, asserts disabled (blob IRQ false positives)
- **EFR32MG24**: SiLabs proprietary BLE blob, 32KB heap, SEMAILBOX enabled for hardware TRNG/crypto entropy, ADC disabled (no battery divider), CMSIS-DAP via onboard SAMD11

### 9.4 Patches

| Patch | Risk | Purpose |
|-------|------|---------|
| 0001-lora-lr11xx-build | LOW | Integrates LR11xx driver into Zephyr LoRa build |
| 0003-lora-sx126x-native | **HIGH** | ~400 lines: DIO1 work queue, errata workarounds, extension API |
| 0005-gnss-air530z-easy | MEDIUM | EASY ephemeris + removes PM (prevents deadlocks) |
| 0006-blobs-py | LOW | Fix `west blobs fetch` KeyError |

### 9.5 Flash Partition Layouts

**nRF52840 SD v6**: SoftDevice 152KB → App 696KB → LFS 128KB → UF2 48KB
**nRF52840 SD v7**: SoftDevice 156KB → App 692KB → LFS 128KB → UF2 48KB
**ESP32 (4MB)**: Boot + App → LFS 192KB
**ESP32-S3 (16MB)**: Boot + App → LFS 384KB
**nRF54L15**: MCUboot 64KB → App 1272KB → LFS 92KB
**EFR32MG24**: MCUboot 48KB → App 1344KB → LFS 144KB

---

## 10. Board Matrix

| Board | SoC | Radio | GPS | Display | Buzzer | Buttons | QSPI | Max Contacts |
|-------|-----|-------|-----|---------|--------|---------|------|-------------|
| RAK4631 | nRF52840 | SX1262 | gnss-nmea | - | - | - | - | 350 |
| RAK3401 1W | nRF52840 | SX1262+SKY66122 (30dBm) | gnss-nmea (opt) | - | - | - | - | 350 |
| WisMesh Tag | nRF52840 | SX1262 | Air530Z | - | Yes | 1+multitap | - | 350 |
| T1000-E | nRF52840 | **LR1110** | AG3335 | - | Yes | 1+multitap | - | 350 |
| ThinkNode M1 | nRF52840 | SX1262 | Air530Z | EPD 200x200 | Yes | 2+multitap | 2MB | 510 |
| Wio Tracker L1 | nRF52840 | SX1262 | L76K | OLED 128x64 | Yes | 5-way joy | 2MB | 510 |
| Ikoka Nano 30dBm | nRF52840 | SX1262+PA | - | - | - | - | - | 350 |
| XIAO nRF54L15 | nRF54L15 | SX1262 | - | - | - | - | - | 450 |
| XIAO ESP32-C3 | ESP32-C3 | SX1262 | - | - | - | - | - | 300 |
| XIAO ESP32-C6 | ESP32-C6 | SX1262 | - | - | - | - | - | 300 |
| LilyGo TLoRa C6 | ESP32-C6 | SX1262 | - | - | - | - | - | 300 |
| Station G2 | ESP32-S3 | SX1262+PA | UART1 | OLED 128x64 | - | 1 button | - | 350 |
| XIAO MG24 | EFR32MG24 | SX1262 | - | - | - | - | - | 350 |

---

## 11. Packet Format Reference

### Wire Format

```
Byte 0: Header
  [1:0] Route type: 0=transport_flood, 1=flood, 2=direct, 3=transport_direct
  [5:2] Payload type (see table in §4.3)
  [7:6] Version (0=v1)

If transport route (bit 0 or both bits set):
  Bytes 1-4: transport_codes[2] (2x uint16_t LE)

Next byte: path_len
  [5:0] Hash count (number of hops)
  [7:6] Hash size mode (0→1B, 1→2B, 2→3B)

Next N bytes: path[] (hash_count × hash_size bytes)

Remaining bytes: payload (type-specific)
```

### Advert Payload

```
[32B pubkey] [4B timestamp LE] [64B Ed25519 signature] [0-32B app_data]

app_data format (AdvertDataHelpers):
  Byte 0: type(3:0) | flags(7:4)
    flags: bit4=lat/lon, bit5=feat1, bit6=feat2, bit7=name
  [optional 8B: lat(float) + lon(float)]
  [optional 2B: features1]
  [optional 2B: features2]
  [remaining: name string]
```

### Encrypted Datagram (REQ/RESPONSE/TXT_MSG)

```
[1B dest_hash] [1B src_hash] [encrypted_payload + 2B MAC]

encrypted_payload (after AES-128-ECB decrypt):
  For TXT_MSG: [4B timestamp] [1B txt_type] [text...]
    txt_type: 0=plain, 1=cli_data, 2=signed_plain
```

---

## 12. BLE Protocol Reference

### Frame Format

Raw binary over BLE NUS. Each frame: `[1B opcode] [payload...]`
Over USB CDC: V3 framing: `[2B LE length] [1B opcode] [payload...]`

### Key Command Opcodes (phone → device)

| Opcode | Name | Payload |
|--------|------|---------|
| 0x01 | CMD_SEND_TXT_MSG | contact_idx + text |
| 0x03 | CMD_GET_CONTACTS | [optional 4B lastmod filter] |
| 0x06 | CMD_GET_SELF_INFO | (none) |
| 0x07 | CMD_SET_SELF_INFO | type + name + lat + lon |
| 0x0B | CMD_GET_MSG_WAITING | (none) |
| 0x0C | CMD_CONFIRM_MSG | (none) |
| 0x11 | CMD_SET_PREF | pref_key + value |
| 0x12 | CMD_DEVICE_QUERY | (none) |
| 0x15 | CMD_SEND_SELF_ADVERT | (none) |
| 0x20 | CMD_NEGOTIATE_VER | target_version |

### Push Notifications (device → phone, async)

| Code | Name |
|------|------|
| 0x80 | PUSH_CODE_ADVERT |
| 0x82 | PUSH_CODE_SEND_CONFIRMED |
| 0x83 | PUSH_CODE_MSG_WAITING |
| 0x8A | PUSH_CODE_NEW_ADVERT |

---

## 13. Data Storage

### File Paths

| Path | Content | Format |
|------|---------|--------|
| `/lfs/_main.id` | Node identity | 64B private key + 32B public key |
| `/lfs/new_prefs` | Preferences | 93B binary (Arduino-compatible) |
| `/lfs/contacts3` or `/ext/contacts3` | Contacts | 152B × N records |
| `/lfs/channels2` or `/ext/channels2` | Channels | 68B × N records |
| `/lfs/adv_blobs` or `/ext/adv_blobs` | Advert cache | Fixed-size blob records |
| `/lfs/repeater/acl` | Client ACL | 136B × N records |
| `/lfs/repeater/regions2` | Region map | Header + 164B × N entries |
| `/lfs/settings/` | BLE bonds + Zephyr settings | File-based settings (all platforms) |

### Preferences Binary Layout (292 bytes)

Field-by-field serialization (NOT raw struct dump). See `memory/prefs-format.md` for full layout,
or `zephcore/helpers/CommonCLI.cpp` `loadPrefs()` for the authoritative source.

Key ranges: name(4-36), radio(72-119), adaptive-delay(80-111, ignored at runtime),
Arduino-bridge(127-151, read+discarded), GPS(156-161), owner_info(170-290), rx_boost/duty(290-291).

---

## 14. Key Call Flows

### 14.1 Receiving a LoRa Packet → Application

```
DIO1 interrupt → Zephyr lora driver → async RX callback
  → LoRaRadioBase::rxCallbackStatic() → SPSC ring buffer write → _rx_cb()
    → k_event_post(MESH_EVENT_LORA_RX) → main thread wakes
      → Dispatcher::loop() → checkRecv() → drain ring buffer
        → tryParsePacket() → score + airtime calc
          → flood: calcRxDelay() → queue inbound or process immediately
          → direct: process immediately
            → Mesh::onRecvPacket() → decrypt → dispatch by type
              → BaseChatMesh::onPeerDataRecv() → onMessageRecv()
                → CompanionMesh: writeFrame() to phone or queueOfflineMessage()
```

### 14.2 Sending a Text Message

```
Phone sends CMD_SEND_TXT_MSG via BLE NUS
  → CompanionMesh::handleProtocolFrame()
    → BaseChatMesh::sendMessage(contact, text)
      → composeMsgPacket(): ECDH secret → AES encrypt → MAC
      → if contact has path: trySendDirect()
      → else: sendFlood()
        → Mesh::sendFlood() → mark seen → queue outbound
          → Dispatcher::checkSend() → CAD check → duty cycle check → LBT → startSendRaw()
```

### 14.3 Repeater Forwarding a Packet

```
Dispatcher::checkRecv() → Mesh::onRecvPacket()
  → flood packet, not for us
    → routeRecvPacket() → allowPacketForward()
      → RepeaterMesh checks: disable_fwd? flood_max? region filter?
        → if allowed: append self hash to path, ACTION_RETRANSMIT_DELAYED
          → re-queued outbound with priority = hop count
```

### 14.4 Noise Floor Calibration Cycle

```
main event loop (every 5s) → Dispatcher::maintenanceLoop()
  → radio->triggerNoiseFloorCalibrate(threshold)
    → guards: in RX? TX active? duty cycle? mid-receive?
    → read 8 RSSI samples, take median
    → first sample: seed directly
    → warmup (<8 ticks): accept unconditionally
    → periodic bypass (every 16th): accept unconditionally
    → otherwise: reject if sample ≥ floor + 14dB
    → EMA: floor += round((sample - floor) / 8)
    → clamp [-120, -50] dBm
```
