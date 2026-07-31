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

ZephCore is a LoRa mesh networking firmware running on Zephyr RTOS. It supports four device roles:

- **Companion**: BLE-connected device paired with a phone app. Full contact/channel/message management.
- **Repeater**: Autonomous headless relay node. CLI administration via authenticated mesh connections or serial UART.
- **Room Server**: Headless store-and-forward shared message room (BBS). Reuses the repeater's ACL/region/CLI; pushes new posts to logged-in clients (per-client sync cursor + ACK).
- **Observer** (ESP32): Listen-only node that publishes received LoRa packets to MQTT over WiFi.

Supported hardware: nRF52840, nRF54L15, ESP32 (classic PICO-D4 and C3/C6/S3), EFR32MG24, and STM32WL (LoRa-E5). Radios: SX126x family (SX1261/62/68, LLCC68, STM32WL sub-GHz), LR1110, SX127x (SX1272/76/78, loramac-node backend), and LR2021 (experimental bring-up). A native Linux port runs the full stack on SBCs (Femtofox, Raspberry Pi) via Zephyr `native_sim` — see `LINUX_NATIVE.md`.

### Upstream Relationship

ZephCore is a port of [Arduino MeshCore](https://github.com/meshcore-dev/MeshCore). The core mesh protocol (Mesh.cpp, Dispatcher.cpp, Packet.cpp, Identity.cpp, Utils.cpp) is shared code. Adapters (`adapters/`) bridge MeshCore's HAL interfaces to Zephyr APIs. Binary file formats (prefs, contacts, channels) are byte-compatible with Arduino MeshCore.

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
│   ├── ContentionTracker.cpp        # Adaptive contention window (EMA, backoff)
│   ├── StaticPoolPacketManager.cpp  # Fixed-size packet pool (32 slots)
│   ├── main_companion.cpp  # Companion mode entry point + event loop
│   ├── main_repeater.cpp   # Repeater mode entry point + event loop
│   └── main_room_server.cpp # Room server mode entry point + event loop
│
├── include/mesh/           # Core interfaces (shared with Arduino MeshCore)
│   ├── Mesh.h, Dispatcher.h, Packet.h, Identity.h, Utils.h
│   ├── MeshCore.h          # Constants: key sizes, packet limits
│   ├── Radio.h             # Abstract radio interface
│   ├── Board.h, Clock.h, RNG.h, RTC.h  # HAL interfaces
│   ├── ContentionTracker.h # Adaptive contention window state
│   ├── LoRaConfig.h        # Default radio parameters
│   ├── RadioIncludes.h     # Compile-time radio driver selection
│   ├── SimpleMeshTables.h  # Hash-based packet deduplication
│   └── StaticPoolPacketManager.h  # Fixed pool allocator
│
├── adapters/               # Zephyr HAL implementations
│   ├── radio/              # LoRa radio drivers
│   │   ├── LoRaRadioBase.cpp/h    # Shared TX/RX state machine, noise floor, AGC
│   │   ├── SX126xRadio.cpp/h      # SX126x adapter (native Zephyr driver, patched)
│   │   ├── SX127xRadio.cpp/h      # SX127x adapter (loramac-node backend)
│   │   ├── LR1110Radio.cpp/h      # LR1110 adapter (custom Zephyr driver)
│   │   ├── LR2021Radio.cpp/h      # LR2021 adapter (custom driver, experimental)
│   │   ├── radio_common.h         # Shared radio types and constants
│   │   ├── lr11xx/                # LR11xx low-level HAL (SPI, GPIO, Semtech SDK)
│   │   └── lr20xx/                # LR20xx low-level HAL (Semtech SDK)
│   ├── ble/ZephyrBLE.cpp/h        # BLE NUS service, pairing, TX congestion
│   ├── board/ZephyrBoard.cpp/h    # Battery ADC, LEDs, reboot, bootloader
│   ├── clock/                     # Millisecond uptime + software RTC + I2C RTC discovery
│   ├── datastore/ZephyrDataStore.cpp/h  # LittleFS persistence
│   ├── gps/ZephyrGPSManager.cpp/h      # GNSS state machine, power mgmt
│   ├── mqtt/ZephyrMQTTPublisher.c/h    # MQTT packet publisher (observer / uplink)
│   ├── ota/wifi_ota.c/h           # WiFi SoftAP + HTTP firmware upload
│   ├── rng/ZephyrRNG.cpp/h        # Hardware CSPRNG with PRNG fallback
│   ├── sensors/                   # I2C env sensors + power monitors
│   ├── transport/                 # TCP companion (native Linux) + serial companion (STM32WL)
│   ├── usb/                       # USB CDC for companion + repeater
│   └── wifi/ZephyrWiFiStation.c/h # WiFi station client (ESP32)
│
├── app/                    # Application layer
│   ├── CompanionMesh.cpp/h       # Phone-connected companion logic
│   ├── RepeaterMesh.cpp/h        # Autonomous repeater logic
│   ├── RepeaterRegionCLI.cpp     # Repeater `region` CLI commands
│   ├── RepeaterUplink.cpp        # Repeater WiFi+MQTT uplink (ESP32)
│   ├── RepeaterDataStore.cpp/h   # Repeater-specific persistence paths
│   ├── RoomServerMesh.cpp/h      # Store-and-forward room server (BBS)
│   ├── RoomServerRegionCLI.cpp   # Room server `region` CLI commands
│   ├── ObserverMesh.cpp/h        # Listen-only WiFi+MQTT observer (ESP32)
│   └── main_observer.cpp, observer_creds.cpp/h
│
├── helpers/                # Shared utilities
│   ├── BaseChatMesh.cpp/h        # Contact/channel/message base class
│   ├── CommonCLI.cpp/h           # Serial/mesh CLI command processor
│   ├── MeshTimeSync.cpp/h        # Mesh clock-consensus estimator (§4.9)
│   ├── AdvertDataHelpers.cpp/h   # Advertisement wire format encoder/decoder
│   ├── ClientACL.cpp/h           # Authenticated client management
│   ├── TransportKeyStore.cpp/h   # Region transport key cache
│   ├── RegionMap.cpp/h           # Region-based flood filtering
│   ├── ContactInfo.h, ChannelDetails.h, NodePrefs.h  # Data structures
│   ├── RateLimiter.h, IdentityStore.h, StatsFormatHelper.h
│   ├── battery_curve.c/h, fatal_reboot.c, oled_power.c/h
│   ├── ui/                       # Shared UI plumbing: display, buzzer, multi-tap input, Doom
│   ├── ui-button/                # Single-button page UI (pages, task)
│   └── ui-joystick/              # 5-way joystick UI (Wio Tracker L1)
│
├── boards/                 # Board definitions
│   ├── common/             # Shared configs, DTS includes, partition layouts
│   ├── nrf52840/           # RAK4631, T1000-E, ThinkNode M1/M3/M6, T-Echo, T114, ...
│   ├── nrf54l/             # XIAO nRF54L15
│   ├── esp32/              # XIAO C3/C6/S3, Heltec V3/V4.x, Station G2, T-Beam, ...
│   ├── mg24/               # XIAO MG24
│   ├── stm32wl/            # Seeed LoRa-E5 mini
│   └── linux_native/       # native_sim presets (Femtofox, RAK6421) — see LINUX_NATIVE.md
│
├── patches/                # Zephyr tree modifications
│   ├── zephyr/             # Unified diffs (SX126x extensions, GNSS, native Linux, ...)
│   └── zephyr-new/         # New files (LR11xx/LR20xx drivers, native Linux SPI/GPIO, DTS bindings)
│
├── lib/monocypher/         # Vendored crypto library (Ed25519/X25519)
├── tools/                  # Formatter (flash erase) + LR1110 firmware updater
├── CMakeLists.txt          # Build orchestration
├── Kconfig                 # All ZephCore configuration options
├── Kconfig.psram           # ESP32 PSRAM auto-enable from devicetree
├── prj.conf                # Base project config
├── sysbuild.conf           # Forces MCUboot when --sysbuild is used
└── west.yml                # West manifest (Zephyr version pin)
```

---

## 3. Layer Architecture

```
┌─────────────────────────────────────────────────┐
│  Phone App (BLE NUS / USB CDC / TCP / UART)     │  External
│  or Serial CLI (USB CDC / PTY)                  │
├─────────────────────────────────────────────────┤
│  CompanionMesh / RepeaterMesh /                 │  App Layer
│  RoomServerMesh / ObserverMesh                  │
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
│    ├── SX127xRadio  ──► loramac-node backend    │
│    ├── LR1110Radio  ──► Custom LR11xx driver    │
│    └── LR2021Radio  ──► Custom LR20xx driver    │
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

**Deduplication**: `SimpleMeshTables` maintains a circular buffer of 160 packet hashes (8 bytes each, SHA-256 truncated); ACKs are deduped through the same packet-hash path. `wasSeen()` is a pure query; call sites insert explicitly via `markSeen()` to prevent duplicate processing and retransmission.

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
     - CAD: if channel busy (`isReceiving()` returns true or radio not ready),
            retry every 100-200ms (jittered) up to 4s total. On 4s timeout,
            call `_radio->recoverRxState()` (cancel + restart, clears IRQ +
            latch + grace timestamp) and re-wake the loop instead of falling
            through to TX.
     - Duty cycle: if exceeded, defer 5 seconds (admin packets exempt)
     - Final `isReceiving()` check right before TX (closes timing gap)
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

### 4.9 Mesh Time Sync (Clock Consensus)

ZephCore-only divergence from Arduino MeshCore (like the Adaptive Contention Window). A node senses its own clock error from the Ed25519-signed timestamps in other nodes' adverts and — **opt-in, default off** (`set meshtimesync on`) — corrects it automatically. There is no trusted reference clock on a mesh, so this is a *consensus estimation* problem: the node assumes the majority of tenured advert senders within 3 flood hops is right. User-facing doc: `MESHTIMESYNC.md` at the repo root.

**Module**: `helpers/MeshTimeSync.{h,cpp}` — role-agnostic estimator, owns no clock. Each role feeds it verified adverts (`onAdvertHeard`), calls `tick()` periodically (15-min pacing internal), and applies STEP verdicts under its own policy.

**Sample table** (per-sender, `CONFIG_ZEPHCORE_TIMESYNC_TABLE_SIZE` slots: 32 default, 16 on RAM-bound companions; 24 B/slot):
- 8-byte pubkey prefix — a security floor, not a tuning knob (shorter prefixes are grindable: an attacker could collide a tenured voter's prefix and reset its tenure with validly-signed adverts).
- Latest advert timestamp (= the vote, per-sender monotonic — replays and flood dupes are inert) + arrival **uptime**. Skew is recomputed at evaluate time from the uptime anchor, so the node's own steps never stale stored samples.
- Tenure tracking: first-heard uptime, advert count. Eligibility = heard ≥ 1 h, ≥ 2 adverts, latest sample ≤ 5 days old (bridges the 47 h flood-advert cadence).
- Self-consistency: consecutive samples must satisfy `|Δadvert_ts − Δuptime| ≤ 45 s + 150 ppm × Δuptime`; violation (sender rebooted/corrected/lying) resets that sender's tenure.
- **Hop-priority admission** (hop cap 3): a new sender may only displace a young entry farther (higher hop) than it; mature entries are protected unless silent > 24 h. Naive LRU churned hub nodes to zero eligible voters in simulation.

**Consensus**: Marzullo interval intersection over eligible votes, each `[skew − r, skew + r]` with `r = 150 s + 15 s × hop` (the 150 s base covers the real fleet's good-clock scatter, not just RF delay). No absolute outlier thresholds against the local clock — clustering does the rejection, so an epoch-reset clock still finds the true cluster. Stepping requires `CONFIG_ZEPHCORE_TIMESYNC_QUORUM` (default 6, floor 3, build-time security knob) eligible senders AND a strict majority inside the intersection; otherwise abstain.

**Correction policy** (priority: GPS / manual set > mesh consensus):
- Any clock set — GPS fix **or** manual set (`time`, `clock sync`, app time set) — arms the same **7-day suppression** of all stepping, bootstrap included, plus drift-envelope pedigree (`noteGPSSync` and `noteManualSync` are identical). A live GPS re-arms it on every fix (so a repeater's 48 h duty cycle keeps GPS owning the clock); a GPS that cannot fix (indoors, dead antenna) becomes mesh-correctable once 7 days pass without a fix. Sensing always continues; a suppressed node shows `hold (suppressed)` in the dry-run.
- Step trigger 10 min, dead band 5 min, step capped **±1 h**, one step per **6 h**, logged loudly. Production contains coherent wrong-time islands (+28 h × 63 repeaters at analysis time); the cap bounds capture drag to 4 h/day.
- **Drift-envelope gate**: with a trusted sync + continuous uptime since (pedigree, RAM-only), corrections beyond `elapsed × 300 ppm + 10 min` are physically impossible for a crystal — refused regardless of quorum.
- **Bootstrap**: local time < firmware build epoch (`FIRMWARE_BUILD_EPOCH`, CMake-injected) is provably wrong → any 3 agreeing senders, step to the cluster's **low edge** (midpoint − 150 s; undershoot so later refinement is always forward = monotonicity-safe).

**Per-role step policy** (policy lives in the role, not the estimator):
| Role | Policy | Why |
|---|---|---|
| Repeater | bidirectional | clock not load-bearing: forwarding/dedup/remote-admin run on `millis()`/hashes; a backward step only mutes own adverts at peers for a window equal to the step |
| Observer | bidirectional | clock only stamps observations — exactly what this fixes |
| Room server | forward-only | post timestamps feed client `sync_since` ordering |
| Companion | forward-only | own clock stamps outgoing DMs; peers hold per-sender replay high-water marks |

**Step application**: the shared policy (suppression/pedigree checks inside `evaluateNow`, forward-only skip, uint32-overflow guard, set clock, one `zephcore_rtc_save` per step — never per evaluation) lives in `MeshTimeSync::runTick()`; when it returns true, the role shifts its wall-clock-anchored bookkeeping by `lastStepDelta()` — repeater: neighbor `heard_timestamp`s, ACL `last_activity`, login/anon/discover rate-limiter resets; room server: ACL + login limiter.

All policy timers (6 h rate limit, 7-day suppression, tenure, sample age) anchor on **uptime, never wall clock** — otherwise the very steps they govern would distort them.

**CLI**: `set meshtimesync {on|off}`, `get meshtimesync` → state + live dry-run (eligible count, votes for/against, skew/radius, would-be verdict) + per-sender evidence table (full table over local USB; remote admin replies are summary-truncated to fit the packet). Sensing always runs, so the dry-run works before enabling.

**Accepted limits**: a coordinated same-offset majority around a node captures it (no consensus survives that — Bitcoin timejacking lesson; mitigations: default-off, manual override, caps); sub-quorum islands abstain forever (bootstrap still heals dead clocks with 3 senders).

---

## 5. Radio Subsystem

### 5.1 Class Hierarchy

```
mesh::Radio (abstract interface)
  └── LoRaRadioBase (shared state machine, ring buffer, noise floor)
        ├── SX126xRadio → Zephyr native SX126x driver + sx126x_ext.h
        ├── SX127xRadio → Zephyr loramac-node backend (SX1272/76/78)
        ├── LR1110Radio → Custom lr11xx_lora.c driver + Semtech HAL
        └── LR2021Radio → Custom lr20xx_lora.c driver + Semtech HAL (experimental)
```

Compile-time selection via the `CONFIG_ZEPHCORE_RADIO_NATIVE` / `_LR1110` / `_LR2021` / `_SX127X` Kconfig options, resolved in `RadioIncludes.h`. The native SX126x path is the default and covers SX1261/62/68, LLCC68, and the STM32WL integrated sub-GHz radio.

### 5.2 LoRaRadioBase State Machine

**TX Flow** (LBT — current default; `cad.mode == LORA_CAD_MODE_LBT` is set unconditionally in `buildModemConfig`):
1. `startSendRaw()` → `isReceiving()` final gate → `_tx_active = 1` → **skip** `hwCancelReceive()` and leave `_in_recv_mode = 1` so the driver sees state == RX → `configureTx()` → async send.
2. SX126x `send_async` entry CAS accepts both `REST_STATE → TX` and `RX → TX`, recording `was_rx`. LBT branch issues `set_standby(RC)` then SetCAD. On CAD-busy: in-driver `sx126x_restart_rx` puts the chip back in RX before `-EBUSY` returns. C++ failure path calls `startReceive()`, which the driver's `lora_recv_async` short-circuits when state is already RX.
3. On TX success: `_in_recv_mode = 0`, TX wait thread blocks on semaphore (5 s timeout).
4. On DIO1 `TX_DONE` interrupt → signal raised → restart RX → update stats.

**RX Flow**:
1. `lora_recv_async()` with callback. SX126x `recv_async` clears `IRQ_ALL` and resets the RX-busy signals on every fresh entry.
2. ISR writes to 8-slot SPSC ring buffer (drops NEW packet on overflow).
3. Main thread drains via `recvRaw()`.

**Config Caching**: Avoids redundant `lora_config()` calls. Fast-path for TX↔RX transitions when only direction differs. `recoverRxState()` clears the cache (`_config_cached = false`) so post-recovery RX goes through the full path.

### 5.2.1 RX-Busy Gate (TX-during-RX prevention)

`LoRaRadioBase::isReceiving()` is the single software source of truth for "currently receiving" and is consulted at three sites: dispatcher initial gate, dispatcher final gate, and `startSendRaw`'s last-moment gate. Logic:

```
isReceiving()
  ├─ false if !_in_recv_mode || _tx_active
  ├─ true  if hwIsReceiving()         ← per-adapter; never clears IRQ
  └─ isChannelActive() RSSI fallback  ← sub-preamble-threshold energy
```

For SX126x, `hwIsReceiving()` → `sx126x_is_receiving()` reads in this order:
1. **`data->rx_packet_active`** latch (no SPI). Set by the work handler on `HEADER_VALID`; cleared on every terminal event and RX (re)start. Covers the full payload phase. Bounded by a payload deadline: `header_seen_at_ms` is stamped when the latch is promoted, and once `sx126x_max_payload_ms()` (255-byte airtime at the current SF/BW, CR 4/8, LDRO on, +25% +100 ms) has elapsed the latch is released and the sticky PREAMBLE/SYNC/HEADER bits cleared. Continuous RX has no symbol timer, so without this a `HEADER_VALID` whose packet never completes would hold the TX gate closed until reboot; the DC parked-RX watchdog does not cover it (DC-only, and it treats the latch as a legitimate in-flight packet).
2. **Mutex-busy conservative** — if the SPI mutex is contended and `state == RX`, return true (the work handler is likely mid-`RxDone`).
3. **`HEADER_VALID` raw bit** — covers the microseconds between DIO1 firing and the work handler running.
4. **`PREAMBLE_DETECTED` raw bit with SF-aware grace** — `PREAMBLE_DETECTED` is masked off DIO1 (fires on noise), but visible in the IRQ register. On first observation, `is_receiving` records `data->preamble_seen_at_ms`; subsequent calls return true until either `HEADER_VALID` promotes the latch (timestamp reset) or `(preamble_len + 8) × 2^SF / BW` ms elapses — at which point the bit is explicitly cleared and TX is allowed. Grace scales with SF: ~82 ms at SF8, ~786 ms at SF12.

The poll path is otherwise non-destructive — IRQ bits are cleared only by the work-handler bulk clear (on any DIO1 event), explicit `clear_irq_status(IRQ_ALL)` at every RX (re)start, the grace-expiry one-bit clear for foreign preambles, and the payload-deadline clear in step 1.

### 5.2.2 CAD-Timeout Recovery

`Dispatcher::checkSend()` tracks `cad_busy_start` while `isReceiving()` keeps the TX gate closed. If 4 s elapse (`getCADFailMaxDuration()`), the dispatcher calls `_radio->recoverRxState()` and returns. `LoRaRadioBase::recoverRxState()` does:

```cpp
hwCancelReceive();              // RX → IDLE → STANDBY → SLEEP (REST_STATE)
atomic_set(&_in_recv_mode, 0);  // resync C++ side
_config_cached = false;         // force full lora_config on the way back
startReceive();                 // CAS(REST → RX) clears latch + IRQ
```

This walks the chip through REST so the driver's `lora_recv_async` entry CAS (`REST_STATE → RX`) actually succeeds — a bare `startReceive()` from `state == RX` would fail with `-EBUSY` and set `_in_recv_mode = 0` while the driver still thinks it's in RX. After recovery, the dispatcher fires `_tx_queued_cb(1, ...)` to re-wake the loop promptly.

### 5.3 Noise Floor EMA

Algorithm in `triggerNoiseFloorCalibrate()`:
- 8 RSSI samples per tick, take median (insertion-sort midpoint)
- Threshold filter: reject samples ≥ floor + 14dB (after 8-tick warmup)
- Periodic bypass: every 16th tick accepts unconditionally
- EMA: `floor += round_nearest((sample - floor) / 8)`, clamped to [-120, -50] dBm

### 5.3.1 Adaptive CAD (LBT detPeak calibration)

`cadDetPeak` is a correlation peak-to-noise threshold in the despreader (not
dBm): it gates on signal *strength* ≈ link budget, blind to distance, so
raising it means "react to strong signals only, ignore faint/echo". The right
LBT sensitivity is site-dependent and cannot be derived from the RSSI floor.
`LoRaRadioBase::cadMaintenance()` (housekeeping tick) runs one calibration CAD
probe per `probe.interval` (default **15 s**) at a signed **level** relative
to the family's per-SF base detPeak, restarts RX, and classifies busy verdicts
with a ground-truth filter. **Key property:** the probe is *skipped* when RSSI >
floor+7 dB, so probes only ever sample the quiet/faint regime — the whole loop
is a faint-rejection tuner and `busy%` is faint-regime, not total occupancy.
Post-busy classification watches a ~12-symbol window for RX re-sync **or** an
RSSI climb above floor+guard (the energy path recovers real packets whose
preamble the probe's RX-restart ate — the fix for the FP over-count that used to
drive the staircase to the ceiling) → `tp`, else `fp`. Counters decay 6-hourly,
reset on any RF param change.

With `cad.auto on` the staircase is **knee-seeking**: probes sample op / op−1 /
op+1 (½/¼/¼); it steps **up** when the level above is ≥`CAD_KNEE_SLOPE_PERMILLE`
(5%) cleaner (steep side, below knee), **down** only on a clean flat plateau
(`≤CAD_PLATEAU_CLEAN_PERMILLE`), else holds — slope-based so convergence is
independent of a site's FP floor. Highest-priority override: **airtime / faint
cap** — step up when the operating busy rate exceeds `cad_busycap` (percent,
`set cad.busycap`, default 25, 0=off); self-targeting since only busy nodes
reach it, and effectively a faint-tolerance dial (lower = reject faint harder).
Each step needs ≥`CAD_STEP_MIN_PROBES` (120); offset clamped **−8…+12**,
persisted via `Dispatcher::onCadOffsetChanged()`. Driver absolute clamp (SX126x
15–40, LR 48–90) is a guardrail; AN1200.48 recommends 21–29 for SX126x (base
`SF+13`), tuned to catch faint — LBT may deliberately sit above it. Probe +
offset plumbing is per-driver extension API (`*_cad_probe`,
`*_cad_set_peak_offset`, `*_cad_base_peak`); LBT CAD runs 4 symbols (set in
`buildModemConfig`), drivers scale their blocking-CAD timeout to
`nSym·Tsym + margin`. CLI: `get cad` (3-rung window, `*`=operating, `bc:`=cap),
`set cad.auto/offset/probe.interval/busycap/reset`. SX127x: unsupported (no HW
CAD). Full mental model + tuning: `ADAPTIVE_CAD.md`.

### 5.4 LR1110 Driver Errata Workarounds

The custom `lr11xx_lora.c` driver handles several LR1110 firmware bugs:
- **CMD_ERROR IRQ**: Benign error flag on several write commands — cleared silently
- **RX buffer drift**: Buffer base shifts 4 bytes per packet → `clear_rxbuffer()` after every RX
- **Header error**: Can shift buffer pointer → standby before RX restart
- **DIO1 stuck HIGH**: 5-cycle detection → full hardware reset + recovery
- **RX duty cycle**: wired via `SetRxDutyCycle` MODE_RX, sized by the shared adapter math (same as SX126x). The earlier "broken, 23-40% loss" verdict was a window-sizing bug (over-sleep + no header budget), not a chip defect — default-off, HW-verify before production use.

### 5.5 SX127x and LR2021 Paths

- **SX127x** (`CONFIG_ZEPHCORE_RADIO_SX127X`): uses Zephyr's loramac-node LoRa backend instead of the native driver (`CONFIG_LORA_MODULE_BACKEND_LORAMAC_NODE`). Patch `0004-lora-sx127x-62k5-bandwidth` adds the 62.5 kHz bandwidth MeshCore defaults to. No RX duty cycle and no RX gain boost on this path. Reference board: TTGO LoRa32 (SX1276).
- **LR2021** (`CONFIG_ZEPHCORE_RADIO_LR2021`): custom driver in `patches/zephyr-new/drivers/lora/lr20xx/` (copied into the Zephyr tree at configure time, like LR11xx). Experimental — bring-up on the ProMicro LR2021 is still in progress; not listed as a supported board.

### 5.6 Default Radio Parameters

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
├── RepeaterMesh (ClientACL, RegionMap, CLI, rate limiting, neighbor tracking)
├── RoomServerMesh (store-and-forward BBS; reuses repeater ACL/region/CLI)
└── ObserverMesh (listen-only; publishes packets to MQTT over WiFi — ESP32)
```

### 6.2 CompanionMesh

Handles the binary BLE protocol with ~50 command opcodes. Key features:
- **Offline queue**: circular buffer with peek/confirm pattern (survives BLE drops); `CONFIG_ZEPHCORE_OFFLINE_QUEUE_SIZE`, default 256 frames (lowered on RAM-bound boards)
- **ACK tracking**: 8-slot table, computes expected ACK = SHA256(secret + hash)[0:4]
- **Contact iteration**: Streaming protocol with `lastmod` filtering for incremental sync
- **Lazy write batching**: Dirty contacts/channels flush after 5-second delay
- **Protocol versioning**: V2/V3 frame format negotiation with phone app
- **Ed25519 signing**: 3-phase flow (start→data→finish) for signing up to 8KB
- **Flood scope**: Transport key filtering for region-scoped sends

### 6.2.1 V-Contact (Loopback Admin Contact)

ZephCore-only feature (no Arduino equivalent). The companion synthesizes a CHAT
contact named `v<node_name>` that exists only toward the connected BLE/USB app.
Chatting with it runs the same text CLI as the USB serial sideband; the reply
comes back as normal chat messages. The firmware also uses it to emit
unsolicited notices: a one-shot low-battery alert and a restart-reason message
(all causes: PIN/SOFTWARE/BROWNOUT/POR/WATCHDOG/LOCKUP — offline-queue only,
so routine power-on "noise" costs nothing over the air).

**Identity**: pubkey = `SHA256("zc-vcontact" || self_pubkey)` — stable per
node, unique per device, and deliberately **not a real keypair**: no private
key exists anywhere.

**No-RF invariants** (all enforced in `CompanionMesh`):
1. `vcontactHandleFrame()` intercepts `CMD_SEND_TXT_MSG` (and the handful of
   other opcodes that must succeed) *before* any contact lookup — the CLI runs
   and the reply is written straight into the offline queue. **No packet
   object is ever created**, so nothing can reach the dispatcher or radio.
2. The v-contact never enters the real contacts table (`CMD_ADD_UPDATE_CONTACT`
   for its key is intercepted to a no-op OK), so it is never in the RF RX
   matching path. Every other pubkey-addressed opcode (login, telemetry,
   binary req, path discovery…) misses `lookupContactByPubKey()` and fails
   `ERR_NOT_FOUND` before a packet exists.
3. Even a hand-crafted over-the-air packet addressed to the derived pubkey is
   inert: unknown dest, undecryptable by everyone including this node.

**App plumbing**: appears as a virtual tail entry in the `CMD_GET_CONTACTS`
iteration (and `+1` in the CONTACT_START total); pushed as `NEW_ADVERT` on
runtime enable and rename, `CONTACT_DELETED` on disable. App-side contact
delete (`CMD_REMOVE_CONTACT`) turns the feature off. Send/ack choreography is
synthesized (SENT + immediate SEND_CONFIRMED, trip time 0). CLI replies are
chunked at ≤150 chars on line breaks (offline-queue frames cap at 172 bytes).

**Clock gating (no 1970 timestamps)**: while the RTC has never been synced
(time < firmware build epoch) the v-contact is *deferred* — withheld from
contact sync and adverts, and notices are buffered in a small RAM slot
(`_vcontact_pending`) instead of queued with an epoch-0 timestamp.
`vcontactClockSynced()` activates it and flushes the buffer; hooked at
`CMD_APP_START` (covers hardware-RTC boards, already valid), successful
`CMD_SET_DEVICE_TIME` (typical app connect flow), and GPS time sync.

**Resend dedupe**: app retry attempts reuse the message timestamp (only the
attempt byte changes); `_vcontact_last_ts` suppresses re-execution — a dupe
gets the full ack choreography but the CLI does not run twice. Side effect:
sending the identical command twice within the same wall-clock second only
executes once (same app-side timestamp). Synthesized `est_timeout` is 3 s so
the app's retry timer doesn't race the loopback confirmation.

**Stats**: `CompanionCLICallbacks` overrides
`formatStatsReply`/`formatRadioStatsReply`/`formatPacketStatsReply` with the
repeater's `StatsFormatHelper` JSON, so `stats-core`/`stats-radio`/
`stats-packets` return real data over USB and the v-contact.

**Notices ride the offline queue** — emitted while nothing is connected, they
are delivered on the first app connect/sync. RAM-backed: lost on reboot (the
restart-reason message partially compensates) and bounded by
`CONFIG_ZEPHCORE_OFFLINE_QUEUE_SIZE`.

**Settings** (companion `v.*` CLI namespace, prefs offsets 152–154):
- `set/get v.contact on|off` — default on.
- `set/get v.batteryalert <mV>|0|default` — default = board auto-shutdown
  threshold + 200 mV (so the alert wins the race against the 90 s shutdown
  confirm window), 3500 mV on boards without auto-shutdown. Alert latches
  once per discharge cycle; re-arms on external power, recovery above
  threshold + 150 mV, or threshold change. Sampling mirrors
  `ui_auto_shutdown_check()` (30 s gate, 3-strike confirm) but lives in
  `main_companion.cpp` so headless builds alert too.

### 6.3 RepeaterMesh

Autonomous operation features:
- **Authentication**: Password-based login with timestamp replay protection (120s window)
- **Permission levels**: GUEST(0), READ_ONLY(1), READ_WRITE(2), ADMIN(3)
- **Region filtering**: `RegionMap` with transport key matching per flood packet
- **Rate limiting**: 4 requests per 120s (discovery), 4 per 180s (anonymous), 4 failed logins per 180s
- **Neighbor tracking**: RSSI/SNR/name/timestamp table (`CONFIG_ZEPHCORE_MAX_NEIGHBOURS`, default 50 slots)
- **Temporary radio params**: `tempradio` command applies freq/bw/sf/cr via `LoRaRadioBase::setRadioOverride()` (does not mutate `_prefs`); auto-revert timer calls `clearRadioOverride()` to fall back to saved prefs
- **WiFi+MQTT uplink** (ESP32, `CONFIG_ZEPHCORE_REPEATER_UPLINK`): `RepeaterUplink.cpp` reports packets observer-style while still repeating; configured via `set uplink.*` CLI

### 6.4 RoomServerMesh

Headless store-and-forward shared message room (BBS). Clients log in with the admin or guest password and post messages; the server pushes each new post to every other logged-in client (per-client sync cursor + ACK). Reuses the repeater's ACL, region filtering, and USB CLI. Entry point `main_room_server.cpp`; build with `boards/common/room_server.conf`.

### 6.5 ObserverMesh

Listen-only node (ESP32 only): receives LoRa packets and publishes them to an MQTT broker over WiFi STA (`adapters/mqtt/`, `adapters/wifi/`). Never transmits. Configured at runtime via serial CLI (credentials in `observer_creds.cpp`); build with `boards/common/observer.conf`.

### 6.6 CommonCLI Commands

System: `ver`, `board`, `reboot`, `start dfu`, `start ota`, `erase`
Config: `set name/freq/radio/tx/flood.max/password/...`, corresponding getters
GPS: `gps on/off/setloc/advert`, `set gps duty <sec>`
Sensors: `sensor get/set/list`
Stats: `stats-core/stats-radio/stats-packets`, `clear stats`
Time: `clock`, `clock sync`, `time <epoch>`, `set meshtimesync on/off`

Full command reference with constraints and remote-admin restrictions: `Repeater_CLI_commands.md`.

---

## 7. Hardware Adapters

### 7.1 BLE (`adapters/ble/`)

- Nordic UART Service (NUS) with AUTHEN permissions on CCC + RX (forces pairing)
- Passkey-based MITM pairing (SC + MITM + Bonding), runtime configurable PIN via `app_passkey` callback
- DisplayOnly IO capability — phone enters passkey displayed on device / known to user
- Advertising always uses `BT_LE_ADV_OPT_USE_IDENTITY` — exposes the stable identity address even when privacy is enabled, preserving Android connect-from-app
- `CONFIG_BT_PRIVACY` **disabled** on nRF52840 / MG24: identity address is advertised directly; both iOS and Android work without RPA. Android's Flutter BLE plugin fails `connectGatt()` to RPA-advertised devices from app context.
- `CONFIG_BT_PRIVACY` **enabled** on ESP32-S3 (`boards/common/esp32_common.conf`): the Espressif controller's privacy-OFF Secure-Connections path produces a MIC failure against iOS (HCI disconnect `0x3d` at encryption start). Privacy ON keeps the controller on its working SC path. `USE_IDENTITY` advertising preserves Android compatibility. Do **not** remove `USE_IDENTITY` while ESP32 privacy is on.
- Pairing triggered reactively: phone hits ATT error 0x05 on secured attribute → initiates SMP pairing (Apple Accessory Design Guidelines §55 compliant — no proactive Security Request)
- TX congestion control: queue (12 frames) + overflow buffer + retry + timeout watchdog
- Fast/slow advertising switching with post-disconnect flap prevention
- DLE (Data Length Extension) to 251 bytes
- Interface coexistence: BLE vs USB, one active at a time
- Debug: build with `debug.conf` plus `-DCONFIG_ZEPHCORE_BLE_LOG_LEVEL_DBG=y` for adapter-level DBG logging

### 7.2 DataStore (`adapters/datastore/`)

- **Internal**: LittleFS on flash (`/lfs`), 256-byte cache for reduced flash I/O
- **External**: Optional LittleFS on QSPI (`/ext`) with auto-migration
- **BLE bonds**: NVS (`storage_partition`, 0xD0000 on nRF52) via Zephyr settings backend (≥1.16.2)
- **Prefs**: 152-byte binary (companion `new_prefs`), Arduino-compatible base + ZephCore extension fields, field-by-field I/O (see §13)
- **Contacts**: 152-byte records, stored on external flash if available
- **Channels**: 68-byte records (4 pad + 32 name + 32 secret)
- **Blobs**: Fixed-size records with LRU eviction by timestamp

**First-boot migration (3-way FS self-heal)**

A marker file `/lfs/_zc_init` is written after the first clean ZephCore boot. On every subsequent boot it is present and the logic below is skipped. On first boot (marker absent), `main_companion.cpp` picks one of three paths before `bt_enable()` runs:

1. **No prefs, or Arduino MeshCore prefs** → full LFS + NVS format. Arduino's `new_prefs` omits `node_lat`/`node_lon`, shifting `freq`/`sf`/`bw` by 16 bytes; `prefsLookLikeArduino()` detects this by range-checking those fields. Covers fresh installs and Arduino → ZephCore migrations.
2. **Valid ZephCore prefs + `/lfs/settings` present** → NVS-only erase (`formatNVSOnly()`). ZephCore ≤1.16.1 stored BLE bonds in `/lfs/settings` (file backend); ≤1.16.1 used 0xD0000 as app code, so bytes there may pass NVS sector validation and hang `settings_load()`. Identity/prefs/contacts are preserved; re-pairing is required.
3. **Valid ZephCore prefs + no `/lfs/settings`** → skip format entirely. NVS was already initialised by ZephCore ≥1.16.2; bonds survive the upgrade.

`loadPrefs()` also range-checks `freq`/`sf`/`bw` after deserialisation and reverts to compile-time defaults on out-of-range values, so a misread Arduino prefs file never corrupts the radio config.

### 7.3 GPS (`adapters/gps/`)

- State machine: OFF → ACQUIRING → STANDBY (with warm standby on supported hardware)
- 3 consecutive good fixes (≥4 satellites) required before reporting
- Multi-constellation: GPS+GLONASS+Galileo+BeiDou with fallback
- T1000-E: Complex 6-GPIO power sequencing with VRTC preservation
- GPS time blocks phone time sync for 2 hours after last fix

**Duty cycle vs always-on**

`gps_wake_interval_ms` (initialised from `prefs.gps_interval`) controls the mode:

- **Duty cycling** (`gps_wake_interval_ms > 0`): after acquiring 3 good fixes the GPS powers down; the state machine wakes it again after the configured standby interval. The fix callback fires and then the GPS sleeps.
- **Always-on** (`gps_wake_interval_ms == 0`): the GPS never powers down. `consecutive_good_fixes` is reset after each promotion so the 3-fix gate cycles continuously, streaming fresh positions. Flash writes and fix callbacks are rate-limited to once per `gps_acquire_timeout_ms` to avoid hammering storage.

`gps_set_poll_interval_sec(0)` switches to always-on live; persisted via `prefs.gps_interval` (set by `set gps duty 0`).

**Timeout split**

Two separate timeouts apply to acquisition:

- `CONFIG_ZEPHCORE_GPS_FIRST_FIX_TIMEOUT_SEC` (default 300s): the cold-start window used for the very first acquisition after `gps_enable()`. Longer to allow almanac download.
- `CONFIG_ZEPHCORE_GPS_FIX_TIMEOUT_SEC` (default 120s): the normal per-wake timeout for all subsequent acquisitions (warm start).

**Repeater mode**

Repeaters and room servers default to `CONFIG_ZEPHCORE_REPEATER_GPS_INTERVAL_SEC` (48 h) for GPS duty — GPS wakes only for a periodic time-sync fix (5-minute acquire window). The interval is now unified with companion via `prefs.gps_interval` and is configurable at runtime via `set gps duty <sec>`; persists across reboots.

### 7.4 USB (`adapters/usb/`)

- **CompanionUSB**: V3-framed CDC (little-endian 16-bit length prefix + payload)
- **RepeaterUSB**: Minimal CDC with 1200-baud DFU touch detection
- Both share message queues with BLE adapter (transport-agnostic mesh layer)

### 7.5 Board (`adapters/board/`)

- Battery ADC with optional regulator-gated voltage divider, 8-sample average (boards with `zephyr,user` ADC node; MG24 has no battery divider, ADC disabled)
- UF2 bootloader entry via GPREGRET magic (0x57 = UF2, 0xA8 = BLE DFU)
- TX LED bracketing for LoRa transmissions (gated by the LED master switch below)
- Bootloader version detection via flash memory scan

**LED master switch** (`helpers/led_gate.{c,h}`, `set leds on|off`, all roles): one process-wide
flag every LED driver consults — heartbeat and unread-message LEDs in `helpers/ui/ui_common.c`, the
`lora-tx-led` in `ZephyrBoard::onBeforeTransmit()`, and the message/shutdown flashes. It lives
outside the UI layer because `ui_common.c` is only compiled when a UI is enabled, while a headless
repeater still blinks on every transmit. `ui_common.c` overrides the weak `zephcore_leds_ui_sync()`
hook so a CLI change also stops a lit heartbeat and refreshes the UI's LEDs page. Persisted in
`NodePrefs.leds_disabled` (companion offset 93; repeater offset 120, magic-encoded — see §13).
Does not cover the display backlight, which has its own UI brightness setting (`display_brightness`).

### 7.6 WiFi / MQTT / TCP Transports

- **`adapters/wifi/ZephyrWiFiStation.c`**: WiFi STA client (ESP32) used by observer and repeater uplink
- **`adapters/mqtt/ZephyrMQTTPublisher.c`**: MQTT publisher for observed/uplinked packets
- **`adapters/ota/wifi_ota.c`**: WiFi SoftAP + HTTP firmware upload to MCUboot slot1 (ESP32, requires `--sysbuild`)
- **`adapters/transport/LinuxTCPTransport.c`**: TCP companion transport on native Linux (port 5000, MeshCore `SerialWifiInterface` framing)
- **`adapters/transport/SerialCompanionTransport.c`**: UART companion transport (STM32WL — drop-in `zephcore_ble_*` provider, auto-selected when `CONFIG_BT=n`)

---

## 8. UI Subsystem

### 8.1 Architecture

Event-driven, no dedicated thread. All UI work on Zephyr work queues.

Two UI frontends share the same plumbing (`helpers/ui/`: display, buzzer, multi-tap input filter, mesh action queue):

- **Button UI** (`helpers/ui-button/`): single-button page cycler — most boards
- **Joystick UI** (`helpers/ui-joystick/`): 5-way joystick menu UI (Wio Tracker L1)

```
Hardware buttons → Zephyr input subsystem → Longpress filter → Multi-tap filter
    → ui_input_cb() → page navigation / action dispatch → schedule_render()
        → render_work (50ms OLED / 200ms EPD debounce) → CFB framebuffer → display
```

Color TFT panels (T114, T096, Wireless Tracker) are wrapped as 1bpp displays for CFB via the `zephcore,mono-tft` shim (`display_mono_tft.c`).

### 8.2 Pages (Button UI)

**Companion** (up to 12 pages): Messages, Recent, Radio, Bluetooth, Advert, GPS, Buzzer (if buzzer present), LEDs, Sensors, Offgrid, DFU, Shutdown

**Repeater** (3 pages): Status, Radio, Shutdown

### 8.2.1 Renderer Split (mono / color)

Pages whose color layout genuinely diverges from the mono layout are split into
dedicated renderers behind a compile-time seam, instead of branching on
capability inline (and never into per-board renderer files):

```
render_<page>_mono()             — mono / tiny / e-ink layout (always compiled)
render_<page>_color()            — RGB565 layout, wrapped in
                                   #if MC_DISPLAY_COLOR_PANEL
render_<page>()                  — thin dispatcher:
                                     #if MC_DISPLAY_COLOR_PANEL
                                     if (mc_display_has_color()) { _color(); return; }
                                     #endif
                                     _mono();
```

`MC_DISPLAY_COLOR_PANEL` is defined (in `display.h`) only when a `tft` node
exists in devicetree. On a mono/e-ink board the color bodies — and every
color-only helper they reference (`draw_activity_graph`, `use_compact_color_home`,
the `activity_*` buffers, …) — are dropped at compile time, so color rendering
costs zero flash/RAM there. Adding a new color board reuses `_color`; it must
never fork a board-specific renderer.

Pages with a **shared** flow that only tints per-row (Recent, GPS, Sensors,
Status) stay as single functions with inline `if (mc_display_has_color())` —
that already is the "one layout, colored" ideal, and the color branch
dead-code-eliminates on mono via the constant-false `mc_display_has_color()`.
Split pages: Messages, Radio, Traffic, Bluetooth, Advert, LEDs, Offgrid, DFU,
Shutdown.

### 8.3 Multi-Tap Input

Single button; tap-count → key-code mapping comes from the board's devicetree `tap-codes` (up to 5). Typical mapping:
- 1 tap → Page next
- 2 taps → LED heartbeat toggle
- 3 taps → Buzzer toggle
- 4 taps → GPS toggle
- 5 taps → Flood advert (immediate, no delay)

### 8.4 Buzzer

Non-blocking RTTTL parser on dedicated work queue. Predefined melodies for startup, shutdown, messages, ACKs. 2-second safety watchdog auto-silences on work queue stall.

### 8.5 Doom Easter Egg

Wolf3D-style raycaster on OLED: textured walls, 2 enemy types, shooting, HUD. Bypasses CFB, writes directly to display. ~1.7KB RAM, ~5KB flash. Enabled via `CONFIG_ZEPHCORE_EASTER_EGG_DOOM`. Button UI: triple-press ENTER on Messages page. Joystick UI: Tools menu → "Doom".

---

## 9. Build System

### 9.1 Config Layering

```
prj.conf (base: console; production defaults — LOG=n, ASSERT=n)
  → boards/common/zephcore_common.conf (ALL boards: BLE, crypto, FS, LoRa, sensors)
    → boards/common/<platform>_common.conf (nrf52/esp32/nrf54l/mg24 specifics)
      → boards/<mcu>/<board>/board.conf (board-specific pins, features)
        → [optional] repeater.conf, debug.conf (user extras, LAST = highest priority)
```

### 9.2 Key Kconfig Choices

- **Role**: `ZEPHCORE_ROLE_COMPANION` (default) vs `ZEPHCORE_ROLE_REPEATER` vs `ZEPHCORE_ROLE_ROOM_SERVER` vs `ZEPHCORE_ROLE_OBSERVER` (selected via `repeater.conf` / `room_server.conf` / `observer.conf`)
- **Radio**: `ZEPHCORE_RADIO_NATIVE` (SX126x, default) vs `ZEPHCORE_RADIO_LR1110` vs `ZEPHCORE_RADIO_LR2021` vs `ZEPHCORE_RADIO_SX127X`
- **Features**: Display, buzzer, buttons, multi-tap, Doom (auto-enabled from DT); PSRAM auto-enable from DT (`Kconfig.psram`)

### 9.3 Platform Notes

- **nRF52840**: Zephyr open-source BLE controller, UF2 bootloader, partial flash erase for BLE coexistence
- **nRF54L15**: Same BLE controller as nRF52, CMSIS-DAP via SAMD11 bridge, no native USB
- **ESP32-C3/C6/S3**: Espressif proprietary BLE blob, 32KB heap, asserts disabled (blob IRQ false positives); simple-boot by default, MCUboot only with `--sysbuild` (WiFi OTA)
- **ESP32 classic (PICO-D4)**: much smaller DRAM — contact/queue caps shrunk in `board.conf`; console/CLI on `uart0` (no native USB); DIO flash mode required (QIO bootloops)
- **EFR32MG24**: SiLabs proprietary BLE blob, 32KB heap, SEMAILBOX enabled for hardware TRNG/crypto entropy, ADC disabled (no battery divider), CMSIS-DAP via onboard SAMD11
- **STM32WL (LoRa-E5)**: no BLE, no USB device — companion protocol and CLI run over USART1; 64KB SRAM caps contacts/queues hard; TRNG entropy; single app partition, flash via SWD
- **Native Linux (`native_sim`)**: real SPI/GPIO via spidev + GPIO chardev; TCP companion transport; file-backed flash — see `LINUX_NATIVE.md`

### 9.4 Patches

Applied automatically at CMake configure time; a failed patch aborts the configure with the offending patch named.

| Patch | Risk | Purpose |
|-------|------|---------|
| 0001-lora-lr11xx-build | LOW | Integrates LR11xx driver into Zephyr LoRa build |
| 0002-lora-lr20xx-build | LOW | Integrates LR20xx driver into Zephyr LoRa build |
| 0003-lora-sx126x-native | **HIGH** | DIO1 work queue, duty cycle, RX-busy gating, extension API, errata workarounds |
| 0004-lora-sx127x-62k5-bandwidth | LOW | Adds 62.5 kHz bandwidth to the loramac-node backend |
| 0005-gnss-air530z-easy | MEDIUM | EASY ephemeris + removes PM (prevents deadlocks) |
| 0006-blobs-py | LOW | Fix `west blobs fetch` KeyError |
| 0007-spi-gpio-native-linux | LOW | Wires native-Linux SPI/GPIO drivers into the Zephyr build |
| 0008-flash-sim-per-node-file | LOW | Flash simulator defaults to per-node settings file (native Linux) |
| 0009-display-ssd16xx-fill-ram-white | LOW | E-paper full-refresh-to-white anti-ghosting helper |

New drivers in `patches/zephyr-new/` (LR11xx, LR20xx, native-Linux SPI/GPIO, DTS bindings) are copied — not patched — into the Zephyr tree at configure time.

### 9.5 Flash Partition Layouts

**nRF52840 SD v6**: SoftDevice 152KB → App 680KB → NVS 16KB → LFS 128KB → UF2 48KB
**nRF52840 SD v7**: SoftDevice 156KB → App 676KB → NVS 16KB → LFS 128KB → UF2 48KB
**ESP32 (4MB)**: Boot + App → LFS 192KB + NVS 16KB
**ESP32-S3 (8/16MB)**: Boot + App → LFS 384KB + NVS 16KB
**nRF54L15**: MCUboot 64KB → App 1272KB → LFS 92KB
**EFR32MG24**: MCUboot 48KB (reserved) → App 1344KB → LFS 144KB
**STM32WL**: App at flash origin → LFS (no bootloader)

---

## 10. Board Matrix

Build strings and flash methods: `boards/supported_boards.md` and `boards/example_board/README.md`.

| Board | SoC | Radio | GPS | Display | Notable extras |
|-------|-----|-------|-----|---------|----------------|
| RAK4631 / WisMesh Pocket | nRF52840 | SX1262 | u-blox MAX-7Q (opt) | WisBlock OLED (opt) | I2C sensors |
| RAK3401 1W | nRF52840 | SX1262+SKY66122 (30dBm) | u-blox MAX-7Q (opt) | - | I2C sensors |
| RAK WisMesh Tag | nRF52840 | SX1262 | AT6558R | - | Accelerometer, buzzer, multitap |
| T1000-E | nRF52840 | **LR1110** | AG3335 | - | Buzzer, button, multitap |
| ThinkNode M1 | nRF52840 | SX1262 | Air530Z | EPD 200x200 (SSD1681) | Buzzer, 2 buttons, QSPI 2MB, RGB LEDs |
| ThinkNode M3 | nRF52840 | **LR1110** | Yes | - | Buzzer, 2 buttons, RGB LEDs |
| ThinkNode M6 | nRF52840 | SX1262 | L76K | - | QSPI, RGB LEDs |
| Wio Tracker L1 | nRF52840 | SX1262 | L76K | OLED 128x64 (SH1106) | 5-way joystick UI, buzzer, QSPI 2MB |
| LilyGo T-Echo | nRF52840 | SX1262 (TCXO 1.8V) | L76K | EPD 1.54" (SSD1681) | BME280, QSPI, touch-button backlight |
| Heltec T114 | nRF52840 | SX1262 | - | TFT 240x135 (ST7789V) | Screenless build via `no_display.conf` |
| Heltec Mesh Node T096 | nRF52840 | SX1262+KCT8103L PA | UC6580 | TFT 160x80 (ST7735S) | Button, LED, battery ADC |
| Ikoka Nano 30dBm | nRF52840 | SX1262+PA (30dBm) | - | - | RGB LEDs |
| GAT562 30S Mesh Kit | nRF52840 | SX1262+PA (1W) | Yes | OLED (SSD1306) | 5-way joystick, buzzer, solar |
| SenseCAP Solar | nRF52840 | SX1262 | L76K | - | QSPI, battery monitor |
| XIAO nRF52840 + Wio-SX1262 | nRF52840 | SX1262 | - | - | - |
| ProMicro SX1262 | nRF52840 | SX1262 (E22-900M30S) | Yes | - | Button, LED, battery ADC |
| muzi works R1 Neo | nRF52840 | SX1262 | Yes | - | Buzzer, button, RX8130CE RTC, latched-rail power-off |
| XIAO nRF54L15 | nRF54L15 | SX1262 | - | - | Contacts capped at 450 |
| XIAO ESP32-C3 | ESP32-C3 | SX1262 | - | - | Contacts capped at 300 |
| XIAO ESP32-C6 | ESP32-C6 | SX1262 | - | - | - |
| LilyGo TLoRa C6 | ESP32-C6 | SX1262 | - | - | - |
| XIAO ESP32-S3 | ESP32-S3 | SX1262 | - | - | 8MB flash, 8MB PSRAM |
| Station G2 | ESP32-S3 | SX1262+PA | UART GNSS | OLED (SH1106) | 16MB flash, 8MB PSRAM |
| Heltec V3 | ESP32-S3 | SX1262 | - | OLED (SSD1306) | Console on `uart0` |
| Heltec V4.2 / V4.3 | ESP32-S3 | SX1262+PA (GC1109 / KCT8103L) | - | OLED (SSD1306) | 16MB flash, 2MB PSRAM |
| Heltec Wireless Tracker | ESP32-S3 | SX1262 | UC6580 | TFT 160x80 (ST7735R) | - |
| LilyGo T-Beam v1.2 | ESP32 (PICO-D4) | SX1262 | Yes | - | AXP2101 PMU; contacts capped at 160 |
| TTGO LoRa32 | ESP32 (PICO-D4) | **SX1276** (loramac-node) | - | - | SX127x reference board |
| XIAO MG24 | EFR32MG24 | SX1262 | - | - | - |
| Seeed LoRa-E5 mini | STM32WL | STM32WL sub-GHz (SX1262-class) | - | - | UART companion/CLI; contacts capped at 24 |

Contact capacity is `CONFIG_ZEPHCORE_MAX_CONTACTS` (default 350) unless capped per-board as noted. Native-Linux presets (Femtofox, RAK6421) are `EXTRA_CONF_FILE` presets, not boards — see `LINUX_NATIVE.md`.

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
Over USB CDC (and native-Linux TCP): framed with a length prefix — `[2B LE length] [1B opcode] [payload...]` (TCP additionally prefixes a `<`/`>` direction byte).

### Key Command Opcodes (phone → device)

The full set (~50 opcodes, `0x01`–`0x41`) is defined at the top of `app/CompanionMesh.cpp`; values match the Arduino MeshCore companion protocol. A sample:

| Opcode | Name | Payload |
|--------|------|---------|
| 0x01 | CMD_APP_START | app version + name (session start) |
| 0x02 | CMD_SEND_TXT_MSG | txt_type + attempt + timestamp + pubkey_prefix + text |
| 0x04 | CMD_GET_CONTACTS | [optional 4B `since` lastmod filter] |
| 0x05 / 0x06 | CMD_GET/SET_DEVICE_TIME | (none) / 4B epoch (forward-only) |
| 0x07 | CMD_SEND_SELF_ADVERT | [optional type byte: flood/zero-hop] |
| 0x08 | CMD_SET_ADVERT_NAME | name string |
| 0x0A | CMD_SYNC_NEXT_MESSAGE | (none) — offline queue peek/confirm |
| 0x0B | CMD_SET_RADIO_PARAMS | freq + bw + sf + cr |
| 0x16 | CMD_DEVICE_QUERY | app target version |
| 0x21–0x23 | CMD_SIGN_START / DATA / FINISH | 3-phase Ed25519 signing (up to 8KB) |

### Push Notifications (device → phone, async)

Codes `0x80`–`0x90` (`PUSH_CODE_*` in `app/CompanionMesh.h`). Most used:

| Code | Name |
|------|------|
| 0x80 | PUSH_CODE_ADVERT |
| 0x81 | PUSH_CODE_PATH_UPDATED |
| 0x82 | PUSH_CODE_SEND_CONFIRMED |
| 0x83 | PUSH_CODE_MSG_WAITING |
| 0x8A | PUSH_CODE_NEW_ADVERT |

---

## 13. Data Storage

### File Paths

| Path | Content | Format |
|------|---------|--------|
| `/lfs/_main.id` | Node identity | 64B private key + 32B public key |
| `/lfs/new_prefs` | Companion preferences | 152B binary, field-by-field (Arduino-compatible superset) |
| `/lfs/contacts3` or `/ext/contacts3` | Contacts | 152B × N records |
| `/lfs/channels2` or `/ext/channels2` | Channels | 68B × N records |
| `/lfs/adv_blobs` or `/ext/adv_blobs` | Advert cache | Fixed-size blob records |
| `/lfs/repeater/*` | Repeater/room-server identity + prefs | 297B prefs; atomic-replace writes |
| `/lfs/repeater/acl` | Client ACL | 136B × N records |
| `/lfs/repeater/regions2` | Region map | Header + 164B × N entries |
| `storage_partition` (NVS, 0xD0000 nRF52) | BLE bonds + Zephyr settings | NVS settings backend (≥1.16.2; old `/lfs/settings` file detected by self-heal) |

### Preferences Binary Layouts

Two distinct field-by-field serializations (NOT raw struct dumps), both Arduino-compatible
in their shared base fields:

**Companion `/lfs/new_prefs` (152 bytes)** — `adapters/datastore/ZephyrDataStore.cpp`
`loadPrefs()`/`savePrefs()` (offset comments inline). Arduino companion layout (name, lat/lon,
radio params, telemetry modes, BLE pin, GPS, autoadd) plus ZephCore extensions from offset 92:
rx_boost(92), leds_disabled(93), reserved(94-95, was APC), default flood scope name/key(96-142),
ble_disabled(143), display/wake/screen-off/auto-shutdown(144-149), rx_duty_cycle(150),
meshtimesync(151).

**Repeater/room-server `/lfs/repeater/prefs` (297 bytes)** — `app/RepeaterDataStore.cpp`
`loadPrefs()`/`savePrefs()` (same field order as `helpers/CommonCLI.cpp`; offset comments inline).
Key ranges: name(4-36), radio(72-119), adaptive-delay(80-111, ignored at runtime),
leds_disabled(120, magic-encoded `0xA0`/`0xA1` — the byte formerly held `agc_reset_interval`, which
stored seconds/4, so any other value is a legacy interval and decodes to "LEDs on"),
Arduino-bridge(127-151, read+discarded), GPS(156-161), owner_info(170-290), rx_boost/duty(290-291),
reserved(292-293, was APC), flood_max_unscoped/advert(294-295), meshtimesync(296). Older shorter files
load cleanly — reads past EOF are no-ops, so newer fields keep their defaults and a one-time
upgrade block migrates them.

---

## 14. Key Call Flows

### 14.1 Receiving a LoRa Packet → Application

```
DIO1 interrupt → Zephyr lora driver → async RX callback
  → LoRaRadioBase::rxCallbackStatic() → SPSC ring buffer write → _rx_cb()
    → k_event_post(MESH_EVENT_LORA_RX) → main thread wakes
      → Dispatcher::loop() → checkRecv() → drain ring buffer
        → tryParsePacket() → score + airtime calc
          → flood: dedup + adaptive contention delay → queue for retransmit
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
