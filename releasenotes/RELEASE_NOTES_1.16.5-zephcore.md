# ZephCore 1.16.5-zephcore

> [!IMPORTANT]
> ## Before you upgrade
>
> **ESP32-S3 / ESP32-C boards — one-time serial reflash required; WiFi-OTA cannot bridge this update.**
> This release moves the application to a new flash location (`0x10000`). A board on the old layout can't
> update to this build over WiFi-OTA or the browser flasher — **flash the `-merged.bin` once over USB/serial.**
> Affected boards: *Heltec V3 / V4 / V4.3, Station G2, Wireless Tracker / V2, XIAO ESP32-S3 / C3 / C6, LilyGo T-Lora C6.*
> Your identity, contacts, channels, prefs, and BLE bonds are preserved. After this reflash, OTA and
> app-updates work again.
>
> **nRF52, classic ESP32 (T-Beam / PICO-D4 / TTGO LoRa32), STM32WL, and native Linux are unaffected** — upgrade as usual.
>
> **From v1.16.2 / v1.16.3 / v1.16.4** — clean flash, no re-bond, bonds and data survive.
>
> **From v1.16.1 or older** — flash it; on first boot it clears BLE bonds automatically (identity, contacts, channels, prefs preserved). Re-bond your phone/desktop once.
>
> **Coming from Arduino MeshCore** — flash it; auto-formats on first boot (new identity, clean storage).
>
> Take this with a grain of salt — try the formatters if anything anomalous happens with your node.

---

The headline of this release:
**Adaptive CAD**, a self-tuning listen-before-talk that calibrates itself to each site;
**V-Contact**, an in-app admin "channel" for companions; and a batch of LoRa-driver reliability fixes.

## Highlights

### Fixed: ESP32 WiFi-OTA silently reverted — upload hit 100%, then booted the *old* firmware
On ESP32-S3 / ESP32-C boards, the app and MCUboot bootloader were using different flash partition tables,
so each OTA update landed at an address the bootloader never checked. Every upload "succeeded" and then
quietly did nothing. Both images now share one partition map, and OTA works correctly. (The XIAO C3/C6 and
T-Lora C6 used the stock layout and were never affected.)

### ESP32 app slot moved to `0x10000`
The ESP32 app slot moved from `0x20000` to `0x10000` — matching the Arduino / ESP-IDF offset. This is the
change that fixes OTA above, and it also enables the configurator's fast **app-only update** (rewrites just
the app, keeps your settings). Requires the one-time serial reflash (see *Before you upgrade*).

### New: Adaptive CAD — listen-before-talk that tunes itself to your site
Every transmission is preceded by a Channel Activity Detection (CAD) check that listens for other LoRa
traffic and holds off if the channel is busy. How sensitive that check is has always been a single
hardcoded number — but the right value depends on where the node lives. A quiet valley node and a
50-network hilltop need very different thresholds: too sensitive and the node keeps deferring for phantom
traffic (wasted airtime, latency); not sensitive enough and it transmits over real packets (collisions).

Nodes now **measure their own RF environment and adjust automatically**. In the background, between
transmissions, the radio runs short calibration probes, tracks how often each sensitivity level produces a
false "busy", and settles on the lowest false-positive threshold for *that* site — responding to a
changing environment within an hour or two. It runs on repeaters and companions, and it is **on by
default**; there is nothing to configure.

- `get cad` — watch what it's doing: the current threshold and per-level false-positive statistics.
- `set cad.auto off` — turn off the auto-adjust to observe or hand-tune with `set cad.offset`.

Quiet sites end up *more* sensitive than the old fixed default (fewer stomped packets); noisy sites back
off until the false alarms stop. CAD also now uses 4 detection symbols everywhere, which improves detection
of an in-progress packet's payload (not just its preamble). Full guide, including the honest limitation
that the "missed detection" side isn't locally observable, is in
**[ADAPTIVE_CAD.md](https://github.com/liquidraver/ZephCore/blob/master/zephcore/ADAPTIVE_CAD.md)**.
(SX127x boards have no hardware CAD and keep their RSSI-based gate.)

### New: V-Contact — admin your companion from the chat app, no cable
Companions now expose a special contact named `v<your-node-name>`. Chat with it and your messages run the
same text CLI you'd use over USB serial — the reply comes back as ordinary chat messages. It's the easiest
way to check or change settings on a companion in the field without plugging in.

It's also used for two unsolicited notices: a **restart-reason message** after a reboot (so an unexpected
brownout or watchdog reset is visible instead of silent) and a **one-shot low-battery alert**. The whole
thing is local to your device — the v-contact has no real keypair and **never creates a packet or touches
the radio**, so none of this costs airtime. On by default; toggle the `v_contact_enabled` variable from the
app to disable.

### LoRa driver reliability (LR1110 / LR2021)
- **Fixed: LR1110 accepted CRC-failed packets.** On the LR11xx/LR20xx family a CRC-failed receive raises
  `RX_DONE` and `CRC_ERROR` together; the driver was acting on `RX_DONE` alone and passing corrupt frames
  up the stack. It now gates on the error bits (matching the SX126x path).
- **Ported the SX126x preamble-grace / RX-busy handling to LR11xx/LR20xx.** A foreign or noise-triggered
  preamble detection could pin the channel-busy state and stall listen-before-talk recovery. The same
  SF-aware grace window and header-valid latch the SX126x driver uses now guard the LR path, so a real
  packet promotes to "receiving" and a false preamble releases after a bounded window.

## Other fixes and improvements

- **Classic ESP32 companion builds** (TTGO LoRa32, T-Beam) now build and link correctly. The Espressif BT
  HCI controller node wasn't wired on TTGO LoRa32 (companion needs BLE), and the CLI backing the V-Contact
  chat wasn't compiled on plain (non-logging) companion builds — both fixed. Contact-table sizes on these
  DRAM-tight classic-ESP32 boards were trimmed to fit alongside the BLE controller.
- **SenseCAP Solar** LED devicetree fix.
- **LilyGo T-Impulse Plus** display fixes (follow-up to the initial port).
- **Repeater + observer hybrid** role wiring fixups (MQTT publisher / uplink credentials / observer mesh).
- **`get`/`set` command keywords are now case-insensitive** (e.g. `Get cad` works). Command *values* —
  passwords, node names, keys — remain case-sensitive as they must be.
- **Platform: Zephyr west manifest advanced** (continuing along the `v4.4-branch`).
- **New `-update` build artifacts** for OTA / manual app-only updates, published alongside the full images.
- **Docs:** new `ADAPTIVE_CAD.md`; `Repeater_CLI_commands.md`, `ARCHITECTURE.md`, and `PROVIDER_CATALOG.md`
  updated for the above.

## Recommended upgrade checklist

1. **ESP32-S3 / ESP32-C boards:** flash `-merged.bin` once over USB/serial. WiFi-OTA can't cross this update; data survives.
2. **Everything else, from v1.16.2 / v1.16.3 / v1.16.4:** just flash — bonds and data survive.
3. **From v1.16.1 or older:** just flash — self-migrates on first boot; re-bond once.
4. **From Official MeshCore:** just flash — auto-formats on first boot.
5. **Anything odd?** Format first.
