# ZephCore 1.17.6a-zephcore

A large release: companion firmware now speaks WiFi (TCP, on boards that support it), preferences
move to upstream's `prefs.json` format, and ESP32-S3 repeaters can finally light-sleep. Underneath,
the mesh core, radio drivers and companion protocol handling were rebuilt against upstream MeshCore's
text so the two codebases read the same again. Also: GPS reliability fixes, more accurate battery
reporting, reduced flash wear, and the remote CLI (protocol v14) now executes commands sent by
contacts you have allowed to use it, matching upstream.

> [!IMPORTANT]
> **This is a pre-release.** The Mesh America configurator keeps offering 1.17.5c; flash this one by
> hand. ESP32-S3 light sleep was bench-validated on a XIAO ESP32-S3; on the Heltec boards that
> enable it by default it is build-tested only, so reports from Heltec repeaters are especially
> welcome (`get pm`, see below).

> [!NOTE]
> A normal upgrade keeps your identity, settings, contacts and phone pairing. Preferences are
> migrated to the new `prefs.json` format on first boot; the old binary file is kept alongside it, so
> a downgrade still boots with your existing settings.

---

## Companion over WiFi

Boards that declare `capabilities: wifi` in their manifest — PSRAM ESP32-S3 boards, Heltec WiFi LoRa
32 V3 (140 contacts), Wireless Tracker (130), and both C6 boards (280) — can now run the companion
app over WiFi instead of BLE/USB, using upstream's `wifi.ssid`/`wifi.pwd`/`wifi.enabled` CLI and TCP
port 5000. ESP32-S3 boards without PSRAM get bounded WiFi buffers and a measured 72 KB heap; PSRAM
boards keep a 72 KB internal heap.

Getting there also fixed two ESP32 WiFi stability bugs that showed up under load (~200+ frames/s
unicast): the WiFi driver's RX callback could block the WiFi task, and the WiFi heap living in PSRAM
could corrupt the shared multi-heap. Both are fixed via upstream Zephyr/hal_espressif patches.

## Preferences are now `prefs.json`

Settings move from ZephCore's own binary layout to upstream's `ConfigSerializer`-based `prefs.json`,
using upstream's key names for shared fields and ZephCore's own under a `zc` namespace. On first boot
after upgrading, the old binary prefs (companion or repeater/room-server) are read once and migrated;
the old file is left in place, so a downgrade still finds its settings.

Three related bugs were fixed along the way:

- A failed write could leave a server's prefs file truncated. Identity, prefs and channel writes now
  go through one checked atomic replace.
- A corrupted `cad_auto` byte turned adaptive CAD off instead of falling back to its default of on.
- A repeater's GPS duty cycle, deliberately set to 300 seconds, was being silently rewritten back to
  48 hours on every boot. That migration path now only runs against legacy prefs files.

## ESP32-S3 repeaters now actually light-sleep

Repeater builds for the Heltec WiFi LoRa 32 V3, V4 and V4.3 and the Wireless Tracker and Wireless
Tracker V2 have shipped with light sleep enabled, but it never worked, for two independent reasons,
both now fixed:

- Every S3 repeater carries WiFi OTA, and the WiFi driver started WiFi at boot — which holds a modem
  power-management lock for as long as WiFi is up. Measured on a XIAO: zero sleep entries with WiFi
  compiled in. WiFi now stops at boot on power-management builds, and starts again on `start ota` or
  a station connect.
- The radio interrupt line (DIO1) could not wake the chip. The wake mechanism used (ESP32 "EXT1")
  allows only one trigger polarity for all its pins; since 1.17.1 the active-low user button was
  armed on it as well, and that silently disarmed the active-high DIO1. DIO1 and the user button are
  now per-pin level wakes that work on any pin, and the interrupt a sleep swallowed is re-raised
  afterwards so the radio and button handlers still run. Boards with DIO1 on a non-RTC pin (XIAO
  ESP32-S3, Station G2) can now light-sleep too; they are not switched on by default yet.

The pins the radio depends on (chip select, reset, RF switch, FEM enable and supply) are now held at
their level through each sleep: light sleep otherwise lets every unheld pin float while the radio
keeps receiving.

Verified on a XIAO ESP32-S3 repeater on a live mesh against a T1000-E on the same channel: asleep
96.6% of uptime, 119 packets heard vs 119, and transmit counters correct.

> [!NOTE]
> **Nothing to change on your side.** `powersaving` (already an upstream command, stored but ignored
> until now) gates light sleep. It defaults to on for light-sleep builds, and the "off" older firmware
> stored without acting on it is read as on once. `powersaving off` keeps the node awake and is kept
> across reboots. `get pm` shows how much of the time the node sleeps and what wakes it.
>
> **USB console:** for 10 minutes after boot, and again after every press of the user button, the node
> stays awake and the USB console works normally. After that, characters typed at a sleeping node are
> lost, and on boards whose console is the chip's own USB port (V4, V4.3, Wireless Tracker V2) the USB
> device disconnects and reconnects as the chip sleeps and wakes. Press the button, or send
> `powersaving off` over LoRa, to get the console back. Remote admin over LoRa is unaffected.
>
> Companion light sleep is still off by default — it isn't ready to ship yet.

## Less flash wear from contact updates and admin logins

The companion was marking its contact list dirty on every advert from a node it *declined* to
auto-add (full table, hop limit, wrong type) — each such advert forced a full rewrite of the contact
store 5 seconds later, even though nothing was actually added. It now only dirties on a real
addition or a changed name, type or position; a plain re-advert just extends the node's liveness
deadline (now 1 hour instead of 10 minutes) without touching flash.

Repeaters and room servers likewise stopped rewriting their access-control list on every admin login
— it's now saved only when the role or secret actually changed — and a preferences save that would
write exactly what is already stored is skipped.

Pending writes are now flushed before every clean reboot and power-off (CLI, UI, bootloader and OTA
paths), not only before the companion app's reboot command.

## Remote CLI now executes on the device (protocol v14)

The companion protocol's v14 additions — `CMD_RUN_CLI_COMMAND`/`PACKET_CLI_REPLY`, and
`TXT_TYPE_CLI_COMMAND` sent *and executed* from a contact with the remote-CLI-allowed flag — are now
implemented in full, matching upstream MeshCore. Previously ZephCore advertised v14 but only accepted
the command types without executing anything sent over the air.

## GPS reliability

- Servers now honour `gps_enabled` at boot — it was previously lost across a reboot. `zc.gps_set`
  upgrades older stored prefs to "on" once, so this doesn't silently turn GPS off for anyone.
- GPS can be turned off on boards with no power-control line, by sending the module its sleep
  command over the UART instead.
- On nRF boards the GPS UART now also stops listening whenever the GPS is off or in standby on boards
  without a power-control line (RAK4631, RAK3401 1W, GAT562 30S, R1 Neo, XIAO nRF54L15), as it already
  did on the others. Tested with 900 suspend/resume cycles into a streaming GPS.
- XIAO nRF54L15: an optional GPS can be wired to the header serial pins (D6 = TX, D7 = RX, 9600 baud
  NMEA) and works with the release firmware.
- On ESP32, light sleep is held off only while a fix is being acquired, not indefinitely.
- The observer role now parks its GPS instead of leaving it running unconfigured.

## Battery and power reporting

- Both UIs now show battery percentage from `ZephyrBoard::getBattPercent()` — a discharge curve or
  fuel-gauge reading — instead of a linear 3.0–4.2 V estimate, which was often noticeably wrong.
- Every companion, including headless ones with no display, now has low-battery auto-shutdown with a
  `v`-contact alert beforehand, and an `autoshutdown` CLI command to control it. Below 2000 mV counts
  as "no battery installed" rather than critically low.
- New upstream commands: `poweroff`/`shutdown` (on nRF, wake with the user button; elsewhere, a power
  cycle) and `get pwrmgt.support`, `.source`, `.bootreason`, `.bootmv`.
- `get pwrmgt.bootreason` no longer reports "Unknown" after a flash: ESP32 reset reasons that Zephyr's
  `hwinfo` driver maps to 0 (USB-Serial-JTAG reset, power glitch, CPU lockup, eFuse) are now labelled
  correctly.

## Admin screen telemetry

The admin screen's telemetry decoder is now upstream's `LPPDataHelpers` (with its 1-byte humidity
size bug fixed), showing every field per channel, including signed current readings, instead of a
partial ZephCore-specific decode.

## Doom

- Fixed a crash after roughly 10 turns: the view direction slowly lost precision every turn until a
  wall's on-screen height rounded to zero and was divided by, rebooting the device. Turning now
  renormalises the view vector.
- The level-1 demon spawned inside a wall and could never be shot, so the exit never opened. It now
  spawns in the open, mirroring the imp.
- The 60-second UI lock could engage mid-game, so the exit button opened the lock screen instead of
  quitting. Doom now keeps the lock and auto-off timers alive on its own.
- Sprites are now drawn with a one-pixel black halo so they stay visible against similarly-lit
  backgrounds.

---

## Also in this release

- **XIAO ESP32-S3 (Wio-SX1262 kit): user button fixed.** GPIO21 is both the kit's button and the
  XIAO's onboard LED, and the heartbeat LED kept driving the button pin. The heartbeat LED is gone on
  this board; the button now has a pull-up and works.
- **Station G2: user button polarity fixed** (it was read inverted, as active-high; upstream reads it
  active-low).
- **Radio drivers were unified.** SX126x, SX127x, LR1110 and LR2021 now share one `LoRaRadio` adapter
  over per-family operation tables instead of four separate classes. The LR1110 and LR2021 SDKs now
  compile from Semtech's LoRa Basics Modem source instead of a vendored fork.
- **A CAD receive-window bound could wrap instead of saturate** at SF12 with a 7.81 kHz bandwidth,
  shared logic between SX126x and the newer radios now clamps correctly.
- **Zephyr bumped to `3e7672a71bd`** (main). In the process, a Bluetooth address-formatting change
  upstream would have orphaned every stored phone pairing and forced a re-pair on every bonded phone;
  ZephCore now builds the persisted BLE settings key itself in the old format, so this was avoided
  before it could ship.
- **The mesh core (Mesh, Dispatcher, Packet, packet pool) was rebuilt from upstream MeshCore's text**,
  with ZephCore's changes re-applied and fenced as `// ZEPHCORE:` comments, cutting the diff against
  upstream from ~4,540 lines to ~1,147. Companion command handling (`BaseChatMesh`, per-command
  handlers) went through the same treatment.
- **A host-side regression suite** (114 cases over production sources) and CI with ASan/UBSan now run
  on every PR and push to `master`.
- **Board manifests**: every board now has a single `zephcore.yml` describing its release role,
  variants and capabilities, replacing board-name regex lists scattered across the build scripts.
- **System design docs published**: `docs/DESIGN.md` and nine architecture decision records covering
  the choices made in this restructure.