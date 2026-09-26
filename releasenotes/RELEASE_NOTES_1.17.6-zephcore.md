# ZephCore 1.17.6-zephcore

A large release: companion firmware now speaks WiFi (TCP, on boards that support it), preferences
move to upstream's `prefs.json` format, and ESP32-S3 repeaters can finally light-sleep. Underneath,
the mesh core, radio drivers and companion protocol handling were rebuilt against upstream MeshCore's
text so the two codebases read the same again. Also: GPS reliability fixes, more accurate battery
reporting, reduced flash wear, and the remote CLI (protocol v14) now executes commands from admin
contacts, matching upstream.

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

## ESP32-S3 repeaters can light-sleep again

Light sleep on S3 repeaters has never actually worked, for two independent reasons, both now fixed:

- Every S3 repeater carries WiFi OTA, and the WiFi driver started WiFi at boot — which holds a modem
  power-management lock for as long as WiFi is up. Measured on a XIAO: zero sleep entries with WiFi
  compiled in. WiFi now stops at boot on power-management builds, and starts again on `start ota` or
  a station connect.
- The radio interrupt line (DIO1) was never a real wake source. Since 1.17.1, boards without a
  hardware RTC pad on DIO1 (XIAO GPIO39, Station G2 GPIO48) couldn't wake from sleep on an incoming
  packet at all. DIO1 and the user button are now armed as proper level-triggered GPIO wakes, and the
  interrupt a sleep swallowed is re-raised afterwards so the radio and button handlers still run.

Verified on a XIAO ESP32-S3 repeater against a T1000-E on the same channel: asleep 96.6% of uptime,
packet-for-packet parity with the awake baseline.

> [!NOTE]
> **Nothing to change on your side.** `powersaving` (already an upstream command) gates this and
> defaults to on for light-sleep-capable builds; a previously-stored "off" is respected. Companion
> light sleep is still off by default — it isn't ready to ship yet.

## Less flash wear from contact updates and admin logins

The companion was marking its contact list dirty on every advert from a node it *declined* to
auto-add (full table, hop limit, wrong type) — forcing a full rewrite of the contact store roughly
every 5 seconds on a busy mesh, even though nothing was actually added. It now only dirties on a real
addition or a changed name, type or position; a plain re-advert just extends the node's liveness
deadline (now 1 hour instead of 10 minutes) without touching flash.

Repeaters and room servers likewise stopped rewriting their access-control list on every admin login
— it's now saved only when the role or secret actually changed.

## Remote CLI now executes on the device (protocol v14)

The companion protocol's v14 additions — `CMD_RUN_CLI_COMMAND`/`PACKET_CLI_REPLY`, and
`TXT_TYPE_CLI_COMMAND` sent *and executed* from a contact with the remote-CLI-allowed flag — are now
implemented in full, matching upstream MeshCore. Previously ZephCore advertised v14 but only accepted
the command types without executing anything sent over the air.

## GPS reliability

- Servers now honour `gps_enabled` at boot — it was previously lost across a reboot. `zc.gps_set`
  upgrades older stored prefs to "on" once, so this doesn't silently turn GPS off for anyone.
- GPS can be turned off on boards with no power-control line, by putting the UART to sleep instead.
- The sleep lock is held only while a fix is being acquired, not indefinitely.
- The observer role now parks its GPS instead of leaving it running unconfigured.

## Battery and power reporting

- Both UIs now show battery percentage from `ZephyrBoard::getBattPercent()` — a discharge curve or
  fuel-gauge reading — instead of a linear 3.0–4.2 V estimate, which was often noticeably wrong.
- Every companion, including headless ones with no display, now has low-battery auto-shutdown with a
  `v`-contact alert beforehand, and an `autoshutdown` CLI command to control it. Below 2000 mV counts
  as "no battery installed" rather than critically low.
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