<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="img/kite-network-logo-thick-bright.svg">
    <img src="img/kite-network-logo-thick-dark.svg" alt="ZephCore logo" width="140">
  </picture>
</p>

<h1 align="center">ZephCore</h1>

<p align="center"><b>MeshCore, but it runs on Zephyr.</b></p>

<p align="center">
  <a href="https://github.com/liquidraver/ZephCore/actions/workflows/build.yml"><img src="https://github.com/liquidraver/ZephCore/actions/workflows/build.yml/badge.svg" alt="build"></a>
  <a href="https://github.com/liquidraver/ZephCore/actions/workflows/tests.yml"><img src="https://github.com/liquidraver/ZephCore/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
  <a href="https://github.com/liquidraver/ZephCore/releases/latest"><img src="https://img.shields.io/github/v/release/liquidraver/ZephCore" alt="latest release"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-blue" alt="MIT"></a>
</p>

<p align="center">
  <a href="https://zephcore.meshcore.dev">Flash it</a> ·
  <a href="docs/supported_boards.md">Boards</a> ·
  <a href="docs/Repeater_CLI_commands.md">CLI</a> ·
  <a href="docs/DESIGN.md">Design</a> ·
  <a href="releasenotes/">Release notes</a>
</p>

---

ZephCore is firmware for [MeshCore](https://github.com/meshcore-dev/MeshCore/) LoRa mesh nodes. It talks to
the same mesh, pairs with the same MeshCore apps and answers the same CLI commands as the original firmware.
What is different is underneath: instead of Arduino, it is built on [Zephyr RTOS](https://zephyrproject.org/).

If you already run MeshCore, a ZephCore node is just another node on your mesh. Your neighbours will not
notice, and neither will your phone.

It is a community port, not the official firmware. The protocol and the apps belong to the MeshCore
project; ZephCore follows them and does not change what goes over the air.

> [!NOTE]
> About 99% of this code was written by AI. A human decides what gets built and tests it on real radios.
> More on that [at the bottom](#who-wrote-this).

## Get it on your device

1. **Web flasher** (easiest): open [zephcore.meshcore.dev](https://zephcore.meshcore.dev) in Chrome or
   Edge, pick your board and role, plug in, flash.
2. **Mesh America Device Configurator**: [apps.meshamerica.com](https://apps.meshamerica.com) carries the
   same builds.
3. **By hand**: download from [Releases](https://github.com/liquidraver/ZephCore/releases). nRF52 boards
   take the `.uf2` by drag and drop; ESP32 boards take the `-merged.bin` at offset 0 with esptool.

> [!WARNING]
> Coming from stock MeshCore (or going back)? On nRF52 the bootloader stays the same, so there is nothing
> extra to flash. But settings do not carry over between the two firmwares in either direction: expect to
> set the node up again.

Then pair with the MeshCore app as usual (default BLE PIN `123456`). Not sure your board is covered? Check
the [supported boards list](docs/supported_boards.md).

## What a node can be

| Role | What it does | How you talk to it |
|---|---|---|
| **Companion** | Your personal node: contacts, channels, offline message queue | MeshCore app over BLE or USB (WiFi on some ESP32 boards) |
| **Repeater** | Relays packets for everyone else | [Text CLI](docs/Repeater_CLI_commands.md) over USB, or remote admin over the mesh |
| **Room server** | A shared message board that keeps posts for clients who were away | Same CLI as the repeater; users log in from the app |
| **Observer** | Listens only and publishes what it hears to MQTT over WiFi (ESP32) | Same CLI |

One role per firmware image; you choose it when you flash.

## What you get on top of stock MeshCore

All of this is local behaviour. Nothing here changes the wire protocol, so mixed meshes work fine.

- **Self-tuning retransmit timing.** Repeaters measure how crowded their neighbourhood is and space their
  retransmits to match, instead of using fixed `txdelay` values. ([ADR 0007](docs/adr/0007-adaptive-contention-window.md))
- **Adaptive listen-before-talk.** Channel-activity detection calibrates itself to the local noise.
  ([ADAPTIVE_CAD.md](docs/ADAPTIVE_CAD.md))
- **Optional RX duty cycle.** The radio chip itself naps between preamble checks to cut receive current,
  on radios that support it. ([ADR 0006](docs/adr/0006-sx126x-rx-duty-cycle-and-busy-gating.md))
- **Mesh time sync.** Nodes without GPS can agree on the time. Off by default.
  ([MESHTIMESYNC.md](docs/MESHTIMESYNC.md))
- **An observer role.** A listen-only node that publishes everything it hears to MQTT over WiFi (ESP32).
- **MQTT uplink on a repeater.** An ESP32 repeater can report what it hears to MQTT over WiFi while it
  keeps repeating.
- **LEDs you control.** Turn them all off with `set leds off`, or choose what each one shows: blink on
  transmit, on receive or both, and a heartbeat that can signal unread messages. The receive blink tells
  you at a glance whether a repeater is hearing anything.
- **2.4 GHz on LR2021 boards.** Set a 2.4 GHz frequency and the radio switches to its high-band path on
  its own.
- **Runs on Linux too.** The same code runs as a normal process on a Raspberry Pi or Femtofox with a real
  SX1262 attached. ([LINUX_NATIVE.md](docs/LINUX_NATIVE.md))

## Hardware

<!-- Generated by `python zephcore/scripts/board_manifest.py docs`; edit the board manifests, not this table. -->
<!-- boards:summary -->
| Platform | Boards | With published firmware |
|---|---|---|
| nRF52840 | 22 | 21 |
| ESP32 family | 16 | 14 |
| nRF54L | 4 | 3 |
| EFR32MG24 | 1 | 1 |
| STM32WL | 1 | 1 |
| Native Linux (presets) | 3 | 3 |
<!-- /boards -->

RAK4631, SenseCAP T1000-E, Wio Tracker L1, Heltec T114 and V3/V4, XIAO ESP32-S3, nRF54L15 and MG24, Seeed
LoRa-E5, Femtofox, Raspberry Pi with a RAK6421 HAT, and more. The full list with exact build strings is in
[docs/supported_boards.md](docs/supported_boards.md). Want to add one? See the
[board porting guide](zephcore/boards/example_board/README.md).

## Build it yourself

The quickest start is the [dev container](.devcontainer/README.md): open the repo in VS Code and choose
**Reopen in Container**. Otherwise install the
[Zephyr SDK and west](https://docs.zephyrproject.org/latest/develop/getting_started/index.html), then:

```bash
# once
west init -l zephcore
west update

# a companion, then a repeater
west build -b wio_tracker_l1 zephcore --pristine
west build -b rak4631 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"

west flash
```

Roles and extras are config fragments in `zephcore/boards/common/`, added through `EXTRA_CONF_FILE`
(separate several with `;`):

| Fragment | Gives you |
|---|---|
| *(none)* | Companion |
| `repeater.conf` | Repeater |
| `room_server.conf` | Room server |
| `observer.conf` | Observer (ESP32) |
| `debug.conf` | Logging and asserts |
| `packet_logging.conf` | One clean RAW/RX/TX line per packet, nothing else |

Every other variant, flashing per platform (ESP32 blobs and `--sysbuild`, nRF54L, MG24) and the Kconfig
options are in [docs/BUILDING.md](docs/BUILDING.md).

## For Zephyr people

ZephCore is a plain west workspace application, and it tries to be a well-behaved one:

- **Zephyr is pinned to a `main` commit**, bumped deliberately. ([ADR 0005](docs/adr/0005-zephyr-main-pin.md))
- **A small patch set** in `zephcore/patches/zephyr` is applied at configure time and fails the build
  loudly if it stops applying. One patch owns each upstream file. If you maintain one of those files,
  that directory is the list of things we would love to see fixed upstream.
- **Radios**: the in-tree SX126x and SX127x drivers, plus out-of-tree LR11xx and LR20xx drivers, behind
  one adapter.
- **Boards are devicetree and Kconfig only.** A board directory declares hardware, never role policy;
  config is layered from `prj.conf` down to `board.conf`.
- **ESP32**: simple boot for companions, MCUboot with sysbuild where WiFi OTA is involved
  ([ADR 0001](docs/adr/0001-esp32-boot-and-flash-layout.md)); native USB on S3 companions
  ([ADR 0002](docs/adr/0002-esp32s3-native-usb-companion.md)).
- **Tests**: the mesh core builds and runs on the host under ASan and UBSan in CI.

## Documentation

| | |
|---|---|
| [DESIGN.md](docs/DESIGN.md) | The system view: roles, layers, rules |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | Component reference |
| [adr/](docs/adr/README.md) | Decisions and why they were made |
| [BUILDING.md](docs/BUILDING.md) | Every build variant, platform notes, Kconfig options |
| [Repeater_CLI_commands.md](docs/Repeater_CLI_commands.md) | Every CLI command |
| [supported_boards.md](docs/supported_boards.md) | Boards and build strings |
| [LINUX_NATIVE.md](docs/LINUX_NATIVE.md) | Running on a Pi or Femtofox |
| [releasenotes/](releasenotes/) | What changed in each release |

## Who wrote this?

Mostly robots. Around 99% of the code in this repository was written by AI (Claude and Cursor). One human
points, argues, says "no, not like that", and flashes the result onto a bench full of real radios to see
whether it actually works.

The ideas that matter are not ours either: the mesh protocol, the apps and most new features come from the
[MeshCore project](https://github.com/meshcore-dev/MeshCore/) and the people doing the hard thinking there.

What keeps it honest: changes are tested on real hardware before release (mostly nRF boards on our own
bench; the ESP32 boards are largely tested by the community, so reports are very welcome), the mesh core
has host tests in CI, and the decisions are written down in [ADRs](docs/adr/README.md) so the next robot
does not undo them.

[![TWONKS: AI Design Logic](img/twonks-ai-design.jpg)](https://www.twonks.co.uk/)

*Comic by [TWONKS](https://www.twonks.co.uk/). Borrowed with love; go read the rest.*

## License

MIT, same as upstream MeshCore; see [LICENSE](LICENSE), which also lists the vendored pieces (Monocypher,
Zephyr patches). Logo by [recrof](https://github.com/recrof), [WTFPL](https://en.wikipedia.org/wiki/WTFPL).
