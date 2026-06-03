# ZephCore Native Linux Port

Run ZephCore as a native Linux process on SBCs like the **Femtofox** (Luckfox Pico Mini + E22-900M30S) or a **Raspberry Pi + RAK6421 HAT**. The full mesh stack runs on top of Zephyr's `native_sim` board, talking to real SPI/GPIO via `/dev/spidev*` and the kernel's **GPIO V2 character-device uAPI** (direct ioctls — no libgpiod dependency).

The companion app connects via a TCP socket on port **5000**, using MeshCore's `SerialWifiInterface` framing (`['<'][len_LSB][len_MSB][payload]` app→node, `['>']…` node→app) — see [Companion app connection](#companion-app-connection).

> **Real-time clock is automatic.** The binary forces the `native_sim` simulated clock into real-time mode at boot (equivalent to always passing `--rt`), because it drives a *real* radio whose BUSY/DIO1 timing happens in wall-clock time. You do **not** need to pass `--rt`. See [adapters/transport/linux_native_setup.c](adapters/transport/linux_native_setup.c).

---

## Prerequisites

On the **build host** (where you run `west build`):

```bash
# Zephyr SDK and west workspace as per the main CLAUDE.md.

# Cross-compile toolchains (for SBC targets) — install BOTH gcc and g++:
# the mesh layer is C++, so the g++ cross-compiler is required (a build with
# only gcc fails at CMake configure with "Tell CMake where to find the compiler").
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf   # Femtofox (RV1103 ARMv7-A)
sudo apt install gcc-aarch64-linux-gnu   g++-aarch64-linux-gnu     # Raspberry Pi (aarch64)
```

On the **target SBC** (where the binary runs): grant your user access to
`/dev/spidev*` and `/dev/gpiochip*` so you don't need `sudo`/root — see
[Running without root](#running-without-root) below. (You *can* just run as
root with `sudo`, but it's not required.)

**Femtofox only:** the Femtofox image ships with `meshtasticd` pre-installed and it holds
the SPI bus / GPIO lines. Uninstall it before running ZephCore or the radio will be
unavailable. Use `foxbuntu-config` → uninstall meshtasticd, or:

```bash
sudo systemctl stop meshtasticd
sudo apt remove meshtasticd
```

No userspace library dependencies — the GPIO driver talks to `/dev/gpiochipN`
directly via the kernel's GPIO V2 chardev uAPI (ioctl). Requires Linux ≥ 5.10
(released Dec 2020 — every modern SBC distro has it). The SPI driver uses
spidev the same way (no library either).

### Enabling spidev on the SBC

**Femtofox / Luckfox Pico Mini:** the official Femtofox image ships with SPI and GPIO
already enabled — `/dev/spidev0.0` and `/dev/gpiochip0`–`3` are present out of the box.
No extra configuration needed; skip straight to adding your user to the `spi`/`gpio` groups.

**Raspberry Pi:** add `dtparam=spi=on` to `/boot/firmware/config.txt` (RPi 5) or `/boot/config.txt` (RPi 4) and reboot. Verify `/dev/spidev0.0` exists.

---

## Running without root

By default `/dev/spidev*` and `/dev/gpiochip*` are owned `root:root`, so the
binary only runs under `sudo`. The binary needs **no** elevated privileges of
its own — it only needs read/write on those two device classes (the TCP port is
5000, non-privileged; the flash file is written in your home dir). Grant access
once with groups + a udev rule:

```bash
# 1. Create the groups (the Femtofox/foxbuntu image ships without them).
sudo groupadd -f spi
sudo groupadd -f gpio

# 2. Add your login user (e.g. "femto") to both.
sudo usermod -aG spi,gpio "$USER"

# 3. udev rule: give those groups access to the SPI + GPIO char devices.
sudo tee /etc/udev/rules.d/90-zephcore.rules >/dev/null <<'EOF'
KERNEL=="spidev*",                 GROUP="spi",  MODE="0660"
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
EOF

# 4. Reload udev and re-trigger so the rule applies to the existing nodes.
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=spidev --subsystem-match=gpio

# 5. Log out and back in (group membership is only picked up at login).
#    Verify:  id        # should list "spi" and "gpio"
#             ls -l /dev/spidev0.0 /dev/gpiochip1    # group spi / gpio, mode 0660
```

Then run the binary **without** `sudo`:

```bash
./zephcore_native_linux.exe
```

> **If the udev rule doesn't stick** (some minimal images use `mdev`/busybox
> instead of systemd-udev, which ignores `/etc/udev/rules.d`), set the
> permissions at boot instead — e.g. a tiny systemd unit or an `rc.local` line:
> ```bash
> chgrp gpio /dev/gpiochip* && chmod 660 /dev/gpiochip*
> chgrp spi  /dev/spidev*   && chmod 660 /dev/spidev*
> ```

> **Femtofox note:** you must still stop/uninstall `meshtasticd` first (it holds
> the SPI bus) — see the Prerequisites section. Running as your user vs root
> doesn't change that.

---

## Quick Start

### 1. Host smoke build (x86-64 Linux, no real radio)

A device preset is required (it provides the SX1262 node); the Femtofox preset
is fine for a host smoke build — the SPI/GPIO opens just fail gracefully with no
hardware:

```bash
rm -rf build && west build -b native_sim/native/64 zephcore -- \
  -DEXTRA_CONF_FILE="boards/linux_native/femtofox.conf"
./build/zephyr/zephcore_native_linux.exe
```

The binary prints something like:

```
UART_0 connected to pseudotty: /dev/pts/3
TCP companion transport listening on :5000
```

`Ctrl-C` to stop.

### 2. Femtofox (Luckfox Pico Mini, ARMv7-A)

Note: use `native_sim` (32-bit), **not** `native_sim/native/64` — the Luckfox Pico Mini
is ARMv7-A (32-bit) and the 64-bit variant will error at CMake configure time.

```bash
west build -b native_sim zephcore -- \
  -DZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
  -DNATIVE_TARGET_HOST=arm \
  -DCROSS_COMPILE=/usr/bin/arm-linux-gnueabihf- \
  -DEXTRA_CONF_FILE="boards/linux_native/femtofox.conf"

scp build/zephyr/zephcore_native_linux.exe femto@femtofox.local:~/
ssh femto@femtofox.local ./zephcore_native_linux.exe   # no sudo once groups are set up
```

(No `--rt` flag needed — real-time mode is forced at boot. See the note at the top.)

### 3. Raspberry Pi + RAK6421 HAT (aarch64)

The `rak6421` preset is named after the **HAT**, not a Pi model — the wiring is
identical on every Pi. The only per-model difference is the Linux gpiochip:

- **Pi 2 / 3 / 4 / Zero 2** → `gpiochip0` → `rak6421.conf`
- **Pi 5** → `gpiochip4` (RP1) → `rak6421_pi5.conf`

The build command is unchanged by the native-Linux work this cycle — the
GPIO-driver fixes, automatic `--rt`, and per-node persistence apply to every
native build. For a 64-bit Pi OS (the usual case on Pi 3/4/5):

```bash
rm -rf build && west build -b native_sim/native/64 zephcore -- \
  -DZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
  -DNATIVE_TARGET_HOST=aarch64 \
  -DCROSS_COMPILE=/usr/bin/aarch64-linux-gnu- \
  -DEXTRA_CONF_FILE="boards/linux_native/rak6421.conf"      # Pi 5: rak6421_pi5.conf
scp build/zephyr/zephcore_native_linux.exe pi@raspberrypi.local:~/
```

(On a **32-bit** Pi OS — e.g. Pi 2, or Pi 3/4 on 32-bit Raspberry Pi OS — build the
armhf way instead: `-b native_sim -DNATIVE_TARGET_HOST=arm -DCROSS_COMPILE=/usr/bin/arm-linux-gnueabihf-`,
same `-DEXTRA_CONF_FILE`.)

`rak6421_pi5` is just `rak6421` with `gpiochip4` — equivalently you can keep using
`rak6421.conf` and override at runtime with `--lora-gpio-chip=/dev/gpiochip4`.

> **`--pristine` in this WSL workspace:** prefer `rm -rf build &&` over `west build --pristine`.
> `--pristine` can feed Windows-style `D:/…` paths to CMake when `.west/config` was
> initialised on Windows. `rm -rf build &&` is equivalent and works everywhere. (On a
> native-Linux build host, plain `--pristine` is fine.)
>
> **Status:** the RAK6421 preset (`rak6421.conf`, Pi Zero 2 W) is **hardware-verified
> end-to-end (2026-06-03)** — SX1262 init, SPI/CS, TCXO, radio-start, and over-the-air
> RX/TX all confirmed on real hardware, alongside the Femtofox (2026-06-02). The Pi 5
> preset (`rak6421_pi5.conf`) shares the same wiring (only the gpiochip differs) and is
> build-verified but not separately hardware-tested. If the radio is dead, follow the
> "Radio does nothing" troubleshooting below.
>
> The RAK13300/RAK13302 **does have a DIO3-powered TCXO and uses DIO2 as the RF
> switch** — both overlays set `dio3-tcxo-voltage`/`dio2-tx-enable` accordingly, and
> hold the module's two `Enable_Pins` (BCM 12 + 13) high via GPIO hogs
> (`CONFIG_GPIO_HOGS=y`). Pins and clock follow the RAK6421 IO Slot 1 reference
> wiring; see the wiring table below.

### 4. Repeater role (no companion)

Add `boards/common/repeater.conf` to the `EXTRA_CONF_FILE` list. The repeater CLI is accessible via the native PTY printed at boot (`/dev/pts/N`):

```bash
screen /dev/pts/3        # local
# or remotely:
socat /dev/pts/3 TCP-LISTEN:6000,reuseaddr,fork &
# then on a remote machine:
nc <sbc-ip> 6000
```

---

## Hardware Wiring

### Femtofox / Luckfox Pico Mini + E22-900M30S (TCXO)

Pin offsets are computed as `(Rockchip GPIO# − 32)` because the kernel splits banks into separate `gpiochip` devices.

| Signal | gpiochip | Offset | Rockchip GPIO |
|---|---|---|---|
| SPI bus | `/dev/spidev0.0` @ 2 MHz | — | — |
| CS | gpiochip1 | 16 | GPIO48 (1C0) |
| DIO1/IRQ | gpiochip1 | 23 | GPIO55 (1C7) |
| BUSY | gpiochip1 | 22 | GPIO54 (1C6) |
| RESET | gpiochip1 | 25 | GPIO57 (1D1) |
| RXEN | gpiochip1 | 24 | GPIO56 (1D0) |

SX1262 extras: DIO2 drives the RF switch (`dio2-tx-enable`), DIO3 powers the TCXO at 1.8V.

Source: `github.com/femtofox/femtofox` → `foxbuntu/.../femtofox_SX1262_TCXO.yaml`.

### Raspberry Pi + RAK6421 HAT + RAK13300/RAK13302 (IO Slot 1)

BCM GPIO numbers (Pi 2/3/4/Zero 2: `gpiochip0` → `rak6421.conf`; Pi 5: `gpiochip4` → `rak6421_pi5.conf`):

| Signal | BCM | WisBlock IO slot 1 |
|---|---|---|
| SPI bus | `/dev/spidev0.0` (CE0 = GPIO 8) | — |
| DIO1/IRQ | 22 | IO6 |
| RESET | 16 | IO4 |
| BUSY | 24 | IO5 |
| RX/TX enable | 12, 13 (`Enable_Pins`; 13 = IO3 antenna switch) | IO3 |

SX1262 extras: **DIO3 powers a 1.8 V TCXO** (`dio3-tcxo-voltage = <SX126X_DIO3_TCXO_1V8>`)
and **DIO2 drives the RF switch** (`dio2-tx-enable` / `DIO2_AS_RF_SWITCH: true`) —
same as the Femtofox. The two `Enable_Pins` (12, 13) are held high at boot via
GPIO hogs in the overlay (`CONFIG_GPIO_HOGS=y` in the `.conf`). The RAK13302 is the
1W SKY66122-boosted variant and additionally wants a per-step `TX_GAIN_LORA` PA
gain table; ZephCore's SX126x driver does not expose that, so this overlay drives
the 13300 fully and the 13302 at default (non-boosted) PA.

---

## Running the Binary

The binary prints its PTY path and TCP listen port at startup. Logs go to stderr; pipe them where you want them.

### Companion app connection

Use the MeshCore companion app's **"Connect via WiFi"** (TCP/Network) mode:

- **Host:** the SBC's IP address
- **Port:** `5000` (default; configurable via `CONFIG_ZEPHCORE_LINUX_TCP_PORT`)

Wire format: **MeshCore SerialWifiInterface framing** (matches ESP32 Arduino reference in `src/helpers/esp32/SerialWifiInterface.cpp`):
- App → Node: `['<' (0x3C)][length_LSB][length_MSB][NUS payload...]`
- Node → App: `['>' (0x3E)][length_LSB][length_MSB][NUS payload...]`

Only one client connects at a time.

### Repeater CLI

The repeater role exposes the same configuration CLI as an MCU repeater's USB-CDC
serial console — see [Repeater_CLI_commands.md](Repeater_CLI_commands.md) for the
full command list (`get`/`set`, `password`, `reboot`, `clock`, …). On native Linux
that console maps onto Zephyr's native-PTY UART (`CONFIG_SERIAL` +
`CONFIG_UART_NATIVE_PTY`). At boot the binary prints the pseudo-terminal it created:

```
uart connected to pseudotty: /dev/pts/3
```

That `/dev/pts/N` path **is** the serial console — point any serial terminal at it.

> **The PTY number changes on every run.** Grab it from the boot output each time
> (or from the journal when running as a service — see below). It is not stable
> across restarts.

#### Attach locally (same machine as the binary)

```bash
screen /dev/pts/3        # detach: Ctrl-a then d ; quit: Ctrl-a then k
# or
picocom /dev/pts/3
# or
minicom -D /dev/pts/3
```

You get the repeater prompt directly. Logs go to the binary's stderr, **not** the
PTY, so the CLI stays clean (no log spam interleaved with your typing).

#### Attach remotely (configure an SBC over the network)

Bridge the PTY to a TCP port with `socat`, then connect from your workstation:

```bash
# on the SBC:
socat /dev/pts/3 TCP-LISTEN:6000,reuseaddr,fork &
# from anywhere on the network:
nc <sbc-ip> 6000
```

#### When running as a systemd service

The binary runs in the background, so there's no terminal to attach to and the
PTY path is only in the journal:

```bash
journalctl -u zephcore | grep pseudotty     # find the current /dev/pts/N
```

Because the path changes every boot and you can't `screen` a backgrounded
process, the practical options are:

- **Expose the CLI on a fixed TCP port** by adding the `socat` bridge as an
  `ExecStartPost=` in the unit (see [Run at boot](#run-at-boot-systemd-service)),
  so the console is always reachable at `<sbc-ip>:6000` regardless of the PTY
  number. Then `nc <sbc-ip> 6000` to configure.
- **Or run the repeater in the foreground** (`sudo systemctl stop zephcore` then
  launch the binary by hand) just while you need to change settings, and restart
  the service afterward.

After changing settings, persist + apply them the same way as on an MCU repeater
(most `set` commands take effect immediately; some require `reboot`). Settings
survive restarts via the per-node flash file — see [Persistent storage](#persistent-storage).

### Run at boot (systemd service)

To start ZephCore automatically at boot and restart it if it ever exits, install
a systemd service. This assumes you've already done the [Running without root](#running-without-root)
setup (the `spi`/`gpio` groups + udev rule), so the service can run as your
normal user — **no root**.

Create `/etc/systemd/system/zephcore.service` (adjust `User=` and the paths to
your setup):

```ini
[Unit]
Description=ZephCore native LoRa mesh
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=femto
SupplementaryGroups=spi gpio
# WorkingDirectory pins where the persistent flash file lands
# (meshcore_settings_<hostname>.bin) and must be writable by User=.
WorkingDirectory=/home/femto
ExecStart=/home/femto/zephcore_native_linux.exe
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

Enable + start it (and have it come back on every reboot):

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now zephcore      # start now + at boot
sudo systemctl status zephcore            # check it's running
journalctl -u zephcore -f                 # follow logs (replaces stderr)
```

Manage it:

```bash
sudo systemctl restart zephcore
sudo systemctl stop zephcore
sudo systemctl disable zephcore           # stop auto-start at boot
```

Notes:
- **No `--rt` flag** is needed — real-time mode is baked into the binary.
- **Stop `meshtasticd` first** (Femtofox): if it's installed it grabs the SPI bus.
  Disable it so it doesn't race your service: `sudo systemctl disable --now meshtasticd`.
- **Companion (TCP) role** works perfectly as a service — connect the app to
  `<sbc-ip>:5000` as usual.
- **Repeater role:** the CLI is a pseudo-terminal whose path (`/dev/pts/N`) is
  printed to the journal at boot (`journalctl -u zephcore | grep pseudotty`). To
  reach it remotely, add a bridge as `ExecStartPost`, e.g.
  `ExecStartPost=/bin/sh -c 'socat ... TCP-LISTEN:6000,fork &'` — or just run the
  repeater in the foreground when you need the CLI.
- If you relocate the binary (e.g. to `/usr/local/bin/zephcore`), keep
  `WorkingDirectory=` pointed at a user-writable dir so the settings file persists.

---

## Persistent storage

Identity, prefs, contacts, channels and BLE-equivalent settings all live in
LittleFS on `/lfs`, which is backed by Zephyr's flash simulator. On native Linux
that simulator is **file-backed** (not RAM), so everything survives restarts.

The backing file is named **`meshcore_settings_<hostname>.bin`** and created in
the binary's working directory on first run (`<hostname>` comes from the Linux
`gethostname()`, so multiple nodes on one host each get their own file). For the
Femtofox that's `meshcore_settings_femtofox.bin`.

```bash
ls -l meshcore_settings_*.bin     # appears after the first run
```

- To move a node's identity to another machine, copy this file.
- To reset a node to a fresh identity, delete it (a new one is created next run).
- An explicit `--flash=/path/to/file.bin` overrides the default name/location.
- **Migrating from an older build** that used the generic `flash.bin`:
  `cp flash.bin meshcore_settings_$(hostname).bin` before first run to keep your
  existing identity.

> Implementation: `patches/zephyr/0008-flash-sim-per-node-file.patch` makes the
> flash simulator default to the per-node filename instead of `flash.bin`.

---

## Runtime Pin Override

For one-off hardware setups or custom wiring, override DT defaults at runtime:

```bash
./zephcore_native_linux.exe \
  --lora-spidev=/dev/spidev0.0 \
  --lora-gpio-chip=/dev/gpiochip1
```

Currently only the **paths** are runtime-overridable; pin offsets come from the DTS overlay you build with. To change pin numbers without rebuilding, create your own `boards/linux_native/<device>.conf` + `.overlay` (copy `femtofox.overlay` as a starting point) and pass it via `-DEXTRA_CONF_FILE`.

Run `./zephcore_native_linux.exe --help` to see all available command-line arguments registered by the native_sim infrastructure (Zephyr drivers, our SPI/GPIO drivers, etc.).

---

## Troubleshooting

### Permission denied opening /dev/spidev0.0 or /dev/gpiochip*

Set up group access — see [Running without root](#running-without-root). On the
Femtofox image the `spi`/`gpio` groups don't exist yet, so you must `groupadd`
them and add a udev rule (just `usermod` is not enough). Or run with `sudo`.

### `/dev/spidev0.0: No such file or directory`

SPI isn't enabled in the device tree. See "Enabling spidev on the SBC" above.

### `Linux kernel headers too old; GPIO V2 chardev uAPI required`

The host adapter requires the GPIO V2 uAPI (kernel ≥ 5.10, late 2020). All current SBC distros (Debian 12+, Ubuntu 22.04+, Raspberry Pi OS bookworm+) ship recent enough kernels. If you hit this, your distro is ancient — upgrade.

### TCP port 5000 already in use

Another service (Flask, Docker registry, AirPlay…) has port 5000. Override:

```bash
west build … -- -DCONFIG_ZEPHCORE_LINUX_TCP_PORT=15000
```

### "TCP companion client disconnected" loops

The transport expects raw NUS bytes with no length prefix. If the connection drops immediately, capture traffic with `tcpdump -i any -X port 5000` and verify the first bytes the app sends look like a MeshCore opcode (e.g. `0x01` = CMD_APP_START), not an HTTP request or other framed protocol.

### `Failed to request line 8` / `CS GPIO configure failed: -5` (Raspberry Pi)

```
<err> gpio_native_linux: Failed to request line 8 on host chip
<err> spi_native_linux: CS GPIO configure failed: -5
<err> sx126x_hal: SPI bus not ready
<err> sx126x: HAL init failed: -19
```

On the Pi, `dtparam=spi=on` binds the spi0 controller to GPIO 7–11 in ALT0, so
**CE0 = BCM GPIO 8 is owned by the SPI controller** and cannot also be requested
as a plain GPIO for chip-select — the request fails with `-EIO`. The fix is to let
spidev drive CE0 in **hardware** (it asserts CS on every transfer) and **not**
declare `cs-gpios` in the overlay. The `rak6421` / `rak6421_pi5` overlays do this
(fixed 2026-06-03). If you copy the Femtofox overlay for a Pi setup, remove its
`cs-gpios` line — that path
only works when CS is a free GPIO (as it is on the Luckfox, GPIO48 on gpiochip1).

### Radio does nothing (no RX, CAD always times out, TX never completes)

The interrupt path (DIO1) and BUSY readback are the spine of all radio ops. If
they're broken at the GPIO layer, the chip stays wedged in standby and *every*
operation fails (CAD `-ETIMEDOUT`, no RX, no TX-done).

> **Don't** try to verify DIO1/BUSY with `gpioget /dev/gpiochip1 23` while the
> binary is running — the kernel grants GPIO lines **exclusively**, so the
> binary holds them and `gpioget` just returns `-EBUSY`. Reading it with the
> binary stopped tells you nothing (nothing is driving the radio). Instead,
> build with LoRa debug logging and read the chip's own registers:
> ```bash
> west build … -- … -DCONFIG_LORA_LOG_LEVEL_DBG=y
> ```
> A healthy boot shows `sx126x_irq_work_handler: IRQ status: …` lines on RX/CAD/TX.

Things that previously broke this exact path (all fixed; listed so you recognize
the symptoms if a regression appears):
- Native GPIO driver requesting no lines (the `GPIO_DISCONNECTED == 0` trap) →
  BUSY stuck-low, commands dropped, chip frozen.
- Double inversion on active-low pins → RESET driven backwards, chip held in reset.
- Missing real-time clock mode → CAD/TX/RX time out before the real radio responds
  (now forced automatically; do not remove `linux_native_setup.c`).

If a regression appears, check `port_get_raw`/`pin_configure` in
`gpio_native_linux.c` and that `linux_native_setup.c` is still compiled.

### `bind(5000) failed: 98` (EADDRINUSE)

Another instance of the binary is already running, or you killed the last one with `kill -9` and the socket is still in TIME_WAIT. `SO_REUSEADDR` is set so this should clear in <1 minute; or `pkill zephcore_native_linux.exe`.

---

## Architecture

```
┌─────────────────────────────────────────────┐
│ ZephCore Mesh (C++)                         │
│   CompanionMesh / RepeaterMesh              │
└──────────────┬──────────────────────────────┘
               │ Zephyr APIs (k_event, k_msgq, k_thread)
               │
┌──────────────▼──────────────────────────────┐
│ Zephyr RTOS @ native_sim (running as Linux  │
│ process via pthreads)                       │
│                                             │
│   • SX1262 driver (unchanged)               │
│   • New spi_native_linux driver       ──┐   │
│   • New gpio_native_linux driver      ──┤   │
│   • LinuxTCPTransport (replaces BLE)  ──┤   │
└──────────────────────────────────────────┼──┘
                                           │
                              host syscalls + zsock_*
                                           │
┌──────────────────────────────────────────▼──┐
│ Linux kernel                                │
│   /dev/spidevX.Y    /dev/gpiochipN          │
│   AF_INET TCP socket    /dev/pts/N          │
└─────────────────────────────────────────────┘
```

The mesh C++ layer and the patched SX1262 Zephyr driver are bit-identical to the MCU builds. Only the SPI/GPIO host bridges and the companion transport differ.

---

## Limitations

- **Not BLE.** The companion app must support TCP/Network mode. No BlueZ integration.
- **Not for production use** per Zephyr's `native_sim` documentation. Works fine for hobby/lab deployments.
- **Single companion client.** One TCP connection at a time, mirroring `BT_MAX_CONN=1`.
- **Persistent storage** is file-backed and survives restarts — see [Persistent storage](#persistent-storage).

---

## Files

All native-Linux board files live under **`boards/linux_native/`**:

- `boards/linux_native/linux_common.conf` + `.overlay` — platform base, auto-applied when `BOARD=native_sim` (no device wiring)
- `boards/linux_native/femtofox.conf` + `.overlay` — Femtofox preset (SX1262 TCXO wiring)
- `boards/linux_native/rak6421.conf` + `.overlay` — RAK6421 HAT, Pi 2/3/4/Zero 2 (gpiochip0)
- `boards/linux_native/rak6421_pi5.conf` + `.overlay` — RAK6421 HAT, Pi 5 (gpiochip4)

> These are `EXTRA_CONF_FILE` **presets**, not Zephyr boards — you still build with
> `-b native_sim … -DEXTRA_CONF_FILE="boards/linux_native/<device>.conf"`, not `-b <device>`.
- `adapters/transport/LinuxTCPTransport.c` — TCP companion transport
- `adapters/transport/linux_native_setup.c` — forces real-time clock mode at boot (bakes in `--rt`)
- `patches/zephyr-new/drivers/spi/spi_native_linux*` — spidev SPI driver
- `patches/zephyr-new/drivers/gpio/gpio_native_linux*` — GPIO driver (kernel GPIO V2 chardev ioctls, no libgpiod)
- `patches/zephyr/0007-spi-gpio-native-linux.patch` — wires the new drivers into Zephyr's `drivers/spi/` and `drivers/gpio/` CMakeLists + Kconfig
- `patches/zephyr/0008-flash-sim-per-node-file.patch` — flash simulator defaults to `meshcore_settings_<hostname>.bin` for persistent per-node storage

---

## Debugging Log

(Updated as issues are found during bringup.)

### Verification status

End-to-end verified under WSL Ubuntu 24.04 (gcc 13.3):

- ✅ All 7 `patches/zephyr/*.patch` apply cleanly (including the new `0007-spi-gpio-native-linux.patch`).
- ✅ All files in `patches/zephyr-new/` copy correctly into the Zephyr tree.
- ✅ Platform detection routes `BOARD=native_sim` to `boards/linux_native/linux_common.conf`.
- ✅ `native_sim/native/64` builds clean → `build/zephyr/zephcore_native_linux.exe` (~4.3 MB ELF).
- ✅ Binary runs. Zephyr OS boots, mesh event loop starts.
- ✅ `spi_native_linux` driver loads and attempts to open `/dev/spidev0.0` (fails in WSL: no SPI hardware).
- ✅ `LinuxTCPTransport` listens on port 5000.
- ✅ TCP client connect/disconnect works; `SerialWifiInterface` (`<`/`>` + 2-byte LE length) framing parses correctly.
- ✅ **End-to-end on real Femtofox hardware (2026-06-02):** radio RX, CAD (LBT), and TX all working; companion app connects and meshes; settings persist across restarts. (Required fixing two bugs in the native GPIO driver — see "Known caveats" below.)
- ✅ **End-to-end on real Raspberry Pi Zero 2 W + RAK6421/RAK13300 hardware (2026-06-03):** SX1262 init, SPI with spidev hardware CS (CE0), DIO3 TCXO, DIO2 RF switch, and over-the-air RX/TX all confirmed. (Required dropping the GPIO chip-select from the Pi overlays — CE0 is owned by the kernel SPI controller; see the `Failed to request line 8` troubleshooting entry.)

Build command verified (run from inside WSL with workspace at `/mnt/d/zephcore`):

```bash
export PATH="$HOME/.local/bin:$PATH"
export ZEPHYR_BASE=/mnt/d/zephcore/zephyr
cd /mnt/d/zephcore && west build -b native_sim zephcore
```

Use `native_sim` (32-bit) for x86-64 host smoke builds and for ARM cross-compile targets
(Femtofox). Use `native_sim/native/64` only if targeting a 64-bit host natively (no
cross-compile). The 32-bit variant requires `libc6-dev-i386` on the build host.

### Known caveats found during implementation

- **Native GPIO driver had two bugs that killed the radio** (fixed 2026-06-02,
  `patches/zephyr-new/drivers/gpio/`):
  1. `rebuild_line()` tested `(flags & GPIO_DISCONNECTED) == GPIO_DISCONNECTED`,
     but `GPIO_DISCONNECTED == 0`, so the test was *always true* and **no GPIO
     line was ever actually requested** — every input read returned 0 and every
     output write was a no-op (SPI still worked only because spidev drove its own
     hardware CS). BUSY read stuck-low → driver never waited for the chip →
     commands issued while BUSY → silently dropped → chip frozen in STBY_RC. Fix:
     test `(flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED`.
  2. The driver passed `GPIO_V2_LINE_FLAG_ACTIVE_LOW` to the kernel while Zephyr's
     generic GPIO layer *already* inverts active-low pins (`data->invert`) →
     **double inversion** drove the active-low RESET line backwards, holding the
     chip in reset. Fix: driver `port_*_raw` are physical/raw; let Zephyr own
     inversion (don't set the kernel `ACTIVE_LOW` flag).
  Symptom of both: TCXO/XOSC/CAD all *look* like the culprit (XOSC_START_ERR,
  CAD `-ETIMEDOUT`) but are downstream — the chip is simply wedged.

- **`--rt` is mandatory and now automatic.** Without real-time clock mode the
  `native_sim` clock free-runs decoupled from wall time, so real-radio timeouts
  (200 ms CAD, TX/RX-done) fire before the hardware responds. Forced at boot via
  `linux_native_setup.c` (`hwtimer_set_real_time_mode(true)`).

- **Double-overlay listing** in `EXTRA_DTC_OVERLAY_FILE`: `linux_common.overlay` gets auto-paired twice — once when the platform conf is selected, once again when the EXTRA_CONF_FILE list is re-walked. Pre-existing CMakeLists.txt quirk affecting all platform confs; harmless (DTC merges idempotently) but cosmetic.

- **Pin paths are runtime-overridable, pin numbers are not.** Only `--lora-spidev=<path>` and `--lora-gpio-chip=<path>` are wired as command-line args. To change actual pin numbers without rebuilding, create a custom preset overlay and pass it via `-DEXTRA_CONF_FILE`. (A future iteration could expose pin offsets as cmdline args too.)
