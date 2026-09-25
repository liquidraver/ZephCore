# Repeater CLI Commands

All commands are sent over USB serial (CDC-ACM). Commands sent remotely over the mesh (non-zero `sender_timestamp`) cannot access USB-only commands.

> The **Room Server** role shares this CLI — the common commands (radio, region, password, advert, gps, etc.) plus `setperm` / `get acl` all apply.

**Sources:**
- `helpers/CommonCLI.cpp` — common commands shared by all roles
- `app/RepeaterMesh.cpp` — repeater-specific commands (`setperm`, `get acl`, `region`, `discover.neighbors`)
- `helpers/CommonCLI.cpp` `handleRegionCmd()` — the `region` sub-CLI (as upstream); the `region load` line reader stays in each role
- `app/RepeaterUplink.cpp` — `get`/`set uplink.*` (ESP32 uplink builds only)
- `app/RoomServerMesh.cpp` — room-server-specific commands (`room.post`)

> **Commands are case-sensitive**, matching Arduino MeshCore. Nothing is lower-cased before matching.

> **Request-tag prefix.** If a command is longer than 4 characters and its **third** character is `|`
> (e.g. `a7|reboot`), the first three characters are stripped before dispatch and echoed back at the
> start of the reply. This is how the phone app correlates replies with requests. It means a command
> whose third character is a literal `|` cannot be sent as-is.

---

## System

| Command | Description |
|---------|-------------|
| `ver` | Firmware version and build date |
| `board` | Board manufacturer name |
| `reboot` | Reboot immediately |
| `poweroff` / `shutdown` | Power the node off, as upstream: GPS, sensor and buzzer rails off, LoRa held in reset, then System OFF (the power latch released first on soft-power boards). Wake with the user button (nRF) or a power cycle — a node without a reachable button stays off until someone gets to it, so think twice before sending it over remote admin. The shutdown reason `User Request` is recorded for `get pwrmgt.bootreason`. Replies `OK - powering off`; deferred like `reboot` |
| `start dfu` | nRF52: reboot into the UF2 bootloader for drag-and-drop update. ESP32-S3: reboot into the ROM download mode on USB-Serial-JTAG (`303a:1001`), so ordinary `esptool write-flash` and browser flashers can reach the chip. Every other chip replies with an error and does **not** reboot — ESP32-C3/C6 and classic ESP32 never lose the port to USB OTG so esptool resets them itself, and nRF54L15/MG24/STM32WL have no USB device peripheral and are flashed over SWD |
| `start ota` | ESP32: start WiFi AP + HTTP OTA server. nRF52: reboot into BLE OTA DFU mode |
| `stop ota` | Stop WiFi OTA server (ESP32 only) |
| `clkreboot` | Set clock to a fixed reference time (15 May 2024 8:50pm UTC) then reboot. On a board with a hardware RTC the reference time is written to the chip too, as upstream, so the reset survives the reboot |
| `powersaving` | Not implemented |

---

## Clock

| Command | Description |
|---------|-------------|
| `clock` | Display current UTC time |
| `clock sync` | Sync clock from the sender's timestamp (only advances, cannot go backwards). Arms the 7-day mesh-time-sync suppression window. |
| `time <unix_timestamp>` | Set RTC to a specific Unix timestamp (cannot go backwards). Arms the 7-day mesh-time-sync suppression window. |

Every clock set (these commands, GPS, the app, mesh time sync, SNTP) is also written to the board's hardware RTC when it has one, so the time survives a power-off.

---

## Advertisement

| Command | Description |
|---------|-------------|
| `advert` | Send a flood-routed self-advertisement (1500 ms delay) |
| `advert.zerohop` | Send a 0-hop (direct only) self-advertisement |

---

## Neighbors

| Command | Description |
|---------|-------------|
| `neighbors` | Display current neighbor list |
| `neighbor.remove <pubkey_hex>` | Remove a neighbor entry by its public key. A prefix is accepted — the hex is truncated to at most 32 bytes and matched at whatever length you give. **Repeater only in effect:** `RoomServerMesh` does not override `removeNeighbor`, so on a room server this replies `OK` and does nothing. |
| `discover.neighbors` | *(repeater only)* Broadcast a node discovery request to find nearby nodes; responses are collected for 60 s (as upstream `bf9c6cb5`). Takes no arguments — anything after it replies `Err - discover.neighbors has no options`. Not implemented on room servers. |

---

## Security & Access Control

| Command | Description |
|---------|-------------|
| `password <new_password>` | Set the admin password (**max 15 characters**) |
| `setperm <perms_hex> <pubkey_hex>` | Set ACL permissions for a node (app format: 2-char hex perms first) |
| `setperm <pubkey_hex> <perms_dec>` | Set ACL permissions for a node (Arduino format: pubkey first, decimal perms) |
| `get acl` | *(USB only)* List all ACL entries, as upstream: `ACL:` then one `<perms hex> <public key hex>` line per entry, printed straight to the console (the `  -> ` reply line is empty). Guest entries (permissions 0) are skipped. |

> **Password length:** admin and guest passwords are capped at **15 characters** (16-byte storage incl. NUL; same limit as Arduino MeshCore). The login-send path silently truncates anything longer, so a password >15 chars will never authenticate. Applies to `set guest.password` as well.

> **`allow.read.only` is room-server only.** `RoomServerMesh.cpp` is its sole consumer; `RepeaterMesh.cpp` never reads it, so on a repeater the setting silently did nothing. The CLI now only exposes it on `CONFIG_ZEPHCORE_ROLE_ROOM_SERVER` builds — a deliberate divergence from Arduino MeshCore, whose shared CommonCLI offers the knob on every role. The pref itself is unchanged: it stays byte 114 of the on-flash prefs layout, identical to Arduino's, so existing prefs files are unaffected.

> **Guest access differs by role, matching Arduino MeshCore.** On a **repeater**, an empty `guest.password` (the default) means *open* guest access — a blank submitted password logs in as `PERM_ACL_GUEST`, which cannot run CLI commands or read the access list, so it gets login plus status/telemetry only. On a **room server**, an empty `guest.password` *disables* guest login, so a room is never accidentally left open; to run an open room use `set allow.read.only on`, which grants read-only (`PERM_ACL_GUEST`), not post rights. Set a non-empty `guest.password` to require one on either role.

---

## Room Server

| Command | Description |
|---------|-------------|
| `room.post <message>` | Post a message to the shared room as the server itself (system post). Pushed to clients like any other post. |

---

## Region Filtering

Regions control which flood packets the repeater forwards. The region tree is hierarchical; the wildcard `*` region is the root.

| Command | Description |
|---------|-------------|
| `region` | Export the current region map (indented text tree) |
| `region load` | Enter interactive region load mode. Paste indented region lines; send a blank line to commit. Any unindented command (e.g. `reboot`) aborts the load without committing, and then runs |
| `region save` | Save the current region map to persistent storage |
| `region def <token> [...]` | Cursor-walk bulk region builder — define a hierarchy in one line (see below) |
| `region put <name> [<parent>]` | Create a region; default parent is the wildcard root. Flood is **allowed** by default (use `region denyf` to deny) |
| `region remove <name>` | Remove a region (must have no children) |
| `region get <name>` | Show a region's parent and flood-allow flag |
| `region home [<name>]` | Get (no arg) or set the home region |
| `region default [<name>\|<null>]` | Get (no arg), set, or clear (`<null>`) the default flood scope. Originated floods (self-adverts, etc.) are scoped with this region's TransportKey. Auto-creates the region if it doesn't exist and persists immediately |
| `region allowf <name>` | Allow flood packets in a region (clears deny-flood flag) |
| `region denyf <name>` | Deny flood packets in a region (sets deny-flood flag) |
| `region list allowed` | List all regions that allow floods |
| `region list denied` | List all regions that deny floods |

**Region load format:** one region per line, indented with spaces to indicate depth. Append `F` after the name to mark flood-allowed (otherwise flood is denied by default).

**`region def` format:** space-separated tokens; a cursor starts at `*`. Each token is `name` (create child of cursor, advance cursor to it) or `name|jump` / `name,jump` (create child of cursor, then move cursor to the existing region `jump`). Does **not** auto-save — follow with `region save`. Reply is the updated region tree. Example — branched tree: `region def west pnw or pdx|pnw wa sw-wa`. Example — flat list: `region def west|* pnw|* or|* pdx|*`.

---

## Statistics & Logging

| Command | Description |
|---------|-------------|
| `clear stats` | Reset all statistics counters |
| `stats-core` | *(USB only)* Display core mesh statistics |
| `stats-radio` | *(USB only)* Display radio statistics |
| `stats-packets` | *(USB only)* Display packet statistics |
| `log start` | Enable packet logging to file |
| `log stop` | Disable packet logging |
| `log erase` | Erase the log file |
| `log` | *(USB only)* Dump the full log file to USB serial |
| `erase` | *(USB only)* Factory reset: erase the entire LittleFS volume, the BLE-bond NVS, and external QSPI flash, then reboot |

> **`erase` is a true factory reset.** It flattens `lfs_partition` (identity, prefs, ACL,
> region map, logs), `storage_partition` (BLE bonds) and `qspi_storage_partition` where
> present — not just the files under `/lfs/repeater/`. The node comes back with a new
> identity and default prefs. Erasing the volume rather than unlinking files is what makes
> it able to recover a volume another firmware has written into: on nRF52840 the Adafruit
> core's filesystem (used by Arduino MeshCore and Meshtastic) sits at 0xED000, inside our
> `lfs_partition`, and its format scribbles the top 7 blocks of our volume. Switching
> between Arduino-core firmware and ZephCore on nRF52840 needs an erase in **both**
> directions — `tools/formatter` or a full chip erase.

---

## GPS

| Command | Description |
|---------|-------------|
| `gps` | GPS status in upstream's form: `on, active\|standby, fix\|no fix, N sats`, `off`, or `Can't find GPS`. `standby` (upstream: `deactivated`) means on but asleep between duty-cycle fixes |
| `get gps` | The detail: `> on state=<off\|standby\|acquiring> sats=N fix=Ns ago lat= lon=`, or `no fix next=Ns` (seconds to the next wake) |
| `gps sync` | Take a fresh fix now (wake the GPS or extend its window); the fix sets the clock. `gps is off` / `gps provider not found` otherwise |
| `gps on` | Enable GPS module |
| `gps off` | Disable GPS module. Persisted: a repeater or room server with GPS off stays off across reboots. Boards whose GPS has no power line (RAK4631, RAK3401 1W, Station G2) put the module to sleep over UART |
| `gps setloc` | Update stored latitude/longitude from the last GPS fix (unchanged if there has been none since boot) |
| `gps advert` | Show current location advertising policy |
| `gps advert none` | Do not include location in advertisements |
| `gps advert share` | Include live GPS location in advertisements |
| `gps advert prefs` | Include stored lat/lon from prefs in advertisements |
| `set gps duty <sec>` | GPS duty interval (standby seconds between fixes). `0` = always-on (continuous; streams fresh fixes, can download a full almanac). Floor 10s, cap 604800 (1 week). Persists to flash, applied live. |
| `set gps duty default` | Reset GPS duty to the role default (repeater/room 48h, companion 300s) |
| `set gps diag <0\|1\|on\|off>` | Arm GPS module-configuration diagnostics (see below). Not persisted — clears on reboot |

**GPS configuration diagnostics.** At boot the firmware configures the GNSS module — constellations, AssistNow/EASY, minimum elevation, fix rate — and on modules driven over raw NMEA those commands are sent **blind**: nothing reads the module's reply, so a silently rejected configuration is indistinguishable from a working one. These two commands make that visible.

```
set gps diag 1        # arm it
gps off               # power-cycle the module...
gps on                # ...which re-runs configuration and records the result
get gps diag          # read it back
```

Sample reply:

```
> diag=on cfg=uart age=910s rx=120 mod=URANUS5 sent=12/336B sys=G3/R4/E0/B3/?0
```

- `rx=` NMEA sentences the driver has parsed. **Check this first** — it is the only field that cannot be misread. Non-zero means the module is alive, at the right baud, and talking, so anything still wrong is signal or antenna. Zero means nothing is arriving at all, and no antenna work will help
- `cfg=` which path ran — `uart` (raw PMTK+PCAS+UBX), `api` (driver GNSS API), `blind` (neither available), or `never-run`
- `mod=` module identification, from a CASIC `$GPTXT` version reply or a u-blox `$PUBX` poll response, or `no-reply`. Only an explicit software-version token is accepted as an identity — TXT sentences also carry warnings, and a warning reported as an identity is worse than no answer
- `sent=` commands/bytes written to the module (UART path), or `sys_ret=`/`rate_ret=` return codes (API path)
- `sys=` tracked satellites per constellation from GSV talker IDs: **G**PS / GLONASS (**R**) / Galileo (**E**) / **B**eiDou / other. A constellation that stops reporting for 30 s decays to zero rather than showing a stale count

`sys=` totalling more than `sats=` in `get gps` is expected, not a discrepancy: GSV counts satellites **tracked**, GGA counts satellites **used in the fix solution**.

**`rx=` first, then `mod=`.** `rx=` is the only field that cannot be misread: non-zero means the module is alive, at the right baud and talking, so anything still wrong is signal or antenna; zero means nothing is arriving at all. `mod=` then tells you whether the module *heard* us — everything on this transport is written blind, so a module that hears nothing looks exactly like one that hears everything and ignores it. `mod=no-reply` with `rx=` climbing means the receive direction works but our transmit does not reach it: wiring or pin assignment, not configuration.

**`sent=` proves transmission, not acceptance.** Only `sys=` shows what the module actually did. A module still running its factory or previously saved configuration reports `G` non-zero with the rest at `0`. Note `B0` is expected on u-blox M8 (BeiDou is deliberately disabled — only three major constellations can run concurrently), and `?0` is normal outside Japan (QZSS is regional).

The generic-NMEA path sends three protocols — PMTK (MediaTek), PCAS (CASIC: Quectel L76K/L76KB, Air530Z) and UBX (u-blox) — because a WisBlock-style GPS slot can hold any of them and each family ignores what it does not understand. Related build option: `CONFIG_ZEPHCORE_GPS_NAV_MODE` sets the CASIC navigation dynamic model (`$PCAS11`), defaulting to stationary for repeaters and room servers and automotive otherwise. It is worth setting because that model is stored *in the module* and survives reflashing the host — a slot module that previously lived in another device can arrive stuck in an airborne model that quietly degrades fixes on a fixed site.

Caveats: the `sys=` tally needs `CONFIG_ZEPHCORE_GPS_SAT_DIAG` (default on for repeaters, off for companions to save RAM) — the reply says so when built without it. Only the raw-UART path is re-run on `gps on`; boards with a real GNSS driver (Air530Z, LC76G) keep reporting their boot-time result, because that path goes through `modem_chat_run_script()`, which is safe only at boot. On those boards `E0` is also expected — the Air530Z driver supports GPS/GLONASS/BeiDou but not Galileo, and the firmware falls back automatically.

---

## Sensor Settings

| Command | Description |
|---------|-------------|
| `sensor list [<start_idx>]` | List custom sensor settings (paginated at 134 chars) |
| `sensor get <key>` | Get a custom sensor setting value by key |
| `sensor set <key> <value>` | Set a custom sensor setting value |

As upstream, the one setting is `gps` (`0`/`1`), listed only on a board with a GPS; `sensor set gps_interval <sec>` also works and sets the duty interval. Neither is saved — `gps on|off` and `set gps duty` are.

---

## Radio (Temporary Override)

| Command | Description |
|---------|-------------|
| `tempradio <freq>,<bw>,<sf>,<cr>,<timeout_mins>` | Apply temporary radio parameters; automatically reverts after `timeout_mins`. Constraints: freq 150–2500 MHz, bw 7–500 kHz (**7–1000 on LR2021 boards**), sf 5–12, cr 5–8. Saved prefs are never mutated — concurrent `set` commands and reboots both restore the real saved values. That now covers adaptive CAD too: the visited preset runs at its own base detPeak (the learned offset addresses a different ladder there), the staircase is suspended for the duration so no visitor value can reach prefs, and the revert restores the learned offset. `get cad` shows `a:tmp` while the window is open. The anti-mute TX-starvation safety still runs, on the visitor's offset. |

---

## Repeater Uplink (ESP32 + `CONFIG_ZEPHCORE_REPEATER_UPLINK`)

These commands configure observer-style WiFi+MQTT packet reporting from repeater role.
All `set uplink.*` changes are saved immediately and only applied after reboot.

| Command | Description |
|---------|-------------|
| `get uplink.status` | Uplink runtime state: enabled flag, WiFi state, MQTT state, reboot-required flag |
| `get uplink.enable` | Uplink enable flag (`on`/`off`) |
| `get uplink.wifi.ssid` | Configured WiFi SSID |
| `get uplink.mqtt.host` | Configured MQTT broker host |
| `get uplink.mqtt.port` | Configured MQTT broker port |
| `get uplink.mqtt.tls` | MQTT TLS mode (`0`/`1`) |
| `get uplink.mqtt.user` | Configured MQTT username |
| `get uplink.mqtt.iata` | Configured IATA/site code used in MQTT topic |
| `set uplink.enable <on\|off>` | Enable or disable repeater uplink *(reboot required)* |
| `set uplink.wifi.ssid <ssid>` | Set WiFi SSID *(reboot required)* |
| `set uplink.wifi.psk <psk>` | Set WiFi password *(reboot required)* |
| `set uplink.mqtt.host <host>` | Set MQTT host *(reboot required)* |
| `set uplink.mqtt.port <port>` | Set MQTT port 1–65535 *(reboot required)* |
| `set uplink.mqtt.tls <0\|1>` | Set MQTT TLS mode *(reboot required)* |
| `set uplink.mqtt.user <user>` | Set MQTT username *(reboot required)* |
| `set uplink.mqtt.password <pass>` | Set MQTT password *(reboot required)* |
| `set uplink.mqtt.iata <code>` | Set MQTT site code *(reboot required)* |

---

## Companion WiFi (`CONFIG_ZEPHCORE_COMPANION_WIFI`)

Companion builds of boards whose `zephcore.yml` declares `capabilities: wifi: true` (the PSRAM ESP32-S3 boards, plus Heltec V3, Wireless Tracker and the two C6 boards with fewer contacts) join a WiFi network and serve the companion protocol over TCP on port 5000, beside BLE and USB. Upstream's commands and replies; every change applies on the next reboot. Reachable over USB, from the app's CLI and, as upstream, over the air from a contact with remote CLI permission — including `get wifi.pwd`.

| Command | Description |
|---------|-------------|
| `set wifi.ssid <name>` | Network to join (up to 32 characters; spaces allowed) |
| `set wifi.pwd <password>` | Its password (up to 63 characters; empty = open network) |
| `get wifi.ssid` / `get wifi.pwd` | The saved values (`(not set)` for no SSID) |
| `set wifi.enabled <0\|1>` / `get wifi.enabled` | Join the saved network at boot (default 1; nothing happens until an SSID is set) |
| `set wifi.clear` | Forget SSID and password |
| `get wifi.status` | `connected` once the link is up and DHCP has an address, else `disconnected` |
| `get wifi.ip` | The address the app connects to (`(not connected)` otherwise) |

## `get` — Read Configuration

| Command | Returns |
|---------|---------|
| `get name` | Node name |
| `get role` | Firmware role: `repeater` or `room_server` (companion builds report `companion`) |
| `get repeat` | Forwarding enabled: `on` or `off` |
| `get radio` | Radio params as `freq,bw,sf,cr` — the same comma-separated form `set radio` takes, so a reply can be edited and sent straight back |
| `get freq` | Frequency in MHz |
| `get freqerr` | Carrier frequency error measured on received packets: `mean N Hz, min A, max B, K pkts`. **LR2021 only** — other radios answer `not available`. Purely diagnostic; nothing acts on it. **The mean only approximates *this* node's reference error once it is averaged over many different peers** — their individual errors cancel, ours does not — so read `K` and the min/max spread before believing it: a tight spread over a handful of packets is one chatty neighbour, not a population. Small values are the expected answer and mean there is nothing to do; LoRa tolerates carrier error up to roughly a quarter of the bandwidth before sensitivity suffers, so at BW 62.5 kHz a few hundred Hz is noise. If it is kHz-scale the correction is board-dependent: XTAL parts have `SetXoscCpTrim`, but **TCXO parts have no chip-side trim at all** (DS §6.11.4: "If a TCXO is configured, this command has no effect"), leaving only a software offset to the programmed frequency. Values beyond ±200 kHz are discarded by the driver and warn once — the field is decoded from three `GetLoraPacketStatus` bytes that DS rev 2.1 does not document, so implausible readings are evidence the field is not real on that firmware rather than a genuine measurement. Reset by `clear stats`. |
| `get tx` | TX power in dBm |
| `get lat` | Stored latitude |
| `get lon` | Stored longitude |
| `get dutycycle` | Duty cycle as percentage (e.g. "50.0%") |
| `get af` | Raw airtime factor value |
| `get txdelay` | Adaptive TX delay status: contention estimate and flood delay factor |
| `get rxdelay` | *(deprecated)* Always returns "adaptive (rxdelay deprecated)" |
| `get direct.txdelay` | *(deprecated)* Always returns "adaptive (direct.txdelay deprecated)" |
| `get backoff.multiplier` | Per-dupe reactive backoff multiplier |
| `get flood.max` | Max flood retransmit hops |
| `get flood.max.unscoped` | Max retransmit hops for un-scoped floods |
| `get flood.max.advert` | Max retransmit hops for ADVERT floods |
| `get flood.advert.interval` | Flood advertisement interval in hours |
| `get advert.interval` | Local advertisement interval in minutes |
| `get allow.read.only` | *(room server only)* Whether read-only clients are allowed |
| `get guest.password` | Guest access password |
| `get owner.info` | Owner/contact info (pipes `\|` display as newlines) |
| `get int.thresh` | Interference threshold |
| `get leds` | LED master switch: `on` or `off` |
| `get leds.radio` | Activity-LED mode: `tx`, `rx`, `all` or `off`. Appends `(no radio LED on this board)` where the board has no `lora-tx-led` alias |
| `get leds.hb` | Heartbeat-LED mode: `all`, `hb`, `unread` or `off`. Appends `(no heartbeat LED on this board)` where the board has neither `led0` nor `led1` |
| `get buzzer` | *(room server only)* Buzzer/vibration mode as `<n> (<name>)`: `0 (silent)`, `1 (sound+vib)`, `2 (vibrate)`, `3 (sound)`. Compiled out on repeater builds (`#ifndef ZEPHCORE_REPEATER`) — a repeater answers `unknown config: buzzer`. |
| `get agc.reset.interval` | Removed — replies `Removed - Automatic AGC reset is on`. Periodic AGC recalibration was deleted (it reset the noise floor to its unseeded sentinel on every fire). Use `set rxduty` to cut RX current. |
| `get multi.acks` | Extra ACK transmit count (`0` or `1`) |
| `get path.hash.mode` | Path hashing algorithm: `0`, `1`, or `2` |
| `get loop.detect` | Loop detection level: `off`, `minimal`, `moderate`, or `strict` |
| `get radio.rxgain` | RX gain boost: `on` or `off` |
| `get radio.fem.rxgain` | External FEM's LNA in the RX path: `on` (through the LNA) or `off` (bypassed). Default `on` |
| `get rxduty` | RX duty cycle mode: `0` or `1` |
| `get display.rotate` | Panel 180-degree rotation: `0` or `1`. Reports the **live panel state**, not the stored byte — the two differ only when a rotation was refused, which is the case worth seeing. Boards whose panel cannot rotate reply `unsupported (panel cannot rotate)` |
| `get input.rotate` | Joystick/D-pad axis swap: `0` or `1` |
| `get tz.offset` | Display timezone: whole hours from UTC, `-12`..`14` (`0` = UTC) |
| `get gps duty` | Now-effective GPS duty interval in seconds (`always on (0)` when continuous) |
| `get gps diag` | What the last GPS module-configuration attempt did — which path ran, bytes sent, and tracked satellites per constellation. See **GPS configuration diagnostics** in the GPS section for the field reference |
| `get meshtimesync` | Mesh time-sync state + live dry-run: on/off, eligible voter count, votes for/against, consensus skew and radius, would-be verdict (`ok`/`in-band`/`step±N`/`abstain (reason)`/`hold (reason)`; a recent clock set — manual or GPS — shows as `hold (suppressed)`, and a backward step a forward-only role would refuse is annotated `(skipped: forward-only)`), step counters, suppression countdown, and a per-sender evidence table (`prefix hops count skew E`, `E` = counted toward the verdict above). Entries that count print first, so a size-capped reply never hides the ones that explain the summary; if the table doesn't fully fit, a trailing `+N more` shows how many were left out. Sensing runs even while off, so this works as a dry-run before enabling. Over remote admin the reply is truncated to the packet size (summary always fits); the full table needs the USB CLI. |
| `get probe.interval` | Seconds between periodic radio measurements (noise-floor sample + CAD probe). 0 = CAD probing off |
| `get loop.wakes` | Debug builds only (`CONFIG_ZEPHCORE_LOOP_WAKE_STATS`, on in `debug.conf`). Event-loop wakes since boot: `wakes=N up=Ss` then a count per event bit (`bN=`, the role's `MESH_EVENT_*` bit numbers; one wake can carry several bits). Wakes per second = wakes / up. Read it twice and take the difference to measure a window. |
| `get dc.restarts` | Duty-cycle re-arm counter — RxTimeout re-arms **plus** parked-RX watchdog recoveries, sharing one total. **Read it as a rate: divide by uptime.** A bare count is not interpretable, and the two sources it merges cost very differently. An RxTimeout re-arm is ~7 ms of deaf time (the `Calibrate(ALL)` gap in the driver's `restart_rx`) after which the chip returns to duty cycle immediately — packets, not power. A watchdog recovery means the chip sat parked in *full RX* for one to two watchdog periods (`2·(preamble+8)` symbols, floored at 250 ms) — power, not packets, since parked RX still receives. The counter cannot tell you which, so read the worst case. **Measured normal: ~250/hr on a high site at SF8/BW 62.5** (one every ~14 s), where the worst case — every event a park — costs about 3.5% of the duty cycle's savings. Nothing to act on below roughly **2000/hr**; above that the parked-RX share starts eating a meaningful fraction of the saving and it becomes worth splitting the counter to find out. A high rate means the preamble detector is tripping without a decodable packet following, which on an elevated site is usually distant marginal traffic rather than interference — cross-check `get cad.stats`, whose adaptive detPeak offset rises independently in a genuinely busy RF environment. Reset by `clear stats`. |
| `get cad` | Always `on` — ZephCore performs CAD/LBT unconditionally and has no enable knob. Kept as a boolean reply for Arduino MeshCore app compatibility; the real status lives in `get cad.stats`. |
| `get cad.auto` | Whether the adaptive-CAD staircase is acting on probe statistics (`on`/`off`). Set with `set cad.auto`. |
| `get cad.offset` | Operating detPeak offset from the chip family's per-SF/per-bandwidth base, signed. Also shown as `o` in `get cad.stats`, alongside the absolute peak it resolves to. |
| `get cad.busycap` | Faint-tolerance / airtime cap in percent, or `0 (off)`. Also shown as `bc` in `get cad.stats`. |
| `get cad.stats` | Adaptive-CAD status: header (`a` auto on/off, `o` operating detPeak offset, `pk` absolute peak with family base, `sp` noise-floor RSSI burst quality as `mean-spread-dB/zero-spread-%` (plus `(burst-count rN/bN/aN/dN)` on the local USB console, omitted over the air to protect the 161 B reply budget, where `r` is completed RSSI reads, `b` reads the chip refused as busy, `a` bursts abandoned because of one, and `d` attempts turned away before a burst even started by the duty-cycle sleep window — on a healthy radio `b`/`a` stay at 0, and a large `a` against a near-zero burst count is the signature of a sampler being refused rather than one losing the odd read) — a non-zero mean proves the 8 reads are independent however high the share climbs; only mean `0.0` with a high share indicts the sampler. See `ADAPTIVE_CAD.md`. `bc` busy cap), then a 3-rung window around the operating offset (`*` marks it) with probe/busy/fp/tp counts and false-positive rate — the three levels the knee controller reads. Probing runs even while `cad.auto` is off (dry-run), so this is the observation tool for picking a site-appropriate detPeak. See `ADAPTIVE_CAD.md`. Not available on SX127x boards (no hardware CAD). |
| `get extra.sf` | LR2021 side detectors: the extra spreading factors currently received alongside `sf`, comma-separated (bare, no `> ` prefix), or `No extra SF configured`. Reflects the saved prefs, not what the chip accepted — if the set became invalid after an `sf`/`bw` change it is reported here but was refused at boot (a `WRN` line says so). |
| `get adc.multiplier` | Battery voltage ADC calibration multiplier |
| `get bootloader.ver` | Bootloader version string |
| `get pwrmgt.support` | `> supported` on nRF52 (VBUS detection, boot voltage), else `> unsupported` — upstream's reply |
| `get pwrmgt.source` | nRF52: `> external` (VBUS present) or `> battery`; elsewhere `ERROR: Power management not supported` |
| `get pwrmgt.bootreason` | `> Reset: <cause>; Shutdown: <reason>`. The cause is this boot's hardware reset cause as labels (`PIN`, `SOFTWARE`, `BROWNOUT`, `POR`, `WATCHDOG`, `LOWPOWER` for a wake from System OFF, ...), followed by `(crash <K_ERR> in <thread>, pc 0x...)` when the previous run ended in a fatal error and rebooted — resolve the pc with `addr2line` against the same build. The shutdown reason is `User Request`, `Low Voltage` or `None` |
| `get pwrmgt.bootmv` | nRF52: battery voltage sampled at boot, `> <mV> mV` |
| `get public.key` | Node's public key as hex. **Not** USB-only — it is answerable over remote admin, matching Arduino MeshCore. A public key is broadcast in every advert, so there is nothing to gate. |
| `get prv.key` | *(USB only)* Node's private key as hex — the 128-char expanded form, the same one `set prv.key` takes |

---

## `set` — Write Configuration

Changes are persisted immediately unless noted. Some require a reboot.

**The literal `default` is accepted by every `set` that takes a number or an
on/off value**, and restores that setting to what a factory-fresh node boots
with — e.g. `set probe.interval default`, `set tx default`, `set cad.auto
default`. The values come from `initNodePrefs()` (`helpers/NodePrefs.h`), so the
keyword cannot drift from the real defaults. `set radio default` restores all
four radio parameters together, since they are one interop-critical set.

> [!NOTE]
> Before 1.17.4 these setters used `atoi()`/`atof()`, which turn any
> non-numeric text — `default` included — into `0`. On the knobs where `0` is
> itself legal and means "off" (`probe.interval`, `cad.busycap`, `flood.max`,
> `rxduty`) that silently disabled the feature and still answered `OK`.
> Unparseable input is now rejected with an error instead.

| Command | Constraints | Description |
|---------|-------------|-------------|
| `set name <name>` | No `[ ] \ : , ? *` | Set node name |
| `set repeat <on\|off>` | | Enable or disable packet forwarding |
| `set radio <freq>,<bw>,<sf>,<cr>` | freq 150–2500, bw 7–500 (7–1000 on LR2021), sf 5–12, cr 5–8 | **Comma-separated**, not space-separated — spaces parse as a single argument and the command is rejected. Set radio params *(reboot required)*. If freq, bw or sf actually changed, this also performs a full `set cad.reset` and replies `OK - reboot to apply (CAD reset)` — the learned detPeak offset and its probe statistics belong to the old preset. The radio keeps running the old preset, at its learned offset, until the reboot; CAD adaptation is suspended in the meantime so it cannot re-dirty the reset. A cr-only change resets nothing. |
| `set freq <mhz>` | 150–2500 *(USB only)* | Set frequency alone *(reboot required)*. Same CAD reset as `set radio` when the frequency actually changes. On LR2021 boards a frequency at or above 1500 MHz also moves the radio to its 2.4 GHz front end — see the note below the table. |
| `set tx <dbm>` | −9 to board max (default 30) | Set TX power |
| `set lat <latitude>` | | Set stored latitude |
| `set lon <longitude>` | | Set stored longitude |
| `set dutycycle <pct>` | 10–100 | Set duty cycle percentage (converted to airtime factor internally as `(100/pct)-1`). The floor is 10%, not Arduino's 1%: the stored airtime factor is bounded to 0–9 on load, so a lower duty cycle would run until the next reboot and then silently become 10%. |
| `set af <value>` | float, 0–9 | Set raw airtime factor directly. Out-of-range values are rejected: the duty cycle is computed as `100/(af+1)`, so `af = -1` is a divide-by-zero and `af < -1` wraps negative. |
| `set txdelay <value>` | | Accepted for prefs compatibility — **ignored** (txdelay is adaptive) |
| `set rxdelay <value>` | | Accepted for prefs compatibility — **ignored** (rxdelay is adaptive) |
| `set direct.txdelay <value>` | | Accepted for prefs compatibility — **ignored** (direct.txdelay is adaptive) |
| `set backoff.multiplier <m>` | 0.0–2.0 | Per-dupe reactive backoff multiplier (0 = disable reactive backoff) |
| `set flood.max <count>` | 0–64 | Maximum flood retransmit hops |
| `set flood.max.unscoped <count>` | 0–64 | Hop limit for un-scoped floods only (default 64 = same as flood.max); scoped/transport floods still use flood.max |
| `set flood.max.advert <count>` | 0–64 | Hop limit for ADVERT floods only (default 8); curbs advert churn independent of flood.max |
| `set flood.advert.interval <hours>` | `0` (off) or 3–168 | How often the repeater floods its own advertisement. `0` disables periodic flood adverts. |
| `set advert.interval <mins>` | `0` (off) or min–240 | How often the repeater sends local (zero-hop) advertisements. `0` — the default — disables them. Stored halved (the pref holds minutes/2), so odd values round down. |
| `set allow.read.only <on\|off>` | | *(room server only)* Allow or deny read-only client connections |
| `set guest.password <pwd>` | | Set guest access password |
| `set owner.info <text>` | Use `\|` for newlines | Owner/contact information |
| `set int.thresh <value>` | | Interference detection threshold |
| `set buzzer <0\|1\|2\|3>` | or `off` / `on` / `vibrate` / `sound` | *(room server only)* `0`/`off` silent, `1`/`on` sound + vibration, `2`/`vibrate` vibration only, `3`/`sound` sound only. Modes 2 and 3 need a vibration motor; without one the node replies `Error: no vibration motor on this board - use 0 or 1`. Applied live and persisted. Compiled out on repeater builds. |
| `set leds <on\|off\|1\|0>` | default **on** | Master switch for every LED on the node, applied live and persisted: heartbeat, unread-message and LoRa TX-activity LEDs, plus the message and shutdown flashes. Works on every role, including headless repeaters where the TX LED is the only one that ever lights. Does **not** cover the display backlight, which is a separate UI brightness setting. |
| `set leds.radio <tx\|rx\|all\|off>` | default **tx** | What the LoRa activity LED reacts to, applied live and persisted. `tx` lights it for the duration of each transmit (the behaviour before this setting existed), `rx` gives a 30 ms blink per valid packet received, `all` does both, `off` keeps it dark. Sits **below** `set leds` — the master switch off keeps it dark whatever this says. Only boards defining the `lora-tx-led` alias have this LED; elsewhere the value is stored but does nothing, and the reply says so. |
| `set leds.hb <hb\|unread\|all\|off>` | default **all** | What the heartbeat LED reacts to, applied live and persisted. `all` is the 4 s liveness tick that widens from 20 ms to 200 ms while messages are unread (the behaviour before this setting existed), `hb` never widens, `unread` stays dark until there are unread messages, `off` keeps it dark. Also sits below `set leds`. See the LED-topology notes below for what this does on single-LED boards. |
| `set agc.reset.interval <ms>` | Accepted, ignored | Removed — replies `Removed - Automatic AGC reset is on`. The prefs byte is still read and written so the on-flash layout stays byte-exact, but nothing acts on it. |
| `set multi.acks <0\|1>` | | Enable extra ACK transmits |
| `set path.hash.mode <mode>` | 0, 1, or 2 | Path hashing algorithm |
| `set loop.detect <mode>` | `off`, `minimal`, `moderate`, `strict` | Loop detection sensitivity |
| `set radio.rxgain <0\|1\|on\|off>` | | RX gain boost, applied live. Replies `Error: unsupported` on radios without RX boost (SX127x); the pref is still saved. |
| `set radio.fem.rxgain <0\|1\|on\|off>` | default **1** | Routes receive through the external FEM's LNA (`1`) or around it via the FEM's bypass path (`0`), applied live. Sensitivity for battery life — `0` costs roughly 17 dB and saves the LNA's supply current. Transmit, and the driver's idle/sleep gating of the FEM, are unaffected either way. Supported only where the FEM's receive path is software-selectable and that select line is wired to the radio node as `lna-bypass-gpios` — today the KCT8103L boards, `heltec_t096`, `heltec_wireless_tracker_v2`, `heltec_wifi_lora32_v43` and `heltec_wifi_lora32_v4_r8`. Every other board reports `Error: unsupported`: `heltec_wifi_lora32_v4`'s GC1109 has no receive-path select (its CPS is don't-care in RX, same as MeshCore); `station_g2`, `gat562_30s`, `ikoka_nano_30dbm` and `promicro_sx1262` have only the DIO2/TXEN/RXEN transmit-receive switch; `rak3401_1watt`'s SKY66122 is enabled by a standalone always-on regulator outside the radio node; and non-SX126x radios (LR1110, LR2021, SX127x) never implement it. The pref is still saved when unsupported. **Do not expect the FEM's chip-enable to be the knob** — deasserting `antenna-enable-gpios` in RX shuts the part down and takes the through path with it (~69 dB measured on a V4.3), which is what 1.17.2 did before this moved to `lna-bypass-gpios`. |
| `set rxduty <0\|1\|on\|off>` | | RX duty cycle mode *(reboot required)*. Window timing auto-sized per SF/BW/preamble from the SX126x datasheet constraints (boot log line `rxduty:` shows the result). Zero-loss guarantee assumes senders on preamble-32 firmware (current MeshCore at SF≤8); legacy preamble-16 senders are only caught ~50% worst-phase — keep off until the local mesh has converted. Presets with 16-symbol preambles (SF≥9) fall back to continuous RX automatically. |
| `set display.rotate <0\|1\|on\|off>` | default **0** | Rotate the display 180 degrees, for cases and upgrade kits that mount the screen upside down (e.g. the Meshnology N37E for the Wio Tracker L1). Applied live — the driver flips the panel's `SEGMENT_MAP` and `COM_OUTPUT_SCAN`, two bytes on the wire, and the next frame comes out rotated with no redraw and no per-frame cost. **Only full-height SSD1306 and SH1106 panels support this** (`rak4631`, `gat562_30s`, `heltec_wifi_lora32_v4`/`v43`/`v4_r8`, `lilygo_t3s3`, `station_g2`, `wio_tracker_l1`); every other panel replies `Error: this panel cannot rotate` and the pref is **not** saved, so a stored value can never disagree with what the screen shows. `lilygo_timpulse_plus` is excluded despite being an SSD1306: its 64x32 glass is windowed into a 128x64 controller at `page-offset 4`, and the COM-scan reversal flips the controller's whole range, which would move the image off the bonded region. E-paper (SSD16xx) is excluded on purpose: its driver accepts a 180-degree orientation but implements it by flipping the RAM entry mode only, which reverses byte order without reversing bit order inside each byte — it would report success and render wrong. |
| `set input.rotate <0\|1\|on\|off>` | default **0** | Swap the joystick/D-pad axes — up/down and left/right — to match an upside-down mount. Applied live. Deliberately **separate** from `display.rotate`: a case can flip the screen without moving the stick, and boards whose panel cannot rotate can still need the axis swap. Works on every board with directional input, in both the joystick UI and the button UI (where it swaps page-prev/page-next). Non-directional keys, tap codes and long-press gestures are unaffected. |
| `set tz.offset <-12..14>` | default **0** (UTC) | Whole-hour offset from UTC applied when formatting the **on-device clock display** — the top bar, the status page and the joystick System -> Time screen, which show e.g. `UTC+2` instead of `UTC`. Accepts `default`. **Display only, by design.** The RTC itself, `clock`, `clock sync` and `time <epoch>` all stay UTC: they round-trip with each other, apps parse them, and an offset that reached the clock would read as a jump to every timestamp consumer on the node (advert timestamps, the ACL's monotonic `sender_timestamp` gate, MeshTimeSync) — a backward clock silently mutes a node on the mesh. Whole hours only, matching upstream MeshCore's command of the same name, so half-hour zones (+5:30, -3:30) cannot be expressed. Accepted on headless nodes, where it is simply inert. |
| `set adc.multiplier <mult>` | `0` (use board default) or 100–30000 | Battery voltage ADC calibration multiplier, set directly. Rejects non-numeric input, NaN/inf and negatives. |
| `set adc.multiplier target <mv>` | 3000–4400 mV | Calibrate against a voltage you measured with a multimeter: rescales the current multiplier so the ADC reads `<mv>`. Replies with the old and new multiplier plus the before/after reading. `Error: no ADC reading on this board` if the board has no battery ADC. |
| `set adc.multiplier full` | board must be fully charged | Same calibration, but against the board's battery-curve 100% point instead of a hand-measured value. Only meaningful on a full charge. |
| `set meshtimesync <on\|off>` | default **off** | Mesh time sync: automatically correct this node's clock from the consensus of Ed25519-signed advert timestamps heard on the mesh. Steps at most ±1 h per step, one step per 6 h; abstains without a quorum (default 6) of tenured agreeing senders; never overrides a clock set in the last 7 days, whether from GPS (re-armed on every fix) or a manual set. See `MESHTIMESYNC.md`. |
| `set cad.auto <on\|off>` | default **on** | Adaptive CAD: let the staircase controller move the operating detPeak offset based on probe statistics. On by default (repeaters and companions); at the default 15 s probe interval it responds to environment change in ~1–2 h. Turn off to observe/hand-tune via `get cad.stats` + `set cad.offset`. See `ADAPTIVE_CAD.md`. |
| `set cad.offset <n>` | −8 to 12, default 0 | Operating detPeak offset from the chip family's base for the current SF, bandwidth and CAD symbol count (Semtech LoRa Basics Modem reference tables; SX126x ~18–34, LR11xx ~50–85, LR20xx its own symbol-indexed table). Negative = more sensitive LBT (catches weaker signals, risks false busy), positive = less sensitive. Wide range so dense hilltops / quiet valleys can settle far from base. The per-family absolute clamp in the driver (SX126x 12–48, LR11xx 40–100, LR20xx 48–90) is a firmware guardrail against a CAD that never/always fires, not a chip limit (`cadDetPeak` is a full `uint8_t`); the driver reports it so the controller narrows this range to match rather than exploring offsets that collapse onto one peak. Applied live; the auto staircase may move it later if `cad.auto` is on. |
| `set probe.interval <sec>` | 0 (off) or 10–255, default **15** | Seconds between periodic radio measurements. ONE reading serves both: the noise-floor RSSI sample (median of 8) and the CAD calibration probe, which consumes that same reading rather than measuring separately — so this is also the noise-floor sampling rate, and it sets how often an idle repeater wakes. Default 15 s → ~1–2 h CAD staircase response; the floor EMA warms up over 8 samples (~2 min) and its unguarded bypass runs every 16th (~4 min). Longer = fewer wakes, slower to track a changing RF environment. 0 disables CAD probing entirely (also freezes auto adaptation); the floor sampler then falls back to its build-time default. |
| `set cad.busycap <pct>` | 0 (off) or 10–90, default **15** | Faint-tolerance / airtime cap: the max percentage of **quiet-moment probes** that may trip on a faint signal before the staircase backs off to a less sensitive detPeak. Not a percentage of TX attempts — probes are prefiltered to quiet moments, so strong traffic never enters the statistic. On a congested hilltop most busy verdicts are distant traffic won on capture anyway, so deferring for all of it starves the node's own airtime. Self-targeting: a quiet node's busy rate never reaches the cap. Shown as `bc:` in `get cad.stats`. 0 disables the cap (pure knee-seeking). |
| `set cad.reset` | | Full CAD reset: clears the accumulated per-level probe statistics **and** returns the operating detPeak offset to the family base (`cad.offset` = 0), applying it to the radio live and persisting it. Use after a change to the base tables, or to undo a staircase that has walked somewhere unhelpful. Before 1.17.4 this cleared only the statistics, leaving the node to re-converge *from* the walked offset with no evidence for why it was there. `set radio`, `set freq` and the companion app's set-radio-params now run this automatically when they move freq, bandwidth or SF, so it is only needed by hand after a firmware base-table change. |
| `set extra.sf <sf> [sf] [sf]` | up to 3 SFs, `0`/`off` clears | **LR2021 only** (`Error: unsupported` elsewhere) — LoRa *side detectors*: demodulate up to three extra spreading factors concurrently with `sf`, on the same bandwidth, so one repeater can serve several SF communities. Which SF a packet arrived on is a chip-side readout, not a guess. Chip constraints, enforced in the driver and reported as `Error: unsupported or invalid extra SF config`: every extra SF must be **greater** than `sf`, all distinct, highest−lowest ≤ 4, and at BW ≥ 500 kHz at most 2 (only 1 when `sf` ≥ 10). **Receive only, and the bridge it creates is one-way.** TX always uses the single configured `sf`, and all detectors share one bandwidth, so this is multi-SF, not multi-channel. A node with `sf 7` + `extra.sf 8` hears SF8 traffic and *does* forward it — but the forward goes out at SF7, so traffic moves SF8 -> SF7 only and nothing comes back. An SF8 node's direct messages are delivered while its ACKs never arrive, so it retries to its limit every time; adverts and one-way flood traffic propagate fine. Because every extra SF must be **greater** than `sf`, the main SF is always the lowest in the set and TX always uses it — so the bridge direction is fixed at high-SF-in / low-SF-out and **cannot be reversed**. Two nodes back to back both point the same way; there is no configuration that carries SF7 -> SF8. Treat it as a collector for slower-SF stragglers, not as a link between two SF islands. Applied live and restored on every RX entry. **Interaction with CAD:** the chip's SF constraint for CAD is the inverse of the one for RX, so the driver switches side detectors off for each LBT CAD and back on when RX re-arms — two extra SPI commands per TX, no configuration required. Persisted; a set that no longer fits after an `sf`/`bw` change is refused at boot and logged. |
| `set prv.key <hex>` | **128-char hex** (64-byte expanded Ed25519 key) | Replace private key; derive new identity *(reboot to apply)*. The length must be exact — `fromHex` rejects anything else with `Error, bad key`. `get prv.key` returns the same 128-char form. Not USB-gated. |

> **2.4 GHz and the wide bandwidths — LR2021 only.** The LR2021 has a second RF path covering 1.9–2.5 GHz, and the driver picks the band automatically from the frequency: at or above **1500 MHz** it selects the high-band power amplifier, the high-band receive path and high-band front-end calibration (the same split Semtech's own reference BSP uses). Below that nothing changes.
>
> Transmit power is clamped to the active band's ceiling — **+22 dBm sub-GHz, +12 dBm on 2.4 GHz** — so a node carrying `set tx 22` into the 2.4 GHz band transmits at 12 while `get tx` still reports what you asked for.
>
> Bandwidths above 500 kHz (**203, 406, 812, 1000 kHz**, accepted as either the round number or the exact one) are the ordinary 2.4 GHz LoRa steps and exist only on this chip. The CLI refuses them on every other radio, because those drivers silently fall back to 125 kHz for a bandwidth they do not implement — which would put the node on a channel width it never reported.
>
> **Both ends must be on the same band.** This is a separate network, not a bridge to sub-GHz, and range is far shorter at equal power. On a board with two antenna ports (the LR2021 LoRa Plus EVK) the 2.4 GHz signal leaves the **HF** socket, so that antenna has to be connected.

---

## Notes

- **LED topology differs by board, and it changes what `leds.radio` / `leds.hb` can do.** Two DT aliases
  decide it: `lora-tx-led` drives the radio activity LED, and `led0` (falling back to `led1`) drives the
  heartbeat. Of the 35 boards in tree:
  - **Separate pins (6)** — `rak4631`, `sensecap_solar`, `thinknode_m1`, `thinknode_m6`, `lilygo_techo`,
    `xiao_nrf52840`. Both settings are fully independent. `xiao_nrf52840` has three LEDs: blue heartbeat,
    green unread, red radio.
  - **One shared pin (8)** — `heltec_t114`, `heltec_t096`, `heltec_wireless_tracker_v2`, `ttgo_tbeam`,
    `gat562_30s`, `rak3401_1watt`, `rak_wismesh_tag`, `lilygo_timpulse_plus`. Here `lora-tx-led` **is** the
    heartbeat pin, so both settings drive one LED. Radio activity takes priority and the heartbeat yields
    while the radio holds the pin, so a transmit is never blanked mid-packet by the heartbeat's off-timer
    (and vice versa). Use `set leds.hb off` for an unambiguous radio indicator.
  - **No radio LED (20)** — including `wio_tracker_l1`, `t1000_e`, `meshtracker_x1`, Heltec V3/V4/V43 and
    both ProMicros. `set leds.radio` is accepted and stored (so the setting survives onto a board that does
    have the LED) but does nothing; the reply says `(no radio LED on this board)`.
  - **No heartbeat LED (1)** — `lilygo_t3s3` has `lora-tx-led` but neither `led0` nor `led1`, so `set leds.hb`
    is stored and inert, and says so.
- **`unread` needs two things the board may not have.** The second LED lights only where `led0` **and** `led1`
  both exist **and** the build is not a repeater; elsewhere unread indication degrades to the heartbeat pulse
  widening from 20 ms to 200 ms on the single LED. More importantly the unread count comes from the
  **button UI only** — the joystick UI and every repeater/observer report 0 permanently. On those,
  `leds.hb all` behaves identically to `hb`, and `leds.hb unread` leaves the LED **dark forever**.
- **`set leds off` still wins.** Both `leds.radio` and `leds.hb` sit below the master switch; with `leds off`
  the node stays dark whatever they say. All three are applied live and persisted.
- **USB-only commands** — `get acl`, `get prv.key`, `set freq`, `log` (dump), `stats-packets`, `stats-radio`, `stats-core`, `erase` — are blocked when the command arrives over the mesh (remote admin). These are the only ones gated on `sender_timestamp == 0`; `get public.key` and `set prv.key` are **not** among them.
- **Adaptive contention window** — `txdelay`, `rxdelay`, and `direct.txdelay` are accepted and stored for Arduino prefs compatibility but have no effect. Use `get txdelay` to inspect the current adaptive state and `set backoff.multiplier` to tune reactive backoff.
- **Region load mode** — after `region load`, every line received is parsed as a region entry until a blank line is sent. The loaded map is only committed to the live region tree at that point; use `region save` to persist it. Region rows must be indented by at least one space, so an **unindented line that starts with a name character aborts the mode and is executed as a normal command** — the escape hatch if a `region load` is started by accident or a client dies mid-transfer. An abort discards the partial map, leaving the live region tree untouched. The exported wildcard header line `*` stays unindented and is ignored as before, so pasting the output of `region` still loads cleanly.
- **Reboot delay** — `start dfu`, `start ota` (nRF52 BLE-DFU path only), `reboot`, `poweroff`, `clkreboot` and `erase` defer the reset by **2 seconds** so the reply can be transmitted over LoRa first. On a companion the handler then keeps deferring in 20 ms steps until the BLE/USB transport has drained, up to a further 3 s grace. On ESP32 `start ota` starts a WiFi AP + HTTP server and does **not** reboot.
