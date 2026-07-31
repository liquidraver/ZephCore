# Repeater CLI Commands

All commands are sent over USB serial (CDC-ACM). Commands sent remotely over the mesh (non-zero `sender_timestamp`) cannot access USB-only commands.

> The **Room Server** role shares this CLI — the common commands (radio, region, password, advert, gps, etc.) plus `setperm` / `get acl` all apply.

**Sources:**
- `helpers/CommonCLI.cpp` — common commands shared by all roles
- `app/RepeaterMesh.cpp` — repeater-specific commands

---

## System

| Command | Description |
|---------|-------------|
| `ver` | Firmware version and build date |
| `board` | Board manufacturer name |
| `reboot` | Reboot immediately |
| `start dfu` | Reboot into UF2 bootloader for drag-and-drop firmware update |
| `start ota` | ESP32: start WiFi AP + HTTP OTA server. nRF52: reboot into BLE OTA DFU mode |
| `stop ota` | Stop WiFi OTA server (ESP32 only) |
| `clkreboot` | Set clock to a fixed reference time (15 May 2024 8:50pm UTC) then reboot |
| `powersaving` | Not implemented |

---

## Clock

| Command | Description |
|---------|-------------|
| `clock` | Display current UTC time |
| `clock sync` | Sync clock from the sender's timestamp (only advances, cannot go backwards). Arms the 7-day mesh-time-sync suppression window. |
| `time <unix_timestamp>` | Set RTC to a specific Unix timestamp (cannot go backwards). Arms the 7-day mesh-time-sync suppression window. |

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
| `neighbor.remove <pubkey_hex>` | Remove a neighbor entry by its public key |
| `discover.neighbors` | Broadcast a node discovery request to find nearby nodes |

---

## Security & Access Control

| Command | Description |
|---------|-------------|
| `password <new_password>` | Set the admin password (**max 15 characters**) |
| `setperm <perms_hex> <pubkey_hex>` | Set ACL permissions for a node (app format: 2-char hex perms first) |
| `setperm <pubkey_hex> <perms_dec>` | Set ACL permissions for a node (Arduino format: pubkey first, decimal perms) |
| `get acl` | *(USB only)* List all ACL entries with permissions and public keys |

> **Password length:** admin and guest passwords are capped at **15 characters** (16-byte storage incl. NUL; same limit as Arduino MeshCore). The login-send path silently truncates anything longer, so a password >15 chars will never authenticate. Applies to `set guest.password` as well.

> **Guest access is off unless a guest password is set.** An empty `guest.password` (the default) disables guest login rather than matching a blank submitted password. To run an open room-server, use `set allow.read.only on` — that grants read-only (`PERM_ACL_GUEST`), not post rights.

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
| `region load` | Enter interactive region load mode. Paste indented region lines; send a blank line to commit |
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
| `erase` | *(USB only)* Format the entire filesystem |

---

## GPS

| Command | Description |
|---------|-------------|
| `gps` | Show GPS status (`on` or `off`) |
| `gps on` | Enable GPS module |
| `gps off` | Disable GPS module |
| `gps setloc` | Update stored latitude/longitude from current GPS fix |
| `gps advert` | Show current location advertising policy |
| `gps advert none` | Do not include location in advertisements |
| `gps advert share` | Include live GPS location in advertisements |
| `gps advert prefs` | Include stored lat/lon from prefs in advertisements |
| `set gps duty <sec>` | GPS duty interval (standby seconds between fixes). `0` = always-on (continuous; streams fresh fixes, can download a full almanac). Floor 10s, cap 604800 (1 week). Persists to flash, applied live. |
| `set gps duty default` | Reset GPS duty to the role default (repeater/room 48h, companion 300s) |

---

## Sensor Settings

| Command | Description |
|---------|-------------|
| `sensor list [<start_idx>]` | List custom sensor settings (paginated at 134 chars) |
| `sensor get <key>` | Get a custom sensor setting value by key |
| `sensor set <key> <value>` | Set a custom sensor setting value |

---

## Radio (Temporary Override)

| Command | Description |
|---------|-------------|
| `tempradio <freq> <bw> <sf> <cr> <timeout_mins>` | Apply temporary radio parameters; automatically reverts after `timeout_mins`. Constraints: freq 150–2500 MHz, bw 7–500 kHz, sf 5–12, cr 5–8. Saved prefs are never mutated — concurrent `set` commands and reboots both restore the real saved values. |

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

## `get` — Read Configuration

| Command | Returns |
|---------|---------|
| `get name` | Node name |
| `get role` | Firmware role (`repeater`) |
| `get repeat` | Forwarding enabled: `on` or `off` |
| `get radio` | Radio params: `freq,bw,sf,cr` |
| `get freq` | Frequency in MHz |
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
| `get allow.read.only` | Whether read-only clients are allowed |
| `get guest.password` | Guest access password |
| `get owner.info` | Owner/contact info (pipes `\|` display as newlines) |
| `get int.thresh` | Interference threshold |
| `get leds` | LED master switch: `on` or `off` |
| `get agc.reset.interval` | Removed - replies `use rxduty instead` |
| `get multi.acks` | Extra ACK transmit count (`0` or `1`) |
| `get path.hash.mode` | Path hashing algorithm: `0`, `1`, or `2` |
| `get loop.detect` | Loop detection level: `off`, `minimal`, `moderate`, or `strict` |
| `get radio.rxgain` | RX gain boost: `0` or `1` |
| `get rxduty` | RX duty cycle mode: `0` or `1` |
| `get gps duty` | Now-effective GPS duty interval in seconds (`always on (0)` when continuous) |
| `get meshtimesync` | Mesh time-sync state + live dry-run: on/off, eligible voter count, votes for/against, consensus skew and radius, would-be verdict (`ok`/`in-band`/`step±N`/`abstain (reason)`/`hold (reason)`; a recent clock set — manual or GPS — shows as `hold (suppressed)`, and a backward step a forward-only role would refuse is annotated `(skipped: forward-only)`), step counters, suppression countdown, and a per-sender evidence table (`prefix hops count skew E`, `E` = counted toward the verdict above). Entries that count print first, so a size-capped reply never hides the ones that explain the summary; if the table doesn't fully fit, a trailing `+N more` shows how many were left out. Sensing runs even while off, so this works as a dry-run before enabling. Over remote admin the reply is truncated to the packet size (summary always fits); the full table needs the USB CLI. |
| `get probe.interval` | Seconds between periodic radio measurements (noise-floor sample + CAD probe). 0 = CAD probing off |
| `get dc.restarts` | Duty-cycle preamble false-positive re-arm counter (RxTimeout re-arms + parked-RX watchdog recoveries). High values mean the preamble detector is tripping on noise/interference without real packets arriving — inflates RX-on time and drains battery; packets are never lost to it. Reset by `clear stats`. |
| `get cad` | Adaptive-CAD status: header (`a` auto on/off, `o` operating detPeak offset, `pk` absolute peak with family base, `sp` noise-floor RSSI burst quality as `mean-spread-dB/zero-spread-%` (plus `(burst-count)` on the local USB console, omitted over the air to protect the 161 B reply budget) — a non-zero mean proves the 8 reads are independent however high the share climbs; only mean `0.0` with a high share indicts the sampler. See `ADAPTIVE_CAD.md`. `bc` busy cap), then a 3-rung window around the operating offset (`*` marks it) with probe/busy/fp/tp counts and false-positive rate — the three levels the knee controller reads. Probing runs even while `cad.auto` is off (dry-run), so this is the observation tool for picking a site-appropriate detPeak. See `ADAPTIVE_CAD.md`. Not available on SX127x boards (no hardware CAD). |
| `get adc.multiplier` | Battery voltage ADC calibration multiplier |
| `get bootloader.ver` | Bootloader version string |
| `get public.key` | *(USB only)* Node's public key as hex |
| `get prv.key` | *(USB only)* Node's private key as hex |

---

## `set` — Write Configuration

Changes are persisted immediately unless noted. Some require a reboot.

| Command | Constraints | Description |
|---------|-------------|-------------|
| `set name <name>` | No `[ ] \ : , ? *` | Set node name |
| `set repeat <on\|off>` | | Enable or disable packet forwarding |
| `set radio <freq> <bw> <sf> <cr>` | freq 150–2500, bw 7–500, sf 5–12, cr 5–8 | Set radio params *(reboot required)* |
| `set freq <mhz>` | 150–2500 *(USB only)* | Set frequency alone *(reboot required)* |
| `set tx <dbm>` | −9 to board max (default 30) | Set TX power |
| `set lat <latitude>` | | Set stored latitude |
| `set lon <longitude>` | | Set stored longitude |
| `set dutycycle <pct>` | 1–100 | Set duty cycle percentage (converted to airtime factor internally) |
| `set af <value>` | float | Set raw airtime factor directly |
| `set txdelay <value>` | | Accepted for prefs compatibility — **ignored** (txdelay is adaptive) |
| `set rxdelay <value>` | | Accepted for prefs compatibility — **ignored** (rxdelay is adaptive) |
| `set direct.txdelay <value>` | | Accepted for prefs compatibility — **ignored** (direct.txdelay is adaptive) |
| `set backoff.multiplier <m>` | 0.0–2.0 | Per-dupe reactive backoff multiplier (0 = disable reactive backoff) |
| `set flood.max <count>` | 0–64 | Maximum flood retransmit hops |
| `set flood.max.unscoped <count>` | 0–64 | Hop limit for un-scoped floods only (default 64 = same as flood.max); scoped/transport floods still use flood.max |
| `set flood.max.advert <count>` | 0–64 | Hop limit for ADVERT floods only (default 8); curbs advert churn independent of flood.max |
| `set flood.advert.interval <hours>` | 3–168 | How often the repeater floods its own advertisement |
| `set advert.interval <mins>` | min–240 | How often the repeater sends local advertisements |
| `set allow.read.only <on\|off>` | | Allow or deny read-only client connections |
| `set guest.password <pwd>` | | Set guest access password |
| `set owner.info <text>` | Use `\|` for newlines | Owner/contact information |
| `set int.thresh <value>` | | Interference detection threshold |
| `set leds <on\|off\|1\|0>` | default **on** | Master switch for every LED on the node, applied live and persisted: heartbeat, unread-message and LoRa TX-activity LEDs, plus the message and shutdown flashes. Works on every role, including headless repeaters where the TX LED is the only one that ever lights. Does **not** cover the display backlight, which is a separate UI brightness setting. |
| `set agc.reset.interval <ms>` | Accepted, ignored | Removed - replies `use rxduty instead` |
| `set multi.acks <0\|1>` | | Enable extra ACK transmits |
| `set path.hash.mode <mode>` | 0, 1, or 2 | Path hashing algorithm |
| `set loop.detect <mode>` | `off`, `minimal`, `moderate`, `strict` | Loop detection sensitivity |
| `set radio.rxgain <0\|1\|on\|off>` | | RX gain boost, applied live. Replies `Error: unsupported` on radios without RX boost (SX127x); the pref is still saved. |
| `set rxduty <0\|1\|on\|off>` | | RX duty cycle mode *(reboot required)*. Window timing auto-sized per SF/BW/preamble from the SX126x datasheet constraints (boot log line `rxduty:` shows the result). Zero-loss guarantee assumes senders on preamble-32 firmware (current MeshCore at SF≤8); legacy preamble-16 senders are only caught ~50% worst-phase — keep off until the local mesh has converted. Presets with 16-symbol preambles (SF≥9) fall back to continuous RX automatically. |
| `set adc.multiplier <mult>` | (0 = use board default) | Battery voltage ADC calibration multiplier |
| `set meshtimesync <on\|off>` | default **off** | Mesh time sync: automatically correct this node's clock from the consensus of Ed25519-signed advert timestamps heard on the mesh. Steps at most ±1 h per step, one step per 6 h; abstains without a quorum (default 6) of tenured agreeing senders; never overrides a clock set in the last 7 days, whether from GPS (re-armed on every fix) or a manual set. See `MESHTIMESYNC.md`. |
| `set cad.auto <on\|off>` | default **on** | Adaptive CAD: let the staircase controller move the operating detPeak offset based on probe statistics. On by default (repeaters and companions); at the default 15 s probe interval it responds to environment change in ~1–2 h. Turn off to observe/hand-tune via `get cad` + `set cad.offset`. See `ADAPTIVE_CAD.md`. |
| `set cad.offset <n>` | −8 to 12, default 0 | Operating detPeak offset from the chip family's per-SF base (SX126x: SF+13; LR11xx/LR20xx: 56–68 table). Negative = more sensitive LBT (catches weaker signals, risks false busy), positive = less sensitive. Wide range so dense hilltops / quiet valleys can settle far from base. The per-family absolute clamp in the driver (SX126x 15–40, LR 48–90) is a firmware guardrail against a CAD that never/always fires, not a chip limit (`cadDetPeak` is a full `uint8_t`). Applied live; the auto staircase may move it later if `cad.auto` is on. |
| `set probe.interval <sec>` | 0 (off) or 10–255, default **15** | Seconds between periodic radio measurements. ONE reading serves both: the noise-floor RSSI sample (median of 8) and the CAD calibration probe, which consumes that same reading rather than measuring separately — so this is also the noise-floor sampling rate, and it sets how often an idle repeater wakes. Default 15 s → ~1–2 h CAD staircase response; the floor EMA warms up over 8 samples (~2 min) and its unguarded bypass runs every 16th (~4 min). Longer = fewer wakes, slower to track a changing RF environment. 0 disables CAD probing entirely (also freezes auto adaptation); the floor sampler then falls back to its build-time default. |
| `set cad.busycap <pct>` | 0 (off) or 10–90, default **25** | Airtime-protection cap: the max percentage of TX attempts the node will let CAD defer before the staircase backs off to a less sensitive detPeak — counting **real** traffic, not just false positives. On a congested hilltop most busy verdicts are distant traffic won on capture anyway, so deferring for all of it starves the node's own airtime. Self-targeting: a quiet node's busy rate never reaches the cap. Shown as `bc:` in `get cad`. 0 disables the cap (pure knee-seeking). |
| `set cad.reset` | | Clear the accumulated per-level CAD probe statistics (RAM only; also cleared automatically on any radio parameter change). |
| `set prv.key <hex>` | 64-char hex (32-byte key) | Replace private key; derive new identity *(reboot to apply)* |

---

## Notes

- **USB-only commands** — `get acl`, `get prv.key`, `get public.key`, `set freq`, `log` (dump), `stats-*`, `erase` — are blocked when the command arrives over the mesh (remote admin).
- **Adaptive contention window** — `txdelay`, `rxdelay`, and `direct.txdelay` are accepted and stored for Arduino prefs compatibility but have no effect. Use `get txdelay` to inspect the current adaptive state and `set backoff.multiplier` to tune reactive backoff.
- **Region load mode** — after `region load`, every line received is parsed as a region entry until a blank line is sent. The loaded map is only committed to the live region tree at that point; use `region save` to persist it.
- **Reboot delay** — `start dfu`, `start ota`, and `reboot` defer the reboot by ~600 ms so the reply can be transmitted over LoRa before the device resets.
