# Repeater CLI Commands

All commands are sent over USB serial (CDC-ACM). Commands sent remotely over the mesh (non-zero `sender_timestamp`) cannot access USB-only commands.

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
| `clock sync` | Sync clock from the sender's timestamp (only advances, cannot go backwards) |
| `time <unix_timestamp>` | Set RTC to a specific Unix timestamp (cannot go backwards) |

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
| `password <new_password>` | Set the admin password |
| `setperm <perms_hex> <pubkey_hex>` | Set ACL permissions for a node (app format: 2-char hex perms first) |
| `setperm <pubkey_hex> <perms_dec>` | Set ACL permissions for a node (Arduino format: pubkey first, decimal perms) |
| `get acl` | *(USB only)* List all ACL entries with permissions and public keys |

---

## Region Filtering

Regions control which flood packets the repeater forwards. The region tree is hierarchical; the wildcard `*` region is the root.

| Command | Description |
|---------|-------------|
| `region` | Export the current region map (indented text tree) |
| `region load` | Enter interactive region load mode. Paste indented region lines; send a blank line to commit |
| `region save` | Save the current region map to persistent storage |
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
| `get tx` | TX power: fixed dBm or APC status |
| `get lat` | Stored latitude |
| `get lon` | Stored longitude |
| `get dutycycle` | Duty cycle as percentage (e.g. "50.0%") |
| `get af` | Raw airtime factor value |
| `get txdelay` | Adaptive TX delay status: contention estimate and flood delay factor |
| `get rxdelay` | *(deprecated)* Always returns "adaptive (rxdelay deprecated)" |
| `get direct.txdelay` | *(deprecated)* Always returns "adaptive (direct.txdelay deprecated)" |
| `get backoff.multiplier` | Per-dupe reactive backoff multiplier |
| `get flood.max` | Max flood retransmit hops |
| `get flood.advert.interval` | Flood advertisement interval in hours |
| `get advert.interval` | Local advertisement interval in minutes |
| `get apc.margin` | Adaptive Power Control target RSSI margin in dB |
| `get allow.read.only` | Whether read-only clients are allowed |
| `get guest.password` | Guest access password |
| `get owner.info` | Owner/contact info (pipes `\|` display as newlines) |
| `get int.thresh` | Interference threshold |
| `get agc.reset.interval` | AGC reset interval in ms (stored in 4 s steps) |
| `get multi.acks` | Extra ACK transmit count (`0` or `1`) |
| `get path.hash.mode` | Path hashing algorithm: `0`, `1`, or `2` |
| `get loop.detect` | Loop detection level: `off`, `minimal`, `moderate`, or `strict` |
| `get radio.rxgain` | RX gain boost: `0` or `1` |
| `get rxduty` | RX duty cycle mode: `0` or `1` |
| `get dc.restarts` | Duty-cycle preamble false-positive re-arm counter. High values mean the preamble detector is tripping on noise/interference without real packets arriving — inflates RX-on time and drains battery. Reset by `clear stats`. |
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
| `set tx <dbm\|apc>` | −9 to board max (default 30), or `apc` | Set TX power fixed or enable Adaptive Power Control |
| `set lat <latitude>` | | Set stored latitude |
| `set lon <longitude>` | | Set stored longitude |
| `set dutycycle <pct>` | 1–100 | Set duty cycle percentage (converted to airtime factor internally) |
| `set af <value>` | float | Set raw airtime factor directly |
| `set txdelay <value>` | | Accepted for prefs compatibility — **ignored** (txdelay is adaptive) |
| `set rxdelay <value>` | | Accepted for prefs compatibility — **ignored** (rxdelay is adaptive) |
| `set direct.txdelay <value>` | | Accepted for prefs compatibility — **ignored** (direct.txdelay is adaptive) |
| `set backoff.multiplier <m>` | 0.0–2.0 | Per-dupe reactive backoff multiplier (0 = disable reactive backoff) |
| `set flood.max <count>` | 0–64 | Maximum flood retransmit hops |
| `set flood.advert.interval <hours>` | 3–168 | How often the repeater floods its own advertisement |
| `set advert.interval <mins>` | min–240 | How often the repeater sends local advertisements |
| `set apc.margin <db>` | 6–30 | Target RSSI margin for Adaptive Power Control |
| `set allow.read.only <on\|off>` | | Allow or deny read-only client connections |
| `set guest.password <pwd>` | | Set guest access password |
| `set owner.info <text>` | Use `\|` for newlines | Owner/contact information |
| `set int.thresh <value>` | | Interference detection threshold |
| `set agc.reset.interval <ms>` | Rounded to 4 s | AGC reset interval |
| `set multi.acks <0\|1>` | | Enable extra ACK transmits |
| `set path.hash.mode <mode>` | 0, 1, or 2 | Path hashing algorithm |
| `set loop.detect <mode>` | `off`, `minimal`, `moderate`, `strict` | Loop detection sensitivity |
| `set radio.rxgain <0\|1>` | | RX gain boost *(reboot required)* |
| `set rxduty <0\|1>` | | RX duty cycle mode *(reboot required)* |
| `set adc.multiplier <mult>` | (0 = use board default) | Battery voltage ADC calibration multiplier |
| `set prv.key <hex>` | 64-char hex (32-byte key) | Replace private key; derive new identity *(reboot to apply)* |

---

## Notes

- **USB-only commands** — `get acl`, `get prv.key`, `get public.key`, `set freq`, `log` (dump), `stats-*`, `erase` — are blocked when the command arrives over the mesh (remote admin).
- **Adaptive contention window** — `txdelay`, `rxdelay`, and `direct.txdelay` are accepted and stored for Arduino prefs compatibility but have no effect. Use `get txdelay` to inspect the current adaptive state and `set backoff.multiplier` to tune reactive backoff.
- **Region load mode** — after `region load`, every line received is parsed as a region entry until a blank line is sent. The loaded map is only committed to the live region tree at that point; use `region save` to persist it.
- **Reboot delay** — `start dfu`, `start ota`, and `reboot` defer the reboot by ~600 ms so the reply can be transmitted over LoRa before the device resets.
