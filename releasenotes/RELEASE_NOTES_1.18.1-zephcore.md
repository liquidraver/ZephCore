# ZephCore 1.18.1-zephcore

Firmware updates over Bluetooth for ESP32 companions, half-hour and 45-minute timezones on the
on-screen clock, the clock zone on repeater screens, and a Zephyr update.

> [!NOTE]
> A normal upgrade keeps your identity, settings, contacts and phone pairing.

---

## Bluetooth updates for ESP32 companions

ESP32 companions can now be updated from a phone over Bluetooth, the way nRF companions already
can. Send `start ota` (or pick DFU / BLE OTA on the device's screen menu): the node opens an
update window for 10 minutes and keeps running. Then, in **nRF Connect Device Manager** (iOS and
Android), connect to the node, pair with its PIN if asked, and upload the `<board>-update.bin`
from the release (choose "Confirm only"). The node restarts on the new firmware by itself and
keeps your identity, settings, contacts and pairing. An update takes about a minute from a phone.

- Use the `-update.bin`, not the `-merged.bin`: the merged file is for flashing over USB.
- The Nordic DFU app used for nRF boards does not work here; use nRF Connect Device Manager.
- The first update to this version still goes over USB or the web flasher. From then on, updates
  can go over Bluetooth.
- `stop ota` closes the window early. Only a phone paired with the node's PIN can upload.
- Available on the Heltec V4, V4.3, V4 R8 and Wireless Tracker V2, LilyGo T3S3, Meshnology W12,
  Station G2, and the XIAO ESP32-S3 and ESP32-C3. The Heltec V3, Heltec Wireless Tracker, LilyGo
  T-Lora C6 and XIAO ESP32-C6 do not have the memory for it next to their WiFi companion, and the
  classic ESP32 boards (T-Beam) cannot boot an update this way; they update over USB as before.

On boards without an over-the-air update, `start ota` used to answer as if it was rebooting into
an update mode and then did nothing; it now says that the build has no OTA. Selecting DFU on the
screen menu of an ESP32 board no longer blanks the screen and leaves the node in an undefined
state.

## Half-hour timezones on the clock

`set tz.offset` now takes half and quarter hours, as upstream MeshCore now does (after its 1.18.0):
`set tz.offset 5.5` for India (+5:30), `5.75` for Nepal (+5:45), `-3.5` for Newfoundland. The
display shows the zone as `UTC+5:30`. Values that are not a whole quarter hour are refused, and
`get tz.offset` now always prints at least one decimal (`2.0`, `5.75`). As before, only the
on-screen clock moves; the node's own clock and every timestamp it sends stay UTC.

Repeaters and room servers with a screen now show their clock in the zone you set. Before, they
accepted `set tz.offset` but kept showing UTC.

If you go back to an older ZephCore, a half-hour setting reads as the whole hour below it (5.5
becomes 5).

## Other changes

- **WiFi companions no longer freeze when the app stops reading.** If an app connected over WiFi
  stopped reading (for example in the middle of a contact sync), the node could stop answering on
  USB, Bluetooth and WiFi, and stay stuck as long as that connection stayed open. The node now
  drops such a connection after 2 seconds, and the app can reconnect.
- **`set leds.hb off` and `hb` now also stop the message blink.** Before, a node with the heartbeat
  LED off still flashed on every DM and channel message. `set leds on` with `set leds.hb off` now keeps
  a T1000-E dark in use while it still blinks three times when you switch it off with the buzzer muted;
  `set leds off` keeps it dark at power-off too.
- **Packets the radio never checked are no longer repeated.** A LoRa header can say a packet
  carries no CRC, and the radio then passes it on without checking it. MeshCore always sends a CRC,
  so such a packet is a misread header or noise; it is now dropped like a packet with a bad CRC, on
  every radio type. It is rare (one in about 48 000 receptions on a busy repeater) and it does not
  explain the damaged packet copies seen on map.meshcore.hu.
- **Zephyr** updated to `b70dbaf045e`. A GPS receiver that fails to start on the first try is now
  really tried again (the two retries did nothing before), and on boards with external flash a
  failed start of that flash is retried at the next mount instead of staying failed until reboot.
