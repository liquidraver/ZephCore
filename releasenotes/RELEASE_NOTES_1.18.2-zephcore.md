# ZephCore 1.18.2-zephcore

Firmware updates over Bluetooth for ESP32 companions, and packets the radio never checked are no
longer repeated.

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

## Other changes

- **Packets the radio never checked are no longer repeated.** A LoRa header can say a packet
  carries no CRC, and the radio then passes it on without checking it. MeshCore always sends a CRC,
  so such a packet is a misread header or noise; it is now dropped like a packet with a bad CRC, on
  every radio type. It is rare (one in about 48 000 receptions on a busy repeater) and it does not
  explain the damaged packet copies seen on map.meshcore.hu.
