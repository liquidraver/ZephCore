# ZephCore 1.17.7-zephcore

Faster GPS fixes on three trackers, a clock that survives a reboot, fixes for USB companions that
could stop responding right after boot or when the port closed during a contact sync, a contacts
import that no longer stalls every few seconds, more reliable detection of clock chips, repeaters
that power down an external flash chip they do not use, a formatter that erases that chip on every
board that has one, and a few fixes ported from upstream MeshCore.

> [!NOTE]
> A normal upgrade keeps your identity, settings, contacts and phone pairing.

---

## GPS: a fix in seconds instead of minutes

On some boards the GPS receiver lost its satellite data every time it went to sleep, so each wake was
a search from scratch. Three boards now keep that data:

- **SenseCAP T1000-E and MeshTracker X1**: the receiver uses its low-power backup sleep. On a
  T1000-E, a fix after a minute asleep took about 9 seconds instead of about 77.
- **SenseCAP Solar**: the receiver stays in standby between fixes instead of being switched off,
  which makes a fix in marginal reception more likely.

Keeping that data uses a little power all the time, so it is only done when the GPS interval is one
hour or less. Longer intervals (a repeater's default is 48 hours) switch the receiver off completely,
and the next fix is a search from scratch. The limit can be changed with `set gps standby <seconds>`
(`0` = always switch off, `default` = one hour); `get gps standby` shows it.

`gps off` also switches the receiver off completely, on every board.

A fix that starts from scratch needs more time, most of all in poor reception. So whenever the
receiver was switched off completely (a GPS interval above the limit, `gps off` then `gps on`, a
boot), a companion gives the next fix 5 minutes instead of 2 before it gives up until the next
interval. Repeaters and room servers already allow 5 minutes every time.

These rules are the same on every board, and a board added later follows them without any extra
work. A board with only one way to switch its GPS off uses that way on both sides of the limit.

> [!NOTE]
> **T1000-E and MeshTracker X1:** until now the receiver kept its clock supply through every sleep,
> whatever the interval, and through `gps off`. With a GPS interval above one hour, and after
> `gps off`, it is now switched off completely, so the next fix takes longer.
> `set gps standby 604800` keeps the old behaviour for intervals up to a week.

## The clock survives a reboot

A reboot no longer sends the clock back to 1970 on boards without a clock chip. The time is restored
at boot and is at most a few seconds behind. A full power loss still resets it until the next GPS fix
or phone sync.

## Clock chips are recognised more reliably

Some boards carry a battery-backed clock chip (DS3231/DS3232/DS1307, PCF8563, RV-3028, RX8130CE). At
boot the firmware checks that the device answering at the chip's address really is that chip, because
other parts can share the address. That check had two faults:

- **A real clock chip could be skipped.** If another firmware had left it in 12-hour mode, or with a
  year out of range, it was treated as "not a clock": no time was read from it and none was ever
  written back, so it stayed that way.
- **An erased memory chip at the same address could be taken for a clock.**

The check now uses the bits each chip's data sheet defines as always zero, and a device is passed over
only when two reads both rule it out. A time is restored only from a read in which every field is
valid.

Thanks to **ptr727** for finding and fixing this
([PR #99](https://github.com/liquidraver/ZephCore/pull/99)).

Four more clock-chip fixes, also from **ptr727**:

- **RAK4631 with a RAK12002 clock module**: the module is now used, and its switchover to the backup
  supply is stored in the chip, so the clock keeps running when the board loses power
  ([PR #98](https://github.com/liquidraver/ZephCore/pull/98)).
- **RX8130CE**: the weekday was written in the wrong encoding. Time and date were not affected
  ([PR #101](https://github.com/liquidraver/ZephCore/pull/101)).
- **A time outside 2000-2099 is no longer written to a clock chip.** The chip stores only two digits
  of the year, so a time before 2000 came back after a reboot as a date late in this century
  ([PR #105](https://github.com/liquidraver/ZephCore/pull/105)).
- **RV-3028 left in 12-hour mode by other firmware**: it is switched back to 24-hour mode at boot, so
  the hours are read correctly
  ([PR #106](https://github.com/liquidraver/ZephCore/pull/106)).

## USB companion: a hang right after boot

A companion on USB could stop responding when the host sent it a command straight after opening the
port following a boot: nothing more came out of the USB port and the node stopped handling mesh
traffic, while Bluetooth kept advertising. Only a reset recovered it.

The cause is in the USB serial driver of the Zephyr version 1.17.6 moved to: with its first 64-byte
buffer already full, it reported free space but accepted no data, and the firmware retried forever.
The driver is patched.

The same patch covers a second way into the same loop: a host computer going to sleep with the port
open.

## USB companion: a hang when the port closed during a contact sync

A companion sending its contact list could get stuck when the last connected app went away before
the list was complete: the app or browser tab was closed, or the tool exited. The node stayed in the
loop that sends the list, with nobody to send it to, and handled nothing else until an app connected
again; that app then received the rest of the list unasked. Present since 1.17.6.

The loop ran for as long as no app was busy. With no app left none is busy and nothing can be sent,
so it never ended. It now also stops when no app is connected.

## USB companion: a host that goes away without closing the port

A USB session ended only when the host closed the port or the cable lost power. A host that went
away any other way (a USB hub resetting, a computer going to sleep with the app open) left the node
treating the app as still connected. With one app that did no harm. With a phone on Bluetooth at the
same time, the node could stop answering the phone once the data waiting for the USB side filled its
buffer.

Now a reset of the USB bus ends the session, and a session on a sleeping computer does not count as a
connected app until the computer wakes. An app that keeps the port open but stops reading can still
hold up a second app.

## Companion: importing contacts no longer stalls every few seconds

Importing a configuration with many contacts from the app paused for a few seconds, again and again.
The node saved its whole contact list 5 seconds after the first change, and while the import was
still running it saved again every 5 seconds; it answers nothing while it saves.

The save now waits until 5 seconds after the last change, as in MeshCore, so an import is saved once
when it is done. The same applies to any other run of contact changes.

## Repeaters: the external flash chip is powered down

Some boards carry a second flash chip, which companions keep contacts and channels on. Repeaters,
room servers and observers store nothing on it. Since 1.17.6 those roles no longer set the chip up at
all, and that left its control pins unconnected, so the chip could sit selected for as long as the
node ran instead of idling. 1.17.4 and earlier set the chip up at boot on every role.

These roles now set the chip up once at start-up and put it into its deep power-down mode, with
chip-select held high. A factory `erase` wakes the chip, erases it and puts it back. Companions are
unchanged: they use the chip, so it stays in standby.

Boards with this chip: T-Echo, T-Impulse Plus, MeshTracker X1, SenseCAP Solar, ThinkNode M1,
ThinkNode M6, Wio Tracker L1, Wio Tracker L1 Pro 1W and XIAO nRF52840.

This was found while looking into a report of a XIAO nRF52840 repeater drawing about 4 mA more on
1.17.6 than on 1.17.4 (thanks to **Kimotu**). Whether it accounts for that difference is not
confirmed yet.

## Formatter: the external flash chip is erased on every board that has one

The nRF52 formatters (`SoftDevice_v6_formatter` and `SoftDevice_v7_formatter`, as `.uf2` and as the
`.zip` the Mesh America configurator uses for its erase step) wipe a node completely: the internal storage, the
Bluetooth pairings and the external flash chip. On some boards the formatter did not reach that chip,
left it as it was and still reported success. A companion flashed afterwards then found the old
contacts and channels on it.

- **MeshTracker X1 and T-Impulse Plus**: the formatter did not know how the chip is wired on these
  boards. It does now.
- **Boards that switch the chip's power supply** (ThinkNode M1, T-Echo, T-Impulse Plus,
  MeshTracker X1): the formatter now switches the supply on itself and waits for the chip to start.
- **A chip in deep power-down**: a repeater, room server or observer on this release puts the chip to
  sleep (see above), and a sleeping chip answers nothing. The formatter now wakes it first.

The erase is also checked: the formatter reads the chip back afterwards and reports a failure if it
is not blank. The last lines of its serial log say whether the external chip was erased, or that no
such chip answered, which is normal on a board without one.

Use the formatters from this release; the file names have not changed. As before, pick the one that
matches the SoftDevice version of your bootloader.

## Also in this release

- **A repeater with adverts off is no longer discoverable**, as in upstream MeshCore: with both
  `flood.advert.interval` and `advert.interval` at `0` it stops answering discovery requests (the
  app's repeater discovery, another repeater's `discover.neighbors`). With either one above `0`, and
  with the defaults (47 hours and `0`), it answers as before. If a repeater is missing from discovery
  after the update, check `get flood.advert.interval`: on firmware before 1.17.4,
  `set flood.advert.interval default` stored `0` by mistake. `set flood.advert.interval default` now
  puts it back to 47 hours.
- **Malformed encrypted packets are rejected earlier**, before any cryptography runs.
- **Large contact lists**: up to 24 contacts can share the same one-byte hash, up from 8.
- **ESP32-S3, ESP32-C3 and ESP32-C6 boards report the MCU temperature** in telemetry, as nRF52 boards
  already did ([#107](https://github.com/liquidraver/ZephCore/issues/107), thanks to **svenlange2**).
  The older ESP32 in the TTGO LoRa32 and T-Beam has no usable sensor and still reports none. On the
  ThinkNode M9 and Meshnology W12 the radio now also recalibrates itself when the temperature has
  moved by 5 C or more, as it already does on nRF boards with an LR1110 or LR2021.
- **`sensor list`** with a negative start index is rejected instead of being used
  ([PR #100](https://github.com/liquidraver/ZephCore/pull/100), thanks to **ptr727**).
- **T1000-E**: a pin that was wrongly driven as a sensor enable is left alone. Sensor readings are
  unchanged.
- **LR2021 with `rxduty` on** (MeshTracker X1, both LR2021 EVK kits): after every noise-floor
  measurement the chip rejected the command that puts it back into its duty cycle. The next channel
  check recovered it and no packets were lost; the driver now returns the chip to standby first, and
  the command is accepted.
- **Joystick UI** (Wio Tracker L1 and other joystick boards): switching the GPS on from the GPS
  screen showed "GPS disabled", and switching it off showed "GPS enabled". The GPS itself switched
  correctly; the message now matches. The 3-tap buzzer and 4-tap GPS shortcuts now show a message
  too, like the LED and advert shortcuts.
- **New board**: Seeed LR2021 LoRa Plus EVK with a XIAO nRF54LM20A. Build it from source; there is no
  published firmware for it yet.
- **Zephyr** updated to `74b7173e9c9`. One change in it would have been visible and is handled:
  e-paper displays would have drawn white text on a black page. The RAK4631's LEDs are now declared
  by ZephCore, since Zephyr's board file no longer describes them; nothing changes on the board.
  It also brings a fix for boards on WiFi: the starting sequence number of a TCP connection (the
  companion port and the update page) was predictable, and is now derived from a secret as intended.
- **Heltec V3 and Wireless Tracker companions** (WiFi and Bluetooth together, no PSRAM) had under
  1 KB of memory left; they now have about 2.5 KB. The Bluetooth controller's stack on ESP32-S3 and
  ESP32-C3 companions was 4 KB and is now 2 KB; it uses about 1.1 KB with WiFi connected. Nothing
  changes in use.
- **Documentation**: the README was reorganised, build and flashing instructions moved to
  `docs/BUILDING.md`, and the example board and porting guide were rewritten for the current board
  layout.
