# ZephCore 1.17.7-zephcore

Faster GPS fixes on three trackers, a clock that survives a reboot, a fix for USB companions that
could stop responding right after boot, more reliable detection of clock chips, repeaters that power
down an external flash chip they do not use, and a few fixes ported from upstream MeshCore.

> [!NOTE]
> A normal upgrade keeps your identity, settings, contacts and phone pairing.

---

## GPS: a fix in seconds instead of minutes

On some boards the GPS receiver lost its satellite data every time it went to sleep, so each wake was
a search from scratch. Three boards now keep that data:

- **SenseCAP T1000-E and MeshTracker X1**: the receiver uses its low-power backup sleep. On a
  T1000-E, a fix after a minute asleep took about 9 seconds instead of about 77.
- **SenseCAP Solar**: the receiver stays in standby between fixes instead of being switched off. In
  marginal reception it got a fix on 5 of 6 wakes, against 2 of 6 before.

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

## USB companion: a hang right after boot

A companion on USB could stop responding when the host sent it a command straight after opening the
port following a boot: nothing more came out of the USB port and the node stopped handling mesh
traffic, while Bluetooth kept advertising. Only a reset recovered it.

The cause is in the USB serial driver of the Zephyr version 1.17.6 moved to: with its first 64-byte
buffer already full, it reported free space but accepted no data, and the firmware retried forever.
On a Wio Tracker L1 debug build, where the boot log fills that buffer, it happened on every such boot.
It was not reproduced on a release build, but the boot banner left only 3 bytes of margin there.

The driver is patched. On the Wio Tracker L1, 6 of 6 debug boots and 3 of 3 release boots now respond,
and the companion protocol test gives the same replies as 1.17.6.

The same patch covers a second way into the same loop: a host computer going to sleep with the port
open. That one is fixed from reading the code and has not been reproduced on hardware.

## Repeaters: the external flash chip is powered down

Some boards carry a second flash chip, which companions keep contacts and channels on. Repeaters,
room servers and observers store nothing on it. Since 1.17.6 those roles no longer set the chip up at
all, and that left its control pins unconnected: on a Wio Tracker L1 repeater the chip-select line
read low, so the chip sat selected for as long as the node ran instead of idling. 1.17.4 and earlier
set the chip up at boot on every role.

These roles now set the chip up once at start-up and put it into its deep power-down mode, with
chip-select held high. A factory `erase` wakes the chip, erases it and puts it back. Companions are
unchanged: they use the chip, so it stays in standby.

Boards with this chip: T-Echo, T-Impulse Plus, MeshTracker X1, SenseCAP Solar, ThinkNode M1,
ThinkNode M6, Wio Tracker L1, Wio Tracker L1 Pro 1W and XIAO nRF52840.

Checked on a Wio Tracker L1 repeater after a normal boot, after a first-boot format and after
`erase`: the chip ignores an ID request, and answers again once it is woken. The current this saves
has not been measured, and the change has not been run on the other boards.

This was found while looking into a report of a XIAO nRF52840 repeater drawing about 4 mA more on
1.17.6 than on 1.17.4 (thanks to **Kimotu**). Whether it accounts for that difference is not
confirmed yet.

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
- **T1000-E**: a pin that was wrongly driven as a sensor enable is left alone. Sensor readings are
  unchanged.
- **LR2021 with `rxduty` on** (MeshTracker X1, both LR2021 EVK kits): after every noise-floor
  measurement the chip rejected the command that puts it back into its duty cycle. The next channel
  check recovered it and no packets were lost; the driver now returns the chip to standby first, and
  the command is accepted.
- **New board**: Seeed LR2021 LoRa Plus EVK with a XIAO nRF54LM20A. Build it from source; there is no
  published firmware for it yet.
- **Zephyr** updated to `74b7173e9c9`. One change in it would have been visible and is handled:
  e-paper displays would have drawn white text on a black page. The RAK4631's LEDs are now declared
  by ZephCore, since Zephyr's board file no longer describes them; nothing changes on the board.
  It also brings a fix for boards on WiFi: the starting sequence number of a TCP connection (the
  companion port and the update page) was predictable, and is now derived from a secret as intended.
- **Heltec V3 and Wireless Tracker companions** (WiFi and Bluetooth together, no PSRAM) had under
  1 KB of memory left; they now have about 2.5 KB. The Bluetooth controller's stack on ESP32-S3 and
  ESP32-C3 companions was 4 KB and is now 2 KB: its measured use is 1.1 KB with WiFi connected.
  Nothing changes in use. Measured on a XIAO ESP32-S3; not run on the two Heltec boards themselves or
  on an ESP32-C3.
- **Documentation**: the README was reorganised, build and flashing instructions moved to
  `docs/BUILDING.md`, and the example board and porting guide were rewritten for the current board
  layout.
