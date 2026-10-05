# ZephCore 1.17.7-zephcore

Faster GPS fixes on three trackers, a clock that survives a reboot, a fix for USB companions that
could stop responding right after boot, more reliable detection of clock chips, and a few fixes ported
from upstream MeshCore.

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

## Also in this release

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
- **Zephyr** updated to `ac03a4a9085`. One change in it would have been visible and is handled:
  e-paper displays would have drawn white text on a black page. The RAK4631's LEDs are now declared
  by ZephCore, since Zephyr's board file no longer describes them; nothing changes on the board.
- **Documentation**: the README was reorganised, build and flashing instructions moved to
  `docs/BUILDING.md`, and the example board and porting guide were rewritten for the current board
  layout.
