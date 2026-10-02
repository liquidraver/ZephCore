# ZephCore 1.17.7-zephcore

Faster GPS fixes on three trackers, a clock that survives a reboot, and a few fixes ported from
upstream MeshCore.

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

On the SenseCAP Solar, standby uses a little power all the time, so it is only used when the GPS
interval is one hour or less. Longer intervals (a repeater's default is 48 hours) and `gps off` switch
the receiver off completely, as before. The limit can be changed with `set gps standby <seconds>`
(`0` = always switch off, `default` = one hour); `get gps standby` shows it.

## The clock survives a reboot

A reboot no longer sends the clock back to 1970 on boards without a clock chip. The time is restored
at boot and is at most a few seconds behind. A full power loss still resets it until the next GPS fix
or phone sync.

## Also in this release

- **Malformed encrypted packets are rejected earlier**, before any cryptography runs.
- **Large contact lists**: up to 24 contacts can share the same one-byte hash, up from 8.
- **T1000-E**: a pin that was wrongly driven as a sensor enable is left alone. Sensor readings are
  unchanged.
- **New board**: Seeed LR2021 LoRa Plus EVK with a XIAO nRF54LM20A.
- **Zephyr** updated to `ac03a4a9085`.
