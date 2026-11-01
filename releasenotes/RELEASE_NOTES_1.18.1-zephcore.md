# ZephCore 1.18.1-zephcore

Half-hour and 45-minute timezones on the on-screen clock, the clock zone on repeater screens, and a
Zephyr update.

> [!NOTE]
> A normal upgrade keeps your identity, settings, contacts and phone pairing.

---

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
- **Zephyr** updated to `b70dbaf045e`. A GPS receiver that fails to start on the first try is now
  really tried again (the two retries did nothing before), and on boards with external flash a
  failed start of that flash is retried at the next mount instead of staying failed until reboot.
