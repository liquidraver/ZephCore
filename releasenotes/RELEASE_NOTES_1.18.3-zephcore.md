# ZephCore 1.18.3-zephcore

A failed-login limit on repeaters and room servers that now really limits, fewer collisions between
neighbours after a busy channel, and a memory fix in the KISS modem.

> [!NOTE]
> A normal upgrade keeps your identity, settings, contacts and phone pairing.

---

## The failed-login limit now stops password guessing

Repeaters and room servers count wrong passwords: four in three minutes, from anyone. Until now
that count only chose a log line. The right password was still accepted at any rate, so a
stranger in radio range could keep guessing as fast as the airtime allowed.

- **After four wrong passwords in three minutes, no password is checked**, the right one included,
  until the three minutes are over. This applies to the repeater and the room server, admin and
  guest passwords alike.
- **Clients already on the access list keep logging in without a password** during that time, as
  the app does when you open a repeater you have used before.
- Anyone in range can now hold password logins closed by sending four wrong ones every three
  minutes. If that happens, log in from a client that is already on the access list, or over USB.

## Fewer collisions after a busy channel

Before a node transmits it listens, and if it hears the channel in use it waits and tries again.
Two things made neighbours that waited out the same packet collide with each other afterwards:

- **The wait only had three possible lengths** (120, 240 or 360 ms), so two waiting nodes picked
  the same one about a third of the time. Both then listened at the same moment, both heard
  nothing, and both transmitted together.
- **A node that was receiving sent the moment that packet ended**, without waiting at all. Every
  node in range that had something ready did the same.

Now the wait is picked from ten short steps, each one listen long (about 12 ms on the default
radio settings), and it is counted from the moment the packet the node was hearing ends, not from
when it first noticed the channel busy or finished handling that packet. Two waiting neighbours
pick the same step about once in ten (on a crowded hilltop the node uses more steps, see below); on a busy site, where most gaps between repeated packets
close within a tenth of a second, a longer spread meant a node rarely got a turn at all. The spread stays the same after a busy
try, so a node that lost one race is not pushed to the back of the next one. During a burst of repeated packets a node now uses the short gaps between
them, where it used to sleep through them: on a busy roof repeater the longest wait to transmit
was over 10 seconds.

A repeater no longer pushes its own forward back each time it hears a neighbour repeat the same
packet. Waiting for each packet to end already makes it give way, and the extra delay made no
measurable difference on a busy roof repeater over a night of traffic. `get backoff.multiplier`
and `set backoff.multiplier`, which tuned that delay, are removed. A saved value is ignored.

**Repeaters forward sooner.** A repeater used to hold every packet it forwards for a random time
of up to about 2 seconds before even listening. Most of the time the channel was busy by then
anyway, so the hold only added delay. Now a forward waits the same short random step as a retry,
counted from the end of the packet that brought it. On a busy roof repeater this cut the typical
delay per hop from 2.6 to 2.1 seconds, with no more collisions and no lost packets. The number
of steps grows with how many neighbours repeat the same packets, from 10 to at most 64, so a
crowded hilltop spreads its neighbours wider. `get txdelay` now shows that estimate and the
number of steps.

Nothing changes in how a busy channel is treated: a node never transmits over traffic it can hear.
The radio restart that used to fire after 4 seconds of receiving now waits for 10 seconds with no
packet completing, so it no longer cuts off receptions in a busy burst; it remains as a safety net
for a stuck radio.

New: **`get lbt.stats`** shows how often listen-before-talk found the channel clear or busy, how
often a busy reading was followed by an actual reception, and the longest time the node has been
held back. `clear stats` resets it. See `docs/CLI_commands.md`.

For digging deeper there is a diagnostic build option, `lbt_logging.conf` (added after
`packet_logging.conf`): it prints one `LBT ...` line for every transmit attempt, with the outcome,
the signal strength right after a busy reading, the wait chosen and how long the packet has
waited since it was queued. On SX126x radios a busy
line also says whether the receiver had just heard the start of a packet that turned out not to be
MeshCore's, which helps tell other LoRa traffic on the channel apart from the mesh. Tools that read the packet
lines, such as mctomqtt, ignore these lines. `ver` shows `+lbtlog` on such a build.

## Steadier noise floor

For a few milliseconds after the receiver starts listening, the radio reports no signal at all and then
climbs to the real level. The node starts listening again many times a minute without the noise-floor
measurement knowing, so now and then that measurement caught the dip and pulled the noise floor
down by about 3 dB for a couple of minutes.

- A reading that would be the first, or that lies 6 dB or more below the current noise floor, is now
  taken again 10 ms later, and the second reading counts.
- A reading below what physics allows for the bandwidth is thrown away instead of being counted
  as the lowest possible value.
- Right after a restart, the first few readings now count equally, so one bad first reading no
  longer takes minutes to wash out.

`get cad.stats` on the USB console shows how often each happened (`c` and `l`).

## Adaptive CAD no longer mistakes undecodable packets for false alarms

The node tunes its listen-before-talk sensitivity by testing it in quiet moments: if the test says
"busy" and no packet follows, it counts a false alarm, and enough false alarms make it less
sensitive. A packet the node can hear but not decode, too weak, or with a damaged header, also
counted as a false alarm, which nudged the node toward ignoring real traffic. Now the radio notes
whether a packet start (preamble) or header showed up after the test; if so it counts as real
traffic. `get cad.stats` shows these as a new `u` column next to `t`.

## Other changes

- **KISS modem: the encrypt command no longer overwrites memory.** A host that sent the
  `EncryptData` command with a plaintext longer than 192 bytes wrote past a buffer in the modem's
  memory. Plaintexts over 184 bytes, the most a packet can carry, are now refused with
  `EncryptFailed`. Only the computer on the modem's serial port could send it.
- **KISS modem: protocol version 2.** A host can now ask which radio controls the modem has
  (`GetCapabilities`), switch the radio's boosted receive gain (`SetRxBoostedGain`) and, on boards
  with a switchable LNA in the front end, turn that LNA on or off (`SetFemState`). These follow
  MeshCore's KISS protocol 2. ZephCore does not take MeshCore's timed AGC reset: the modem keeps
  resetting a stuck receiver only after a long silence, as every ZephCore role does, so the AGC
  interval commands answer `Unsupported`. None of these settings is saved across a restart.
- **SenseCAP Solar: dimmer heartbeat LED.** The white LED's heartbeat blink can now be dimmed with
  `set leds.brightness <0-100>`, and a newly set-up node starts at 10%, as in MeshCore. A node you
  already use keeps its current brightness (100%); `set leds.brightness 10` gives the new default.
