# ZephCore 1.18.3-zephcore

A failed-login limit on repeaters and room servers that now really limits, and a memory fix in the
KISS modem.

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
