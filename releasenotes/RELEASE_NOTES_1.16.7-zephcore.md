# ZephCore 1.16.7-zephcore

> [!IMPORTANT]
> ## Before you upgrade
>
> **From v1.16.6 / v1.16.5** — clean flash, no re-bond, bonds and data survive.
>
> **From v1.16.2 / v1.16.3 / v1.16.4** — clean flash, bonds and data survive. If you are on an
> **ESP32-S3 / ESP32-C board** and have *not* yet taken the v1.16.5 update, you still owe the one-time
> serial reflash: the app moved to flash offset `0x10000` in v1.16.5, and a board on the old layout
> cannot cross that update over WiFi-OTA or the browser flasher — **flash the `-merged.bin` once over
> USB/serial.** Affected boards: *Heltec V3 / V4 / V4.3, Station G2, Wireless Tracker / V2, XIAO
> ESP32-S3 / C3 / C6, LilyGo T-Lora C6.* Identity, contacts, channels, prefs, and BLE bonds are
> preserved. nRF52, classic ESP32 (T-Beam / PICO-D4 / TTGO LoRa32), STM32WL, and native Linux are
> unaffected.
>
> **From v1.16.1 or older** — flash it; on first boot it clears BLE bonds automatically (identity,
> contacts, channels, prefs preserved). Re-bond your phone/desktop once.
>
> **Coming from Arduino MeshCore** — flash it; auto-formats on first boot (new identity, clean storage).
>
> Take this with a grain of salt — try the formatters if anything anomalous happens with your node.

---

A stability-and-hardware release. The headline is a **whole-node wedge fixed** in the GPS power-management
path that shipped in v1.16.6, plus **RX duty cycle finally working on LR1110 boards** and a new safety
margin that stops signal-independent duty-cycle packet drops on every radio. Rounded out by joystick-UI
fixes and a color-display improvement for the Heltec T114.

## Highlights

### Fixed: a GPS node could wedge completely on standby (regression in v1.16.6)

v1.16.6 introduced GPS-UART suspend to save power (an armed nRF UARTE receiver costs ~0.5–1 mA even with
the GPS module powered down). It had a race that could **hang the entire node** — no LoRa, no USB, no BLE,
CLI answering `-> busy`, recoverable only by reboot.

The upstream nRF UARTE suspend path spins forever waiting for an `RXTO` event after issuing `STOPRX`. In
interrupt-driven mode the receiver runs one byte at a time and re-arms from its ISR; if a byte happened to
finish in the narrow window where the driver had already disabled the `ENDRX` interrupt, the receiver was
left stopped-and-un-rearmed, `STOPRX` produced no `RXTO`, and the wait never returned — with the main mesh
thread stuck inside it. Any nRF board that powers its GPS module down and suspends the port on standby
could hit this.

Fixed in two independent layers, so neither alone is load-bearing:

- **`ZephyrGPSManager` settles ~5 ms before suspending** the GPS UART, letting the last in-flight byte
  finish and the RX ISR re-arm, so the receiver is armed-and-idle when `STOPRX` fires — the state that
  reliably yields `RXTO`. Standby happens at most every few minutes, so the cost is negligible.
- **New patch `0010` bounds the driver's `RXTO` wait** to 4 ms instead of spinning forever. On timeout it
  falls through to the unconditional `nrf_uarte_disable()` that force-stops the receiver anyway — so even
  if a byte still lands in the race window, the node can never hang.

### RX duty cycle now works on LR1110 boards

LR1110 boards (T1000-E, ThinkNode M9, and other LR11xx radios) now get real **RX duty-cycle sniff mode**
instead of sitting in continuous receive — a substantial receive-power saving on battery nodes, the same
mechanism SX126x boards have had. This took a significant rework of the LR11xx LoRa driver to land
correctly.

### New: duty-cycle safety margin stops signal-independent packet drops

`set`-able via `CONFIG_ZEPHCORE_LORA_DC_MARGIN_PCT` (default **15%**), shared by every radio
(SX126x / LR11xx / LR20xx).

The theoretical per-cycle "deaf-time" budget that duty cycle is computed from assumes the sleep clock and
the wake transition are exact. They are not: the chip's sleep timer runs on an internal RC oscillator
(RC64k on SX126x, RTC on LR11xx) that drifts several percent over temperature, and the wake-transition
figure is a datasheet number the datasheet itself calls "not accurate and may vary." Either overshoot
silently pushes real deaf time past the budget and drops exactly the fraction of packets whose preamble
phase lands on the window edge — a **strength-independent loss that shows up as random duty-cycle packet
drops**, not as weak-signal loss.

The margin derates the whole budget before the sleep period is programmed, trading a little radio-off time
for robustness. Set it to `0` to restore the old zero-margin behaviour; lower it below 15% once you have
measured your board's clock-plus-transition on air.

### Joystick UI fixes

- **The 2-tap and 5-tap actions were swapped.** The everyday action (LED heartbeat toggle) is now **2 taps**
  and the rarer one (flood advert) is **5 taps** — matching the button-UI mapping and the on-device help.
  Applies to both the joystick UI and single-button boards.
- **Fixed: a node booted with LEDs disabled could never turn its LEDs back on from the UI.** The toggle
  called only the heartbeat helper and left the underlying `leds_disabled` gate at its boot value, so
  `leds_disabled=1` was effectively one-way. It now flips the real gate.
- **The LED toggle now shows an on-screen "LEDs: ON / OFF" confirmation**, so you can tell it registered.

### Heltec T114 color display improvements

From [PR #62](https://github.com/liquidraver/ZephCore/pull/62) by [@Calvario](https://github.com/Calvario):
the button UI now **respects `CONFIG_ZEPHCORE_DISPLAY_LARGE_FONT`** on color panels, and the gray used for
secondary text was retuned (`0x8410` → `0xEF7D`) so it actually reads as a visible gray on the T114's TFT,
which crushes darker grays to invisibility. Thanks to Steve Calvário.

## Other changes

- **west manifest bumped** (routine pinned-tree advance).

## Recommended upgrade checklist

1. **Running v1.16.6 on a GPS-equipped nRF node?** Update — this release fixes the standby wedge described
   above. Bonds and data survive.
2. **ESP32-S3 / ESP32-C boards that skipped v1.16.5:** flash `-merged.bin` once over USB/serial. Data survives.
3. **Everything else, from v1.16.2 onward:** just flash — bonds and data survive.
4. **From v1.16.1 or older:** just flash — self-migrates on first boot; re-bond once.
5. **From Official MeshCore:** just flash — auto-formats on first boot.
6. **Anything odd?** Format first.
