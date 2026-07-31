# muzi works R1 Neo

A pocket LoRa node built on the **nRF52840 + SX1262** (RAK4630 stamp module on
a custom muzi baseboard), with GNSS, buzzer, battery-backed RTC and a single
user button. No display.

The port is cross-derived from several sources, because none of them agrees
with the others on every pin: Arduino MeshCore's
[`variants/muziworks_r1_neo`](https://github.com/meshcore-dev/MeshCore/pull/2007),
other third-party firmware for the same board, and the vendor's own
documentation. Where they conflict, the reasoning is recorded in the DTS
comments and below.

## Build

```bash
# Companion (BLE — default role)
west build -b muziworks_r1neo zephcore --pristine

# Repeater (USB-CDC CLI, no BLE)
west build -b muziworks_r1neo zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Companion + debug logging
west build -b muziworks_r1neo zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/debug.conf"
```

Flash: drag `build/zephyr/zephyr.uf2` onto the UF2 drive, or `west flash` with
a J-Link. Bootloader is Adafruit UF2 / SoftDevice **s140 v6.1.1**, app @
`0x26000` (`SD_FWID 0x00B6`) — same layout as RAK4631, so the stock bootloader
is reused and no re-flash is needed.

## Hardware

| Function   | Part                 | Bus / notes                                  |
|------------|----------------------|----------------------------------------------|
| MCU        | nRF52840 (RAK4630)   | BLE5, native USB, 1 MB flash / 256 KB RAM    |
| LoRa       | SX1262               | DIO2 keys the RF switch, DIO3 feeds a 1.8 V TCXO |
| GNSS       | unconfirmed (GPS/BDS)| UART0 @ 9600, power gated on P1.01           |
| RTC        | Epson RX8130CE       | I2C0 @ 0x32, battery-backed                  |
| Battery    | ADC AIN7 (P0.31)     | divider ratio unconfirmed — see below        |
| Input      | 1 user button        | P0.26, active-HIGH, external pull-down       |
| Buzzer     | piezo                | P0.03 (PWM0)                                 |
| LEDs       | green / blue         | P1.04 / P0.28, both active-HIGH              |
| Power      | I/O controller + DCDC| latch P0.13, "MCU on" P0.29                  |

No QSPI external flash — contacts and channels live in internal flash.

## Pin map

| Signal          | nRF52840 | Signal              | nRF52840 |
|-----------------|----------|---------------------|----------|
| LoRa SCK        | P1.11    | LoRa CS             | P1.10    |
| LoRa MOSI       | P1.12    | LoRa RST            | P1.06    |
| LoRa MISO       | P1.13    | LoRa BUSY           | P1.14    |
| LoRa DIO1       | P1.15    | SX1262 power enable | P1.05    |
| GPS MCU-TX / RX | P0.24 / P0.25 | GPS enable      | P1.01    |
| GPS PPS         | P0.02 (unused) | RTC SDA / SCL  | P0.19 / P0.20 |
| Battery ADC     | P0.31    | Battery charge stat | P1.02 (unused) |
| User button     | P0.26    | Buzzer              | P0.03    |
| LED green/blue  | P1.04 / P0.28 | DCDC latch     | P0.13    |
| I/O ctrl "MCU on" | P0.29  | MCU_SIGNAL          | P0.30 (unused) |

### Notes worth knowing

- **GPS TX/RX are swapped relative to the net names.** The schematic labels
  `UART_GPS_RX` (P0.24) and `UART_GPS_TX` (P0.25) from the *module's* point of
  view, so the MCU's receive pin is P0.25. Arduino MeshCore takes the labels
  literally and ends up listening on the module's own input pin; ZephCore uses
  the corrected mapping. Getting this backwards is silent — the UART simply
  never sees a byte.
- **Soft power, not a switch.** The rail is gated by an I/O controller that
  only holds the DCDC up while P0.13 is asserted, and P0.29 tells it the MCU
  is alive (this also enables button and LED passthrough). Both are `gpio-hog`
  at boot and released at shutdown through the `poweroff_gpios` node — without
  that release, `sys_poweroff()` would leave the rail latched, because nRF52
  GPIO output latches survive System OFF.
- **Long-press = real power-off.** Hold the button ≥1 s: the buzzer plays the
  shutdown melody, the radio and GNSS are parked, then the latch drops and the
  board is genuinely off (not a ~1 µA System OFF doze). Press again to boot.
- **Multi-tap actions:** 2 taps = LED heartbeat toggle, 3 = buzzer mute toggle,
  4 = GPS on/off, 5 = flood advert. A single tap maps to "page next",
  which is a no-op here since there is no display.
- **NFC pins** (P0.09/P0.10) are freed via `nfct-pins-as-gpios`; they are NC on
  this board. Harmless, and consistent with the other RAK-based boards.

## To verify on real hardware

1. **Battery multiplier.** `vbat-mv-multiplier = 6146` is a placeholder. The
   sources disagree on the divider: Arduino MeshCore's constant implies
   ~1.73 (→ 6259, the RAK4631-family value), the vendor documentation states
   1.667 (→ 6031). Measure the cell with a DMM, compare against `get batt`,
   and set the value with ZephCore's convention:
   `multiplier = divider_ratio * 1.005 * 3600`.
2. **GNSS lock.** Confirm NMEA arrives after the TX/RX correction above. If it
   does not, the next suspect is the module identity — swap
   `compatible = "luatos,air530z"` for `"gnss-nmea-generic"` plus a
   `gps-enable` alias (pattern in `heltec_t114` / `thinknode_m9`).
3. **Power-off current.** After a long-press shutdown, the board should draw
   effectively nothing. If it still drains, the latch release order in
   `poweroff_gpios` may need adjusting.
4. **Power-off with USB attached.** The rail may be held up externally, in
   which case the shutdown falls through to a plain System OFF instead of a
   true power-off. Expected; confirm it still wakes cleanly on button press.
5. **Reboot.** A soft reset floats P0.13 while the SoC restarts, so the I/O
   controller may cut the rail and turn the board off instead of rebooting.
   Upstream MeshCore reports the same class of problem on this board. If it
   reproduces, it is a baseboard behaviour, not something the DTS can fix —
   document it rather than chasing it in firmware.
