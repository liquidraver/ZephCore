# LilyGo T-Impulse Plus

A LoRa wristband/tracker built on the **nRF52840 + SX1262**. ZephCore is the
first mesh firmware ported to it — there is no upstream Meshtastic or MeshCore
variant. The port is derived from LilyGo's official
[T-Impulse-Plus repo](https://github.com/Xinyuan-LilyGO/T-Impulse-Plus)
(`pin_config.h`, `variant.h`, RadioLib examples and the shipped datasheets).

## Build

```bash
# Companion (BLE — default role, recommended for the wristband form factor)
west build -b lilygo_timpulse_plus zephcore --pristine

# Repeater (USB-CDC CLI, no BLE)
west build -b lilygo_timpulse_plus zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Companion + debug logging
west build -b lilygo_timpulse_plus zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/debug.conf"
```

Flash: drag `build/zephyr/zephyr.uf2` onto the UF2 drive (double-tap the BOOT
button to enter the Adafruit bootloader), or `west flash` with a J-Link.
Bootloader is Adafruit UF2 / SoftDevice **s140 v6.1.1**, app @ `0x26000`
(`SD_FWID 0x00B6`) — the stock bootloader is reused, no re-flash needed.

## Hardware

| Function        | Part               | Bus / notes                                   |
|-----------------|--------------------|-----------------------------------------------|
| MCU             | nRF52840           | BLE5, native USB, 1 MB flash / 256 KB RAM     |
| LoRa            | SX1262 (AcSip S62F)| 22 dBm, embedded 32 MHz TCXO on DIO3          |
| Display         | SSD1315 64x32 OLED | I2C0 @ 0x3C (64x32 glass at RAM col 32/row 32) |
| GNSS            | u-blox MIA-M10Q    | UART0 @ 38400, enable P1.10                    |
| Ext flash       | ZD25WQ32C 32 Mbit  | QSPI -> `/ext` LittleFS (contacts/channels)   |
| IMU             | ICM-20948          | 2nd I2C bus @ 0x69 — present, not used         |
| Charger         | SGM41562           | 2nd I2C bus @ 0x03 — present, not used         |
| Battery         | ADC AIN3 (P0.05)   | divider enable P0.25 (HIGH to read)           |
| Input           | TTP223 touch       | P1.04, active-HIGH                             |
| Haptic          | vibration motor    | P0.22 (PWM0) — exposed as the buzzer output   |
| Peripheral rail | RT9080 LDO         | enable P0.14 (gates LoRa/OLED/GPS/QSPI)        |

## Pin map (from LilyGo `pin_config.h`)

| Signal            | nRF52840 | Signal              | nRF52840 |
|-------------------|----------|---------------------|----------|
| LoRa SCK          | P0.03    | LoRa CS             | P1.14    |
| LoRa MOSI         | P0.28    | LoRa RST            | P0.02    |
| LoRa MISO         | P0.30    | LoRa BUSY           | P0.31    |
| LoRa DIO1         | P0.29    | RF TX enable (VC1)  | P1.13    |
| RF RX enable (VC2)| P1.07    | OLED SDA / SCL      | P0.20 / P0.15 |
| GPS MCU-TX / RX   | P1.11 / P1.12 | GPS enable      | P1.10    |
| QSPI SCK / CS     | P0.04 / P0.12 | QSPI IO0/1/2/3  | P0.06 / P1.09 / P0.08 / P0.26 |
| Battery ADC / en  | P0.05 / P0.25 | RT9080 rail en  | P0.14    |
| Touch button      | P1.04    | Haptic motor        | P0.22    |
| LED (blue)        | P0.17    | BOOT (DFU only)     | P0.24    |

### Notes worth knowing
- **RF switch is GPIO, not DIO2.** The S62F uses an external DPDT antenna
  switch on VC1/VC2. Wired via the patched native sx126x driver's
  `tx-enable-gpios` (VC1) / `rx-enable-gpios` (VC2). DIO2 (P1.15) is unused.
- **GPS RX/TX are swapped relative to the net names.** LilyGo's `GPS_UART_RX`/
  `GPS_UART_TX` are labelled from the GPS module's side; the MCU receive pin is
  `GPS_UART_TX` (P1.12) and the MCU transmit pin is `GPS_UART_RX` (P1.11).
- **Single-button UI** (like T-Echo): short tap = next page, double tap =
  previous page, long press (>=1 s) = ENTER. Confirmation needs two ENTERs, so
  `UI_CONFIRM_WINDOW_MS` is widened to 3 s.
- **The ICM-20948 IMU and SGM41562 charger** sit on a second I2C bus
  (SDA P1.08 / SCL P0.11) that ZephCore does not use — that bus is left
  disabled. Battery state comes from the ADC, not the charger IC.

## To verify on real hardware

These were derived from the vendor sources and build clean, but a physical
unit should confirm:

1. **TCXO voltage** — set to 1.8 V. LilyGo's RadioLib example validated the
   1.6 V default; drop `dio3-tcxo-voltage` to `SX126X_DIO3_TCXO_1V6` if the
   radio fails to start.
2. **Battery divider ratio** — `vbat-mv-multiplier` assumes 2:1 (7236). Adjust
   if the reported voltage is off.
3. **OLED orientation** — standard 0.96" landscape config; flip
   `segment-remap`/`com-invdir` in the overlay if the panel is mounted rotated.
4. **GPS baud** — 38400 (from the vendor example); adjust `current-speed` if
   the M10 was left at another rate.

The QSPI flash needs no verification: the ZD25WQ32C JEDEC id (`ba 60 16`) and
deep-power-down timings are datasheet-confirmed, and `/ext` reuses the same
`nordic,qspi-nor` + LittleFS automount path as the older MX25R boards — a blank
chip auto-formats on first mount, no formatter step required.

## Display / UI status (known incomplete)

The panel is a **64x32** glass mapped into the SSD1315's 128x64 RAM at column
32 / row 32 (LilyGo `Display.ino` inits `Adafruit_SSD1306(128,64)` then
`setOffsetCursor(32,32)`). The devicetree now expresses this — `width=64`,
`height=32`, `segment-offset=32`, `page-offset=4` — but two things remain:

1. **Offsets are inferred, not bench-verified.** If the visible window is
   misplaced, try the alternate `multiplex-ratio=31` + `display-offset=32` +
   `page-offset=0`.
2. **Tiny UI (implemented, needs bench polish).** `helpers/ui-button/ui_pages.c`
   now has a runtime "tiny mode" (auto-enabled when the panel is <48 px tall):
   it drops the top bar + page-dots + separator, uses the full height for
   content with tightened line spacing, and flashes the page title for ~1 s on
   each change (there is no persistent bar — matches the "no bar, full content"
   choice). Compact layouts are provided for Messages, Radio, GPS and Status;
   the remaining pages (Recent, Traffic, Sensors, toggles) render through the
   normal path at full height and may still clip horizontally until they get
   their own compact branch. 128x64 boards are unaffected. Pixel positioning
   was written from the datasheet, not a bench — expect to nudge it on hardware.

Separately, early hardware testing showed **occasional spontaneous reboots** —
untriaged, likely power/brownout on TX or a fault loop; capture an RTT/USB
console log from a debug build to get a backtrace.
