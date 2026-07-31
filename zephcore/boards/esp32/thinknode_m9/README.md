# Elecrow ThinkNode M9

ESP32-S3 handheld with LR1110 radio, ST7789 320x240 TFT, CC1167Q GPS, PCF8563
RTC, buzzer, STC8H I2C keypad, QMI8658 IMU + QMC6309 magnetometer, and an SD
slot (IMU/SD not ported — see below).

Ported 2026-07-18 from Arduino MeshCore `variants/thinknode_m9` (which itself
landed upstream only days earlier), cross-checked against a second,
independently developed firmware for this board and against the Elecrow V1.0
schematic. **Not yet validated on hardware.**

## Build

```bash
# Companion
west build -b thinknode_m9/esp32s3/procpu zephcore --pristine
west flash --esp-device COMX

# Repeater
west build -b thinknode_m9/esp32s3/procpu zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf"
```

Plain builds are ESP Simple Boot (self-contained `zephyr.bin`); WiFi-OTA and
release builds use `--sysbuild` + MCUboot like the other S3 boards (see
top-level CLAUDE.md).

## LR1110 radio firmware updater

First ESP32 target for `tools/lr1110_updater`:

```bash
west build -b thinknode_m9/esp32s3/procpu zephcore/tools/lr1110_updater --pristine
west flash --esp-device COMX
```

Console output arrives on the native USB Serial/JTAG port. After the updater
reports success, flash the main firmware over the same port (`west flash` —
no DFU mode needed; esptool resets the chip itself). Identity/prefs/contacts
in high flash are untouched by the updater flash.

## Verified against Arduino MeshCore (2026-07-18 desk check)

- **PSRAM**: upstream `boards/thinknode_m9.json` says `memory_type: qio_opi`,
  16 MB flash → WROOM-1-**N16R8** + `CONFIG_SPIRAM_MODE_OCT=y` confirmed.
- **MADCTL**: Arduino ends at **0xA0** (`landscapeScreen()` 0x60, then
  `DISPLAY_FLIP_VERTICALLY` overwrites with MV|MY) — DTS uses 0xA0.
- **Panel inversion**: Arduino init sends INVON; Zephyr st7789v inverts by
  default (no `inversion-off` prop set) — matched.
- **GPS UART direction**: `Serial1.setPins(PIN_GPS_TX=2, PIN_GPS_RX=3)` and
  arduino-esp32 `setPins(rx, tx)` → MCU RX=GPIO2, TX=GPIO3 (the apparent
  ini-vs-variant.h swap is only naming perspective; both agree).
- **GPS EN/RESET behavior**: `MicroNMEALocationProvider` drives EN LOW-to-run
  and holds RESET HIGH when stopped, 10 ms release pulse on start — exactly
  what `ZephyrGPSManager`'s sequence does with the `gps-enable` (active-LOW)
  and `gps-reset` (active-HIGH) aliases here.
- **RF switch at TX**: our LR11xx driver always selects the HP PA, so the chip
  applies `rfswitch-tx-hp` (0x02 = DIO5 L, DIO6 H) — same as Arduino's
  `MODE_TX_HP` at `LORA_TX_POWER=22`.
- **Battery**: MeshCore's ESP32Board hardcodes `2 * analogReadMilliVolts()` —
  the 2:1 divider assumption (`vbat-mv-multiplier = 8800`) matches Arduino.
- **Upstream quirk not copied**: `ThinkNodeM9Board::getIRQGpio()` returns
  `LORA_DIO0`=41, which is the **BUSY** pin; the RadioLib `Module(...)` ctor
  uses DIO1=42 as IRQ. We follow the working wiring (dio1-gpios = 42) —
  the second implementation names them correctly (IRQ = DIO1 = 42).

## Verified against a second independent firmware (second desk check)

- **GPS module is NOT an L76K** — MeshCore's `GPS_L76K` define is another
  copy-paste artifact. The module identifies as **CC1167Q** to the *Unicore*
  (`$PDTINFO`) probe family, which also explains the 115200 default baud
  (a CASIC L76K would default to 9600). The DTS therefore uses
  `gnss-nmea-generic` (plain NMEA, no vendor commands) rather than the CASIC
  air530z driver.
- **GPS baud 115200** — both implementations agree; resolved.
- **GPS STANDBY (GPIO10)** — driven by *neither* implementation; left
  completely untouched, matching both.
- **Battery** — the second implementation uses a 2.0 multiplier on ADC unit
  2 channel 2 → triple-confirms GPIO13 = ADC unit 2 ch 2 at 2:1. Its
  measured OCV table is imported as [battery_curve.c](battery_curve.c).
- **No user button** — no button GPIO in either implementation (input is
  the keyboard; canned-messages UI). No `sw0` is correct, not a gap.
- **No LED** — no LED pin in either implementation; MeshCore's `PIN_LED 13`
  (colliding with `PIN_VBAT_READ 13`) confirmed a bug.
- **RF switch table** — both implementations carry the identical DIO5/DIO6
  truth table; our DTS bitmasks match both.
- **SD card CS (GPIO48)** — the second implementation parks all three CS
  pins HIGH before radio init because the slot shares SPI2. Our base DTS
  does the same with a `sd-cs-park` gpio-hog (also covers the standalone
  LR1110 updater).
- **Charge/power sense** — GPIO1 = external-power detect, **active LOW**;
  GPIO8 = charger DONE. Not consumed by ZephCore yet.
- **Keypad PORTED** — STC8H helper MCU at **I2C 0x6C** on i2c1 (SDA=20
  SCL=21), `KB_INT` GPIO12 idle-low/rising-edge, backlight GPIO46.
  Registers: 0x01-0x04 battery mV little-endian, 0x05 pressed key, 0x06
  state (write 0x01 = sleep). Driver: `helpers/input/stc8h_keypad.c`,
  binding `zephcore,stc8h-keypad`. This is the board's ONLY input, so
  `Kconfig.thinknode_m9` selects `ZEPHCORE_UI_KEYBOARD` -> the full
  companion UI. **Arrow-key codes are still unknown** — no reference
  documents them; the driver logs unrecognised codes at INFO so they can be
  identified on hardware and added to `key_map[]`. Mapped so far: 0x0D
  enter, 0x86 back, 0x82 home, 0x83 menu, 0x81/0x85 function keys ->
  page prev/next, 0x87 and 0xA3 long-press variants. Printable ASCII
  (0x20-0x7E) passes through untranslated.
- **Also on the sensor bus** — QMI8658 IMU + QMC6309 magnetometer.
  Unported; no address collision with the PCF8563 @0x51.

## Verified against the Elecrow V1.0 schematic (third desk check)

Net-level facts from `Think Node M9_V1.0.sch` (2026-06-23). The text layer
gives nets and part numbers, not GPIO pairings — pin numbers stay
firmware-sourced (both firmwares agree on all of them).

- **VEXT (GPIO18) scope resolved** — the net is `VDD_PERIPH_EN`, gating a
  rail that feeds the display (`DISPLAY_VDD`), the RTC's main supply
  (`RTC_3V3`; the button cell `BUTTON-CELL`/BAT2 only backs up timekeeping)
  and the I2C sensors. `tft_pwr_enable` therefore has `regulator-boot-on`
  so the rail is up before RTC autodiscover / sensor scan, matching
  Arduino's unconditional `board.begin()` write.
- **GPS rails confirmed** — the module has its own switched supply
  (`GPS_EN` → `GPS_VDD`), so screenless/VEXT-off states never affect it;
  its `ON_OFF` and `N_RESET` pins are wired (`GPS_ON/OFF`, `GPS_RST` nets),
  and a `GPS_BAT`/`BACKUP_POWER` rail keeps ephemeris alive for warm starts.
  Module symbol is generic ("GNSS MOD", 18-pin, TXD/RXD/1PPS/ON_OFF/VBAT/
  SDA/SCL) — no part number in the schematic; CC1167Q identification stands
  on the `$PDTINFO` probe response.
- **USB is a UART bridge** — USB-C data goes through a CP2102/CH9102-class
  bridge (U3, DTR/RTS auto-program transistors Q12/Q13) onto `RXD0_H`/
  `TXD0_H` = UART0. Our console-on-uart0 routing therefore lands exactly on
  the USB-C port. Whether the S3's native D+/D− (GPIO19/20) also reach the
  connector is unclear — MeshCore claims `ST7789_TE`=19, which would rule it
  out — so the USB-OTG companion transport may be dead hardware on this
  board; BLE is the primary companion path (verify OTG enumeration once,
  drop the include if dead).
- **Power button is a hardware latch** — `POWER_EN` latch nets + an
  `ESP32_WAKEUP` line; no firmware-readable user button at schematic level
  either. Confirms the no-`sw0` model for the third time.
- **RF switch is a discrete SP3T** (U8: V1/V2 control, J1/J2/J3 ports) driven
  by `RFSW0_V1`/`RFSW1_V2` = LR1110 DIO5/DIO6 — the 2-line table confirmed
  at net level; DIO1 is the `INTERRUPT` net; `VTCXO` net feeds the TCXO.
- **Charger is an LGS4056HDA-4.35** — the suffix is the 4.35 V-termination
  variant, so the pack may charge above the 4200 mV top of the imported OCV
  curve (100% would then read early). Check the charged-battery voltage at
  bring-up and stretch the curve if it really terminates at 4.35 V.
- **Keyboard matrix is driven by a second MCU** (nets `ESP32-2_*` — the
  schematic symbol is another ESP32-S3 template; whatever ships speaks the
  STC8H protocol at I2C 0x6C). SD card has its own switched `SD_3V3` rail.

## Bring-up verification list

Still unverified on hardware, in rough priority order:

1. **Keypad arrow codes** — the one blocker for a usable UI. Attach a serial
   console, press each arrow key, and read the `unmapped keypad code 0x..`
   lines; add them to `key_map[]` in `helpers/input/stc8h_keypad.c` as
   `INPUT_KEY_UP/DOWN/LEFT/RIGHT`. Until then the UI changes pages but
   cannot move within a list. Confirm `keypad at 0x6C ready` appears at boot.
2. **Display init params** — gamma/porch/vcom inherited from Heltec T114's
   ST7789V (no board-specific init values exist in any reference); check
   contrast/colors. If mirrored or upside down, swap `mdac` 0xA0 ↔ 0x60.
3. **GPS identity/output** — confirm NMEA at 115200 and that the CC1167Q
   talks standard sentences (it should; both upstreams parse plain NMEA).
   If a fix never arrives, probe `$PDTINFO` manually.
4. **Battery calibration** — the 2:1 divider is what both upstreams assume;
   sanity-check `get bat` against a multimeter once, and check whether the
   4.35 V charger variant actually charges above 4.20 V (OCV curve top).
5. **USB-OTG companion transport** — see the schematic section above:
   verify whether the native USB pads reach the USB-C connector at all.
6. **rx-boosted** — deliberately ON (both upstreams leave boosted gain off /
   unset; every other ZephCore LR1110 board uses it). Verify RX sensitivity
   and TX power on air.
7. **ADC2 vs WiFi** — battery reads ride ADC unit 2 (GPIO13), which WiFi also
   uses; expect intermittent read failures in `wifi_ota.conf` builds.

## Not ported

- **STC8H I2C keyboard** (@0x6C, second bus SDA=20 SCL=21, INT=12,
  backlight=46) — Arduino MeshCore doesn't consume it either; the register
  map above is the protocol reference when ZephCore grows keyboard support.
- **QMI8658 IMU / QMC6309 magnetometer** — no ZephCore use case yet.
- **SD card** (CS=48 on the shared SPI) — unused, CS parked HIGH via gpio-hog.
- **Ext-power detect** (GPIO1, active LOW) / **charge DONE** (GPIO8),
  **GPS PPS** (GPIO4), **GPS STANDBY** (GPIO10).
