# Heltec Mesh Node T096

Custom ZephCore board target for the Heltec Mesh Node T096.

Build:

```bash
west build -b heltec_t096 zephcore --pristine
```

## Hardware

- MCU: nRF52840
- Radio: SX1262
- RF front-end: KCT8103L PA/FEM, official maximum LoRa output power up to 28+/-1 dBm
- GNSS: UC6580
- Display: ST7735S 0.96" TFT, 160x80 pixels

## Confirmed Pinout

Sources: official Heltec Mesh Node T096 datasheet/schematic, with MeshCore `variants/heltec_t096` and Zephyr's Heltec Wireless Tracker ST7735 160x80 node used as implementation references.

| Function | nRF52840 pin | Schematic net | ZephCore mapping |
| --- | --- | --- | --- |
| LoRa SCK | P1.08 | LoRa_SCK | `spi2` SCK |
| LoRa MISO | P0.14 | LoRa_MISO | `spi2` MISO |
| LoRa MOSI | P0.11 | LoRa_MOSI | `spi2` MOSI |
| LoRa NSS / CS | P0.05 | LoRa_NSS | `cs-gpios` |
| LoRa reset | P0.16 | LoRa_RST | `reset-gpios` |
| LoRa busy | P0.19 | LoRa_BUSY | `busy-gpios` |
| LoRa DIO1 | P0.21 | DIO1 | `dio1-gpios` |
| SX1262 DIO2 / FEM CPS | SX1262 DIO2 | PA_CPS | `dio2-tx-enable` |
| FEM enable | P0.12 | PA_CSD | `antenna-enable-gpios` |
| FEM TX select | P1.09 | PA_CTX | `tx-enable-gpios` |
| FEM rail enable | P0.30 | VFEM_Ctrl | fixed regulator `vfem_enable` |
| GNSS UART RX into MCU | P0.23 | GNSS_TX | `uart0` RX |
| GNSS UART TX from MCU | P0.25 | GNSS_RX | `uart0` TX |
| GNSS reset | P1.14 | GNSS_RST | `gps-reset` alias |
| GNSS power | P0.06 | VGNSS_CTRL | fixed regulator `gps_power` |
| GNSS PPS | P1.11 | PPS | documented, not consumed |
| Display SCK | P0.20 | SCLK | `spi3` SCK |
| Display MOSI | P0.17 | SDIN | `spi3` MOSI |
| Display CS | P0.22 | CS | `spi3` `cs-gpios` |
| Display DC | P0.15 | RS | MIPI-DBI `dc-gpios` |
| Display reset | P0.13 | RESET | MIPI-DBI `reset-gpios` |
| Display backlight | P1.12 | LEDA | `disp_pwr_enable`, active-low |
| Display/peripheral rail | P0.26 | Vext_Ctrl | `tft_pwr_enable`, active-high |
| LED | P0.28 | LED | `led0`, `lora-tx-led` |
| Button | P1.10 | Button | `sw0` |
| Battery ADC | P0.03 / AIN1 | ADC_IN | ADC channel 1 |
| Battery ADC enable | P1.15 | ADC_Ctrl | fixed regulator `vbat_enable` |

## Display

- Controller: ST7735S, modeled with Zephyr's `sitronix,st7735r` ST7735R/ST7735S driver.
- Resolution: 160x80 pixels.
- Bus: SPI3 through Zephyr MIPI-DBI, write-only.
- Orientation/init: ST7735 mini 160x80 parameters from Zephyr's Heltec Wireless Tracker node, with `madctl = 0xa8` matching MeshCore's T096 `DISPLAY_ROTATION=1` plus BGR override.
- UI path: `zephcore,mono-tft` wraps the color TFT as a 1bpp display for ZephCore's CFB companion UI.
- Power: `Vext_Ctrl` is boot-enabled so the ST7735 init sequence reaches a powered panel; `LEDA` follows ZephCore display on/off as the backlight control.

## Not Enabled

- External flash: the schematic shows `F_SPI_*` nets and a flash footprint, but the flash device and parameters are not confirmed.

## T096 vs T114 Differences

- LoRa SPI/control pins are different across SCK, MISO, MOSI, CS, reset, busy, and DIO1.
- T096 adds a KCT8103L PA/FEM path with `PA_CSD`, `PA_CTX`, `PA_CPS`, and `VFEM_Ctrl`; T114 uses SX1262 DIO2 RF switch without those MCU PA/FEM GPIOs.
- T096 GNSS is UC6580 at 115200 baud with `VGNSS_CTRL`, `GNSS_RST`, and `PPS`; T114 uses ATGM336H-style wiring at 9600 baud.
- T096 battery ADC is P0.03/AIN1 with enable P1.15; T114 battery ADC is P0.04/AIN2 with enable P0.06.
- T096 display is an ST7735S 160x80 panel on P0.20/P0.17 SPI3 with P0.26 rail enable and P1.12 active-low backlight; T114 display is an ST7789V 240x135 panel with different pins and power polarity.
- T096 LED is P0.28 active-high; T114 LED is P1.03 active-low.
- T096 button is P1.10, matching T114's user button pin.

## Hardware Smoke Test Plan

1. Flash UF2 by double-tap reset and drag-drop.
2. Confirm USB CDC/logging appears.
3. Confirm BLE advertises as ZephCore/MeshCore companion.
4. Confirm MeshCore app pairing works.
5. Confirm the display wakes, shows the normal ZephCore companion status/pairing UI, and blanks/backlights off after the configured timeout.
6. Confirm SX1262 init succeeds with no reset, busy, or DIO1 errors.
7. Confirm RX starts.
8. Test low-power TX first.
9. Confirm PA/FEM control before testing higher TX power.
10. Test PA progressively.
11. Confirm GNSS only after radio and BLE are stable.
