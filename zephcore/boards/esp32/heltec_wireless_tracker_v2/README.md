# Heltec Wireless Tracker V2

ZephCore board string:

```bash
west build -b heltec_wireless_tracker_v2/esp32s3/procpu zephcore --pristine --sysbuild
```

Build-verified ZephCore roles:

- Companion: default build
- Repeater: `-DEXTRA_CONF_FILE=boards/common/repeater.conf`
- Room server: `-DEXTRA_CONF_FILE=boards/common/room_server.conf`
- Observer: `-DEXTRA_CONF_FILE=boards/common/observer.conf`
- Repeater uplink: `-DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/repeater_uplink.conf"`

Flash with the ESP32 runner after a sysbuild:

```bash
west flash --build-dir build --esp-device /dev/ttyACM0
```

On macOS, replace the device with the board's `/dev/cu.usbmodem*` path. If the
ROM bootloader does not auto-enter download mode, connect USB-C, hold `USER`,
tap `RST`, then release `USER`.

The equivalent manual sysbuild flash layout is:

```bash
esptool.py --chip esp32s3 --port /dev/ttyACM0 write_flash \
  0x00000 build/mcuboot/zephyr/zephyr.bin \
  0x20000 build/zephcore/zephyr/zephyr.signed.bin
```

## Hardware

| Function | ESP32-S3 GPIO | ZephCore node/property | Source |
| --- | --- | --- | --- |
| LoRa NSS | GPIO8 | `&spi2 cs-gpios` | Heltec schematic, MeshCore |
| LoRa SCK/MOSI/MISO | GPIO9/GPIO10/GPIO11 | `&spi2` pinctrl | Heltec schematic, MeshCore |
| LoRa RST/BUSY/DIO1 | GPIO12/GPIO13/GPIO14 | `reset-gpios`, `busy-gpios`, `dio1-gpios` | Heltec schematic, MeshCore |
| SX1262 DIO2 RF switch | SX1262 DIO2 | `dio2-tx-enable` | MeshCore |
| SX1262 DIO3 TCXO | SX1262 DIO3 | `dio3-tcxo-voltage = 1.8V` | MeshCore |
| VFEM_Ctrl | GPIO7 | `fem_power` regulator | Heltec schematic, MeshCore |
| PA_CSD | GPIO4 | `antenna-enable-gpios` | Heltec schematic, MeshCore |
| PA_CTX | GPIO5 | `tx-enable-gpios` | Heltec schematic, MeshCore |
| PA_CPS | GPIO46 | Unused/unclaimed | Heltec schematic/pin map |
| UC6580 GNSS UART | RX GPIO33, TX GPIO34 | `&uart2`, `gnss-nmea-generic` | Heltec schematic, MeshCore |
| GNSS_RST | GPIO35 | `gps-enable` alias, active-high run | Heltec schematic, MeshCore |
| GNSS PPS | GPIO36 | Unused input | Heltec schematic |
| TFT CS/SCLK/MOSI | GPIO38/GPIO41/GPIO42 | `&spi3` pinctrl | Heltec schematic, MeshCore |
| TFT DC/RST | GPIO40/GPIO39 | MIPI-DBI `dc-gpios`, `reset-gpios` | Heltec schematic, MeshCore |
| TFT backlight | GPIO21 | `disp_pwr_enable` regulator | Heltec schematic, MeshCore |
| Vext rail | GPIO3 | `vext_enable`/`tft_pwr_enable` regulator | Heltec schematic, MeshCore |
| Battery ADC | GPIO1 | `zephyr,user io-channels = <&adc0 0>` | Heltec schematic, MeshCore |
| Battery ADC enable | GPIO2 | `vbat_enable` regulator | Heltec schematic, MeshCore |
| Status/TX LED | GPIO18 | `led0`, `lora-tx-led` | Heltec schematic, MeshCore |
| User button | GPIO0 | `sw0` | Heltec schematic, MeshCore |
| I2C header | SDA GPIO6, SCL GPIO17 | `&i2c0`, sensors auto-detect | MeshCore |

## Notes

- This is a separate first-class board from Zephyr's upstream
  `heltec_wireless_tracker` target, which models the V1.1 board.
- Build validation covers the ESP32-S3FN8 CPU/flash layout, native USB
  serial/JTAG console selection, SX1262 SPI/control devicetree, UC6580 UART
  wiring, status LED, user button, battery ADC node, ST7735R display node, and
  the ZephCore role link paths listed above.
- Hardware has not been exercised in this workspace. Treat LoRa RF switching,
  GNSS runtime behavior, battery calibration, TFT orientation/backlight, USB
  flashing auto-reset, and LED/button polarity as untested until checked on a
  physical Wireless Tracker V2.
- Battery ADC is on ESP32-S3 ADC unit 1 (`&adc0`) channel 0, GPIO1. ADC unit 2
  channel 0 is GPIO11 and must stay disabled because GPIO11 is LoRa MISO.
- MeshCore's V2 target drives `PA_CSD` and `PA_CTX` but does not drive
  `PA_CPS`/GPIO46. ZephCore leaves GPIO46 unclaimed pending hardware validation
  of that KCT8103L control path.
- `GNSS_RST` is exposed through ZephCore's `gps-enable` alias as an active-high
  logical "run" line: high releases reset, low holds UC6580 off.
