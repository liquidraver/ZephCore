# Supported Boards

Board strings for use with `west build -b <board> zephcore`.

## nRF52840

```
rak4631
rak3401_1watt
wio_tracker_l1
t1000_e
thinknode_m1
thinknode_m3
thinknode_m6
rak_wismesh_tag
ikoka_nano_30dbm
sensecap_solar
xiao_nrf52840
lilygo_techo
promicro_sx1262
heltec_t114
heltec_t096
gat562_30s
```

> **RAK WisMesh Pocket** (WisBlock pocket): use `-b rak4631` — same board string and firmware as **RAK4631**.
>
> **Heltec T114** screenless build: append `boards/nrf52840/heltec_t114/no_display.conf` to `EXTRA_CONF_FILE` for units without the TFT module.
>
> **Heltec Mesh Node T096** (`heltec_t096`): nRF52840 with SX1262 + KCT8103L PA/FEM, UC6580 GNSS, and ST7735S 160x80 TFT companion display. The external SPI flash footprint is documented in the board notes but left disabled until the device parameters are confirmed.

## ESP32

```
xiao_esp32c3
xiao_esp32c6/esp32c6/hpcore
xiao_esp32s3/esp32s3/procpu
lilygo_tlora_c6/esp32c6/hpcore
station_g2/esp32s3/procpu
heltec_wifi_lora32_v3/esp32s3/procpu
heltec_wifi_lora32_v4/esp32s3/procpu
heltec_wifi_lora32_v43/esp32s3/procpu
heltec_wireless_tracker/esp32s3/procpu
ttgo_tbeam/esp32/procpu
```

> ESP32 boards require `west blobs fetch hal_espressif` before first build.
>
> Heltec V3 console/shell use `uart0` (UART serial) in ZephCore.
>
> **Heltec Wireless Tracker** (`heltec_wireless_tracker/esp32s3/procpu`): V1.1
> ESP32-S3-FN8 companion with SX1262, ST7735R 160x80 TFT, and UC6580 GPS.
>
> **LilyGo T-Beam** (`ttgo_tbeam/esp32/procpu`): classic ESP32 (PICO-D4) with
> SX1262, AXP2101 PMU, and GNSS. Use this for the **v1.2 SX1262** variant — the
> upstream Zephyr DTS models the SX1276 radio, which ZephCore overrides to
> SX1262 in `board.overlay`. Console/CLI are on `uart0` (onboard USB-UART).

## MG24 (Silicon Labs)

```
xiao_mg24
```

> Requires `west blobs fetch hal_silabs` and pyocd.

## nRF54L

```
xiao_nrf54l15/nrf54l15/cpuapp
```

> Requires `--no-sysbuild` flag: `west build -b xiao_nrf54l15/nrf54l15/cpuapp zephcore --no-sysbuild`
