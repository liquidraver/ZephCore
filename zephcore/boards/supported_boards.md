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
```

> **RAK WisMesh Pocket** (WisBlock pocket): use `-b rak4631` — same board string and firmware as **RAK4631**.

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
```

> ESP32 boards require `west blobs fetch hal_espressif` before first build.
>
> Heltec V3 console/shell use `uart0` (UART serial) in ZephCore.

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
