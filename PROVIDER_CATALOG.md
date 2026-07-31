# Mesh America configurator — provider catalog

ZephCore firmware is published to the [Mesh America Device Configurator](https://apps.meshamerica.com)
as a third-party provider catalog. Users pick a board in the app and flash ZephCore
directly from the browser.

## What to send Mesh America

```
Provider name: ZephCore
Catalog URL:   https://raw.githubusercontent.com/liquidraver/ZephCore/firmware-dist/catalog.json
```

(Send this once, after the first `master` build has run and created the `firmware-dist`
branch. Validate first at <https://apps.meshamerica.com/validate>.)

## How it works

`gen_provider_catalog.py` turns a release's firmware assets into `catalog.json`
(the provider-spec schema: `version -> {notes, files:[{type,name,url}]}` with
absolute HTTPS URLs). The `build firmware` workflow runs it in the
`publish-catalog` job and force-pushes the firmware + catalog to the
`firmware-dist` orphan branch on every `master` build.

**Hosting / CORS.** The configurator runs in the browser and `fetch()`es both the
catalog and each firmware binary, so every response needs
`Access-Control-Allow-Origin: *`. GitHub **Release assets do not** send it (they
redirect to `release-assets.githubusercontent.com`, which omits the header), so we
cannot serve firmware from the normal release. Instead:

| Artifact | Host | Why |
|----------|------|-----|
| `catalog.json` | `raw.githubusercontent.com/.../firmware-dist/catalog.json` | CORS `*`, always fresh (raw doesn't cache branch refs) |
| firmware binaries | `cdn.jsdelivr.net/gh/liquidraver/ZephCore@firmware-dist/<file>` | CORS `*`, CDN-backed; filenames carry the git hash so they're effectively immutable |

The normal timestamped GitHub Release (with all the same binaries) is still created
for manual downloads and OTA — the configurator pipeline is additive.

## Device mapping

Boards whose `name` matches an official MeshCore device **fold** into that device
(our firmware shows as extra options under the existing tile). Boards MeshCore
doesn't have become **new** ZephCore-badged tiles. The mapping table lives in
`BOARDS` in `gen_provider_catalog.py`; adjust it there when boards are added or
renamed. Names must match MeshCore's canonical `config.json`
(`meshcore-dev/flasher.meshcore.io`) **exactly** to fold — the validator flags
mismatches.

Notes on the mapping:
- **ESP32 offers both `flash-wipe` and `flash-update`.** `flash-wipe` = the `-merged.bin`
  (MCUboot + app) written at `0x0`. `flash-update` = the `-update.bin` (MCUboot-signed app)
  written at `0x10000`. As of 2026-07-09 ZephCore's ESP32 app slot is at `0x10000` (see
  `boards/esp32/<board>/partitions.overlay`), matching Mesh America's fixed flash-update
  offset — so app-only updates that keep settings now work. `build.sh` ships `-update.bin`
  for both ESP32 companion and repeater roles. **Classic-ESP32 (T-Beam)** uses simple boot
  (no MCUboot), produces no `-update.bin`, and is naturally flash-wipe-only.
- **Companion firmware** is one image serving BLE + USB; it's listed under both
  `companionBle` and `companionUsb` (same file). Native-Linux companions use a
  custom `companionTcp` role.
- **Heltec T114** lists screen and "No screen" variants (subTitle) under one tile.
- **Native-Linux** boards (femtofox, rak6421) are `noflash` download-only tiles.
- **Thumbnails:** folded devices inherit MeshCore's image automatically (device-level
  fields come from the official entry on merge), so only the new tiles set a `tooltip`.
  Those reuse the configurator's own image set via jsDelivr (`IMG_BASE` in the script);
  boards without dedicated art fall back to a neutral LoRa icon. Swap in ZephCore-branded
  art by pointing `img` at your own absolute HTTPS URLs.

- **Erase / "wipe settings" (nRF52) uses ZephCore's own formatter.** MeshCore's official erase
  targets a different flash layout and only *partially* wipes a ZephCore node (observed on a Wio
  Tracker L1: nuked `channels`, left identity/prefs/contacts, LittleFS half-corrupted). Since spec
  §4a we override it: each nRF52 firmware option carries an `erase` pointing at ZephCore's
  formatter. The formatter is **SoftDevice-specific** (v6 and v7 have different partition maps), so
  `SOFTDEVICE` in the generator maps every nRF52 board to `SoftDevice_v6_formatter.zip` or
  `SoftDevice_v7_formatter.zip`; `build.sh` publishes both from `formatter/` under stable, un-hashed
  names so the erase URLs never change. The `.zip` drives the automated DFU erase flow; the `.uf2`
  is published too as the manual drag-and-drop fallback.
  - `erase` goes on the **firmware option**, never the device object — a device-level `erase` is an
    official-only field that providers get silently ignored + warned for (§4a).
  - ESP32 has no `erase` (the field is nRF52-only); `flash-wipe` already reinstalls cleanly.
  - Users can still factory-reset on-device without the flasher: the `erase` command over the USB
    serial CLI, the app's factory reset, or a reflash (ZephCore auto-formats on first boot when it
    detects an incompatible/blank FS). All format `/lfs` + `/ext` + the NVS bond partition.

## Local test

```
python gen_provider_catalog.py --firmware-dir firmware \
  --url-base "https://cdn.jsdelivr.net/gh/liquidraver/ZephCore@firmware-dist" \
  --version v0.0.0 --out catalog.json
```

It validates the output against the spec's hard requirements and prints a
fold/new summary; a non-zero exit means a schema error.
