# ADR 0005 — Track Zephyr `main`, pinned to a commit

Status: accepted (recorded 2026-09-25)

## Context

The Zephyr tree is pinned to a specific commit on **`main`**, not on a release tag. It tracked `v4.4-branch` from
2026-06 until the 2026-09-19 jump back to main. That jump needed a few non-obvious changes; they should not be
undone:

- LoRa API 0.9.0: `lora_recv_duty_cycle_async`, const `lora_modem_config`.
- `uart_irq_update()` returns void. The UART ISRs loop
  `for(;;){ uart_irq_update(); if (uart_irq_is_pending() <= 0) break; … }`.
- Ring buffers use `ring_buf_get_ptr`/`ring_buf_consume`; `CONFIG_RING_BUFFER` now means the legacy API and is not set.
- Work queues are named through `k_work_queue_config.name`.
- Every flash partition we define carries `compatible = "zephyr,mapped-partition"`.
- `CONFIG_PSA_CRYPTO=y` is set for BT-off roles.
- `sysbuild.conf` sets `SB_CONFIG_BOOT_SIGNATURE_TYPE_NONE`, because the RSA default makes MCUboot overflow its
  64 KB slot without any build error.
- `wio_tracker_l1` is **upstream's** in-tree board (`boards/seeed/wio_tracker_l1`) plus our `board.overlay`. It is
  no longer a custom board.
- **nRF 32.768 kHz source is devicetree, not Kconfig**: the split clock drivers read it from the node. A board
  with no LF crystal sets `&lfclk { k32src = "rc"; k32src-accuracy-ppm = <…>; };` and may keep
  `CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC_CALIBRATION=y`. The old `CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC` / `_XTAL` /
  `_<n>PPM` choices are **silently dropped** (a Kconfig warning, no build error; a crystal-less board ends up on a
  floating LFXO: BLE drops, timer skew). `promicro_sx1262` and `me25ls02` are the RC boards. nRF54L boards must
  also enable `&xo` and `&lfclk`.

Patch `0003` (SX126x) is rebased behaviour-preserving (step A: upstream's own duty-cycle machinery removed in
favour of ours), then reduced (step B, 2026-09-19: upstream's DC command helpers, `duty_cycle` fields and
`sx126x_duty_cycle_stop()` reused; each piece benched). Switching to upstream's `SX126X_STATE_RX_DUTY_CYCLE`
state model was deliberately not done — see the patch preamble.

## Decision

Pin a `main` commit in `west.yml`; advance by the west update procedure (update `revision:`,
`west update`, verify the patches apply with a build).

## Consequences

Every bump re-checks the patch watch list.
