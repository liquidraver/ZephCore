# ADR 0006 — SX126x: RX duty cycle from datasheet limits; four-layer RX-busy gating

Status: accepted (recorded 2026-09-25; code in `patches/zephyr/0003-lora-sx126x-native.patch`)

## Context

**RX duty cycle (sniff mode)** — window timing is computed in `LoRaRadio::startReceive()` from primary-source
constraints (SX1261/2 DS §13.1.7 + AN1200.36): deaf time per cycle (sleep +
`hwWakeupTimeUs()` transition) ≤ `(P − 2D − 1)·Tsym` catch budget, and `2R + S ≥ (P + 14)·Tsym` so an early
preamble detect never times out mid-packet. `D = CONFIG_ZEPHCORE_LORA_DC_MIN_SYMBOLS` (8, per AN1200.36).
Drivers accept explicit periods only (no auto-compute). The SX126x patch additionally: pins RX gain (0x08AC) and
TX modulation (0x0889) into the warm-start retention list **once at chip init**, not per DC re-arm (DS §9.6 —
mandatory, else every wake after the first sleep loses 3 dB; the list lives in retention memory and survives
those wakes, so rewriting it per packet only added SPI to the DC deaf window).
`CONFIG_ZEPHCORE_LORA_RETENTION_DEBUG`, enabled by `boards/common/debug.conf`, reads the list and 0x08AC back at
install and after every DC packet, before `restart_rx` rewrites the gain, and warns if either was lost. The patch
also sets `StopTimerOnPreamble=1`, and runs a parked-RX watchdog (two-strike, sampling period
`2×(preamble+8)` symbols floored at 250 ms) that re-arms the cycle after a false preamble detect parks the chip
in RX. Presets whose preamble can't cover the budget (16 symbols at SF≥9, or TCXO transition too large) fall
back to continuous RX. `get dc.restarts` counts false-preamble re-arms.

**RX-busy gating (TX-during-RX prevention)** lives entirely in the SX126x patch and works in four layers:

1. **`rx_packet_active` latch** on `struct sx126x_data` — set by the work handler when `HEADER_VALID` fires on
   DIO1; cleared on `RX_DONE` / `CRC_ERR` / `RX_TX_TIMEOUT` / every RX (re)start / TX-state entry. The latch is
   the primary "we are receiving" signal across the entire payload phase. **Bounded by a payload deadline**
   (patch 0013, `header_seen_at_ms` + `sx126x_max_payload_ms()`): continuous RX has no symbol timer, so a
   `HEADER_VALID` whose packet never completes would otherwise pin the TX gate forever and silently mute the
   node. The bound is max-length-packet airtime at the current SF/BW +25% +100 ms, deliberately generous — it is
   a stuck-state safety net, not a timing mechanism.
2. **Preamble grace** in `sx126x_is_receiving()` — `PREAMBLE_DETECTED` is masked off DIO1 (would fire on noise)
   but visible in the IRQ register. When the poll observes it, a timestamp is recorded; the gate stays busy until
   either `HEADER_VALID` promotes the latch or an SF-aware grace `(preamble_len + 8) × 2^SF / BW` ms elapses.
   Foreign sync words release after the grace; real packets always promote the latch inside it. The poll path
   clears sticky reception bits only to *release* an expired preamble grace or a blown payload deadline (all
   three drivers); otherwise it is read-only.
3. **CAD-busy in-driver RX restore** — `sx126x_lora_send_async` accepts entry CAS from both `REST_STATE → TX` and
   `RX → TX`, so the LBT branch tracks `was_rx` and has `sx126x_restart_rx` put the chip back in RX before
   `-EBUSY` propagates to the C++ caller. LR11xx and LR20xx mirror this in their own `send_async` LBT branches.
   `sx126x_lora_recv_async` has an idempotent fast-path so the C++ failure-path `startReceive()` is a no-op when
   the driver already restored RX.
4. **CAD-timeout recovery** — `mesh::Radio::recoverRxState()` (default no-op, overridden in `LoRaRadio` as
   `hwCancelReceive → _in_recv_mode=0 → _config_cached=false → startReceive`). `Dispatcher::checkSend()`'s 4 s
   CAD-timeout block calls it and re-wakes the loop instead of falling through to TX.

The C++ adapter virtual is `hwIsReceiving()` (formerly `hwIsPreambleDetected` — renamed when the latch made the
old name a lie). It calls the family's `*_is_receiving()` through the ops table (`LoRaRadioOps.h`); that function
may clear bits only for those two releases — never a bit a pending DIO1 event still needs.

## Decision

As above; the other radio families mirror layers 3-4.

## Consequences

Any change to preamble length, DC timing or the IRQ handling must keep the two duty-cycle inequalities and the
four gating layers; bench-check with `get dc.restarts` and CAD stats.
