/*
 * SPDX-License-Identifier: MIT
 * LR11xx Zephyr LoRa driver
 *
 * Implements the standard Zephyr lora_driver_api using the Semtech lr11xx_driver
 * SDK. All SPI access, DIO1 IRQ handling, and radio state management is internal.
 */

#define DT_DRV_COMPAT semtech_lr1110

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include <zephyr/drivers/lora/lr11xx_lora.h>
#include "lr11xx_hal_zephyr.h"
#include "lr11xx_radio.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_system.h"
#include "lr11xx_system_types.h"
#include "lr11xx_regmem.h"

LOG_MODULE_REGISTER(lr11xx_lora, CONFIG_LORA_LOG_LEVEL);

/* Dedicated DIO1 work queue — keeps LoRa interrupt processing off the
 * system work queue so USB/BLE/timer work items cannot delay packet RX. */
#define LR11XX_DIO1_WQ_STACK_SIZE 2560
K_THREAD_STACK_DEFINE(lr11xx_dio1_wq_stack, LR11XX_DIO1_WQ_STACK_SIZE);

/* Wedge-recovery watchdog — its own low-priority queue so the confirm poll
 * (a BUSY-high dwell of up to LR11XX_WEDGE_CONFIRM_MS) never blocks DIO1 RX
 * processing. */
#define LR11XX_WEDGE_WQ_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(lr11xx_wedge_wq_stack, LR11XX_WEDGE_WQ_STACK_SIZE);

#define LR11XX_WEDGE_CHECK_MS   3000  /* base cadence between health checks     */
#define LR11XX_WEDGE_IDLE_MS   12000  /* only probe after this much DIO1 silence */
#define LR11XX_WEDGE_CONFIRM_MS  250  /* continuous BUSY-high past this = wedged */

/* GetVersion "use case" byte — which member of the family answered. */
#define LR11XX_TYPE_LR1110  0x01
#define LR11XX_TYPE_LR1120  0x02
#define LR11XX_TYPE_LR1121  0x03

/* LR1110 firmware below 0x0303 cannot change the LoRa sync word.  MeshCore
 * runs on the private word (0x12); a chip that silently ignores
 * SetLoRaSyncWord stays on the public one (0x34) and is invisible to the
 * mesh — it transmits and receives nothing anyone else hears, with nothing
 * in the log pointing at the cause.  Upstream Zephyr's native lr11xx driver
 * refuses to initialise at all below this version; we log and continue,
 * because a diagnosable radio is more useful in the field than an absent
 * one, and the two firmware revisions seen on real hardware here (0x0307,
 * 0x0401) both clear it comfortably. */
#define LR11XX_MIN_FW_SYNC_WORD  0x0303

/* ── Driver data structures ─────────────────────────────────────────── */

struct lr11xx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec dio1;
	uint16_t tcxo_voltage_mv;
	uint32_t tcxo_startup_delay_ms;
	bool rx_boosted;
	/* RF switch DIO bitmasks */
	uint8_t rfswitch_enable;
	uint8_t rfswitch_standby;
	uint8_t rfswitch_rx;
	uint8_t rfswitch_tx;
	uint8_t rfswitch_tx_hp;
	uint8_t rfswitch_gnss;
	/* PA config */
	uint8_t pa_hp_sel;
	uint8_t pa_duty_cycle;
};

struct lr11xx_data {
	const struct device *dev;
	struct lr11xx_hal_context hal_ctx;
	struct k_mutex spi_mutex;

	/* Cached modem config from lora_config() */
	struct lora_modem_config modem_cfg;
	bool configured;

	/* Async RX state */
	lora_recv_cb async_rx_cb;
	void *async_rx_user_data;

	/* Async TX state */
	struct k_poll_signal *tx_signal;

	/* DIO1 work — runs on dedicated queue, not system work queue */
	struct k_work dio1_work;
	struct k_work_q dio1_wq;

	/* Radio state */
	volatile bool tx_active;
	volatile bool in_rx_mode;

	/* Extension features (duty cycle, boost) */
	bool rx_duty_cycle_enabled;
	bool rx_boost_enabled;
	bool rx_boost_applied;  /* RX boost register written to hardware */

	/* Stored duty-cycle timing from recv_duty_cycle() — the re-arm paths
	 * (start_rx / restart_rx) reuse these exact values, never recompute:
	 * window sizing is owned by the adapter layer (LoRaRadioBase). */
	uint32_t dc_rx_ms;
	uint32_t dc_sleep_ms;

	/* Duty-cycle re-arms triggered by a timeout, i.e. the false-preamble
	 * case: UM §7.2.6 restarts the window timer with 2*RxPeriod +
	 * SleepPeriod on preamble detection, and when that expires with no
	 * packet the chip leaves the loop and the host puts it back.  A climbing
	 * rate means the window is catching noise rather than packets.  Exposed
	 * as `get dc.restarts`, which reported a hardcoded 0 on this radio until
	 * nothing counted them. */
	atomic_t dc_timeout_restarts;

	/* CAD state */
	lora_cad_cb cad_cb;
	void *cad_user_data;
	struct k_sem cad_sem;
	int cad_result;
	/* No cad_active flag: there was one, written at four sites and read at
	 * none.  The SX126x driver's copy IS read — sx126x_handle_irq_timeout()
	 * returns early on it — so this was inherited without the read that gave
	 * it a purpose.
	 *
	 * It is not needed here, and the reason is the locking model, not luck:
	 * sx126x_irq_work_handler() takes no lock at all, so there its timeout
	 * path genuinely races a CAD being set up on the mesh thread and the
	 * flag is the only thing preventing a teardown.  This handler holds
	 * spi_mutex across its whole body, and every CAD entry point holds that
	 * same mutex across SetStandby -> clear_irq_status(ALL) -> SetCad — so
	 * the two cannot interleave, and any TIMEOUT latched before the CAD is
	 * discarded by the clear inside lr11xx_do_cad(). */
	/* Adaptive-CAD: signed offset applied to the per-SF base detPeak on
	 * every LBT CAD; cad_probe_peak overrides for one calibration probe. */
	int8_t cad_peak_offset;
	uint8_t cad_probe_peak;
	/* CAD_RX exit-mode bookkeeping.  With LR11XX_RADIO_CAD_EXIT_MODE_RX a
	 * positive CAD keeps the chip in Rx on the signal it just found instead
	 * of dropping to standby, so the calibration probe can be TOLD whether
	 * the detection was real -- by a packet arriving, or by the chip's own
	 * cad_timeout expiring -- rather than having to guess from a poll.  On a
	 * negative CAD the chip enters standby exactly as with STANDBYRC
	 * (lr11xx_radio_types.h: "If the CAD operation is negative with
	 * RADIO_CAD_EXIT_MODE_RX ... the LR11XX enters Standby RC mode").
	 *
	 * cad_exit_rx marks the CAD in flight as armed that way; cad_rx_state
	 * records which terminal interrupt resolved it. */
	bool cad_exit_rx;
	atomic_t cad_rx_state;

	/* Deferred hardware init — heavy SPI/radio work runs on first config() */
	bool hw_initialized;

	/* DIO1 stuck-HIGH detection: counts consecutive empty IRQ cycles.
	 * If DIO1 stays HIGH with no actionable IRQ for too many cycles,
	 * the LR1110 is hung — trigger a hardware reset. */
	int dio1_stuck_count;

	/* Timestamp (k_uptime_get_32(), ms) of the first sighting of a
	 * latched PREAMBLE_DETECTED by lr11xx_is_receiving() in the current
	 * RX cycle; 0 = none tracked.  Ported from the SX126x preamble-grace
	 * logic: PREAMBLE_DETECTED is not DIO1-routed but latches in the IRQ
	 * status register, and the chip never auto-clears it on a foreign
	 * sync word — without a software bound a foreign/noise preamble pins
	 * the TX gate until the next DIO1 bulk-clear or the dispatcher's 4 s
	 * CAD-fail recovery.  Real packets latch SYNC_WORD_HEADER_VALID
	 * within the grace window; after grace expires with no header, the
	 * bit is cleared and TX released.  All accesses under spi_mutex. */
	uint32_t preamble_seen_at_ms;

	/* Timestamp (k_uptime_get_32(), ms) at which SYNC_WORD_HEADER_VALID was
	 * seen for the reception in progress; 0 = no payload phase being timed.
	 *
	 * Stamped by the DIO1 work handler, which is why that IRQ is routed to
	 * DIO1: with a duty cycle armed lr11xx_is_receiving() may not touch the
	 * bus, so this timestamp is the ONLY thing it can answer from, and while
	 * nothing wrote it there the RX-busy gate was dead on every duty-cycled
	 * node.  The poll path still stamps it too, for the window before the
	 * work item runs.
	 *
	 * Bounds the payload phase the same way preamble_seen_at_ms bounds the
	 * preamble phase: continuous RX (SetRx 0xFFFFFF) has no symbol timer, so
	 * a header whose packet never completes produces no terminal IRQ and
	 * would pin the TX gate true forever — the node keeps receiving but
	 * never transmits again, silently.  Released after
	 * lr11xx_max_payload_ms(), and cleared by every RX (re)start through
	 * lr11xx_reset_rx_busy_signals().  All writes under spi_mutex. */
	uint32_t header_seen_at_ms;

	/* Wedge-recovery watchdog: the LR1110 can rarely be left BUSY-high with
	 * DIO1 low (a command racing the autonomous SetRxDutyCycle sleep phase) —
	 * no IRQ ever fires, so the event-driven driver never re-arms and the node
	 * goes permanently deaf.  last_dio1_ms records the last proof-of-life (a
	 * handled DIO1 event); the watchdog re-arms RX via a hardware reset when
	 * BUSY stays continuously high past any legitimate DC cycle with no DIO1. */
	uint32_t last_dio1_ms;
	struct k_work_delayable wedge_work;
	struct k_work_q wedge_wq;

	/* RX data buffer — filled in DIO1 handler, passed to callback */
	uint8_t rx_buf[256];
};

/* ── Helpers ────────────────────────────────────────────────────────── */

/* Reset all software state that says "we are currently receiving": the
 * preamble-grace timestamp and the payload-phase deadline.  Paired write so
 * the two never drift out of sync — same shape as the SX126x driver's
 * sx126x_reset_rx_busy_signals().  Called from every RX (re)start site and
 * before TX / CAD entry. */
/* cad_rx_state values -- see the field comment in the data struct. */
#define LR11XX_CAD_RX_IDLE   0
#define LR11XX_CAD_RX_ARMED  1
#define LR11XX_CAD_RX_PACKET 2
#define LR11XX_CAD_RX_TMOUT  3

/* Resolve an in-flight CAD_RX with the terminal event that just arrived.
 * A no-op unless one is armed, so the packet path pays one atomic compare.
 * Returns true when this call is the one that resolved it, which is also how
 * the caller tells "the probe's own cad_timeout" apart from an unrelated
 * timeout that happens to raise the same IRQ. */
static inline bool lr11xx_cad_rx_resolve(struct lr11xx_data *data, int outcome)
{
	return atomic_cas(&data->cad_rx_state, LR11XX_CAD_RX_ARMED, outcome);
}

static inline void lr11xx_reset_rx_busy_signals(struct lr11xx_data *data)
{
	data->preamble_seen_at_ms = 0;
	data->header_seen_at_ms = 0;
}

static lr11xx_radio_lora_bw_t bw_enum_to_lr11xx(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_10_KHZ:  return LR11XX_RADIO_LORA_BW_10;
	case BW_15_KHZ:  return LR11XX_RADIO_LORA_BW_15;
	case BW_20_KHZ:  return LR11XX_RADIO_LORA_BW_20;
	case BW_31_KHZ:  return LR11XX_RADIO_LORA_BW_31;
	case BW_41_KHZ:  return LR11XX_RADIO_LORA_BW_41;
	case BW_62_KHZ:  return LR11XX_RADIO_LORA_BW_62;
	case BW_125_KHZ: return LR11XX_RADIO_LORA_BW_125;
	case BW_250_KHZ: return LR11XX_RADIO_LORA_BW_250;
	case BW_500_KHZ: return LR11XX_RADIO_LORA_BW_500;
	default:         return LR11XX_RADIO_LORA_BW_125;
	}
}

static lr11xx_radio_lora_cr_t cr_enum_to_lr11xx(enum lora_coding_rate cr)
{
	switch (cr) {
	case CR_4_5: return LR11XX_RADIO_LORA_CR_4_5;
	case CR_4_6: return LR11XX_RADIO_LORA_CR_4_6;
	case CR_4_7: return LR11XX_RADIO_LORA_CR_4_7;
	case CR_4_8: return LR11XX_RADIO_LORA_CR_4_8;
	default:     return LR11XX_RADIO_LORA_CR_4_8;
	}
}

static lr11xx_system_tcxo_supply_voltage_t get_tcxo_voltage(uint16_t mv)
{
	if (mv >= 3300) return LR11XX_SYSTEM_TCXO_CTRL_3_3V;
	if (mv >= 3000) return LR11XX_SYSTEM_TCXO_CTRL_3_0V;
	if (mv >= 2700) return LR11XX_SYSTEM_TCXO_CTRL_2_7V;
	if (mv >= 2400) return LR11XX_SYSTEM_TCXO_CTRL_2_4V;
	if (mv >= 2200) return LR11XX_SYSTEM_TCXO_CTRL_2_2V;
	if (mv >= 1800) return LR11XX_SYSTEM_TCXO_CTRL_1_8V;
	return LR11XX_SYSTEM_TCXO_CTRL_1_6V;
}

/* Get kHz value from Zephyr BW enum — needed for duty cycle timing */
static float bw_enum_to_khz(enum lora_signal_bandwidth bw)
{
	return lr11xx_radio_get_lora_bw_in_hz(bw_enum_to_lr11xx(bw)) / 1000.0f;
}

/* ── Hardware reset (BUSY stuck recovery) ───────────────────────────── */

static void lr11xx_hardware_reset(struct lr11xx_data *data,
				  const struct lr11xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_INF("LR1110 hardware reset (BUSY stuck recovery)");

	lr11xx_hal_reset(ctx);

	/* TCXO */
	if (cfg->tcxo_voltage_mv > 0) {
		lr11xx_system_set_tcxo_mode(ctx, get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    164);
	}

	lr11xx_system_set_reg_mode(ctx, LR11XX_SYSTEM_REG_MODE_DCDC);

	/* RF switch */
	lr11xx_system_rfswitch_cfg_t rfsw = {
		.enable  = cfg->rfswitch_enable,
		.standby = cfg->rfswitch_standby,
		.rx      = cfg->rfswitch_rx,
		.tx      = cfg->rfswitch_tx,
		.tx_hp   = cfg->rfswitch_tx_hp,
		.tx_hf   = 0,
		.gnss    = cfg->rfswitch_gnss,
		.wifi    = 0,
	};
	lr11xx_system_set_dio_as_rf_switch(ctx, &rfsw);

	lr11xx_system_calibrate(ctx, 0x3F);

	/* Tight ±2 MHz image calibration around actual frequency */
	uint16_t freq_mhz = data->modem_cfg.frequency / 1000000;
	lr11xx_system_calibrate_image_in_mhz(ctx, freq_mhz - 2, freq_mhz + 2);

	lr11xx_radio_set_rx_tx_fallback_mode(ctx, LR11XX_RADIO_FALLBACK_STDBY_RC);

	lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_LORA);

	lr11xx_system_clear_errors(ctx);
	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);

	/* RX boost lost on hardware reset — flag it for re-apply in
	 * start_rx after modem config (where radio is fully configured). */
	data->rx_boost_applied = false;

	lr11xx_hal_enable_dio1_irq(&data->hal_ctx);
}

/* ── Apply modem configuration ──────────────────────────────────────── */

/* Share the LDRO decision between the hardware and its airtime estimate. */
static uint8_t lr11xx_ldro(const struct lora_modem_config *mc)
{
	uint32_t bw_hz = lr11xx_radio_get_lora_bw_in_hz(bw_enum_to_lr11xx(mc->bandwidth));
	uint32_t symbol_time_us = ((1U << (uint8_t)mc->datarate) * 1000000U) / bw_hz;
	return symbol_time_us > 16380;
}

static void lr11xx_apply_modem_config(struct lr11xx_data *data,
				      const struct lr11xx_config *cfg,
				      bool tx_mode)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	lr11xx_radio_set_rf_freq(ctx, mc->frequency);

	lr11xx_radio_mod_params_lora_t mod = {
		.sf   = (lr11xx_radio_lora_sf_t)mc->datarate,
		.bw   = bw_enum_to_lr11xx(mc->bandwidth),
		.cr   = cr_enum_to_lr11xx(mc->coding_rate),
		.ldro = lr11xx_ldro(mc),
	};
	lr11xx_radio_set_lora_mod_params(ctx, &mod);

	lr11xx_radio_pkt_params_lora_t pkt = {
		.preamble_len_in_symb = mc->preamble_len,
		.header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = 255,
		.crc = mc->packet_crc_disable ? LR11XX_RADIO_LORA_CRC_OFF
					      : LR11XX_RADIO_LORA_CRC_ON,
		.iq = mc->iq_inverted ? LR11XX_RADIO_LORA_IQ_INVERTED
				      : LR11XX_RADIO_LORA_IQ_STANDARD,
	};
	lr11xx_radio_set_lora_pkt_params(ctx, &pkt);

	lr11xx_radio_set_lora_sync_word(ctx, mc->public_network ? 0x34 : 0x12);

	if (tx_mode) {
		/* PA config BEFORE TX params — the power byte in SetTxParams
		 * is interpreted against the currently selected PA (Semtech
		 * convention; RadioLib LR11x0::setOutputPower orders it the
		 * same).  Reversed order only bites the first TX after a
		 * (hardware) reset, when the chip default PA is still live. */
		lr11xx_radio_pa_cfg_t pa = {
			.pa_sel = LR11XX_RADIO_PA_SEL_HP,
			.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
			.pa_duty_cycle = cfg->pa_duty_cycle,
			.pa_hp_sel = cfg->pa_hp_sel,
		};
		lr11xx_radio_set_pa_cfg(ctx, &pa);
		lr11xx_radio_set_tx_params(ctx, mc->tx_power,
					   LR11XX_RADIO_RAMP_48_US);
	}

	lr11xx_system_set_dio_irq_params(
		ctx,
		LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TX_DONE |
		LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR |
		LR11XX_SYSTEM_IRQ_HEADER_ERROR |
		/* SYNC_WORD_HEADER_VALID is routed because it is the ONLY thing
		 * that can stamp header_seen_at_ms, and that timestamp is the
		 * entire RX-busy answer once a duty cycle is armed: the poll path
		 * below is forbidden to touch the bus there, so with this bit off
		 * DIO1 nothing ever wrote the field and lr11xx_is_receiving()
		 * returned false unconditionally on every duty-cycled node --
		 * blinding the TX gate, the noise-floor sampler and the CAD probe
		 * alike.
		 *
		 * It is safe to route where PREAMBLE_DETECTED is not: a header
		 * only latches after a sync-word match AND a header CRC, so it
		 * fires at most once per real packet rather than on noise.  This
		 * is the same split the SX126x driver makes for the same reason
		 * (preamble off DIO1, header on) -- the two families now agree.
		 *
		 * The cost is one extra wake per header-decoded packet, on
		 * packets the node is receiving anyway, and it needs a handler
		 * branch that treats the event as "reception continues" rather
		 * than letting the safety net restart RX over it (see there). */
		LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID |
		/* CAD_DONE/CAD_DETECTED MUST be redirected to DIO1 or the blocking
		 * LBT CAD (run before every TX, cad.mode=LBT) never completes — its
		 * semaphore is signaled from the DIO1 handler, so without this the
		 * CAD times out (~200ms) every TX and LBT is dead. The SDK redirects
		 * no IRQ to DIO by default; the SX126x driver keeps these in its mask
		 * too. CAD_DONE only sets during a CAD op, so it's inert during RX. */
		LR11XX_SYSTEM_IRQ_CAD_DONE | LR11XX_SYSTEM_IRQ_CAD_DETECTED,
		0);
}

/* ── RX duty cycle ──────────────────────────────────────────────────── */

/* SetRxDutyCycle(MODE_RX) works the same way as the SX126x: on a positive
 * over-the-air (preamble) detection the chip auto-recomputes its RX timeout
 * to 2*rx_period + sleep_period and stays in RX for the whole packet (SWDR001
 * lr11xx_radio.h step 2).  Because that extension is native, the LR1110 needs
 * NO StopTimerOnPreamble and NO parked-RX watchdog: a false detection lets the
 * bounded 2*rx+sleep timer expire and re-arm via the normal RX-timeout path,
 * so there is no infinite-park failure mode to recover from (unlike the
 * SX126x, whose StopTimerOnPreamble freezes the timer).  Window sizing —
 * including the shared clock/transition safety margin — is owned by the
 * adapter (LoRaRadioBase::startReceive); the driver only re-arms with the
 * stored timing.
 *
 * The earlier "fundamentally broken, 23-40% loss" verdict was a window-sizing
 * bug in the adapter, not a chip defect.  One item is still HW-unverified: the
 * chip's sleep-with-retention warm wakes should keep the boosted RX gain
 * (the SX126x needs an explicit retention-list write for this; the LR11xx SDK
 * exposes no such list, implying it is automatic — measure sensitivity across
 * cycles on a live board to confirm).  We re-apply SetRxBoosted on every host
 * restart regardless, as cheap insurance. */

/* ── Start RX (internal) ────────────────────────────────────────────── */

static void lr11xx_start_rx(struct lr11xx_data *data,
			    const struct lr11xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	/* Standby first — wake from any sleep state */
	data->hal_ctx.radio_is_sleeping = true;
	lr11xx_status_t rc = lr11xx_system_set_standby(ctx,
						       LR11XX_SYSTEM_STANDBY_CFG_RC);
	if (rc != LR11XX_STATUS_OK) {
		LOG_ERR("standby failed (rc=%d) — triggering HW reset", rc);
		lr11xx_hardware_reset(data, cfg);
	}

	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	lr11xx_reset_rx_busy_signals(data);

	/* Apply modem config for RX */
	lr11xx_apply_modem_config(data, cfg, false);

	/* Apply RX boost — persistent register, only written once.
	 * Deferred from hw_init to here so radio is fully configured.
	 *
	 * NOT re-applied per duty-cycle (re)start.  It used to be, as "cheap
	 * insurance against the SX126x-style silent -3 dB loss across warm
	 * starts" — but that was an analogy to a different chip, not a fact
	 * about this one, and the UM contradicts it.  §7.2.6: sending
	 * SetRxDutyCycle in standby means "the context (device configuration) is
	 * saved", and at the end of each sleep window "the device automatically
	 * restarts the process of restoring context"; §6 defines the retention
	 * bit as retaining "device state and firmware data".  SetRxBoosted
	 * (§7.2.12) is an ordinary configuration command, so it is inside that
	 * saved context.
	 *
	 * The SX126x genuinely does need its re-apply — DS §9.6 requires Rx gain
	 * (0x08AC) to be pinned into an explicit warm-start retention list, and
	 * skipping that costs 3 dB on every wake after the first.  The LR11xx has
	 * no such list because it retains configuration wholesale.  Do not port
	 * that fix here by analogy; the LR2021 driver carries the same warning. */
	if (data->rx_boost_enabled && !data->rx_boost_applied) {
		lr11xx_radio_cfg_rx_boosted(ctx, true);
		data->rx_boost_applied = true;
	}

	if (data->rx_duty_cycle_enabled &&
	    data->dc_rx_ms != 0 && data->dc_sleep_ms != 0) {
		/* Sniff mode: standard-RX listening with the chip's autonomous
		 * sleep/wake.  MODE_RX uses the same preamble-detect-and-extend
		 * (2*rx + sleep) mechanism as the SX126x — catches a preamble
		 * mid-window.  Timing is computed by the adapter. */
		lr11xx_radio_set_rx_duty_cycle(ctx, data->dc_rx_ms,
			data->dc_sleep_ms, LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX);
	} else {
		/* Start continuous RX.
		 * 0xFFFFFF is the magic RTC-step value for continuous RX.
		 * Must use the raw RTC-step API — set_rx() converts from ms,
		 * which overflows uint32_t and gives a ~131 s timeout instead. */
		lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);
	}

	/* LR1110 firmware sets CMD_ERROR IRQ flag on several write commands
	 * (SetModParams, SetSyncWord, SetRxBoosted, SetRx) across all
	 * tested FW versions (0x0307, 0x0401).  The commands succeed —
	 * status byte returns OK, BUSY deasserts normally, radio operates
	 * correctly.  RadioLib has the same behavior but never notices
	 * because it doesn't read the IRQ register after write commands.
	 * Clear here so CMD_ERROR doesn't leak into the DIO1 handler. */
	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);

	data->in_rx_mode = true;
	data->tx_active = false;
}

/* ── Lightweight RX restart (no modem reconfig) ─────────────────────── */

/* Used after RX done / CRC error / timeout — frequency/modulation unchanged,
 * skip most of lr11xx_apply_modem_config.
 * Full lr11xx_start_rx() kept for initial start and TX→RX.
 *
 * Packet params are NOT re-applied here — they persist through SetRx.
 * RadioLib (Arduino) also skips re-applying packet params on RX restart.
 * Only TX changes pld_len, and TX→RX goes through full start_rx(). */
/* Returns 0 if the receiver is believed to be back on air, <0 if a command the
 * re-arm depends on was rejected and the caller should escalate.
 *
 * "Believed" on the duty-cycle path: verifying there would mean a GetStatus
 * after SetRxDutyCycle, and that NSS edge terminates the very cycle being
 * checked (UM §7.2.6) — the probe would cause the fault it looks for.  The check
 * sits on the SetStandby instead, which is the command that actually fails on a
 * wedged chip and is safe to poll because the standby has just ended any live
 * cycle. */
/* `in_standby` says the caller already knows the chip is parked in STDBY_RC, so
 * the re-arm can go straight to SetRxDutyCycle.
 *
 * True on every path that follows RX_DONE: UM §7.2.6 has the chip leave the loop
 * and "return to the configured Fallback mode" on reception, and start_rx()
 * programs that fallback as STDBY_RC.  So the standby is already done, by the
 * chip, to spec — re-issuing it and then reading GetStatus to confirm it costs
 * two extra commands (one of them a read, the expensive kind) on the per-packet
 * hot path, and buys nothing: a chip that just delivered a packet is
 * demonstrably taking commands.
 *
 * False only where no RX_DONE was raised — the false-preamble timeout — because
 * there the loop may still be running and driving an NSS edge into it is the
 * race the manual warns about.  That path keeps both the standby and the probe. */
static int lr11xx_restart_rx(struct lr11xx_data *data, bool in_standby)
{
	void *ctx = &data->hal_ctx;

	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	lr11xx_reset_rx_busy_signals(data);

	if (data->rx_duty_cycle_enabled &&
	    data->dc_rx_ms != 0 && data->dc_sleep_ms != 0) {
		/* Re-arm sniff mode with the stored timing (e.g. after RX_DONE
		 * or a false-preamble 2*rx+sleep timeout dropped the chip to
		 * standby).  Re-apply boost — same warm-start caution as
		 * start_rx. */

		/* SetStandby first.  The cycle is not necessarily over when we
		 * get here: UM §7.2.6 terminates the loop on exactly three
		 * things — a packet detected, a host SetStandby, or an NSS wake
		 * from sleep — and a LoRa header error is none of them (it
		 * raises no RX_DONE), so the chip is still cycling on that path.
		 * Re-arming then means driving an NSS edge into a live cycle,
		 * and the manual is explicit about that case: the device "is
		 * woken up from Sleep mode with a falling edge of NSS.  In that
		 * case, the user should send the SetStandby() command to avoid
		 * race conditions".  LR2021 DS §6.3.8 says the same in nearly
		 * the same words, and there the missing standby was observed on
		 * hardware as a latched CMD_ERROR with DIO1 stuck high, five
		 * strikes to a reset, ~88 ms deaf per noise header error.  Free
		 * when the chip is already in standby. */
		if (!in_standby) {
			lr11xx_system_set_standby(ctx,
						  LR11XX_SYSTEM_STANDBY_CFG_RC);

			/* Safe to poll: the standby above ended any live cycle,
			 * so this NSS edge has nothing left to disturb.  A chip
			 * that will not even take a SetStandby will not take a
			 * duty cycle either. */
			lr11xx_system_stat1_t s1 = { 0 };

			if (lr11xx_system_get_status(ctx, &s1, NULL, NULL) ==
				    LR11XX_STATUS_OK &&
			    s1.command_status != LR11XX_SYSTEM_CMD_STATUS_OK &&
			    s1.command_status != LR11XX_SYSTEM_CMD_STATUS_DATA) {
				LOG_WRN("restart_rx: standby rejected (cmd=%d) "
					"before duty-cycle re-arm",
					(int)s1.command_status);
				return -EIO;
			}
		}

		/* No boost re-apply: retained across the duty-cycle sleep, see
		 * the note in lr11xx_start_rx().  This is the per-packet hot
		 * path — the write bought nothing and cost a command on it. */
		lr11xx_radio_set_rx_duty_cycle(ctx, data->dc_rx_ms,
			data->dc_sleep_ms, LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX);
		data->in_rx_mode = true;
		return 0;
	}

	lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);
	/* RX boost persists through SetRx — no re-apply needed. */
	data->in_rx_mode = true;

	/* No duty cycle armed, so polling is free of the NSS hazard. */
	{
		lr11xx_system_stat2_t s2 = { 0 };

		if (lr11xx_system_get_status(ctx, NULL, &s2, NULL) ==
			    LR11XX_STATUS_OK &&
		    s2.chip_mode != LR11XX_SYSTEM_CHIP_MODE_RX) {
			LOG_WRN("restart_rx: chip is in mode %d, not RX",
				(int)s2.chip_mode);
			return -EIO;
		}
	}
	return 0;
}

/* ── DIO1 IRQ handler (work queue, thread context) ──────────────────── */

static void lr11xx_dio1_callback(void *user_data);

static uint32_t lr11xx_cad_rx_timeout_steps(struct lr11xx_data *data);

static void lr11xx_dio1_work_handler(struct k_work *work)
{
	struct lr11xx_data *data = CONTAINER_OF(work, struct lr11xx_data,
						dio1_work);
	const struct lr11xx_config *cfg = data->dev->config;
	void *ctx = &data->hal_ctx;
	bool rx_restarted = false;

	/* Proof of life for the wedge watchdog: the chip generated an IRQ. */
	data->last_dio1_ms = k_uptime_get_32();

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Read IRQ status then clear ALL bits.  Must use ALL_MASK because
	 * LR1110 triggers CMD_ERROR when ClearIrq is called with a mask
	 * that does NOT include the CMD_ERROR bit (all FW versions).
	 * By always clearing ALL, CMD_ERROR gets cleared as part of the
	 * operation. */
	lr11xx_system_irq_mask_t irq = 0;
	lr11xx_status_t rc = lr11xx_system_get_irq_status(ctx, &irq);
	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);

	if (rc != LR11XX_STATUS_OK) {
		LOG_ERR("Failed to read IRQ status (rc=%d)", rc);
		goto safety_check;
	}

	/* CMD_ERROR (bit 22) is expected — LR1110 firmware sets it on
	 * several write commands (SetModParams, SetSyncWord, SetRxBoosted,
	 * SetRx) as a benign side effect on all FW versions (0x0307, 0x0401).
	 * Commands succeed, radio operates correctly.  RadioLib has the same
	 * behavior but never notices because it doesn't read IRQ after writes.
	 * ERROR (bit 23) indicates an actual hardware fault. */
	if (irq & LR11XX_SYSTEM_IRQ_ERROR) {
		LOG_ERR("IRQ hardware ERROR: 0x%08x", irq);
	}

	/* Any valid IRQ clears the stuck counter */
	if (irq != 0) {
		data->dio1_stuck_count = 0;
	}

	/* "A valid header has been seen for the reception in progress" — from
	 * this IRQ word, or from the latch an earlier pass stamped.
	 *
	 * The latch half is load-bearing now that SYNC_WORD_HEADER_VALID is
	 * routed to DIO1: the handler fires mid-packet and bulk-clears the IRQ
	 * register, so by the time RX_DONE arrives the live bit is gone and only
	 * the latch remembers the header.  Testing the live bit alone would read
	 * a foreign header error landing on our good packet as a genuine one and
	 * drop the packet — precisely the ~7-10% under-load LR1110 loss the two
	 * tests below exist to prevent.  Computed here, before any branch can
	 * reset the latch. */
	bool hdr_valid_seen =
		(irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) != 0 ||
		data->header_seen_at_ms != 0;

	/* ── RX done ──
	 * Deliver on RX_DONE unless it is a GENUINE reception failure:
	 *   - CRC_ERROR: a CRC-failed packet asserts RX_DONE+CRC_ERROR together
	 *     on this chip family, so an ungated read would deliver corruption.
	 *   - HEADER_ERROR with NO valid header (SYNC_WORD_HEADER_VALID clear):
	 *     no good header decoded, the buffer is suspect.
	 * A HEADER_ERROR that coincides with a VALID header is a foreign/colliding
	 * signal's error while OUR packet decoded fine — RadioLib's readData keeps
	 * the packet in exactly this case (CRC_ERR || (HDR_ERR && !HDR_VALID)), and
	 * the SX126x path never routes HEADER_ERROR at all.  Dropping these good
	 * packets was the LR1110 under-load packet-loss bug (verified 2026-07-24:
	 * ~7-10% loss vs SX1262, load-dependent, size-skewed). */
	/* Ground truth for a CAD_RX probe: anything that proves a transmitter
	 * was actually there.  A CRC or header error counts as much as a clean
	 * packet -- the probe is asking whether the detection was real, not
	 * whether the packet was usable. */
	if (irq & (LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR |
		   LR11XX_SYSTEM_IRQ_HEADER_ERROR)) {
		lr11xx_cad_rx_resolve(data, LR11XX_CAD_RX_PACKET);
	}

	if ((irq & LR11XX_SYSTEM_IRQ_RX_DONE) &&
	    !(irq & LR11XX_SYSTEM_IRQ_CRC_ERROR) &&
	    !((irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) && !hdr_valid_seen)) {
		lr11xx_radio_rx_buffer_status_t rx_stat;
		lr11xx_radio_get_rx_buffer_status(ctx, &rx_stat);

		if (rx_stat.pld_len_in_bytes > 0 &&
		    rx_stat.pld_len_in_bytes <= 255) {
			lr11xx_radio_pkt_status_lora_t pkt_stat;
			lr11xx_radio_get_lora_pkt_status(ctx, &pkt_stat);

			lr11xx_regmem_read_buffer8(ctx, data->rx_buf,
						   rx_stat.buffer_start_pointer,
						   rx_stat.pld_len_in_bytes);

			/* Buffer-shift errata, header-error variant: if a foreign
			 * HEADER_ERROR coalesced with this good packet, a standby
			 * must reset the +4 buffer_start_pointer drift so it cannot
			 * corrupt the NEXT packet's read.  The payload is already
			 * captured above, so resetting now keeps the packet AND the
			 * errata protection.  (A plain RX_DONE with no header error
			 * gets the same reset from the STDBY_RC rx/tx fallback; this
			 * makes the coalesced case explicit, not fallback-reliant.) */
			if (irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) {
				lr11xx_system_set_standby(ctx,
							  LR11XX_SYSTEM_STANDBY_CFG_RC);
			}

			/* LR1110 errata: RX buffer base shifts 4 bytes per
			 * received packet.  Without clearing, after ~64
			 * packets the offset wraps the 256-byte buffer and
			 * corrupts data.  RadioLib does the same clear. */
			lr11xx_regmem_clear_rxbuffer(ctx);

			/* Lightweight RX restart — no reconfig needed */
			lr11xx_restart_rx(data, true);
			rx_restarted = true;

			/* When SNR < 0 the packet RSSI is dominated by
			 * noise — use the signal-only RSSI estimate for a
			 * more accurate reading on weak links. */
			int16_t rssi = pkt_stat.rssi_pkt_in_dbm;

			if (pkt_stat.snr_pkt_in_db < 0 &&
			    pkt_stat.signal_rssi_pkt_in_dbm > rssi) {
				rssi = pkt_stat.signal_rssi_pkt_in_dbm;
			}

			k_mutex_unlock(&data->spi_mutex);

			/* Fire callback outside mutex */
			if (data->async_rx_cb) {
				data->async_rx_cb(data->dev, data->rx_buf,
						  rx_stat.pld_len_in_bytes,
						  rssi,
						  pkt_stat.snr_pkt_in_db,
						  data->async_rx_user_data);
			}
			return;
		}

		LOG_WRN("RX: invalid len %d", rx_stat.pld_len_in_bytes);
		lr11xx_restart_rx(data, true);
		rx_restarted = true;
	}

	/* ── CAD done ── */
	if (irq & LR11XX_SYSTEM_IRQ_CAD_DONE) {
		bool detected = (irq & LR11XX_SYSTEM_IRQ_CAD_DETECTED) != 0;

		/* CAD_RX with a positive verdict: the chip is in Rx on the
		 * signal it found, not in standby.  Arm the tracker so whichever
		 * terminal interrupt follows records what it was.
		 *
		 * rx_restarted is what stops the safety net at the bottom of
		 * this handler from undoing that.  in_rx_mode is now true and no
		 * branch below sets the flag, so without it every positive probe
		 * ended in "DIO1 safety: no IRQ handled" + lr11xx_restart_rx(),
		 * which forced standby and tore down the very reception the
		 * probe exists to observe: no terminal IRQ ever followed, the
		 * outcome read back as unresolved, and every busy sample was
		 * discarded.  Same meaning as the foreign-header-error branch
		 * below -- reception continues, nothing needs restarting. */
		if (data->cad_exit_rx && detected) {
			/* CAD_ONLY has returned the chip to STBY_RC, so arm the
			 * follow-on Rx explicitly.  This is the same call the
			 * normal receive path uses (see lr11xx_start_rx), so the
			 * receiver is configured exactly as it is when the node is
			 * receiving normally -- which the chip's own CAD_RX exit
			 * evidently is not.  Bounded by the same max-payload
			 * figure cadTimeout used, so a real packet still completes
			 * inside the window and the terminal IRQ (RX_DONE /
			 * CRC_ERROR / HEADER_ERROR / TIMEOUT) stays guaranteed. */
			lr11xx_radio_set_rx_with_timeout_in_rtc_step(
				ctx, lr11xx_cad_rx_timeout_steps(data));
			data->in_rx_mode = true;
			atomic_set(&data->cad_rx_state, LR11XX_CAD_RX_ARMED);
			rx_restarted = true;
		}

		if (data->cad_cb) {
			lora_cad_cb cb = data->cad_cb;
			void *ud = data->cad_user_data;

			data->cad_cb = NULL;
			data->cad_user_data = NULL;
			k_mutex_unlock(&data->spi_mutex);
			cb(data->dev, detected, ud);
			return;
		}

		/* Blocking CAD: signal the semaphore */
		data->cad_result = detected ? 1 : 0;
		k_sem_give(&data->cad_sem);
	}

	/* ── TX done ── */
	if (irq & LR11XX_SYSTEM_IRQ_TX_DONE) {
		data->tx_active = false;

		/* Full restart — modem was reconfigured for TX */
		lr11xx_start_rx(data, cfg);
		rx_restarted = true;

		/* Raise TX signal */
		if (data->tx_signal) {
			k_poll_signal_raise(data->tx_signal, 0);
		}
	}

	/* ── Timeout ── */
	if (irq & LR11XX_SYSTEM_IRQ_TIMEOUT) {
		/* cad_timeout expired with nothing decoded: the detection had
		 * no packet behind it.  Resolved before the branches below,
		 * which put the receiver back on air. */
		bool was_cad_rx = lr11xx_cad_rx_resolve(data,
							LR11XX_CAD_RX_TMOUT);

		if (data->tx_active) {
			/* The chip's Tx timeout fired, so the transmission was
			 * stopped and TX_DONE will never arrive.  This branch
			 * used to do nothing at all here: no log, no recovery,
			 * no signal — the radio simply sat in the post-TX state
			 * until the host wait expired.  Mirror the TX_DONE path
			 * so the receiver goes back on air, and say what
			 * happened.  tx_signal is deliberately NOT raised: the
			 * C++ wait thread treats a raised signal as a completed
			 * send, so raising it here would book a packet that
			 * never left.  Its own timeout owns the accounting. */
			LOG_ERR("TX timeout — chip stopped the transmission, "
				"packet lost");
			data->tx_active = false;
			lr11xx_start_rx(data, cfg);
			rx_restarted = true;
		} else {
			/* Under a duty cycle this is the false-preamble case:
			 * the 2*RxPeriod + SleepPeriod window the preamble
			 * detect restarted (UM §7.2.6) expired with no packet,
			 * so the chip left the loop.  Counting it is what makes
			 * `get dc.restarts` mean something on this radio.
			 *
			 * Not when this timeout is the CAD_RX probe's own
			 * cad_timeout: that is a calibration false positive, it
			 * raises the identical IRQ, and counting it would mix
			 * one tick per busy probe into a figure that is supposed
			 * to describe duty-cycle preamble behaviour. */
			if (data->rx_duty_cycle_enabled && !was_cad_rx) {
				atomic_inc(&data->dc_timeout_restarts);
			}
			if (lr11xx_restart_rx(data, false) < 0) {
				/* Escalate to the full path: forces standby,
				 * reprograms the modem and re-issues SetRx from
				 * scratch.  The LR11xx counterpart of the
				 * SX126x's retry-from-sleep.  If that fails too
				 * the stuck-DIO1 counter still reaches its
				 * hardware reset. */
				LOG_WRN("Timeout: light re-arm failed — full RX restart");
				lr11xx_start_rx(data, cfg);
			}
			rx_restarted = true;
		}
	}

	/* ── CRC / header error ──
	 * A HEADER_ERROR that coincides with a VALID header but no RX_DONE means a
	 * foreign/colliding signal errored while OUR good packet is still
	 * mid-reception.  Do NOT abort it: the old reflex standby()+restart here
	 * dropped the in-flight packet, and doing that on every foreign header
	 * error under load is what cost the LR1110 packets (the SX126x never routes
	 * HEADER_ERROR for exactly this reason).  Leave the chip in RX; the good
	 * packet's own RX_DONE delivers it above. */
	if ((irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) && hdr_valid_seen &&
	    !(irq & LR11XX_SYSTEM_IRQ_RX_DONE)) {
		LOG_DBG("RX: foreign HDR err during valid header — not aborting");
		rx_restarted = true;  /* reception continues; skip safety-net restart */
	} else if (irq & (LR11XX_SYSTEM_IRQ_CRC_ERROR |
			  LR11XX_SYSTEM_IRQ_HEADER_ERROR)) {
		LOG_DBG("RX error: CRC=%d HDR=%d RXDONE=%d",
			(irq & LR11XX_SYSTEM_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) ? 1 : 0,
			(irq & LR11XX_SYSTEM_IRQ_RX_DONE) ? 1 : 0);

		/* LR1110 errata: a real header error (no valid header) drifts the
		 * reported buffer_start_pointer +4 per subsequent packet until a
		 * standby resets it (Arduino MeshCore's CustomLR1110 standbys on
		 * header error).  Only for a genuine header error — a valid header
		 * is handled above and must never trigger this abort. */
		if (irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) {
			lr11xx_system_set_standby(ctx,
						  LR11XX_SYSTEM_STANDBY_CFG_RC);
		}

		/* Drop whatever the failed packet left in the RX buffer — RadioLib
		 * clears it on the CRC-error read path too. */
		lr11xx_regmem_clear_rxbuffer(ctx);

		if (!data->tx_active) {
			lr11xx_restart_rx(data, true);
			rx_restarted = true;
		}

		k_mutex_unlock(&data->spi_mutex);

		/* Notify callback with NULL data for error counting */
		if (data->async_rx_cb) {
			data->async_rx_cb(data->dev, NULL, 0, 0, 0,
					  data->async_rx_user_data);
		}
		return;
	}

	/* ── Header valid ──
	 * A packet is past sync word and header CRC and is now in its payload
	 * phase.  Stamp the latch lr11xx_is_receiving() answers from; bounded
	 * there by lr11xx_max_payload_ms(), and cleared by the terminal event's
	 * lr11xx_reset_rx_busy_signals().
	 *
	 * Gated on no terminal bit for OUR packet in this same pass -- those
	 * branches above own the outcome and have already reset the signals, so
	 * stamping after them would re-arm a gate for a packet that is finished.
	 * HEADER_ERROR is deliberately NOT in that gate: with a valid header it
	 * means a foreign signal errored while our packet is still arriving,
	 * which is exactly a reception to keep the gate closed for.
	 *
	 * rx_restarted, for the same reason the CAD_RX branch sets it: nothing
	 * needs restarting, the chip is mid-packet, and letting the safety net
	 * below fire here would destroy the very packet this branch exists to
	 * protect -- the failure that kept this IRQ off DIO1 in the first
	 * place. */
	if ((irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) &&
	    !(irq & (LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR |
		     LR11XX_SYSTEM_IRQ_TIMEOUT))) {
		uint32_t now = k_uptime_get_32();

		/* 1 as the "set" sentinel if k_uptime is 0 right after boot. */
		data->header_seen_at_ms = (now == 0) ? 1U : now;
		/* The latch is the truth source now; a preamble timestamp left
		 * behind would outlive it and be read as a live grace window. */
		data->preamble_seen_at_ms = 0;
		rx_restarted = true;
	}

safety_check:
	/* Safety net: if we should be in RX but no IRQ branch restarted it,
	 * force a restart.  This catches:
	 *   - SPI failure reading IRQ status (irq=0, rc!=OK)
	 *   - CMD_ERROR-only DIO1 (benign, but radio fell back to standby)
	 *   - Unknown IRQ bits not handled above
	 * Without this, the radio stays in STDBY_RC (fallback mode) and
	 * never receives again — permanently deaf. */
	if (!rx_restarted && data->in_rx_mode && !data->tx_active) {
		LOG_ERR("DIO1 safety: no IRQ handled (0x%08x rc=%d), "
			"restarting RX", irq, rc);
		/* State genuinely unknown here — an SPI failure reading the IRQ
		 * register or an unhandled bit means we cannot claim the chip
		 * left the loop.  Keep the defensive standby; this is the one
		 * path where it is not redundant. */
		lr11xx_restart_rx(data, false);
	}

	/* Edge-triggered DIO1: if the pin is still HIGH after processing,
	 * a new IRQ arrived during handling.  No rising edge will fire,
	 * so re-submit work to process the pending flags.
	 *
	 * Guard against DIO1 stuck HIGH: if we loop here with no
	 * actionable IRQ, the LR1110 is in a bad state.  After 5
	 * consecutive empty cycles, do a full hardware reset. */
	if (gpio_pin_get_dt(&data->hal_ctx.dio1)) {
		data->dio1_stuck_count++;
		if (data->dio1_stuck_count >= 5) {
			LOG_ERR("DIO1 stuck HIGH for %d cycles — "
				"hardware reset", data->dio1_stuck_count);
			data->dio1_stuck_count = 0;
			lr11xx_hardware_reset(data, cfg);
			lr11xx_start_rx(data, cfg);
		} else {
			k_work_submit_to_queue(&data->dio1_wq,
					       &data->dio1_work);
		}
	} else {
		data->dio1_stuck_count = 0;
	}

	k_mutex_unlock(&data->spi_mutex);
}

/* HAL DIO1 callback → submits work */
static void lr11xx_dio1_callback(void *user_data)
{
	struct lr11xx_data *data = (struct lr11xx_data *)user_data;
	k_work_submit_to_queue(&data->dio1_wq, &data->dio1_work);
}

/* Forward declaration — hw_init is defined after driver API functions */
static int lr11xx_hw_init(struct lr11xx_data *data,
			  const struct lr11xx_config *cfg);

/* ── Driver API: config ─────────────────────────────────────────────── */

static int lr11xx_lora_config(const struct device *dev,
			      const struct lora_modem_config *config)
{
	struct lr11xx_data *data = dev->data;
	const struct lr11xx_config *cfg = dev->config;

	/* Deferred hardware init — first config() triggers the heavy work */
	if (!data->hw_initialized) {
		int ret = lr11xx_hw_init(data, cfg);
		if (ret != 0) {
			LOG_ERR("Hardware init failed: %d", ret);
			return ret;
		}
	}

	memcpy(&data->modem_cfg, config, sizeof(*config));
	data->configured = true;

	/* Tight ±2 MHz image calibration for the configured frequency */
	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	uint16_t freq_mhz = config->frequency / 1000000;
	lr11xx_system_calibrate_image_in_mhz(&data->hal_ctx,
					      freq_mhz - 2, freq_mhz + 2);
	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("config: %uHz SF%d BW%d CR%d pwr=%d tx=%d",
		config->frequency, config->datarate, config->bandwidth,
		config->coding_rate, config->tx_power, config->tx);

	return 0;
}

/* ── Driver API: airtime ────────────────────────────────────────────── */

static uint32_t lr11xx_lora_airtime(const struct device *dev,
				    uint32_t data_len)
{
	struct lr11xx_data *data = dev->data;
	struct lora_modem_config *mc = &data->modem_cfg;

	/* Use Semtech's calculation, including SF5/6 synchronization and the
	 * actual LDRO/CRC configuration.  The old SF>=11 && BW<=125 shortcut
	 * underestimated SF10/BW62.5 and SF12/BW250, including the TX watchdog
	 * budgets derived from this API. */
	lr11xx_radio_mod_params_lora_t mod = {
		.sf = (lr11xx_radio_lora_sf_t)mc->datarate,
		.bw = bw_enum_to_lr11xx(mc->bandwidth),
		.cr = cr_enum_to_lr11xx(mc->coding_rate),
		.ldro = lr11xx_ldro(mc),
	};
	lr11xx_radio_pkt_params_lora_t pkt = {
		.preamble_len_in_symb = mc->preamble_len,
		.header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = (uint8_t)data_len,
		.crc = mc->packet_crc_disable ? LR11XX_RADIO_LORA_CRC_OFF
					      : LR11XX_RADIO_LORA_CRC_ON,
	};
	return lr11xx_radio_get_lora_time_on_air_in_ms(&pkt, &mod);
}

/* Forward declaration — needed by LBT in send_async */
static int lr11xx_lora_cad(const struct device *dev, k_timeout_t timeout);

/* Blocking-CAD wait budget scaled to the actual CAD duration:
 * nSym * Tsym + startup radio-side, plus IRQ latency margin.  A fixed
 * 200 ms fits 2-symbol CAD everywhere but is exceeded by 4-symbol CAD
 * on slow presets (SF12 @ 62.5 kHz = ~262 ms). */
static uint32_t lr11xx_cad_timeout_ms(struct lr11xx_data *data)
{
	struct lora_modem_config *mc = &data->modem_cfg;
	uint8_t sf = (uint8_t)mc->datarate;
	uint8_t symb_nb = mc->cad.symbol_num ?
			  (uint8_t)mc->cad.symbol_num : 2;
	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(mc->bandwidth) * 1000.0f);

	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 200;
	}

	uint32_t tsym_us = ((1UL << sf) * 1000000UL) / bw_hz;
	/* +1 symbol covers radio startup + internal processing tail */
	uint32_t ms = ((symb_nb + 1U) * tsym_us) / 1000U + 100U;

	return MAX(ms, 200U);
}

/* ── Driver API: send_async ─────────────────────────────────────────── */

static int lr11xx_lora_send_async(const struct device *dev,
				  uint8_t *buf, uint32_t data_len,
				  struct k_poll_signal *async)
{
	struct lr11xx_data *data = dev->data;
	const struct lr11xx_config *cfg = dev->config;
	void *ctx = &data->hal_ctx;

	if (!data->configured) return -EINVAL;
	if (data->tx_active) return -EBUSY;
	if (data_len > 255 || data_len == 0) return -EINVAL;

	/* LBT: perform blocking CAD before transmitting.  On CAD-busy, restore
	 * RX in-driver before returning -EBUSY so the C++ layer doesn't have
	 * to do a full cancel-then-restart round-trip.  lr11xx_lora_cad
	 * transitions the chip to STANDBY and clears data->in_rx_mode as
	 * part of running CAD; capture the pre-CAD state to know whether
	 * to re-arm. */
	if (data->modem_cfg.cad.mode == LORA_CAD_MODE_LBT) {
		bool was_in_rx = data->in_rx_mode;
		int cad_ret = lr11xx_lora_cad(dev,
					      K_MSEC(lr11xx_cad_timeout_ms(data)));
		if (cad_ret > 0) {
			if (was_in_rx && data->async_rx_cb != NULL) {
				k_mutex_lock(&data->spi_mutex, K_FOREVER);
				lr11xx_start_rx(data, cfg);
				k_mutex_unlock(&data->spi_mutex);
			}
			return -EBUSY;
		}
		if (cad_ret < 0 && cad_ret != -ENOSYS) {
			LOG_WRN("LBT: CAD failed (%d), proceeding with TX",
				cad_ret);
		}
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Cancel RX */
	data->async_rx_cb = NULL;
	data->in_rx_mode = false;

	lr11xx_hal_disable_dio1_irq(&data->hal_ctx);

	/* Standby — wake from sleep if needed */
	data->hal_ctx.radio_is_sleeping = true;
	lr11xx_status_t rc = lr11xx_system_set_standby(ctx,
						       LR11XX_SYSTEM_STANDBY_CFG_RC);
	if (rc != LR11XX_STATUS_OK) {
		LOG_ERR("TX standby failed — HW reset");
		lr11xx_hardware_reset(data, cfg);
	}

	/* Apply TX config */
	lr11xx_apply_modem_config(data, cfg, true);

	/* Set TX packet length */
	lr11xx_radio_pkt_params_lora_t pkt = {
		.preamble_len_in_symb = data->modem_cfg.preamble_len,
		.header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = (uint8_t)data_len,
		.crc = data->modem_cfg.packet_crc_disable
			? LR11XX_RADIO_LORA_CRC_OFF
			: LR11XX_RADIO_LORA_CRC_ON,
		.iq = data->modem_cfg.iq_inverted
			? LR11XX_RADIO_LORA_IQ_INVERTED
			: LR11XX_RADIO_LORA_IQ_STANDARD,
	};
	lr11xx_radio_set_lora_pkt_params(ctx, &pkt);

	/* Write TX buffer */
	lr11xx_regmem_write_buffer8(ctx, buf, data_len);

	/* Clear IRQ, enable DIO1 */
	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	lr11xx_reset_rx_busy_signals(data);
	lr11xx_hal_enable_dio1_irq(&data->hal_ctx);

	/* Store signal and start TX.
	 *
	 * Chip-side Tx safeguard, scaled from airtime.  UM §7.2.3: "If the RTC
	 * event fires before the end of transmission, it will trigger a TIMEOUT
	 * IRQ, and stop the device transmission" — so this must exceed real
	 * airtime.  The fixed 10 s it replaces carried the right reasoning with
	 * the arithmetic never done: the comment said the worst preset "exceeds
	 * 5 s", but SF12/BW62.5 at 255 bytes CR 4/8 is 28.6 s, so 10 s truncated
	 * every packet from 76 B up there, and from 31 B at SF12/BW31.25.
	 *
	 * Floored at the previous 10000 so nothing that works today tightens.
	 * Programmed in RTC steps rather than through the millisecond wrapper:
	 * lr11xx_radio_convert_time_in_ms_to_rtc_step() computes ms * 32768 in
	 * uint32 and overflows above 131071 ms, which SF12/BW7.81 at 255 B
	 * (~229 s) reaches.  Saturating at the 24-bit field is 512 s. */
	data->tx_signal = async;
	data->tx_active = true;
	{
		uint32_t air_ms = lr11xx_lora_airtime(dev, data_len);
		uint32_t tmo_ms = air_ms + (air_ms / 4U) + 500U;
		uint64_t steps;

		if (tmo_ms < 10000U) {
			tmo_ms = 10000U;
		}
		steps = ((uint64_t)tmo_ms * 32768U) / 1000U;
		if (steps > 0x00FFFFFFU) {
			steps = 0x00FFFFFFU;
		}
		LOG_DBG("SET_TX: airtime=%u ms, timeout=%u ms (%u steps)",
			air_ms, tmo_ms, (uint32_t)steps);
		lr11xx_radio_set_tx_with_timeout_in_rtc_step(ctx,
							     (uint32_t)steps);
	}

	k_mutex_unlock(&data->spi_mutex);
	return 0;
}

/* ── Driver API: send (sync) ────────────────────────────────────────── */

static int lr11xx_lora_send(const struct device *dev,
			    uint8_t *buf, uint32_t data_len)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);

	int ret = lr11xx_lora_send_async(dev, buf, data_len, &done);
	if (ret < 0) return ret;

	uint32_t air_time = lr11xx_lora_airtime(dev, data_len);
	ret = k_poll(&evt, 1, K_MSEC(2 * air_time + 1000));
	if (ret < 0) {
		LOG_ERR("TX sync timeout");
		return ret;
	}

	return 0;
}

/* ── Driver API: recv_async ─────────────────────────────────────────── */

static int lr11xx_lora_recv_async(const struct device *dev,
				  lora_recv_cb cb, void *user_data)
{
	struct lr11xx_data *data = dev->data;
	const struct lr11xx_config *cfg = dev->config;

	/* NULL cb = cancel RX */
	if (cb == NULL) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->async_rx_cb = NULL;
		data->async_rx_user_data = NULL;
		data->in_rx_mode = false;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) return -EINVAL;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;

	lr11xx_start_rx(data, cfg);

	k_mutex_unlock(&data->spi_mutex);
	return 0;
}

/* ── Driver API: recv_duty_cycle ─────────────────────────────────────── */

static int lr11xx_lora_recv_duty_cycle(const struct device *dev,
				       k_timeout_t rx_period,
				       k_timeout_t sleep_period,
				       lora_recv_cb cb, void *user_data)
{
	struct lr11xx_data *data = dev->data;
	const struct lr11xx_config *cfg = dev->config;

	/* NULL cb = cancel (same as recv_async cancel) */
	if (cb == NULL) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->async_rx_cb = NULL;
		data->async_rx_user_data = NULL;
		data->rx_duty_cycle_enabled = false;
		data->in_rx_mode = false;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	/* Explicit timing only — the adapter (LoRaRadioBase) owns the window
	 * sizing.  No driver-side auto-compute. */
	if (K_TIMEOUT_EQ(rx_period, K_FOREVER) ||
	    K_TIMEOUT_EQ(sleep_period, K_FOREVER)) {
		LOG_ERR("recv_duty_cycle: explicit rx/sleep periods required");
		return -EINVAL;
	}

	uint32_t rx_ms = k_ticks_to_ms_ceil32(rx_period.ticks);
	uint32_t slp_ms = k_ticks_to_ms_ceil32(sleep_period.ticks);
	if (rx_ms < 1) rx_ms = 1;
	if (slp_ms < 1) slp_ms = 1;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;
	data->dc_rx_ms = rx_ms;
	data->dc_sleep_ms = slp_ms;
	data->rx_duty_cycle_enabled = true;

	/* start_rx() reads rx_duty_cycle_enabled + dc_*_ms and issues
	 * SetRxDutyCycle(MODE_RX) with a forced boost re-apply. */
	lr11xx_start_rx(data, cfg);

	k_mutex_unlock(&data->spi_mutex);
	LOG_INF("recv_duty_cycle: rx=%ums sleep=%ums", rx_ms, slp_ms);
	return 0;
}

/* ── Driver API: recv (sync) ─────────────────────────────────────────── */

static int lr11xx_lora_recv(const struct device *dev, uint8_t *buf,
			    uint8_t size, k_timeout_t timeout,
			    int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	return -ENOTSUP;
}

/* ── LR11xx extension API ───────────────────────────────────────────── */

/* ── Duty-cycle ownership ─────────────────────────────────────────────
 *
 * UM §7.2.6 lists exactly three ways the loop ends: a packet is received (the
 * chip raises RX_DONE and returns to the configured fallback mode), the host
 * issues SetStandby during the Rx window, or the chip is woken from the sleep
 * phase by a falling edge of NSS — and for that last case the manual instructs
 * "the user should send the SetStandby(...) command to avoid race conditions".
 *
 * The host issues every NSS edge, so the host can always know whether the cycle
 * is still running — but only if it never issues one speculatively and hopes.
 * The previous approach did hope: a BUSY read before the command, skipping if
 * high.  That cannot be made correct on this family for two independent
 * reasons.  BUSY is high during the sleep phase (where a command is fatal) AND
 * through ordinary Rx (where it is harmless), so the pin does not distinguish
 * the two — measured on a T1000-E as 406 of 407 sampler bursts refused, and 76
 * of 76 with the duty cycle switched off entirely.  And even a correct reading
 * is check-then-act: the chip can enter its sleep phase between the GPIO read
 * and the NSS assert.
 *
 * So the driver takes ownership instead.  A caller that must talk to the chip
 * brackets its work in suspend/resume: the cycle is ended deliberately with the
 * SetStandby the manual asks for, the work happens against a chip in a known
 * state, and the cycle is re-armed explicitly.  Nothing is left to timing.
 *
 * Cost is bounded and small — a stand-down plus re-arm is four short commands,
 * and the sampler that drives it runs once per noise-floor interval.
 *
 * Both helpers require data->spi_mutex held by the caller.
 */
static bool lr11xx_dc_suspend(struct lr11xx_data *data)
{
	if (!data->rx_duty_cycle_enabled || !data->in_rx_mode) {
		return false;
	}

	/* Sanctioned termination, and simultaneously the remedy the manual
	 * prescribes for an NSS edge that has already woken the chip — so this
	 * is correct whether or not the cycle actually survived to this point. */
	lr11xx_system_set_standby(&data->hal_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
	data->in_rx_mode = false;
	return true;
}

static void lr11xx_dc_resume(struct lr11xx_data *data, bool was_armed)
{
	if (!was_armed) {
		return;
	}

	/* Same re-arm sequence as restart_rx().  No boost re-apply — retained
	 * across the duty-cycle sleep, see the note in lr11xx_start_rx().  The
	 * IRQ/latch state is cleared so the new cycle starts from a known point. */
	lr11xx_system_clear_irq_status(&data->hal_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	lr11xx_reset_rx_busy_signals(data);
	lr11xx_radio_set_rx_duty_cycle(&data->hal_ctx, data->dc_rx_ms,
				       data->dc_sleep_ms,
				       LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX);
	data->in_rx_mode = true;
}

static uint32_t lr11xx_preamble_grace_ms(struct lr11xx_data *data);

/* Is a duty-cycled reception already under way?  Ask before lr11xx_dc_suspend().
 *
 * dc_suspend() stands the chip down unconditionally, which ends any reception
 * in progress, and with the cycle armed lr11xx_is_receiving() cannot see the
 * preamble phase at all: it answers from the header latch only, because a bus
 * access in the sleep phase kills the cycle.  So every discretionary stand-down
 * -- the noise-floor sampler above all -- could land between preamble detect
 * and header, and lost that packet without a trace (T1000-E bench, 2026-09-19:
 * each missed packet raised no IRQ at all, and coincided with a sampler burst
 * whose RSSI read the packet itself, ~30 dB over the floor).
 *
 * Reading the IRQ register here is safe in both phases.  In the sleep phase
 * the NSS edge wakes the chip, but the caller is about to terminate and re-arm
 * the cycle anyway -- the same sanctioned termination dc_suspend() documents.
 * With a preamble latched the chip is in Rx (StopTimerOnPreamble holds it
 * there), where bus access is routine.  PREAMBLE_DETECTED is not DIO1-routed but
 * does latch in the register.  A preamble older than the SF-aware grace with no
 * header is treated as stale (foreign sync word / false detect), exactly as the
 * continuous-RX poll in lr11xx_is_receiving() does, so it cannot starve the
 * callers: they proceed, and dc_resume() clears it.
 *
 * Caller holds data->spi_mutex. */
static bool lr11xx_dc_rx_in_flight(struct lr11xx_data *data)
{
	lr11xx_system_irq_mask_t irq = 0;

	if (!data->rx_duty_cycle_enabled || !data->in_rx_mode) {
		return false;
	}
	if (lr11xx_system_get_irq_status(&data->hal_ctx, &irq) != LR11XX_STATUS_OK) {
		return false;
	}
	if (irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {
		return true;
	}
	if (irq & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {
		uint32_t now = k_uptime_get_32();
		uint32_t seen = data->preamble_seen_at_ms;

		if (seen == 0) {
			data->preamble_seen_at_ms = (now == 0) ? 1U : now;
			return true;
		}
		return (now - seen) < lr11xx_preamble_grace_ms(data);
	}
	return false;
}

/* Front-end settle to wait after entering Rx before the first GetRssiInst.
 *
 * DS Table 13-82 puts the RSSI averaging window at ~936 us*kHz / BW and the
 * post-Rx-entry delay at 12-15 of those windows; the C++ sampler will not read
 * until 16 have passed (rssi_settle_delay_us(), radio_common.h).  An Rx entry
 * made HERE has to clear the same bar, or the reading it brackets comes from a
 * front end the firmware itself considers unsettled.
 *
 * The flat 1 ms this replaces clears it from BW20.83 up but not below --
 * BW15.63 wants 1008 us, BW10.42 1504 us, BW7.81 2144 us -- and `set radio`
 * accepts bandwidths that low.  Floored at 1000 so no preset that works today
 * waits less than it did.  Deliberately the same 936/16 the C++ side uses: two
 * settle models that could disagree would be worse than one that is wrong. */
static uint32_t lr11xx_rssi_settle_us(struct lr11xx_data *data)
{
	uint32_t bw_khz = (uint32_t)bw_enum_to_khz(data->modem_cfg.bandwidth);
	uint32_t settle;

	if (bw_khz == 0) {
		return 1000U;
	}
	settle = ((936U + bw_khz - 1U) / bw_khz) * 16U;

	return settle < 1000U ? 1000U : settle;
}

/* Sleep the whole millisecond, busy-wait only the remainder.  k_sleep() rounds
 * up to a tick and the tick period is a board Kconfig, so asking it for a
 * sub-millisecond excess is not portable; this keeps the old wait exactly and
 * adds the deficit on top. */
static void lr11xx_rssi_settle(struct lr11xx_data *data)
{
	uint32_t settle_us = lr11xx_rssi_settle_us(data);

	k_sleep(K_MSEC(1));
	if (settle_us > 1000U) {
		k_busy_wait(settle_us - 1000U);
	}
}

int16_t lr11xx_get_rssi_inst(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;
	int8_t rssi;
	int16_t out = -128;

	/* Non-blocking: a contended bus means the sampler simply retries.  -128
	 * is the sentinel LoRaRadioBase::triggerNoiseFloorCalibrate expects. */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return -128;
	}
	if (lr11xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return -128;
	}

	bool armed = lr11xx_dc_suspend(data);

	if (armed) {
		/* GetRssiInst measures a live receiver; the stand-down left the
		 * chip in standby, where there is nothing to measure.  Enter
		 * continuous Rx for the reading, then hand the cycle back.
		 *
		 * The settle wait is the receiver's, not the bus's: UM Rx timing
		 * has the value valid once the front end has settled after Rx
		 * entry, and lr11xx_rssi_settle() sizes that from the bandwidth
		 * (a flat 1 ms was short of it below BW20.83).  It is also the
		 * entire cost of the manoeuvre — against a noise-floor interval
		 * measured in seconds, the receiver is off air for well under a
		 * hundredth of a percent of the time. */
		lr11xx_radio_set_rx_with_timeout_in_rtc_step(&data->hal_ctx,
							     0xFFFFFF);
		lr11xx_rssi_settle(data);
	}

	if (lr11xx_radio_get_rssi_inst(&data->hal_ctx, &rssi) ==
	    LR11XX_STATUS_OK) {
		out = (int16_t)rssi;
	}

	lr11xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);

	return out;
}

/* Grace period for the PREAMBLE_DETECTED -> SYNC_WORD_HEADER_VALID gap,
 * SF/BW-aware — same formula as the SX126x driver: (preamble_len + 8)
 * symbols covers worst-case preamble remainder + sync word + header
 * decode with margin. */
static uint32_t lr11xx_preamble_grace_ms(struct lr11xx_data *data)
{
	uint8_t sf = (uint8_t)data->modem_cfg.datarate;
	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(data->modem_cfg.bandwidth) * 1000.0f);
	uint16_t preamble = data->modem_cfg.preamble_len;

	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 1000;  /* safe default ~1 s if config is uninitialised */
	}
	uint64_t us = ((uint64_t)(preamble + 8U) << sf) * 1000000ULL / bw_hz;

	return (uint32_t)((us + 999U) / 1000U);
}

/* Upper bound on the payload phase: airtime of a maximum-length (255 byte)
 * explicit-header packet at the current SF/BW, worst-case coding rate 4/8,
 * plus margin.  Bounds the lifetime of the SYNC_WORD_HEADER_VALID busy state
 * in lr11xx_is_receiving().
 *
 * Why the header bit needs a deadline at all: it is cleared only by the
 * terminal DIO1 event's bulk clear or an RX (re)start, and continuous RX
 * (SetRx 0xFFFFFF) has no symbol timer, so neither is guaranteed to arrive.
 * A header whose packet never completes would pin the TX gate true forever
 * and silently mute the node until reboot.  Same reasoning and same formula
 * as sx126x_max_payload_ms() (patch 0013); Arduino MeshCore added the
 * equivalent bound for the LR11x0 in 0bd871cd.
 *
 * Deliberately generous — this is a stuck-state safety net, and releasing
 * early would let TX start on top of a packet that is still arriving.  LDRO
 * (DE) is pinned at 1 because that yields the larger symbol count, i.e. the
 * safer bound. */
static uint32_t lr11xx_max_payload_ms(struct lr11xx_data *data)
{
	uint8_t sf = (uint8_t)data->modem_cfg.datarate;
	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(data->modem_cfg.bandwidth) * 1000.0f);

	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 30000;  /* safe default if config is uninitialised */
	}

	/* Semtech payload-symbol count with PL=255, CRC on, explicit header,
	 * CR = 4/8 (coded_bits = 8), DE = 1:
	 *   n = 8 + ceil((8*PL - 4*SF + 28 + 16) / (4*(SF - 2*DE))) * 8
	 * DE=1 so the divisor is 4*(SF-2); at SF5 that is 12, never zero. */
	uint32_t numer = 8U * 255U + 28U + 16U;
	uint32_t denom = 4U * (uint32_t)(sf - 2U);

	if (numer > 4U * (uint32_t)sf) {
		numer -= 4U * (uint32_t)sf;
	}
	uint32_t n_sym = 8U + ((numer + denom - 1U) / denom) * 8U;

	/* n_sym * 2^sf * 1000000 / bw_hz -> us, then +25% and +100 ms margin. */
	uint64_t us = ((uint64_t)n_sym << sf) * 1000000ULL / bw_hz;
	uint32_t ms = (uint32_t)((us + 999U) / 1000U);

	return ms + (ms / 4U) + 100U;
}

/* The cad_timeout a CAD_RX probe programs, in RTC steps at 32768 Hz.
 *
 * Computed here rather than through lr11xx_radio_convert_time_in_ms_to_rtc_step()
 * for the reason the Tx path already documents: that helper is
 * `(uint32_t)(time_in_ms * LR11XX_RTC_FREQ_IN_HZ / 1000)`, so the multiply
 * overflows uint32 above 131071 ms.  lr11xx_max_payload_ms() crosses that at
 * SF12/BW15.63 (~136 s), SF12/BW10.42 (~204 s), SF12/BW7.81 (~286 s) and
 * SF11/BW7.81 (~152 s), where the wrapped value would end the Rx almost
 * immediately -- truncating a real reception (UM: the chip stays in Rx until a
 * packet is demodulated or the timer expires) and booking the probe as a false
 * positive.  Same 64-bit maths and same 24-bit saturation as the Tx timeout;
 * the ceiling is 512 s, which no preset reaches. */
static uint32_t lr11xx_cad_rx_timeout_steps(struct lr11xx_data *data)
{
	uint64_t steps = ((uint64_t)lr11xx_max_payload_ms(data) * 32768U) / 1000U;

	if (steps > 0x00FFFFFFU) {
		steps = 0x00FFFFFFU;
	}
	return (uint32_t)steps;
}

int lr11xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			 uint32_t spacing_us)
{
	struct lr11xx_data *data = dev->data;
	int got = 0;

	/* One stand-down for the WHOLE burst, not one per sample.
	 *
	 * lr11xx_get_rssi_inst() has to suspend the duty cycle, enter continuous
	 * Rx, wait 1 ms for the front end, read, and then hand the cycle back --
	 * and lr11xx_dc_resume() clears every IRQ and zeroes the RX-busy latch,
	 * because a re-armed cycle must start from a known point.  The noise-floor
	 * sampler wants a median of eight, and calling the single-shot read eight
	 * times therefore cost eight cycle tear-downs, eight latch wipes and 8 ms
	 * of settle every sampling interval -- on a receiver whose whole job is to
	 * be listening.  That is the same fault the CAD classifier had, in a
	 * different function; here it fires whether or not a probe follows.
	 *
	 * Bracketing once collapses it to one of each.  It also holds the SPI
	 * mutex for a single short span instead of taking it eight times, and the
	 * reads are spaced by the caller's averaging window exactly as before, so
	 * they stay independent. */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return 0;
	}
	/* A packet already on its way in: same verdict as a preamble landing
	 * inside the window -- the caller abandons the burst and retries. */
	if (lr11xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return -EBUSY;
	}

	bool armed = lr11xx_dc_suspend(data);

	if (armed) {
		lr11xx_radio_set_rx_with_timeout_in_rtc_step(&data->hal_ctx,
							     0xFFFFFF);
		/* Start the window clean so the check after the loop reports
		 * only what arrived INSIDE it -- these bits are latched and
		 * nothing clears them between duty-cycle re-arms, so a stale
		 * preamble from earlier in the cycle would otherwise condemn
		 * every burst.  Safe here and nowhere else: the cycle is already
		 * torn down and dc_resume() clears the lot again on the way out,
		 * so this destroys no state the poll path could still want.
		 * CMD_ERROR rides along because the LR1110 raises it on any
		 * ClearIrq whose mask excludes it (all FW versions). */
		lr11xx_system_clear_irq_status(&data->hal_ctx,
					       LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
					       LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID |
					       LR11XX_SYSTEM_IRQ_CMD_ERROR);
		lr11xx_rssi_settle(data);
	}

	for (int i = 0; i < n; i++) {
		int8_t rssi;

		if (i) {
			k_busy_wait(spacing_us);
		}
		if (lr11xx_radio_get_rssi_inst(&data->hal_ctx, &rssi) != LR11XX_STATUS_OK) {
			break;
		}
		out[i] = (int16_t)rssi;
		got++;
	}

	/* Did a transmitter turn up inside the window?  Then these samples
	 * measure it, not the floor, and the caller must throw them away.
	 *
	 * This covers a gap the caller's own guards cannot.  cadMaintenance()
	 * re-checks isReceiving() immediately before its CAD probe, and that
	 * now works on this family -- but dc_resume() below zeroes
	 * header_seen_at_ms on its way out, one call earlier, so a reception
	 * that began inside this bracket is invisible to the re-check by the
	 * time it runs.  Reporting the contamination from in here, where the
	 * evidence still exists, closes it: the sampler abandons the burst,
	 * _sample_fresh stays clear, and the probe -- which rides on that flag
	 * -- does not run this pass.
	 *
	 * Only meaningful with a cycle armed.  Without one this function makes
	 * no Rx entry of its own and clears nothing, so the caller's guards see
	 * the reception unaided. */
	if (armed) {
		lr11xx_system_irq_mask_t irq = 0;

		if (lr11xx_system_get_irq_status(&data->hal_ctx, &irq) ==
		    LR11XX_STATUS_OK &&
		    (irq & (LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
			    LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID))) {
			got = -EAGAIN;
		}
	}

	lr11xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);

	return got;
}

bool lr11xx_is_receiving(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	/* Payload phase, from the latch the DIO1 handler stamps on
	 * SYNC_WORD_HEADER_VALID.  Read before the duty-cycle split because it
	 * is the one signal both sides share — and the only one the armed side
	 * has at all.
	 *
	 * This path runs on every TX gate, so it cannot bracket itself in
	 * suspend/resume the way the samplers do — standing the cycle down to
	 * ask "am I receiving?" would end the very reception being asked about.
	 * It must not fall back to a BUSY read either: BUSY is high both in the
	 * sleep phase and through ordinary Rx on this family, so a BUSY-high
	 * "not receiving" answer is wrong exactly when it matters — mid-packet,
	 * with the TX gate asking for permission to transmit over it.
	 *
	 * That leaves the latch, which is why the header IRQ is routed to DIO1
	 * (see lr11xx_configure_irq): the handler needs no bus access to keep
	 * this current, and until it did the armed side had nothing to read. */
	uint32_t hdr_seen = data->header_seen_at_ms;

	if (hdr_seen != 0 &&
	    (k_uptime_get_32() - hdr_seen) < lr11xx_max_payload_ms(data)) {
		return true;
	}

	if (data->rx_duty_cycle_enabled) {
		/* Never stamped, or the payload deadline has blown.  No bus
		 * access is permitted here to look any further, and none is
		 * wanted: the latch is the whole answer on this side.  Drop a
		 * stale one so the compare above stops repeating, but only if
		 * the mutex is free — the DIO1 handler owns this field, and if
		 * it is mid-update it will maintain the field itself. */
		if (hdr_seen != 0 &&
		    k_mutex_lock(&data->spi_mutex, K_NO_WAIT) == 0) {
			LOG_WRN("RX header latched %u ms with no packet, releasing TX gate",
				k_uptime_get_32() - hdr_seen);
			data->header_seen_at_ms = 0;
			k_mutex_unlock(&data->spi_mutex);
		}
		return false;
	}

	/* Non-blocking — skip if SPI busy */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return false;
	}

	lr11xx_system_irq_mask_t irq = 0;
	lr11xx_system_get_irq_status(&data->hal_ctx, &irq);

	/* Header landed: payload phase in progress.  The bit stays latched
	 * until the terminal DIO1 event bulk-clears it, so this covers the
	 * whole packet — bounded by a payload deadline, because in continuous
	 * RX that terminal event is not guaranteed to arrive and a header that
	 * never completes would otherwise mute TX until reboot. */
	if (irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {
		uint32_t now = k_uptime_get_32();

		/* The DIO1 handler normally stamps this before any poll sees
		 * the bit; the poll can still get there first in the window
		 * before the work item runs, so it stays able to stamp.  A latch
		 * still inside its deadline already returned true at the top of
		 * the function, so reaching here with one set means the deadline
		 * is blown and the gate has to be released. */
		if (data->header_seen_at_ms == 0) {
			data->header_seen_at_ms = (now == 0) ? 1U : now;
			k_mutex_unlock(&data->spi_mutex);
			return true;
		}
		LOG_WRN("RX header latched %u ms with no packet, releasing TX gate",
			now - data->header_seen_at_ms);
		/* Drop the sticky reception bits so the next poll starts clean.
		 * CMD_ERROR goes with them — the LR1110 sets it whenever ClearIrq
		 * is called with a mask that excludes it (all FW versions). */
		lr11xx_system_clear_irq_status(&data->hal_ctx,
					       LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
					       LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID |
					       LR11XX_SYSTEM_IRQ_HEADER_ERROR |
					       LR11XX_SYSTEM_IRQ_CMD_ERROR);
		lr11xx_reset_rx_busy_signals(data);
		k_mutex_unlock(&data->spi_mutex);
		return false;
	}

	/* PREAMBLE_DETECTED with SF-aware grace (SX126x-parity).  Within
	 * grace, keep reporting busy so a real packet has time to land its
	 * header; after grace with no header, assume foreign sync word,
	 * clear the bit and release TX. */
	if (irq & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {
		uint32_t now = k_uptime_get_32();
		uint32_t seen = data->preamble_seen_at_ms;

		if (seen == 0) {
			data->preamble_seen_at_ms = (now == 0) ? 1U : now;
			k_mutex_unlock(&data->spi_mutex);
			return true;
		}
		if ((now - seen) < lr11xx_preamble_grace_ms(data)) {
			k_mutex_unlock(&data->spi_mutex);
			return true;
		}
		/* Grace expired.  Clear CMD_ERROR along with the preamble
		 * bit — the LR1110 sets CMD_ERROR whenever ClearIrq is
		 * called with a mask that excludes it (all FW versions). */
		lr11xx_system_clear_irq_status(&data->hal_ctx,
					       LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
					       LR11XX_SYSTEM_IRQ_CMD_ERROR);
		lr11xx_reset_rx_busy_signals(data);
		k_mutex_unlock(&data->spi_mutex);
		return false;
	}

	/* No preamble, no header: nothing in flight. */
	lr11xx_reset_rx_busy_signals(data);
	k_mutex_unlock(&data->spi_mutex);
	return false;
}

uint32_t lr11xx_get_dc_timeout_restarts(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	return (uint32_t)atomic_get(&data->dc_timeout_restarts);
}

void lr11xx_reset_dc_timeout_restarts(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	atomic_set(&data->dc_timeout_restarts, 0);
}

uint32_t lr11xx_get_wakeup_time_us(const struct device *dev)
{
	const struct lr11xx_config *cfg = dev->config;
	/* Context restore + PLL lock (~1.5 ms) plus the TCXO startup delay
	 * where fitted.  Per the LR11xx SetRxDutyCycle model the wake
	 * transition is deaf time between sleep and RX, counting against the
	 * adapter's preamble-catch budget — same accounting as the SX126x. */
	uint32_t us = 1500;

	if (cfg->tcxo_voltage_mv > 0) {
		us += cfg->tcxo_startup_delay_ms * 1000U;
	}
	return us;
}

void lr11xx_set_rx_boost(const struct device *dev, bool enable)
{
	struct lr11xx_data *data = dev->data;

	/* Skip if already in the desired state — SetRxBoosted is a
	 * persistent register, redundant calls are wasteful. */
	if (data->rx_boost_enabled == enable) {
		return;
	}

	data->rx_boost_enabled = enable;
	LOG_DBG("RX boost %s", enable ? "enabled" : "disabled");

	if (data->in_rx_mode && data->configured) {
		/* Radio is fully configured — safe to apply immediately */
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		lr11xx_radio_cfg_rx_boosted(&data->hal_ctx, enable);
		/* Clear spurious CMD_ERROR from SetRxBoosted (FW artifact) */
		lr11xx_system_clear_irq_status(&data->hal_ctx,
					       LR11XX_SYSTEM_IRQ_ALL_MASK);
		data->rx_boost_applied = enable;
		k_mutex_unlock(&data->spi_mutex);
	} else {
		/* Defer to next start_rx where radio will be configured */
		data->rx_boost_applied = false;
	}
}

uint32_t lr11xx_get_random(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;
	uint32_t random = 0;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr11xx_system_get_random_number(&data->hal_ctx, &random);
	k_mutex_unlock(&data->spi_mutex);

	return random;
}

/* ── Extension API: receiver hygiene ─────────────────────────────────
 *
 * Warm sleep to drop the analog front end, then recalibrate on the way back up
 * — Semtech's stated remedy for a jammed AGC, and the same sequence as Arduino
 * MeshCore's lr11x0ResetAGC() (helpers/radiolib/LR11x0Reset.h).  Driven from
 * LoRaRadioBase::agcMaintenance() on RX silence, never on the packet path.
 *
 * Unlike the LR2021, calibrate(0x3F) here DOES include image rejection and
 * reverts it to the 902-928 MHz default, so the image cal must be re-issued —
 * exactly what the Arduino helper does.  That makes recal_fe moot on this part:
 * both entry points pay it because the calibration forces it.
 */
static void lr11xx_recalibrate_locked(struct lr11xx_data *data)
{
	void *ctx = &data->hal_ctx;
	lr11xx_system_sleep_cfg_t sleep_cfg = {
		.is_warm_start = true,
		.is_rtc_timeout = false,
	};

	lr11xx_system_set_sleep(ctx, sleep_cfg, 0);
	k_sleep(K_USEC(500));
	data->hal_ctx.radio_is_sleeping = true;

	lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
	lr11xx_system_calibrate(ctx, 0x3F);

	if (data->configured) {
		uint16_t freq_mhz = data->modem_cfg.frequency / 1000000;

		lr11xx_system_calibrate_image_in_mhz(ctx, freq_mhz - 4,
						     freq_mhz + 4);
	}

	/* Let the calibration actually finish before handing the chip back.
	 *
	 * Calibrate asserts BUSY for the duration, but the HAL's wait_on_busy()
	 * samples the pin within a couple of GPIO reads of releasing NSS and
	 * returns immediately if BUSY has not risen yet — the same "BUSY has not
	 * risen" race behind the stale-reply guard in the HAL and the GetTemp
	 * wedge.  Lose it here and the caller's startReceive() issues
	 * SetRxDutyCycle into a chip that is still calibrating, the command is
	 * dropped, and the radio never re-enters Rx: deaf until the next reset
	 * 60 s later.  Measured 2026-08-23: the 60 s window after a reset carried
	 * a 7.4% miss rate against 0.6% elsewhere, and one deaf stretch began at
	 * one reset and ended exactly at the next.
	 *
	 * A flat settle wait sidesteps the race entirely — this path is already
	 * a warm sleep plus a full recalibration, so it is nobody's fast path. */
	k_sleep(K_MSEC(10));

	if (data->rx_boost_enabled) {
		lr11xx_radio_cfg_rx_boosted(ctx, true);
		data->rx_boost_applied = true;
	}

	/* Contract with the caller: leave the driver out of RX so its
	 * startReceive() performs a real re-entry. */
	data->in_rx_mode = false;
	lr11xx_reset_rx_busy_signals(data);
}

/* Redo the frequency-dependent calibrations after temperature drift.
 *
 * This is NOT an "AGC reset".  That is an SX126x remedy for an SX126x fault —
 * Semtech prescribe warm sleep plus recalibration for a jammed AGC on that part,
 * and ZephCore inherited the idea from Arduino MeshCore's `agc_reset_interval`.
 * Neither the LR11xx UM nor anything measured here describes such a fault on
 * this generation, and running the sequence speculatively cost real packets
 * (T1000-E, 2026-08-23: 7.4%% miss rate in the 60 s after a fire, against 0.6%%
 * elsewhere).  So the driver no longer offers one; it offers the operation the
 * datasheet does call for, under the name of what it actually does.
 *
 * Leaves the driver out of RX — the caller must startReceive() afterwards. */
void lr11xx_recalibrate(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	if (!data->configured) {
		return;
	}

	/* Bounded, for the same reason sx126x_reset_agc() bounds its own: a long
	 * TX or RX holds this mutex for the full airtime, and K_FOREVER would
	 * stall the dispatcher thread for all of it. */
	if (k_mutex_lock(&data->spi_mutex, K_MSEC(50)) != 0) {
		LOG_DBG("recalibrate: mutex busy, deferring");
		return;
	}
	if (lr11xx_dc_rx_in_flight(data)) {
		LOG_DBG("recalibrate: reception in flight, deferring");
		k_mutex_unlock(&data->spi_mutex);
		return;
	}

	/* Own the cycle across the whole sequence.  This opens with SetSleep,
	 * which is fatal to a chip already in its own sleep phase, and the
	 * recalibration leaves the radio in standby regardless — so the cycle
	 * has to be re-armed here rather than left to whoever calls next. */
	bool armed = lr11xx_dc_suspend(data);

	lr11xx_recalibrate_locked(data);

	lr11xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);
}

/* Junction temperature in whole degrees C, INT16_MIN if unavailable.
 * UM: T = (Temp(10:0)/2047 * Vana - Vbe25) * 1000/VbeSlope + 25, with typicals
 * Vana 1.35 V, Vbe25 0.7295 V, VbeSlope -1.7 mV/C.  Runs once per maintenance
 * pass, so the float is free. */
int16_t lr11xx_get_chip_temp_c(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;
	uint16_t raw = 0;
	lr11xx_status_t rc;

	if (!data->configured) {
		return INT16_MIN;
	}
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return INT16_MIN;
	}
	if (lr11xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return INT16_MIN;
	}

	/* Bracketed rather than skipped: GetTemp into a chip parked in the
	 * duty-cycle sleep phase both wedges it BUSY-high and, per UM §7.2.6,
	 * silently ends the cycle.  Owning the stand-down removes both. */
	bool armed = lr11xx_dc_suspend(data);

	rc = lr11xx_system_get_temp(&data->hal_ctx, &raw);

	lr11xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);

	if (rc != LR11XX_STATUS_OK) {
		return INT16_MIN;
	}

	float v = ((float)(raw & 0x07FF) / 2047.0f) * 1.35f - 0.7295f;
	float t = v * (1000.0f / -1.7f) + 25.0f;

	return (int16_t)t;
}

/* ── Deferred hardware init (runs on first lora_config call) ────────── */

/* ── Driver API: CAD ────────────────────────────────────────────────── */

/* Recommended cad_detect_peak, from Semtech's own reference stack:
 * LoRa Basics Modem v4.9.0, ral_lr11xx.c ral_lr11xx_get_lora_cad_det_peak().
 *
 * Provenance matters here, because the table this replaces was wrong twice
 * over.  It was `{56,56,56,58,58,60,64,68}`, labelled "from SX1261/62/68 /
 * LR1110 reference (same silicon IP)" — but that is byte-for-byte the *LR20xx*
 * 2-symbol row (ral_lr20xx.c), i.e. the wrong chip family, sampled at the wrong
 * symbol count.  The SX126x scale is ~20-35 and shares nothing with this one.
 * The error was worst at SF6/SF7, where it read 56 against Semtech's 52, and it
 * is what drove field units eight rungs down to the offset rail: measured on
 * two T1000-E companions at SF7/BW62.5, both pinned at o:-8 with a flat, clean
 * FP curve, one of them also sitting on the driver's own peak clamp.
 *
 * Unlike the LR20xx, this family's detPeak is strongly bandwidth-dependent —
 * at SF7 Semtech spans 52/64/77 across BW125/250/500, ~12 counts per octave,
 * against ~1-3 per octave on the SX126x.  A bandwidth-blind base table is
 * therefore a much larger error here than it is there, which is precisely why
 * the SX1262 in the same room settled at offset -1 while these walked to -8.
 *
 * Below BW125 Semtech returns RAL_STATUS_UNKNOWN_VALUE and offers nothing.  We
 * run BW62.5 by default, so that gap is our normal operating point, and the
 * sub-125 row below is MEASURED rather than published.
 *
 * Provenance and its honest limit: over 2026-08-30..09-01 two LR1110 T1000-Es —
 * one here, one in another country, on different sites — both converged to an
 * absolute detPeak of 44 at SF7/BW62.5, i.e. offset -7 against the BW125 row's
 * 51 (52 minus the 4-symbol correction).  Two independent sites agreeing to the
 * count is what makes this a measurement; three SX1262s in the same campaign
 * landed within one count of each other across a house, a roof and a
 * mountaintop, which is the general finding that the site scales the FP curve
 * without moving its bend.
 *
 * The correction is applied FLAT, as a -7 translation of the whole BW125 row.
 * A constant preserves the per-SF shape Semtech actually measured; scaling each
 * SF by its own bandwidth slope would lean on their noisiest dimension (SF9
 * steps +5 then +15 across the two published octaves) and distort that shape
 * from a single anchor point.  Flat also errs more sensitive at high SF, the
 * safe side of an asymmetric offset range (-8 down, +12 up).
 *
 * Sanity check on the magnitude, since only one SF was measured: Semtech's own
 * SF7 trend is 52/64/77 across BW125/250/500, about 12 counts per octave, so a
 * linear extrapolation one octave down would predict -12 and a base of 40.  The
 * measured -7 sits between zero (what we shipped before) and that, which is the
 * shape expected from a curve flattening at the narrow end.  The measurement is
 * not fighting the trend; it lands inside the bracket the trend allows.
 *
 * ONE SF MEASURED, SEVEN EXTRAPOLATED.  That is the real limit of this row, and
 * it is acceptable only because the closed-loop staircase exists to find the
 * local value from a starting point — this makes the start honest, it does not
 * claim to be the answer. */
static uint8_t lr11xx_cad_detect_peak(uint8_t sf, uint16_t bw_khz, uint8_t symb_nb)
{
	/*        SF5 SF6 SF7 SF8 SF9 SF10 SF11 SF12 */
	static const uint8_t bw500[8] = { 65, 70, 77, 85, 78, 80, 79, 82 };
	static const uint8_t bw250[8] = { 60, 61, 64, 72, 63, 71, 73, 75 };
	static const uint8_t bw125[8] = { 56, 52, 52, 58, 58, 62, 66, 68 };
	/* bw125 - 7, measured at SF7/BW62.5 on two sites.  See the note above. */
	static const uint8_t bw_sub125[8] = { 49, 45, 45, 51, 51, 55, 59, 61 };
	const uint8_t *row;
	int peak;

	if (sf < 5 || sf > 12) {
		sf = 9;  /* mid-range fallback */
	}

	if (bw_khz >= 500) {
		row = bw500;
	} else if (bw_khz >= 250) {
		row = bw250;
	} else if (bw_khz >= 125) {
		row = bw125;
	} else {
		/* Narrower than BW125: Semtech publishes nothing, we measured. */
		row = bw_sub125;
	}
	peak = (int)row[sf - 5];

	/* More symbols means more looks at the same correlation, so the same
	 * detection quality is reached at a lower threshold.  Semtech applies
	 * this correction after the table lookup; we run 4 symbols everywhere
	 * (LORA_CAD_SYMB_4 in LoRaRadioBase::buildModemConfig), so it always
	 * bites, and omitting it was one further count of the SF7 error. */
	if (symb_nb >= 8) {
		peak -= 2;
	} else if (symb_nb >= 4) {
		peak -= 1;
	}

	return (uint8_t)peak;
}

/* The detPeak range this driver will actually program.  Exported through
 * lr11xx_cad_peak_min/max() so the C++ adaptive-CAD controller can narrow its
 * offset window to match: where base+offset falls outside this, several offsets
 * collapse onto one peak and the staircase reads sampling noise between
 * identical configurations as curvature.  That is not hypothetical — it is the
 * documented failure mode on the LR2021 (see LR2021Radio::hwCadPeakMin), and
 * the old 48 floor here reproduced it on the LR1110 at SF7.
 *
 * 40 is DELIBERATELY left where it was when the sub-125 row was measured down
 * to 45 (base 44 after the 4-symbol correction), which means it now binds:
 * 44 + CAD_LEVEL_MIN(-8) = 36 is below it, so CadController::levelMinEff() narrows the
 * offset window to -4..+12 at SF6 and SF7 below BW125.  That narrowing is
 * intended, and it must not be "fixed" by lowering this constant.
 *
 * The reason is what the sub-125 row is built on: two LR1110s, on different
 * sites in different countries, converged to the SAME absolute detPeak.  A base
 * anchored by two independent agreeing measurements does not need eight rungs
 * of downward travel — it needs to be centred, which it now is.  The old -8
 * window was sized for a base that was wrong by seven counts; carrying that
 * much headroom onto a corrected base would be carrying the symptom across the
 * fix.  CAD_SWEEP_MIN is -4, so the dry-run sweep still fits exactly.
 *
 * Only SF6 and SF7 below BW125 narrow at all.  SF5 keeps the full window by one
 * count (48 - 8 = 40), and every other cell sits well clear:
 *
 *   sub-125 base (4 sym)   SF5 48  SF6 44  SF7 44  SF8 50 ... SF12 60
 *   effective min offset       -8      -4      -4      -8         -8
 *
 * SF7/BW62.5 is of course the default preset, so the one configuration that
 * narrows is the one that matters — which is the point, since it is also the
 * only one anyone has measured.  The upper bound is untouched: the highest base
 * is 85 (SF8, BW500) against CAD_LEVEL_MAX +12.
 *
 * Watch item: if a third, quieter LR1110 site ever rails at -4, that is the
 * signal to revisit this — and it is the third data point the sub-125 row wants
 * in any case. */
#define LR11XX_CAD_PEAK_MIN 40
#define LR11XX_CAD_PEAK_MAX 100

uint8_t lr11xx_cad_peak_min(void)
{
	return LR11XX_CAD_PEAK_MIN;
}

uint8_t lr11xx_cad_peak_max(void)
{
	return LR11XX_CAD_PEAK_MAX;
}

static int lr11xx_do_cad(struct lr11xx_data *data)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	/* Both the table lookup and the timeout need the symbol count, so it is
	 * resolved before the base peak rather than after it. */
	uint8_t symb_nb = mc->cad.symbol_num ? (uint8_t)mc->cad.symbol_num : 2;
	uint16_t bw_khz = (uint16_t)bw_enum_to_khz(mc->bandwidth);
	uint8_t detect_peak = lr11xx_cad_detect_peak(sf, bw_khz, symb_nb);

	if (mc->cad.detection_peak != 0) {
		detect_peak = mc->cad.detection_peak;
	} else if (data->cad_peak_offset != 0) {
		/* Adaptive-CAD operating offset (base +/- learned delta).
		 * LR11xx detPeak scale is ~50-85 — never mix with SX126x. */
		int peak = (int)detect_peak + data->cad_peak_offset;

		if (peak < LR11XX_CAD_PEAK_MIN) {
			peak = LR11XX_CAD_PEAK_MIN;
		} else if (peak > LR11XX_CAD_PEAK_MAX) {
			peak = LR11XX_CAD_PEAK_MAX;
		}
		detect_peak = (uint8_t)peak;
	}
	if (data->cad_probe_peak != 0) {
		/* One-shot calibration probe: absolute peak wins over all. */
		detect_peak = data->cad_probe_peak;
	}

	lr11xx_radio_cad_params_t cad = {
		.cad_symb_nb = symb_nb,
		.cad_detect_peak = detect_peak,
		.cad_detect_min = mc->cad.detection_minimum ? mc->cad.detection_minimum : 10,
		/* Exit mode is per-CAD.  The calibration probe arms CAD_RX so a
		 * positive detection flows straight into Rx on the signal it
		 * found; the pre-TX LBT keeps STANDBYRC, where the verdict is
		 * all the caller wants and entering Rx would leave the chip
		 * somewhere the transmit path does not expect.  Detection is
		 * identical either way -- same symbols, same detPeak, same
		 * detMin -- so the probe still measures what LBT runs.
		 *
		 * cad_timeout bounds the Rx a positive CAD_RX enters, in RTC
		 * steps at 32768 Hz -- the unit LR11XX_RTC_FREQ_IN_HZ names
		 * (30.5176 us), not the 31.25 us the header's prose claims, and
		 * the LR20xx driver already documents that these doc comments
		 * are not reliable.  Max-length-packet airtime is the right
		 * bound: anything shorter would cut off the packet the detection
		 * was for.  See lr11xx_cad_rx_timeout_steps() for why the
		 * vendor's millisecond wrapper is not used to convert it. */
		/* Always CAD_ONLY on the wire.  A CAD_RX exit leaves this chip
		 * in CHIP_MODE_RX but DEAF: measured 2026-09-02, twelve armed
		 * windows produced no RX_DONE, SYNC_WORD_HEADER_VALID,
		 * CRC_ERROR or HEADER_ERROR (all four routed to DIO1) while a
		 * packet arrived every 9 s and ~3 were expected inside them --
		 * only the chip's own cadTimeout ever came back.  A controlled
		 * A/B put the cost at 14%% of received packets with probing on
		 * versus 0%% with it off, which C1 forbids outright, and it is
		 * also why `tp` could never be recorded on this family.
		 *
		 * So do not use the chip's CAD_RX exit.  Take the documented
		 * STANDBYRC exit and arm the follow-on Rx ourselves with a
		 * plain SetRx in the DIO1 handler -- the same call the normal
		 * receive path uses, which demonstrably works (120/120 in that
		 * control).  Same remedy as the SX126x, where the chip's
		 * cadTimeout was likewise not honoured. */
		.cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC,
		.cad_timeout = 0,
	};

	lr11xx_radio_set_cad_params(ctx, &cad);

	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	lr11xx_reset_rx_busy_signals(data);
	lr11xx_radio_set_cad(ctx);

	return 0;
}

static int lr11xx_lora_cad(const struct device *dev, k_timeout_t timeout)
{
	struct lr11xx_data *data = dev->data;
	int ret;

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	bool was_in_rx = data->in_rx_mode;

	if (was_in_rx) {
		data->in_rx_mode = false;
		lr11xx_system_set_standby(&data->hal_ctx,
					  LR11XX_SYSTEM_STANDBY_CFG_RC);
	}

	k_sem_reset(&data->cad_sem);
	data->cad_result = -ETIMEDOUT;
	data->cad_cb = NULL;

	ret = lr11xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	if (ret < 0) {
		return ret;
	}

	ret = k_sem_take(&data->cad_sem, timeout);
	if (ret == -EAGAIN) {
		return -ETIMEDOUT;
	}

	return data->cad_result;
}

static int lr11xx_lora_cad_async(const struct device *dev,
				  lora_cad_cb cb, void *user_data)
{
	struct lr11xx_data *data = dev->data;

	if (cb == NULL) {
		data->cad_cb = NULL;
		data->cad_user_data = NULL;
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	if (data->in_rx_mode) {
		data->in_rx_mode = false;
		lr11xx_system_set_standby(&data->hal_ctx,
					  LR11XX_SYSTEM_STANDBY_CFG_RC);
	}

	data->cad_cb = cb;
	data->cad_user_data = user_data;

	int ret = lr11xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	return ret;
}

/* ── Extension API: adaptive CAD ────────────────────────────────────── */

void lr11xx_cad_set_peak_offset(const struct device *dev, int8_t offset)
{
	struct lr11xx_data *data = dev->data;

	data->cad_peak_offset = offset;
}

uint8_t lr11xx_cad_base_peak(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	/* Must mirror lr11xx_do_cad()'s lookup exactly — bandwidth and symbol
	 * count included.  This is what `get cad` prints as the base and what
	 * the C++ staircase offsets from, so a base that disagreed with the
	 * peak actually programmed would make every rung a lie. */
	return lr11xx_cad_detect_peak(
		(uint8_t)data->modem_cfg.datarate,
		(uint16_t)bw_enum_to_khz(data->modem_cfg.bandwidth),
		data->modem_cfg.cad.symbol_num
			? (uint8_t)data->modem_cfg.cad.symbol_num : 2);
}

int lr11xx_cad_probe(const struct device *dev, int8_t peak_offset)
{
	struct lr11xx_data *data = dev->data;
	int base = (int)lr11xx_cad_base_peak(dev);
	int peak = base + peak_offset;
	int ret;

	if (peak < LR11XX_CAD_PEAK_MIN) {
		peak = LR11XX_CAD_PEAK_MIN;
	} else if (peak > LR11XX_CAD_PEAK_MAX) {
		peak = LR11XX_CAD_PEAK_MAX;
	}

	/* One-shot absolute override consumed by lr11xx_do_cad().  Probes and
	 * LBT both run on the mesh loop thread, so no concurrent CAD exists --
	 * which is also what makes cad_exit_rx safe as a plain flag. */
	data->cad_probe_peak = (uint8_t)peak;
	data->cad_exit_rx = true;
	atomic_set(&data->cad_rx_state, LR11XX_CAD_RX_IDLE);
	ret = lr11xx_lora_cad(dev, K_MSEC(lr11xx_cad_timeout_ms(data)));
	data->cad_exit_rx = false;
	data->cad_probe_peak = 0;

	/* 2 rather than 1 tells the caller the chip is in Rx on the signal it
	 * detected, so it must NOT re-enter Rx itself, and that an outcome will
	 * be readable from lr11xx_cad_rx_outcome() once the chip resolves it. */
	if (ret > 0) {
		return 2;
	}

	return ret;
}

uint32_t lr11xx_cad_rx_timeout_ms(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	/* Derived from the exact step count do_cad() programs, so the caller's
	 * wait and the chip's deadline cannot drift apart -- including when the
	 * 24-bit saturation in lr11xx_cad_rx_timeout_steps() shortens it. */
	return (uint32_t)(((uint64_t)lr11xx_cad_rx_timeout_steps(data) * 1000U)
			  / 32768U);
}

int lr11xx_cad_rx_outcome(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;
	atomic_val_t st = atomic_get(&data->cad_rx_state);

	if (st != LR11XX_CAD_RX_PACKET && st != LR11XX_CAD_RX_TMOUT) {
		return 0;  /* nothing armed, or still awaiting the terminal IRQ */
	}

	atomic_set(&data->cad_rx_state, LR11XX_CAD_RX_IDLE);
	return (st == LR11XX_CAD_RX_PACKET) ? 1 : 2;
}

/* ── Deferred hardware init ─────────────────────────────────────────── */

static int lr11xx_hw_init(struct lr11xx_data *data,
			  const struct lr11xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_INF("LR11xx hardware init starting");

	/* Hardware reset + chip detection with retry.  Noisy power-up or
	 * slow TCXO startup can cause the first attempt to fail.  RadioLib
	 * retries up to 10 times — we use 3 which is plenty for Zephyr
	 * where POST_KERNEL init runs well after power stabilises. */
	lr11xx_system_version_t ver;
	bool found = false;

	for (int attempt = 0; attempt < 3; attempt++) {
		lr11xx_hal_status_t hal_rc = lr11xx_hal_reset(ctx);
		if (hal_rc != LR11XX_HAL_STATUS_OK) {
			LOG_WRN("LR11xx reset failed (attempt %d)", attempt);
			k_msleep(10);
			continue;
		}

		lr11xx_status_t st = lr11xx_system_get_version(ctx, &ver);
		if (st == LR11XX_STATUS_OK) {
			found = true;
			break;
		}

		LOG_WRN("LR11xx get_version failed (attempt %d)", attempt);
		k_msleep(10);
	}

	if (!found) {
		LOG_ERR("LR11xx not found after 3 attempts");
		return -EIO;
	}

	LOG_INF("LR11xx HW:0x%02X Type:0x%02X FW:0x%04X",
		ver.hw, ver.type, ver.fw);

	if (ver.type == LR11XX_TYPE_LR1110 && ver.fw < LR11XX_MIN_FW_SYNC_WORD) {
		LOG_ERR("LR1110 FW 0x%04X < 0x%04X: sync word cannot be changed, "
			"node will stay on the public sync word and be invisible "
			"to the mesh — upgrade the chip firmware",
			ver.fw, LR11XX_MIN_FW_SYNC_WORD);
	}

	/* TCXO */
	if (cfg->tcxo_voltage_mv > 0) {
		lr11xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    164);
		LOG_DBG("TCXO: %dmV", cfg->tcxo_voltage_mv);
	}

	/* DC-DC */
	lr11xx_system_set_reg_mode(ctx, LR11XX_SYSTEM_REG_MODE_DCDC);

	/* RF switch */
	lr11xx_system_rfswitch_cfg_t rfsw = {
		.enable  = cfg->rfswitch_enable,
		.standby = cfg->rfswitch_standby,
		.rx      = cfg->rfswitch_rx,
		.tx      = cfg->rfswitch_tx,
		.tx_hp   = cfg->rfswitch_tx_hp,
		.tx_hf   = 0,
		.gnss    = cfg->rfswitch_gnss,
		.wifi    = 0,
	};
	lr11xx_system_set_dio_as_rf_switch(ctx, &rfsw);
	LOG_INF("RF switch: en=0x%02x rx=0x%02x tx=0x%02x txhp=0x%02x",
		rfsw.enable, rfsw.rx, rfsw.tx, rfsw.tx_hp);

	/* Calibrate all 6 blocks (LF RC, HF RC, PLL, ADC, IMG, PLL TX).
	 * LR11xx has 6 cal blocks (0x3F), not 7 like SX126x. */
	lr11xx_system_calibrate(ctx, 0x3F);
	LOG_INF("Calibration OK");

	/* After RX/TX, fall back to STBY_RC (not FS).  Without this the
	 * chip may linger in FS mode, affecting RX chain re-init. */
	lr11xx_radio_set_rx_tx_fallback_mode(ctx, LR11XX_RADIO_FALLBACK_STDBY_RC);

	/* LoRa mode */
	lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_LORA);

	/* Clear any errors accumulated during init (calibration, TCXO, etc.)
	 * and any pending IRQ bits — otherwise CMD_ERROR (bit 22) will fire
	 * DIO1 immediately after we enable it. */
	uint16_t sys_errors = 0;
	lr11xx_system_get_errors(ctx, &sys_errors);
	if (sys_errors) {
		LOG_WRN("System errors at init: 0x%04x — clearing", sys_errors);
	}
	lr11xx_system_clear_errors(ctx);
	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);

	/* Enable DIO1 */
	lr11xx_hal_enable_dio1_irq(&data->hal_ctx);

	/* Set default boost from DTS — actual hardware register write
	 * is deferred to start_rx() where the radio is fully configured
	 * (frequency, modulation, pkt params).  RadioLib/Arduino calls
	 * SetRxBoosted after full configuration.  Calling it here (before
	 * frequency is set) triggers CMD_ERROR on all LR1110 FW versions. */
	data->rx_boost_enabled = cfg->rx_boosted;
	data->rx_boost_applied = false;

	data->hw_initialized = true;
	LOG_INF("LR11xx driver ready");
	return 0;
}

/* ── Wedge-recovery watchdog ────────────────────────────────────────── */

/* Detects the "BUSY stuck high, DIO1 silent" wedge and re-arms RX with a
 * hardware reset.  False-positive free by construction: a healthy chip —
 * continuous RX or autonomous DC cycling — always drops BUSY low within one
 * duty-cycle period, so a *continuous* BUSY-high dwell longer than any
 * legitimate cycle, with no DIO1 event for >12 s, can only be a genuine wedge.
 * Passive: reads the BUSY GPIO only (no SPI), so it cannot itself disturb the
 * chip or race the autonomous DC state machine.  Runs on its own queue at the
 * DIO1 priority; the confirm poll yields every 2 ms so a real packet arriving
 * mid-confirm is still processed promptly on the DIO1 queue. */
static void lr11xx_wedge_watchdog_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct lr11xx_data *data = CONTAINER_OF(dwork, struct lr11xx_data,
						wedge_work);
	const struct lr11xx_config *cfg = data->dev->config;

	/* Only monitor steady RX. */
	if (!data->in_rx_mode || data->tx_active || !data->configured) {
		goto rearm;
	}

	/* Recent DIO1 activity = chip provably alive. */
	if ((k_uptime_get_32() - data->last_dio1_ms) < LR11XX_WEDGE_IDLE_MS) {
		goto rearm;
	}

	/* Idle: either quiet RF (chip still DC-cycling) or a wedge.  Poll BUSY
	 * continuously for one confirm window — a healthy chip shows a low edge
	 * within a cycle; a wedge stays high the whole window. */
	{
		uint32_t start = k_uptime_get_32();
		bool saw_low = false;

		while ((k_uptime_get_32() - start) < LR11XX_WEDGE_CONFIRM_MS) {
			if (!gpio_pin_get_dt(&data->hal_ctx.busy)) {
				saw_low = true;
				break;
			}
			k_msleep(2);
		}
		if (saw_low) {
			goto rearm;   /* alive */
		}
	}

	LOG_ERR("LR11xx wedge (BUSY stuck, DIO1 silent >%ums) — hardware reset",
		LR11XX_WEDGE_IDLE_MS);
	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr11xx_hardware_reset(data, cfg);
	lr11xx_start_rx(data, cfg);
	k_mutex_unlock(&data->spi_mutex);
	data->last_dio1_ms = k_uptime_get_32();

rearm:
	k_work_schedule_for_queue(&data->wedge_wq, &data->wedge_work,
				  K_MSEC(LR11XX_WEDGE_CHECK_MS));
}

/* ── Driver init (lightweight — runs at POST_KERNEL) ────────────────── */

static int lr11xx_lora_init(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;
	const struct lr11xx_config *cfg = dev->config;
	int ret;

	data->dev = dev;
	data->hw_initialized = false;

	k_mutex_init(&data->spi_mutex);
	k_sem_init(&data->cad_sem, 0, 1);
	k_work_init(&data->dio1_work, lr11xx_dio1_work_handler);

	/* Start dedicated DIO1 work queue at high priority */
	k_work_queue_start(&data->dio1_wq, lr11xx_dio1_wq_stack,
			   K_THREAD_STACK_SIZEOF(lr11xx_dio1_wq_stack),
			   K_PRIO_COOP(7),
			   &(const struct k_work_queue_config){ .name = "lr11xx_dio1" });

	/* Wedge-recovery watchdog on its own queue (see handler).  Same priority
	 * as DIO1 so it never preempts RX; its confirm poll yields every 2 ms. */
	k_work_init_delayable(&data->wedge_work, lr11xx_wedge_watchdog_handler);
	k_work_queue_start(&data->wedge_wq, lr11xx_wedge_wq_stack,
			   K_THREAD_STACK_SIZEOF(lr11xx_wedge_wq_stack),
			   K_PRIO_COOP(7),
			   &(const struct k_work_queue_config){ .name = "lr11xx_wedge" });
	data->last_dio1_ms = k_uptime_get_32();
	k_work_schedule_for_queue(&data->wedge_wq, &data->wedge_work,
				  K_MSEC(LR11XX_WEDGE_CHECK_MS));

	/* Check SPI bus */
	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Fill HAL context from DTS config */
	memset(&data->hal_ctx, 0, sizeof(data->hal_ctx));
	data->hal_ctx.spi_dev = cfg->bus.bus;
	data->hal_ctx.spi_cfg = cfg->bus.config;
	/* Override CS — we control NSS manually via gpio_pin_set_dt,
	 * the SPI controller must NOT drive CS. The DTS cs-gpios on
	 * the SPI bus assigns the pin, but our HAL does manual NSS.
	 * Clear BOTH cs_is_gpio AND the port pointer so the SPI framework's
	 * spi_context_cs_control() does not try to drive CS (which would
	 * NULL-deref on the cleared port pointer). */
	data->hal_ctx.spi_cfg.cs.cs_is_gpio = false;
	data->hal_ctx.spi_cfg.cs.gpio.port = NULL;
	data->hal_ctx.nss.port = cfg->bus.config.cs.gpio.port;
	data->hal_ctx.nss.pin = cfg->bus.config.cs.gpio.pin;
	data->hal_ctx.nss.dt_flags = cfg->bus.config.cs.gpio.dt_flags;
	data->hal_ctx.reset = cfg->reset;
	data->hal_ctx.busy = cfg->busy;
	data->hal_ctx.dio1 = cfg->dio1;
	data->hal_ctx.tcxo_voltage_mv = cfg->tcxo_voltage_mv;
	data->hal_ctx.tcxo_startup_us = cfg->tcxo_startup_delay_ms * 1000;
	data->hal_ctx.radio_is_sleeping = false;

	/* Init HAL GPIOs */
	ret = lr11xx_hal_init(&data->hal_ctx);
	if (ret != 0) {
		LOG_ERR("HAL init failed: %d", ret);
		return ret;
	}

	/* Set DIO1 callback — routes through HAL work queue to our handler */
	lr11xx_hal_set_dio1_callback(&data->hal_ctx, lr11xx_dio1_callback,
				     data);

	LOG_INF("LR11xx driver registered (hw init deferred to first config)");
	return 0;
}

/* ── Device instantiation ───────────────────────────────────────────── */

static DEVICE_API(lora, lr11xx_lora_api) = {
	.config     = lr11xx_lora_config,
	.airtime    = lr11xx_lora_airtime,
	.send       = lr11xx_lora_send,
	.send_async = lr11xx_lora_send_async,
	.recv       = lr11xx_lora_recv,
	.recv_async = lr11xx_lora_recv_async,
	.cad        = lr11xx_lora_cad,
	.cad_async  = lr11xx_lora_cad_async,
	/* MODE_RX sniff: same preamble-detect-and-extend (2*rx+sleep) as the
	 * SX126x.  The earlier "broken" verdict was a window-sizing bug
	 * (over-sleep + no header budget), not a chip defect — now sized by
	 * the shared adapter math.  Default-off via prefs; HW-verify on a
	 * live LR1110 before trusting in production. */
	.recv_duty_cycle_async = lr11xx_lora_recv_duty_cycle,
};

#define LR11XX_INIT(n)                                                     \
	static const struct lr11xx_config lr11xx_config_##n = {            \
		.bus = SPI_DT_SPEC_INST_GET(n,                             \
			SPI_WORD_SET(8) | SPI_OP_MODE_CONTROLLER |         \
			SPI_TRANSFER_MSB),                                 \
		.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),            \
		.busy  = GPIO_DT_SPEC_INST_GET(n, busy_gpios),            \
		.dio1  = GPIO_DT_SPEC_INST_GET(n, dio1_gpios),            \
		.tcxo_voltage_mv =                                         \
			DT_INST_PROP_OR(n, tcxo_voltage_mv, 0),           \
		.tcxo_startup_delay_ms =                                   \
			DT_INST_PROP_OR(n, tcxo_startup_delay_ms, 5),     \
		.rx_boosted = DT_INST_PROP(n, rx_boosted),                 \
		.rfswitch_enable  = DT_INST_PROP_OR(n, rfswitch_enable, 0),\
		.rfswitch_standby = DT_INST_PROP_OR(n, rfswitch_standby,0),\
		.rfswitch_rx      = DT_INST_PROP_OR(n, rfswitch_rx, 0),   \
		.rfswitch_tx      = DT_INST_PROP_OR(n, rfswitch_tx, 0),   \
		.rfswitch_tx_hp   = DT_INST_PROP_OR(n, rfswitch_tx_hp, 0),\
		.rfswitch_gnss    = DT_INST_PROP_OR(n, rfswitch_gnss, 0), \
		.pa_hp_sel        = DT_INST_PROP_OR(n, pa_hp_sel, 7),     \
		.pa_duty_cycle    = DT_INST_PROP_OR(n, pa_duty_cycle, 4), \
	};                                                                 \
	static struct lr11xx_data lr11xx_data_##n;                         \
	DEVICE_DT_INST_DEFINE(n, lr11xx_lora_init, NULL,                   \
			      &lr11xx_data_##n, &lr11xx_config_##n,        \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,      \
			      &lr11xx_lora_api);

DT_INST_FOREACH_STATUS_OKAY(LR11XX_INIT)
