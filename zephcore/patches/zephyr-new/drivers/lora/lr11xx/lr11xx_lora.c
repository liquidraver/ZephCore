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
#include <zephyr/drivers/lora/zc_lora_timing.h>
#include "lr11xx_cad_peak.h"
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

/* Below 0x0303 SetLoRaSyncWord is ignored and the radio is deaf to the mesh.
 * Logged, not refused (upstream refuses): a diagnosable radio beats an absent
 * one, and the firmware seen in the field (0x0307, 0x0401) clears it. */
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
	 * window sizing is owned by the adapter layer (LoRaRadio). */
	uint32_t dc_rx_ms;
	uint32_t dc_sleep_ms;

	/* Duty-cycle false-preamble re-arms (UM §7.2.6: the 2*Rx+Sleep window
	 * expired with no packet). `get dc.restarts`. */
	atomic_t dc_timeout_restarts;

	/* CAD state */
	lora_cad_cb cad_cb;
	void *cad_user_data;
	struct k_sem cad_sem;
	int cad_result;
	/* No cad_active flag (the SX126x needs one; this driver does not): the DIO1
	 * handler and every CAD entry point hold spi_mutex, so they cannot
	 * interleave, and lr11xx_do_cad() clears any TIMEOUT latched before it. */
	/* Adaptive-CAD: signed offset applied to the per-SF base detPeak on
	 * every LBT CAD; cad_probe_peak overrides for one calibration probe. */
	int8_t cad_peak_offset;
	uint8_t cad_probe_peak;
	/* CAD_RX probe bookkeeping: cad_exit_rx marks the CAD in flight as a probe
	 * whose positive verdict continues into Rx; cad_rx_state records which
	 * terminal interrupt (packet or timeout) resolved it. */
	bool cad_exit_rx;
	atomic_t cad_rx_state;

	/* Deferred hardware init — heavy SPI/radio work runs on first config() */
	bool hw_initialized;

	/* DIO1 stuck-HIGH detection: counts consecutive empty IRQ cycles.
	 * If DIO1 stays HIGH with no actionable IRQ for too many cycles,
	 * the LR1110 is hung — trigger a hardware reset. */
	int dio1_stuck_count;

	/* First sighting of a latched PREAMBLE_DETECTED by the poll (ms, 0 = none).
	 * Not DIO1-routed and never auto-cleared on a foreign sync word, so it is
	 * released after the SF-aware grace. Under spi_mutex. */
	uint32_t preamble_seen_at_ms;

	/* When SYNC_WORD_HEADER_VALID was seen (ms, 0 = no payload phase). Stamped
	 * by the DIO1 handler (the only RX-busy signal with a duty cycle armed) and
	 * by the poll; released after lr11xx_max_payload_ms(), because continuous
	 * RX has no timer to end a packet that never completes. Writes under
	 * spi_mutex. */
	uint32_t header_seen_at_ms;

	/* Last DIO1 proof of life, for the wedge watchdog (BUSY stuck high, DIO1
	 * silent: a command that raced the DC sleep phase). */
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
		/* HEADER_VALID on DIO1: the only thing that can stamp header_seen_at_ms
		 * while a duty cycle forbids the poll any bus access. Safe to route (at
		 * most once per real packet, after sync word + header CRC), unlike
		 * PREAMBLE_DETECTED; same split as the SX126x. */
		LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID |
		/* CAD_DONE/DETECTED on DIO1, or the blocking LBT CAD never completes (its
		 * semaphore is given by the DIO1 handler). Inert outside a CAD. */
		LR11XX_SYSTEM_IRQ_CAD_DONE | LR11XX_SYSTEM_IRQ_CAD_DETECTED,
		0);
}

/* ── RX duty cycle ──────────────────────────────────────────────────── */

/* SetRxDutyCycle(MODE_RX): a preamble detect extends the window to
 * 2*rx + sleep natively, so no StopTimerOnPreamble and no parked-RX
 * watchdog are needed. Window sizing belongs to the adapter. */

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

	/* RX boost, written once. Not re-applied per duty-cycle re-arm: the LR11xx
	 * retains its configuration across the sleep phase (UM §7.2.6); only the
	 * SX126x needs a retention list. Do not port that fix here by analogy. */
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

	/* LR1110 firmware raises a benign CMD_ERROR on several writes (all FW);
	 * clear it so it does not leak into the DIO1 handler. */
	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);

	data->in_rx_mode = true;
	data->tx_active = false;
}

/* ── Lightweight RX restart (no modem reconfig) ─────────────────────── */

/* Lightweight RX re-arm after RX done / error / timeout: modulation and
 * packet params persist through SetRx (TX->RX takes the full start_rx()). */
/* Returns 0 if the receiver is believed back on air, <0 if SetStandby was
 * rejected (escalate). Not verified after SetRxDutyCycle: that GetStatus
 * NSS edge would end the cycle it checks. */
/* `in_standby`: the chip is already in STDBY_RC (after RX_DONE the loop
 * returns to that fallback), so skip the standby and its probe on the
 * per-packet path. False where the loop may still be running. */
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

		/* SetStandby first: after a header error the cycle is still running, and
		 * an NSS edge into a live cycle is the race UM §7.2.6 warns about (seen on
		 * the LR2021 as CMD_ERROR + DIO1 stuck high). Free when already in standby. */
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

/* The DIO1 handler is one pass over the IRQ word, case by case, in this order.
 * Each case helper runs with spi_mutex held. A helper returns true when it has
 * released the mutex to invoke a callback, and the handler then returns at
 * once; otherwise it reports through *rx_restarted whether it left the receiver
 * running, which the safety net at the end relies on. */

/* ── RX done ──
 * Deliver unless CRC_ERROR, or HEADER_ERROR with no valid header. A header
 * error during a valid header is a foreign signal; keep the packet
 * (RadioLib's rule; dropping these cost ~7-10% under load). */
static bool lr11xx_irq_rx_done(struct lr11xx_data *data, uint32_t irq,
			       bool hdr_valid_seen, bool *rx_restarted)
{
	void *ctx = &data->hal_ctx;

	if (!(irq & LR11XX_SYSTEM_IRQ_RX_DONE) ||
	    (irq & LR11XX_SYSTEM_IRQ_CRC_ERROR) ||
	    ((irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) && !hdr_valid_seen)) {
		return false;
	}

	lr11xx_radio_rx_buffer_status_t rx_stat;
	lr11xx_radio_get_rx_buffer_status(ctx, &rx_stat);

	if (rx_stat.pld_len_in_bytes > 0 &&
	    rx_stat.pld_len_in_bytes <= 255) {
		lr11xx_radio_pkt_status_lora_t pkt_stat;
		lr11xx_radio_get_lora_pkt_status(ctx, &pkt_stat);

		lr11xx_regmem_read_buffer8(ctx, data->rx_buf,
					   rx_stat.buffer_start_pointer,
					   rx_stat.pld_len_in_bytes);

		/* Buffer-shift errata: a coalesced foreign HEADER_ERROR drifts the next
		 * read by +4; standby resets it, after the payload is captured. */
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
		*rx_restarted = true;

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
		return true;
	}

	LOG_WRN("RX: invalid len %d", rx_stat.pld_len_in_bytes);
	lr11xx_restart_rx(data, true);
	*rx_restarted = true;
	return false;
}

/* ── CAD done ── */
static bool lr11xx_irq_cad_done(struct lr11xx_data *data, uint32_t irq,
				bool *rx_restarted)
{
	if (!(irq & LR11XX_SYSTEM_IRQ_CAD_DONE)) {
		return false;
	}

	bool detected = (irq & LR11XX_SYSTEM_IRQ_CAD_DETECTED) != 0;

	/* Positive probe: Rx continues on the detected signal. rx_restarted keeps
	 * the safety net from tearing down the reception being measured. */
	if (data->cad_exit_rx && detected) {
		/* CAD_ONLY left the chip in STBY_RC: arm the follow-on Rx with the normal
		 * SetRx, bounded by the max-payload figure so a terminal IRQ is certain. */
		lr11xx_radio_set_rx_with_timeout_in_rtc_step(
			&data->hal_ctx, lr11xx_cad_rx_timeout_steps(data));
		data->in_rx_mode = true;
		atomic_set(&data->cad_rx_state, LR11XX_CAD_RX_ARMED);
		*rx_restarted = true;
	}

	if (data->cad_cb) {
		lora_cad_cb cb = data->cad_cb;
		void *ud = data->cad_user_data;

		data->cad_cb = NULL;
		data->cad_user_data = NULL;
		k_mutex_unlock(&data->spi_mutex);
		cb(data->dev, detected, ud);
		return true;
	}

	/* Blocking CAD: signal the semaphore */
	data->cad_result = detected ? 1 : 0;
	k_sem_give(&data->cad_sem);
	return false;
}

/* ── TX done ── */
static void lr11xx_irq_tx_done(struct lr11xx_data *data,
			       const struct lr11xx_config *cfg, uint32_t irq,
			       bool *rx_restarted)
{
	if (!(irq & LR11XX_SYSTEM_IRQ_TX_DONE)) {
		return;
	}

	data->tx_active = false;

	/* Full restart — modem was reconfigured for TX */
	lr11xx_start_rx(data, cfg);
	*rx_restarted = true;

	/* Raise TX signal */
	if (data->tx_signal) {
		k_poll_signal_raise(data->tx_signal, 0);
	}
}

/* ── Timeout ── */
static void lr11xx_irq_timeout(struct lr11xx_data *data,
			       const struct lr11xx_config *cfg, uint32_t irq,
			       bool *rx_restarted)
{
	if (!(irq & LR11XX_SYSTEM_IRQ_TIMEOUT)) {
		return;
	}

	/* cad_timeout expired with nothing decoded: the detection had
	 * no packet behind it.  Resolved before the branches below,
	 * which put the receiver back on air. */
	bool was_cad_rx = lr11xx_cad_rx_resolve(data,
						LR11XX_CAD_RX_TMOUT);

	if (data->tx_active) {
		/* Chip TX timeout: TX_DONE will never come. Re-arm RX but do NOT raise
		 * tx_signal (it means "sent"); the adapter's wait thread owns the loss. */
		LOG_ERR("TX timeout — chip stopped the transmission, "
			"packet lost");
		data->tx_active = false;
		lr11xx_start_rx(data, cfg);
		*rx_restarted = true;
		return;
	}

	/* Duty-cycle false preamble (UM §7.2.6): counted for `get dc.restarts`,
	 * except when it is a CAD_RX probe's own timeout. */
	if (data->rx_duty_cycle_enabled && !was_cad_rx) {
		atomic_inc(&data->dc_timeout_restarts);
	}
	if (lr11xx_restart_rx(data, false) < 0) {
		/* Escalate: full restart (standby, modem reprogram, SetRx). If that fails
		 * too, the stuck-DIO1 counter still reaches its hardware reset. */
		LOG_WRN("Timeout: light re-arm failed — full RX restart");
		lr11xx_start_rx(data, cfg);
	}
	*rx_restarted = true;
}

/* ── CRC / header error ──
 * A header error during a valid header with no RX_DONE is a foreign signal
 * over OUR packet in flight: do not abort, its RX_DONE delivers it. */
static bool lr11xx_irq_rx_error(struct lr11xx_data *data, uint32_t irq,
				bool hdr_valid_seen, bool *rx_restarted)
{
	void *ctx = &data->hal_ctx;

	if ((irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) && hdr_valid_seen &&
	    !(irq & LR11XX_SYSTEM_IRQ_RX_DONE)) {
		LOG_DBG("RX: foreign HDR err during valid header — not aborting");
		*rx_restarted = true;  /* reception continues; skip safety-net restart */
		return false;
	}
	if (!(irq & (LR11XX_SYSTEM_IRQ_CRC_ERROR |
		     LR11XX_SYSTEM_IRQ_HEADER_ERROR))) {
		return false;
	}

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
		*rx_restarted = true;
	}

	k_mutex_unlock(&data->spi_mutex);

	/* Notify callback with NULL data for error counting */
	if (data->async_rx_cb) {
		data->async_rx_cb(data->dev, NULL, 0, 0, 0,
				  data->async_rx_user_data);
	}
	return true;
}

/* ── Header valid ──
 * Stamp the payload-phase latch, unless a terminal bit for our packet came
 * in the same pass. rx_restarted: the chip is mid-packet, and the safety
 * net must not restart RX over it. */
static void lr11xx_irq_header_valid(struct lr11xx_data *data, uint32_t irq,
				    bool *rx_restarted)
{
	if (!(irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) ||
	    (irq & (LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR |
		    LR11XX_SYSTEM_IRQ_TIMEOUT))) {
		return;
	}

	uint32_t now = k_uptime_get_32();

	/* 1 as the "set" sentinel if k_uptime is 0 right after boot. */
	data->header_seen_at_ms = (now == 0) ? 1U : now;
	/* The latch is the truth source now; a preamble timestamp left
	 * behind would outlive it and be read as a live grace window. */
	data->preamble_seen_at_ms = 0;
	*rx_restarted = true;
}

/* DIO1 still high after processing: a new IRQ arrived meanwhile, and an
 * edge-triggered pin will not fire again, so resubmit. Five empty passes
 * in a row mean a stuck chip: hardware reset. */
static void lr11xx_dio1_recheck_pin(struct lr11xx_data *data,
				    const struct lr11xx_config *cfg)
{
	if (!gpio_pin_get_dt(&data->hal_ctx.dio1)) {
		data->dio1_stuck_count = 0;
		return;
	}

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
}

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

	/* CMD_ERROR is benign on this firmware; ERROR (bit 23) is a real fault. */
	if (irq & LR11XX_SYSTEM_IRQ_ERROR) {
		LOG_ERR("IRQ hardware ERROR: 0x%08x", irq);
	}

	/* Any valid IRQ clears the stuck counter */
	if (irq != 0) {
		data->dio1_stuck_count = 0;
	}

	/* Valid header seen, live or latched: the bulk clear removes the live bit
	 * mid-packet, so only the latch still knows by RX_DONE. Read before any
	 * branch resets it. */
	bool hdr_valid_seen =
		(irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) != 0 ||
		data->header_seen_at_ms != 0;

	/* Ground truth for a CAD_RX probe: anything that proves a transmitter
	 * was actually there.  A CRC or header error counts as much as a clean
	 * packet -- the probe is asking whether the detection was real, not
	 * whether the packet was usable. */
	if (irq & (LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR |
		   LR11XX_SYSTEM_IRQ_HEADER_ERROR)) {
		lr11xx_cad_rx_resolve(data, LR11XX_CAD_RX_PACKET);
	}

	if (lr11xx_irq_rx_done(data, irq, hdr_valid_seen, &rx_restarted) ||
	    lr11xx_irq_cad_done(data, irq, &rx_restarted)) {
		return;  /* mutex released to deliver a callback */
	}
	lr11xx_irq_tx_done(data, cfg, irq, &rx_restarted);
	lr11xx_irq_timeout(data, cfg, irq, &rx_restarted);
	if (lr11xx_irq_rx_error(data, irq, hdr_valid_seen, &rx_restarted)) {
		return;
	}
	lr11xx_irq_header_valid(data, irq, &rx_restarted);

safety_check:
	/* Safety net: RX expected but nothing re-armed it (SPI failure, CMD_ERROR-
	 * only DIO1, unknown bit). Otherwise the chip sits in STDBY_RC, deaf. */
	if (!rx_restarted && data->in_rx_mode && !data->tx_active) {
		LOG_ERR("DIO1 safety: no IRQ handled (0x%08x rc=%d), "
			"restarting RX", irq, rc);
		/* State genuinely unknown here — an SPI failure reading the IRQ
		 * register or an unhandled bit means we cannot claim the chip
		 * left the loop.  Keep the defensive standby; this is the one
		 * path where it is not redundant. */
		lr11xx_restart_rx(data, false);
	}

	lr11xx_dio1_recheck_pin(data, cfg);

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

	/* LBT: blocking CAD first. On busy, restore RX here before -EBUSY (CAD
	 * leaves standby and clears in_rx_mode, so remember the pre-CAD state). */
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

	/* Chip TX timeout (UM §7.2.3: it stops the transmission) scaled from
	 * airtime +25% +500 ms, floored at the old 10 s, saturated at 24 bits: the
	 * SDK's ms helper overflows above 131 s. */
	data->tx_signal = async;
	data->tx_active = true;
	{
		uint32_t air_ms = lr11xx_lora_airtime(dev, data_len);
		uint32_t tmo_ms = air_ms + (air_ms / 4U) + 500U;
		uint32_t steps;

		if (tmo_ms < 10000U) {
			tmo_ms = 10000U;
		}
		steps = zc_lora_ms_to_steps24(tmo_ms, 32768U);
		LOG_DBG("SET_TX: airtime=%u ms, timeout=%u ms (%u steps)",
			air_ms, tmo_ms, steps);
		lr11xx_radio_set_tx_with_timeout_in_rtc_step(ctx, steps);
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

	/* Explicit timing only — the adapter (LoRaRadio) owns the window
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
 * Any NSS edge during the sleep phase silently ends the RxDutyCycle loop
 * (UM §7.2.6), and BUSY cannot tell sleep from ordinary Rx on this family.
 * So a caller that must talk to the chip brackets its work: suspend ends
 * the cycle with SetStandby, resume re-arms it. Caller holds spi_mutex.
 * Rationale and measurements: devdocs LLD 04 §13. */
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

/* Is a duty-cycled reception under way? Ask before dc_suspend(), which
 * would end it; with a cycle armed is_receiving() cannot see the preamble
 * phase. Reading IRQs is safe in both phases. A preamble past its grace
 * with no header is stale. Caller holds spi_mutex. */
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

/* Settle after Rx entry before the first GetRssiInst: 16 RSSI averaging
 * windows (DS Table 13-82), the same model as the C++ sampler, floored at
 * the old 1 ms. */
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
	 * is the sentinel LoRaRadio::triggerNoiseFloorCalibrate expects. */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return -128;
	}
	if (lr11xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return -128;
	}

	bool armed = lr11xx_dc_suspend(data);

	if (armed) {
		/* The stand-down left the chip in standby: enter continuous Rx for the
		 * reading (after the front-end settle), then hand the cycle back. */
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
	return zc_lora_preamble_grace_ms(
		(uint8_t)data->modem_cfg.datarate,
		(uint32_t)(bw_enum_to_khz(data->modem_cfg.bandwidth) * 1000.0f),
		data->modem_cfg.preamble_len);
}

/* Payload-phase bound for the header latch (zc_lora_timing.h). */
static uint32_t lr11xx_max_payload_ms(struct lr11xx_data *data)
{
	return zc_lora_max_payload_ms(
		(uint8_t)data->modem_cfg.datarate,
		(uint32_t)(bw_enum_to_khz(data->modem_cfg.bandwidth) * 1000.0f));
}

/* CAD_RX Rx bound in RTC steps (32768 Hz), 64-bit and saturated: the SDK
 * helper overflows above 131 s, which SF12/BW7.81 (~273 s) exceeds. */
static uint32_t lr11xx_cad_rx_timeout_steps(struct lr11xx_data *data)
{
	return zc_lora_ms_to_steps24(lr11xx_max_payload_ms(data), 32768U);
}

int lr11xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			 uint32_t spacing_us)
{
	struct lr11xx_data *data = dev->data;
	int got = 0;

	/* One stand-down for the whole burst: a per-sample read cost eight cycle
	 * tear-downs, latch wipes and settles per sampling interval. */
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
		/* Clear the latched reception bits so the check after the loop sees only
		 * what arrived inside the window (the cycle is already down). */
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

	/* A reception inside the window contaminates the samples: report -EAGAIN.
	 * Only this bracket can see it; dc_resume() zeroes the latch before the
	 * caller's own isReceiving() re-check runs. */
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

	/* Payload phase, from the DIO1-stamped latch: the only answer with a cycle
	 * armed. Never brackets (that would end the reception asked about) and
	 * never reads BUSY (high in sleep and in ordinary Rx alike). */
	uint32_t hdr_seen = data->header_seen_at_ms;

	if (hdr_seen != 0 &&
	    (k_uptime_get_32() - hdr_seen) < lr11xx_max_payload_ms(data)) {
		return true;
	}

	if (data->rx_duty_cycle_enabled) {
		/* No latch, or its deadline blew: no bus access allowed here. Drop a stale
		 * latch if the mutex is free; otherwise the DIO1 handler owns it. */
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

		/* The poll can see the bit before the work item runs, so it may stamp. A
		 * live latch returned true above, so one set here has blown its deadline. */
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

/* ── Extension API: receiver hygiene ─────────────────────────────────
 *
 * Warm sleep, then recalibrate (as Arduino's lr11x0ResetAGC()).
 * calibrate(0x3F) here includes image rejection and reverts it to
 * 902-928 MHz, so the image cal is re-issued afterwards. */
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

	/* Settle before handing the chip back: wait_on_busy() can miss a BUSY that
	 * has not risen yet, and a SetRxDutyCycle into a calibrating chip is
	 * dropped (deaf until the next reset). */
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

/* Redo the frequency-dependent calibrations after temperature drift. Not
 * an AGC reset: that is an SX126x remedy, and on this part it cost packets.
 * Leaves the driver out of RX; the caller must startReceive(). */
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

/* ── Driver API: CAD ────────────────────────────────────────────────── */

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
		/* Per-CAD exit: the probe continues into Rx on a detection, LBT does not.
		 * Detection itself is identical. cad_timeout is in 32768 Hz RTC steps
		 * (the SDK prose says 31.25 us; it is wrong). */
		/* Always CAD_ONLY on the wire: the chip's own CAD_RX exit leaves it in Rx
		 * but deaf (14% of packets lost, 2026-09-02). The DIO1 handler arms the
		 * follow-on Rx with a plain SetRx instead, as on the SX126x. */
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

/* Wedge watchdog: BUSY continuously high past any legitimate DC cycle with
 * no DIO1 for 12 s can only be a wedge; hardware reset and re-arm. Reads
 * the BUSY GPIO only, so it cannot disturb the chip. */
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
	/* The HAL drives NSS by hand: clear both cs_is_gpio and the port so
	 * spi_context_cs_control() leaves CS alone. */
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
		.dio1  = GPIO_DT_SPEC_INST_GET(n, irq_gpios),            \
		.tcxo_voltage_mv =                                         \
			DT_INST_PROP_OR(n, tcxo_voltage, 0),           \
		.tcxo_startup_delay_ms =                                   \
			DT_INST_PROP_OR(n, tcxo_power_startup_delay_ms, 5),     \
		.rx_boosted = DT_INST_PROP(n, rx_boosted),                 \
		.rfswitch_enable  = DT_INST_PROP_OR(n, rfsw_enable, 0),\
		.rfswitch_standby = DT_INST_PROP_OR(n, rfsw_standby,0),\
		.rfswitch_rx      = DT_INST_PROP_OR(n, rfsw_rx, 0),   \
		.rfswitch_tx      = DT_INST_PROP_OR(n, rfsw_tx, 0),   \
		.rfswitch_tx_hp   = DT_INST_PROP_OR(n, rfsw_tx_hp, 0),\
		.rfswitch_gnss    = DT_INST_PROP_OR(n, rfsw_gnss, 0), \
		.pa_hp_sel        = DT_INST_PROP_OR(n, pa_hp_sel, 7),     \
		.pa_duty_cycle    = DT_INST_PROP_OR(n, pa_duty_cycle, 4), \
	};                                                                 \
	static struct lr11xx_data lr11xx_data_##n;                         \
	DEVICE_DT_INST_DEFINE(n, lr11xx_lora_init, NULL,                   \
			      &lr11xx_data_##n, &lr11xx_config_##n,        \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,      \
			      &lr11xx_lora_api);

DT_INST_FOREACH_STATUS_OKAY(LR11XX_INIT)
