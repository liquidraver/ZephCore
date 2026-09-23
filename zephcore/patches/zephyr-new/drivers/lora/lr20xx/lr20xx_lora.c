/*
 * SPDX-License-Identifier: MIT
 * LR20xx Zephyr LoRa driver
 *
 * Implements the standard Zephyr lora_driver_api using the Semtech lr20xx_driver
 * SDK. All SPI access, DIO1 IRQ handling, and radio state management is internal.
 */

#define DT_DRV_COMPAT semtech_lr2021

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include <zephyr/drivers/lora/lr20xx_lora.h>
#include "lr20xx_hal_zephyr.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_common_types.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_lora_types.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_system.h"
#include "lr20xx_system_types.h"
#include "lr20xx_workarounds.h"
#include "lr20xx_regmem.h"
#include "lr20xx_patch.h"
/* Defines the PRAM image itself (pram_lr2021 / pram_lr2021_size).  In C these
 * are const objects at file scope and therefore have external linkage, so this
 * header must be included from exactly one translation unit — this one. */
#include "lr20xx_pram_lr2021.h"

LOG_MODULE_REGISTER(lr20xx_lora, CONFIG_LORA_LOG_LEVEL);

/* Dedicated DIO1 work queue — keeps LoRa interrupt processing off the system
 * work queue.  4 KB (siblings use 2560): lr20xx_hardware_reset() runs from this
 * handler and carries the PRAM load, which they have no equivalent of. */
#define LR20XX_DIO1_WQ_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(lr20xx_dio1_wq_stack, LR20XX_DIO1_WQ_STACK_SIZE);

/* Hardware limit on concurrent LoRa side detectors (ConfigureSideDetectors
 * takes n in [0:3]). */
#define LR20XX_MAX_SIDE_DETECTORS 3

/* ── Driver data structures ─────────────────────────────────────────── */

struct lr20xx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec dio1;
	uint16_t tcxo_voltage_mv;
	uint32_t tcxo_startup_delay_ms;
	bool rx_boosted;
	/* Chip-side DIO wired to the MCU IRQ line (5..11) */
	uint8_t irq_dio;
	/* RF switch DIO bitmasks (bit 0 = DIO5, bit 1 = DIO6, ...) */
	uint8_t rfswitch_enable;
	uint8_t rfswitch_standby;
	/* RX masks are split per band: a front end that shuts its sub-GHz PA
	 * down during 2.4 GHz RX (Meshnology W12 / GC1109 CSD) needs the two
	 * to differ.  Boards that set only the combined `rfswitch-rx` get the
	 * same mask in both, which is the pre-split behaviour. */
	uint8_t rfswitch_rx_lf;
	uint8_t rfswitch_rx_hf;
	uint8_t rfswitch_tx;
	uint8_t rfswitch_tx_hp;
};

struct lr20xx_data {
	const struct device *dev;
	struct lr20xx_hal_context hal_ctx;
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

	/* Extension features */
	bool rx_duty_cycle_enabled;
	bool rx_boost_enabled;
	bool rx_boost_applied;


	/* Duty-cycle re-arms triggered by a timeout, i.e. the false-preamble
	 * case: the preamble-restarted rx_max_time + cycle_time window expired
	 * with no packet, so the chip left the loop and the host put it back.
	 * The rate is the instrument for duty-cycle tuning — a climbing count
	 * means the window is catching noise, not packets.  Exposed as
	 * `get dc.restarts`, which reported a hardcoded 0 on this radio until
	 * nothing counted them. */
	atomic_t dc_timeout_restarts;

	/* Stored duty-cycle timing from recv_duty_cycle() — re-arm paths
	 * must reuse these exact values, never recompute: the window sizing
	 * (detection budget, datasheet completion rule, wake transition) is
	 * owned by the adapter layer. */
	uint32_t dc_rx_ms;
	uint32_t dc_sleep_ms;

	/* CAD state */
	lora_cad_cb cad_cb;
	void *cad_user_data;
	struct k_sem cad_sem;
	int cad_result;	/* 0=free, 1=busy, <0=error */
	/* No cad_active flag: there was one, written at five sites and read at
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
	 * discarded by the clear inside lr20xx_do_cad(). */
	/* Adaptive-CAD: signed offset applied to the per-SF base detPeak on
	 * every LBT CAD; cad_probe_peak overrides for one calibration probe. */
	int8_t cad_peak_offset;
	uint8_t cad_probe_peak;
	/* Adaptive-CAD probe ground truth.  The chip's own CAD_RX exit is NOT
	 * used (its cad_timeout is 24 bits of 32 MHz periods = 524 ms, well
	 * under a 1704 ms max-length packet).  Instead a probe runs CAD_ONLY
	 * and this driver arms the follow-on Rx itself with a normal SetRx,
	 * whose timeout is 24 bits of RTC steps = 512 s -- so the ceiling that
	 * once excluded this family from probing does not apply.  Same remedy
	 * as the SX126x and LR11xx, both of which had their chip CAD_RX exits
	 * fail differently on hardware. */
	bool cad_probe_rx;
	atomic_t cad_rx_state;

	/* CAD_DONE -> carrier-up latency (k_cycle_get_32 units); 0 = no CAD
	 * preceded this transmit.  Only reached on the >524 ms fallback path. */
	uint32_t cad_done_cycles;

	/* CAD_LBT arming, consumed and cleared by lr20xx_do_cad() so an ordinary
	 * adaptive-CAD probe can never inherit exit-to-TX and transmit. */
	bool cad_lbt_exit_tx;
	uint32_t cad_lbt_tx_timeout;   /* 32 MHz periods, <= 0x00FFFFFF */

	/* Deferred hardware init */
	bool hw_initialized;

	/* Chip-side DIO carrying the IRQ line, cached from the devicetree at
	 * hw init.  Reporting only — so the debug dump can name the pin this
	 * board actually wired instead of assuming one. */
	uint8_t irq_dio;

	/* DIO1 stuck-HIGH detection */
	int dio1_stuck_count;
	bool tcxo_disabled;   /* set when the TCXO fallback has already fired */

	/* RX-busy tracking for lr20xx_is_receiving().  Both bits are latched
	 * (DS 5.7) and read non-destructively — continuous RX has no timeout, so
	 * each needs a software release or a noise preamble pins the TX gate
	 * forever.  All writes under spi_mutex.
	 *
	 * PREAMBLE_DETECTED is kept off DIO1 (it fires on noise) and is
	 * poll-only.  SYNC_WORD_HEADER_VALID is routed to DIO1 and stamps
	 * header_seen_at_ms from the work handler: with a duty cycle armed
	 * is_receiving() may not touch the bus, so that timestamp is the ONLY
	 * thing it can answer from, and while nothing wrote it the RX-busy gate
	 * was dead on every duty-cycled node. */
	uint32_t preamble_seen_at_ms;

	uint32_t header_seen_at_ms;

	/* LoRa side detectors — up to 3 extra spreading factors demodulated
	 * concurrently with the main one, on the same bandwidth.  Stored here
	 * because the chip forgets them on every SetModulationParams, so they
	 * have to be re-issued after each apply_modem_config, and because CAD
	 * has to switch them off (see lr20xx_apply_side_detectors).
	 * side_det_num == 0 means the feature is off. */
	uint8_t side_det_sf[LR20XX_MAX_SIDE_DETECTORS];
	uint8_t side_det_num;
	bool side_det_applied;   /* currently programmed into the chip */

	/* Carrier frequency error, accumulated from received packets.  Only the
	 * mean over many peers says anything about our own reference; theirs
	 * cancel, ours does not.  Diagnostic only — nothing acts on it. */
	int32_t freq_off_last_hz;
	int32_t freq_off_min_hz;
	int32_t freq_off_max_hz;
	int64_t freq_off_sum_hz;
	uint32_t freq_off_count;

	/* RX data buffer */
	uint8_t rx_buf[256];
};

/* ── Debug: dump full chip state (log builds only) ──────────────────── */

#if IS_ENABLED(CONFIG_LOG)
/* One line of chip state.  Takes the whole driver instance: the old signature
 * took a command context and a HAL context separately, and every one of the
 * nine call sites passed the same object twice. */
static void dump_chip_state(struct lr20xx_data *data, const char *label)
{
	struct lr20xx_hal_context *hal = &data->hal_ctx;
	void *ctx = hal;
	lr20xx_system_stat1_t s1 = {0};
	lr20xx_system_stat2_t s2 = {0};
	lr20xx_system_irq_mask_t irq = 0;
	lr20xx_system_errors_t err = 0;

	/* Sample the pins BEFORE issuing any SPI.  DS §5: "The BUSY pin is
	 * automatically asserted on the falling edge of the NSS" — read after
	 * the two commands below and this dump reports its own footprint, not
	 * the chip's state.  Every BUSY=1 in the X1 bring-up logs came from
	 * that, and it cost a whole (wrong) conclusion about this chip holding
	 * BUSY high during Rx.  It does not: "In Rx mode, BUSY goes low as soon
	 * as the chip is ready to receive data." */
	int busy = gpio_pin_get_dt(&hal->busy);
	int dio = gpio_pin_get_dt(&hal->dio1);

	lr20xx_system_get_status(ctx, &s1, &s2, &irq);
	lr20xx_system_get_errors(ctx, &err);

	/* Name the DIO the board actually uses.  This was hardcoded "DIO9",
	 * which is right on promicro_lr2021 and wrong on meshtracker_x1 (DIO8,
	 * per irq-dio in its DTS and the Seeed block diagram) — a label that is
	 * correct on one board and silently lying on another is worse than one
	 * that is obviously generic. */
	LOG_INF("[%s] cmd=%d mode=%d err=0x%04x irq=0x%08x BUSY=%d DIO%u=%d",
		label, s1.command_status, s2.chip_mode, err, irq, busy,
		(unsigned)data->irq_dio, dio);
}
#define DUMP_CHIP_STATE(data, label) dump_chip_state(data, label)
#else
#define DUMP_CHIP_STATE(data, label) do { } while (0)
#endif /* IS_ENABLED(CONFIG_LOG) */

/* ── Helpers ────────────────────────────────────────────────────────── */

/* Reset all software state that says "we are currently receiving": the
 * preamble-grace timestamp and the payload-phase deadline.  Paired write so the
 * two never drift out of sync — same shape as lr11xx_reset_rx_busy_signals()
 * and sx126x_reset_rx_busy_signals().  Called from every RX (re)start site and
 * before TX / CAD entry. */
static inline void lr20xx_reset_rx_busy_signals(struct lr20xx_data *data)
{
	data->preamble_seen_at_ms = 0;
	data->header_seen_at_ms = 0;
}

static lr20xx_radio_lora_bw_t bw_enum_to_lr20xx(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_31_KHZ:  return LR20XX_RADIO_LORA_BW_31;
	case BW_41_KHZ:  return LR20XX_RADIO_LORA_BW_41;
	case BW_62_KHZ:  return LR20XX_RADIO_LORA_BW_62;
	case BW_125_KHZ: return LR20XX_RADIO_LORA_BW_125;
	case BW_250_KHZ: return LR20XX_RADIO_LORA_BW_250;
	case BW_500_KHZ: return LR20XX_RADIO_LORA_BW_500;
	/* The wide set. Nothing in the sub-GHz plans reaches these, but they are
	 * the ordinary bandwidths for 2.4 GHz LoRa — 203/406/812 are the
	 * SX128x-compatible steps, which is what other 2.4 GHz gear speaks. */
	case BW_200_KHZ:  return LR20XX_RADIO_LORA_BW_203;
	case BW_400_KHZ:  return LR20XX_RADIO_LORA_BW_406;
	case BW_800_KHZ:  return LR20XX_RADIO_LORA_BW_812;
	case BW_1000_KHZ: return LR20XX_RADIO_LORA_BW_1000;
	default:         return LR20XX_RADIO_LORA_BW_125;
	}
}

static lr20xx_radio_lora_cr_t cr_enum_to_lr20xx(enum lora_coding_rate cr)
{
	switch (cr) {
	case CR_4_5: return LR20XX_RADIO_LORA_CR_4_5;
	case CR_4_6: return LR20XX_RADIO_LORA_CR_4_6;
	case CR_4_7: return LR20XX_RADIO_LORA_CR_4_7;
	case CR_4_8: return LR20XX_RADIO_LORA_CR_4_8;
	default:     return LR20XX_RADIO_LORA_CR_4_8;
	}
}

/* SetTcxoMode's start_time is the deadline by which the 32 MHz oscillator must
 * be detected, counted in 32 MHz clock periods (datasheet S6.11.3) — NOT in
 * 32.768 kHz RTC ticks, which is what every other timeout on this chip uses.
 * Getting that wrong turns a 5 ms allowance into 5 us, no TCXO starts that
 * fast, and the chip raises HF_XOSC_START_ERR exactly as documented. */
static inline uint32_t tcxo_start_time_periods(uint32_t delay_ms)
{
	return delay_ms * 32000U;   /* 32000 periods per ms at 32 MHz */
}

static lr20xx_system_tcxo_supply_voltage_t get_tcxo_voltage(uint16_t mv)
{
	if (mv >= 3300) return LR20XX_SYSTEM_TCXO_CTRL_3_3V;
	if (mv >= 3000) return LR20XX_SYSTEM_TCXO_CTRL_3_0V;
	if (mv >= 2700) return LR20XX_SYSTEM_TCXO_CTRL_2_7V;
	if (mv >= 2400) return LR20XX_SYSTEM_TCXO_CTRL_2_4V;
	if (mv >= 2200) return LR20XX_SYSTEM_TCXO_CTRL_2_2V;
	if (mv >= 1800) return LR20XX_SYSTEM_TCXO_CTRL_1_8V;
	/* DS Table 6-65 value 0x01.  No board declares 1700 mV today — this is
	 * future-proofing, not a fix: without it such a board would silently be
	 * supplied 1.6 V. */
	if (mv >= 1700) return LR20XX_SYSTEM_TCXO_CTRL_1_7V;
	return LR20XX_SYSTEM_TCXO_CTRL_1_6V;
}

/* Get kHz value from Zephyr BW enum — used for LDRO/PPM calculation */
static float bw_enum_to_khz(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_7_KHZ:   return 7.81f;
	case BW_10_KHZ:  return 10.42f;
	case BW_15_KHZ:  return 15.63f;
	case BW_20_KHZ:  return 20.83f;
	case BW_31_KHZ:  return 31.25f;
	case BW_41_KHZ:  return 41.67f;
	case BW_62_KHZ:  return 62.5f;
	case BW_125_KHZ: return 125.0f;
	case BW_250_KHZ: return 250.0f;
	case BW_500_KHZ: return 500.0f;
	/* Real chip bandwidths, not the enum's round names — these feed the LDRO
	 * symbol-time test and the airtime maths, so they have to be the values
	 * the modem actually runs at (DS Table: 203, 406, 812, 1000 kHz). */
	case BW_200_KHZ:  return 203.0f;
	case BW_400_KHZ:  return 406.0f;
	case BW_800_KHZ:  return 812.0f;
	case BW_1000_KHZ: return 1000.0f;
	default:         return 125.0f;
	}
}

/* LDRO rule: symbol time > 16.38 ms, NOT the SF-based rule DS 9.9.1
 * recommends.  Deliberate — LDRO must match between transmitter and receiver,
 * and the rest of the mesh (SX126x, RadioLib) uses symbol time.  One copy so
 * apply_modem_config, apply_side_detectors and airtime cannot disagree. */
static inline bool lr20xx_ldro_enabled(uint8_t sf, uint32_t bw_hz)
{
	if (bw_hz == 0U) {
		return false;
	}
	/* SF12 is the worst case: 4096 * 1e6 fits in uint32_t. */
	return (((1U << sf) * 1000000U) / bw_hz) > 16380U;
}

/* The CAD symbol count actually programmed.  LoRaRadioBase asks for
 * LORA_CAD_SYMB_4 via mc->cad.symbol_num; the 4 here is the fallback when it is
 * left zero.  Single source so the CAD window, its detPeak (DS Table 6-19 is
 * 2-D over both) and the blocking-wait budget can never disagree. */
static inline uint8_t lr20xx_cad_symb_nb(const struct lora_modem_config *mc)
{
	return mc->cad.symbol_num ? (uint8_t)mc->cad.symbol_num : 4;
}

/* ── IRQ DIO ────────────────────────────────────────────────────────── */

static inline lr20xx_system_dio_t lr20xx_irq_dio(const struct lr20xx_config *cfg)
{
	return (lr20xx_system_dio_t)cfg->irq_dio;
}

/* DIO5 can only be pulled up; every other DIO gets a pull-down. */
static inline lr20xx_system_dio_drive_t
lr20xx_irq_dio_pull(const struct lr20xx_config *cfg)
{
	return cfg->irq_dio == LR20XX_SYSTEM_DIO_5
		       ? LR20XX_SYSTEM_DIO_DRIVE_PULL_UP
		       : LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN;
}

/* ── Configure RF switch DIOs ───────────────────────────────────────── */

/* DIO5..DIO11 inclusive — the full RF-switch-capable range on this chip.
 * Bit i of every rfswitch-* mask is DIO(5 + i), so the masks stay uint8_t. */
#define LR20XX_RFSWITCH_DIO_COUNT \
	(LR20XX_SYSTEM_DIO_11 - LR20XX_SYSTEM_DIO_5 + 1)

static void lr20xx_configure_rfswitch(void *ctx, const struct lr20xx_config *cfg)
{
	/* LR20xx RF switch uses per-DIO configuration.
	 * DIO5..DIO11 map to enable bitmask bits 0..6 (the chip has no RF
	 * switch DIOs outside 5..11 — see lr20xx_system_dio_t).
	 * For each enabled DIO, compute which operational modes
	 * should drive it HIGH by looking at the per-mode bitmasks.
	 *
	 * This used to stop at bit 3 (DIO5..DIO8), which silently dropped the
	 * top three DIOs from any board that used them: the enable bit was set
	 * in devicetree, the loop never reached it, so the DIO kept its default
	 * function and the front end it drives never switched.  The Meshnology
	 * W12 puts its entire sub-GHz front end there (DIO9 = GC1109 CTX,
	 * DIO10 = CPS, DIO11 = CSD), and with those three left unconfigured the
	 * FEM stays in the shutdown its 10k pull-downs assert at reset — the
	 * node is both deaf and mute while every register readback looks fine. */
	for (int i = 0; i < LR20XX_RFSWITCH_DIO_COUNT; i++) {
		if (!(cfg->rfswitch_enable & BIT(i))) {
			continue;
		}

		lr20xx_system_dio_t dio = (lr20xx_system_dio_t)(LR20XX_SYSTEM_DIO_5 + i);

		/* Never repurpose the interrupt line.  A board that sets the IRQ
		 * DIO's bit in rfswitch-enable by mistake would otherwise have its
		 * only IRQ pin switched to RF-switch duty here, and the driver
		 * would then wait forever on TX_DONE/RX_DONE with no way to tell
		 * why.  Skipping it turns a silent hang into a warning. */
		if (dio == lr20xx_irq_dio(cfg)) {
			LOG_WRN("rfswitch: DIO%d is the IRQ line, not configuring "
				"it as an RF switch", (int)dio);
			continue;
		}

		/* Set this DIO function to RF switch control */
		lr20xx_system_set_dio_function(ctx, dio,
					       LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
					       LR20XX_SYSTEM_DIO_DRIVE_NONE);

		/* Build the per-DIO mode bitmask:
		 * which operational modes drive this DIO HIGH */
		lr20xx_system_dio_rf_switch_cfg_t sw_cfg = 0;

		if (cfg->rfswitch_standby & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_STANDBY;
		}
		if (cfg->rfswitch_rx_lf & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF;
		}
		if (cfg->rfswitch_rx_hf & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF;
		}
		if (cfg->rfswitch_tx & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF;
		}
		if (cfg->rfswitch_tx_hp & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF;
		}

		lr20xx_system_set_dio_rf_switch_cfg(ctx, dio, sw_cfg);
	}
}

/* ── PA power lookup table (LF / sub-GHz) ─────────────────────────────
 *
 * Semtech's LR20XX_PA_LF_CFG_TABLE (examples/radio_hal/lr20xx_pa_pwr_cfg.h,
 * Clear BSD), indexed -10..+22 dBm.  half_power is in HALF-dBm (DS Table 7-20;
 * the SDK parameter is named power_half_dbm).  The three fields are a matched
 * triple per power level — do not override one from devicetree.
 * See ARCHITECTURE.md 5.5.1. */
struct lr20xx_pa_pwr_entry {
	int8_t  half_power;      /* -> SetTxParams, in half-dBm */
	uint8_t pa_duty_cycle;   /* -> SetPaConfig pa_lf_duty_cycle */
	uint8_t pa_lf_slices;    /* -> SetPaConfig pa_lf_slices */
};

#define LR20XX_LF_MIN_PWR (-10)
#define LR20XX_LF_MAX_PWR 22

static const struct lr20xx_pa_pwr_entry pa_lf_table[] = {
	{ -18, 3, 6 }, /* -10 dBm */
	{ -13, 2, 5 }, /*  -9 dBm */
	{ -13, 6, 1 }, /*  -8 dBm */
	{  -6, 6, 0 }, /*  -7 dBm */
	{   4, 1, 0 }, /*  -6 dBm */
	{   4, 2, 0 }, /*  -5 dBm */
	{   2, 1, 3 }, /*  -4 dBm */
	{  14, 0, 0 }, /*  -3 dBm */
	{   9, 0, 3 }, /*  -2 dBm */
	{  11, 3, 0 }, /*  -1 dBm */
	{  16, 1, 0 }, /*   0 dBm */
	{  11, 7, 0 }, /*   1 dBm */
	{  18, 2, 0 }, /*   2 dBm */
	{  16, 5, 0 }, /*   3 dBm */
	{  17, 7, 0 }, /*   4 dBm */
	{  21, 1, 2 }, /*   5 dBm */
	{  25, 3, 0 }, /*   6 dBm */
	{  32, 0, 1 }, /*   7 dBm */
	{  32, 2, 0 }, /*   8 dBm */
	{  27, 3, 1 }, /*   9 dBm */
	{  32, 2, 1 }, /*  10 dBm */
	{  28, 5, 1 }, /*  11 dBm */
	{  30, 5, 1 }, /*  12 dBm */
	{  34, 4, 1 }, /*  13 dBm */
	{  31, 5, 4 }, /*  14 dBm */
	{  34, 4, 4 }, /*  15 dBm */
	{  34, 5, 6 }, /*  16 dBm */
	{  39, 3, 5 }, /*  17 dBm */
	{  37, 6, 6 }, /*  18 dBm */
	{  40, 5, 5 }, /*  19 dBm */
	{  41, 7, 4 }, /*  20 dBm */
	{  43, 7, 4 }, /*  21 dBm */
	{  44, 7, 7 }, /*  22 dBm */
};

BUILD_ASSERT(ARRAY_SIZE(pa_lf_table) ==
		     (size_t)(LR20XX_LF_MAX_PWR - LR20XX_LF_MIN_PWR + 1),
	     "PA LF table must span [LR20XX_LF_MIN_PWR, LR20XX_LF_MAX_PWR] "
	     "exactly — an off-by-one here silently mis-sets every power level");

/* ── PA power lookup table (HF / 2.4 GHz + S-band) ────────────────────
 *
 * Semtech's LR20XX_PA_HF_CFG_TABLE from the same header, indexed -17..+12 dBm.
 * Transcribed mechanically, and cross-checked against the independently
 * published `tx-power-cfg-hf` in Semtech's Wio-LR2021 shield overlay
 * (usp_zephyr, measured @2445 MHz) — the two agree entry for entry.
 *
 * Field reuse is Semtech's, not ours: on the HF path `pa_duty_cycle` goes to
 * pa_hf_duty_cycle, and `pa_lf_slices` carries the LF PA's "unused" default of
 * 7 (DS §7.4.1). pa_lf_duty_cycle takes its own unused default of 6. */
#define LR20XX_HF_MIN_PWR (-17)
#define LR20XX_HF_MAX_PWR 12

static const struct lr20xx_pa_pwr_entry pa_hf_table[] = {
	{  -39, 29, 7 }, /* -17 dBm */
	{  -39, 16, 7 }, /* -16 dBm */
	{  -35, 19, 7 }, /* -15 dBm */
	{  -32, 19, 7 }, /* -14 dBm */
	{  -29, 19, 7 }, /* -13 dBm */
	{  -27, 16, 7 }, /* -12 dBm */
	{  -24, 17, 7 }, /* -11 dBm */
	{  -22, 16, 7 }, /* -10 dBm */
	{  -19, 18, 7 }, /*  -9 dBm */
	{  -17, 16, 7 }, /*  -8 dBm */
	{  -14, 21, 7 }, /*  -7 dBm */
	{  -12, 18, 7 }, /*  -6 dBm */
	{   -7, 30, 7 }, /*  -5 dBm */
	{   -8, 16, 7 }, /*  -4 dBm */
	{   -5, 24, 7 }, /*  -3 dBm */
	{   -2, 27, 7 }, /*  -2 dBm */
	{    1, 29, 7 }, /*  -1 dBm */
	{    4, 30, 7 }, /*   0 dBm */
	{    6, 30, 7 }, /*   1 dBm */
	{    7, 28, 7 }, /*   2 dBm */
	{    8, 25, 7 }, /*   3 dBm */
	{   10, 25, 7 }, /*   4 dBm */
	{   15, 31, 7 }, /*   5 dBm */
	{   16, 30, 7 }, /*   6 dBm */
	{   18, 30, 7 }, /*   7 dBm */
	{   21, 31, 7 }, /*   8 dBm */
	{   22, 30, 7 }, /*   9 dBm */
	{   24, 30, 7 }, /*  10 dBm */
	{   24, 26, 7 }, /*  11 dBm */
	{   24, 16, 7 }, /*  12 dBm */
};

BUILD_ASSERT(ARRAY_SIZE(pa_hf_table) ==
		     (size_t)(LR20XX_HF_MAX_PWR - LR20XX_HF_MIN_PWR + 1),
	     "PA HF table must span [LR20XX_HF_MIN_PWR, LR20XX_HF_MAX_PWR] "
	     "exactly — an off-by-one here silently mis-sets every power level");

/* DS §7.4.1: "Only values from 16-31 are authorized… If the HF PA is not used,
 * set the parameter to 16 (default)."  Semtech's BSP writes 16 here too. */
#define LR20XX_PA_HF_DUTY_CYCLE_UNUSED 16

/* Unused-LF defaults, for when the HF PA is the one driving.  Same source. */
#define LR20XX_PA_LF_DUTY_CYCLE_UNUSED 6
#define LR20XX_PA_LF_SLICES_UNUSED     7

/* Which of the chip's two RF paths a frequency belongs to.
 *
 * 1.5 GHz is Semtech's own split, taken from ral_lr20xx_bsp.c, which uses the
 * identical test in get_tx_cfg and get_rx_cfg.  It sits in the dead zone
 * between the LF path (150-960 MHz) and the HF path (1.9-2.5 GHz), so the exact
 * value never has to be argued about — anything a board can legally tune to
 * lands unambiguously on one side. */
#define LR20XX_HF_BAND_THRESHOLD_HZ 1500000000U

static inline bool lr20xx_freq_is_hf(uint32_t freq_hz)
{
	return freq_hz >= LR20XX_HF_BAND_THRESHOLD_HZ;
}

static inline lr20xx_radio_common_rx_path_t lr20xx_rx_path_for(uint32_t freq_hz)
{
	return lr20xx_freq_is_hf(freq_hz) ? LR20XX_RADIO_COMMON_RX_PATH_HF
					  : LR20XX_RADIO_COMMON_RX_PATH_LF;
}

/* Build the PA configuration for a power level on whichever path the frequency
 * selects.  Mirrors ral_lr20xx_bsp_get_tx_cfg(): band first, then clamp to that
 * band's own limits, then index that band's own table.
 *
 * The clamp matters as much as the table.  The two paths have different
 * ceilings — +22 dBm on LF, +12 dBm on HF — so a node carrying the sub-GHz
 * default of 22 into the 2.4 GHz band must be pulled down to 12 rather than
 * indexed off the end of the HF table. */
static void lr20xx_get_pa_cfg_for_power(int8_t power_dbm, uint32_t freq_hz,
					lr20xx_radio_common_pa_cfg_t *pa,
					int8_t *half_power_out)
{
	if (lr20xx_freq_is_hf(freq_hz)) {
		if (power_dbm < LR20XX_HF_MIN_PWR) {
			power_dbm = LR20XX_HF_MIN_PWR;
		}
		if (power_dbm > LR20XX_HF_MAX_PWR) {
			power_dbm = LR20XX_HF_MAX_PWR;
		}

		const struct lr20xx_pa_pwr_entry *e =
			&pa_hf_table[power_dbm - LR20XX_HF_MIN_PWR];

		pa->pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF;
		pa->pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
		pa->pa_lf_duty_cycle = LR20XX_PA_LF_DUTY_CYCLE_UNUSED;
		pa->pa_lf_slices     = e->pa_lf_slices;
		pa->pa_hf_duty_cycle = e->pa_duty_cycle;

		*half_power_out = e->half_power;
		return;
	}

	if (power_dbm < LR20XX_LF_MIN_PWR) {
		power_dbm = LR20XX_LF_MIN_PWR;
	}
	if (power_dbm > LR20XX_LF_MAX_PWR) {
		power_dbm = LR20XX_LF_MAX_PWR;
	}

	int idx = power_dbm - LR20XX_LF_MIN_PWR;
	const struct lr20xx_pa_pwr_entry *e = &pa_lf_table[idx];

	pa->pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF;
	pa->pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
	pa->pa_lf_duty_cycle = e->pa_duty_cycle;
	pa->pa_lf_slices     = e->pa_lf_slices;
	pa->pa_hf_duty_cycle = LR20XX_PA_HF_DUTY_CYCLE_UNUSED;

	*half_power_out = e->half_power;
}

static lr20xx_status_t lr20xx_calibrate_front_end(void *ctx, uint32_t freq_hz);
static bool lr20xx_dc_suspend(struct lr20xx_data *data);
static bool lr20xx_dc_rx_in_flight(struct lr20xx_data *data);
static void lr20xx_dc_resume(struct lr20xx_data *data, bool was_armed);

/* ── Firmware Patch RAM (PRAM) ──────────────────────────────────────── */

/* Base address the patch image is written to — DS §22.3.1, and the same value
 * lr20xx_patch.c uses internally (it does not export it). */
#define LR20XX_PRAM_BASE_ADDRESS 0x801000

/* Load and activate the firmware Patch RAM.
 *
 * DS §22.3: "using the chip without the PRAM can create performance issues and
 * unexpected bugs. The use of the PRAM is therefore highly recommended", and
 * §22: "Most of the workarounds are implemented in the Firmware Patch RAM".
 * Semtech's own driver confirms the relationship — v2.0.2 added PRAM support and
 * deleted its BLE, RTToF and DC-DC workaround functions in the same release,
 * because the patch supersedes them.
 *
 * The image is volatile: lost on reset and on cold start, preserved by sleep
 * with retention (§22.3).  Duty-cycle sleep is retention by definition (§6.3.8)
 * and this driver issues no other sleep, so the reset paths are the only places
 * it has to be (re)loaded.  Costs 2240 bytes of flash and +80 nA of retention
 * sleep current.
 *
 * Best-effort: a chip without the patch still works, so a failure here is a
 * warning, not a reason to fail init. */
static void lr20xx_load_pram(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;
	lr20xx_system_version_t chip = { 0 };
	lr20xx_patch_version_t pram = { 0 };
	lr20xx_status_t rc;

	/* DS §22.3.1: "There is one dedicated to the LR2021 and another one for
	 * the LR2012/LR2022.  The GetVersion(...) API indicates which PRAM is
	 * needed."  Per DS Table 6-40 the LR2021 answers 0x01/0x18; the other two
	 * answer 0x02/0x00 and want lr20xx_pram_lr20x2.h, which is not vendored
	 * because this driver only binds to semtech,lr2021.  Read the version
	 * here rather than taking it from the caller so both reset paths get the
	 * same check. */
	if (lr20xx_system_get_version(ctx, &chip) != LR20XX_STATUS_OK) {
		LOG_WRN("PRAM: get_version failed — skipping patch load");
		return;
	}
	if (chip.major != 0x01 || chip.minor != 0x18) {
		LOG_WRN("PRAM: chip reports FW %u.%u, not the LR2021's 1.24 — "
			"no matching patch image vendored, skipping",
			chip.major, chip.minor);
		return;
	}

	rc = lr20xx_patch_load_pram(ctx, LR20XX_PRAM_BASE_ADDRESS, pram_lr2021,
				    pram_lr2021_size);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("PRAM: load failed (rc=%d) — running unpatched", rc);
		return;
	}

	rc = lr20xx_patch_enable_pram(ctx);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("PRAM: enable failed (rc=%d) — running unpatched", rc);
		return;
	}

	/* Reads back the magic word at 0x800FF8 (DS §22.3.2) — the only proof
	 * the chip actually took the patch, so it is worth the two extra reads
	 * on a path that runs once per reset. */
	if (lr20xx_patch_get_version(ctx, &pram) != LR20XX_STATUS_OK ||
	    !pram.is_pram_loaded) {
		LOG_ERR("PRAM: magic word absent after load — running unpatched");
		return;
	}

	LOG_INF("PRAM loaded: type=0x%02x version=0x%02x (%u words)",
		pram.pram_type, pram.pram_version, pram_lr2021_size);
}

/* ── Hardware reset (BUSY stuck recovery) ───────────────────────────── */

static void lr20xx_hardware_reset(struct lr20xx_data *data,
				  const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_WRN("LR2021 hardware reset (BUSY stuck recovery)");

	lr20xx_hal_reset(ctx);

	/* The reset wiped the patch — DS §22.3: "The PRAM is lost after a reset
	 * or a cold start", and §22.3.1 requires it be reloaded "after a reset,
	 * as part of the reset sequence". */
	lr20xx_load_pram(data);

	/* SIMO workaround skipped — LDO mode.  (The citation here used to be
	 * "DS §22.6"; rev 2.1 has no such section — §22 ends at 22.3.  The
	 * conclusion still holds via Table 6-26: simo_usage 0x00 SIMO_OFF is the
	 * reset default and we never issue SetRegMode.) */

	if (cfg->tcxo_voltage_mv > 0) {
		/* Timeout in RTC ticks (30.52 µs/tick) */
		lr20xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    tcxo_start_time_periods(cfg->tcxo_startup_delay_ms));
	}

	/* LDO mode — no cfg_lfclk, no set_reg_mode, no DCDC workarounds.
	 *
	 * cfg_lfclk(LF_RC) was tried here on the theory that LF_XOSC_START_ERR
	 * meant the LF domain needed an explicit enable-and-wait (DS §6.11.1).
	 * It changed nothing on hardware — the error still appears — so it is
	 * not the cause and the command is not carried.  RC is the reset
	 * default and the X1 routes no DIO11, so RC is what we get either way. */

	lr20xx_configure_rfswitch(ctx, cfg);

	lr20xx_system_set_dio_function(ctx, lr20xx_irq_dio(cfg),
				       LR20XX_SYSTEM_DIO_FUNC_IRQ,
				       lr20xx_irq_dio_pull(cfg));

	lr20xx_radio_common_set_rx_tx_fallback_mode(ctx,
						    LR20XX_RADIO_FALLBACK_STDBY_RC);

	lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);

	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	/* No delay after Calibrate — DS §6.4.1: "Upon completion of the
	 * calibration, the chip automatically enters Standby RC mode", BUSY
	 * deasserts, and the next command's check_device_ready() waits on it. */
	lr20xx_system_calibrate(ctx, 0x6F);

	/* The reset wiped the front-end calibration — DS §6.4.2: "There is no
	 * Front End calibration performed after a POR or cold start".  Every
	 * caller of this function is post-config (the DIO1 stuck-recovery path
	 * and three standby-failure paths), and lr20xx_start_rx() follows
	 * immediately, so calibrate for the frequency actually about to be
	 * received on. */
	if (data->configured) {
		lr20xx_calibrate_front_end(ctx, data->modem_cfg.frequency);
	}

	data->rx_boost_applied = false;

	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	LOG_WRN("LR2021 recovered from hardware reset");
}

/* ── Front-end calibration ──────────────────────────────────────────── */

/* Front-end calibration can legitimately fail on RSSI saturation from a nearby
 * interferer, and repeating it is the documented cure — RadioLib retries up to
 * 10 times for exactly this, and gives up immediately on any other error since
 * repeating those cannot help. Getting this wrong is expensive: an uncalibrated
 * front end makes the chip reject every set_rx with RXFREQ_NO_FRONT_END_CALIB,
 * which then looks like a broken RX path rather than a failed calibration.
 */
#define LR20XX_MAX_CAL_ATTEMPTS 10

/* Calibrate the front end at the operating frequency.
 *
 * CalibFE takes point frequencies in 4 MHz steps (bit 15 = LF/HF path), not a
 * band like the SX126x/LR11xx CalibrateImage.  The SDK rounds the argument UP
 * where the chip's own default truncates DOWN, so both neighbours are
 * calibrated, nearest first.  Never called on the Tx/Rx path — DS 6.4.2 keeps
 * the values on chip across retention sleep.  See ARCHITECTURE.md 5.5.1. */
static lr20xx_status_t lr20xx_calibrate_front_end(void *ctx, uint32_t freq_hz)
{
	/* DS §6.4.2: "Frequencies are given in 4MHz steps (Ex: 900MHz ->
	 * 0x00E1)".  Passing an exact multiple makes the SDK's ceil a no-op, so
	 * each slot lands on precisely the grid point intended. */
	const uint32_t step = 4000000U;
	uint32_t lo = (freq_hz / step) * step;
	uint32_t hi = ((freq_hz + step - 1U) / step) * step;
	uint32_t first = lo, second = 0;

	if (lo != hi) {
		/* Nearest neighbour first: it goes in slot 1, which is also the
		 * slot the chip's no-argument default would overwrite. */
		if ((freq_hz - lo) <= (hi - freq_hz)) {
			first = lo;
			second = hi;
		} else {
			first = hi;
			second = lo;
		}
	}

	/* All three frequency slots are always sent, unused ones as 0, so
	 * CalibFE gets its full 8-byte form. The datasheet allows both "not
	 * providing" the extra slots and "setting other calibration frequencies
	 * to 0", but a short 4-byte command is the one thing that lines up with
	 * the CMD_PERR observed across exactly this call, with a clean error
	 * word and the chip in STBY_RC (so not a mode violation). Zeroed slots
	 * are documented as no-ops (DS §6.4.2). */
	const lr20xx_radio_common_rx_path_t cal_path =
		lr20xx_rx_path_for(freq_hz);
	lr20xx_radio_common_front_end_calibration_value_t fe_cal[3] = {
		{ .rx_path = cal_path, .frequency_in_hertz = first },
		{ .rx_path = cal_path, .frequency_in_hertz = second },
		{ .rx_path = cal_path, .frequency_in_hertz = 0 },
	};

	for (int i = 0; i < LR20XX_MAX_CAL_ATTEMPTS; i++) {
		lr20xx_system_errors_t errs = 0;
		lr20xx_system_stat1_t s1 = { 0 };
		bool rejected = false;
		lr20xx_status_t rc;

		lr20xx_system_clear_errors(ctx);

		rc = lr20xx_radio_common_calibrate_front_end_helper(ctx, fe_cal, 3);

		/* The SDK return code reflects the SPI write, not whether the chip
		 * executed the command.  A CalibFE issued from Rx or Tx answers
		 * CMD_FAIL (DS §6.4.2) while rc stays OK and no error bit is set,
		 * so without this check the caller is told the front end was
		 * calibrated when it was not — and the first symptom is degraded
		 * RX with a RxFreqNoCalErr nobody connects to this call.  Must be
		 * read BEFORE get_errors(): Stat reports the previous command.
		 * DS Table 6-38: 0x2 CMD_OK, 0x3 CMD_DAT, anything else rejected. */
		if (lr20xx_system_get_status(ctx, &s1, NULL, NULL) ==
		    LR20XX_STATUS_OK) {
			rejected = (s1.command_status != 2 &&
				    s1.command_status != 3);
		}

		lr20xx_system_get_errors(ctx, &errs);

		if (rejected) {
			/* Not retried: the chip refused on mode grounds, and
			 * repeating the command from the same mode cannot help. */
			LOG_ERR("FE cal(%u Hz) REJECTED (cmd_status=%d) — CalibFE "
				"needs STDBY_RC/XOSC/FS, not Rx or Tx (DS §6.4.2); "
				"front end left uncalibrated",
				freq_hz, s1.command_status);
			return LR20XX_STATUS_ERROR;
		}

		if (rc == LR20XX_STATUS_OK &&
		    !(errs & LR20XX_SYSTEM_ERRORS_SRC_SATURATION_CALIB_MASK)) {
			/* Report the grid points actually programmed, not just
			 * the operating frequency — the two differ, and a
			 * bring-up log that hides that is how the 4 MHz
			 * quantisation goes unnoticed. */
			LOG_DBG("FE cal(%u Hz -> %u/%u MHz) ok on attempt %d "
				"(errors=0x%04x)", freq_hz,
				first / 1000000U, second / 1000000U,
				i + 1, errs);
			return LR20XX_STATUS_OK;
		}

		if (!(errs & LR20XX_SYSTEM_ERRORS_SRC_SATURATION_CALIB_MASK)) {
			LOG_ERR("FE cal(%u Hz) failed: rc=%d errors=0x%04x "
				"(not saturation — retrying will not help)",
				freq_hz, rc, errs);
			return (rc != LR20XX_STATUS_OK) ? rc : LR20XX_STATUS_ERROR;
		}

		LOG_WRN("FE cal(%u Hz) RSSI saturation (errors=0x%04x), "
			"attempt %d/%d", freq_hz, errs, i + 1,
			LR20XX_MAX_CAL_ATTEMPTS);
		k_msleep(5);
	}

	LOG_ERR("FE cal(%u Hz) gave up after %d attempts — RX will be refused",
		freq_hz, LR20XX_MAX_CAL_ATTEMPTS);
	return LR20XX_STATUS_ERROR;
}

/* ── Apply modem configuration ──────────────────────────────────────── */

/* Bring-up only: read the chip's own command status and name the command that
 * failed. Return codes from the SDK reflect the SPI write, not whether the chip
 * accepted the command, so a rejection is otherwise invisible until it shows up
 * as a CmdError IRQ several commands later. */
#if IS_ENABLED(CONFIG_LOG)
static void lr20xx_check_cmd(void *ctx, const char *what)
{
	lr20xx_system_stat1_t s1 = {0};

	if (lr20xx_system_get_status(ctx, &s1, NULL, NULL) != LR20XX_STATUS_OK) {
		return;
	}
	/* 2 = CMD_OK, 3 = CMD_DAT (successful read) */
	if (s1.command_status != 2 && s1.command_status != 3) {
		LOG_ERR("command REJECTED after %s: cmd=%d", what,
			s1.command_status);
	}
}
#define CHECK_CMD(ctx, what) lr20xx_check_cmd((ctx), (what))
#else
#define CHECK_CMD(ctx, what) do { } while (0)
#endif

/* Program the stored side-detector set into the chip (or clear it).
 *
 * The chip drops every side detector on SetModulationParams, so this has to
 * run after each apply_modem_config and after anything else that reprograms
 * the modem — the caller owns spi_mutex and must have the chip out of Rx/Tx.
 *
 * `enable == false` writes the n=0 form, which is how the datasheet says to
 * turn the feature off.  That is needed around CAD: the constraints are
 * directional and mutually exclusive with ours — normal Rx requires the main
 * SF to be *lower* than every side SF, CAD requires it to be *higher*.  Since
 * this driver runs LBT CAD before every TX, side detectors are switched off
 * for the CAD and restored when RX is re-armed.  (Upstream MeshCore hit the
 * same wall from the other side: RadioLib returned -706 on the startReceive
 * after a hardware CAD with side detectors enabled.) */
static void lr20xx_apply_side_detectors(struct lr20xx_data *data, bool enable)
{
	void *ctx = &data->hal_ctx;
	lr20xx_radio_lora_side_detector_cfg_t det[LR20XX_MAX_SIDE_DETECTORS];
	uint8_t syncwords[LR20XX_MAX_SIDE_DETECTORS];
	uint8_t n = enable ? data->side_det_num : 0;
	uint32_t bw_hz;

	if (n == 0) {
		if (!data->side_det_applied) {
			return;   /* already off — don't spend an SPI command */
		}
		lr20xx_radio_lora_configure_side_detectors(ctx, NULL, 0);
		data->side_det_applied = false;
		return;
	}

	bw_hz = (uint32_t)(bw_enum_to_khz(data->modem_cfg.bandwidth) * 1000.0f);

	for (uint8_t i = 0; i < n; i++) {
		det[i].sf = (lr20xx_radio_lora_sf_t)data->side_det_sf[i];
		/* Same LDRO rule as the main modem config — evaluated per side
		 * detector, since each runs at its own SF and a longer symbol
		 * crosses the 16.38 ms threshold before the main one does. */
		det[i].ppm = lr20xx_ldro_enabled(data->side_det_sf[i], bw_hz)
				     ? LR20XX_RADIO_LORA_PPM_1_4
				     : LR20XX_RADIO_LORA_NO_PPM;
		det[i].iq = data->modem_cfg.iq_inverted
				? LR20XX_RADIO_LORA_IQ_INVERTED
				: LR20XX_RADIO_LORA_IQ_STANDARD;
		/* One mesh, one sync word: side detectors carry the same
		 * public/private word as the main detector. */
		syncwords[i] = data->modem_cfg.public_network ? 0x34 : 0x12;
	}

	lr20xx_radio_lora_configure_side_detectors(ctx, det, n);
	CHECK_CMD(ctx, "configure_side_detectors");
	lr20xx_radio_lora_set_side_detector_syncwords(ctx, syncwords, n);
	CHECK_CMD(ctx, "set_side_detector_syncwords");
	data->side_det_applied = true;

	LOG_DBG("side detectors: %u active (SF%u/%u/%u)", n,
		data->side_det_sf[0],
		n > 1 ? data->side_det_sf[1] : 0,
		n > 2 ? data->side_det_sf[2] : 0);
}

static void lr20xx_apply_modem_config(struct lr20xx_data *data,
				      const struct lr20xx_config *cfg,
				      bool tx_mode)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;
	lr20xx_status_t rc;

	/* Half of what follows is mode-restricted, and this runs on the TX path
	 * while the chip may still be in RX:
	 *   SetPacketType — "only works when the chip is in Standby RC, Standby
	 *                    Xosc, or Fs mode", and must come first in a setup
	 *   CalibFE       — "does not work if device is in Rx or Tx mode"
	 * Both answer CMD_FAIL from the wrong mode, which surfaces only as a
	 * CmdError IRQ several commands later. Force standby so the sequence is
	 * always issued from a legal mode, as RadioLib does before its own
	 * mode-restricted calls. */
	lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC);
	CHECK_CMD(ctx, "set_standby");

	rc = lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
	LOG_DBG("modem_cfg: set_pkt_type=%d", rc);
	CHECK_CMD(ctx, "set_pkt_type");

	/* No FE calibration here: this runs three times per transmitted packet and
	 * DS 6.4.2 keeps the values on chip.  See ARCHITECTURE.md 5.5.1. */
	rc = lr20xx_radio_common_set_rf_freq(ctx, mc->frequency);
	LOG_DBG("modem_cfg: set_rf_freq(%u)=%d", mc->frequency, rc);
	CHECK_CMD(ctx, "set_rf_freq");

	/* Always configure the RX path after setting frequency
	 * (reference does this on every set_rf_freq call).
	 *
	 * The path follows the frequency: ral_lr20xx_bsp_get_rx_cfg() picks HF
	 * at or above 1.5 GHz and LF below.  Getting this wrong is not subtle —
	 * receiving 2.4 GHz down the sub-GHz path is simply deaf. */
	rc = lr20xx_radio_common_set_rx_path(
		ctx, lr20xx_rx_path_for(mc->frequency),
		data->rx_boost_enabled
			? LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_7
			: LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
	data->rx_boost_applied = data->rx_boost_enabled;
	CHECK_CMD(ctx, "set_rx_path");

	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(mc->bandwidth) * 1000.0f);

	lr20xx_radio_lora_mod_params_t mod = {
		.sf  = (lr20xx_radio_lora_sf_t)mc->datarate,
		.bw  = bw_enum_to_lr20xx(mc->bandwidth),
		.cr  = cr_enum_to_lr20xx(mc->coding_rate),
		.ppm = lr20xx_ldro_enabled((uint8_t)mc->datarate, bw_hz)
			       ? LR20XX_RADIO_LORA_PPM_1_4
			       : LR20XX_RADIO_LORA_NO_PPM,
	};
	rc = lr20xx_radio_lora_set_modulation_params(ctx, &mod);
	LOG_DBG("modem_cfg: set_mod(SF%d BW%d CR%d PPM%d)=%d",
		mod.sf, mod.bw, mod.cr, mod.ppm, rc);
	CHECK_CMD(ctx, "set_mod_params");
	/* SetModulationParams disables every side detector, chip-side. */
	data->side_det_applied = false;

	/* DCDC workaround removed — LDO mode, RadioLib doesn't do it */

	lr20xx_radio_lora_pkt_params_t pkt = {
		.preamble_len_in_symb = mc->preamble_len,
		.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = 255,
		.crc = mc->packet_crc_disable ? LR20XX_RADIO_LORA_CRC_DISABLED
					      : LR20XX_RADIO_LORA_CRC_ENABLED,
		.iq = mc->iq_inverted ? LR20XX_RADIO_LORA_IQ_INVERTED
				      : LR20XX_RADIO_LORA_IQ_STANDARD,
	};
	rc = lr20xx_radio_lora_set_packet_params(ctx, &pkt);
	LOG_DBG("modem_cfg: set_pkt(pre=%d len=%d crc=%d iq=%d)=%d",
		pkt.preamble_len_in_symb, pkt.pld_len_in_bytes,
		pkt.crc, pkt.iq, rc);
	CHECK_CMD(ctx, "set_pkt_params");

	rc = lr20xx_radio_lora_set_syncword(ctx,
				       mc->public_network ? 0x34 : 0x12);
	LOG_DBG("modem_cfg: set_syncword(0x%02x)=%d",
		mc->public_network ? 0x34 : 0x12, rc);
	CHECK_CMD(ctx, "set_syncword");

	if (tx_mode) {
		/* PA config + TX params from Semtech's reference tables, picked
		 * per band from the frequency just programmed above.
		 * DS §7.4.3: SetPaConfig must precede SetTxParams. */
		lr20xx_radio_common_pa_cfg_t pa;
		int8_t half_power;

		lr20xx_get_pa_cfg_for_power(mc->tx_power, mc->frequency,
					    &pa, &half_power);
		rc = lr20xx_radio_common_set_pa_cfg(ctx, &pa);
		LOG_DBG("modem_cfg: set_pa_cfg(sel=%d mode=%d duty=%d slices=%d hf_duty=%d)=%d",
			pa.pa_sel, pa.pa_lf_mode, pa.pa_lf_duty_cycle,
			pa.pa_lf_slices, pa.pa_hf_duty_cycle, rc);

		rc = lr20xx_radio_common_set_tx_params(ctx, half_power,
						  LR20XX_RADIO_COMMON_RAMP_48_US);
		LOG_DBG("modem_cfg: set_tx_params(%d half-dBm = %d.%d dBm, ramp=0x05)=%d",
			half_power, half_power / 2,
			(half_power & 1) ? 5 : 0, rc);
	}

	/* The events the DIO1 handler acts on, plus SYNC_WORD_HEADER_VALID.
	 *
	 * PREAMBLE_DETECTED is deliberately NOT here: it fires on noise, and an
	 * unhandled DIO1 assertion drives the safety path, which restarts RX and
	 * destroys the very packet that was arriving.  It stays readable in the
	 * IRQ register for lr20xx_is_receiving()'s grace window.
	 *
	 * SYNC_WORD_HEADER_VALID used to sit alongside it, which was wrong on
	 * both counts.  A header only latches after a sync-word match AND a
	 * header CRC, so it fires at most once per real packet, not on noise;
	 * and the safety-path objection is answered by handling it rather than
	 * by hiding it (the handler now has a branch that marks the reception as
	 * continuing).  Leaving it off DIO1 meant nothing ever stamped
	 * header_seen_at_ms, and that timestamp is the entire RX-busy answer
	 * once a duty cycle is armed -- the poll path is forbidden to touch the
	 * bus there -- so lr20xx_is_receiving() returned false unconditionally on
	 * every duty-cycled node, blinding the TX gate, the noise-floor sampler
	 * and the CAD probe alike.
	 *
	 * This IS the split the SX126x driver uses, which the old comment
	 * claimed while doing something else: preamble off DIO1, header on. */
	rc = lr20xx_system_set_dio_irq_cfg(ctx, lr20xx_irq_dio(cfg),
		LR20XX_SYSTEM_IRQ_RX_DONE |
		LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID |
		LR20XX_SYSTEM_IRQ_TX_DONE |
		LR20XX_SYSTEM_IRQ_CAD_DONE |
		LR20XX_SYSTEM_IRQ_CAD_DETECTED |
		LR20XX_SYSTEM_IRQ_TIMEOUT |
		LR20XX_SYSTEM_IRQ_CRC_ERROR |
		LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR |
		LR20XX_SYSTEM_IRQ_ERROR |
		LR20XX_SYSTEM_IRQ_CMD_ERROR);
	LOG_DBG("modem_cfg: set_dio_irq=%d", rc);
	CHECK_CMD(ctx, "set_dio_irq");

	/* Re-arm side detectors for RX; leave them off for the TX path (their
	 * constraints are RX-directional, and set_mod_params just cleared
	 * them anyway). */
	lr20xx_apply_side_detectors(data, !tx_mode);

	DUMP_CHIP_STATE(data, tx_mode ? "modem-TX" : "modem-RX");
}

/* ── RX duty cycle ──────────────────────────────────────────────────── */

/**
 * Re-issue the SetRxDutyCycle command with the timing stored by
 * lr20xx_lora_recv_duty_cycle().  Returns true on success, false if no
 * timing has been provided yet (falls back to continuous RX).
 */
/* Unstick the AGC, per Semtech's stated remedy: warm sleep to power the analog
 * front end down, then recalibrate on the way back up.  Same shape as Arduino
 * MeshCore's sx126xResetAGC() / lr11x0ResetAGC() (helpers/radiolib/SX126xReset.h,
 * LR11x0Reset.h) and
 * as our own SX126x duty-cycle re-arm.  Caller must hold spi_mutex.
 *
 * Two LR2021-specific differences from those ports, both saving real time:
 *
 * 1. NO CalibFE afterwards.  On the SX126x and LR11x0, Calibrate(ALL) includes
 *    image calibration and resets it to the 902-928 MHz default, which is why
 *    both Arduino helpers re-issue calibrateImage()/calibrateImageRejection().
 *    On the LR2021 the front end is a separate command: DS Table 6-28 lists the
 *    Calibrate blocks as LF_RC, HF_RC, PLL, AAF, MU and PA_OFF — no image, no
 *    front end.  CalibFE (§6.4.2) owns those and its values survive retention
 *    sleep, so re-running it here would cost ~9 ms to reproduce what is already
 *    on the chip.  An earlier draft ran CalibFE on every invocation, which is
 *    most of why this sequence looked expensive.
 *
 * 2. No fixed post-Calibrate delay.  DS §6.4.1: "Upon completion of the
 *    calibration, the chip automatically enters Standby RC mode" — BUSY
 *    deasserts and the next command's check_device_ready() waits on it, so the
 *    SX126x's 5 ms busy-wait has no counterpart here.
 *
 * Budget: 1 ms for the HAL's post-sleep-opcode wait, 1 ms for the warm start
 * (DS Table 3-23 TSPDRCW, "retrieve calibration from retained registers"), plus
 * the Calibrate itself, which the datasheet does not put a number on. */
static void lr20xx_recalibrate_locked(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;
	lr20xx_system_sleep_cfg_t sleep_cfg = {
		.is_clk_32k_enabled       = false,
		/* Retention: keeps the modem config and the PRAM image (DS
		 * §22.3).  A cold sleep here would drop both. */
		.is_ram_retention_enabled = true,
	};

	lr20xx_system_set_sleep_mode(ctx, &sleep_cfg, 0);
	lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC);

	/* 0x6F = every block in Table 6-28.  Legal only outside Rx/Tx, which the
	 * standby above guarantees. */
	lr20xx_system_calibrate(ctx, 0x6F);

	/* Re-assert the Rx path after calibrating AAF and MU, both of which are
	 * receive-chain blocks.  Retention preserves the setting across the sleep
	 * (see the note in lr20xx_apply_rx_duty_cycle), but nothing documents what
	 * Calibrate leaves behind — and both Arduino helpers re-apply boost here
	 * for the same reason.  One command. */
	if (data->rx_boost_enabled) {
		lr20xx_radio_common_set_rx_path(
			ctx, lr20xx_rx_path_for(data->modem_cfg.frequency),
			LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_7);
		data->rx_boost_applied = true;
	}

	/* Front end only on the drift path.  Image/FE calibration is valid for a
	 * temperature range rather than forever (DS: "necessary if there is a
	 * frequency change > 10MHz, or a temperature change > 10 C"), so it is
	 * exactly what a temperature swing invalidates.  (This used to be
	 * conditional, skipped by an "AGC reset" caller that no longer exists —
	 * that fault belongs to the SX126x, not to this part.) */
	if (data->configured) {
		lr20xx_calibrate_front_end(ctx, data->modem_cfg.frequency);
	}

	/* Contract with the caller: leave the driver OUT of
	 * RX.  The chip is parked in STDBY_RC here, and the caller's
	 * startReceive() must perform a real re-entry — without this the
	 * idempotent fast paths in recv_async / recv_duty_cycle would see
	 * in_rx_mode still set, refresh the callback and return, leaving the
	 * receiver switched off. */
	data->in_rx_mode = false;
	lr20xx_reset_rx_busy_signals(data);
}

/* ── Extension API: receiver hygiene ─────────────────────────────────── */

/* Redo the frequency-dependent calibrations after temperature drift: warm
 * sleep, Calibrate (LF_RC/HF_RC/PLL/AAF/MU/PA_OFF), rx-boost re-apply, then
 * CalibFE at the operating frequency.
 *
 * Deliberately NOT an "AGC reset".  That is an SX126x remedy for an SX126x
 * fault; nothing in the LR2021 datasheet describes it on this part, and firing
 * the sequence speculatively costs packets — measured on the LR1110 sibling as
 * a 7.4%% miss rate in the 60 s afterwards against 0.6%% elsewhere.  The driver
 * therefore exposes only the operation the datasheet does call for.
 *
 * Leaves the driver out of RX — the caller must startReceive() afterwards. */
void lr20xx_recalibrate(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	if (!data->configured) {
		return;
	}

	if (k_mutex_lock(&data->spi_mutex, K_MSEC(50)) != 0) {
		LOG_DBG("recalibrate: mutex busy, deferring");
		return;
	}

	if (lr20xx_dc_rx_in_flight(data)) {
		LOG_DBG("recalibrate: reception in flight, deferring");
		k_mutex_unlock(&data->spi_mutex);
		return;
	}

	/* Own the cycle across the sequence: it opens with SetSleep, which is
	 * fatal to a chip already in its own sleep phase, and the recalibration
	 * leaves the radio in standby regardless. */
	bool armed = lr20xx_dc_suspend(data);

	lr20xx_recalibrate_locked(data);

	lr20xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);
}

/* Junction temperature in whole degrees C, INT16_MIN if unavailable.
 * VALUE_FORMAT_UNIT returns 13.5sb: integer part in the high byte, fraction in
 * the low.  13-bit resolution costs nothing here — this runs once per
 * maintenance tick, never on the packet path. */
int16_t lr20xx_get_chip_temp_c(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	uint16_t raw = 0;
	lr20xx_status_t rc;

	if (!data->configured) {
		return INT16_MIN;
	}
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return INT16_MIN;
	}

	if (lr20xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return INT16_MIN;
	}

	/* Bracketed rather than skipped — GetTemp into the sleep phase both
	 * risks the chip and, per DS §6.3.8, silently ends the cycle. */
	bool armed = lr20xx_dc_suspend(data);

	rc = lr20xx_system_get_temp(&data->hal_ctx,
				    LR20XX_SYSTEM_VALUE_FORMAT_UNIT,
				    LR20XX_SYSTEM_MEAS_RES_13_BITS,
				    LR20XX_SYSTEM_TEMP_SRC_VBE, &raw);

	lr20xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);

	if (rc != LR20XX_STATUS_OK) {
		return INT16_MIN;
	}
	return (int16_t)((int8_t)(raw >> 8));
}

static bool lr20xx_apply_rx_duty_cycle(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;

	if (data->dc_rx_ms == 0 || data->dc_sleep_ms == 0) {
		LOG_WRN("No duty-cycle timing stored — continuous RX");
		data->rx_duty_cycle_enabled = false;
		lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			ctx, 0xFFFFFF);
		return false;
	}

	/* Deliberately NO RX-boost re-apply here, unlike lr11xx_restart_rx().
	 * The LR2021 retains it across the duty-cycle sleep and the datasheet is
	 * explicit on every step of that:
	 *   §4.4.1  — retention "save[s] all previously programmed chip settings"
	 *   §6.3.8  — "In RxDutyCycleMode, the transceiver context is retained
	 *              during sleep operations", and the inter-window sleep is
	 *              always the with-retention kind
	 *   §7.3.4  — the Rx gain tables, whose entries are G1..G11 plus
	 *              G12_boost0..7 and G13_boost0..7, are "automatically stored
	 *              in the retention memory"
	 * SetRxPath is an ordinary command, so the boost is part of that retained
	 * context.  SetAdditionalRegToRetain (§6.3.4) is not a counter-argument:
	 * it takes "the address of an additional peripheral register", for
	 * directly-poked internals outside the command-programmed context — the
	 * datasheet's only worked example is the SX1276 freq-hop compatibility
	 * register 0xF30A24, set by a workaround rather than by a command.
	 *
	 * This is where the SX126x and the LR2021 genuinely differ: the SX126x
	 * does NOT retain Rx gain (DS §9.6 requires pinning it into the warm-start
	 * retention list, and skipping that costs 3 dB on every wake after the
	 * first).  Do not port that fix here by analogy — it is a real gap on that
	 * chip and a no-op write on this one. */

	/* Clear the CMD_PERR the HAL's clockless wake frame provokes: it is
	 * self-inflicted by the wake and says nothing about the command below.
	 * Left set it holds DIO1 high and poisons the re-arm. */
	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_CMD_ERROR);

	/* The second SetRxDutyCycle field is cycle_time, NOT sleep time —
	 * DS §6.3.8 Table 6-14: "cycle_time specifies the duration of the period
	 * between the start of consecutive Rx windows", and the chip sleeps for
	 * cycle_time - rx_max_time.  The vendor SDK calls the argument
	 * `sleep_period_in_ms` and marshals it straight into those bytes, which
	 * is correct for the LR11xx (whose field really is SleepPeriod) and
	 * wrong here — the LR2021 redefined it.  Passing the sleep time raw
	 * makes cycle_time < rx_max_time for every preset we use, and the
	 * datasheet is explicit: "If cycle_time is lower than rx_max_time a
	 * CMD_ERR is returned in the status of the next command."  That is the
	 * CMD_ERROR IRQ and the rejected commands seen on the X1 the moment a
	 * duty cycle armed.  Convert here rather than at the call sites so
	 * dc_sleep_ms keeps its plain meaning everywhere else. */
	lr20xx_radio_common_set_rx_duty_cycle(ctx, data->dc_rx_ms,
		data->dc_rx_ms + data->dc_sleep_ms,
		LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_RX);

	LOG_DBG("RX duty cycle re-armed: rx=%ums sleep=%ums (cycle=%ums)",
		data->dc_rx_ms, data->dc_sleep_ms,
		data->dc_rx_ms + data->dc_sleep_ms);
	return true;
}

/* ── Duty-cycle ownership ─────────────────────────────────────────────
 *
 * Mirror of the LR11xx helpers, for the same reason and from the same text.
 * DS §6.3.8 ends the loop on exactly three events — a packet is received (the
 * chip "goes back to the configured fallback mode"), the host issues SetStandby
 * during the Rx window, or the chip is woken "with a falling edge of NSS", for
 * which the datasheet adds "A SetStandby command should also be sent, to avoid
 * the race conditions".  Word for word the LR11xx rule.
 *
 * So the same conclusion follows: the host issues every NSS edge and can always
 * know whether the cycle is alive, but only by owning it rather than trying to
 * dodge the sleep phase.  lr20xx_is_chip_busy() is a check-then-act guard — the
 * chip can enter sleep between the GPIO read and the NSS assert — and with a
 * cycle armed its answer is ambiguous anyway, because BUSY is high both in the
 * sleep phase and through ordinary Rx on this family (the reason that helper is
 * gated on rx_duty_cycle_enabled in the first place).
 *
 * The stand-down is deliberately SetStandby and never an NSS poke: waking this
 * part by NSS while it is mid-command is what produced the CMD_PERR rejection
 * storm on the MeshTracker X1 (see check_device_ready()).
 *
 * NOT yet verified on hardware — the X1 was not available when this was
 * written.  The LR11xx equivalent was measured: packet parity with the duty
 * cycle armed restored from 10.7% to 100%, and the RSSI sampler from 1 of 407
 * bursts completing to all of them.
 *
 * Both require data->spi_mutex held by the caller.
 */
static bool lr20xx_dc_suspend(struct lr20xx_data *data)
{
	if (!data->rx_duty_cycle_enabled || !data->in_rx_mode) {
		return false;
	}

	lr20xx_system_set_standby_mode(&data->hal_ctx,
				       LR20XX_SYSTEM_STANDBY_MODE_RC);
	data->in_rx_mode = false;
	return true;
}

static void lr20xx_dc_resume(struct lr20xx_data *data, bool was_armed)
{
	if (!was_armed) {
		return;
	}

	/* Clear stale IRQ state before re-arming, matching what this driver's
	 * own re-arm callers already do either side of
	 * lr20xx_apply_rx_duty_cycle() — a PREAMBLE or HEADER bit latched before
	 * the stand-down would otherwise be read as belonging to the new cycle.
	 * (apply_rx_duty_cycle() itself clears only CMD_ERROR, which covers the
	 * HAL's wake frame, not the receive path.) */
	lr20xx_system_clear_irq_status(&data->hal_ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	lr20xx_reset_rx_busy_signals(data);
	lr20xx_apply_rx_duty_cycle(data);
	data->in_rx_mode = true;
}

static uint32_t lr20xx_preamble_grace_ms(struct lr20xx_data *data);

/* Is a duty-cycled reception already under way?  Ask before lr20xx_dc_suspend().
 *
 * Port of lr11xx_dc_rx_in_flight() -- same fault, same fix.  dc_suspend()
 * stands the chip down unconditionally, which ends a reception in progress,
 * and with the cycle armed lr20xx_is_receiving() answers from the header latch
 * only, so the preamble phase is invisible to every discretionary stand-down
 * (noise-floor sampler, RSSI read, recalibration, temperature read).  On the
 * LR1110 that lost ~1 packet in 40 at an 800 ms cadence (T1000-E bench,
 * 2026-09-19); each miss raised no IRQ at all and coincided with a sampler
 * burst that read the packet itself.
 *
 * Non-destructive status read (get_status, not get_and_clear).  In the sleep
 * phase the access wakes the chip through the HAL exactly as the caller's
 * SetStandby is about to, and the caller then terminates and re-arms the cycle
 * as before.  With a preamble latched the chip is in Rx, where bus access is
 * routine.  A preamble older than the SF-aware grace with no header is stale
 * and does not block the caller; dc_resume() clears it.
 *
 * Caller holds data->spi_mutex. */
static bool lr20xx_dc_rx_in_flight(struct lr20xx_data *data)
{
	lr20xx_system_irq_mask_t irq = 0;

	if (!data->rx_duty_cycle_enabled || !data->in_rx_mode) {
		return false;
	}
	if (lr20xx_system_get_status(&data->hal_ctx, NULL, NULL, &irq) != LR20XX_STATUS_OK) {
		return false;
	}
	if (irq & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {
		return true;
	}
	if (irq & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {
		uint32_t now = k_uptime_get_32();
		uint32_t seen = data->preamble_seen_at_ms;

		if (seen == 0) {
			data->preamble_seen_at_ms = (now == 0) ? 1U : now;
			return true;
		}
		return (now - seen) < lr20xx_preamble_grace_ms(data);
	}
	return false;
}

/* GPIO-only "can the host talk to this chip right now" check, no SPI.
 *
 * Gated on an armed duty cycle: unlike the SX126x, the LR2021 holds BUSY high
 * during ordinary continuous RX, so reporting raw BUSY would tell the caller
 * the radio is permanently unavailable — which gates TX as well as the probes
 * and mutes the node.  With a cycle armed, BUSY high does mean the chip is in
 * its own sleep window and must not be disturbed. */
bool lr20xx_is_chip_busy(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	return data->rx_duty_cycle_enabled &&
	       gpio_pin_get_dt(&data->hal_ctx.busy);
}

/* Stand an armed duty cycle down for TX.  Polling BUSY for a gap does not work
 * here: the loop is event-driven, so a deferred send has nothing to re-wake it.
 * DS 6.3.8 — an NSS edge ends the cycle and "a SetStandby command should also be
 * sent, to avoid the race conditions". */
static void lr20xx_dc_takeover(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;

	if (!data->rx_duty_cycle_enabled) {
		return;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC);
	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_CMD_ERROR);
	k_mutex_unlock(&data->spi_mutex);

	LOG_DBG("duty cycle stood down for TX");
}

/* ── Start RX (internal) ────────────────────────────────────────────── */

static void lr20xx_start_rx(struct lr20xx_data *data,
			     const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;
	lr20xx_status_t rc;

	/* Standby first — wake from any state (radio_is_sleeping is managed
	 * by the HAL via sleep opcode detection; do not set it here). */
	rc = lr20xx_system_set_standby_mode(ctx,
					    LR20XX_SYSTEM_STANDBY_MODE_RC);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("standby failed (rc=%d) — triggering HW reset", rc);
		lr20xx_hardware_reset(data, cfg);
	}

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	/* Clear RX FIFO before entering RX (Semtech reference does this) */
	lr20xx_radio_fifo_clear_rx(ctx);

	lr20xx_apply_modem_config(data, cfg, false);

	/* set_rx_path is now always called inside apply_modem_config,
	 * with boost mode set according to rx_boost_enabled. */

	if (data->rx_duty_cycle_enabled) {
		lr20xx_apply_rx_duty_cycle(data);
		/* apply may have disabled duty cycle if preamble too short */
	} else {
		lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			ctx, 0xFFFFFF);
	}

	/* Clear any IRQ flags set during modem configuration */
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	data->in_rx_mode = true;
	data->tx_active = false;
	/* Fresh RX cycle: any preamble/header timestamps belong to the old one
	 * and the bulk clear above just dropped the bits they were tracking. */
	lr20xx_reset_rx_busy_signals(data);

	/* DEBUG: dump state AFTER SET_RX — should show mode=4 (RX). The
	 * "modem-RX" dump inside apply_modem_config is taken before SET_RX. */
	DUMP_CHIP_STATE(data, "post-SET_RX");
}

/* ── Lightweight RX restart (no modem reconfig) ─────────────────────── */

/* Returns 0 if the receiver is believed to be back on air, <0 if a command the
 * re-arm depends on was rejected and the caller should escalate.
 *
 * "Believed" is deliberate on the duty-cycle path: verification there would mean
 * a GetStatus after SetRxDutyCycle, and an NSS falling edge terminates the cycle
 * being checked (DS §6.3.8) — the probe would cause the fault it looks for.  So
 * the check is placed on the SetStandby that precedes the re-arm, which is the
 * command that actually fails when the chip is wedged, and is safe to poll
 * because the standby has just ended any live cycle.  Continuous RX has no such
 * constraint and is checked after SetRx. */
static int lr20xx_restart_rx(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	lr20xx_reset_rx_busy_signals(data);

	/* LBT CAD switches side detectors off (their SF constraint is the
	 * inverse of the Rx one), so restore them on the way back into RX.
	 * No-op when none are configured or they are still programmed. */
	lr20xx_apply_side_detectors(data, true);

	/* TX rewrites payload_len to the length it sent, and in explicit-header
	 * RX that field is a filter: "accept 1..payload_len, reject anything
	 * longer with a header error". Left alone, a node silently stops hearing
	 * every packet bigger than its own last transmission. RadioLib restores
	 * this on each RX entry for the same reason. */
	{
		lr20xx_radio_lora_pkt_params_t pkt = {
			.preamble_len_in_symb = data->modem_cfg.preamble_len,
			.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
			.pld_len_in_bytes = 255,
			.crc = data->modem_cfg.packet_crc_disable
				? LR20XX_RADIO_LORA_CRC_DISABLED
				: LR20XX_RADIO_LORA_CRC_ENABLED,
			.iq = data->modem_cfg.iq_inverted
				? LR20XX_RADIO_LORA_IQ_INVERTED
				: LR20XX_RADIO_LORA_IQ_STANDARD,
		};
		lr20xx_radio_lora_set_packet_params(ctx, &pkt);
		CHECK_CMD(ctx, "restart_rx set_pkt_params");
	}

	/* "If the device is already in Rx mode, the command fails as timeout
	 * cannot be updated" (datasheet, SetRx) — and in continuous mode the
	 * chip never leaves Rx, it just keeps searching. So after a packet there
	 * is nothing to re-arm: issuing SetRx only earns a CMD_FAIL, whose
	 * CmdError comes back through DIO1 as another unhandled IRQ.
	 *
	 * Skipping it also keeps the receiver on air. Forcing a mode change
	 * would cost the HF oscillator restart (702 us) plus PLL lock (32 us)
	 * on the path taken after every single packet. */
	if (!data->rx_duty_cycle_enabled) {
		lr20xx_system_stat2_t s2 = {0};

		if (lr20xx_system_get_status(ctx, NULL, &s2, NULL) == LR20XX_STATUS_OK &&
		    s2.chip_mode == LR20XX_SYSTEM_CHIP_MODE_RX) {
			data->in_rx_mode = true;
			return 0;
		}
	}

	if (data->rx_duty_cycle_enabled) {
		lr20xx_system_stat1_t s1 = { 0 };

		/* SetStandby first.  A header error does not terminate the
		 * duty-cycle loop — DS §6.3.8 lists exactly three terminators
		 * (packet reception, a host SetStandby, an NSS wake from sleep)
		 * and a header error is none of them — so the chip can still be
		 * cycling here.  Re-arming then drives an NSS edge into a live
		 * cycle, and the datasheet is explicit for that case: on a wake
		 * "by the device with a falling edge of NSS. A SetStandby
		 * command should also be sent, to avoid the race conditions".
		 * Observed on hardware without it: latched CMD_ERROR, DIO1 held
		 * high, five strikes to a reset, ~88 ms deaf per noise header
		 * error.  Near-free when already in STDBY_RC.
		 * See ARCHITECTURE.md 5.5.1. */
		lr20xx_system_set_standby_mode(ctx,
					       LR20XX_SYSTEM_STANDBY_MODE_RC);

		/* Safe to poll: the standby above ended any live cycle, so this
		 * NSS edge has nothing left to disturb.  If the chip would not
		 * even take a SetStandby, arming a duty cycle on top of it is
		 * pointless — say so and let the caller escalate. */
		if (lr20xx_system_get_status(ctx, &s1, NULL, NULL) ==
			    LR20XX_STATUS_OK &&
		    s1.command_status != 2 && s1.command_status != 3) {
			LOG_WRN("restart_rx: standby rejected (cmd=%d) before "
				"duty-cycle re-arm", s1.command_status);
			return -EIO;
		}

		lr20xx_apply_rx_duty_cycle(data);
		data->in_rx_mode = true;
		return 0;
	}

	lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);
	CHECK_CMD(ctx, "restart_rx set_rx");
	data->in_rx_mode = true;

	/* No duty cycle armed, so polling is free of the NSS hazard. */
	{
		lr20xx_system_stat2_t s2 = { 0 };

		if (lr20xx_system_get_status(ctx, NULL, &s2, NULL) ==
			    LR20XX_STATUS_OK &&
		    s2.chip_mode != LR20XX_SYSTEM_CHIP_MODE_RX) {
			LOG_WRN("restart_rx: chip is in mode %d, not RX",
				s2.chip_mode);
			return -EIO;
		}
	}
	return 0;
}

/* Is the receiver still live?  Conservative: true (leave it alone) whenever it
 * cannot tell.  Never probes an armed duty cycle — GetStatus asserts NSS and an
 * NSS edge terminates the cycle (DS 6.3.8).  Caller holds spi_mutex. */
static bool lr20xx_rx_confirmed_live(struct lr20xx_data *data)
{
	lr20xx_system_stat2_t s2 = {0};

	/* Never probe an armed duty cycle.  GetStatus asserts NSS and an NSS
	 * falling edge terminates the cycle (DS §6.3.8) — that is precisely the
	 * damage this path exists to avoid causing.  A duty cycle also re-arms
	 * itself from every terminal event, so it needs no help here. */
	if (data->rx_duty_cycle_enabled) {
		return true;
	}

	if (lr20xx_system_get_status(&data->hal_ctx, NULL, &s2, NULL) !=
	    LR20XX_STATUS_OK) {
		return false;
	}
	return s2.chip_mode == LR20XX_SYSTEM_CHIP_MODE_RX;
}

/* ── DIO1 IRQ handler (work queue, thread context) ──────────────────── */

/* IRQ bit 16 means "call get_errors" per the datasheet; bit 17 is a rejected
 * host command. Both are useless on their own — the error word names the
 * actual fault. Throttled: these can fire every RX restart. */
static void lr20xx_log_chip_errors(void *ctx, const char *what)
{
	static const struct {
		uint16_t mask;
		const char *name;
	} err_names[] = {
		{ BIT(0),  "HF_XOSC_START" },
		{ BIT(1),  "LF_XOSC_START" },
		{ BIT(2),  "PLL_LOCK" },
		{ BIT(3),  "LF_RC_CALIB" },
		{ BIT(4),  "HF_RC_CALIB" },
		{ BIT(5),  "PLL_CALIB" },
		{ BIT(6),  "AAF_CALIB" },
		{ BIT(7),  "IMG_CALIB" },
		{ BIT(8),  "CHIP_BUSY" },
		{ BIT(9),  "RXFREQ_NO_FRONT_END_CALIB" },
		{ BIT(10), "MEAS_UNIT_ADC_CALIB" },
		{ BIT(11), "PA_OFFSET_CALIB" },
		{ BIT(12), "PPF_CALIB" },
		{ BIT(13), "SRC_CALIB" },
		{ BIT(14), "SRC_SATURATION_CALIB" },
		{ BIT(15), "SRC_TOLERANCE_CALIB" },
	};
	static uint32_t seen;
	lr20xx_system_errors_t errs = 0;

	if (lr20xx_system_get_errors(ctx, &errs) != LR20XX_STATUS_OK) {
		LOG_ERR("chip %s error; get_errors() failed too", what);
		return;
	}

	/* First few in full, then one in 64 — the console cannot keep up */
	if (seen++ >= 4 && (seen & 0x3F) != 0) {
		return;
	}

	LOG_ERR("chip %s error: errors=0x%04x", what, errs);
	for (int i = 0; i < (int)ARRAY_SIZE(err_names); i++) {
		if (errs & err_names[i].mask) {
			LOG_ERR("    %s", err_names[i].name);
		}
	}
}

/* Fold one packet's frequency error into the running stats.
 *
 * Sanity bound first: the SDK sign-extends a 24-bit field, so a garbage read
 * would land anywhere in +/-8 MHz.  Anything past +/-200 kHz is not a receivable
 * LoRa packet at the bandwidths this driver uses (the widest is 500 kHz, and
 * the demodulator's own capture range is a fraction of that), so a value out
 * there means the three extra GetLoraPacketStatus bytes are not what v2.0.2
 * thinks they are on this firmware.  Drop it rather than poison the mean, and
 * say so once. */
#define LR20XX_FREQ_OFFSET_SANE_HZ 200000

static void lr20xx_track_freq_offset(struct lr20xx_data *data, int32_t off_hz)
{
	if (off_hz > LR20XX_FREQ_OFFSET_SANE_HZ ||
	    off_hz < -LR20XX_FREQ_OFFSET_SANE_HZ) {
		static bool warned;

		if (!warned) {
			warned = true;
			LOG_WRN("freq offset %d Hz is out of range — the extra "
				"GetLoraPacketStatus bytes may not carry it on "
				"this FW; ignoring further outliers", off_hz);
		}
		return;
	}

	data->freq_off_last_hz = off_hz;

	if (data->freq_off_count == 0) {
		data->freq_off_min_hz = off_hz;
		data->freq_off_max_hz = off_hz;
	} else if (off_hz < data->freq_off_min_hz) {
		data->freq_off_min_hz = off_hz;
	} else if (off_hz > data->freq_off_max_hz) {
		data->freq_off_max_hz = off_hz;
	}

	/* Saturate rather than wrap.  The sum is nowhere near trouble — bounded
	 * to +/-200 kHz per packet by the gate above, int64_t needs ~4.6e13
	 * packets — but the uint32_t count wraps at 4.29e9 (~14 years at
	 * 10 pkt/s), and it wraps to *zero*, which the summary below would then
	 * divide by.  Unreachable in practice; undefined if reached.  Freezing
	 * leaves the mean valid and stale, which is a defensible answer; the
	 * per-packet last/min/max above keep updating either way. */
	if (data->freq_off_count == UINT32_MAX) {
		return;
	}
	data->freq_off_sum_hz += off_hz;
	data->freq_off_count++;

	LOG_DBG("freq offset: %d Hz", off_hz);

	/* Periodic summary at INFO.  The per-packet value above is only visible
	 * with CONFIG_LORA_LOG_LEVEL_DBG=y, which makes this whole driver a
	 * firehose (six lines per apply_modem_config alone) and starts dropping
	 * messages on a soak long enough to matter.  The mean is the number
	 * worth watching anyway — one packet's offset is our reference error
	 * plus the transmitter's, so only the average over many peers says
	 * anything about ours.  Every 16 packets keeps it readable in an
	 * ordinary debug.conf build. */
	if ((data->freq_off_count & 0x0F) == 0) {
		LOG_INF("freq offset: mean %d Hz (min %d, max %d, %u pkts)",
			(int)(data->freq_off_sum_hz /
			      (int64_t)data->freq_off_count),
			data->freq_off_min_hz, data->freq_off_max_hz,
			data->freq_off_count);
	}
}

static void lr20xx_dio1_callback(void *user_data);

/* RTC tick backing SetRx timeouts; the field is 24 bits, so the ceiling is
 * 16777215/32768 = 512 s -- three orders above any packet we send. */
#define LR20XX_RTC_FREQ_HZ                  32768U

/* cad_rx_state values -- mirrors the SX126x/LR11xx probe outcome tracker. */
#define LR20XX_CAD_RX_IDLE   0
#define LR20XX_CAD_RX_ARMED  1
#define LR20XX_CAD_RX_PACKET 2
#define LR20XX_CAD_RX_TMOUT  3

static uint32_t lr20xx_max_payload_ms(struct lr20xx_data *data);

/* Resolve an in-flight probe Rx with the terminal event that just arrived.
 * No-op unless one is armed, so the packet path pays one atomic compare. */
static inline bool lr20xx_cad_rx_resolve(struct lr20xx_data *data, int outcome)
{
	return atomic_cas(&data->cad_rx_state, LR20XX_CAD_RX_ARMED, outcome);
}

static void lr20xx_dio1_work_handler(struct k_work *work)
{
	struct lr20xx_data *data = CONTAINER_OF(work, struct lr20xx_data,
						dio1_work);
	const struct lr20xx_config *cfg = data->dev->config;
	void *ctx = &data->hal_ctx;
	bool rx_restarted = false;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Combined get + clear IRQ status */
	lr20xx_system_irq_mask_t irq = 0;
	lr20xx_status_t rc = lr20xx_system_get_and_clear_irq_status(ctx, &irq);

	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("Failed to read IRQ status (rc=%d)", rc);
		goto safety_check;
	}

	if (irq & (LR20XX_SYSTEM_IRQ_ERROR | LR20XX_SYSTEM_IRQ_CMD_ERROR)) {
		lr20xx_log_chip_errors(ctx, (irq & LR20XX_SYSTEM_IRQ_CMD_ERROR)
					    ? "cmd rejected" : "hardware");
		/* Latched until cleared, or every later poll re-reports it */
		lr20xx_system_clear_errors(ctx);
	}

	/* Supply faults, observed passively: these bits latch in the status word
	 * regardless of DIO1 routing, so noticing them here costs nothing and
	 * changes no behaviour.  Deliberately NOT added to the DIO1 mask — no
	 * branch handles them, so they would fall into the safety restart. */
	if (irq & (LR20XX_SYSTEM_IRQ_LOW_BATTERY |
		   LR20XX_SYSTEM_IRQ_PA_OVP_OCP)) {
		uint16_t vbat_mv = 0;

		lr20xx_system_get_vbat(ctx, LR20XX_SYSTEM_VALUE_FORMAT_UNIT,
				       LR20XX_SYSTEM_MEAS_RES_12_BITS, &vbat_mv);
		LOG_ERR("SUPPLY FAULT: %s%s (chip VBAT now %u mV)",
			(irq & LR20XX_SYSTEM_IRQ_LOW_BATTERY) ? "LOW_BATTERY " : "",
			(irq & LR20XX_SYSTEM_IRQ_PA_OVP_OCP) ? "PA_OVP/OCP" : "",
			vbat_mv);
	}

	/* Error-only IRQs are not progress. Resetting the counter on them let
	 * an error that re-fires on every RX restart spin forever, never
	 * reaching the stuck-DIO1 escape hatch below. */
	if (irq & ~(LR20XX_SYSTEM_IRQ_ERROR | LR20XX_SYSTEM_IRQ_CMD_ERROR)) {
		data->dio1_stuck_count = 0;
	}

	/* ── RX done ──
	 * Gated on no error bits: a CRC-failed packet asserts RX_DONE and
	 * CRC_ERROR together on this chip family (same as SX126x/LR11xx),
	 * so an ungated done-first read would deliver corrupted payloads
	 * as valid.  A good packet coalesced with an earlier header error
	 * in the same handler window is dropped too — the aborted packet
	 * may have left bytes in the RX FIFO, misaligning the read.  The
	 * error branch below owns the window instead. */
	/* Ground truth for a probe Rx: anything proving a transmitter was
	 * there.  A CRC or header error counts as much as a clean packet. */
	if (irq & (LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_CRC_ERROR |
		   LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR)) {
		lr20xx_cad_rx_resolve(data, LR20XX_CAD_RX_PACKET);
	}

	if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) &&
	    !(irq & (LR20XX_SYSTEM_IRQ_CRC_ERROR |
		     LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR))) {
		uint16_t pkt_len = 0;
		lr20xx_radio_common_get_rx_packet_length(ctx, &pkt_len);

		if (pkt_len > 0 && pkt_len <= 255) {
			lr20xx_radio_lora_packet_status_t pkt_stat;
			lr20xx_radio_lora_get_packet_status(ctx, &pkt_stat);

			lr20xx_radio_fifo_read_rx(ctx, data->rx_buf,
						  (uint16_t)pkt_len);

			/* Restart RX before firing callback */
			lr20xx_restart_rx(data);
			rx_restarted = true;

			/* When SNR < 0, use signal RSSI for a more
			 * accurate reading on weak links.
			 *
			 * This branch is in fact DEAD, and deliberately left
			 * that way.  Per the vendor struct doc
			 * (lr20xx_radio_lora_types.h), a negative snr_pkt_raw
			 * gives rssi_signal = rssi_pkt + snr/4, i.e. strictly
			 * lower — so the `>` can never hold.  The LR11xx driver
			 * carries the identical dead branch.  Removing it here
			 * alone would shift reported RSSI on this radio relative
			 * to the others (nothing would change, but the next
			 * reader would assume it had); fix both or neither. */
			int16_t rssi = pkt_stat.rssi_pkt_in_dbm;
			int8_t snr = ((int8_t)pkt_stat.snr_pkt_raw + 2) >> 2;

			if (snr < 0 &&
			    pkt_stat.rssi_signal_pkt_in_dbm > rssi) {
				rssi = pkt_stat.rssi_signal_pkt_in_dbm;
			}

			/* Carrier frequency error for this packet.  Driver
			 * v2.0.2 reads 9 bytes from GetLoraPacketStatus where DS
			 * rev 2.1 Table 9-13 documents only 6 — this field comes
			 * from the three extra bytes, so the driver is ahead of
			 * the datasheet here.  RSSI/SNR/length decode from
			 * rbuffer[0..5] and are unaffected either way; treat a
			 * wildly implausible offset as evidence the extra bytes
			 * are junk on this FW rather than as a real reading. */
			lr20xx_track_freq_offset(data,
						 pkt_stat.freq_offset_hz);

			k_mutex_unlock(&data->spi_mutex);

			if (data->async_rx_cb) {
				data->async_rx_cb(data->dev, data->rx_buf,
						  (uint8_t)pkt_len,
						  rssi, snr,
						  data->async_rx_user_data);
			}
			return;
		}

		LOG_WRN("RX: invalid len %d", pkt_len);
		lr20xx_restart_rx(data);
		rx_restarted = true;
	}

	/* ── CAD done ── */
	if (irq & LR20XX_SYSTEM_IRQ_CAD_DONE) {
		bool detected = (irq & LR20XX_SYSTEM_IRQ_CAD_DETECTED) != 0;

		/* Stamp as early as possible in the
		 * handler — everything after this point is the host round-trip
		 * being measured.  Only a free channel leads to a transmit, so
		 * only that case is worth stamping. */
		data->cad_done_cycles = detected ? 0U : k_cycle_get_32();
		if (!detected && data->cad_done_cycles == 0U) {
			data->cad_done_cycles = 1U;   /* 0 is the "none" sentinel */
		}

		LOG_DBG("CAD done: %s", detected ? "activity" : "free");

		if (data->cad_probe_rx && detected) {
			/* CAD_ONLY has returned the chip to STDBY_RC.  Arm the
			 * follow-on Rx with the same call the normal receive path
			 * uses, bounded by max-length-packet airtime, so whichever
			 * terminal IRQ follows (RX_DONE / CRC_ERROR / HEADER_ERROR /
			 * TIMEOUT) is the probe's ground truth.  RTC steps, not the
			 * 524 ms cad_timeout field. */
			uint32_t steps = (uint32_t)(((uint64_t)lr20xx_max_payload_ms(data)
						     * LR20XX_RTC_FREQ_HZ) / 1000U);

			if (steps > 0x00FFFFFFU) {
				steps = 0x00FFFFFFU;
			}
			atomic_set(&data->cad_rx_state, LR20XX_CAD_RX_ARMED);
			lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(ctx, steps);
			data->in_rx_mode = true;
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
	if (irq & LR20XX_SYSTEM_IRQ_TX_DONE) {
		LOG_DBG("TX done");
		data->tx_active = false;

		lr20xx_start_rx(data, cfg);
		rx_restarted = true;

		if (data->tx_signal) {
			k_poll_signal_raise(data->tx_signal, 0);
		}
	}

	/* ── Timeout ── */
	if (irq & LR20XX_SYSTEM_IRQ_TIMEOUT) {
		/* The probe's own Rx window expiring with nothing decoded: the
		 * detection had no packet behind it. */
		lr20xx_cad_rx_resolve(data, LR20XX_CAD_RX_TMOUT);
		if (data->tx_active) {
			/* The chip's Tx safeguard fired.  DS §6.3.6: "the
			 * transmission is stopped prematurely" — the packet is
			 * gone and TX_DONE will never arrive.  Leaving tx_active
			 * set here (as this branch used to) made the whole event
			 * invisible: no branch ran, nothing was logged above DBG,
			 * and the radio sat in the post-TX state until the host's
			 * own wait expired.  Say it and put the receiver back on
			 * air, same as the TX_DONE path.
			 *
			 * tx_signal is deliberately NOT raised: the C++ wait
			 * thread counts a raised signal as a completed send and
			 * would increment packets_sent for a packet that never
			 * left.  Its own timeout owns the accounting. */
			LOG_ERR("TX timeout — chip stopped the transmission, "
				"packet lost");
			data->tx_active = false;
			lr20xx_start_rx(data, cfg);
			rx_restarted = true;
		} else {
			/* Under a duty cycle this is the false-preamble case:
			 * the preamble-restarted rx_max_time + cycle_time window
			 * expired with no packet, so the chip left the loop.
			 * Counting it is what makes `get dc.restarts` mean
			 * something on this radio. */
			if (data->rx_duty_cycle_enabled) {
				atomic_inc(&data->dc_timeout_restarts);
			}
			LOG_DBG("Timeout IRQ — restarting RX");
			if (lr20xx_restart_rx(data) < 0) {
				/* The light re-arm could not get the chip back on
				 * air.  Escalate to the full path, which forces
				 * standby, reprograms the modem and re-issues
				 * SetRx from scratch — the LR20xx counterpart of
				 * the SX126x's retry-from-sleep.  If that fails
				 * too the chip is wedged, and the stuck-DIO1
				 * counter below still reaches its reset. */
				LOG_WRN("Timeout: light re-arm failed — full RX restart");
				lr20xx_start_rx(data, cfg);
			}
			rx_restarted = true;
		}
	}

	/* ── CRC / Header error ──
	 * Plain `CRC || HDR` — no SYNC_WORD_HEADER_VALID gate.  IRQ status
	 * is bulk-cleared on every handler entry, so a set header error
	 * always belongs to this window; a SYNC_VALID bit latched by
	 * another packet in the same window must not suppress it. */
	if (irq & (LR20XX_SYSTEM_IRQ_CRC_ERROR |
		   LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR)) {
		LOG_WRN("RX error: CRC=%d HDR=%d RXDONE=%d",
			(irq & LR20XX_SYSTEM_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) ? 1 : 0,
			(irq & LR20XX_SYSTEM_IRQ_RX_DONE) ? 1 : 0);

		/* Drop whatever the failed (or coalesced) packet left in
		 * the RX FIFO so the next packet's read starts aligned —
		 * lr20xx_restart_rx does not clear it (only full start_rx
		 * does). */
		lr20xx_radio_fifo_clear_rx(ctx);

		if (!data->tx_active) {
			lr20xx_restart_rx(data);
			rx_restarted = true;
		}

		k_mutex_unlock(&data->spi_mutex);

		if (data->async_rx_cb) {
			data->async_rx_cb(data->dev, NULL, 0, 0, 0,
					  data->async_rx_user_data);
		}
		return;
	}

	/* ── Header valid ──
	 * A packet is past sync word and header CRC and is now in its payload
	 * phase.  Stamp the latch lr20xx_is_receiving() answers from; bounded
	 * there by lr20xx_max_payload_ms(), and cleared by the terminal event's
	 * lr20xx_reset_rx_busy_signals().
	 *
	 * Gated on no terminal bit in this same pass -- those branches above own
	 * the outcome and have already reset the signals, so stamping after them
	 * would re-arm a gate for a packet that is finished.  (The header-error
	 * branch above returns, so it cannot reach here at all.)
	 *
	 * rx_restarted, for the same reason the CAD_RX branch sets it: nothing
	 * needs restarting, the chip is mid-packet, and letting the safety net
	 * below fire here would destroy the very packet this branch exists to
	 * protect -- the failure that kept this IRQ off DIO1 in the first
	 * place. */
	if ((irq & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) &&
	    !(irq & (LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_CRC_ERROR |
		     LR20XX_SYSTEM_IRQ_TIMEOUT))) {
		uint32_t now = k_uptime_get_32();

		/* 1 as the "set" sentinel if k_uptime is 0 right after boot. */
		data->header_seen_at_ms = (now == 0) ? 1U : now;
		/* The latch is the truth source now; a preamble timestamp left
		 * behind would outlive it and be read as a live grace window. */
		data->preamble_seen_at_ms = 0;
		rx_restarted = true;
	}

safety_check:
	/* A zero IRQ word means the edge was already consumed — a duplicate
	 * re-submit, or the restart path cleared it. Nothing failed, so do not
	 * tear RX down and do not count it toward the stuck-DIO1 reset. */
	if (irq == 0) {
		goto edge_recheck;
	}

	/* Error-only wake: the errors were already logged and cleared, so DIO1
	 * drops and edge_recheck ends the cycle.  Restart RX only if the receiver
	 * demonstrably stopped — blindly restarting re-issues the command that was
	 * refused, which is what turned one refusal into a hardware reset.
	 * dio1_stuck_count is still not reset here, so a genuinely stuck chip
	 * still reaches the escape hatch. */
	if ((irq & ~(LR20XX_SYSTEM_IRQ_ERROR |
		     LR20XX_SYSTEM_IRQ_CMD_ERROR)) == 0) {
		if (!rx_restarted && data->in_rx_mode && !data->tx_active &&
		    !lr20xx_rx_confirmed_live(data)) {
			LOG_WRN("DIO1: error-only IRQ (0x%08x) left the chip "
				"out of RX — restarting", irq);
			lr20xx_restart_rx(data);
		}
		goto edge_recheck;
	}

	if (!rx_restarted && data->in_rx_mode && !data->tx_active) {
		LOG_WRN("DIO1 safety: no IRQ handled (0x%08x rc=%d), "
			"restarting RX", irq, rc);
		lr20xx_restart_rx(data);
	}

edge_recheck:
	/* Edge-triggered DIO1: if still HIGH, re-submit for pending flags.
	 * Guard against stuck DIO1: after 5 empty cycles, hardware reset. */
	if (gpio_pin_get_dt(&data->hal_ctx.dio1)) {
		data->dio1_stuck_count++;
		if (data->dio1_stuck_count >= 5) {
			LOG_ERR("DIO1 stuck HIGH for %d cycles — "
				"hardware reset", data->dio1_stuck_count);
			data->dio1_stuck_count = 0;
			lr20xx_hardware_reset(data, cfg);
			lr20xx_start_rx(data, cfg);
		} else {
			k_work_submit_to_queue(&data->dio1_wq,
					       &data->dio1_work);
		}
	} else {
		data->dio1_stuck_count = 0;
	}

	k_mutex_unlock(&data->spi_mutex);
}

static void lr20xx_dio1_callback(void *user_data)
{
	struct lr20xx_data *data = (struct lr20xx_data *)user_data;
	k_work_submit_to_queue(&data->dio1_wq, &data->dio1_work);
}

/* Forward declaration */
static int lr20xx_hw_init(struct lr20xx_data *data,
			  const struct lr20xx_config *cfg);

/* ── Driver API: config ─────────────────────────────────────────────── */

static int lr20xx_lora_config(const struct device *dev,
			      const struct lora_modem_config *config)
{
	struct lr20xx_data *data = dev->data;

	if (!data->hw_initialized) {
		int ret = lr20xx_hw_init(data, dev->config);
		if (ret != 0) {
			LOG_ERR("Hardware init failed: %d", ret);
			return ret;
		}
	}

	memcpy(&data->modem_cfg, config, sizeof(*config));
	data->configured = true;

	/* Front-end calibration at the operating frequency.  This is the ONE
	 * place it belongs on a configured radio: DS §6.4.2 keeps the values on
	 * chip across every sleep this driver issues, and Semtech's own
	 * ral_lr20xx_init() calibrates at init and never on the Tx/Rx path. */
	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Standby first — CalibFE "does not work if device is in Rx or Tx mode"
	 * (DS §6.4.2).  On the reconfigure path the chip IS still in Rx: the C++
	 * layer's hwCancelReceive() lowers lora_recv_async(NULL)'s bookkeeping
	 * without commanding the chip, and continuous RX only ends when the host
	 * changes the mode (DS §6.3.5).  Without this the calibration is refused
	 * for every `set freq/sf/bw/cr`, tempradio set and tempradio clear, and
	 * refused invisibly — see the note in lr20xx_calibrate_front_end().
	 * Free at boot, where hw_init() has already left the chip in STDBY_RC. */
	lr20xx_system_set_standby_mode(&data->hal_ctx,
				       LR20XX_SYSTEM_STANDBY_MODE_RC);

	lr20xx_calibrate_front_end(&data->hal_ctx, config->frequency);

	DUMP_CHIP_STATE(data, "config-FEcal");
	k_mutex_unlock(&data->spi_mutex);

	LOG_DBG("config: %uHz SF%d BW%d CR%d pwr=%d tx=%d",
		config->frequency, config->datarate, config->bandwidth,
		config->coding_rate, config->tx_power, config->tx);

	return 0;
}

/* ── Driver API: airtime ────────────────────────────────────────────── */

static uint32_t lr20xx_lora_airtime(const struct device *dev,
				    uint32_t data_len)
{
	struct lr20xx_data *data = dev->data;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	float bw = bw_enum_to_khz(mc->bandwidth) * 1000.0f;
	uint8_t cr = (uint8_t)mc->coding_rate + 4;

	float ts = (float)(1 << sf) / bw;
	/* Must be the predicate the modem is actually programmed with, not an
	 * SF-based approximation of it — see lr20xx_ldro_enabled(). */
	int de = lr20xx_ldro_enabled(sf, (uint32_t)bw) ? 1 : 0;
	float n_payload = 8.0f + fmaxf(
		ceilf((8.0f * data_len - 4.0f * sf + 28.0f + 16.0f) /
		      (4.0f * (sf - 2.0f * de))) * cr,
		0.0f);
	float t_preamble = (mc->preamble_len + 4.25f) * ts;
	float t_payload = n_payload * ts;

	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

/* ── Driver API: send_async ─────────────────────────────────────────── */

static int lr20xx_lora_cad(const struct device *dev, k_timeout_t timeout);

/* Blocking-CAD wait budget scaled to the actual CAD duration:
 * nSym * Tsym + startup radio-side, plus IRQ latency margin.  A fixed
 * 200 ms fits 2-symbol CAD everywhere but is exceeded by 4-symbol CAD
 * on slow presets (SF12 @ 62.5 kHz = ~262 ms). */
static uint32_t lr20xx_cad_timeout_ms(struct lr20xx_data *data)
{
	struct lora_modem_config *mc = &data->modem_cfg;
	uint8_t sf = (uint8_t)mc->datarate;
	/* Shared helper so the budget is sized for the window actually
	 * programmed — this used to default to 2 symbols where lr20xx_do_cad()
	 * defaults to 4, which would have under-budgeted the wait if
	 * buildModemConfig() ever stopped setting symbol_num explicitly. */
	uint8_t symb_nb = lr20xx_cad_symb_nb(mc);
	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(mc->bandwidth) * 1000.0f);

	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 200;
	}

	uint32_t tsym_us = ((1UL << sf) * 1000000UL) / bw_hz;
	/* +1 symbol covers radio startup + internal processing tail */
	uint32_t ms = ((symb_nb + 1U) * tsym_us) / 1000U + 100U;

	return MAX(ms, 200U);
}

/* ── Hardware CAD_LBT ───────────────────────────────────────────────────
 *
 * CadExitMode = 0x10 (DS Table 6-18): the chip runs the CAD and, on a clear
 * channel, goes straight to Tx with no host round-trip.  Payload and packet
 * params must be staged BEFORE SetLoraCAD, and DIO1 must stay enabled across it
 * (CAD_DONE and TX_DONE both arrive on it).  Busy exits to STDBY_RC, so the host
 * still re-arms RX there.
 *
 * cad_timeout doubles as the Tx timeout: 24 bits of 32 MHz periods = 524 ms max.
 * Longer transmits take the classic host path instead of being truncated.
 * See ARCHITECTURE.md 5.5.1. */
#define LR20XX_CAD_LBT_MAX_TX_TIMEOUT_STEPS 0x00FFFFFFU
#define LR20XX_CAD_LBT_STEPS_PER_MS         32000U

/* The OTHER time base on this chip.  SetTx/SetRx/SetRxDutyCycle timeouts are
 * counted in 32.768 kHz RTC periods (DS §6.3.17: "expressed in periods of the
 * 32.768kHz RTC"), not the 32 MHz periods cad_timeout uses above — the two sit
 * three lines apart here precisely so nobody reaches for the wrong one.  Both
 * fields are 24 bits, so the RTC one tops out at 16777215/32768 = 512 s. */
#define LR20XX_RTC_STEP_MAX                 0x00FFFFFFU
/* Never shorten the Tx safeguard below what shipped before it was scaled. */
#define LR20XX_TX_TIMEOUT_FLOOR_MS          5000U

static int lr20xx_do_cad(struct lr20xx_data *data);

static int lr20xx_lora_send_cad_lbt(const struct device *dev,
				    uint8_t *buf, uint32_t data_len,
				    struct k_poll_signal *async,
				    uint32_t tx_timeout_ms)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	void *ctx = &data->hal_ctx;
	bool was_in_rx = data->in_rx_mode;
	int ret;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Same reasoning as the normal path: async_rx_cb stays registered. */
	data->in_rx_mode = false;
	lr20xx_reset_rx_busy_signals(data);

	/* DIO1 deliberately NOT disabled — CAD_DONE and TX_DONE both arrive on
	 * it, and with CAD_LBT there is no host step in between to re-enable. */

	if (lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC)
	    != LR20XX_STATUS_OK) {
		LOG_ERR("CAD_LBT standby failed — HW reset");
		lr20xx_hardware_reset(data, cfg);
	}

	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	/* Full TX-side modem setup must complete before the CAD. */
	lr20xx_apply_modem_config(data, cfg, true);

	lr20xx_radio_lora_pkt_params_t pkt = {
		.preamble_len_in_symb = data->modem_cfg.preamble_len,
		.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = (uint8_t)data_len,
		.crc = data->modem_cfg.packet_crc_disable
			? LR20XX_RADIO_LORA_CRC_DISABLED
			: LR20XX_RADIO_LORA_CRC_ENABLED,
		.iq = data->modem_cfg.iq_inverted
			? LR20XX_RADIO_LORA_IQ_INVERTED
			: LR20XX_RADIO_LORA_IQ_STANDARD,
	};
	lr20xx_radio_lora_set_packet_params(ctx, &pkt);
	lr20xx_radio_fifo_write_tx(ctx, buf, (uint16_t)data_len);

	/* Armed before SetLoraCAD: on a clear channel the chip starts Tx
	 * autonomously, so TX_DONE can land before this function returns. */
	data->tx_signal = async;
	data->tx_active = true;

	data->cad_lbt_exit_tx = true;
	data->cad_lbt_tx_timeout = tx_timeout_ms * LR20XX_CAD_LBT_STEPS_PER_MS;

	k_sem_reset(&data->cad_sem);
	data->cad_result = -ETIMEDOUT;
	data->cad_cb = NULL;

	ret = lr20xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	if (ret < 0) {
		goto abort;
	}

	/* CAD_DONE still fires in CAD_LBT mode; it is how the host learns the
	 * outcome.  Budget = the CAD window plus the transmit it may have
	 * already started. */
	ret = k_sem_take(&data->cad_sem,
			 K_MSEC(lr20xx_cad_timeout_ms(data) + tx_timeout_ms));
	if (ret == -EAGAIN) {
		LOG_WRN("CAD_LBT: no CAD_DONE within budget — falling back");
		ret = -EIO;
		goto abort;
	}

	if (data->cad_result > 0) {
		/* Busy: the chip is parked in the STDBY_RC fallback and never
		 * transmitted.  Nothing raised tx_signal, so clear the TX state
		 * and re-arm RX exactly as the classic busy path does. */
		LOG_DBG("CAD_LBT: channel busy");
		data->tx_active = false;
		data->tx_signal = NULL;
		if (was_in_rx || data->rx_duty_cycle_enabled) {
			k_mutex_lock(&data->spi_mutex, K_FOREVER);
			lr20xx_start_rx(data, cfg);
			k_mutex_unlock(&data->spi_mutex);
		}
		return -EBUSY;
	}

	/* Clear: the chip is transmitting (or has already finished).  TX_DONE
	 * drives the rest, same as the normal path. */
	LOG_INF("CAD_LBT: chip took CAD->TX itself (tx_timeout=%u ms)",
		tx_timeout_ms);
	return 0;

abort:
	data->tx_active = false;
	data->tx_signal = NULL;
	data->cad_lbt_exit_tx = false;
	data->cad_lbt_tx_timeout = 0;
	if (was_in_rx || data->rx_duty_cycle_enabled) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		lr20xx_start_rx(data, cfg);
		k_mutex_unlock(&data->spi_mutex);
	}
	return ret;
}

static int lr20xx_lora_send_async(const struct device *dev,
				  uint8_t *buf, uint32_t data_len,
				  struct k_poll_signal *async)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	void *ctx = &data->hal_ctx;

	if (!data->configured) return -EINVAL;
	if (data->tx_active) return -EBUSY;
	if (data_len > 255 || data_len == 0) return -EINVAL;

	/* Own the chip for the whole transmit, LBT CAD included — the CAD below
	 * commands it too, so this has to come first. */
	lr20xx_dc_takeover(data);

	/* Hand CAD->TX to the chip where the transmit fits
	 * inside cad_timeout's 524 ms ceiling.  Above it, fall through to the
	 * classic two-step path rather than truncate the packet. */
	if (data->modem_cfg.cad.mode == LORA_CAD_MODE_LBT) {
		uint32_t airtime_ms = lr20xx_lora_airtime(dev, data_len);
		uint32_t tx_timeout_ms = airtime_ms + (airtime_ms / 4U) + 50U;

		if ((uint64_t)tx_timeout_ms * LR20XX_CAD_LBT_STEPS_PER_MS <=
		    LR20XX_CAD_LBT_MAX_TX_TIMEOUT_STEPS) {
			return lr20xx_lora_send_cad_lbt(dev, buf, data_len,
							async, tx_timeout_ms);
		}
		LOG_DBG("CAD_LBT: airtime %u ms needs %u ms Tx timeout, over the "
			"524 ms cad_timeout ceiling — using host CAD->TX",
			airtime_ms, tx_timeout_ms);
	}

	/* LBT: perform blocking CAD before transmitting.  On CAD-busy, restore
	 * RX in-driver before returning -EBUSY so the C++ layer doesn't have
	 * to do a full cancel-then-restart round-trip.  lr20xx_lora_cad
	 * transitions the chip to STANDBY and clears data->in_rx_mode as
	 * part of running CAD; capture the pre-CAD state to know whether
	 * to re-arm. */
	if (data->modem_cfg.cad.mode == LORA_CAD_MODE_LBT) {
		bool was_in_rx = data->in_rx_mode;
		int cad_ret;

		/* Discard any stamp left by a CAD that was not this transmit's.
		 * The CAD_DONE handler stamps cad_done_cycles on every free CAD
		 * — adaptive-CAD probes every ~15 s, and CAD_LBT, neither of
		 * which consumes it — but the only reader is the latency log
		 * below.  Without this the reported CAD->TX gap can be measured
		 * from a probe that ran seconds earlier, which is worse than not
		 * reporting it: it is a plausible number that is simply wrong. */
		data->cad_done_cycles = 0U;

		cad_ret = lr20xx_lora_cad(dev,
					  K_MSEC(lr20xx_cad_timeout_ms(data)));
		if (cad_ret > 0) {
			LOG_DBG("LBT: channel busy");
			/* Re-arm whenever there was anything to re-arm.  The
			 * async_rx_cb condition was safe while a busy CAD only
			 * cost the RX mode, but lr20xx_dc_takeover() above has
			 * already stood the duty cycle down by this point — so
			 * bailing out without restarting leaves the radio
			 * parked in standby, deaf, with nothing scheduled to
			 * bring it back.  rx_duty_cycle_enabled covers the case
			 * the callback check would miss. */
			if (was_in_rx || data->rx_duty_cycle_enabled) {
				k_mutex_lock(&data->spi_mutex, K_FOREVER);
				lr20xx_start_rx(data, cfg);
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

	/* async_rx_cb deliberately NOT cleared: the TX_DONE handler puts the chip
	 * back on air before the C++ TX-wait thread re-registers it, and packets
	 * arriving in that window were being dropped uncounted.  in_rx_mode and
	 * tx_active already gate the RX paths. */
	data->in_rx_mode = false;
	lr20xx_reset_rx_busy_signals(data);

	lr20xx_hal_disable_dio1_irq(&data->hal_ctx);

	/* Standby */
	lr20xx_status_t rc = lr20xx_system_set_standby_mode(ctx,
							    LR20XX_SYSTEM_STANDBY_MODE_RC);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("TX standby failed — HW reset");
		lr20xx_hardware_reset(data, cfg);
	}

	/* Clear errors before modem config */
	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	lr20xx_apply_modem_config(data, cfg, true);

	/* Set TX-specific packet length */
	lr20xx_radio_lora_pkt_params_t pkt = {
		.preamble_len_in_symb = data->modem_cfg.preamble_len,
		.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = (uint8_t)data_len,
		.crc = data->modem_cfg.packet_crc_disable
			? LR20XX_RADIO_LORA_CRC_DISABLED
			: LR20XX_RADIO_LORA_CRC_ENABLED,
		.iq = data->modem_cfg.iq_inverted
			? LR20XX_RADIO_LORA_IQ_INVERTED
			: LR20XX_RADIO_LORA_IQ_STANDARD,
	};
	lr20xx_radio_lora_set_packet_params(ctx, &pkt);

	/* Write to TX FIFO */
	lr20xx_radio_fifo_write_tx(ctx, buf, (uint16_t)data_len);

	/* Clear ALL errors + IRQs right before set_tx */
	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	data->tx_signal = async;
	data->tx_active = true;

	/* CAD_DONE -> carrier-up, the gap CAD_LBT would
	 * close.  Reported in microseconds; k_cycle_get_32() wraps, but the
	 * unsigned subtraction is correct across one wrap and the interval is
	 * milliseconds against a 32-bit counter, so a double wrap is not
	 * reachable here. */
	if (data->cad_done_cycles != 0U) {
		uint32_t d = k_cycle_get_32() - data->cad_done_cycles;

		LOG_INF("CAD->TX latency: %u us",
			(unsigned)k_cyc_to_us_near32(d));
		data->cad_done_cycles = 0U;
	}

	/* Chip-side Tx safeguard — DS §6.3.6: "Tx timeout can be used as a
	 * safeguard in case transmission fails and TxDone interrupt never
	 * occurs", and when it fires "the transmission is stopped prematurely".
	 * So it has to exceed real airtime, and a fixed budget cannot: at
	 * SF12/BW62.5 a 31-byte packet already needs more than 5 s, and at
	 * SF12/BW31.25 so does a 6-byte one.  Scaled from the same airtime the
	 * CAD_LBT path uses, floored at the previous 5000 ms so no preset that
	 * works today gets a tighter deadline.
	 *
	 * Programmed in RTC steps rather than through the SDK's millisecond
	 * wrapper: that wrapper computes `ms * 32768` in uint32 and overflows
	 * above 131 071 ms, which is reachable here (SF12/BW7.81 at 255 bytes is
	 * ~229 s of airtime).  Saturating at the 24-bit field maximum is 512 s,
	 * comfortably past any airtime this chip can produce. */
	{
		uint32_t tx_air_ms = lr20xx_lora_airtime(dev, data_len);
		uint32_t tx_tmo_ms = tx_air_ms + (tx_air_ms / 4U) + 500U;
		uint64_t tx_tmo_steps;

		if (tx_tmo_ms < LR20XX_TX_TIMEOUT_FLOOR_MS) {
			tx_tmo_ms = LR20XX_TX_TIMEOUT_FLOOR_MS;
		}
		tx_tmo_steps = ((uint64_t)tx_tmo_ms * LR20XX_RTC_FREQ_HZ) / 1000U;
		if (tx_tmo_steps > LR20XX_RTC_STEP_MAX) {
			tx_tmo_steps = LR20XX_RTC_STEP_MAX;
		}

		LOG_DBG("SET_TX: airtime=%u ms, timeout=%u ms (%u steps)",
			tx_air_ms, tx_tmo_ms, (uint32_t)tx_tmo_steps);
		lr20xx_radio_common_set_tx_with_timeout_in_rtc_step(
			ctx, (uint32_t)tx_tmo_steps);
	}

	/* Command status here is SET_TX's own (2=accepted, 1=rejected,
	 * 0=not executed) and the mode should have left standby. */
	DUMP_CHIP_STATE(data, "post-SET_TX");

	k_mutex_unlock(&data->spi_mutex);

	return 0;
}

/* ── Driver API: send (sync) ────────────────────────────────────────── */

static int lr20xx_lora_send(const struct device *dev,
			    uint8_t *buf, uint32_t data_len)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);

	int ret = lr20xx_lora_send_async(dev, buf, data_len, &done);
	if (ret < 0) return ret;

	uint32_t air_time = lr20xx_lora_airtime(dev, data_len);
	ret = k_poll(&evt, 1, K_MSEC(2 * air_time + 1000));
	if (ret < 0) {
		LOG_ERR("TX sync timeout");
		return ret;
	}

	return 0;
}

/* ── Driver API: recv_async ─────────────────────────────────────────── */

static int lr20xx_lora_recv_async(const struct device *dev,
				  lora_recv_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;

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

	/* Idempotent fast path: the chip is already in continuous RX, so there
	 * is nothing to program — refresh the callback and leave the receiver on
	 * air.  The usual caller is LoRaRadioBase::startSendRaw()'s failure path:
	 * the LBT branch of send_async has already run lr20xx_start_rx() before
	 * returning -EBUSY, so without this every CAD-busy verdict paid a SECOND
	 * full rebuild (standby -> ClearRxFifo -> apply_modem_config -> SetRx) on
	 * top of the driver's own.  startSendRaw's comment has asserted this was
	 * "a no-op there" ever since the SX126x gained its Phase 2 fast path; on
	 * this driver it was not, until now.
	 *
	 * rx_duty_cycle_enabled is part of the guard, not decoration.  This is
	 * the CONTINUOUS entry point and it clears that flag below, so a caller
	 * moving a duty-cycled radio to continuous RX has to take the full path:
	 * fast-pathing there would leave the cycle running on a chip whose
	 * bookkeeping claims continuous, and nothing would ever stand it down. */
	if (data->in_rx_mode && !data->rx_duty_cycle_enabled) {
		data->async_rx_cb = cb;
		data->async_rx_user_data = user_data;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;
	data->rx_duty_cycle_enabled = false;

	lr20xx_start_rx(data, cfg);

	k_mutex_unlock(&data->spi_mutex);

	return 0;
}

/* ── Driver API: recv (sync) ────────────────────────────────────────── */

static int lr20xx_lora_recv(const struct device *dev, uint8_t *buf,
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

/* ── LR20xx extension API ───────────────────────────────────────────── */

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
static uint32_t lr20xx_rssi_settle_us(struct lr20xx_data *data)
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
static void lr20xx_rssi_settle(struct lr20xx_data *data)
{
	uint32_t settle_us = lr20xx_rssi_settle_us(data);

	k_sleep(K_MSEC(1));
	if (settle_us > 1000U) {
		k_busy_wait(settle_us - 1000U);
	}
}

int16_t lr20xx_get_rssi_inst(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	int16_t rssi = 0;
	uint8_t half_dbm = 0;
	int16_t out = -128;

	/* Non-blocking: a contended bus means the sampler retries next tick.
	 * -128 is the sentinel LoRaRadioBase::triggerNoiseFloorCalibrate wants. */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return -128;
	}
	if (lr20xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return -128;
	}

	bool armed = lr20xx_dc_suspend(data);

	if (armed) {
		/* GetRssiInst reads a live receiver; the stand-down left the chip
		 * in standby, where there is nothing to measure.  Enter
		 * continuous Rx for the reading and hand the cycle back after.
		 * lr20xx_rssi_settle() sizes the front-end settle from the
		 * bandwidth (a flat 1 ms was short of it below BW20.83), and
		 * against a noise-floor interval measured in seconds it costs
		 * well under a hundredth of a percent of airtime. */
		lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			&data->hal_ctx, 0xFFFFFF);
		lr20xx_rssi_settle(data);
	}

	if (lr20xx_radio_common_get_rssi_inst(&data->hal_ctx, &rssi,
					      &half_dbm) == LR20XX_STATUS_OK) {
		out = rssi;
	}

	lr20xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);

	return out;
}

/* Grace period for the PREAMBLE_DETECTED -> SYNC_WORD_HEADER_VALID gap,
 * SF/BW-aware — same formula as the LR11xx and SX126x drivers:
 * (preamble_len + 8) symbols covers worst-case preamble remainder + sync word
 * + header decode with margin. */
static uint32_t lr20xx_preamble_grace_ms(struct lr20xx_data *data)
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

/* Upper bound on the payload phase: max-length packet airtime at the current
 * SF/BW, CR 4/8, LDRO on (the larger symbol count, i.e. the safer bound), +25%
 * +100 ms.  A stuck-state safety net, not a timing mechanism — continuous RX has
 * no symbol timer, so a header whose packet never completes would pin the TX
 * gate until reboot. */
static uint32_t lr20xx_max_payload_ms(struct lr20xx_data *data)
{
	uint8_t sf = (uint8_t)data->modem_cfg.datarate;
	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(data->modem_cfg.bandwidth) * 1000.0f);

	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 30000;  /* safe default if config is uninitialised */
	}

	/* Semtech payload-symbol count with PL=255, CRC on, explicit header,
	 * CR = 4/8 (coded_bits = 8), DE = 1:
	 *   n = 8 + ceil((8*PL - 4*SF + 28 + 16) / (4*(SF - 2*DE))) * 8
	 * The divisor is 4*(SF - 2*DE), not 4*(SF - DE) — same form
	 * lr20xx_lora_airtime() uses.  With DE hard-coded to 1 that is 4*(SF-2),
	 * which at SF5 is 12, so the divisor is always non-zero.
	 *
	 * DE = 1 unconditionally because this is a ceiling, and LDRO on yields
	 * the larger symbol count: at SF12/BW62.5 it is 416 symbols against 384
	 * with the wrong divisor, i.e. 27.3 s of real payload airtime that the
	 * bound has to clear. */
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

int lr20xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			 uint32_t spacing_us)
{
	struct lr20xx_data *data = dev->data;
	int got = 0;

	/* One stand-down for the WHOLE burst, not one per sample.
	 *
	 * lr20xx_get_rssi_inst() has to suspend the duty cycle, enter continuous
	 * Rx, wait 1 ms for the front end, read, and then hand the cycle back --
	 * and lr20xx_dc_resume() clears every IRQ and zeroes the RX-busy latch,
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
	if (lr20xx_dc_rx_in_flight(data)) {
		k_mutex_unlock(&data->spi_mutex);
		return -EBUSY;
	}

	bool armed = lr20xx_dc_suspend(data);

	if (armed) {
		lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			&data->hal_ctx, 0xFFFFFF);
		/* Start the window clean so the check after the loop reports
		 * only what arrived INSIDE it -- these bits are latched (DS 5.7)
		 * and nothing clears them between duty-cycle re-arms, so a stale
		 * preamble from earlier in the cycle would otherwise condemn
		 * every burst.  Safe here and nowhere else: the cycle is already
		 * torn down and dc_resume() clears the lot again on the way out,
		 * so this destroys no state the poll path could still want.
		 * CMD_ERROR is deliberately absent -- that is an LR11xx quirk
		 * with no LR20xx equivalent, as lr20xx_is_receiving() notes. */
		lr20xx_system_clear_irq_status(&data->hal_ctx,
					       LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
					       LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID);
		lr20xx_rssi_settle(data);
	}

	for (int i = 0; i < n; i++) {
		int16_t rssi = 0;
		uint8_t half_dbm = 0;

		if (i) {
			k_busy_wait(spacing_us);
		}
		if (lr20xx_radio_common_get_rssi_inst(&data->hal_ctx, &rssi,
						  &half_dbm) != LR20XX_STATUS_OK) {
			break;
		}
		out[i] = rssi;
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
	 * the reception unaided.  get_status() clears nothing. */
	if (armed) {
		lr20xx_system_irq_mask_t irq = 0;

		if (lr20xx_system_get_status(&data->hal_ctx, NULL, NULL, &irq) ==
		    LR20XX_STATUS_OK &&
		    (irq & (LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
			    LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID))) {
			got = -EAGAIN;
		}
	}

	lr20xx_dc_resume(data, armed);
	k_mutex_unlock(&data->spi_mutex);

	return got;
}

bool lr20xx_is_receiving(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	if (!data->in_rx_mode || data->tx_active) {
		return false;
	}

	/* Sleep-window guard, and the one that matters most: this path is the
	 * TX gate and runs on every send attempt, far more often than the
	 * 15 s probe.  Asking a sleeping chip "are you receiving?" would end
	 * the cycle being asked about (DS §6.3.8 — an NSS falling edge
	 * terminates the loop, and every SPI transaction asserts NSS).  A
	 * sleeping chip is also by definition not mid-packet, so not-receiving
	 * is the truthful answer, not just the safe one.  CAD still gates TX,
	 * so the channel check is not lost.  Same guard the LR11xx applies.
	 *
	 * Payload phase, from the latch the DIO1 handler stamps on
	 * SYNC_WORD_HEADER_VALID.  Read before the duty-cycle split because it
	 * is the one signal both sides share — and the only one the armed side
	 * has at all.  That path cannot bracket itself in suspend/resume the way
	 * the samplers do (standing the cycle down to ask "am I receiving?"
	 * would end the very reception being asked about), nor fall back to a
	 * BUSY read, because BUSY is high through ordinary Rx on this family too
	 * — a BUSY-high "not receiving" is wrong exactly when it matters,
	 * mid-packet, with the TX gate asking permission to transmit over it.
	 *
	 * That leaves the latch, which is why the header IRQ is routed to DIO1
	 * (see the set_dio_irq_cfg call): the handler needs no bus access to keep
	 * this current, and until it did the armed side had nothing to read. */
	uint32_t hdr_seen = data->header_seen_at_ms;

	if (hdr_seen != 0 &&
	    (k_uptime_get_32() - hdr_seen) < lr20xx_max_payload_ms(data)) {
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

	/* Use non-destructive get_status to check for preamble/header
	 * without racing the DIO1 work handler.  lr20xx_system_get_status()
	 * goes through lr20xx_hal_direct_read and clears nothing — the poll
	 * path MUST stay that way.  The only writes below are the two
	 * deliberate releases. */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return false;
	}

	lr20xx_system_irq_mask_t irq = 0;
	lr20xx_system_get_status(&data->hal_ctx, NULL, NULL, &irq);

	/* Header landed: payload phase in progress.  The bit stays latched
	 * until the terminal DIO1 event bulk-clears it, so this covers the
	 * whole packet — bounded by a payload deadline, because in continuous
	 * RX that terminal event is not guaranteed to arrive and a header that
	 * never completes would otherwise mute TX until reboot. */
	if (irq & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {
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
		lr20xx_system_clear_irq_status(&data->hal_ctx,
					       LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
					       LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID |
					       LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR);
		lr20xx_reset_rx_busy_signals(data);
		k_mutex_unlock(&data->spi_mutex);
		return false;
	}

	/* PREAMBLE_DETECTED with SF-aware grace.  Within grace, keep reporting
	 * busy so a real packet has time to land its header; after grace with
	 * no header, assume noise or a foreign sync word, clear the bit and
	 * release TX.  (Unlike the LR11xx, CMD_ERROR is deliberately not in the
	 * clear mask: that chip sets it on any ClearIrq whose mask excludes it,
	 * a quirk with no LR20xx equivalent — this driver already issues
	 * partial-mask clears elsewhere without provoking one, and folding it in
	 * here would hide genuine command rejections.) */
	if (irq & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {
		uint32_t now = k_uptime_get_32();
		uint32_t seen = data->preamble_seen_at_ms;

		if (seen == 0) {
			data->preamble_seen_at_ms = (now == 0) ? 1U : now;
			k_mutex_unlock(&data->spi_mutex);
			return true;
		}
		if ((now - seen) < lr20xx_preamble_grace_ms(data)) {
			k_mutex_unlock(&data->spi_mutex);
			return true;
		}
		lr20xx_system_clear_irq_status(&data->hal_ctx,
					       LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED);
		lr20xx_reset_rx_busy_signals(data);
		k_mutex_unlock(&data->spi_mutex);
		return false;
	}

	/* No preamble, no header: nothing in flight. */
	lr20xx_reset_rx_busy_signals(data);
	k_mutex_unlock(&data->spi_mutex);
	return false;
}

uint32_t lr20xx_get_freq_offset(const struct device *dev,
				struct lr20xx_freq_offset_stats *out)
{
	struct lr20xx_data *data = dev->data;
	uint32_t count;

	if (out == NULL) {
		return 0;
	}

	/* Blocking lock: this is a diagnostic reader on the CLI/telemetry path,
	 * not the TX gate, so waiting is cheap and a torn read of a 64-bit sum
	 * against the DIO1 work handler is not. */
	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	count = data->freq_off_count;
	out->count = count;
	out->last_hz = data->freq_off_last_hz;
	out->min_hz = count ? data->freq_off_min_hz : 0;
	out->max_hz = count ? data->freq_off_max_hz : 0;
	out->mean_hz = count
			       ? (int32_t)(data->freq_off_sum_hz / (int64_t)count)
			       : 0;

	k_mutex_unlock(&data->spi_mutex);

	return count;
}

void lr20xx_reset_freq_offset(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	data->freq_off_last_hz = 0;
	data->freq_off_min_hz = 0;
	data->freq_off_max_hz = 0;
	data->freq_off_sum_hz = 0;
	data->freq_off_count = 0;
	k_mutex_unlock(&data->spi_mutex);
}

uint32_t lr20xx_get_dc_timeout_restarts(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	return (uint32_t)atomic_get(&data->dc_timeout_restarts);
}

void lr20xx_reset_dc_timeout_restarts(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	atomic_set(&data->dc_timeout_restarts, 0);
}

uint32_t lr20xx_get_wakeup_time_us(const struct device *dev)
{
	const struct lr20xx_config *cfg = dev->config;

	/* DS Table 3-23: warm start (retention) 1 ms + STDBY_RC->Rx 115 us,
	 * rounded to 1200 for margin, plus the TCXO restart — duty-cycle sleep
	 * powers the VTCXO regulator down.  See ARCHITECTURE.md 5.5.1. */
	uint32_t us = 1200;

	if (cfg->tcxo_voltage_mv > 0) {
		us += cfg->tcxo_startup_delay_ms * 1000U;
	}
	return us;
}

int lr20xx_configure_side_detectors(const struct device *dev,
				    const uint8_t *sfs, uint8_t num)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	uint8_t main_sf = (uint8_t)data->modem_cfg.datarate;
	uint8_t lowest = main_sf, highest = main_sf;
	uint8_t max_allowed = LR20XX_MAX_SIDE_DETECTORS;

	if (num > LR20XX_MAX_SIDE_DETECTORS) {
		return -EINVAL;
	}

	/* DS 9.9.6, checked here so the chip's limits live next to the chip:
	 * main SF below every side SF, all distinct, highest-lowest <= 4,
	 * BW > 500 kHz caps the count at 2, main SF >= 10 caps it at 1.  The last
	 * two are independent rules — the vendor header makes the SF one
	 * conditional on the BW one, which accepts sets the chip rejects. */
	if (data->modem_cfg.bandwidth >= BW_500_KHZ && max_allowed > 2) {
		max_allowed = 2;
	}
	if (main_sf >= 10 && max_allowed > 1) {
		max_allowed = 1;
	}
	if (num > max_allowed) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < num; i++) {
		if (sfs[i] < 5 || sfs[i] > 12 || sfs[i] <= main_sf) {
			return -EINVAL;
		}
		for (uint8_t j = 0; j < i; j++) {
			if (sfs[j] == sfs[i]) {
				return -EINVAL;
			}
		}
		if (sfs[i] < lowest)  { lowest = sfs[i]; }
		if (sfs[i] > highest) { highest = sfs[i]; }
	}
	if (highest - lowest > 4) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->side_det_num = num;
	for (uint8_t i = 0; i < num; i++) {
		data->side_det_sf[i] = sfs[i];
	}

	/* Take effect now.  A full RX restart is the cheapest correct way to
	 * get there: the command needs the chip out of Rx, and start_rx runs
	 * apply_modem_config, which re-applies the set on our behalf. */
	if (data->in_rx_mode && !data->tx_active) {
		lr20xx_start_rx(data, cfg);
	} else {
		/* Not receiving — nothing to reprogram yet, the next RX entry
		 * picks the stored set up. */
		data->side_det_applied = false;
	}

	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("side detectors configured: %u", num);
	return 0;
}

void lr20xx_set_rx_boost(const struct device *dev, bool enable)
{
	struct lr20xx_data *data = dev->data;

	if (data->rx_boost_enabled == enable) {
		return;
	}

	data->rx_boost_enabled = enable;
	LOG_DBG("RX boost %s", enable ? "enabled" : "disabled");

	if (data->in_rx_mode && data->configured) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		/* Keep the band: SetRxPath carries both, so re-sending it with
		 * a hardcoded LF would quietly drop a 2.4 GHz node onto the
		 * sub-GHz path the moment `set rx.boost` was toggled. */
		lr20xx_radio_common_set_rx_path(
			&data->hal_ctx,
			lr20xx_rx_path_for(data->modem_cfg.frequency),
			enable ? LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_7
			       : LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
		data->rx_boost_applied = enable;
		k_mutex_unlock(&data->spi_mutex);
	} else {
		data->rx_boost_applied = false;
	}
}

uint32_t lr20xx_get_random(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	uint32_t random = 0;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr20xx_system_get_random_number(
		&data->hal_ctx,
		LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_PLL |
		LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_ADC,
		&random);
	k_mutex_unlock(&data->spi_mutex);

	return random;
}

/* ── Driver API: CAD ────────────────────────────────────────────────── */

/* Datasheet Table 6-19 in full: rows = CAD symbol count 1..4, columns =
 * SF5..SF12.  LoRaRadioBase asks for LORA_CAD_SYMB_4, so the bottom row is
 * what we normally use — but cad_symb_nb is overridable from
 * mc->cad.symbol_num, and a 1-D table indexed only by SF would silently keep
 * handing out 4-symbol peaks for a 2-symbol window.
 *
 * A higher det_peak is a *less* sensitive CAD.  The table this replaced was
 * RadioLib's 2-symbol set, which is systematically higher than the 4-symbol
 * row, so pairing it with a 4-symbol window made LBT more willing to talk over
 * faint traffic.  Adaptive CAD offsets from this base, so getting the base
 * right shifts the whole operating range. */
static const uint8_t lr20xx_cad_peak_table[4][8] = {
	/*        SF5 SF6 SF7 SF8 SF9 SF10 SF11 SF12 */
	/* 1 */ { 60, 60, 60, 64, 64, 66,  70,  74 },
	/* 2 */ { 56, 56, 56, 58, 58, 60,  64,  68 },
	/* 3 */ { 51, 51, 52, 54, 56, 60,  60,  65 },
	/* 4 */ { 51, 51, 51, 54, 56, 60,  60,  64 },
};

/* The detPeak range this driver will actually program.  Exported through
 * lr20xx_cad_peak_min/max() so the C++ adaptive-CAD controller can narrow its
 * offset window to match: where base+offset falls outside this, several offsets
 * collapse onto one peak and the staircase reads sampling noise between
 * identical configurations as curvature.
 *
 * 48 sits three steps below the lowest value DS Table 6-19 documents anywhere
 * (51, at SF5-7 with 3-4 symbols), so the range already allows more sensitivity
 * than the datasheet tabulates; the datasheet gives no absolute bound. */
#define LR20XX_CAD_PEAK_MIN 48
#define LR20XX_CAD_PEAK_MAX 90

uint8_t lr20xx_cad_peak_min(void)
{
	return LR20XX_CAD_PEAK_MIN;
}

uint8_t lr20xx_cad_peak_max(void)
{
	return LR20XX_CAD_PEAK_MAX;
}

static uint8_t lr20xx_cad_detect_peak(uint8_t sf, uint8_t symb_nb)
{
	if (sf < 5 || sf > 12) {
		sf = 9;   /* mid-range fallback, matches the old default of 56 */
	}
	/* Counts above 4 are not tabulated; the 4-symbol row is the most
	 * sensitive, and a longer window is never less sensitive than it. */
	if (symb_nb < 1) {
		symb_nb = 1;
	} else if (symb_nb > 4) {
		symb_nb = 4;
	}
	return lr20xx_cad_peak_table[symb_nb - 1][sf - 5];
}

static int lr20xx_do_cad(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	uint8_t symb_nb = lr20xx_cad_symb_nb(mc);
	lr20xx_radio_lora_cad_params_t cad = {
		.cad_symb_nb = symb_nb,
		/* Best-effort ("fast") CAD.  DS §6.3.11: "CadDone time is
		 * shortened in case of early no detection", and the vendor
		 * header calls 8 the recommended best-effort value.  Most LBT
		 * CADs find nothing, so this shortens the common pre-TX
		 * blocking path.  0 would mean the exact symbol count.
		 *
		 * Side detectors are already off during CAD, so the datasheet's
		 * multi-SF caveat ("the main SF determines the not-detect
		 * condition") does not apply.
		 *
		 * If CAD behaviour is ever suspect, `get cad`'s per-level line
		 * is the instrument: a materially LOWER busy rate is a red flag
		 * rather than a win, since it more likely means CAD stopped
		 * detecting than that the channel got quieter. */
		.pnr_delta = 8,
		/* CAD_LBT hands the CAD->TX transition to the
		 * chip.  Note the two CAD commands have different exit-mode
		 * encodings — generic RSSI CAD (DS Table 6-22) is 0x01 for Tx,
		 * LoRa CAD (Table 6-18) is 0x10.  Always the SDK enum, never
		 * RadioLib's constants: RadioLib calls the *other* command. */
		.cad_exit_mode = data->cad_lbt_exit_tx
					 ? LR20XX_RADIO_LORA_CAD_EXIT_MODE_TX
					 : LR20XX_RADIO_LORA_CAD_EXIT_MODE_STANDBYRC,
		/* DS §6.3.11 (ds.txt:9435): "cad_timeout defines the timeout
		 * duration after CAD for Rx (CAD_RX) or Tx (CAD_LBT), expressed
		 * in periods of 32MHz crystal oscillator."  NOT the "PLL step of
		 * 31.25us" the vendor header's doc comment claims, and not RTC
		 * steps — getting this wrong truncates transmissions. */
		.cad_timeout_in_pll_step = data->cad_lbt_exit_tx
						   ? data->cad_lbt_tx_timeout
						   : 0,
		/* Peak is looked up for the window actually programmed, so the
		 * two can never drift apart (DS Table 6-19 is 2-D). */
		.cad_detect_peak = lr20xx_cad_detect_peak(sf, symb_nb),
	};

	if (mc->cad.detection_peak != 0) {
		cad.cad_detect_peak = mc->cad.detection_peak;
	} else if (data->cad_peak_offset != 0) {
		/* Adaptive-CAD operating offset (base +/- learned delta).
		 * LR20xx detPeak scale matches LR11xx (~48-90). */
		int peak = (int)cad.cad_detect_peak + data->cad_peak_offset;

		if (peak < LR20XX_CAD_PEAK_MIN) {
			peak = LR20XX_CAD_PEAK_MIN;
		} else if (peak > LR20XX_CAD_PEAK_MAX) {
			peak = LR20XX_CAD_PEAK_MAX;
		}
		cad.cad_detect_peak = (uint8_t)peak;
	}
	if (data->cad_probe_peak != 0) {
		/* One-shot calibration probe: absolute peak wins over all. */
		cad.cad_detect_peak = data->cad_probe_peak;
	}

	/* Side detectors must be off for CAD: normal Rx needs the main SF
	 * below every side SF, CAD needs it above — the two cannot hold at
	 * once.  restart_rx puts them back. */
	lr20xx_apply_side_detectors(data, false);

	lr20xx_radio_lora_configure_cad_params(ctx, &cad);
	CHECK_CMD(ctx, "configure_cad_params");

	/* Clear any pending IRQ flags, then start CAD */
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	lr20xx_reset_rx_busy_signals(data);
	lr20xx_radio_lora_set_cad(ctx);
	CHECK_CMD(ctx, "set_cad");

	/* One-shot: consumed here so no later CAD (an adaptive-CAD probe, or a
	 * plain LBT with the payload no longer staged) can inherit exit-to-TX
	 * and transmit whatever is left in the FIFO. */
	data->cad_lbt_exit_tx = false;
	data->cad_lbt_tx_timeout = 0;

	return 0;
}

/* Blocking CAD.  Leaves the chip in STANDBY on every exit — success, busy,
 * timeout and error alike — and deliberately does NOT restore RX: the caller
 * knows whether it is about to transmit (in which case re-entering RX would be
 * undone immediately) or must go back on air.  Both current callers honour
 * that: lr20xx_lora_send_async()'s LBT branch re-arms on the busy path, and
 * LoRaRadioBase::cadMaintenance() calls startReceive() after every probe.  A
 * third caller that forgets leaves the node deaf with nothing scheduled to
 * recover it, so the contract is stated here rather than left to be inferred. */
static int lr20xx_lora_cad(const struct device *dev, k_timeout_t timeout)
{
	struct lr20xx_data *data = dev->data;
	int ret;

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Stop async RX if active — CAD needs the radio */
	bool was_in_rx = data->in_rx_mode;

	if (was_in_rx) {
		data->in_rx_mode = false;
		lr20xx_system_set_standby_mode(&data->hal_ctx,
					       LR20XX_SYSTEM_STANDBY_MODE_RC);
	}

	k_sem_reset(&data->cad_sem);
	data->cad_result = -ETIMEDOUT;
	data->cad_cb = NULL;

	ret = lr20xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	if (ret < 0) {
		return ret;
	}

	/* Wait for DIO1 handler to signal CAD_DONE */
	ret = k_sem_take(&data->cad_sem, timeout);
	if (ret == -EAGAIN) {
		return -ETIMEDOUT;
	}

	return data->cad_result;
}

static int lr20xx_lora_cad_async(const struct device *dev,
				  lora_cad_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;

	if (cb == NULL) {
		/* Cancel pending CAD.  Under spi_mutex like every other write to
		 * the CAD state: the DIO1 handler reads cad_cb to decide whether
		 * to dispatch a callback, and this used to race it.  Dead today
		 * (nothing in the tree calls lora_cad_async), which is exactly
		 * why it is worth fixing now rather than when someone wires it
		 * up and inherits an invisible race. */
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->cad_cb = NULL;
		data->cad_user_data = NULL;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	bool was_in_rx = data->in_rx_mode;

	if (was_in_rx) {
		data->in_rx_mode = false;
		lr20xx_system_set_standby_mode(&data->hal_ctx,
					       LR20XX_SYSTEM_STANDBY_MODE_RC);
	}

	data->cad_cb = cb;
	data->cad_user_data = user_data;

	int ret = lr20xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	return ret;
}

/* ── Extension API: adaptive CAD ────────────────────────────────────── */

void lr20xx_cad_set_peak_offset(const struct device *dev, int8_t offset)
{
	struct lr20xx_data *data = dev->data;

	data->cad_peak_offset = offset;
}

uint8_t lr20xx_cad_base_peak(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	return lr20xx_cad_detect_peak((uint8_t)data->modem_cfg.datarate,
				      lr20xx_cad_symb_nb(&data->modem_cfg));
}

int lr20xx_cad_probe(const struct device *dev, int8_t peak_offset)
{
	struct lr20xx_data *data = dev->data;
	int base = (int)lr20xx_cad_base_peak(dev);
	int peak = base + peak_offset;
	int ret;

	if (peak < LR20XX_CAD_PEAK_MIN) {
		peak = LR20XX_CAD_PEAK_MIN;
	} else if (peak > LR20XX_CAD_PEAK_MAX) {
		peak = LR20XX_CAD_PEAK_MAX;
	}

	/* One-shot absolute override consumed by lr20xx_do_cad().  Probes and
	 * LBT both run on the mesh loop thread, so no concurrent CAD exists. */
	data->cad_probe_peak = (uint8_t)peak;
	data->cad_probe_rx = true;
	atomic_set(&data->cad_rx_state, LR20XX_CAD_RX_IDLE);
	ret = lr20xx_lora_cad(dev, K_MSEC(lr20xx_cad_timeout_ms(data)));
	data->cad_probe_rx = false;
	data->cad_probe_peak = 0;

	/* 2 tells the caller the chip is in Rx on the signal CAD found, so it
	 * must NOT re-enter Rx itself, and that an outcome will be readable
	 * from lr20xx_cad_rx_outcome() once a terminal IRQ lands. */
	if (ret > 0) {
		return 2;
	}

	return ret;
}

uint32_t lr20xx_cad_rx_timeout_ms(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	/* Same bound do_cad's follow-on SetRx programs, so the caller's wait
	 * and the chip's deadline cannot drift apart. */
	return lr20xx_max_payload_ms(data);
}

int lr20xx_cad_rx_outcome(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	atomic_val_t st = atomic_get(&data->cad_rx_state);

	if (st != LR20XX_CAD_RX_PACKET && st != LR20XX_CAD_RX_TMOUT) {
		return 0;  /* nothing armed, or still awaiting the terminal IRQ */
	}

	atomic_set(&data->cad_rx_state, LR20XX_CAD_RX_IDLE);
	return (st == LR20XX_CAD_RX_PACKET) ? 1 : 2;
}

/* ── Driver API: recv_duty_cycle ────────────────────────────────────── */

static int lr20xx_lora_recv_duty_cycle(const struct device *dev,
				       k_timeout_t rx_period,
				       k_timeout_t sleep_period,
				       lora_recv_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;

	if (cb == NULL) {
		/* Cancel — same as recv_async(NULL) */
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->async_rx_cb = NULL;
		data->async_rx_user_data = NULL;
		data->in_rx_mode = false;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	/* Explicit timing only — the adapter owns the window sizing. */
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

	/* Idempotent fast path, the duty-cycle counterpart of the one in
	 * recv_async and the same shape as the SX126x's: a cycle already armed
	 * on exactly these periods has nothing to re-arm, so refresh the
	 * callback and stay on air instead of tearing a live cycle down and
	 * paying a full rebuild for it.  Timings are computed before the lock so
	 * they can be compared here.
	 *
	 * Any other combination — continuous RX, or a cycle on different periods
	 * — falls through to the full path below, which re-arms via
	 * lr20xx_apply_rx_duty_cycle() and therefore still gets the AGC reset. */
	if (data->in_rx_mode && data->rx_duty_cycle_enabled &&
	    data->dc_rx_ms == rx_ms && data->dc_sleep_ms == slp_ms) {
		data->async_rx_cb = cb;
		data->async_rx_user_data = user_data;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;

	void *ctx = &data->hal_ctx;

	lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	lr20xx_radio_fifo_clear_rx(ctx);
	lr20xx_apply_modem_config(data, cfg, false);

	/* Store for the re-arm paths (lr20xx_start_rx / lr20xx_restart_rx)
	 * so every re-entry uses exactly this timing. */
	data->dc_rx_ms = rx_ms;
	data->dc_sleep_ms = slp_ms;
	data->rx_duty_cycle_enabled = true;
	/* Arm through the shared helper so the cycle_time conversion lives in
	 * exactly one place — this site and the re-arm path had to agree, and
	 * two copies of that conversion is how they would stop agreeing. */
	lr20xx_apply_rx_duty_cycle(data);
	LOG_INF("recv_duty_cycle: rx=%ums sleep=%ums (cycle=%ums)",
		rx_ms, slp_ms, rx_ms + slp_ms);

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	data->in_rx_mode = true;
	data->tx_active = false;
	lr20xx_reset_rx_busy_signals(data);

	k_mutex_unlock(&data->spi_mutex);
	return 0;
}

/* ── Deferred hardware init ─────────────────────────────────────────── */

static int lr20xx_hw_init(struct lr20xx_data *data,
			  const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_INF("LR20xx hardware init starting");

	data->irq_dio = cfg->irq_dio;

	lr20xx_system_version_t ver;
	bool found = false;

	for (int attempt = 0; attempt < 3; attempt++) {
		lr20xx_hal_status_t hal_rc = lr20xx_hal_reset(ctx);
		if (hal_rc != LR20XX_HAL_STATUS_OK) {
			LOG_WRN("LR20xx reset failed (attempt %d)", attempt);
			k_msleep(10);
			continue;
		}

		lr20xx_status_t st = lr20xx_system_get_version(ctx, &ver);
		if (st == LR20XX_STATUS_OK) {
			found = true;
			break;
		}

		LOG_WRN("LR20xx get_version failed (attempt %d)", attempt);
		k_msleep(10);
	}

	if (!found) {
		LOG_ERR("LR20xx not found after 3 attempts");
		return -EIO;
	}

	LOG_INF("LR20xx SDK get_version: major=%u minor=%u", ver.major, ver.minor);

	/* The frame A / frame B bring-up dumps lived here: two hexdumps of
	 * GET_VERSION, one per candidate SPI framing, to settle which protocol
	 * this chip actually speaks.  Answered — the two-window read in
	 * lr20xx_spi_read_frame() is correct, and TX/RX both work on it.
	 * Removed rather than left firing on every boot.
	 *
	 * The only base FW version the datasheet documents is 1.24 (0x01/0x18).
	 * Anything else still runs — the mismatch is worth a line in the log
	 * when a bring-up goes sideways, not a hard failure. */
	if (ver.major != 0x01 || ver.minor != 0x18) {
		LOG_WRN("Unexpected LR2021 FW %u.%u (datasheet documents 1.24)",
			ver.major, ver.minor);
	}

	/* Patch the firmware before anything is configured or calibrated — DS
	 * §22.3.1: load "after a reset, as part of the reset sequence".  The
	 * TCXO→XTAL fallback below re-enters this function, which resets the chip
	 * again and so reaches this point again; that is required, since the
	 * reset drops the patch. */
	lr20xx_load_pram(data);

	DUMP_CHIP_STATE(data, "post-reset");

	/* SIMO DC-DC workaround REMOVED — datasheet §22.6 says it's only
	 * needed when SetRegMode simo_usage=0x02 (SIMO_NORMAL).
	 * We run in LDO mode (default, simo_usage=0x00). */

	/* A board that declares a TCXO but is actually fitted with a plain
	 * crystal fails to start its 32 MHz reference once TCXO mode is on, and
	 * every later command is rejected. The datasheet (S1.9.3) puts the TCXO
	 * on XTA with the VTCXO regulator, so the two builds are indistinguishable
	 * from software — the chip's own HF_XOSC_START error is the only signal.
	 * Fall back to XTAL rather than come up dead, as MeshCore's CustomLR2021
	 * does when begin() reports a rejected command. */
	if (cfg->tcxo_voltage_mv > 0 && !data->tcxo_disabled) {
		uint32_t tcxo_ticks = tcxo_start_time_periods(cfg->tcxo_startup_delay_ms);
		lr20xx_status_t tcxo_rc = lr20xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    tcxo_ticks);
		LOG_DBG("init: set_tcxo(%dmV, %u ticks)=%d",
			cfg->tcxo_voltage_mv, tcxo_ticks, tcxo_rc);
	} else {
		LOG_DBG("init: TCXO disabled (XTAL mode)");
	}

	/* RadioLib does NOT call cfg_lfclk or set_reg_mode.
	 * Stay in LDO mode (chip default after reset).
	 * DCDC mode + wrong SET_REG_MODE encoding was likely
	 * preventing TX. */
	lr20xx_status_t st;

	lr20xx_configure_rfswitch(ctx, cfg);
	LOG_DBG("RF switch: en=0x%02x stby=0x%02x rxlf=0x%02x rxhf=0x%02x "
		"tx=0x%02x txhp=0x%02x",
		cfg->rfswitch_enable, cfg->rfswitch_standby,
		cfg->rfswitch_rx_lf, cfg->rfswitch_rx_hf,
		cfg->rfswitch_tx, cfg->rfswitch_tx_hp);

	st = lr20xx_system_set_dio_function(ctx, lr20xx_irq_dio(cfg),
				       LR20XX_SYSTEM_DIO_FUNC_IRQ,
				       lr20xx_irq_dio_pull(cfg));

	st = lr20xx_radio_common_set_rx_tx_fallback_mode(ctx,
						    LR20XX_RADIO_FALLBACK_STDBY_RC);

	DUMP_CHIP_STATE(data, "pre-cal");

	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	/* Calibrate all analog blocks: 0x6F = LF_RC|HF_RC|PLL|AAF|MU|PA_OFF
	 *
	 * No BUSY poll after this.  lr20xx_system_calibrate() returns through
	 * lr20xx_hal_write(), which already ends in the interrupt-driven
	 * wait_on_busy() (3 s bound, logs on timeout) — a hand-rolled 1 ms
	 * polling loop only re-implements it at worse granularity and would
	 * essentially always find BUSY already low. */
	st = lr20xx_system_calibrate(ctx, 0x6F);

	DUMP_CHIP_STATE(data, "post-cal");

	/* The 32 MHz reference is what calibration needs, so this is where a
	 * wrong clock source shows up. One retry only — if XTAL fails too, the
	 * fault is not the clock config and looping would just hide it. */
	{
		lr20xx_system_errors_t clk_err = 0;

		lr20xx_system_get_errors(ctx, &clk_err);
		if ((clk_err & LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK) &&
		    cfg->tcxo_voltage_mv > 0 && !data->tcxo_disabled) {
			LOG_WRN("HF XOSC did not start with TCXO at %d mV "
				"(errors=0x%04x) — retrying in XTAL mode",
				cfg->tcxo_voltage_mv, clk_err);
			data->tcxo_disabled = true;
			return lr20xx_hw_init(data, cfg);
		}
	}

	/* No FE calibration here: lora_config(), this function's only caller,
	 * calibrates at the operating frequency the moment it returns. */

	/* Verify: set packet type to LoRa and read it back */
	st = lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);

	/* dcdc_reset removed — not needed in LDO mode, RadioLib doesn't do it */

	lr20xx_radio_common_pkt_type_t pkt_readback = 0xFF;
	lr20xx_radio_common_get_pkt_type(ctx, &pkt_readback);

	DUMP_CHIP_STATE(data, "init-done");

	/* The status-poll probe lived here, asking whether merely polling
	 * GetStatus raised CmdError — i.e. whether our own polling was
	 * manufacturing the CMD_ERROR storm.  It reported "is NOT" on every
	 * run, and the real source is now known: the NSS wake pulse that
	 * check_device_ready() used to fire at a merely-busy chip, which the
	 * chip read as a clockless malformed command.  Question closed.
	 *
	 * DEBUG: what voltage does the chip see on its OWN supply pin? (mV,
	 * after MU calibration).  If this reads low (≪3000mV) while the board
	 * is powered, the LR2021 VBAT/supply pin is floating or miswired — that
	 * is why it flags LOW_BATTERY and aborts TX no matter the system rail
	 * or external battery. ~3000-3300mV ⇒ supply is fine, sag is transient. */
	{
		uint16_t vbat_mv = 0;
		lr20xx_status_t vrc = lr20xx_system_get_vbat(ctx,
			LR20XX_SYSTEM_VALUE_FORMAT_UNIT,
			LR20XX_SYSTEM_MEAS_RES_12_BITS, &vbat_mv);
		LOG_INF("init: chip VBAT reads %u mV (rc=%d)", vbat_mv, vrc);
	}

	lr20xx_system_clear_errors(ctx);
	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	data->rx_boost_enabled = cfg->rx_boosted;
	data->rx_boost_applied = false;

	data->hw_initialized = true;
	LOG_INF("LR20xx driver ready");
	return 0;
}

/* ── Driver init (lightweight — runs at POST_KERNEL) ────────────────── */

static int lr20xx_lora_init(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	int ret;

	data->dev = dev;
	data->hw_initialized = false;

	k_mutex_init(&data->spi_mutex);
	k_sem_init(&data->cad_sem, 0, 1);
	k_work_init(&data->dio1_work, lr20xx_dio1_work_handler);

	k_work_queue_start(&data->dio1_wq, lr20xx_dio1_wq_stack,
			   K_THREAD_STACK_SIZEOF(lr20xx_dio1_wq_stack),
			   K_PRIO_COOP(7),
			   &(const struct k_work_queue_config){ .name = "lr20xx_dio1" });

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	memset(&data->hal_ctx, 0, sizeof(data->hal_ctx));
	data->hal_ctx.spi_dev = cfg->bus.bus;
	data->hal_ctx.spi_cfg = cfg->bus.config;
	/* Manual NSS control — disable SPI peripheral CS */
	data->hal_ctx.spi_cfg.cs.cs_is_gpio = false;
	data->hal_ctx.spi_cfg.cs.gpio.port = NULL;
	data->hal_ctx.nss.port  = cfg->bus.config.cs.gpio.port;
	data->hal_ctx.nss.pin   = cfg->bus.config.cs.gpio.pin;
	data->hal_ctx.nss.dt_flags = cfg->bus.config.cs.gpio.dt_flags;
	data->hal_ctx.reset = cfg->reset;
	data->hal_ctx.busy  = cfg->busy;
	data->hal_ctx.dio1  = cfg->dio1;
	data->hal_ctx.radio_is_sleeping = false;

	ret = lr20xx_hal_init(&data->hal_ctx);
	if (ret != 0) {
		LOG_ERR("HAL init failed: %d", ret);
		return ret;
	}

	lr20xx_hal_set_dio1_callback(&data->hal_ctx, lr20xx_dio1_callback,
				     data);

	LOG_INF("LR20xx driver registered (hw init deferred to first config)");
	return 0;
}

/* ── Device instantiation ───────────────────────────────────────────── */

static DEVICE_API(lora, lr20xx_lora_api) = {
	.config          = lr20xx_lora_config,
	.airtime         = lr20xx_lora_airtime,
	.send            = lr20xx_lora_send,
	.send_async      = lr20xx_lora_send_async,
	.recv            = lr20xx_lora_recv,
	.recv_async      = lr20xx_lora_recv_async,
	.cad             = lr20xx_lora_cad,
	.cad_async       = lr20xx_lora_cad_async,
	.recv_duty_cycle_async = lr20xx_lora_recv_duty_cycle,
};

#define LR20XX_INIT(n)                                                       \
	static const struct lr20xx_config lr20xx_config_##n = {              \
		.bus = SPI_DT_SPEC_INST_GET(n,                               \
			SPI_WORD_SET(8) | SPI_OP_MODE_CONTROLLER |           \
			SPI_TRANSFER_MSB),                                   \
		.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),              \
		.busy  = GPIO_DT_SPEC_INST_GET(n, busy_gpios),              \
		.dio1  = GPIO_DT_SPEC_INST_GET(n, dio1_gpios),              \
		.tcxo_voltage_mv =                                           \
			DT_INST_PROP_OR(n, tcxo_voltage_mv, 0),             \
		.tcxo_startup_delay_ms =                                     \
			DT_INST_PROP_OR(n, tcxo_startup_delay_ms, 5),       \
		.rx_boosted       = DT_INST_PROP(n, rx_boosted),            \
		.irq_dio          = DT_INST_PROP_OR(n, irq_dio, 9),         \
		.rfswitch_enable  = DT_INST_PROP_OR(n, rfswitch_enable, 0), \
		.rfswitch_standby = DT_INST_PROP_OR(n, rfswitch_standby, 0),\
		/* rfswitch-rx-lf / -hf are optional and carry no binding      \
		 * default, so an absent one falls back to the combined       \
		 * rfswitch-rx — which keeps every pre-split board (e.g.      \
		 * promicro_lr2021) byte-identical. */                        \
		.rfswitch_rx_lf   = DT_INST_PROP_OR(n, rfswitch_rx_lf,      \
				    DT_INST_PROP_OR(n, rfswitch_rx, 0)),    \
		.rfswitch_rx_hf   = DT_INST_PROP_OR(n, rfswitch_rx_hf,      \
				    DT_INST_PROP_OR(n, rfswitch_rx, 0)),    \
		.rfswitch_tx      = DT_INST_PROP_OR(n, rfswitch_tx, 0),     \
		.rfswitch_tx_hp   = DT_INST_PROP_OR(n, rfswitch_tx_hp, 0),  \
	};                                                                   \
	static struct lr20xx_data lr20xx_data_##n;                           \
	DEVICE_DT_INST_DEFINE(n, lr20xx_lora_init, NULL,                     \
			      &lr20xx_data_##n, &lr20xx_config_##n,          \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,        \
			      &lr20xx_lora_api);

DT_INST_FOREACH_STATUS_OKAY(LR20XX_INIT)
