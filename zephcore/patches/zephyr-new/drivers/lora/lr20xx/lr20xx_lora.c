/*
 * SPDX-License-Identifier: Apache-2.0
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

#include "lr20xx_lora.h"
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

LOG_MODULE_REGISTER(lr20xx_lora, CONFIG_LORA_LOG_LEVEL);

/* Dedicated DIO1 work queue — keeps LoRa interrupt processing off the
 * system work queue so USB/BLE/timer work items cannot delay packet RX. */
#define LR20XX_DIO1_WQ_STACK_SIZE 2560
K_THREAD_STACK_DEFINE(lr20xx_dio1_wq_stack, LR20XX_DIO1_WQ_STACK_SIZE);

/* ── Driver data structures ─────────────────────────────────────────── */

struct lr20xx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec dio1;
	uint16_t tcxo_voltage_mv;
	uint32_t tcxo_startup_delay_ms;
	bool rx_boosted;
	/* RF switch DIO bitmasks (bit 0 = DIO5, bit 1 = DIO6, ...) */
	uint8_t rfswitch_enable;
	uint8_t rfswitch_standby;
	uint8_t rfswitch_rx;
	uint8_t rfswitch_tx;
	uint8_t rfswitch_tx_hp;
	/* PA config */
	uint8_t pa_hp_sel;    /* maps to pa_lf_slices in LR20xx */
	uint8_t pa_duty_cycle; /* maps to pa_lf_duty_cycle in LR20xx */
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

	/* Deferred hardware init */
	bool hw_initialized;

	/* DIO1 stuck-HIGH detection */
	int dio1_stuck_count;

	/* RX data buffer */
	uint8_t rx_buf[256];
};

/* ── Debug: dump full chip state ────────────────────────────────────── */

static void dump_chip_state(void *ctx, struct lr20xx_hal_context *hal,
			    const char *label)
{
	lr20xx_system_stat1_t s1 = {0};
	lr20xx_system_stat2_t s2 = {0};
	lr20xx_system_irq_mask_t irq = 0;
	lr20xx_system_errors_t err = 0;

	lr20xx_system_get_status(ctx, &s1, &s2, &irq);
	lr20xx_system_get_errors(ctx, &err);

	int busy = gpio_pin_get_dt(&hal->busy);
	int dio9 = gpio_pin_get_dt(&hal->dio1);

	LOG_INF("[%s] cmd=%d mode=%d err=0x%04x irq=0x%08x BUSY=%d DIO9=%d",
		label, s1.command_status, s2.chip_mode, err, irq, busy, dio9);
}

/* ── Helpers ────────────────────────────────────────────────────────── */

static lr20xx_radio_lora_bw_t bw_enum_to_lr20xx(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_31_KHZ:  return LR20XX_RADIO_LORA_BW_31;
	case BW_41_KHZ:  return LR20XX_RADIO_LORA_BW_41;
	case BW_62_KHZ:  return LR20XX_RADIO_LORA_BW_62;
	case BW_125_KHZ: return LR20XX_RADIO_LORA_BW_125;
	case BW_250_KHZ: return LR20XX_RADIO_LORA_BW_250;
	case BW_500_KHZ: return LR20XX_RADIO_LORA_BW_500;
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

static lr20xx_system_tcxo_supply_voltage_t get_tcxo_voltage(uint16_t mv)
{
	if (mv >= 3300) return LR20XX_SYSTEM_TCXO_CTRL_3_3V;
	if (mv >= 3000) return LR20XX_SYSTEM_TCXO_CTRL_3_0V;
	if (mv >= 2700) return LR20XX_SYSTEM_TCXO_CTRL_2_7V;
	if (mv >= 2400) return LR20XX_SYSTEM_TCXO_CTRL_2_4V;
	if (mv >= 2200) return LR20XX_SYSTEM_TCXO_CTRL_2_2V;
	if (mv >= 1800) return LR20XX_SYSTEM_TCXO_CTRL_1_8V;
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
	default:         return 125.0f;
	}
}

/* ── Configure RF switch DIOs ───────────────────────────────────────── */

static void lr20xx_configure_rfswitch(void *ctx, const struct lr20xx_config *cfg)
{
	/* LR20xx RF switch uses per-DIO configuration.
	 * DIO5..DIO8 map to enable bitmask bits 0..3.
	 * For each enabled DIO, compute which operational modes
	 * should drive it HIGH by looking at the per-mode bitmasks. */
	for (int i = 0; i < 4; i++) {
		if (!(cfg->rfswitch_enable & BIT(i))) {
			continue;
		}

		lr20xx_system_dio_t dio = (lr20xx_system_dio_t)(LR20XX_SYSTEM_DIO_5 + i);

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
		if (cfg->rfswitch_rx & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF |
				  LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF;
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

/* ── PA power lookup table (from Semtech lr20xx_pa_pwr_cfg.h) ──────── */

struct lr20xx_pa_pwr_entry {
	int8_t  half_power;
	uint8_t pa_duty_cycle;
	uint8_t pa_lf_slices;
};

#define LR20XX_LF_MIN_PWR (-10)
#define LR20XX_LF_MAX_PWR 22

/* Calibrated per-dBm PA config — each row is [half_power, duty_cycle, slices]
 * for the corresponding output power from -10 to +22 dBm inclusive.
 *
 * 10–22 dBm: from LR2021 datasheet Rev 1.1, Table 7-16 (915MHz ref design).
 *   TX_PARAM is in 0.5dB steps; half_power = TX_PARAM * 2.
 *   For integer dBm targets, we take the exact row from the table.
 *
 * -10 to +9 dBm: from Semtech SDK lr20xx_pa_pwr_cfg.h (not in datasheet).
 */
static const struct lr20xx_pa_pwr_entry pa_lf_table[] = {
	{ -18, 3, 6 }, /* -10 dBm  (SDK) */
	{ -13, 2, 5 }, /*  -9 dBm  (SDK) */
	{ -13, 6, 1 }, /*  -8 dBm  (SDK) */
	{  -6, 6, 0 }, /*  -7 dBm  (SDK) */
	{   4, 1, 0 }, /*  -6 dBm  (SDK) */
	{   4, 2, 0 }, /*  -5 dBm  (SDK) */
	{   2, 1, 3 }, /*  -4 dBm  (SDK) */
	{  14, 0, 0 }, /*  -3 dBm  (SDK) */
	{   9, 0, 3 }, /*  -2 dBm  (SDK) */
	{  11, 3, 0 }, /*  -1 dBm  (SDK) */
	{  16, 1, 0 }, /*   0 dBm  (SDK) */
	{  11, 7, 0 }, /*   1 dBm  (SDK) */
	{  18, 2, 0 }, /*   2 dBm  (SDK) */
	{  16, 5, 0 }, /*   3 dBm  (SDK) */
	{  17, 7, 0 }, /*   4 dBm  (SDK) */
	{  21, 1, 2 }, /*   5 dBm  (SDK) */
	{  25, 3, 0 }, /*   6 dBm  (SDK) */
	{  32, 0, 1 }, /*   7 dBm  (SDK) */
	{  32, 2, 0 }, /*   8 dBm  (SDK) */
	{  27, 3, 1 }, /*   9 dBm  (SDK) */
	{  32, 2, 1 }, /*  10 dBm  (DS Table 7-16: TX_PARAM=16) */
	{  32, 2, 2 }, /*  11 dBm  (DS Table 7-16: TX_PARAM=16) */
	{  30, 5, 1 }, /*  12 dBm  (DS Table 7-16: TX_PARAM=15) */
	{  31, 4, 3 }, /*  13 dBm  (DS Table 7-16: TX_PARAM=15.5) */
	{  34, 4, 2 }, /*  14 dBm  (DS Table 7-16: TX_PARAM=17) */
	{  33, 5, 4 }, /*  15 dBm  (DS Table 7-16: TX_PARAM=16.5) */
	{  36, 4, 4 }, /*  16 dBm  (DS Table 7-16: TX_PARAM=18) */
	{  36, 5, 6 }, /*  17 dBm  (DS Table 7-16: TX_PARAM=18) */
	{  38, 5, 6 }, /*  18 dBm  (DS Table 7-16: TX_PARAM=19) */
	{  39, 6, 6 }, /*  19 dBm  (DS Table 7-16: TX_PARAM=19.5) */
	{  41, 6, 6 }, /*  20 dBm  (DS Table 7-16: TX_PARAM=20.5) */
	{  42, 7, 7 }, /*  21 dBm  (DS Table 7-16: TX_PARAM=21) */
	{  44, 7, 6 }, /*  22 dBm  (DS Table 7-16: TX_PARAM=22) */
};

static void lr20xx_get_pa_cfg_for_power(int8_t power_dbm,
					lr20xx_radio_common_pa_cfg_t *pa,
					int8_t *half_power_out)
{
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
	pa->pa_hf_duty_cycle = 16;

	*half_power_out = e->half_power;
}

/* ── Hardware reset (BUSY stuck recovery) ───────────────────────────── */

static void lr20xx_hardware_reset(struct lr20xx_data *data,
				  const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_WRN("LR2021 hardware reset (BUSY stuck recovery)");

	lr20xx_hal_reset(ctx);

	/* SIMO workaround skipped — LDO mode (see DS §22.6) */

	if (cfg->tcxo_voltage_mv > 0) {
		/* Timeout in RTC ticks (30.52 µs/tick) */
		lr20xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    (cfg->tcxo_startup_delay_ms * 1000U) / 31U);
	}

	/* LDO mode — no cfg_lfclk, no set_reg_mode, no DCDC workarounds */

	lr20xx_configure_rfswitch(ctx, cfg);

	/* DIO9 is the physical IRQ line (pin 15 on NiceRF module → MCU P0.10) */
	lr20xx_system_set_dio_function(ctx, LR20XX_SYSTEM_DIO_9,
				       LR20XX_SYSTEM_DIO_FUNC_IRQ,
				       LR20XX_SYSTEM_DIO_DRIVE_NONE);

	lr20xx_radio_common_set_rx_tx_fallback_mode(ctx,
						    LR20XX_RADIO_FALLBACK_STDBY_RC);

	lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);

	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	lr20xx_system_calibrate(ctx, 0x6F);
	k_msleep(5);

	/* Front-end calibration — single LF frequency (RadioLib approach) */
	{
		lr20xx_radio_common_front_end_calibration_value_t fe_cal = {
			.rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF,
			.frequency_in_hertz = 868000000,
		};
		lr20xx_radio_common_calibrate_front_end_helper(ctx, &fe_cal, 1);
	}

	data->rx_boost_applied = false;

	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	LOG_WRN("LR2021 recovered from hardware reset");
}

/* ── Apply modem configuration ──────────────────────────────────────── */

static void lr20xx_apply_modem_config(struct lr20xx_data *data,
				      const struct lr20xx_config *cfg,
				      bool tx_mode)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;
	lr20xx_status_t rc;

	rc = lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
	LOG_INF("modem_cfg: set_pkt_type=%d", rc);

	rc = lr20xx_radio_common_set_rf_freq(ctx, mc->frequency);
	LOG_INF("modem_cfg: set_rf_freq(%u)=%d", mc->frequency, rc);

	/* Always configure the RX path after setting frequency
	 * (reference does this on every set_rf_freq call). */
	rc = lr20xx_radio_common_set_rx_path(
		ctx, LR20XX_RADIO_COMMON_RX_PATH_LF,
		data->rx_boost_enabled
			? LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4
			: LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
	data->rx_boost_applied = data->rx_boost_enabled;
	LOG_INF("modem_cfg: set_rx_path(LF, boost=%d)=%d",
		data->rx_boost_enabled, rc);

	/* LR20xx uses PPM offset instead of explicit LDRO.
	 * PPM_1_4 (1 bin every 4) is equivalent to LDRO for high-SF
	 * wide-time-on-air configurations. Use recommended value. */
	lr20xx_radio_lora_mod_params_t mod = {
		.sf  = (lr20xx_radio_lora_sf_t)mc->datarate,
		.bw  = bw_enum_to_lr20xx(mc->bandwidth),
		.cr  = cr_enum_to_lr20xx(mc->coding_rate),
		.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(
			(lr20xx_radio_lora_sf_t)mc->datarate,
			bw_enum_to_lr20xx(mc->bandwidth)),
	};
	rc = lr20xx_radio_lora_set_modulation_params(ctx, &mod);
	LOG_INF("modem_cfg: set_mod(SF%d BW%d CR%d PPM%d)=%d",
		mod.sf, mod.bw, mod.cr, mod.ppm, rc);

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
	LOG_INF("modem_cfg: set_pkt(pre=%d len=%d crc=%d iq=%d)=%d",
		pkt.preamble_len_in_symb, pkt.pld_len_in_bytes,
		pkt.crc, pkt.iq, rc);

	rc = lr20xx_radio_lora_set_syncword(ctx,
				       mc->public_network ? 0x34 : 0x12);
	LOG_INF("modem_cfg: set_syncword(0x%02x)=%d",
		mc->public_network ? 0x34 : 0x12, rc);

	if (tx_mode) {
		/* PA config: Semtech SDK 3-byte packed format is CORRECT per
		 * datasheet Table 7-15 (p132):
		 *   Byte 2: (pa_sel<<7) | rfu(4:0) | pa_lf_mode(1:0)
		 *   Byte 3: pa_lf_duty_cycle(3:0) | pa_lf_slices(3:0)
		 *   Byte 4: rfu(2:0) | pa_hf_duty_cycle(4:0)
		 *
		 * RadioLib's 5-byte format was WRONG (misread the datasheet).
		 * The SDK's lr20xx_radio_common_set_pa_cfg() packs correctly.
		 *
		 * PA values from datasheet Table 7-16 (915MHz ref design).
		 */
		lr20xx_radio_common_pa_cfg_t pa;
		int8_t half_power;

		lr20xx_get_pa_cfg_for_power(mc->tx_power, &pa, &half_power);
		rc = lr20xx_radio_common_set_pa_cfg(ctx, &pa);
		LOG_INF("modem_cfg: set_pa_cfg(sel=%d mode=%d duty=%d slices=%d hf_duty=%d)=%d",
			pa.pa_sel, pa.pa_lf_mode, pa.pa_lf_duty_cycle,
			pa.pa_lf_slices, pa.pa_hf_duty_cycle, rc);
		LOG_INF("modem_cfg: PA SPI bytes: 0x%02x 0x%02x 0x%02x",
			(uint8_t)((pa.pa_sel << 7) | pa.pa_lf_mode),
			(uint8_t)((pa.pa_lf_duty_cycle << 4) | pa.pa_lf_slices),
			pa.pa_hf_duty_cycle);

		rc = lr20xx_radio_common_set_tx_params(ctx, half_power,
						  LR20XX_RADIO_COMMON_RAMP_48_US);
		LOG_INF("modem_cfg: set_tx_params(half_pwr=%d ramp=0x05)=%d",
			half_power, rc);

		dump_chip_state(ctx, &data->hal_ctx, "post-PA");
	}

	rc = lr20xx_system_set_dio_irq_cfg(ctx, LR20XX_SYSTEM_DIO_9,
		LR20XX_SYSTEM_IRQ_ALL_MASK &
		~(LR20XX_SYSTEM_IRQ_FIFO_RX | LR20XX_SYSTEM_IRQ_FIFO_TX));
	LOG_INF("modem_cfg: set_dio_irq=%d", rc);

	/* Verify packet type is still LoRa after all config */
	lr20xx_radio_common_pkt_type_t pkt_check = 0xFF;
	lr20xx_radio_common_get_pkt_type(ctx, &pkt_check);

	dump_chip_state(ctx, &data->hal_ctx, tx_mode ? "modem-TX" : "modem-RX");
	LOG_INF("modem_cfg: pkt_type_verify=%d (1=LORA)", pkt_check);
}

/* ── RX duty cycle ──────────────────────────────────────────────────── */

/* Minimum preamble symbols needed for detection (same as SX126x) */
#define LR20XX_DC_MIN_SYMBOLS_SF7_PLUS  8
#define LR20XX_DC_MIN_SYMBOLS_SF6_LESS  12
#define LR20XX_DC_TCXO_DELAY_US         1000

/**
 * Compute duty cycle timing from current modem config and issue the
 * SetRxDutyCycle command.  Returns true on success, false if the
 * preamble is too short (falls back to continuous RX).
 *
 * Mirrors the SX126x symbol-based calculation: sleep through
 * (preamble_len - min_symbols - 2) symbols, wake for the remainder
 * plus margin.
 */
static bool lr20xx_apply_rx_duty_cycle(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	float bw_khz = bw_enum_to_khz(mc->bandwidth);
	uint16_t preamble_len = mc->preamble_len;

	uint16_t min_symbols = (sf >= 7) ? LR20XX_DC_MIN_SYMBOLS_SF7_PLUS
					 : LR20XX_DC_MIN_SYMBOLS_SF6_LESS;

	int16_t sleep_symbols = (int16_t)preamble_len - (int16_t)min_symbols;
	if (sleep_symbols <= 0) {
		LOG_WRN("Preamble too short for duty cycle "
			"(need >%d symbols, have %d) — continuous RX",
			min_symbols, preamble_len);
		data->rx_duty_cycle_enabled = false;
		lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			ctx, 0xFFFFFF);
		return false;
	}

	/* symbol_us = 2^SF / BW_kHz * 1000 */
	uint32_t symbol_us = (uint32_t)((float)(1 << sf) * 1000.0f / bw_khz);

	/* Shave 2 symbols off sleep for timing margin */
	int16_t sleep_symbols_safe = sleep_symbols - 2;
	if (sleep_symbols_safe < 1) {
		sleep_symbols_safe = 1;
	}
	uint32_t sleep_period_us = (uint16_t)sleep_symbols_safe * symbol_us;

	uint32_t preamble_total_us = (preamble_len + 1) * symbol_us;
	int32_t wake_calc1 = ((int32_t)preamble_total_us -
			      ((int32_t)sleep_period_us -
			       LR20XX_DC_TCXO_DELAY_US)) / 2;
	uint32_t wake_calc2 = (min_symbols + 1) * symbol_us;

	uint32_t wake_period_us =
		(wake_calc1 > 0 && (uint32_t)wake_calc1 > wake_calc2)
		? (uint32_t)wake_calc1 : wake_calc2;

	uint32_t rx_ms  = (wake_period_us + 999) / 1000;  /* round up */
	uint32_t slp_ms = (sleep_period_us + 999) / 1000;

	if (rx_ms < 1) {
		rx_ms = 1;
	}
	if (slp_ms < 1) {
		slp_ms = 1;
	}

	lr20xx_radio_common_set_rx_duty_cycle(ctx, rx_ms, slp_ms,
		LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_RX);

	LOG_DBG("RX duty cycle: SF%d rx=%ums sleep=%ums", sf, rx_ms, slp_ms);
	return true;
}

/* ── Start RX (internal) ────────────────────────────────────────────── */

static void lr20xx_start_rx(struct lr20xx_data *data,
			     const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;
	lr20xx_status_t rc;

	LOG_INF("start_rx: t=%lld", k_uptime_get());

	/* Standby first — wake from any state (radio_is_sleeping is managed
	 * by the HAL via sleep opcode detection; do not set it here). */
	rc = lr20xx_system_set_standby_mode(ctx,
					    LR20XX_SYSTEM_STANDBY_MODE_RC);
	LOG_INF("start_rx: standby=%d", rc);
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
		rc = lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			ctx, 0xFFFFFF);
		LOG_INF("start_rx: set_rx(continuous)=%d", rc);
	}

	/* Clear any IRQ flags set during modem configuration */
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	data->in_rx_mode = true;
	data->tx_active = false;

	dump_chip_state(ctx, &data->hal_ctx, "start-RX");
}

/* ── Lightweight RX restart (no modem reconfig) ─────────────────────── */

static void lr20xx_restart_rx(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	if (data->rx_duty_cycle_enabled) {
		lr20xx_apply_rx_duty_cycle(data);
	} else {
		lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
			ctx, 0xFFFFFF);
	}

	data->in_rx_mode = true;
}

/* ── DIO1 IRQ handler (work queue, thread context) ──────────────────── */

static void lr20xx_dio1_callback(void *user_data);

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

	LOG_DBG("DIO1 IRQ: 0x%08x tx=%d t=%lld", irq, data->tx_active,
		k_uptime_get());

	if (irq & LR20XX_SYSTEM_IRQ_ERROR) {
		LOG_WRN("IRQ hardware ERROR: 0x%08x", irq);
	}

	if (irq != 0) {
		data->dio1_stuck_count = 0;
	}

	/* ── RX done ── */
	if (irq & LR20XX_SYSTEM_IRQ_RX_DONE) {
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
			 * accurate reading on weak links. */
			int16_t rssi = pkt_stat.rssi_pkt_in_dbm;
			int8_t snr = ((int8_t)pkt_stat.snr_pkt_raw + 2) >> 2;

			if (snr < 0 &&
			    pkt_stat.rssi_signal_pkt_in_dbm > rssi) {
				rssi = pkt_stat.rssi_signal_pkt_in_dbm;
			}

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
		LOG_DBG("Timeout IRQ — restarting RX");
		if (!data->tx_active) {
			lr20xx_restart_rx(data);
			rx_restarted = true;
		}
	}

	/* ── CRC / Header error ── */
	if (irq & LR20XX_SYSTEM_IRQ_CRC_ERROR ||
	    ((irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) &&
	     !(irq & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID))) {
		LOG_WRN("RX error: CRC=%d HDR=%d",
			(irq & LR20XX_SYSTEM_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) ? 1 : 0);

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

safety_check:
	if (!rx_restarted && data->in_rx_mode && !data->tx_active) {
		LOG_WRN("DIO1 safety: no IRQ handled (0x%08x rc=%d), "
			"restarting RX", irq, rc);
		lr20xx_restart_rx(data);
	}

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
			      struct lora_modem_config *config)
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

	/* Image calibration at operating frequency */
	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* FE cal raw value: ceil(freq / 4MHz), bit 15 = HF flag */
	uint16_t fe_raw = (uint16_t)((config->frequency + 3999999U) / 4000000U);
	LOG_INF("config: FE cal freq=%uHz raw=0x%04x", config->frequency, fe_raw);

	lr20xx_radio_common_front_end_calibration_value_t cal = {
		.rx_path          = LR20XX_RADIO_COMMON_RX_PATH_LF,
		.frequency_in_hertz = config->frequency,
	};
	lr20xx_status_t cal_rc = lr20xx_radio_common_calibrate_front_end_helper(
		&data->hal_ctx, &cal, 1);
	LOG_INF("config: FE cal=%d", cal_rc);

	dump_chip_state(&data->hal_ctx, &data->hal_ctx, "config-FEcal");
	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("config: %uHz SF%d BW%d CR%d pwr=%d tx=%d",
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
	int de = (sf >= 11 && bw <= 125000.0f) ? 1 : 0;
	float n_payload = 8.0f + fmaxf(
		ceilf((8.0f * data_len - 4.0f * sf + 28.0f + 16.0f) /
		      (4.0f * (sf - 2.0f * de))) * cr,
		0.0f);
	float t_preamble = (mc->preamble_len + 4.25f) * ts;
	float t_payload = n_payload * ts;

	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

/* ── Driver API: send_async ─────────────────────────────────────────── */

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

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = NULL;
	data->in_rx_mode = false;

	lr20xx_hal_disable_dio1_irq(&data->hal_ctx);

	LOG_INF("=== TX BEGIN len=%u ===", data_len);
	dump_chip_state(ctx, &data->hal_ctx, "TX-enter");

	/* Standby */
	lr20xx_status_t rc = lr20xx_system_set_standby_mode(ctx,
							    LR20XX_SYSTEM_STANDBY_MODE_RC);
	LOG_INF("TX: standby=%d", rc);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("TX standby failed — HW reset");
		lr20xx_hardware_reset(data, cfg);
	}

	dump_chip_state(ctx, &data->hal_ctx, "TX-standby");

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
	rc = lr20xx_radio_lora_set_packet_params(ctx, &pkt);
	LOG_INF("TX: set_pkt_params(len=%d)=%d", data_len, rc);

	/* Write to TX FIFO */
	rc = lr20xx_radio_fifo_write_tx(ctx, buf, (uint16_t)data_len);
	LOG_INF("TX: fifo_write(%u bytes)=%d", data_len, rc);

	/* Clear ALL errors + IRQs right before set_tx */
	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	dump_chip_state(ctx, &data->hal_ctx, "TX-pre-setTX");

	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	data->tx_signal = async;
	data->tx_active = true;

	/* THE CRITICAL CALL — set_tx sends opcode 0x020D */
	lr20xx_status_t tx_rc = lr20xx_radio_common_set_tx(ctx, 5000);
	LOG_INF("TX: set_tx(5000ms)=%d (0=OK, HAL-level)", tx_rc);

	dump_chip_state(ctx, &data->hal_ctx, "TX-post-setTX");

	/* Wait 10ms and re-check — did the chip stay in TX or fall out? */
	k_msleep(10);
	dump_chip_state(ctx, &data->hal_ctx, "TX-10ms-later");

	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("=== TX END ===");
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

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;

	lr20xx_start_rx(data, cfg);

	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("recv_async: %s%s",
		data->rx_duty_cycle_enabled
			? "RX duty cycle"
			: "continuous RX",
		data->rx_boost_enabled ? ", boosted" : "");

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

int16_t lr20xx_get_rssi_inst(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	int16_t rssi = 0;
	uint8_t half_dbm = 0;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr20xx_radio_common_get_rssi_inst(&data->hal_ctx, &rssi, &half_dbm);
	k_mutex_unlock(&data->spi_mutex);

	return rssi;
}

bool lr20xx_is_receiving(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	if (!data->in_rx_mode || data->tx_active) {
		return false;
	}

	/* Use non-destructive get_status to check for preamble/header
	 * without racing the DIO1 work handler. */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return false;
	}

	lr20xx_system_irq_mask_t irq = 0;
	lr20xx_system_get_status(&data->hal_ctx, NULL, NULL, &irq);
	k_mutex_unlock(&data->spi_mutex);

	return (irq & (LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
		       LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID)) != 0;
}

void lr20xx_set_rx_duty_cycle(const struct device *dev, bool enable)
{
	struct lr20xx_data *data = dev->data;

	if (data->rx_duty_cycle_enabled == enable) {
		return;
	}

	data->rx_duty_cycle_enabled = enable;
	LOG_INF("RX duty cycle %s (takes effect on next RX start)",
		enable ? "enabled" : "disabled");
}

void lr20xx_set_rx_boost(const struct device *dev, bool enable)
{
	struct lr20xx_data *data = dev->data;

	if (data->rx_boost_enabled == enable) {
		return;
	}

	data->rx_boost_enabled = enable;
	LOG_INF("RX boost %s", enable ? "enabled" : "disabled");

	if (data->in_rx_mode && data->configured) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		lr20xx_radio_common_set_rx_path(
			&data->hal_ctx, LR20XX_RADIO_COMMON_RX_PATH_LF,
			enable ? LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4
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

void lr20xx_reset_agc(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	void *ctx = &data->hal_ctx;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Warm sleep — powers down analog frontend (resets AGC gain state).
	 * is_ram_retention_enabled=true = warm sleep (equivalent to LR11xx warm_start). */
	lr20xx_system_sleep_cfg_t sleep_cfg = {
		.is_clk_32k_enabled       = false,
		.is_ram_retention_enabled = true,
	};
	lr20xx_system_set_sleep_mode(ctx, &sleep_cfg, 0);
	k_sleep(K_USEC(500));

	lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC);

	lr20xx_system_calibrate(ctx, 0x6F);

	if (data->configured) {
		lr20xx_radio_common_front_end_calibration_value_t cal = {
			.rx_path          = LR20XX_RADIO_COMMON_RX_PATH_LF,
			.frequency_in_hertz = data->modem_cfg.frequency,
		};
		lr20xx_radio_common_calibrate_front_end_helper(ctx, &cal, 1);
	}

	if (data->rx_boost_enabled) {
		lr20xx_radio_common_set_rx_path(
			ctx, LR20XX_RADIO_COMMON_RX_PATH_LF,
			LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4);
		data->rx_boost_applied = true;
	}

	k_mutex_unlock(&data->spi_mutex);
}

/* ── Deferred hardware init ─────────────────────────────────────────── */

static int lr20xx_hw_init(struct lr20xx_data *data,
			  const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_INF("LR20xx hardware init starting");

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

	/* CRITICAL: Read raw 4 bytes from GET_VERSION (opcode 0x0101) to
	 * determine if this is LR11x0 or LR20xx silicon.
	 * LR11x0 returns: [hw_type, device_use, fw_major, fw_minor] (4 bytes)
	 * LR20xx returns: [major, minor] (2 bytes, extra bytes would be 0x00)
	 * If byte[0]=0x01/0x02/0x03, it's LR1110/LR1120/LR1121 (LR11x0!) */
	{
		const uint8_t cmd[2] = { 0x01, 0x01 };
		uint8_t raw[4] = { 0 };
		lr20xx_hal_read(ctx, cmd, 2, raw, 4);
		LOG_INF("GET_VERSION raw bytes: 0x%02x 0x%02x 0x%02x 0x%02x",
			raw[0], raw[1], raw[2], raw[3]);
		if (raw[0] == 0x01) {
			LOG_WRN("*** CHIP IDENTIFIES AS LR1110 (LR11x0 family!) ***");
		} else if (raw[0] == 0x02) {
			LOG_WRN("*** CHIP IDENTIFIES AS LR1120 (LR11x0 family!) ***");
		} else if (raw[0] == 0x03) {
			LOG_WRN("*** CHIP IDENTIFIES AS LR1121 (LR11x0 family!) ***");
		} else {
			LOG_INF("Chip type byte=0x%02x (LR20xx if not 0x01-0x03)", raw[0]);
		}
	}

	dump_chip_state(ctx, &data->hal_ctx, "post-reset");

	/* SIMO DC-DC workaround REMOVED — datasheet §22.6 says it's only
	 * needed when SetRegMode simo_usage=0x02 (SIMO_NORMAL).
	 * We run in LDO mode (default, simo_usage=0x00). */
	LOG_INF("init: LDO mode — SIMO workaround skipped (per DS §22.6)");

	if (cfg->tcxo_voltage_mv > 0) {
		uint32_t tcxo_ticks = (cfg->tcxo_startup_delay_ms * 1000U) / 31U;
		lr20xx_status_t tcxo_rc = lr20xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    tcxo_ticks);
		LOG_INF("init: set_tcxo(%dmV, %u ticks)=%d",
			cfg->tcxo_voltage_mv, tcxo_ticks, tcxo_rc);
	} else {
		LOG_INF("init: TCXO disabled (XTAL mode)");
	}

	/* RadioLib does NOT call cfg_lfclk or set_reg_mode.
	 * Stay in LDO mode (chip default after reset).
	 * DCDC mode + wrong SET_REG_MODE encoding was likely
	 * preventing TX. */
	lr20xx_status_t st;
	LOG_INF("init: LDO mode (RadioLib-compatible, no DCDC)");

	lr20xx_configure_rfswitch(ctx, cfg);
	LOG_INF("RF switch: en=0x%02x stby=0x%02x rx=0x%02x tx=0x%02x txhp=0x%02x",
		cfg->rfswitch_enable, cfg->rfswitch_standby, cfg->rfswitch_rx,
		cfg->rfswitch_tx, cfg->rfswitch_tx_hp);

	st = lr20xx_system_set_dio_function(ctx, LR20XX_SYSTEM_DIO_9,
				       LR20XX_SYSTEM_DIO_FUNC_IRQ,
				       LR20XX_SYSTEM_DIO_DRIVE_NONE);
	LOG_INF("init: set_dio9_func(IRQ)=%d", st);

	st = lr20xx_radio_common_set_rx_tx_fallback_mode(ctx,
						    LR20XX_RADIO_FALLBACK_STDBY_RC);
	LOG_INF("init: set_fallback(STDBY_RC)=%d", st);

	dump_chip_state(ctx, &data->hal_ctx, "pre-cal");

	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	/* Calibrate all analog blocks: 0x6F = LF_RC|HF_RC|PLL|AAF|MU|PA_OFF */
	st = lr20xx_system_calibrate(ctx, 0x6F);
	LOG_INF("init: calibrate(0x6F)=%d", st);

	/* RadioLib waits for BUSY to go LOW after calibrate.
	 * We poll the BUSY pin (max 500ms timeout). */
	{
		int64_t cal_start = k_uptime_get();
		while (gpio_pin_get_dt(&data->hal_ctx.busy)) {
			k_msleep(1);
			if ((k_uptime_get() - cal_start) > 500) {
				LOG_ERR("BUSY stuck HIGH after calibrate!");
				break;
			}
		}
		LOG_INF("init: calibrate BUSY wait %lldms",
			k_uptime_get() - cal_start);
	}

	dump_chip_state(ctx, &data->hal_ctx, "post-cal");

	/* Front-end calibration at 868 MHz LF.
	 * raw_value = ceil(868000000/4000000) = 217 = 0x00D9 */
	lr20xx_radio_common_front_end_calibration_value_t fe_cal[3] = {
		{ .rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF,
		  .frequency_in_hertz = 868000000 },
		{ .rx_path = 0, .frequency_in_hertz = 0 },
		{ .rx_path = 0, .frequency_in_hertz = 0 },
	};
	st = lr20xx_radio_common_calibrate_front_end_helper(ctx, fe_cal, 1);
	LOG_INF("init: FE_cal(868MHz LF, raw=0x00D9)=%d", st);

	dump_chip_state(ctx, &data->hal_ctx, "post-FEcal");

	/* Verify: set packet type to LoRa and read it back */
	st = lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
	LOG_INF("init: set_pkt_type(LORA)=%d", st);

	/* dcdc_reset removed — not needed in LDO mode, RadioLib doesn't do it */

	lr20xx_radio_common_pkt_type_t pkt_readback = 0xFF;
	lr20xx_radio_common_get_pkt_type(ctx, &pkt_readback);
	LOG_INF("init: pkt_type readback=%d (expect 1=LORA)", pkt_readback);

	dump_chip_state(ctx, &data->hal_ctx, "init-done");

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
	k_work_init(&data->dio1_work, lr20xx_dio1_work_handler);

	k_work_queue_start(&data->dio1_wq, lr20xx_dio1_wq_stack,
			   K_THREAD_STACK_SIZEOF(lr20xx_dio1_wq_stack),
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&data->dio1_wq.thread, "lr20xx_dio1");

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
	.config     = lr20xx_lora_config,
	.airtime    = lr20xx_lora_airtime,
	.send       = lr20xx_lora_send,
	.send_async = lr20xx_lora_send_async,
	.recv       = lr20xx_lora_recv,
	.recv_async = lr20xx_lora_recv_async,
};

#define LR20XX_INIT(n)                                                       \
	static const struct lr20xx_config lr20xx_config_##n = {              \
		.bus = SPI_DT_SPEC_INST_GET(n,                               \
			SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |               \
			SPI_TRANSFER_MSB),                                   \
		.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),              \
		.busy  = GPIO_DT_SPEC_INST_GET(n, busy_gpios),              \
		.dio1  = GPIO_DT_SPEC_INST_GET(n, dio1_gpios),              \
		.tcxo_voltage_mv =                                           \
			DT_INST_PROP_OR(n, tcxo_voltage_mv, 0),             \
		.tcxo_startup_delay_ms =                                     \
			DT_INST_PROP_OR(n, tcxo_startup_delay_ms, 5),       \
		.rx_boosted       = DT_INST_PROP(n, rx_boosted),            \
		.rfswitch_enable  = DT_INST_PROP_OR(n, rfswitch_enable, 0), \
		.rfswitch_standby = DT_INST_PROP_OR(n, rfswitch_standby, 0),\
		.rfswitch_rx      = DT_INST_PROP_OR(n, rfswitch_rx, 0),     \
		.rfswitch_tx      = DT_INST_PROP_OR(n, rfswitch_tx, 0),     \
		.rfswitch_tx_hp   = DT_INST_PROP_OR(n, rfswitch_tx_hp, 0),  \
		.pa_hp_sel        = DT_INST_PROP_OR(n, pa_hp_sel, 7),       \
		.pa_duty_cycle    = DT_INST_PROP_OR(n, pa_duty_cycle, 4),   \
	};                                                                   \
	static struct lr20xx_data lr20xx_data_##n;                           \
	DEVICE_DT_INST_DEFINE(n, lr20xx_lora_init, NULL,                     \
			      &lr20xx_data_##n, &lr20xx_config_##n,          \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,        \
			      &lr20xx_lora_api);

DT_INST_FOREACH_STATUS_OKAY(LR20XX_INIT)
