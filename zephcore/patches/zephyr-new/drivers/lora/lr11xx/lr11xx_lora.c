/*
 * SPDX-License-Identifier: Apache-2.0
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

#include "lr11xx_lora.h"
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
	bool rx_duty_cycle_enabled; /* unused — LR1110 always continuous RX */
	bool rx_boost_enabled;
	bool rx_boost_applied;  /* RX boost register written to hardware */

	/* CAD state */
	lora_cad_cb cad_cb;
	void *cad_user_data;
	struct k_sem cad_sem;
	int cad_result;
	bool cad_active;

	/* Deferred hardware init — heavy SPI/radio work runs on first config() */
	bool hw_initialized;

	/* DIO1 stuck-HIGH detection: counts consecutive empty IRQ cycles.
	 * If DIO1 stays HIGH with no actionable IRQ for too many cycles,
	 * the LR1110 is hung — trigger a hardware reset. */
	int dio1_stuck_count;

	/* RX data buffer — filled in DIO1 handler, passed to callback */
	uint8_t rx_buf[256];
};

/* ── Helpers ────────────────────────────────────────────────────────── */

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

static void lr11xx_apply_modem_config(struct lr11xx_data *data,
				      const struct lr11xx_config *cfg,
				      bool tx_mode)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	lr11xx_radio_set_rf_freq(ctx, mc->frequency);

	/* LDRO must be enabled when symbol time > 16.38ms (SF11+/BW125 etc) */
	uint32_t bw_hz = (uint32_t)(bw_enum_to_khz(mc->bandwidth) * 1000.0f);
	uint32_t symbol_time_us = ((1U << (uint8_t)mc->datarate) * 1000000U) / bw_hz;
	uint8_t ldro = (symbol_time_us > 16380) ? 1 : 0;

	lr11xx_radio_mod_params_lora_t mod = {
		.sf   = (lr11xx_radio_lora_sf_t)mc->datarate,
		.bw   = bw_enum_to_lr11xx(mc->bandwidth),
		.cr   = cr_enum_to_lr11xx(mc->coding_rate),
		.ldro = ldro,
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
		lr11xx_radio_set_tx_params(ctx, mc->tx_power,
					   LR11XX_RADIO_RAMP_48_US);
		lr11xx_radio_pa_cfg_t pa = {
			.pa_sel = LR11XX_RADIO_PA_SEL_HP,
			.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
			.pa_duty_cycle = cfg->pa_duty_cycle,
			.pa_hp_sel = cfg->pa_hp_sel,
		};
		lr11xx_radio_set_pa_cfg(ctx, &pa);
	}

	lr11xx_system_set_dio_irq_params(
		ctx,
		LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TX_DONE |
		LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR |
		LR11XX_SYSTEM_IRQ_HEADER_ERROR,
		0);
}

/* ── RX duty cycle — disabled on LR1110 ─────────────────────────────── */

/* LR1110 SetRxDutyCycle is fundamentally broken: both MODE_RX and
 * MODE_CAD fail to detect in-progress preambles, dropping 23-40% of
 * packets depending on the sleep fraction.  The SX1262 handles this
 * correctly.  LR1110 always uses continuous RX instead. */

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

	/* Apply modem config for RX */
	lr11xx_apply_modem_config(data, cfg, false);

	/* Apply RX boost — persistent register, only written once.
	 * Deferred from hw_init to here so radio is fully configured. */
	if (data->rx_boost_enabled && !data->rx_boost_applied) {
		lr11xx_radio_cfg_rx_boosted(ctx, true);
		data->rx_boost_applied = true;
	}

	/* Start continuous RX.
	 * 0xFFFFFF is the magic RTC-step value for continuous RX.
	 * Must use the raw RTC-step API — set_rx() converts from ms,
	 * which overflows uint32_t and gives a ~131 s timeout instead.
	 * LR1110 always uses continuous RX (duty cycle is broken). */
	lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);

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
static void lr11xx_restart_rx(struct lr11xx_data *data)
{
	void *ctx = &data->hal_ctx;

	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);

	/* RX boost persists through SetRx — no re-apply needed. */
	data->in_rx_mode = true;
}

/* ── DIO1 IRQ handler (work queue, thread context) ──────────────────── */

static void lr11xx_dio1_callback(void *user_data);

static void lr11xx_dio1_work_handler(struct k_work *work)
{
	struct lr11xx_data *data = CONTAINER_OF(work, struct lr11xx_data,
						dio1_work);
	const struct lr11xx_config *cfg = data->dev->config;
	void *ctx = &data->hal_ctx;
	bool rx_restarted = false;

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

	/* ── RX done ── */
	if (irq & LR11XX_SYSTEM_IRQ_RX_DONE) {
		lr11xx_radio_rx_buffer_status_t rx_stat;
		lr11xx_radio_get_rx_buffer_status(ctx, &rx_stat);

		if (rx_stat.pld_len_in_bytes > 0 &&
		    rx_stat.pld_len_in_bytes <= 255) {
			lr11xx_radio_pkt_status_lora_t pkt_stat;
			lr11xx_radio_get_lora_pkt_status(ctx, &pkt_stat);

			lr11xx_regmem_read_buffer8(ctx, data->rx_buf,
						   rx_stat.buffer_start_pointer,
						   rx_stat.pld_len_in_bytes);

			/* LR1110 errata: RX buffer base shifts 4 bytes per
			 * received packet.  Without clearing, after ~64
			 * packets the offset wraps the 256-byte buffer and
			 * corrupts data.  RadioLib does the same clear. */
			lr11xx_regmem_clear_rxbuffer(ctx);

			/* Lightweight RX restart — no reconfig needed */
			lr11xx_restart_rx(data);
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
		lr11xx_restart_rx(data);
		rx_restarted = true;
	}

	/* ── CAD done ── */
	if (irq & LR11XX_SYSTEM_IRQ_CAD_DONE) {
		bool detected = (irq & LR11XX_SYSTEM_IRQ_CAD_DETECTED) != 0;

		data->cad_active = false;

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
		if (!data->tx_active) {
			lr11xx_restart_rx(data);
			rx_restarted = true;
		}
	}

	/* ── CRC / Header error ── */
	if (irq & LR11XX_SYSTEM_IRQ_CRC_ERROR ||
	    ((irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) &&
	     !(irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID))) {
		LOG_DBG("RX error: CRC=%d HDR=%d",
			(irq & LR11XX_SYSTEM_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) ? 1 : 0);

		/* LR1110 errata: header error can shift the RX buffer
		 * pointer by 4 bytes. Standby clears the shift. */
		if (irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) {
			lr11xx_system_set_standby(ctx,
						  LR11XX_SYSTEM_STANDBY_CFG_RC);
		}

		if (!data->tx_active) {
			lr11xx_restart_rx(data);
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
		lr11xx_restart_rx(data);
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
			      struct lora_modem_config *config)
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

/* Forward declaration — needed by LBT in send_async */
static int lr11xx_lora_cad(const struct device *dev, k_timeout_t timeout);

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
		int cad_ret = lr11xx_lora_cad(dev, K_MSEC(200));
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
	lr11xx_hal_enable_dio1_irq(&data->hal_ctx);

	/* Store signal and start TX */
	data->tx_signal = async;
	data->tx_active = true;
	lr11xx_radio_set_tx(ctx, 5000);

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

int16_t lr11xx_get_rssi_inst(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;
	int8_t rssi;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr11xx_radio_get_rssi_inst(&data->hal_ctx, &rssi);
	k_mutex_unlock(&data->spi_mutex);

	return (int16_t)rssi;
}

bool lr11xx_is_receiving(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	/* Non-blocking — skip if SPI busy */
	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return false;
	}

	lr11xx_system_irq_mask_t irq;
	lr11xx_system_get_irq_status(&data->hal_ctx, &irq);
	k_mutex_unlock(&data->spi_mutex);

	return (irq & (LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
		       LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID)) != 0;
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

void lr11xx_reset_agc(const struct device *dev)
{
	struct lr11xx_data *data = dev->data;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Warm sleep — powers down analog frontend (resets AGC gain state) */
	lr11xx_system_sleep_cfg_t sleep_cfg = {
		.is_warm_start = true,
		.is_rtc_timeout = false,
	};
	lr11xx_system_set_sleep(&data->hal_ctx, sleep_cfg, 0);
	k_sleep(K_USEC(500));

	/* Wake to STDBY_RC for calibration */
	lr11xx_system_set_standby(&data->hal_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);

	/* Recalibrate all blocks */
	lr11xx_system_calibrate(&data->hal_ctx, 0x3F);

	/* Re-calibrate image rejection for the actual operating frequency */
	if (data->configured) {
		uint16_t freq_mhz = data->modem_cfg.frequency / 1000000;
		lr11xx_system_calibrate_image_in_mhz(&data->hal_ctx,
						     freq_mhz - 4, freq_mhz + 4);
	}

	/* Re-apply RX boost if it was enabled */
	if (data->rx_boost_enabled) {
		lr11xx_radio_cfg_rx_boosted(&data->hal_ctx, true);
		lr11xx_system_clear_irq_status(&data->hal_ctx,
					       LR11XX_SYSTEM_IRQ_ALL_MASK);
		data->rx_boost_applied = true;
	}

	k_mutex_unlock(&data->spi_mutex);
}

/* ── Deferred hardware init (runs on first lora_config call) ────────── */

/* ── Driver API: CAD ────────────────────────────────────────────────── */

/* Recommended cad_detect_peak values per SF for 2-symbol CAD.
 * From Semtech SX1261/62/68 / LR1110 reference (same silicon IP). */
static uint8_t lr11xx_cad_detect_peak(uint8_t sf)
{
	switch (sf) {
	case 5:  case 6:  return 56;
	case 7:           return 56;
	case 8:           return 58;
	case 9:           return 58;
	case 10:          return 60;
	case 11:          return 64;
	case 12:          return 68;
	default:          return 60;
	}
}

static int lr11xx_do_cad(struct lr11xx_data *data)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	uint8_t symb_nb = 2;
	uint8_t detect_peak = lr11xx_cad_detect_peak(sf);

	if (mc->cad.symbol_num != 0) {
		symb_nb = (uint8_t)mc->cad.symbol_num;
	}
	if (mc->cad.detection_peak != 0) {
		detect_peak = mc->cad.detection_peak;
	}

	lr11xx_radio_cad_params_t cad = {
		.cad_symb_nb = symb_nb,
		.cad_detect_peak = detect_peak,
		.cad_detect_min = mc->cad.detection_minimum ? mc->cad.detection_minimum : 10,
		.cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC,
		.cad_timeout = 0,
	};

	lr11xx_radio_set_cad_params(ctx, &cad);

	lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	data->cad_active = true;
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
		data->cad_active = false;
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
		data->cad_active = false;
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
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&data->dio1_wq.thread, "lr11xx_dio1");

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
	/* .recv_duty_cycle = NULL — LR1110 duty cycle broken */
};

#define LR11XX_INIT(n)                                                     \
	static const struct lr11xx_config lr11xx_config_##n = {            \
		.bus = SPI_DT_SPEC_INST_GET(n,                             \
			SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |             \
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
