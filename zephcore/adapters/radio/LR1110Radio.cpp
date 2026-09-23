/*
 * SPDX-License-Identifier: MIT
 * LR1110 hardware hooks for LoRaRadioBase.
 */

#include "LR1110Radio.h"
#include <zephyr/kernel.h>

/* LR11xx driver extension API */
extern "C" {
#include <zephyr/drivers/lora/lr11xx_lora.h>
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr1110_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

LR1110Radio::LR1110Radio(const struct device *lora_dev, MainBoard &board)
	: LoRaRadioBase(lora_dev, board)
{
}

uint32_t LR1110Radio::getDutyCycleTimeoutRestarts() const
{
	return lr11xx_get_dc_timeout_restarts(_dev);
}

void LR1110Radio::resetDutyCycleTimeoutRestarts()
{
	lr11xx_reset_dc_timeout_restarts(_dev);
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

bool LR1110Radio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return false;
	}
	return true;
}

void LR1110Radio::hwCancelReceive()
{
	lora_recv_async(_dev, NULL, NULL);
}

int LR1110Radio::hwSendAsync(uint8_t *buf, uint32_t len,
			     struct k_poll_signal *sig)
{
	return lora_send_async(_dev, buf, len, sig);
}

int16_t LR1110Radio::hwGetCurrentRSSI()
{
	return lr11xx_get_rssi_inst(_dev);
}

int LR1110Radio::hwGetRssiBurst(int16_t *out, int n, uint32_t spacing_us)
{
	return lr11xx_get_rssi_burst(_dev, out, n, spacing_us);
}

bool LR1110Radio::hwIsReceiving()
{
	/* MUST be non-destructive: never clear IRQ bits from this path.
	 * Foreign-preamble release is hardware-driven (chip-internal release
	 * on HEADER_ERROR / sync timeout). The driver's lr11xx_is_receiving()
	 * reads IRQ status without clearing. */
	return lr11xx_is_receiving(_dev);
}

/* No hwResetAgc() override, and no hwNeedsAgcReset(): this part has no
 * jammed-AGC fault to remedy — that one belongs to the SX126x.  The driver
 * exposes lr11xx_recalibrate() instead, which is the temperature-drift path
 * this family genuinely does need, named for what it actually does. */


void LR1110Radio::hwRecalibrate()
{
	lr11xx_recalibrate(_dev);
}

/* This family's UM gives an explicit temperature threshold for image
 * calibration, so drift recalibration is active here.  The temperature itself
 * comes from the board, not from lr11xx_get_chip_temp_c() — see
 * LoRaRadioBase::imageCalMaintenance() for why the radio is not asked. */
bool LR1110Radio::hwHasDriftRecal()
{
	return true;
}

void LR1110Radio::hwSetRxBoost(bool enable)
{
	lr11xx_set_rx_boost(_dev, enable);
}

uint32_t LR1110Radio::hwWakeupTimeUs()
{
	return lr11xx_get_wakeup_time_us(_dev);
}

int LR1110Radio::hwCadProbe(int8_t level)
{
	return lr11xx_cad_probe(_dev, level);
}

int LR1110Radio::hwCadRxOutcome()
{
	return lr11xx_cad_rx_outcome(_dev);
}

uint32_t LR1110Radio::hwCadRxTimeoutMs()
{
	return lr11xx_cad_rx_timeout_ms(_dev);
}

void LR1110Radio::hwCadSetPeakOffset(int8_t offset)
{
	lr11xx_cad_set_peak_offset(_dev, offset);
}

uint8_t LR1110Radio::hwCadBasePeak()
{
	return lr11xx_cad_base_peak(_dev);
}

/* The detPeak range lr11xx_do_cad() will actually program.  Must match the
 * driver's clamp exactly: if the adapter thinks the range is wider, the
 * staircase explores offsets that collapse onto one peak and reads the noise
 * between them as curvature.  Same reasoning as LR2021Radio::hwCadPeakMin —
 * and the same symptom was observed here on T1000-E companions, which sat at
 * o:-8 with the driver's old floor of 48 already reached. */
uint8_t LR1110Radio::hwCadPeakMin()
{
	return lr11xx_cad_peak_min();
}

uint8_t LR1110Radio::hwCadPeakMax()
{
	return lr11xx_cad_peak_max();
}

} /* namespace mesh */
