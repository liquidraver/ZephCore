/*
 * SPDX-License-Identifier: Apache-2.0
 * LR1110 hardware hooks for LoRaRadioBase.
 */

#include "LR1110Radio.h"
#include <zephyr/kernel.h>

/* LR11xx driver extension API */
extern "C" {
#include "lr11xx_lora.h"
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr1110_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

K_THREAD_STACK_DEFINE(lr11xx_tx_wait_stack, TX_WAIT_THREAD_STACK_SIZE);

LR1110Radio::LR1110Radio(const struct device *lora_dev, MainBoard &board,
			 NodePrefs *prefs)
	: LoRaRadioBase(lora_dev, board, prefs)
{
}

void LR1110Radio::begin()
{
	startTxThread(lr11xx_tx_wait_stack,
		      K_THREAD_STACK_SIZEOF(lr11xx_tx_wait_stack));
	LoRaRadioBase::begin();
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

void LR1110Radio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, const_cast<struct lora_modem_config *>(&cfg));
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
	}
}

void LR1110Radio::hwStartReceive()
{
	int ret = lora_recv_async(_dev, rxCallbackStatic, this);
	if (ret < 0) {
		LOG_ERR("lora_recv_async failed: %d", ret);
		atomic_set(&_in_recv_mode, 0);
		return;
	}
	atomic_set(&_in_recv_mode, 1);

	/* RX boost: set once via setRxBoost(), LR1110 SetRxBoosted
	 * command persists through SetRx calls. */
	if (_rx_duty_cycle_enabled) {
		lr11xx_set_rx_duty_cycle(_dev, true);
	}
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

bool LR1110Radio::hwIsPreambleDetected()
{
	return lr11xx_is_receiving(_dev);
}

void LR1110Radio::hwSetRxBoost(bool enable)
{
	lr11xx_set_rx_boost(_dev, enable);
}

void LR1110Radio::hwSetRxDutyCycle(bool enable)
{
	lr11xx_set_rx_duty_cycle(_dev, enable);
}

void LR1110Radio::hwResetAGC()
{
	/* Warm sleep → Calibrate(ALL) → re-calibrate image → re-apply RX boost */
	lr11xx_reset_agc(_dev);
}

} /* namespace mesh */
