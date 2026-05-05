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

bool LR1110Radio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, const_cast<struct lora_modem_config *>(&cfg));
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

bool LR1110Radio::hwIsPreambleDetected()
{
	return lr11xx_is_receiving(_dev);
}

void LR1110Radio::hwSetRxBoost(bool enable)
{
	lr11xx_set_rx_boost(_dev, enable);
}

void LR1110Radio::hwResetAGC()
{
	/* Warm sleep → Calibrate(ALL) → re-calibrate image → re-apply RX boost */
	lr11xx_reset_agc(_dev);
}

} /* namespace mesh */
