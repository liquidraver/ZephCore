/*
 * SPDX-License-Identifier: Apache-2.0
 * LR2021 hardware hooks for LoRaRadioBase.
 */

#include "LR2021Radio.h"
#include <zephyr/kernel.h>

/* LR20xx driver extension API */
extern "C" {
#include "lr20xx_lora.h"
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr2021_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

K_THREAD_STACK_DEFINE(lr20xx_tx_wait_stack, TX_WAIT_THREAD_STACK_SIZE);

LR2021Radio::LR2021Radio(const struct device *lora_dev, MainBoard &board,
			 NodePrefs *prefs)
	: LoRaRadioBase(lora_dev, board, prefs)
{
}

void LR2021Radio::begin()
{
	startTxThread(lr20xx_tx_wait_stack,
		      K_THREAD_STACK_SIZEOF(lr20xx_tx_wait_stack));
	LoRaRadioBase::begin();
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

bool LR2021Radio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, const_cast<struct lora_modem_config *>(&cfg));
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return false;
	}
	return true;
}

void LR2021Radio::hwCancelReceive()
{
	lora_recv_async(_dev, NULL, NULL);
}

int LR2021Radio::hwSendAsync(uint8_t *buf, uint32_t len,
			     struct k_poll_signal *sig)
{
	return lora_send_async(_dev, buf, len, sig);
}

int16_t LR2021Radio::hwGetCurrentRSSI()
{
	return lr20xx_get_rssi_inst(_dev);
}

bool LR2021Radio::hwIsPreambleDetected()
{
	return lr20xx_is_receiving(_dev);
}

void LR2021Radio::hwSetRxBoost(bool enable)
{
	lr20xx_set_rx_boost(_dev, enable);
}

void LR2021Radio::hwResetAGC()
{
	lr20xx_reset_agc(_dev);
}

} /* namespace mesh */
