/*
 * SPDX-License-Identifier: Apache-2.0
 * SX126x hardware hooks for LoRaRadioBase — native Zephyr driver.
 */

#include "SX126xRadio.h"
#include <zephyr/kernel.h>

/* Native SX126x driver extension API */
extern "C" {
#include "sx126x_ext.h"
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sx126x_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

K_THREAD_STACK_DEFINE(sx126x_tx_wait_stack, TX_WAIT_THREAD_STACK_SIZE);

SX126xRadio::SX126xRadio(const struct device *lora_dev, MainBoard &board,
			  NodePrefs *prefs)
	: LoRaRadioBase(lora_dev, board, prefs)
{
}

void SX126xRadio::begin()
{
	startTxThread(sx126x_tx_wait_stack,
		      K_THREAD_STACK_SIZEOF(sx126x_tx_wait_stack));
	LoRaRadioBase::begin();

#if IS_ENABLED(CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH)
	/* Apply undocumented register 0x8B5 RX improvement (MeshCore PR#1398).
	 * Must run after lora_config() has been called (via startReceive above). */
	sx126x_apply_heltec_reg_patch(_dev);
	LOG_INF("Applied Heltec reg 0x8B5 RX patch");
#endif
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

void SX126xRadio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, const_cast<struct lora_modem_config *>(&cfg));
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
	}
}

void SX126xRadio::hwCancelReceive()
{
	lora_recv_async(_dev, NULL, NULL);
}

int SX126xRadio::hwSendAsync(uint8_t *buf, uint32_t len,
			     struct k_poll_signal *sig)
{
	return lora_send_async(_dev, buf, len, sig);
}

int16_t SX126xRadio::hwGetCurrentRSSI()
{
	return sx126x_get_rssi_inst(_dev);
}

bool SX126xRadio::hwIsPreambleDetected()
{
	return sx126x_is_receiving(_dev);
}

void SX126xRadio::hwSetRxBoost(bool enable)
{
	sx126x_set_rx_boost(_dev, enable);
}

void SX126xRadio::hwResetAGC()
{
	/* Warm sleep → Calibrate(ALL) → re-calibrate image → re-apply RX settings */
	sx126x_reset_agc(_dev);
}

bool SX126xRadio::hwIsChipBusy()
{
	return sx126x_is_chip_busy(_dev);
}

} /* namespace mesh */
