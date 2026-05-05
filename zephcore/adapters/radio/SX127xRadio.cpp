/*
 * SPDX-License-Identifier: Apache-2.0
 * SX127x hardware hooks for LoRaRadioBase — Zephyr loramac-node driver.
 *
 * The SX127x driver (loramac-node/sx127x.c) exposes only the standard
 * Zephyr LoRa API.  Chip-specific features that the SX126x native driver
 * provides via sx126x_ext.h are not available here:
 *
 *   hwGetCurrentRSSI()    — returns -80 dBm sentinel (no hardware path)
 *   hwIsPreambleDetected()— always false (no preamble-detect IRQ exposed)
 *   hwSetRxBoost()        — no-op (SX127x has no RX boost register)
 *   hwResetAGC()          — no-op (loramac-node manages AGC internally)
 *   hwIsChipBusy()        — inherited false (no BUSY pin on SX127x)
 *
 * Everything else (configure, send, receive) uses the standard API.
 */

#include "SX127xRadio.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sx127x_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

K_THREAD_STACK_DEFINE(sx127x_tx_wait_stack, TX_WAIT_THREAD_STACK_SIZE);

SX127xRadio::SX127xRadio(const struct device *lora_dev, MainBoard &board,
			  NodePrefs *prefs)
	: LoRaRadioBase(lora_dev, board, prefs)
{
	/* SX127x has no RX boost feature — start with boost disabled */
	_rx_boost_enabled = false;
	/* loramac-node requires full lora_config() on every TX/RX direction
	 * change — Radio.SetTxConfig() and Radio.SetRxConfig() configure
	 * completely disjoint internal state in the loramac-node library. */
	_loramac_node = true;
}

void SX127xRadio::begin()
{
	startTxThread(sx127x_tx_wait_stack,
		      K_THREAD_STACK_SIZEOF(sx127x_tx_wait_stack));
	LoRaRadioBase::begin();
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

bool SX127xRadio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, const_cast<struct lora_modem_config *>(&cfg));
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return false;
	}
	return true;
}

void SX127xRadio::hwCancelReceive()
{
	lora_recv_async(_dev, NULL, NULL);
}

int SX127xRadio::hwSendAsync(uint8_t *buf, uint32_t len,
			     struct k_poll_signal *sig)
{
	return lora_send_async(_dev, buf, len, sig);
}

int16_t SX127xRadio::hwGetCurrentRSSI()
{
	/* SX127x loramac-node driver does not expose an instantaneous RSSI
	 * function via the standard Zephyr API.  Return a plausible noise-floor
	 * sentinel so LoRaRadioBase::triggerNoiseFloorCalibrate() converges
	 * to a reasonable value rather than being seeded with garbage. */
	return -80;
}

bool SX127xRadio::hwIsPreambleDetected()
{
	/* No preamble-detect IRQ accessible through the standard Zephyr LoRa
	 * API for the loramac-node driver.  Returning false means TX will
	 * not abort for an in-progress preamble — acceptable on SX127x. */
	return false;
}

void SX127xRadio::hwSetRxBoost(bool enable)
{
	/* SX127x does not have a dedicated RX boost / high-sensitivity mode
	 * switch.  Sensitivity is controlled via lora_config tx_power and
	 * the DTS power-amplifier-output property.  Nothing to do here. */
	ARG_UNUSED(enable);
}

void SX127xRadio::hwResetAGC()
{
	/* The loramac-node SX127x driver manages AGC recalibration internally
	 * (RadioSetRxConfig re-programs all gain registers on every RX config
	 * call).  No explicit AGC reset is needed or possible via the standard
	 * API. */
}

void SX127xRadio::resetAGC()
{
	/* hwResetAGC() is a no-op, so skip the base-class resetAGC() entirely.
	 * The base class calls startReceive() after hwResetAGC(), but the
	 * loramac-node modem mutex (STATE_BUSY during async RX) causes
	 * lora_recv_async() to return -EBUSY, setting _in_recv_mode = 0 and
	 * corrupting the state machine.  The loramac-node driver self-manages
	 * AGC, so nothing needs to happen here. */
}

} /* namespace mesh */
