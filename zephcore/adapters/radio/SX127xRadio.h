/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Radio adapter for SX127x (SX1272/SX1276/SX1278) using Zephyr loramac-node driver.
 *
 * The SX127x loramac-node driver supports the standard Zephyr LoRa API
 * (lora_config, lora_send_async, lora_recv_async) but has no extension
 * API for instantaneous RSSI, preamble detection, or RX boost.
 * Those features are stubbed out below.
 */

#pragma once

#include "LoRaRadioBase.h"

namespace mesh {

class SX127xRadio : public LoRaRadioBase {
public:
	SX127xRadio(const struct device *lora_dev, MainBoard &board,
		    NodePrefs *prefs = nullptr);

	void begin() override;

protected:
	/* Hardware primitives */
	void hwConfigure(const struct lora_modem_config &cfg) override;
	void hwCancelReceive() override;
	int hwSendAsync(uint8_t *buf, uint32_t len,
			struct k_poll_signal *sig) override;

	/* SX127x via loramac-node has no instantaneous RSSI API.
	 * Returns a fixed sentinel (-80 dBm) — noise floor calibration
	 * will converge on this value rather than the real noise floor. */
	int16_t hwGetCurrentRSSI() override;

	/* SX127x has no preamble-detected IRQ accessible via standard API.
	 * Always returns false — TX will not abort for a detected preamble. */
	bool hwIsPreambleDetected() override;

	/* SX127x has no RX boost / LNA gain switch via standard API. No-op. */
	void hwSetRxBoost(bool enable) override;

	/* SX127x loramac-node driver manages AGC automatically. No-op. */
	void hwResetAGC() override;

	/* Override public resetAGC(): hwResetAGC() is a no-op, and the
	 * loramac-node modem mutex makes the base-class startReceive() call
	 * fail with -EBUSY (modem STATE_BUSY during async RX), which would
	 * set _in_recv_mode = 0 and corrupt the state machine. */
	void resetAGC() override;

	/* SX127x has no BUSY pin. Default (false) from base is correct. */
	/* bool hwIsChipBusy() — inherited, returns false */
};

} /* namespace mesh */
