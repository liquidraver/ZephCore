/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Radio adapter for SX126x (SX1261/SX1262/SX1268) using native Zephyr driver
 */

#pragma once

#include "LoRaRadioBase.h"

namespace mesh {

class SX126xRadio : public LoRaRadioBase {
public:
	SX126xRadio(const struct device *lora_dev, MainBoard &board,
		    NodePrefs *prefs = nullptr);

	void begin() override;

	/* Duty-cycle preamble false-positive stats (SX126x-specific) */
	uint32_t getDutyCycleTimeoutRestarts() const override;
	void resetDutyCycleTimeoutRestarts() override;

protected:
	/* Hardware primitives */
	bool hwConfigure(const struct lora_modem_config &cfg) override;
	void hwCancelReceive() override;
	int hwSendAsync(uint8_t *buf, uint32_t len,
			struct k_poll_signal *sig) override;
	int16_t hwGetCurrentRSSI() override;
	bool hwIsPreambleDetected() override;
	void hwSetRxBoost(bool enable) override;
	void hwResetAGC() override;
	bool hwIsChipBusy() override;
};

} /* namespace mesh */
