/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Radio adapter for LR2021 using Zephyr LoRa driver
 *
 * Thin wrapper around LoRaRadioBase — only hardware-specific hooks.
 */

#pragma once

#include "LoRaRadioBase.h"

namespace mesh {

class LR2021Radio : public LoRaRadioBase {
public:
	LR2021Radio(const struct device *lora_dev, MainBoard &board,
		    NodePrefs *prefs = nullptr);

	void begin() override;

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
};

} /* namespace mesh */
