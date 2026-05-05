/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Radio adapter for LR1110/LR1120/LR1121 using Zephyr LoRa driver
 *
 * Thin wrapper around LoRaRadioBase — only hardware-specific hooks.
 */

#pragma once

#include "LoRaRadioBase.h"

namespace mesh {

class LR1110Radio : public LoRaRadioBase {
public:
	LR1110Radio(const struct device *lora_dev, MainBoard &board,
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
