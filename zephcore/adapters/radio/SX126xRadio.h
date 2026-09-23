/*
 * SPDX-License-Identifier: MIT
 * ZephCore Radio adapter for SX126x (SX1261/SX1262/SX1268) using native Zephyr driver
 */

#pragma once

#include "LoRaRadioBase.h"

namespace mesh {

class SX126xRadio : public LoRaRadioBase {
public:
	SX126xRadio(const struct device *lora_dev, MainBoard &board);

	void begin() override;

	/* Select the external FEM's LNA or its bypass path for RX
	 * (radio.fem.rxgain).  Delegates to the driver, which owns the
	 * receive-path select pin; returns false on a board that wires no such
	 * pin, i.e. one whose FEM gain is not software-selectable. */
	bool setFemRxEnable(bool enable) override;

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
	bool hwIsReceiving() override;
	void hwSetRxBoost(bool enable) override;
	bool hwNeedsAgcReset() override;
	void hwResetAgc() override;
	void hwRecalibrate() override;
	bool hwIsChipBusy() override;
	uint32_t hwWakeupTimeUs() override;
	int hwCadProbe(int8_t level) override;
	int hwCadRxOutcome() override;
	uint32_t hwCadRxTimeoutMs() override;
	void hwCadSetPeakOffset(int8_t offset) override;
	uint8_t hwCadBasePeak() override;
	uint8_t hwCadPeakMin() override;
	uint8_t hwCadPeakMax() override;
};

} /* namespace mesh */
