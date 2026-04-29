/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr MainBoard base - nRF52840 DK and generic
 */

#pragma once

#include <mesh/Board.h>

namespace mesh {

class ZephyrBoard : public MainBoard {
public:
	uint16_t getBattMilliVolts() override;
	float getMCUTemperature() override;
	bool setAdcMultiplier(float multiplier) override;
	float getAdcMultiplier() const override;
	const char *getManufacturerName() const override;
	void onBeforeTransmit() override;
	void onAfterTransmit() override;
	void reboot() override;
	void rebootToBootloader();        /* Reboot into UF2 mass storage bootloader */
	bool getBootloaderVersion(char *version, size_t max_len) override;
	bool startOTAUpdate(const char *id, char reply[]) override;  /* Reboot into BLE OTA DFU */
	void clearBootloaderMagic();      /* Clear stale GPREGRET values at startup */
	uint8_t getStartupReason() const override;

private:
	/* Runtime override for vbat-mv-multiplier. Units match DT `vbat-mv-multiplier`
	 * (mV scale such that `mv = multiplier * raw / 4096`). 0 = use DT default. */
	float _adc_multiplier_override = 0.0f;
};

} /* namespace mesh */
