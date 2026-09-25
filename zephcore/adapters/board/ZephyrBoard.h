/*
 * SPDX-License-Identifier: MIT
 * Zephyr MainBoard base - nRF52840 DK and generic
 */

#pragma once

#include <mesh/MeshCore.h>

namespace mesh {

class ZephyrBoard : public MainBoard {
public:
	/* One ADC burst (or fuel-gauge read) serves every caller for
	 * BATT_CACHE_MS: UI, telemetry, stats, alerts. Any thread. */
	uint16_t getBattMilliVolts() override;
	/* The board's discharge curve (or the fuel gauge's state of charge);
	 * what both UIs display. Not a MainBoard method: nothing
	 * upstream-shaped needs it. */
	uint8_t  getBattPercent();
	float getMCUTemperature() override;
	bool setAdcMultiplier(float multiplier) override;
	float getAdcMultiplier() const override;
	const char *getManufacturerName() const override;
	void onBeforeTransmit() override;
	void onAfterTransmit() override;
	void onPacketReceived() override;
	void reboot() override;
	void powerOff() override;         /* `poweroff` / `shutdown`: a user request */
	void rebootToBootloader();        /* Reboot into UF2 mass storage bootloader */
	bool getBootloaderVersion(char *version, size_t max_len) override;
	bool startOTAUpdate(const char *id, char reply[]) override;  /* Reboot into BLE OTA DFU */
	void clearBootloaderMagic();      /* Clear stale GPREGRET values at startup */
	uint8_t getStartupReason() const override;
	bool isExternalPowered() override;  /* nRF52: VBUS present (USB/charger); else false */

	/* get pwrmgt.*: upstream's power-management getters. */
	uint16_t getBootVoltage() override { return _boot_mv; }
	uint32_t getResetReason() const override;
	const char *getResetReasonString(uint32_t reason) override;
	uint8_t getShutdownReason() const override;
	const char *getShutdownReasonString(uint8_t reason) override;
	/* Boot, once the ADC is up: the battery voltage for getBootVoltage(). */
	void captureBootVoltage() { _boot_mv = getBattMilliVolts(); }

private:
	uint16_t readBattMilliVolts();
	uint16_t _boot_mv = 0;
	uint16_t _batt_mv = 0;
	int64_t _batt_read_ms = -1;   /* uptime of _batt_mv; -1 = none yet */
	/* Runtime override for vbat-mv-multiplier. Units match DT `vbat-mv-multiplier`
	 * (mV scale such that `mv = multiplier * raw / 4096`). 0 = use DT default. */
	float _adc_multiplier_override = 0.0f;
};

} /* namespace mesh */
