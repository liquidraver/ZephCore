/*
 * SPDX-License-Identifier: MIT
 * ZephCore MainBoard interface
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace mesh {

#define BD_STARTUP_NORMAL    0
#define BD_STARTUP_RX_PACKET 1

class MainBoard {
public:
	virtual uint16_t getBattMilliVolts() = 0;
	virtual uint8_t  getBattPercent() { return 0; }
	virtual float getMCUTemperature() { return NAN; }
	virtual bool setAdcMultiplier(float multiplier) { (void)multiplier; return false; }
	virtual float getAdcMultiplier() const { return 0.0f; }
	virtual const char *getManufacturerName() const = 0;
	virtual void onBeforeTransmit() {}
	virtual void onAfterTransmit() {}
	virtual void reboot() = 0;
	virtual void powerOff() {}
	virtual void sleep(uint32_t secs) { (void)secs; }
	virtual uint32_t getGpio() { return 0; }
	virtual void setGpio(uint32_t values) { (void)values; }
	virtual uint8_t getStartupReason() const = 0;
	virtual bool getBootloaderVersion(char *version, size_t max_len) { (void)version; (void)max_len; return false; }
	virtual bool startOTAUpdate(const char *id, char reply[]) { (void)id; (void)reply; return false; }

	virtual bool isExternalPowered() { return false; }
	virtual uint16_t getBootVoltage() { return 0; }
	virtual uint32_t getResetReason() const { return 0; }
	virtual const char *getResetReasonString(uint32_t reason) { (void)reason; return "Not available"; }
	virtual uint8_t getShutdownReason() const { return 0; }
	virtual const char *getShutdownReasonString(uint8_t reason) { (void)reason; return "Not available"; }
};

} /* namespace mesh */
