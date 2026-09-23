/*
 * SPDX-License-Identifier: MIT
 * SX126x hardware hooks for LoRaRadioBase — native Zephyr driver.
 */

#include "SX126xRadio.h"
#include <zephyr/kernel.h>

/* Native SX126x driver extension API */
extern "C" {
#include <zephyr/drivers/lora/sx126x_ext.h>
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sx126x_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

SX126xRadio::SX126xRadio(const struct device *lora_dev, MainBoard &board)
	: LoRaRadioBase(lora_dev, board)
{
}

void SX126xRadio::begin()
{
	LoRaRadioBase::begin();

#if IS_ENABLED(CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH)
	/* Apply undocumented register 0x8B5 RX improvement (MeshCore PR#1398).
	 * Must run after lora_config() has been called (via startReceive above). */
	sx126x_apply_heltec_reg_patch(_dev);
	LOG_INF("Applied Heltec reg 0x8B5 RX patch");
#endif
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

bool SX126xRadio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return false;
	}
	return true;
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

bool SX126xRadio::hwIsReceiving()
{
	/* rx_packet_active latch + IRQ read. The driver clears sticky bits only
	 * to release an expired preamble grace or payload deadline. */
	return sx126x_is_receiving(_dev);
}

/* The one family that actually has the jammed-AGC fault — see the patch preamble
 * in patches/zephyr/0003-lora-sx126x-native.patch, where this remedy and its
 * trigger are argued out.  The LR parts deliberately do not declare it. */
bool SX126xRadio::hwNeedsAgcReset()
{
	return true;
}

void SX126xRadio::hwResetAgc()
{
	/* Leaves the chip in STANDBY and the driver state at IDLE, so the
	 * caller's startReceive() performs a real re-entry. */
	sx126x_reset_agc(_dev);
}

void SX126xRadio::hwRecalibrate()
{
	/* Calibrate(ALL) on this part already includes image rejection and
	 * sx126x_reset_agc() re-issues CalibrateImage for the operating
	 * frequency, so the drift path needs nothing extra. */
	sx126x_reset_agc(_dev);
}

/* No hwHasDriftRecal() override: unlike the LR parts, the SX126x datasheet
 * gives no temperature threshold for image calibration, so drift-triggered
 * recalibration stays inactive on this family — unchanged from before, when the
 * same result came about because the part exposes no junction-temperature
 * readout for the trigger to read.  (The trigger now reads board temperature,
 * which every board has, so the family gate has to be explicit.) */

void SX126xRadio::hwSetRxBoost(bool enable)
{
	sx126x_set_rx_boost(_dev, enable);
}

bool SX126xRadio::setFemRxEnable(bool enable)
{
	/* Pure driver-side -- no SPI, no chip state.  The driver applies it to
	 * lna-bypass-gpios straight away when the radio is parked in RX, and
	 * otherwise at the next RX/TX/sleep transition.  Returns false when this
	 * board wires no receive-path select, so the CLI reports "unsupported"
	 * rather than acknowledging a setting that cannot do anything -- note the
	 * gate is that line, not antenna-enable-gpios, so a board with a FEM whose
	 * only control is the chip enable (heltec_wifi_lora32_v4) lands here too. */
	if (!sx126x_set_fem_rx_enable(_dev, enable)) {
		return false;
	}
	LOG_INF("FEM RX gain %s", enable ? "enabled" : "disabled");
	return true;
}

bool SX126xRadio::hwIsChipBusy()
{
	return sx126x_is_chip_busy(_dev);
}

uint32_t SX126xRadio::hwWakeupTimeUs()
{
	return sx126x_get_wakeup_time_us(_dev);
}

int SX126xRadio::hwCadProbe(int8_t level)
{
	return sx126x_cad_probe(_dev, level);
}

int SX126xRadio::hwCadRxOutcome()
{
	return sx126x_cad_rx_outcome(_dev);
}

uint32_t SX126xRadio::hwCadRxTimeoutMs()
{
	return sx126x_cad_rx_timeout_ms(_dev);
}

void SX126xRadio::hwCadSetPeakOffset(int8_t offset)
{
	sx126x_cad_set_peak_offset(_dev, offset);
}

uint8_t SX126xRadio::hwCadBasePeak()
{
	return sx126x_cad_base_peak(_dev);
}

/* The detPeak range sx126x_do_cad() will actually program.  Must match the
 * driver's clamp exactly: if the adapter thinks the range is wider, the
 * staircase explores offsets that collapse onto one peak and reads the noise
 * between them as curvature.  Same reasoning as LR2021Radio::hwCadPeakMin. */
uint8_t SX126xRadio::hwCadPeakMin()
{
	return sx126x_cad_peak_min();
}

uint8_t SX126xRadio::hwCadPeakMax()
{
	return sx126x_cad_peak_max();
}

uint32_t SX126xRadio::getDutyCycleTimeoutRestarts() const
{
	return sx126x_get_dc_timeout_restarts(_dev);
}

void SX126xRadio::resetDutyCycleTimeoutRestarts()
{
	sx126x_reset_dc_timeout_restarts(_dev);
}

} /* namespace mesh */
