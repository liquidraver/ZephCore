/*
 * SPDX-License-Identifier: MIT
 * LR2021 hardware hooks for LoRaRadioBase.
 */

#include "LR2021Radio.h"
#include <zephyr/kernel.h>

/* LR20xx driver extension API */
extern "C" {
#include <zephyr/drivers/lora/lr20xx_lora.h>
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr2021_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

LR2021Radio::LR2021Radio(const struct device *lora_dev, MainBoard &board)
	: LoRaRadioBase(lora_dev, board)
{
}

bool LR2021Radio::configSideDetectors(const uint8_t *sfs, uint8_t num)
{
	/* The driver validates against the configured SF/BW, so this must run
	 * after the modem is configured — which it is: the CLI path only
	 * reaches here on a live radio. */
	int ret = lr20xx_configure_side_detectors(_dev, sfs, num);

	if (ret < 0) {
		LOG_WRN("side detector config rejected: %d", ret);
		return false;
	}
	return true;
}

void LR2021Radio::resetStats()
{
	LoRaRadioBase::resetStats();
	lr20xx_reset_freq_offset(_dev);
}

uint32_t LR2021Radio::getDutyCycleTimeoutRestarts() const
{
	return lr20xx_get_dc_timeout_restarts(_dev);
}

void LR2021Radio::resetDutyCycleTimeoutRestarts()
{
	lr20xx_reset_dc_timeout_restarts(_dev);
}

int LR2021Radio::formatFreqErrorStatus(char *buf, int cap)
{
	struct lr20xx_freq_offset_stats st;

	if (lr20xx_get_freq_offset(_dev, &st) == 0) {
		return snprintf(buf, cap, "no packets measured yet");
	}

	/* Spread and count matter as much as the mean: the mean only
	 * approximates THIS node's reference error once it is averaged over
	 * many different peers, because their individual errors cancel and ours
	 * does not.  A tight spread over a handful of packets is one neighbour,
	 * not a population. */
	return snprintf(buf, cap,
			"mean %d Hz, min %d, max %d, %u pkts",
			st.mean_hz, st.min_hz, st.max_hz,
			(unsigned)st.count);
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

bool LR2021Radio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return false;
	}
	return true;
}

void LR2021Radio::hwCancelReceive()
{
	lora_recv_async(_dev, NULL, NULL);
}

int LR2021Radio::hwSendAsync(uint8_t *buf, uint32_t len,
			     struct k_poll_signal *sig)
{
	return lora_send_async(_dev, buf, len, sig);
}

int16_t LR2021Radio::hwGetCurrentRSSI()
{
	return lr20xx_get_rssi_inst(_dev);
}

int LR2021Radio::hwGetRssiBurst(int16_t *out, int n, uint32_t spacing_us)
{
	return lr20xx_get_rssi_burst(_dev, out, n, spacing_us);
}

/* Deliberately no hwIsChipBusy() override, matching the LR11xx.
 *
 * It backs LoRaRadioBase::isRadioReady(), which gates TX in startSendRaw() as
 * well as the probes.  Wiring BUSY into the transmit path stalled sends until
 * the dispatcher's 4 s CAD timeout, because the event-driven loop has nothing
 * to re-wake a deferred send.  The duty-cycle sleep window is handled where it
 * belongs instead: skipped in the two incidental pollers
 * (lr20xx_is_receiving, lr20xx_get_rssi_inst) and stood down deliberately for
 * TX inside the driver. */

bool LR2021Radio::hwIsReceiving()
{
	/* The poll itself is non-destructive: lr20xx_is_receiving() reads the
	 * IRQ register via get_status(), which clears nothing.
	 *
	 * Foreign-preamble release is NOT hardware-driven, whatever the comment
	 * that stood here (copy-pasted from LR1110Radio.cpp) claimed.  DS §5.7:
	 * IRQ status bits are latched until ClearIrq, and continuous RX has no
	 * timeout — so the driver releases them in software, on an SF-aware
	 * grace for the preamble and a max-airtime deadline for the header,
	 * exactly as the LR11xx and SX126x drivers do. */
	return lr20xx_is_receiving(_dev);
}

void LR2021Radio::hwSetRxBoost(bool enable)
{
	lr20xx_set_rx_boost(_dev, enable);
}

int LR2021Radio::hwCadProbe(int8_t level)
{
	return lr20xx_cad_probe(_dev, level);
}

/* Re-enables adaptive CAD on this family.  It was excluded in Batch B
 * (plan B-D2) because the chip's CAD_RX exit uses cad_timeout, 24 bits of
 * 32 MHz periods = 524 ms, against 1704 ms of max-length airtime -- a
 * CAD_RX there would truncate receptions.  The driver no longer uses that
 * exit: a probe runs CAD_ONLY and the DIO1 handler arms the follow-on Rx
 * with a normal SetRx, whose timeout is 24 bits of RTC steps = 512 s.  The
 * ceiling that justified the exclusion does not exist on that path, and the
 * same construction is what fixed the SX126x and LR11xx after both of their
 * chip CAD_RX exits were measured misbehaving on hardware.
 *
 * NOT hardware-verified: no LR2021 was available when this was written.
 * Verify on a MeshTracker X1 with the same test the LR11xx fix used --
 * reception count while probing (must stay ~100%) and tp becoming
 * non-zero -- before trusting it in the field. */
int LR2021Radio::hwCadRxOutcome()
{
	return lr20xx_cad_rx_outcome(_dev);
}

uint32_t LR2021Radio::hwCadRxTimeoutMs()
{
	return lr20xx_cad_rx_timeout_ms(_dev);
}

void LR2021Radio::hwCadSetPeakOffset(int8_t offset)
{
	lr20xx_cad_set_peak_offset(_dev, offset);
}

uint8_t LR2021Radio::hwCadBasePeak()
{
	return lr20xx_cad_base_peak(_dev);
}

/* The detPeak range lr20xx_do_cad() will actually program.  Must match the
 * driver's clamp exactly: if the adapter thinks the range is wider, the
 * staircase explores offsets that collapse onto one peak and reads the noise
 * between them as curvature — which is how it random-walked to -8 at SF7,
 * where base 51 puts everything from -3 down onto peak 48. */
uint8_t LR2021Radio::hwCadPeakMin()
{
	return lr20xx_cad_peak_min();
}

uint8_t LR2021Radio::hwCadPeakMax()
{
	return lr20xx_cad_peak_max();
}

/* No hwResetAgc() override, and no hwNeedsAgcReset(): this part has no
 * jammed-AGC fault to remedy — that one belongs to the SX126x.  The driver
 * exposes lr20xx_recalibrate() instead, which is the temperature-drift path
 * this family genuinely does need, named for what it actually does. */


void LR2021Radio::hwRecalibrate()
{
	/* Front-end calibration included: this is the temperature-drift path, and
	 * image/FE cal is the part a temperature swing actually invalidates. */
	lr20xx_recalibrate(_dev);
}

/* DS 6.4.2 gives an image/FE calibration temperature range (and advises redoing
 * PLL/AAF beyond +/-20 C), so drift recalibration is active here.  The reading
 * comes from the board — see LoRaRadioBase::imageCalMaintenance(). */
bool LR2021Radio::hwHasDriftRecal()
{
	return true;
}

uint32_t LR2021Radio::hwWakeupTimeUs()
{
	/* Per-device, because the TCXO term dominates and is board-specific:
	 * meshtracker_x1 declares tcxo-startup-delay-ms = <5>, promicro_lr2021
	 * is XTAL-only.  Inheriting the base class's flat 1500 us under-counted
	 * the X1's real deaf time by ~4.6 ms, which oversized the duty-cycle
	 * sleep and dropped window-edge preambles regardless of signal strength.
	 * Same shape as the SX126x and LR11xx overrides. */
	return lr20xx_get_wakeup_time_us(_dev);
}

} /* namespace mesh */
