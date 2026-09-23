/*
 * SPDX-License-Identifier: MIT
 * LR20xx Zephyr LoRa driver — extension API
 *
 * Functions extending the standard Zephyr lora_driver_api with
 * LR20xx-specific features (RX boost, RSSI readout, preamble detection,
 * duty cycle, AGC reset).
 */

#ifndef LR20XX_LORA_H
#define LR20XX_LORA_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get instantaneous RSSI (for noise floor calibration)
 *
 * @param dev LoRa device
 * @return RSSI in dBm, or -128 on error
 */
int16_t lr20xx_get_rssi_inst(const struct device *dev);

/* Read n RSSI samples spaced spacing_us apart, bracketing the duty-cycle
 * stand-down ONCE for the whole burst instead of once per sample.  Returns the
 * number of valid samples written; fewer than n means the read was refused
 * partway and the caller should discard the burst.  A NEGATIVE return (-EAGAIN)
 * means discard for a different reason: the receiver detected a preamble or
 * header inside the window, so the samples describe that signal rather than the
 * noise floor.  The two are worth distinguishing -- one indicts the sampler,
 * the other reports the channel. */
int lr20xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			 uint32_t spacing_us);

/**
 * @brief Running carrier-frequency-error statistics
 *
 * Accumulated from every successfully received packet.  A single packet's
 * offset is the sum of this node's reference error and the transmitter's, so
 * only @c mean_hz over many packets from many peers says anything about our own
 * XTAL/TCXO; @c min_hz / @c max_hz show the spread that mean is drawn from.
 */
struct lr20xx_freq_offset_stats {
	int32_t last_hz;   /**< Most recent packet's offset */
	int32_t mean_hz;   /**< Arithmetic mean over @c count packets */
	int32_t min_hz;    /**< Most negative seen */
	int32_t max_hz;    /**< Most positive seen */
	uint32_t count;    /**< Packets accumulated (0 = nothing measured) */
};

/**
 * @brief Read the carrier frequency error accumulated since boot or reset
 *
 * Driver v2.0.2 decodes this from three bytes GetLoraPacketStatus grew beyond
 * what DS rev 2.1 Table 9-13 documents, so the driver is ahead of the datasheet
 * here — sanity-check against a known-good transmitter before trusting it.
 * Readings beyond +/-200 kHz are discarded as implausible for a packet that
 * demodulated at all, and warn once.
 *
 * @param dev LoRa device
 * @param out Filled in on return; must not be NULL
 * @return number of packets accumulated (0 = nothing measured yet)
 */
uint32_t lr20xx_get_freq_offset(const struct device *dev,
				struct lr20xx_freq_offset_stats *out);

/**
 * @brief Clear the accumulated frequency-error statistics
 *
 * @param dev LoRa device
 */
void lr20xx_reset_freq_offset(const struct device *dev);

/**
 * @brief Duty-cycle re-arms caused by a timeout since boot or reset
 *
 * Counts the false-preamble case: the preamble-restarted rx_max_time +
 * cycle_time window expired with no packet, the chip left the duty-cycle loop,
 * and the driver put it back.  A climbing rate means the RX window is catching
 * noise rather than packets, which is the signal for retuning it.  Backs
 * `get dc.restarts`, which reported a hardcoded 0 on this radio before anything
 * counted them.
 *
 * @param dev LoRa device
 * @return re-arm count
 */
uint32_t lr20xx_get_dc_timeout_restarts(const struct device *dev);

/**
 * @brief Clear the duty-cycle timeout re-arm counter
 *
 * @param dev LoRa device
 */
void lr20xx_reset_dc_timeout_restarts(const struct device *dev);

/**
 * @brief Radio deaf time for one duty-cycle wake transition, in microseconds
 *
 * DS Table 3-23: warm start (Sleep with retention -> STDBY_RC) 1 ms, plus
 * STDBY_RC -> Rx 115 us, plus the TCXO restart where one is fitted — duty-cycle
 * sleep powers the VTCXO regulator down, so the oscillator restarts on every
 * wake.  Counts against the adapter's preamble-catch budget.
 *
 * @param dev LoRa device
 * @return deaf time in microseconds
 */
uint32_t lr20xx_get_wakeup_time_us(const struct device *dev);

/**
 * @brief Check if radio is actively receiving a packet
 *
 * Checks IRQ status for preamble/sync word detection.
 * Uses non-blocking mutex — returns false if SPI is busy.
 *
 * @param dev LoRa device
 * @return true if preamble or sync word detected
 */
bool lr20xx_is_receiving(const struct device *dev);

/**
 * @brief Enable/disable RX boosted mode
 *
 * @param dev LoRa device
 * @param enable true to enable boost
 */
void lr20xx_set_rx_boost(const struct device *dev, bool enable);

/**
 * @brief GPIO-only check for "chip is in a duty-cycle sleep window" (no SPI)
 *
 * True only while an RX duty cycle is armed and BUSY is high — the chip is in
 * its own sleep window (DS Table 4-1: "Sleep: busy=1") and an NSS falling edge
 * would end the cycle (DS §6.3.8).
 *
 * Deliberately gated on the duty cycle rather than reading raw BUSY, but NOT
 * for the reason once given here: BUSY is *not* held high during continuous
 * RX.  DS §5.2 is explicit — "In Rx mode, BUSY goes low as soon as the chip is
 * ready to receive data."  (The BUSY=1 readings the X1 bring-up logs showed
 * came from sampling the pin after issuing SPI, i.e. the dump's own footprint;
 * dump_chip_state() now samples before.)  The gate is still right: without it
 * a stray BUSY would gate TX as well as the probes and mute the node.
 *
 * @param dev LoRa device
 * @return true if the host should not issue commands right now
 */
bool lr20xx_is_chip_busy(const struct device *dev);

/**
 * @brief Configure LoRa side detectors (multi-SF receive)
 *
 * Programs up to 3 extra spreading factors to be demodulated concurrently
 * with the configured one, on the same bandwidth.  Applies immediately and
 * is re-applied on every RX entry (the chip drops the set on each
 * SetModulationParams, and LBT CAD has to switch it off — the datasheet's
 * SF constraints for CAD are the inverse of those for RX).
 *
 * Datasheet constraints, enforced here: every side SF must be greater than
 * the configured SF, all SFs distinct, highest - lowest <= 4, and at
 * BW >= 500 kHz at most 2 side detectors (1 when the main SF is >= 10).
 *
 * @param dev LoRa device
 * @param sfs Array of side spreading factors (5..12)
 * @param num Number of side detectors, 0..3 (0 disables the feature)
 * @return 0 on success, -EINVAL if the set violates a chip constraint
 */
int lr20xx_configure_side_detectors(const struct device *dev,
				    const uint8_t *sfs, uint8_t num);

/**
 * @brief Get a random number from the radio hardware RNG
 *
 * @param dev LoRa device
 * @return Random 32-bit value
 */
uint32_t lr20xx_get_random(const struct device *dev);

/**
 * @brief Redo the frequency-dependent calibrations (temperature drift path).
 *
 * Warm sleep, Calibrate, rx-boost re-apply, then CalibFE at the operating
 * frequency — the front end being what a temperature swing actually
 * invalidates.  Deliberately not an AGC reset: that fault, and its remedy,
 * belong to the SX126x.  Defers if the chip is busy (duty-cycle sleep).
 *
 * Leaves the driver out of RX — the caller must startReceive() afterwards.
 *
 * @param dev LoRa device
 */
void lr20xx_recalibrate(const struct device *dev);

/**
 * @brief Junction temperature in whole degrees C, or INT16_MIN if unavailable.
 *
 * @param dev LoRa device
 */
int16_t lr20xx_get_chip_temp_c(const struct device *dev);

/**
 * @brief Set the adaptive-CAD operating detPeak offset
 *
 * Signed delta applied to the per-SF base cadDetPeak on every LBT CAD.
 * Takes effect on the next CAD — no reconfigure needed.  Clamped
 * in-driver to the LR20xx scale (48-90).
 *
 * @param dev    LoRa device
 * @param offset Signed offset from the base table value
 */
void lr20xx_cad_set_peak_offset(const struct device *dev, int8_t offset);

/**
 * @brief Per-SF base cadDetPeak for the currently configured SF
 *
 * @param dev LoRa device
 * @return Base detPeak (56-68 on this family)
 */
uint8_t lr20xx_cad_base_peak(const struct device *dev);

/**
 * @brief Absolute detPeak range this driver will program, inclusive
 *
 * The adaptive-CAD controller needs these to keep its offset window inside the
 * range where every level is a distinct configuration.  Outside it the clamp
 * folds several offsets onto one peak, and the staircase compares rungs that
 * are physically identical.
 *
 * @return lowest / highest detPeak the driver will ever send
 */
uint8_t lr20xx_cad_peak_min(void);
uint8_t lr20xx_cad_peak_max(void);

/**
 * @brief Run one blocking calibration CAD at base detPeak + peak_offset
 *
 * Uses the operating modem config (SF/BW/symbol count).  Leaves the chip
 * in STANDBY — the caller must restart RX afterwards.  Mesh loop thread
 * only.
 *
 * @param dev         LoRa device
 * @param peak_offset Signed offset from the base table value
 * @return 1 = activity detected, 0 = channel free, <0 = error
 */
int lr20xx_cad_probe(const struct device *dev, int8_t peak_offset);
uint32_t lr20xx_cad_rx_timeout_ms(const struct device *dev);
int lr20xx_cad_rx_outcome(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* LR20XX_LORA_H */
