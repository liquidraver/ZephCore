/*
 * SPDX-License-Identifier: MIT
 * LR11xx Zephyr LoRa driver — extension API
 *
 * Functions extending the standard Zephyr lora_driver_api with
 * LR11xx-specific features (duty cycle, RX boost, RSSI readout,
 * preamble detection).
 */

#ifndef LR11XX_LORA_H
#define LR11XX_LORA_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get instantaneous RSSI (for noise floor calibration)
 *
 * Reads the current RSSI from the radio while in RX mode.
 * Protected by internal SPI mutex.
 *
 * @param dev LoRa device
 * @return RSSI in dBm, or -128 on error
 */
int16_t lr11xx_get_rssi_inst(const struct device *dev);

/* Read n RSSI samples spaced spacing_us apart, bracketing the duty-cycle
 * stand-down ONCE for the whole burst instead of once per sample.  Returns the
 * number of valid samples written; fewer than n means the read was refused
 * partway and the caller should discard the burst.  A NEGATIVE return (-EAGAIN)
 * means discard for a different reason: the receiver detected a preamble or
 * header inside the window, so the samples describe that signal rather than the
 * noise floor.  The two are worth distinguishing -- one indicts the sampler,
 * the other reports the channel. */
int lr11xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			 uint32_t spacing_us);

/**
 * @brief Check if radio is actively receiving a packet
 *
 * Checks IRQ status for preamble/sync word detection.
 * Uses non-blocking mutex — returns false if SPI is busy.
 *
 * @param dev LoRa device
 * @return true if preamble or sync word detected
 */
bool lr11xx_is_receiving(const struct device *dev);

/**
 * @brief Enable/disable RX boosted mode
 *
 * Boosted mode increases LNA gain for +3dB sensitivity at +2mA cost.
 *
 * @param dev LoRa device
 * @param enable true to enable boost
 */
void lr11xx_set_rx_boost(const struct device *dev, bool enable);

/**
 * @brief Radio deaf time per duty-cycle wake transition, in microseconds
 *
 * Context restore + PLL lock plus the DTS-configured TCXO startup delay
 * where fitted.  Used by the adapter (LoRaRadioBase) to size duty-cycle
 * windows so the wake transition is charged against the preamble-catch
 * budget — same accounting as the SX126x.
 *
 * @param dev LoRa device
 * @return Transition deaf time in microseconds
 */
uint32_t lr11xx_get_wakeup_time_us(const struct device *dev);

/**
 * @brief Duty-cycle re-arms caused by a timeout since boot or reset
 *
 * Counts the false-preamble case: UM §7.2.6 restarts the window timer with
 * 2*RxPeriod + SleepPeriod on preamble detection, and when that expires with no
 * packet the chip leaves the loop and the driver puts it back.  A climbing rate
 * means the RX window is catching noise rather than packets.  Backs
 * `get dc.restarts`, which reported a hardcoded 0 on this radio before anything
 * counted them.
 *
 * @param dev LoRa device
 * @return re-arm count
 */
uint32_t lr11xx_get_dc_timeout_restarts(const struct device *dev);

/**
 * @brief Clear the duty-cycle timeout re-arm counter
 *
 * @param dev LoRa device
 */
void lr11xx_reset_dc_timeout_restarts(const struct device *dev);

/**
 * @brief Get a random number from the radio
 *
 * Uses LR11xx hardware RNG.
 *
 * @param dev LoRa device
 * @return Random 32-bit value
 */
uint32_t lr11xx_get_random(const struct device *dev);

/**
 * @brief Redo the frequency-dependent calibrations (temperature drift path).
 *
 * Warm sleep, Calibrate(ALL) — which on this part already includes image
 * rejection — then image calibration at the operating frequency and a rx-boost
 * re-apply.  Deliberately NOT an AGC reset: that fault, and its remedy, belong
 * to the SX126x.  Defers if the chip is busy (duty-cycle sleep).
 *
 * Leaves the driver out of RX — the caller must startReceive() afterwards.
 *
 * @param dev LoRa device
 */
void lr11xx_recalibrate(const struct device *dev);

/**
 * @brief Junction temperature in whole degrees C, or INT16_MIN if unavailable.
 *
 * @param dev LoRa device
 */
int16_t lr11xx_get_chip_temp_c(const struct device *dev);

/**
 * @brief Set the adaptive-CAD operating detPeak offset
 *
 * Signed delta applied to the per-SF base cadDetPeak on every LBT CAD.
 * Takes effect on the next CAD — no reconfigure needed.  Clamped
 * in-driver to the LR11xx scale (48-90).
 *
 * @param dev    LoRa device
 * @param offset Signed offset from the base table value
 */
void lr11xx_cad_set_peak_offset(const struct device *dev, int8_t offset);

/**
 * @brief Per-SF base cadDetPeak for the currently configured SF
 *
 * @param dev LoRa device
 * @return Base detPeak for the current SF, bandwidth and CAD symbol count
 *         (roughly 50-85 on this family; strongly bandwidth-dependent)
 */
uint8_t lr11xx_cad_base_peak(const struct device *dev);

/**
 * @brief Lowest detPeak this driver will program.
 *
 * The C++ adaptive-CAD controller narrows its offset window to this range so it
 * never explores offsets that collapse onto one peak.
 *
 * @return Minimum absolute detPeak
 */
uint8_t lr11xx_cad_peak_min(void);

/**
 * @brief Highest detPeak this driver will program.
 *
 * @return Maximum absolute detPeak
 */
uint8_t lr11xx_cad_peak_max(void);

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
/* Armed with the CAD_RX exit mode: a NEGATIVE CAD exits to standby and the
 * caller must restart Rx, a POSITIVE one leaves the chip in Rx on the signal it
 * detected and the caller must not.
 * Returns 2 = detected, chip in Rx (see lr11xx_cad_rx_outcome());
 *         0 = channel free, chip in standby; <0 = error. */
int lr11xx_cad_probe(const struct device *dev, int8_t peak_offset);

/* Outcome of the Rx a positive lr11xx_cad_probe() entered.  The chip always
 * resolves it with a terminal interrupt, so this reports an observed event
 * rather than an inference.  Call once after the cad_timeout deadline; it
 * consumes the result.
 * Returns 1 = a packet arrived (detection was real);
 *         2 = cad_timeout expired with nothing decoded;
 *         0 = nothing armed, or the terminal interrupt has not arrived yet. */
int lr11xx_cad_rx_outcome(const struct device *dev);

/* The cad_timeout a CAD_RX probe programs, in milliseconds. */
uint32_t lr11xx_cad_rx_timeout_ms(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* LR11XX_LORA_H */
