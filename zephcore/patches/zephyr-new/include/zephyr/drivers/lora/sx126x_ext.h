/*
 * SPDX-License-Identifier: MIT
 * SX126x native driver — extension API
 *
 * Functions extending the standard Zephyr lora_driver_api with
 * SX126x-specific features (duty cycle, RX boost, external FEM gating,
 * RSSI readout, preamble detection).
 */

#ifndef SX126X_EXT_H
#define SX126X_EXT_H

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
 * Uses non-blocking mutex — returns -128 if SPI is busy.
 *
 * @param dev LoRa device
 * @return RSSI in dBm, or -128 on error
 */
int16_t sx126x_get_rssi_inst(const struct device *dev);

/**
 * @brief Check if radio is actively receiving a packet
 *
 * Checks IRQ status for preamble/header detection.
 * Uses non-blocking mutex — returns false if SPI is busy.
 *
 * @param dev LoRa device
 * @return true if preamble or header detected
 */
bool sx126x_is_receiving(const struct device *dev);

/**
 * @brief Enable/disable RX boosted mode
 *
 * Boosted mode increases LNA gain for +3dB sensitivity at +2mA cost.
 *
 * @param dev LoRa device
 * @param enable true to enable boost
 */
void sx126x_set_rx_boost(const struct device *dev, bool enable);

/**
 * @brief Select an external FEM's LNA or its bypass path for RX
 *
 * Acts on lna-bypass-gpios, the FEM's receive-path select -- KCT8103L CTX on
 * the Heltec boards, where DIO2 into the FEM's CPS pin does the TX/RX
 * switching and leaves this line meaning nothing but "LNA or bypass".
 * Passing false routes RX around the LNA: its gain (~17 dB measured) and its
 * supply current both drop out, but the antenna stays connected to the
 * receiver.  TX and the driver's idle/sleep gating are unaffected.
 *
 * This is deliberately NOT antenna-enable-gpios.  That line is the FEM's chip
 * enable, owned by the RX/TX/sleep state machine; deasserting it during RX
 * shuts the part down, and a shut-down FEM passes nothing -- the node goes
 * deaf by tens of dB rather than losing the LNA's share.  ZephCore 1.17.2
 * shipped that mistake; see the KCT8103L handling in MeshCore's
 * variants/heltec_v4/LoRaFEMControl.cpp for the reference behaviour.
 *
 * Default is enabled (LNA in the path).
 *
 * @param dev    LoRa device
 * @param enable true for the LNA path, false for the bypass path
 * @return true if this board wires a receive-path select, false if it has
 *         none (in which case the call did nothing)
 */
bool sx126x_set_fem_rx_enable(const struct device *dev, bool enable);

/**
 * @brief Check if the radio chip is busy (cannot accept SPI commands)
 *
 * Reads the BUSY GPIO pin directly — no SPI, no blocking.
 * Returns true when the chip is in its duty-cycle sleep phase.
 * Safe to call at any time.
 *
 * @param dev LoRa device
 * @return true if BUSY pin is high (chip sleeping / processing)
 */
bool sx126x_is_chip_busy(const struct device *dev);

/**
 * @brief Apply undocumented register 0x8B5 RX improvement for Heltec V4
 *
 * Sets the LSB of register 0x8B5 which consistently improves RX reception
 * on boards with GC1109 or KCT8103L PA (Heltec V4/V4.3). Described by
 * Heltec engineer @Quency-D in MeshCore PR#1398.
 *
 * Must be called after the first lora_config() completes.
 *
 * @param dev LoRa device
 */
void sx126x_apply_heltec_reg_patch(const struct device *dev);

/**
 * @brief Reset AGC by performing warm sleep + full recalibration
 *
 * Warm sleep powers down the analog frontend (resets AGC gain state),
 * then Calibrate(0x7F) refreshes all blocks (ADC, PLL, image, oscillators).
 * Re-applies DIO2 RF switch, RX boosted gain, and image calibration afterward.
 *
 * Must be called while NOT actively receiving a packet.
 *
 * @param dev LoRa device
 */
void sx126x_reset_agc(const struct device *dev);

/**
 * @brief Radio deaf time per duty-cycle wake transition, in microseconds
 *
 * Context save/restore + PLL lock (~1.5 ms) plus the DTS-configured TCXO
 * startup delay where fitted.  Per datasheet rev 2.2 §13.1.7, the TCXO
 * delay is inserted between the sleep and RX periods of every duty-cycle
 * wake — it counts against the preamble-catch budget, outside both
 * programmed periods.  Used by the adapter to size duty-cycle windows.
 *
 * @param dev LoRa device
 * @return Transition deaf time in microseconds
 */
uint32_t sx126x_get_wakeup_time_us(const struct device *dev);

/**
 * @brief Get duty-cycle preamble false-positive counter
 *
 * Returns the number of times duty-cycle RX tripped IRQ_RX_TX_TIMEOUT
 * and was silently re-armed. High values mean the preamble detector is
 * firing on noise/neighbour interference without a real packet arriving,
 * which inflates RX-on time beyond the nominal duty cycle and shortens
 * battery life.
 *
 * @param dev LoRa device
 * @return Cumulative re-arm count since last reset
 */
uint32_t sx126x_get_dc_timeout_restarts(const struct device *dev);

/**
 * @brief Reset the duty-cycle preamble false-positive counter to zero.
 *
 * @param dev LoRa device
 */
void sx126x_reset_dc_timeout_restarts(const struct device *dev);

/**
 * @brief Set the adaptive-CAD operating detPeak offset
 *
 * Signed delta applied to the per-SF base cadDetPeak on every LBT CAD.
 * Takes effect on the next CAD — no reconfigure needed.  The absolute
 * value is clamped in-driver to a sane window for the chip family.
 *
 * @param dev    LoRa device
 * @param offset Signed offset from the base table value
 */
void sx126x_cad_set_peak_offset(const struct device *dev, int8_t offset);

/**
 * @brief Base cadDetPeak for the current SF, bandwidth and CAD symbol count
 *
 * @param dev LoRa device
 * @return Base detPeak (roughly 18-34 on this family)
 */
uint8_t sx126x_cad_base_peak(const struct device *dev);

/**
 * @brief Lowest detPeak this driver will program.
 *
 * The C++ adaptive-CAD controller narrows its offset window to this range so it
 * never explores offsets that collapse onto one peak.
 *
 * @return Minimum absolute detPeak
 */
uint8_t sx126x_cad_peak_min(void);

/**
 * @brief Highest detPeak this driver will program.
 *
 * @return Maximum absolute detPeak
 */
uint8_t sx126x_cad_peak_max(void);

/**
 * @brief Run one blocking calibration CAD at base detPeak + peak_offset
 *
 * Uses the operating modem config (SF/BW/symbol count).  Armed with the
 * CAD_RX exit mode, so the two verdicts leave the chip in different places:
 * a NEGATIVE CAD exits to standby and the caller must restart RX, while a
 * POSITIVE one leaves the chip in Rx on the signal it detected and the caller
 * must NOT.  The return value distinguishes them.  Must be called from the
 * mesh loop thread only (same thread as the LBT CAD).
 *
 * @param dev         LoRa device
 * @param peak_offset Signed offset from the base table value
 * @return 2 = activity detected, chip left in Rx (see sx126x_cad_rx_outcome())
 *         0 = channel free, chip in standby, caller restarts RX
 *         <0 = error
 */
int sx126x_cad_probe(const struct device *dev, int8_t peak_offset);

/**
 * @brief Outcome of the Rx a positive sx126x_cad_probe() entered
 *
 * The chip always resolves that Rx with a terminal interrupt -- a packet, or
 * RX_TX_TIMEOUT at cadTimeout -- so this reports an observed event rather than
 * an inference, and never has to be polled in a loop.  Call it once after the
 * cadTimeout deadline has passed; it consumes the result.
 *
 * @param dev LoRa device
 * @return 1 = a packet completed (RX_DONE or CRC_ERR): the detection was real
 *         2 = cadTimeout expired with nothing decoded: no packet followed
 *         0 = nothing armed, or the terminal interrupt has not arrived yet
 */
int sx126x_cad_rx_outcome(const struct device *dev);

/**
 * @brief The cadTimeout a CAD_RX probe programs, in milliseconds
 *
 * Max-length-packet airtime at the current SF/BW.  Exposed so a caller waiting
 * to read sx126x_cad_rx_outcome() uses the chip's own deadline rather than a
 * guess of its own.
 */
uint32_t sx126x_cad_rx_timeout_ms(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* SX126X_EXT_H */
