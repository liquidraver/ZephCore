/*
 * SPDX-License-Identifier: Apache-2.0
 * SX126x native driver — extension API
 *
 * Functions extending the standard Zephyr lora_driver_api with
 * SX126x-specific features (duty cycle, RX boost, RSSI readout,
 * preamble detection).
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
 * @brief Enable/disable RX duty cycle mode
 *
 * When enabled, the radio alternates between RX and sleep using the
 * RadioLib preamble detection algorithm. Saves ~60-70% RX current.
 *
 * @param dev LoRa device
 * @param enable true to enable, false for continuous RX
 */
void sx126x_set_rx_duty_cycle(const struct device *dev, bool enable);

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

#ifdef __cplusplus
}
#endif

#endif /* SX126X_EXT_H */
