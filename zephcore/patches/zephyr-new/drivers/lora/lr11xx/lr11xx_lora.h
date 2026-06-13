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
 * @brief Get a random number from the radio
 *
 * Uses LR11xx hardware RNG.
 *
 * @param dev LoRa device
 * @return Random 32-bit value
 */
uint32_t lr11xx_get_random(const struct device *dev);

/**
 * @brief Reset AGC by performing warm sleep + full recalibration
 *
 * Warm sleep powers down the analog frontend (resets AGC gain state),
 * then Calibrate(0x3F) refreshes all blocks. Re-applies image
 * calibration for the operating frequency and RX boost afterward.
 *
 * Must be called while NOT actively receiving a packet.
 *
 * @param dev LoRa device
 */
void lr11xx_reset_agc(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* LR11XX_LORA_H */
