/*
 * SPDX-License-Identifier: MIT
 * LR11xx Zephyr LoRa driver — extension API used by the ZephCore adapter
 * (LR1110Radio). Everything here takes the driver's SPI mutex; calls that
 * touch the chip bracket a running duty cycle (see lr11xx_lora.c).
 */

#ifndef LR11XX_LORA_H
#define LR11XX_LORA_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Instantaneous RSSI in dBm, or -128 if the read was refused. */
int16_t lr11xx_get_rssi_inst(const struct device *dev);

/* n RSSI samples spacing_us apart, one duty-cycle stand-down for the burst.
 * Returns the samples written (< n: refused partway, discard), or -EAGAIN when
 * a preamble/header landed in the window (a busy channel, not a sampler fault). */
int lr11xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			  uint32_t spacing_us);

/* RX-busy gate: the DIO1-stamped header latch (bounded by the payload deadline)
 * and, without a duty cycle, the preamble grace. Never waits for the mutex. */
bool lr11xx_is_receiving(const struct device *dev);

/* RX boosted gain: more sensitivity for ~2 mA more current. */
void lr11xx_set_rx_boost(const struct device *dev, bool enable);

/* Deaf time per duty-cycle wake (context restore + PLL + TCXO start), in us. */
uint32_t lr11xx_get_wakeup_time_us(const struct device *dev);

/* Duty-cycle false-preamble re-arms since boot or reset (`get dc.restarts`). */
uint32_t lr11xx_get_dc_timeout_restarts(const struct device *dev);
void lr11xx_reset_dc_timeout_restarts(const struct device *dev);

/* Redo the frequency-dependent calibrations after temperature drift. Leaves the
 * driver out of RX: the caller must restart it. */
void lr11xx_recalibrate(const struct device *dev);

/* Adaptive CAD. The offset is added to the base detPeak on every LBT CAD from
 * the next one on, clamped to [lr11xx_cad_peak_min(), lr11xx_cad_peak_max()]. */
void lr11xx_cad_set_peak_offset(const struct device *dev, int8_t offset);
/* Base detPeak for the current SF, bandwidth and CAD symbol count. */
uint8_t lr11xx_cad_base_peak(const struct device *dev);
uint8_t lr11xx_cad_peak_min(void);
uint8_t lr11xx_cad_peak_max(void);

/* One blocking calibration CAD at base + peak_offset. Returns 0 = free (chip in
 * standby, the caller restarts Rx), 2 = detected (chip in Rx on the signal; the
 * caller must not restart it and reads lr11xx_cad_rx_outcome() later), <0 on
 * error. Mesh thread only. */
int lr11xx_cad_probe(const struct device *dev, int8_t peak_offset);

/* Outcome of the Rx a detecting probe entered, read once after its deadline:
 * 1 = a packet arrived, 2 = the Rx timed out with nothing decoded, 0 = nothing
 * armed or not resolved yet. Consumes the result. */
int lr11xx_cad_rx_outcome(const struct device *dev);

/* The Rx bound a detecting probe programs, in ms. */
uint32_t lr11xx_cad_rx_timeout_ms(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* LR11XX_LORA_H */
