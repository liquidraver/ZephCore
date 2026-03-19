/*
 * SPDX-License-Identifier: Apache-2.0
 * Universal QSPI flash probe and erase — bare-metal nRF52840 QSPI register access
 *
 * Probes all known pin configurations to detect external QSPI flash,
 * then performs a full chip erase if found. No Zephyr QSPI driver needed.
 */

#ifndef QSPI_PROBE_H
#define QSPI_PROBE_H

/**
 * Probe all known QSPI pin configurations for connected flash.
 * If found, erase the entire chip.
 *
 * @return 0 if a chip was found and erased, -1 if no chip found or not supported
 */
int qspi_probe_and_erase(void);

#endif /* QSPI_PROBE_H */
