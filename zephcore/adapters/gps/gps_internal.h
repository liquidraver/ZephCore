/*
 * SPDX-License-Identifier: MIT
 *
 * The GPS manager's internals, shared by its three files and nothing else:
 *   ZephyrGPSManager.cpp  state machine, fix validation, init, public API
 *   gps_power.cpp         module power: GPIO/regulator, UART sleep, UART PM
 *   gps_module_cfg.cpp    module configuration (GNSS API or PMTK/PCAS/UBX),
 *                         its diagnostics report
 * Main thread only, except where a function says otherwise.
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#if defined(CONFIG_SOC_NRF52840)
#include <nrfx.h>
#endif

/* ========== GNSS Support ========== */
#if DT_HAS_COMPAT_STATUS_OKAY(gnss_nmea_generic) || \
	DT_HAS_COMPAT_STATUS_OKAY(quectel_lc76g) || \
	DT_HAS_COMPAT_STATUS_OKAY(luatos_air530z)
#define HAS_GNSS 1
#include <zephyr/drivers/gnss.h>
#else
#define HAS_GNSS 0
#endif

/* ========== GPS Feature Detection ==========
 * Every HAS_GPS_* predicate is defined HERE, before first use. They are pure
 * devicetree tests with no side effects, kept apart from the variables they
 * gate so that ordering can never drift again.
 *
 * Why this block exists: HAS_GPS_UART used to be defined ~500 lines below its
 * first `#if`, and an undefined identifier in `#if` is silently 0 — so the
 * entire PMTK/UBX module-configuration path compiled to nothing on every
 * board, and GPS ran at module defaults (GPS-only constellations, no AOP).
 * If you add another HAS_GPS_* macro, define it in this block.
 */

/* GNSS module hangs off a UART we can write to (any compatible). */
#if HAS_GNSS && DT_NODE_HAS_STATUS(DT_NODELABEL(gnss), okay) && \
	DT_NODE_HAS_STATUS(DT_BUS(DT_NODELABEL(gnss)), okay)
#define HAS_GPS_UART 1
#else
#define HAS_GPS_UART 0
#endif

/* The nRF register diagnostics read the GNSS module's own UARTE, which is not
 * always UARTE0 (the RAK4631's GNSS is on uart1). */
#if HAS_GPS_UART && defined(CONFIG_SOC_NRF52840) && \
	DT_NODE_HAS_COMPAT(DT_BUS(DT_NODELABEL(gnss)), nordic_nrf_uarte)
#define GPS_NRF_UARTE ((NRF_UARTE_Type *)DT_REG_ADDR(DT_BUS(DT_NODELABEL(gnss))))
#endif

/* Discrete GPS power-enable GPIO (gps-enable alias). */
#if DT_NODE_EXISTS(DT_ALIAS(gps_enable))
#define HAS_GPS_POWER_CONTROL 1
#else
#define HAS_GPS_POWER_CONTROL 0
#endif

/* GPS powered from a PMU regulator rail (chosen zephcore,gps-power). */
#if DT_NODE_EXISTS(DT_CHOSEN(zephcore_gps_power))
#define HAS_GPS_POWER_REGULATOR 1
#else
#define HAS_GPS_POWER_REGULATOR 0
#endif

#if HAS_GNSS

/* ---- ZephyrGPSManager.cpp ---- */
extern const struct device *gnss_dev;
#ifdef CONFIG_ZEPHCORE_GPS_SAT_DIAG
/* Satellites tracked per constellation (GPS, GLONASS, Galileo, BeiDou,
 * other), 0 for one that has stopped reporting. Any thread. */
void gps_sat_tally(uint8_t out[5]);
#endif

/* ---- gps_power.cpp ---- */
/* keep_vrtc (off only): T1000-E warm standby. */
void gps_power_control(bool on, bool keep_vrtc = false);
/* The power line or rail if the board has one, else the UART sleep commands. */
void gps_module_power(bool on, bool keep_vrtc = true);
/* Suspend/resume the GNSS UART (nRF UARTE: releases HFCLK). */
void gps_uart_set_power(bool on);
#if HAS_GPS_POWER_CONTROL
void gps_dump_gpio_states(void);
#endif

/* ---- gps_module_cfg.cpp ---- */
/* Boot only: configure the module (see the file). */
void gps_module_configure(void);
/* Re-run the UART configuration when `set gps diag 1` asked for it. */
void gps_diag_maybe_reconfigure(void);
void gps_uart_dump_hw_state(void);
#if HAS_GPS_UART
extern const struct device *const gps_uart_dev;
/* Bytes to the module, blind (counted while configuring). */
void gps_uart_send(const uint8_t *data, size_t len);
#endif

#endif /* HAS_GNSS */
