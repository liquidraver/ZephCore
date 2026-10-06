/*
 * SPDX-License-Identifier: MIT
 * GPS manager: module configuration and its diagnostics (see gps_internal.h).
 */

#include "gps_internal.h"
#include "ZephyrGPSManager.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_DECLARE(zephcore_gps, CONFIG_ZEPHCORE_GPS_LOG_LEVEL);

/* Runtime diagnostics toggle. Declared OUT here, not inside the HAS_GNSS
 * block: gps_set_diag()/gps_get_diag_report() are part of the
 * unconditional public API and are compiled on boards with no GNSS at all,
 * so a declaration hidden behind HAS_GNSS breaks every such board. */
static bool gps_diag_on = false;

#if HAS_GNSS

/* Multi-constellation configuration, run once at boot: the one moment a
 * modem_chat script is safe (the chip is up and streaming). Settings a power
 * cut loses are re-sent blind after each power-on (gps_module_reapply_step). */
static bool gnss_configured = false;

/* ========== Configuration Diagnostics ==========
 * Module configuration is sent blind — nothing in the protocol path tells us
 * the module accepted it. This records what was attempted so the operator can
 * read it back over the CLI on a release build. `gps_set_diag(true)` also
 * clears gnss_configured, so the next GPS enable re-runs configuration and
 * refreshes the record. RAM only, never persisted. */
enum gps_cfg_path {
	GPS_CFG_NEVER = 0,  /* configuration has not run yet */
	GPS_CFG_API,        /* driver implements the GNSS API (air530z, lc76g, ...) */
	GPS_CFG_UART,       /* passive NMEA listener — raw PMTK + UBX sent */
	GPS_CFG_BLIND,      /* no GNSS API and no writable UART — module defaults */
};

static struct {
	uint8_t path;        /* enum gps_cfg_path */
	int8_t api_ret;      /* gnss_set_enabled_systems() result */
	int8_t rate_ret;     /* gnss_set_fix_rate() result */
	uint8_t cmds;        /* config commands written to the UART */
	uint16_t bytes;      /* bytes written to the UART */
	int64_t at_ms;       /* uptime when configuration last ran */
} gps_cfg_diag;


/* Module identification string from the driver's version query; weak, so
 * boards whose driver has none still link. A captured reply is also the only
 * proof that our TX line reaches the module. */
extern "C" int zephcore_gnss_version_get(char *buf, size_t len) __attribute__((weak));

/* Count of NMEA sentences the GNSS driver has parsed. This is the one signal
 * in the whole diagnostic that cannot be misread: no satellites, a module
 * silenced by a disabled output protocol, and a module stranded at the wrong
 * baud all look identical otherwise. Non-zero means the module is alive and
 * talking at our baud, so anything still wrong is signal; zero means nothing
 * is arriving and no antenna work will change that. */
extern "C" uint32_t zephcore_gnss_rx_count(void) __attribute__((weak));

/* True only while gps_configure_via_uart() is running (see gps_uart_send). */
static bool gps_cfg_counting = false;

/* ========== Vendor-Specific Configuration Commands ==========
 * The generic-NMEA boards' GPS slot can hold any module, so the PMTK, PCAS and
 * UBX configuration are all sent; each module ignores the others' protocol. */

#if HAS_GPS_UART

/* The UART the GNSS module is connected to. Works for any GNSS-on-UART node
 * regardless of compatible string. */
const struct device *const gps_uart_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(gnss)));

/* Send raw bytes to the GPS UART with blocking poll_out. Safe beside the
 * driver's pipe: GNSS modules do not contend for our TX line. */
void gps_uart_send(const uint8_t *data, size_t len)
{
	if (!device_is_ready(gps_uart_dev)) {
		return;
	}
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(gps_uart_dev, data[i]);
	}
	/* Count only configuration traffic — the same helper carries the
	 * sleep/wake commands, which would otherwise inflate the tally. */
	if (gps_cfg_counting) {
		gps_cfg_diag.cmds++;
		gps_cfg_diag.bytes += (uint16_t)len;
	}
}

/* --- MediaTek-family (PMTK) configuration ---
 * For genuine MTK parts (L76B). Inert on CASIC parts (L76K, Air530Z). */

/* PMTK353: Enable GPS + GLONASS + Galileo + BeiDou (no QZSS).
 * Default is GPS-only. Multi-constellation dramatically improves TTFF
 * and fix reliability, especially indoors or with limited sky view. */
static const char pmtk_constellations[] = "$PMTK353,1,1,1,1,0*2B\r\n";

/* PMTK869: Enable EASY (Embedded Assist System).
 * Caches predicted satellite ephemeris in the GNSS module's internal flash.
 * Reduces TTFF from 15-45s (cold) to 1-3s (warm) for up to 3 days after
 * last fix. Setting persists in flash — resending is a harmless no-op. */
static const char pmtk_easy[] = "$PMTK869,1,1*35\r\n";

/* PMTK286: Enable AIC (Active Interference Cancellation).
 * Filters out narrowband jammers (e.g. harmonics from nearby electronics,
 * LoRa radio leakage). Improves sensitivity by ~2dB in noisy environments.
 * Especially useful when GPS antenna is near the SX1262 + SKY66122 PA. */
static const char pmtk_aic[] = "$PMTK286,1*23\r\n";

/* --- CASIC (PCAS) configuration ---
 * For an L76K/L76KB or Air530Z in a generic-NMEA slot; boards that always
 * carry one use the air530z driver instead. Inert on the other families. */

/* PCAS03: NMEA sentence selection. Field order is
 * GGA,GLL,GSA,GSV,RMC,VTG,ZDA,ANT,... — keep GGA + RMC (position, time) and
 * add GSV only when the satellite tally needs it, to keep the 9600-baud link
 * from spending its budget on sentences nobody parses. */
#ifdef CONFIG_ZEPHCORE_GPS_SAT_DIAG
static const char pcas_sentences[] = "$PCAS03,1,0,0,1,1,0,0,0,0,0,0,0,0*1F\r\n";
#else
static const char pcas_sentences[] = "$PCAS03,1,0,0,0,1,0,0,0,0,0,0,0,0*1E\r\n";
#endif

/* PCAS04,7 = GPS + BeiDou + GLONASS, everything the part supports.
 * (No Galileo on these modules — that is silicon, not configuration.) */
static const char pcas_constellations[] = "$PCAS04,7*1E\r\n";

/* PCAS11: navigation dynamic model. Stored IN THE MODULE and survives
 * reflashing the host, so a slot module that previously lived in another
 * device can arrive stuck in an automotive or airborne model that quietly
 * degrades fixes on a fixed site. See CONFIG_ZEPHCORE_GPS_NAV_MODE. */
#if CONFIG_ZEPHCORE_GPS_NAV_MODE == 0
static const char pcas_nav_mode[] = "$PCAS11,0*1D\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 1
static const char pcas_nav_mode[] = "$PCAS11,1*1C\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 2
static const char pcas_nav_mode[] = "$PCAS11,2*1F\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 3
static const char pcas_nav_mode[] = "$PCAS11,3*1E\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 4
static const char pcas_nav_mode[] = "$PCAS11,4*19\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 5
static const char pcas_nav_mode[] = "$PCAS11,5*18\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 6
static const char pcas_nav_mode[] = "$PCAS11,6*1B\r\n";
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 7
static const char pcas_nav_mode[] = "$PCAS11,7*1A\r\n";
#endif

/* PCAS06,0: ask the module to identify itself. A CASIC part answers in-band
 * with a $GPTXT sentence — the only readable reply available on this
 * otherwise write-only path, and therefore the only positive proof that the
 * MCU's TX line reaches the module at all. Captured by the GNSS driver and
 * surfaced as `mod=` in "get gps diag". */
static const char pcas_version_query[] = "$PCAS06,0*1B\r\n";

/* u-blox equivalent of the query above. u-blox ignore $PCAS06 entirely, so
 * without this a u-blox module always reports "no reply" and the TX-path
 * proof — the whole point of asking — is unavailable on those boards. The
 * $PUBX,04 reply carries time and clock status, not a version; what matters
 * is that a reply arrives at all, which only happens if the module received
 * the request. */
static const char pubx_version_query[] = "$PUBX,04*37\r\n";

/* --- u-blox NMEA output trim ($PUBX,40) ---
 * At 9600 baud the multi-GNSS sentence set does not fit in a second, so GLL,
 * GSA and VTG are switched off, and GSV unless the satellite tally needs it. */
static const char pubx_off_gll[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n";
static const char pubx_off_gsa[] = "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n";
static const char pubx_off_vtg[] = "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n";
#ifndef CONFIG_ZEPHCORE_GPS_SAT_DIAG
static const char pubx_off_gsv[] = "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n";
#endif

/* --- u-blox ZOE-M8Q (UBX binary) configuration --- */

/* UBX-CFG-PRT: force UART1 to 9600 8N1 with UBX and NMEA in and out, sent
 * first. A module saved as "UBX only" by another host is otherwise silent. */
static const uint8_t ubx_cfg_prt_uart1[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08,
	0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x8E, 0x95
};

/* UBX-CFG-GNSS for u-blox M8: GPS + Galileo + GLONASS + QZSS on, BeiDou and
 * SBAS off. Blocks are 8 bytes each (payload 4 + 8*n); resTrkCh must be 0 on
 * the disabled blocks, and GPS and QZSS go together. The receiver rejects
 * the whole message for one bad block. u-blox 7 parts refuse it. */
static const uint8_t ubx_cfg_gnss[] = {
	0xB5, 0x62, 0x06, 0x3E, 0x34, 0x00, 0x00, 0x00, 0xFF, 0x06, 0x00, 0x08,
	0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x00,
	0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0xEE, 0x64
};

/* UBX-CFG-NMEA: NMEA 4.10, without which Galileo and BeiDou cannot be
 * reported. gsvTalkerId must stay 0 for the per-constellation tally. */
static const uint8_t ubx_cfg_nmea_410[] = {
	0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x75, 0x57
};

/* UBX-CFG-NAV5: 5 degree minimum satellite elevation (apply mask 0x0002);
 * the dynamic model stays at the factory default. */
static const uint8_t ubx_cfg_nav5_minelev[] = {
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x02, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0xE7
};

/* UBX-CFG-NAVX5: Enable AssistNow Autonomous (AOP).
 * u-blox equivalent of Quectel EASY — the receiver autonomously predicts
 * satellite orbits from previously downloaded ephemeris data. Predictions
 * stay valid for 3-6 days, reducing TTFF from 26-30s (cold) to 2-5s.
 * No server connection needed — runs entirely on-chip.
 * mask1 bit 14 = aop, aopCfg bit 0 = enable. */
static const uint8_t ubx_cfg_navx5_aop[] = {
	0xB5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x04, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x66
};

/* UBX-CFG-CFG: Save all configuration to BBR + Flash + EEPROM.
 * Persists constellation, nav model, SBAS settings across power cycles
 * and backup mode. Without this, ZOE-M8Q reverts to factory defaults
 * after a full power loss (though BBR survives backup mode). */
static const uint8_t ubx_cfg_save[] = {
	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF
};

/* Settle time after a command that makes the receiver restart its navigation
 * engine — constellation changes on every family do this. A command issued
 * into a restarting engine is simply lost, and because everything here is
 * written blind the loss is silent. 20-50 ms was optimistic. */
#define GPS_CFG_SETTLE_MS      120
#define GPS_CFG_RESTART_MS     500

/* Send an NMEA command string (including \r\n). */
static void gps_send_nmea(const char *cmd, uint32_t settle_ms = GPS_CFG_SETTLE_MS)
{
	gps_uart_send((const uint8_t *)cmd, strlen(cmd));
	k_msleep(settle_ms);
}

/* Send a UBX binary frame. */
static void gps_send_ubx(const uint8_t *frame, size_t len,
			 uint32_t settle_ms = GPS_CFG_SETTLE_MS)
{
	gps_uart_send(frame, len);
	k_msleep(settle_ms);
}

/* u-blox dynamic model for CFG-NAV5, translated from the CASIC-numbered
 * CONFIG_ZEPHCORE_GPS_NAV_MODE so one setting drives both families.
 * u-blox: 0 portable, 2 stationary, 3 pedestrian, 4 automotive, 5 sea,
 * 6-8 airborne. */
#if   CONFIG_ZEPHCORE_GPS_NAV_MODE == 1
#define UBX_DYNMODEL 2
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 2
#define UBX_DYNMODEL 3
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 3
#define UBX_DYNMODEL 4
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 4
#define UBX_DYNMODEL 5
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE >= 5
#define UBX_DYNMODEL 6
#elif CONFIG_ZEPHCORE_GPS_NAV_MODE == 0
#define UBX_DYNMODEL 0
#endif

/* Send CFG-NAV5 with the dynamic model patched in and the checksum redone.
 * A stationary model on a fixed node suppresses position wander and lets the
 * receiver apply much tighter velocity assumptions; leaving a roof-mounted
 * repeater on the factory Portable model throws that away. Patched at runtime
 * rather than as eight hard-coded frames — one place to get the checksum
 * right instead of eight. */
#ifdef UBX_DYNMODEL
static void gps_send_ubx_nav5(void)
{
	uint8_t f[sizeof(ubx_cfg_nav5_minelev)];

	memcpy(f, ubx_cfg_nav5_minelev, sizeof(f));
	f[6] = 0x03;            /* mask: dyn (bit0) + minEl (bit1) */
	f[8] = UBX_DYNMODEL;    /* dynModel */

	uint8_t ck_a = 0, ck_b = 0;
	for (size_t i = 2; i < sizeof(f) - 2; i++) {
		ck_a = (uint8_t)(ck_a + f[i]);
		ck_b = (uint8_t)(ck_b + ck_a);
	}
	f[sizeof(f) - 2] = ck_a;
	f[sizeof(f) - 1] = ck_b;

	gps_send_ubx(f, sizeof(f));
}
#endif

/* Configure the GPS module with optimal settings for a mesh repeater.
 * Sends both PMTK (Quectel) and UBX (u-blox) commands — the module that
 * isn't present ignores bytes it doesn't understand. */
static void gps_configure_via_uart(void)
{
	LOG_INF("GPS: Configuring via UART (PMTK + UBX dual-protocol)");

	gps_cfg_diag.cmds = 0;
	gps_cfg_diag.bytes = 0;
	gps_cfg_counting = true;

	/* --- MediaTek-family (PMTK) --- */
	gps_send_nmea(pmtk_constellations, GPS_CFG_RESTART_MS);
	gps_send_nmea(pmtk_easy);
	gps_send_nmea(pmtk_aic);
	LOG_INF("GPS: PMTK config sent (constellations, EASY, AIC)");

	/* --- CASIC (PCAS) ---
	 * Version query last, so its $GPTXT reply is not stepped on by a
	 * constellation restart. */
	gps_send_nmea(pcas_sentences);
	gps_send_nmea(pcas_constellations, GPS_CFG_RESTART_MS);
#if CONFIG_ZEPHCORE_GPS_NAV_MODE >= 0
	gps_send_nmea(pcas_nav_mode);
#endif
	gps_send_nmea(pcas_version_query);
	LOG_INF("GPS: PCAS config sent (sentences, constellations, nav mode, version query)");

	/* --- u-blox ZOE-M8Q (UBX) ---
	 * Order matters: NMEA 4.10 first, CFG-GNSS with a long settle (it restarts
	 * the navigation engine), CFG-CFG last so only accepted settings are saved. */
	/* Make sure NMEA output is even switched on before anything else — a
	 * module saved as UBX-only by a previous host is otherwise mute. */
	gps_send_ubx(ubx_cfg_prt_uart1, sizeof(ubx_cfg_prt_uart1), GPS_CFG_RESTART_MS);

	/* Trim the NMEA stream BEFORE enabling more constellations — the extra
	 * GSV traffic must have somewhere to fit. */
	gps_send_nmea(pubx_off_gll);
	gps_send_nmea(pubx_off_gsa);
	gps_send_nmea(pubx_off_vtg);
#ifndef CONFIG_ZEPHCORE_GPS_SAT_DIAG
	gps_send_nmea(pubx_off_gsv);
#endif
	gps_send_ubx(ubx_cfg_nmea_410, sizeof(ubx_cfg_nmea_410));
	gps_send_ubx(ubx_cfg_gnss, sizeof(ubx_cfg_gnss), GPS_CFG_RESTART_MS);
#ifdef UBX_DYNMODEL
	gps_send_ubx_nav5();
#else
	gps_send_ubx(ubx_cfg_nav5_minelev, sizeof(ubx_cfg_nav5_minelev));
#endif
	gps_send_ubx(ubx_cfg_navx5_aop, sizeof(ubx_cfg_navx5_aop));
	gps_send_ubx(ubx_cfg_save, sizeof(ubx_cfg_save));
	/* Ask a u-blox to say something back — the TX-path proof for this
	 * family, sent last so the reply is not stepped on by a restart. */
	gps_send_nmea(pubx_version_query);
	LOG_INF("GPS: UBX config sent (NMEA 4.10, multi-GNSS, 5° min elev, AOP, saved)");

	gps_cfg_counting = false;
}
#endif /* HAS_GPS_UART */

/* Re-run module configuration on a GPS enable, only when diagnostics are
 * armed. Only the raw-UART path can be re-run; on API-driver boards
 * `get gps diag` keeps the boot-time result. */
void gps_diag_maybe_reconfigure(void)
{
	if (!gps_diag_on || gnss_configured || gnss_dev == NULL) {
		return;
	}
#if HAS_GPS_UART
	if (gps_cfg_diag.path == GPS_CFG_UART || gps_cfg_diag.path == GPS_CFG_NEVER) {
		/* The module has just been powered; give it time to boot before
		 * clocking configuration at it (same ~300 ms the modem needs). */
		k_msleep(300);
		gps_cfg_diag.path = GPS_CFG_UART;
		gps_cfg_diag.at_ms = k_uptime_get();
		gps_configure_via_uart();
	}
#endif
	gnss_configured = true;
}

void gps_module_configure(void)
{
	if (gnss_configured || gnss_dev == NULL) {
		return;
	}

	/* Enable all available constellation systems for faster TTFF.
	 * Try GPS+GLONASS+Galileo+BeiDou first (AG3335 supports all).
	 * Fall back to GPS+GLONASS+BeiDou if Galileo not supported (L76KB). */
	gnss_systems_t systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS |
				 GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;
	int ret = gnss_set_enabled_systems(gnss_dev, systems);
	if (ret == -EINVAL) {
		/* Some systems not supported — try without Galileo */
		systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_BEIDOU;
		ret = gnss_set_enabled_systems(gnss_dev, systems);
	}
	gps_cfg_diag.api_ret = (int8_t)ret;
	gps_cfg_diag.at_ms = k_uptime_get();

	if (ret == 0) {
		LOG_INF("GPS: Multi-constellation enabled via GNSS API");
		gps_cfg_diag.path = GPS_CFG_API;
	} else if (ret == -ENOSYS || ret == -ENOTSUP) {
#if HAS_GPS_UART
		/* gnss-nmea-generic is a passive listener — no GNSS API.
		 * Configure everything via direct UART commands instead. */
		gps_cfg_diag.path = GPS_CFG_UART;
		gps_configure_via_uart();
#else
		LOG_INF("GPS: No GNSS API and no UART access — using module defaults");
		gps_cfg_diag.path = GPS_CFG_BLIND;
#endif
	} else {
		/* Not retried: the API path is safe at boot only (see
		 * gps_manager_init). Recorded, so get gps diag shows the error
		 * rather than "never-run". */
		LOG_WRN("GPS: Failed to set constellations: %d", ret);
		gps_cfg_diag.path = GPS_CFG_API;
		return;
	}

	/* Set 1Hz fix rate (explicit, don't rely on chip defaults) */
	ret = gnss_set_fix_rate(gnss_dev, 1000);
	gps_cfg_diag.rate_ret = (int8_t)ret;
	if (ret == 0) {
		LOG_INF("GPS: Fix rate set to 1Hz");
	} else if (ret != -ENOSYS && ret != -ENOTSUP) {
		LOG_WRN("GPS: Failed to set fix rate: %d", ret);
	}

	gnss_configured = true;
}

#if HAS_GPS_UART && defined(CONFIG_ZEPHCORE_GPS_REAPPLY)
/* The boot-time settings a power cut loses, one sentence per step; the caller
 * waits each entry's gap with a work item. UBX, PCAS02 and PCAS06 stay
 * boot-only. Each constellation command is followed by its restart settle. */
struct reapply_cmd {
	const char *sentence;
	uint16_t gap_ms;        /* wait before the next one */
};

static const struct reapply_cmd reapply_casic[] = {
	{ pcas_sentences, GPS_CFG_SETTLE_MS },
#if CONFIG_ZEPHCORE_GPS_NAV_MODE >= 0
	/* The driver's GNSS_LUATOS_AIR530Z_NAV_MODE mirrors this value, so
	 * this matches what either path set at boot. */
	{ pcas_nav_mode, GPS_CFG_SETTLE_MS },
#endif
	{ pcas_constellations, 0 },
};
static const struct reapply_cmd reapply_generic[] = {
	{ pmtk_constellations, GPS_CFG_RESTART_MS },
	{ pmtk_easy, GPS_CFG_SETTLE_MS },
	{ pmtk_aic, GPS_CFG_SETTLE_MS },
	{ pcas_sentences, GPS_CFG_SETTLE_MS },
#if CONFIG_ZEPHCORE_GPS_NAV_MODE >= 0
	{ pcas_nav_mode, GPS_CFG_SETTLE_MS },
#endif
	{ pcas_constellations, 0 },
};

uint32_t gps_module_reapply_step(uint8_t step)
{
	bool generic = (gps_cfg_diag.path == GPS_CFG_UART);
	const struct reapply_cmd *list = generic ? reapply_generic : reapply_casic;
	size_t n = generic ? ARRAY_SIZE(reapply_generic) : ARRAY_SIZE(reapply_casic);

	if (step >= n) {
		return 0;
	}
	gps_uart_send((const uint8_t *)list[step].sentence, strlen(list[step].sentence));
	return ((size_t)step + 1 < n) ? list[step].gap_ms : 0;
}
#endif

/* ========== GPS UART Diagnostics ========== */

/**
 * Dump the GNSS UARTE's hardware register state (nRF52840).
 * Reads PSEL (pin select), ENABLE, BAUDRATE, and ERRORSRC directly
 * from the peripheral registers — no assumptions, just facts.
 */
void gps_uart_dump_hw_state(void)
{
#ifdef GPS_NRF_UARTE
	NRF_UARTE_Type *uart = GPS_NRF_UARTE;

	uint32_t psel_txd = uart->PSEL.TXD;
	uint32_t psel_rxd = uart->PSEL.RXD;
	uint32_t enable   = uart->ENABLE;
	uint32_t baudrate = uart->BAUDRATE;
	uint32_t errorsrc = uart->ERRORSRC;

	/* PSEL format: bit 31 = CONNECT (0=connected, 1=disconnected),
	 * bits 4:0 = pin, bit 5 = port */
	bool txd_connected = !(psel_txd & (1U << 31));
	bool rxd_connected = !(psel_rxd & (1U << 31));
	uint8_t txd_port = (psel_txd >> 5) & 1;
	uint8_t txd_pin  = psel_txd & 0x1F;
	uint8_t rxd_port = (psel_rxd >> 5) & 1;
	uint8_t rxd_pin  = psel_rxd & 0x1F;

	LOG_INF("UART0 HW state:");
	LOG_INF("  ENABLE=0x%02x (8=enabled)", enable);
	LOG_INF("  PSEL.TXD=0x%08x → P%d.%02d %s",
		psel_txd, txd_port, txd_pin,
		txd_connected ? "CONNECTED" : "DISCONNECTED");
	LOG_INF("  PSEL.RXD=0x%08x → P%d.%02d %s",
		psel_rxd, rxd_port, rxd_pin,
		rxd_connected ? "CONNECTED" : "DISCONNECTED");
	LOG_INF("  BAUDRATE=0x%08x ERRORSRC=0x%x", baudrate, errorsrc);

	/* Clear any error flags */
	if (errorsrc) {
		uart->ERRORSRC = errorsrc;
		LOG_WRN("  UART errors cleared: overrun=%d parity=%d framing=%d break=%d",
			(errorsrc >> 0) & 1, (errorsrc >> 1) & 1,
			(errorsrc >> 2) & 1, (errorsrc >> 3) & 1);
	}
#endif
}

#endif /* HAS_GNSS */

void gps_set_diag(bool on)
{
	gps_diag_on = on;
#if HAS_GNSS
	if (on) {
		/* Re-arm configuration so the next GPS enable ("gps off" then
		 * "gps on") actually re-runs it and refreshes the record. */
		gnss_configured = false;
	}
#endif
}

void gps_get_diag_report(char *buf, size_t len)
{
#if HAS_GNSS
	static const char *const path_str[] = { "never", "api", "uart", "blind" };
	uint8_t path = gps_cfg_diag.path;
	const char *pname = (path < ARRAY_SIZE(path_str)) ? path_str[path] : "?";

	if (path == GPS_CFG_NEVER) {
		snprintf(buf, len, "diag=%s cfg=never-run (enable, then 'gps off'/'gps on')",
			 gps_diag_on ? "on" : "off");
		return;
	}

	uint32_t age_s = (uint32_t)((k_uptime_get() - gps_cfg_diag.at_ms) / 1000);
	size_t n = (size_t)snprintf(buf, len, "diag=%s cfg=%s age=%us",
				    gps_diag_on ? "on" : "off", pname, age_s);
	if (n >= len) {
		return;
	}

	/* Sentences parsed from the module. Print this before anything else
	 * that could be misinterpreted — rx=0 makes every other field moot. */
	if (zephcore_gnss_rx_count != NULL) {
		n += (size_t)snprintf(buf + n, len - n, " rx=%u",
				      (unsigned)zephcore_gnss_rx_count());
		if (n >= len) {
			return;
		}
	}

	/* Module identity, when the driver could obtain it. Present means the
	 * module answered us, i.e. the TX path is good; absent on a driver that
	 * asks is a strong hint the module never hears our configuration. */
	if (zephcore_gnss_version_get != NULL) {
		char ver[40];
		if (zephcore_gnss_version_get(ver, sizeof(ver)) > 0) {
			n += (size_t)snprintf(buf + n, len - n, " mod=%s", ver);
		} else {
			n += (size_t)snprintf(buf + n, len - n, " mod=no-reply");
		}
		if (n >= len) {
			return;
		}
	}

	if (path == GPS_CFG_UART) {
		/* Bytes actually clocked out to the module. Note these are sent
		 * blind — this proves transmission, not acceptance. Constellation
		 * tallies below are the acceptance evidence. */
		n += (size_t)snprintf(buf + n, len - n, " sent=%u/%uB",
				      gps_cfg_diag.cmds, gps_cfg_diag.bytes);
	} else if (path == GPS_CFG_API) {
		n += (size_t)snprintf(buf + n, len - n, " sys_ret=%d rate_ret=%d",
				      gps_cfg_diag.api_ret, gps_cfg_diag.rate_ret);
	}
	if (n >= len) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_GPS_SAT_DIAG
	uint8_t shown[5];
	gps_sat_tally(shown);
	snprintf(buf + n, len - n, " sys=G%u/R%u/E%u/B%u/?%u",
		 shown[0], shown[1], shown[2], shown[3], shown[4]);
#else
	snprintf(buf + n, len - n, " (build without GPS_SAT_DIAG: no per-constellation proof)");
#endif
#else
	ARG_UNUSED(gps_diag_on);
	snprintf(buf, len, "no GNSS on this board");
#endif
}

