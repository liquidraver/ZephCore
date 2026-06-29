/*
 * ZephCore - UI Page System
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Defines the multi-page UI matching Arduino's ui-new implementation.
 * Each page has a render function that draws to the display.
 */

#ifndef ZEPHCORE_UI_PAGES_H
#define ZEPHCORE_UI_PAGES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Page Definitions ========== */
enum ui_page {
	UI_PAGE_MESSAGES = 0,   /* MSG count + connection status */
	UI_PAGE_RECENT,         /* Recently heard contacts */
	UI_PAGE_RADIO,          /* LoRa params (freq, SF, BW, CR, power, noise) */
	UI_PAGE_BLUETOOTH,      /* BLE toggle */
	UI_PAGE_ADVERT,         /* Send broadcast advert */
	UI_PAGE_GPS,            /* GPS status / position */
	UI_PAGE_BUZZER,         /* Buzzer mute toggle */
	UI_PAGE_LEDS,           /* LED enable/disable toggle */
	UI_PAGE_SENSORS,        /* Environment sensor data */
	UI_PAGE_OFFGRID,        /* Offgrid mode (client repeat) toggle */
	UI_PAGE_DFU,            /* BLE DFU bootloader entry */
	UI_PAGE_SHUTDOWN,       /* Hibernate / power off */
	UI_PAGE_STATUS,         /* Repeater status (uptime, time, packets) */
	UI_PAGE_COUNT
};

/* ========== Page State (set by main app) ========== */
struct ui_state {
	/* Global / top bar */
	char     node_name[24];    /* device node name for top bar */
	uint16_t battery_mv;
	uint8_t  battery_pct;
	uint32_t rtc_epoch;        /* Unix epoch from RTC (0 = not set) */

	/* Messages page */
	uint16_t msg_count;
	bool     ble_connected;
	char     device_name[25];

	/* Recently heard */
	struct {
		char     name[16];
		int16_t  rssi;
		uint32_t age_s;       /* seconds since heard (recomputed each refresh) */
	} recent[4];
	uint8_t  recent_count;

	/* Radio page */
	uint32_t lora_freq_hz;
	uint8_t  lora_sf;
	uint16_t lora_bw_khz_x10;  /* bandwidth in 0.1 kHz (e.g. 625 = 62.5 kHz) */
	uint8_t  lora_cr;
	int8_t   lora_tx_power;
	int16_t  lora_noise_floor;
	int8_t   lora_effective_tx_power;
	bool     lora_apc_enabled;
	int8_t   lora_apc_reduction;
	int16_t  lora_apc_margin_x10;
	uint8_t  lora_apc_target_margin;
	uint8_t  lora_sync_word;
	uint16_t lora_preamble_len;
	bool     lora_rx_duty_cycle;
	bool     lora_radio_ready;
	bool     lora_in_rx;
	bool     lora_tx_active;
	uint32_t lora_packets_rx;
	uint32_t lora_packets_tx;
	uint32_t lora_packets_err;

	/* Bluetooth page */
	bool     ble_enabled;      /* true = BLE active, false = serial mode */

	/* GPS page */
	bool     gps_available;    /* true = GNSS hardware detected at boot */
	bool     gps_enabled;      /* GPS hardware on/off */
	bool     gps_has_fix;
	uint8_t  gps_satellites;
	int32_t  gps_lat_mdeg;    /* latitude in milli-degrees */
	int32_t  gps_lon_mdeg;    /* longitude in milli-degrees */
	int32_t  gps_alt_mm;      /* altitude in millimeters */
	uint8_t  gps_state;       /* 0=OFF, 1=STANDBY, 2=ACQUIRING */
	uint32_t gps_last_fix_age_s;  /* seconds since last fix (UINT32_MAX=never) */
	uint32_t gps_next_search_s;   /* seconds until next search (0=now/off) */

	/* Buzzer page */
	bool     buzzer_quiet;     /* true = muted */

	/* LEDs page */
	bool     leds_disabled;    /* true = LEDs off */

	/* Sensors page */
	int16_t  temperature_c10;  /* centi-degrees C */
	uint32_t pressure_pa;      /* pascals */
	uint16_t humidity_rh10;    /* 0.1% RH */
	uint16_t light_lux;        /* lux */

	/* Current page */
	enum ui_page current_page;

	/* Offgrid mode (client repeat) */
	bool     offgrid_enabled;    /* true = forwarding packets */

	/* Transient feedback (shown briefly after action) */
	uint32_t advert_sent_time;   /* uptime ms when advert was sent (0=idle) */
	bool     advert_was_flood;   /* true if last advert was flood */
	uint32_t offgrid_confirm_time; /* uptime ms when offgrid toggle confirm started (0=idle) */
	uint32_t dfu_confirm_time;   /* uptime ms when DFU confirm started (0=idle) */
	uint32_t shutdown_confirm_time; /* uptime ms when shutdown confirm started (0=idle) */
};

/**
 * Signal that an advert was just sent (shows feedback on advert page).
 * @param flood true if flood advert, false if zero-hop
 */
void ui_pages_advert_sent(bool flood);

/**
 * Get the global UI state for updating from main app.
 */
struct ui_state *ui_pages_get_state(void);

/**
 * Render the current page to the display.
 * Draws page indicator dots, then the page-specific content.
 */
void ui_pages_render(void);

/**
 * Render a splash screen.
 */
void ui_pages_render_splash(void);

/**
 * Navigate to next page (wrap around).
 */
void ui_pages_next(void);

/**
 * Navigate to previous page (wrap around).
 */
void ui_pages_prev(void);

/**
 * Set the current page directly.
 */
void ui_pages_set(enum ui_page page);

/**
 * Get the current page.
 */
enum ui_page ui_pages_current(void);

/**
 * Set the node name displayed in the top bar.
 */
void ui_pages_set_node_name(const char *name);


#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_UI_PAGES_H */
