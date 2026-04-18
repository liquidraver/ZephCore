/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore - Repeater (USB CLI, Event-Driven)
 *
 * This is the main entry point for the repeater role.
 * Repeaters use USB serial CLI for configuration (no BLE).
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_repeater_main, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/reboot.h>
#include "oled_power.h"

/* BLE controller assert handler — BT is compiled even for repeater (via zephcore_common.conf) */
#if IS_ENABLED(CONFIG_BT_CTLR_ASSERT_HANDLER)
extern "C" void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	LOG_ERR("!!! BLE CONTROLLER ASSERT: %s:%u !!!", file ? file : "?", line);
	k_sleep(K_MSEC(100));
	sys_reboot(SYS_REBOOT_COLD);
}
#endif

/* USB CDC with 1200 baud touch detection (when not using auto-init) */
#if !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
#include <ZephyrRepeaterUSB.h>
#endif

#include <app/RepeaterDataStore.h>
#include <app/RepeaterMesh.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <ZephyrSensorManager.h>

/* UI subsystem (display, buttons, buzzer) */
#include "ui_task.h"

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <mesh/RadioIncludes.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_WIFI_OTA)
#include "wifi_ota.h"
#endif

/* LED configuration */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif
#if DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
#endif

/* USB CLI configuration */
#define USB_RING_BUF_SIZE 512
#define CLI_LINE_BUF_SIZE 256

/*
 * Event-driven mesh loop - replaces 50ms polling with true event signaling.
 * Events are signaled from ISR/callbacks, mesh loop wakes immediately.
 */
#define MESH_EVENT_LORA_RX       BIT(0)  /* LoRa packet received */
#define MESH_EVENT_LORA_TX_DONE  BIT(1)  /* LoRa TX complete */
#define MESH_EVENT_CLI_RX        BIT(2)  /* CLI command received */
#define MESH_EVENT_HOUSEKEEPING  BIT(3)  /* Periodic housekeeping (noise floor, etc.) */
#define MESH_EVENT_GPS_ACTION    BIT(4)  /* GPS state change (must run on main thread!) */
#define MESH_EVENT_TX_DRAIN      BIT(5)  /* Outbound packet delay expired, run checkSend */
#define MESH_EVENT_ALL           (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | MESH_EVENT_CLI_RX | MESH_EVENT_HOUSEKEEPING | MESH_EVENT_GPS_ACTION | MESH_EVENT_TX_DRAIN)

/* Housekeeping interval - infrequent to preserve power savings */
#define HOUSEKEEPING_INTERVAL_MS CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS

/* Event object for mesh loop */
static struct k_event mesh_events;

/* USB CDC state */
static const struct device *usb_dev;
static uint8_t usb_ring_buf_data[USB_RING_BUF_SIZE];
static struct ring_buf usb_ring_buf;
static char cli_line_buf[CLI_LINE_BUF_SIZE];
static char cli_reply_buf[256];
static uint16_t cli_line_idx;

/* Work items for event-driven processing */
static void cli_rx_work_fn(struct k_work *work);
static void housekeeping_timer_fn(struct k_timer *timer);
static void tx_drain_work_fn(struct k_work *work);
static void initial_advert_work_fn(struct k_work *work);
K_WORK_DEFINE(cli_rx_work, cli_rx_work_fn);
K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);
K_WORK_DELAYABLE_DEFINE(initial_advert_work, initial_advert_work_fn);

/* Housekeeping timer for periodic tasks (noise floor calibration, etc.) */
K_TIMER_DEFINE(housekeeping_timer, housekeeping_timer_fn, NULL);

/* Forward declarations */
#ifdef ZEPHCORE_LORA
static RepeaterMesh *repeater_mesh_ptr;
#endif

/* Print string to USB serial */
static void cli_print(const char *str)
{
	if (!usb_dev) return;
	while (*str) {
		uart_poll_out(usb_dev, *str++);
	}
}

/* USB CDC UART interrupt callback */
static void cli_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
			if (recv_len > 0) {
				ring_buf_put(&usb_ring_buf, buf, recv_len);
				k_work_submit(&cli_rx_work);
			}
		}
	}
}

/* CLI RX work - processes line-based CLI commands
 * Matches Arduino behavior: echo each char, then "  -> reply" on enter
 */
static void cli_rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t byte;

	while (ring_buf_get(&usb_ring_buf, &byte, 1) == 1) {
		/* Process command on \r OR \n (support echo from Linux) */
		if (byte == '\r' || byte == '\n') {
			if (cli_line_idx > 0) {
				cli_line_buf[cli_line_idx] = '\0';

				/* Debug: log received command */
				LOG_INF("CLI cmd len=%d: %.40s%s", cli_line_idx,
					cli_line_buf, cli_line_idx > 40 ? "..." : "");

				/* Process CLI command */
#ifdef ZEPHCORE_LORA
				if (repeater_mesh_ptr) {
					cli_reply_buf[0] = '\0';
					repeater_mesh_ptr->handleCommand(0, cli_line_buf, cli_reply_buf);
					if (cli_reply_buf[0] != '\0') {
						/* Arduino format: newline, then "  -> reply" */
						cli_print("\r\n  -> ");
						cli_print(cli_reply_buf);
					}
				}
#endif
				cli_line_idx = 0;
			}
			/* New line for next command */
			cli_print("\r\n");
		} else if (byte == 0x7F || byte == 0x08) {
			/* Backspace - echo backspace sequence */
			if (cli_line_idx > 0) {
				cli_line_idx--;
				if (usb_dev) {
					uart_poll_out(usb_dev, '\b');
					uart_poll_out(usb_dev, ' ');
					uart_poll_out(usb_dev, '\b');
				}
			}
		} else if (cli_line_idx < sizeof(cli_line_buf) - 1) {
			/* Echo character back (like Arduino) */
			if (usb_dev) {
				uart_poll_out(usb_dev, byte);
			}
			cli_line_buf[cli_line_idx++] = (char)byte;
		}
	}
}

/* Housekeeping timer callback - signals event to wake mesh loop periodically */
static void housekeeping_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_HOUSEKEEPING);
}

#ifdef ZEPHCORE_LORA
/* LoRa RX callback - called from ISR context when packet received */
static void lora_rx_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

/* LoRa TX complete callback */
static void lora_tx_done_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_TX_DONE);
}

/* TX drain — Dispatcher queued a packet with a delay.
 * Schedule a precise wake so checkSend runs when the delay expires. */
static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_event_post(&mesh_events, MESH_EVENT_TX_DRAIN);
}

/* Deferred initial advertisement — gives GPS time to get a fix at boot */
static void initial_advert_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
#ifdef ZEPHCORE_LORA
	if (repeater_mesh_ptr) {
		LOG_INF("Sending deferred initial advertisement");
		repeater_mesh_ptr->sendSelfAdvertisement(500, false);
	}
#endif
}

static void tx_queued_callback(uint32_t delay_ms, void *user_data)
{
	ARG_UNUSED(user_data);
	k_work_reschedule(&tx_drain_work, K_MSEC(delay_ms));
}
#endif

/* Global instances */
static mesh::ZephyrRTCClock rtc_clock;

/* GPS event callback - called when GPS work handlers need the main thread
 * to process a state transition (wake from standby, fix done, timeout).
 * Runs from system work queue context — just posts an event, no blocking. */
static void gps_event_callback(void)
{
	k_event_post(&mesh_events, MESH_EVENT_GPS_ACTION);
}

static RepeaterDataStore data_store;

/* GPS fix callback - syncs RTC from GPS time.
 * Repeaters do NOT update prefs lat/lon from GPS — prefs coordinates are the
 * user's manually-set position used for adverts.  Precise GPS position is
 * served only via telemetry requests (gps_get_last_known_position). */
static void gps_fix_callback(double lat, double lon, int64_t utc_time)
{
	if (utc_time > 0) {
		LOG_INF("GPS fix: RTC sync time=%lld", utc_time);
		rtc_clock.setCurrentTime((uint32_t)utc_time);
	}

	int lat_deg = (int)lat;
	int lon_deg = (int)lon;
	int lat_frac = (int)((lat - lat_deg) * 1000000);
	int lon_frac = (int)((lon - lon_deg) * 1000000);
	if (lat_frac < 0) lat_frac = -lat_frac;
	if (lon_frac < 0) lon_frac = -lon_frac;
	LOG_INF("GPS fix: lat=%d.%06d lon=%d.%06d (telemetry only)",
		lat_deg, lat_frac, lon_deg, lon_frac);
}

#ifdef ZEPHCORE_LORA
static mesh::ZephyrBoard zephyr_board;

/* Radio prefs — initialized with defaults in main(), updated after mesh.begin()
 * loads persisted prefs. Passed to radio adapter at static construction time. */
static NodePrefs radio_prefs;

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
/* LR1110 via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LR1110Radio lora_radio(lora_dev, zephyr_board, &radio_prefs);
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_SX127X)
/* SX127x via Zephyr loramac-node driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX127xRadio lora_radio(lora_dev, zephyr_board, &radio_prefs);
#else
/* SX126x via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX126xRadio lora_radio(lora_dev, zephyr_board, &radio_prefs);
#endif

static mesh::ZephyrMillisecondClock ms_clock;
static mesh::ZephyrRNG zephyr_rng;
static mesh::SimpleMeshTables mesh_tables;

/* RepeaterMesh requires: board, radio, ms_clock, rng, rtc, tables */
static RepeaterMesh repeater_mesh(zephyr_board, lora_radio, ms_clock, zephyr_rng, rtc_clock, mesh_tables);
#endif

/* Repeater event loop */
static void repeater_event_loop(void)
{
	LOG_INF("starting event-driven loop");

	/* Print startup banner (no prompt - Arduino style) */
	cli_print("\r\n=== ZephCore Repeater ===\r\n");

	/* Start housekeeping timer for periodic maintenance tasks */
	k_timer_start(&housekeeping_timer, K_MSEC(HOUSEKEEPING_INTERVAL_MS),
		      K_MSEC(HOUSEKEEPING_INTERVAL_MS));

	for (;;) {
		/* Wait for any mesh event - blocks until signaled */
		uint32_t events = k_event_wait(&mesh_events, MESH_EVENT_ALL, false, K_FOREVER);
		k_event_clear(&mesh_events, events);

		/* GPS state transitions must run on main thread (GNSS driver
		 * modem_chat blocks on system work queue semaphore). */
		if (events & MESH_EVENT_GPS_ACTION) {
			gps_process_event();
		}

#ifdef ZEPHCORE_LORA
		/* Packet processing — only on radio/CLI/TX events */
		if (repeater_mesh_ptr &&
		    (events & (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE |
			       MESH_EVENT_CLI_RX | MESH_EVENT_TX_DRAIN))) {
			repeater_mesh_ptr->loop();
		}
#endif

		/* Periodic housekeeping — maintenance + display refresh */
		if (events & MESH_EVENT_HOUSEKEEPING) {
#ifdef ZEPHCORE_LORA
			/* Radio maintenance: noise floor calibration, AGC reset,
			 * RX watchdog.  Separated from loop() so these never run
			 * on packet-driven events. */
			if (repeater_mesh_ptr) {
				repeater_mesh_ptr->maintenanceLoop();
			}
#endif

			ui_set_clock(rtc_clock.getCurrentTime());

#ifdef ZEPHCORE_LORA
			/* Refresh radio params (noise floor changes from calibration) */
			if (repeater_mesh_ptr) {
				NodePrefs *p = repeater_mesh_ptr->getNodePrefs();
				ui_set_radio_params(
					(uint32_t)(p->freq * 1000000.0f + 0.5f),
					p->sf,
					(uint16_t)(p->bw * 10.0f + 0.5f),
					p->cr,
					p->tx_power_dbm,
					lora_radio.getNoiseFloor());
			}

			/* Battery every ~60s (12 × 5s housekeeping) */
			static uint8_t batt_counter;
			if (++batt_counter >= 12) {
				batt_counter = 0;
				ui_set_battery(zephyr_board.getBattMilliVolts(), 0);
			}
#endif

			ui_refresh_display();
		}
	}
}

int main(void)
{
	/* Initialize radio prefs with safe defaults before anything else */
	initNodePrefs(&radio_prefs);
	strcpy(radio_prefs.node_name, "Repeater");

#ifdef ZEPHCORE_LORA
	/* Clear any stale bootloader magic from previous sessions.
	 * Prevents nRF52 boards from re-entering bootloader after reboot. */
	zephyr_board.clearBootloaderMagic();
#endif

	/* Wait for USB CDC to enumerate before any logging */
	k_sleep(K_MSEC(2000));
	LOG_INF("=== ZephCore Repeater starting ===");

#if IS_ENABLED(CONFIG_ZEPHCORE_WIFI_OTA)
	/* Confirm MCUboot image early — if we just booted after OTA,
	 * this marks the image as good so MCUboot keeps it. */
	wifi_ota_confirm_image();
#endif

	/* Configure LEDs */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
	if (gpio_is_ready_dt(&led0)) {
		gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
	if (gpio_is_ready_dt(&led1)) {
		gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	}
#endif

	/* Initialize repeater data store */
	if (!data_store.begin()) {
		LOG_ERR("RepeaterDataStore init failed");
	}

	/* Initialize sensor manager */
	sensor_manager_init();

	/* Set GPS to repeater mode: power off now, wake every 48h for time sync only.
	 * This prevents GPS from draining power on boards that have it (e.g., Wio Tracker). */
	if (gps_is_available()) {
		gps_set_fix_callback(gps_fix_callback);
		gps_set_event_callback(gps_event_callback);
		gps_set_repeater_mode(true);
	}

	/* Initialize UI (display + buttons).  Shows splash screen, then auto-
	 * transitions to STATUS page.  Display sleeps after auto-off timeout;
	 * user button is the only wake source for repeater. */
	ui_init();
#if !IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	oled_sleep();
#endif

	/* Log environment sensor availability */
	if (env_sensors_available()) {
		LOG_INF("Environment sensors available");
	}

#ifdef ZEPHCORE_LORA
	repeater_mesh_ptr = &repeater_mesh;

	/* Initialize mesh event object BEFORE begin() — radio callbacks post events */
	k_event_init(&mesh_events);

	/* Set LoRa callbacks for event-driven packet processing */
	lora_radio.setRxCallback(lora_rx_callback, nullptr);
	lora_radio.setTxDoneCallback(lora_tx_done_callback, nullptr);
	repeater_mesh.setTxQueuedCallback(tx_queued_callback, nullptr);

	/* Load or generate identity BEFORE begin() */
	mesh::LocalIdentity self_identity;
	if (!data_store.loadIdentity(self_identity)) {
		LOG_INF("No identity found, generating new keypair...");
		self_identity = mesh::LocalIdentity(&zephyr_rng);
		/* Ensure pub_key[0] is not reserved (0x00 or 0xFF) */
		int count = 0;
		while (count < 10 && (self_identity.pub_key[0] == 0x00 || self_identity.pub_key[0] == 0xFF)) {
			self_identity = mesh::LocalIdentity(&zephyr_rng);
			count++;
		}
		data_store.saveIdentity(self_identity);
		LOG_INF("New identity saved");
	}
	repeater_mesh.self_id = self_identity;

	/* Log repeater ID (first 8 bytes of public key) */
	LOG_INF("Repeater ID: %02x%02x%02x%02x%02x%02x%02x%02x...",
		self_identity.pub_key[0], self_identity.pub_key[1],
		self_identity.pub_key[2], self_identity.pub_key[3],
		self_identity.pub_key[4], self_identity.pub_key[5],
		self_identity.pub_key[6], self_identity.pub_key[7]);

	/* Pre-load persisted prefs into radio_prefs BEFORE repeater_mesh.begin().
	 *
	 * Rationale: lora_radio was constructed at static-init time with a pointer
	 * to radio_prefs (see line ~286).  When repeater_mesh.begin() runs, it
	 * calls Mesh::begin() -> Dispatcher::begin() -> _radio->begin(), which
	 * reads freq/bw/sf/cr through that pointer to configure the hardware.
	 *
	 * Without this pre-load, the radio boots on the compile-time defaults
	 * from initNodePrefs() (freq=869.618, EU ISM band) regardless of what the
	 * user configured.  RepeaterMesh::begin() then loads the persisted prefs
	 * into its own _prefs member, so CLI/UI readback shows the correct saved
	 * values — but the hardware is already configured on the stale defaults
	 * and never gets reconfigured.  The result: device appears operational on
	 * the configured frequency but is physically tuned to 869.618 MHz, so
	 * transmissions are not heard and no packets can be received.
	 *
	 * This went unnoticed in the EU because 869.618 happens to match the
	 * default; US/CA users on 910.525 (and any other non-default freq) hit it.
	 *
	 * Mirrors the temp_prefs pattern in main_companion.cpp.  The subsequent
	 * setPrefs() rebind (after begin()) points the radio at the live prefs in
	 * RepeaterMesh so CLI `set radio` changes take effect on reconfigure(). */
	data_store.loadPrefs(radio_prefs);

	/* Start mesh with data store - this loads prefs, ACL, regions */
	repeater_mesh.begin(&data_store);

	/* Rebind radio prefs pointer to the live prefs inside repeater_mesh.
	 * radio_prefs above was a one-time copy for static init; from here on,
	 * the radio must read from the same struct RepeaterMesh mutates so that
	 * CLI-driven reconfigure() calls see current values. */
	lora_radio.setPrefs(repeater_mesh.getNodePrefs());

	/* Generate default node name from hardware device ID if not set */
	NodePrefs* prefs = repeater_mesh.getNodePrefs();
	if (strlen(prefs->node_name) == 0 || strcmp(prefs->node_name, "Repeater") == 0) {
		uint8_t dev_id[8];
		ssize_t id_len = hwinfo_get_device_id(dev_id, sizeof(dev_id));
		if (id_len >= 4) {
			snprintf(prefs->node_name, sizeof(prefs->node_name),
				 "Repeater-%02X%02X%02X%02X", dev_id[0], dev_id[1], dev_id[2], dev_id[3]);
		}
	}

	/* Apply RX boost and duty cycle from prefs */
	lora_radio.setRxBoost(prefs->rx_boost != 0);
	lora_radio.enableRxDutyCycle(prefs->rx_duty_cycle != 0);

	/* Feed initial UI state from loaded prefs */
	ui_set_node_name(prefs->node_name);
	ui_set_radio_params(
		(uint32_t)(prefs->freq * 1000000.0f + 0.5f),  /* MHz → Hz */
		prefs->sf,
		(uint16_t)(prefs->bw * 10.0f + 0.5f),         /* kHz → 0.1 kHz */
		prefs->cr,
		prefs->tx_power_dbm,
		lora_radio.getNoiseFloor());
	ui_set_battery(zephyr_board.getBattMilliVolts(), 0);
	ui_set_gps_available(gps_is_available());

	/* Defer initial advertisement by 10s — gives GPS time for a quick fix.
	 * Advert payload (including coords) is built when the work fires. */
	LOG_INF("Initial advertisement scheduled in 10s");
	k_work_schedule(&initial_advert_work, K_SECONDS(10));
#endif

	/* Initialize USB serial for CLI */
#if !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT) && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)
	zephcore_usbd_init();
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)
	usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
#else
	/* No CDC ACM (e.g. ESP32 usb_serial) — use chosen console UART */
	usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
	if (device_is_ready(usb_dev)) {
		LOG_INF("USB CDC device ready: %s", usb_dev->name);
		ring_buf_init(&usb_ring_buf, sizeof(usb_ring_buf_data), usb_ring_buf_data);

		/* Set up UART interrupt callback */
		uart_irq_callback_set(usb_dev, cli_uart_isr);
		uart_irq_rx_enable(usb_dev);
	} else {
		LOG_ERR("USB CDC device not ready");
		usb_dev = NULL;
	}

	/*
	 * Event-driven architecture: main thread runs repeater event loop.
	 * No BLE - all configuration via USB serial CLI.
	 */
#ifdef ZEPHCORE_LORA
	repeater_event_loop();  /* Never returns */
#else
	for (;;) {
		k_sleep(K_FOREVER);
	}
#endif

	return 0;
}
