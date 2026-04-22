/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore - CompanionMesh (Event-Driven)
 *
 * BLE stack is in adapters/ble/ZephyrBLE.cpp.
 * USB CDC transport is in adapters/usb/ZephyrUsbCompanion.cpp.
 * UI ↔ mesh actions are in helpers/ui/ui_mesh_actions.cpp.
 * This file handles: mesh event loop, LoRa, GPS, data storage, init.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_main, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <ZephyrDataStore.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/hwinfo.h>
#include <ZephyrSensorManager.h>
#include "ui_task.h"
#include "ui_mesh_actions.h"
#include "oled_power.h"
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_BUZZER)
#include "buzzer.h"
#endif

#include <ZephyrBLE.h>

#if IS_ENABLED(CONFIG_LOG)
#include <ZephyrCompanionUSB.h>
#endif

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <mesh/RadioIncludes.h>
#ifdef ZEPHCORE_LORA
#include <app/CompanionMesh.h>
#endif

/*
 * BLE Controller assert handler — without this, a controller assert
 * silently freezes the CPU at highest IRQ priority with zero log output.
 * This prints the file/line and reboots so we can actually see what happened.
 */
#if IS_ENABLED(CONFIG_BT_CTLR_ASSERT_HANDLER)
#include <zephyr/sys/reboot.h>
extern "C" void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	LOG_ERR("!!! BLE CONTROLLER ASSERT: %s:%u !!!", file ? file : "?", line);
	/* Force flush logs before reboot */
	k_sleep(K_MSEC(100));
	sys_reboot(SYS_REBOOT_COLD);
}
#endif

/*
 * Event-driven mesh loop - replaces 50ms polling with true event signaling.
 * Events are signaled from ISR/callbacks, mesh loop wakes immediately.
 * NO periodic timer - CPU sleeps until actual events occur.
 */
#define MESH_EVENT_LORA_RX       BIT(0)  /* LoRa packet received */
#define MESH_EVENT_LORA_TX_DONE  BIT(1)  /* LoRa TX complete (event-driven!) */
#define MESH_EVENT_BLE_RX        BIT(2)  /* BLE frame received */
#define MESH_EVENT_HOUSEKEEPING  BIT(3)  /* Periodic housekeeping (noise floor, etc.) */
#define MESH_EVENT_UI_ACTION     BIT(4)  /* Button action from UI (deferred to mesh thread) */
#define MESH_EVENT_GPS_ACTION    BIT(5)  /* GPS state change (must run on main thread!) */
#define MESH_EVENT_TX_DRAIN      BIT(6)  /* Outbound packet delay expired, run checkSend */
#define MESH_EVENT_ALL           (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | MESH_EVENT_BLE_RX | MESH_EVENT_HOUSEKEEPING | MESH_EVENT_UI_ACTION | MESH_EVENT_GPS_ACTION | MESH_EVENT_TX_DRAIN)

/* Housekeeping interval - infrequent to preserve power savings */
#define HOUSEKEEPING_INTERVAL_MS CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS

/* Event-driven mesh loop - k_event for signaling from ISR/callbacks */
static struct k_event mesh_events;

/* Work items for event-driven processing */
static void rx_process_work_fn(struct k_work *work);
static void contact_iter_work_fn(struct k_work *work);
static void housekeeping_timer_fn(struct k_timer *timer);

K_WORK_DEFINE(rx_process_work, rx_process_work_fn);
K_WORK_DEFINE(contact_iter_work, contact_iter_work_fn);

/* Housekeeping timer for periodic tasks (noise floor calibration, etc.)
 * Fires every 5 seconds to wake event loop for maintenance without
 * compromising event-driven power savings. */
K_TIMER_DEFINE(housekeeping_timer, housekeeping_timer_fn, NULL);

/* Forward declarations */
#ifdef ZEPHCORE_LORA
static CompanionMesh *companion_mesh_ptr;
#endif

/* ========== BLE callbacks → main ========== */

/* BLE RX callback — called from BLE adapter when a frame arrives.
 * Queues to recv_queue and wakes mesh event loop. */
static void ble_on_rx_frame(const uint8_t *data, uint16_t len)
{
	struct {
		uint16_t len;
		uint8_t buf[MAX_FRAME_SIZE];
	} f;

	if (len == 0 || len > MAX_FRAME_SIZE) {
		return;
	}

	f.len = len;
	memcpy(f.buf, data, len);

	if (k_msgq_put(zephcore_ble_get_recv_queue(), &f, K_NO_WAIT) != 0) {
		LOG_WRN("recv queue full");
		return;
	}

	/* Trigger RX processing and signal mesh event */
	k_work_submit(&rx_process_work);
	k_event_post(&mesh_events, MESH_EVENT_BLE_RX);
}

/* BLE TX idle callback — called when TX queue is empty.
 * Continues contact iteration. */
static void ble_on_tx_idle(void)
{
	k_work_submit(&contact_iter_work);
}

/* BLE connected callback — notify UI, clear USB state if needed */
static void ble_on_connected(void)
{
	ui_notify(UI_EVENT_BLE_CONNECTED);
}

/* BLE disconnected callback — clean up state and notify UI */
static void ble_on_disconnected(void)
{
#if IS_ENABLED(CONFIG_LOG)
	zephcore_usb_companion_reset_rx();
#endif
#ifdef ZEPHCORE_LORA
	/* Silently cancel any in-progress contact iteration.
	 * Without this, reconnect triggers resetContactIterator() which sends
	 * a stale PACKET_CONTACT_END to the NEW connection, confusing the
	 * phone's sync state machine. */
	companion_mesh_ptr->cancelContactIterator();
	/* Reset message sync — un-ACKed peeked message stays in queue
	 * and will be re-sent on next CMD_SYNC_NEXT_MESSAGE. */
	companion_mesh_ptr->cancelSyncPending();
	/* Free Ed25519 sign buffer if allocated mid-operation */
	companion_mesh_ptr->cleanupSignState();
#endif
	ui_notify(UI_EVENT_BLE_DISCONNECTED);
}

static const struct ble_callbacks ble_cbs = {
	.on_rx_frame = ble_on_rx_frame,
	.on_tx_idle = ble_on_tx_idle,
	.on_connected = ble_on_connected,
	.on_disconnected = ble_on_disconnected,
};

/* ========== Frame send/receive (mesh ↔ BLE/USB) ========== */

static size_t write_frame(const uint8_t *src, size_t len)
{
	return zephcore_ble_send(src, (uint16_t)len);
}

/**
 * Push notification callback - sends push frames to BLE.
 * Push frames have format: [push_code] [data...]
 */
static void push_callback(uint8_t code, const uint8_t *data, size_t len)
{
	/* No point serializing if nobody is listening */
	if (!zephcore_ble_is_connected()) return;

	/* Push frame: code byte + optional data */
	uint8_t push_buf[1 + MAX_FRAME_SIZE - 1];
	size_t total_len = 1 + len;

	if (total_len > MAX_FRAME_SIZE) {
		LOG_WRN("data too long %u", (unsigned)len);
		total_len = MAX_FRAME_SIZE;
		len = MAX_FRAME_SIZE - 1;
	}

	push_buf[0] = code;
	if (len > 0 && data != nullptr) {
		memcpy(&push_buf[1], data, len);
	}

	LOG_DBG("code=0x%02x len=%u (total frame)", code, (unsigned)total_len);
	write_frame(push_buf, total_len);
}

/* RX processing work - handles received BLE/USB frames */
static void rx_process_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	struct {
		uint16_t len;
		uint8_t buf[MAX_FRAME_SIZE];
	} f;

	/* Process all queued frames */
	while (k_msgq_get(zephcore_ble_get_recv_queue(), &f, K_NO_WAIT) == 0) {
#ifdef ZEPHCORE_LORA
		/* handleProtocolFrame() resets the contact iterator internally
		 * (line 1229) for any non-CMD_GET_CONTACTS command — no need
		 * to call resetContactIterator() here.  Doing so sent a stale
		 * PACKET_CONTACT_END before the command was even processed. */
		if (!companion_mesh_ptr->handleProtocolFrame(f.buf, f.len)) {
			LOG_DBG("rx_process: unknown cmd 0x%02x len=%u", f.buf[0], (unsigned)f.len);
			uint8_t err_rsp[] = { 0x01, 0x01 };  /* PACKET_ERROR, ERR_UNSUPPORTED */
			write_frame(err_rsp, sizeof(err_rsp));
		}
#endif
	}
}

/* Contact iteration work - runs when TX queue has space */
static void contact_iter_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
#ifdef ZEPHCORE_LORA
	/* Don't iterate while BLE TX is congested — wait for drain to clear.
	 * Also check 2/3 high-water mark (catches stray frames before full). */
	if (zephcore_ble_is_congested()) {
		return;
	}

	uint32_t used = k_msgq_num_used_get(zephcore_ble_get_send_queue());
	uint32_t queue_size = CONFIG_ZEPHCORE_BLE_QUEUE_SIZE;
	bool has_space = (used < (queue_size * 2 / 3));

	if (has_space && companion_mesh_ptr) {
		if (companion_mesh_ptr->continueContactIteration()) {
			/* More contacts to send - kick TX drain which will re-trigger us */
			zephcore_ble_kick_tx();
		}
	}
#endif
}

/*
 * Event-driven mesh loop - runs in main thread context.
 * Wakes on actual events: LoRa RX, LoRa TX done, BLE RX.
 * Plus a 5-second housekeeping timer for noise floor calibration, etc.
 */
static void mesh_event_loop(void)
{
	LOG_INF("starting event-driven loop");

	/* Start housekeeping timer for periodic maintenance tasks */
	k_timer_start(&housekeeping_timer, K_MSEC(HOUSEKEEPING_INTERVAL_MS),
		      K_MSEC(HOUSEKEEPING_INTERVAL_MS));

	for (;;) {
		/* Wait for any mesh event - blocks until signaled */
		uint32_t events = k_event_wait(&mesh_events, MESH_EVENT_ALL, false, K_FOREVER);

		/* Clear the events we're handling */
		k_event_clear(&mesh_events, events);

#ifdef ZEPHCORE_LORA
		/* Handle deferred UI button actions (flood advert, pref saves).
		 * Must run in this thread — LoRa TX and flash writes block. */
		if (events & MESH_EVENT_UI_ACTION) {
			mesh_handle_ui_actions();
		}

		/* Handle deferred GPS state transitions (wake/timeout).
		 * MUST run here (main thread) because GNSS configuration uses
		 * modem_chat_run_script() which blocks on a semaphore signaled
		 * from the system work queue. Running it on the system work
		 * queue would deadlock. */
		if (events & MESH_EVENT_GPS_ACTION) {
			gps_process_event();
		}

		/* Packet processing — only on radio/BLE/TX events */
		if (companion_mesh_ptr &&
		    (events & (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE |
			       MESH_EVENT_BLE_RX | MESH_EVENT_TX_DRAIN))) {
			companion_mesh_ptr->loop();
		}

		/* Periodic housekeeping — maintenance + UI refresh */
		if (events & MESH_EVENT_HOUSEKEEPING) {
			/* Radio maintenance: noise floor calibration, AGC reset,
			 * RX watchdog.  Separated from loop() so these never run
			 * on packet-driven events. */
			if (companion_mesh_ptr) {
				companion_mesh_ptr->maintenanceLoop();
			}

			mesh_housekeeping_ui_refresh();
		}
#endif
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
	/* Signal event to wake mesh loop immediately */
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

/* LoRa TX complete callback - called from work queue when async TX finishes */
static void lora_tx_done_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	/* Signal event to wake mesh loop for TX completion handling */
	k_event_post(&mesh_events, MESH_EVENT_LORA_TX_DONE);
}

/* TX drain — Dispatcher queued a packet with a delay.
 * Schedule a precise wake so checkSend runs when the delay expires. */
static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_event_post(&mesh_events, MESH_EVENT_TX_DRAIN);
}

static K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);

static void tx_queued_callback(uint32_t delay_ms, void *user_data)
{
	ARG_UNUSED(user_data);
	k_work_reschedule(&tx_drain_work, K_MSEC(delay_ms));
}
#endif

static mesh::ZephyrRTCClock rtc_clock;
static ZephyrDataStore data_store(rtc_clock);

#ifdef ZEPHCORE_LORA
static mesh::ZephyrBoard zephyr_board;

/* NodePrefs placeholder - will be set from CompanionMesh */
static NodePrefs temp_prefs;

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
/* LR1110 via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LR1110Radio lora_radio(lora_dev, zephyr_board, &temp_prefs);
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR2021)
/* LR2021 via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LR2021Radio lora_radio(lora_dev, zephyr_board, &temp_prefs);
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_SX127X)
/* SX127x via Zephyr loramac-node driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX127xRadio lora_radio(lora_dev, zephyr_board, &temp_prefs);
#else
/* SX126x via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX126xRadio lora_radio(lora_dev, zephyr_board, &temp_prefs);
#endif

static uint16_t get_battery_mv(void)
{
	return zephyr_board.getBattMilliVolts();
}

static void radio_reconfigure(void)
{
	lora_radio.reconfigure();
}

static mesh::ZephyrMillisecondClock ms_clock;
static mesh::ZephyrRNG zephyr_rng;
static mesh::SimpleMeshTables mesh_tables;
static mesh::StaticPoolPacketManager packet_mgr;
static CompanionMesh companion_mesh(lora_radio, ms_clock, zephyr_rng, rtc_clock,
	packet_mgr, mesh_tables, data_store);
#endif

/* GPS enable callback - logs state changes
 * Note: GPS persistence is handled in CompanionMesh.cpp CMD_SET_CUSTOM_VAR handler */
static void gps_enable_callback(bool enabled)
{
	LOG_INF("GPS %s", enabled ? "enabled" : "disabled");
	ui_set_gps_enabled(enabled);
}

/* GPS event callback - called when GPS work handlers need the main thread
 * to process a state transition (wake from standby, timeout, etc.).
 * Runs from system work queue context — just posts an event, no blocking. */
static void gps_event_callback(void)
{
	k_event_post(&mesh_events, MESH_EVENT_GPS_ACTION);
}

/* GPS fix callback - called when GPS acquires a valid fix.
 * Updates mesh node position and RTC. */
static void gps_fix_callback(double lat, double lon, int64_t utc_time)
{
	/* Sync RTC from GPS time */
	if (utc_time > 0) {
		LOG_INF("GPS fix: RTC sync time=%lld", utc_time);
		rtc_clock.setCurrentTime((uint32_t)utc_time);
	}

#ifdef ZEPHCORE_LORA
	/* Update node position for mesh advertising */
	if (lat != companion_mesh.prefs.node_lat || lon != companion_mesh.prefs.node_lon) {
		companion_mesh.prefs.node_lat = lat;
		companion_mesh.prefs.node_lon = lon;
		/* Log as integer degrees (Zephyr LOG doesn't support %f by default) */
		int lat_deg = (int)lat;
		int lon_deg = (int)lon;
		int lat_frac = (int)((lat - lat_deg) * 1000000);
		int lon_frac = (int)((lon - lon_deg) * 1000000);
		if (lat_frac < 0) lat_frac = -lat_frac;
		if (lon_frac < 0) lon_frac = -lon_frac;
		LOG_INF("GPS fix: position updated lat=%d.%06d lon=%d.%06d",
			lat_deg, lat_frac, lon_deg, lon_frac);
		/* Save to storage */
		data_store.savePrefs(companion_mesh.prefs);
	}

	/* Update UI with GPS data */
	{
		struct gps_position gpos;

		gps_get_position(&gpos);
		int32_t lat_mdeg = (int32_t)(lat * 1000.0);
		int32_t lon_mdeg = (int32_t)(lon * 1000.0);

		ui_set_gps_data(true, (uint8_t)gpos.satellites,
				lat_mdeg, lon_mdeg, gpos.altitude_mm);
	}
#else
	ARG_UNUSED(lat);
	ARG_UNUSED(lon);
#endif
}

/* bt_ready callback — BLE stack is up, start advertising */
static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return;
	}

#ifdef ZEPHCORE_LORA
	zephcore_ble_start(companion_mesh.getDeviceName());
#else
	zephcore_ble_start(NULL);
#endif
}

int main(void)
{
	/* Clear any stale bootloader magic from previous sessions */
	zephyr_board.clearBootloaderMagic();

	/* Brief settle — deferred logging will catch up once USB CDC enumerates */
	k_sleep(K_MSEC(100));
	LOG_INF("=== ZephCore starting ===");

	/* Log reset reason so we can diagnose random reboots */
	{
		uint32_t cause;
		if (hwinfo_get_reset_cause(&cause) == 0) {
			LOG_INF("Reset cause: 0x%08x%s%s%s%s%s%s", cause,
				(cause & RESET_PIN)       ? " PIN"       : "",
				(cause & RESET_SOFTWARE)  ? " SOFTWARE"  : "",
				(cause & RESET_BROWNOUT)  ? " BROWNOUT"  : "",
				(cause & RESET_POR)       ? " POR"       : "",
				(cause & RESET_WATCHDOG)  ? " WATCHDOG"  : "",
				(cause & RESET_CPU_LOCKUP)? " LOCKUP"    : "");
			hwinfo_clear_reset_cause();
		}
	}

	if (!ZephyrDataStore::mount()) {
		LOG_ERR("LittleFS mount failed");
	}
	data_store.begin();

	/* Initialize sensor manager (GPS, environment sensors) */
	sensor_manager_init();

	/* Initialize UI subsystem (buttons, buzzer, display).
	 * If display is present, ui_init() handles display init + auto-off.
	 * If no display, fall back to raw OLED sleep for power saving. */
	ui_init();
#if !IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	oled_sleep();
#endif

	/* Register GPS callbacks (GPS state restored from prefs later) */
	if (gps_is_available()) {
		LOG_INF("GPS available");
		gps_set_enable_callback(gps_enable_callback);
		gps_set_fix_callback(gps_fix_callback);
		gps_set_event_callback(gps_event_callback);
	}

	/* Log environment sensor availability */
	if (env_sensors_available()) {
		LOG_INF("Environment sensors available");
	}

#ifdef ZEPHCORE_LORA
	/* Initialize prefs with defaults */
	memset(&companion_mesh.prefs, 0, sizeof(companion_mesh.prefs));
	companion_mesh.prefs.freq = 869.618f;
	companion_mesh.prefs.bw = 62.5f;
	companion_mesh.prefs.sf = 8;
	companion_mesh.prefs.cr = 8;
	companion_mesh.prefs.tx_power_dbm = 22;
	companion_mesh.prefs.rx_delay_base = 0.0f;  /* Disabled for companion */
	companion_mesh.prefs.airtime_factor = 10.0f; /* 10% duty cycle (EU 868 default) */
	companion_mesh.prefs.rx_duty_cycle = 1;     /* Companions: duty cycle ON by default (power save) */
	companion_mesh.prefs.rx_boost = 1;          /* Default: boosted RX (+3dB sensitivity, +2mA) */
	companion_mesh.prefs.apc_enabled = 0;       /* Default: APC off */
	companion_mesh.prefs.apc_margin = 20;       /* Companions: more conservative margin (mobile) */

	/* Load prefs from storage */
	data_store.loadPrefs(companion_mesh.prefs);

	/* Apply saved BLE PIN (0 = use Kconfig default) */
	if (companion_mesh.prefs.ble_pin >= 100000 && companion_mesh.prefs.ble_pin <= 999999) {
		zephcore_ble_set_passkey(companion_mesh.prefs.ble_pin);
		LOG_INF("BLE passkey loaded from prefs: %06u", companion_mesh.prefs.ble_pin);
	}

	/* Copy prefs to temp_prefs for radio (radio was initialized before companion_mesh) */
	temp_prefs = companion_mesh.prefs;

	/* Load contacts and channels */
	data_store.loadContacts(&companion_mesh);
	data_store.loadChannels(&companion_mesh);

	/* Add Public channel if no channels loaded (first boot) */
	if (companion_mesh.getNumChannels() == 0) {
		// Public channel PSK: "izOH6cXN6mrJ5e26oRXNcg==" decoded
		static const uint8_t public_psk[16] = {
			0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
			0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72
		};
		companion_mesh.addChannel("Public", public_psk, 16);
		data_store.saveChannels(&companion_mesh);
		LOG_INF("Added default Public channel");
	}

	/* Load or generate identity */
	mesh::LocalIdentity self_identity;
	if (!data_store.loadMainIdentity(self_identity)) {
		self_identity = mesh::LocalIdentity(&zephyr_rng);
		/* Ensure pub_key[0] is not reserved (0x00 or 0xFF in MeshCore protocol) */
		int count = 0;
		while (count < 10 && (self_identity.pub_key[0] == 0x00 || self_identity.pub_key[0] == 0xFF)) {
			self_identity = mesh::LocalIdentity(&zephyr_rng);
			count++;
		}
		data_store.saveMainIdentity(self_identity);
	}
	companion_mesh.self_id = self_identity;

	/* Set default node name from first 4 bytes of public key if not set.
	 * Must happen after identity load so pub_key is available. */
	if (companion_mesh.prefs.node_name[0] == '\0') {
		snprintf(companion_mesh.prefs.node_name, sizeof(companion_mesh.prefs.node_name),
			 "%02X%02X%02X%02X",
			 self_identity.pub_key[0], self_identity.pub_key[1],
			 self_identity.pub_key[2], self_identity.pub_key[3]);
	}

	/* Set callbacks */
	companion_mesh.setWriteFrameCallback(write_frame);
	companion_mesh.setPushCallback(push_callback);
	companion_mesh.setBatteryCallback(get_battery_mv);
	companion_mesh.setRadioReconfigureCallback(radio_reconfigure);
	companion_mesh.setPinChangeCallback([](uint32_t new_pin) {
		zephcore_ble_set_passkey(new_pin);
	});
	companion_mesh_ptr = &companion_mesh;

	/* Set LoRa callbacks for event-driven packet processing */
	lora_radio.setRxCallback(lora_rx_callback, nullptr);
	lora_radio.setTxDoneCallback(lora_tx_done_callback, nullptr);
	companion_mesh.setTxQueuedCallback(tx_queued_callback, nullptr);

	/* Start mesh */
	companion_mesh.begin();

	/* Redirect radio prefs pointer to the live companion_mesh.prefs.
	 * The radio was constructed with &temp_prefs (a one-time copy needed
	 * for static init).  After begin(), point at the real prefs so that
	 * CMD_SET_RADIO_PARAMS changes are visible to lora_radio.reconfigure(). */
	lora_radio.setPrefs(&companion_mesh.prefs);

	/* Push initial state to UI display */
	ui_set_node_name(companion_mesh.prefs.node_name);
	ui_set_radio_params(
		(uint32_t)(companion_mesh.prefs.freq * 1000000.0f + 0.5f),
		companion_mesh.prefs.sf,
		(uint16_t)(companion_mesh.prefs.bw * 10.0f + 0.5f),
		companion_mesh.prefs.cr,
		companion_mesh.prefs.tx_power_dbm,
		lora_radio.getNoiseFloor());
	ui_set_battery(zephyr_board.getBattMilliVolts(), 0);
	ui_set_gps_available(gps_is_available());
	ui_set_gps_enabled(companion_mesh.prefs.gps_enabled != 0);
	ui_set_ble_enabled(true);  /* BLE starts advertising at boot */

	/* Restore offgrid mode (client repeat) state from persisted prefs */
	ui_set_offgrid_mode(companion_mesh.prefs.client_repeat != 0);
	LOG_INF("offgrid mode: %s (from prefs)",
		companion_mesh.prefs.client_repeat ? "on" : "off");

	/* Restore buzzer mute state from persisted prefs, then play
	 * startup chime only if buzzer is not muted.
	 * buzzer_init() defaults to quiet=true, so we must always
	 * call buzzer_set_quiet() to apply the saved preference. */
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_BUZZER)
	buzzer_set_quiet(companion_mesh.prefs.buzzer_quiet);
	ui_set_buzzer_quiet(companion_mesh.prefs.buzzer_quiet);
	LOG_INF("buzzer: quiet=%s (from prefs)",
		companion_mesh.prefs.buzzer_quiet ? "true" : "false");
	ui_play_startup_chime();
#endif

	/* Restore LED enabled/disabled state from persisted prefs.
	 * If LEDs were disabled, stop the heartbeat LED cycle. */
	bool leds_off = companion_mesh.prefs.leds_disabled != 0;
	ui_set_leds_disabled(leds_off);
	ui_set_heartbeat_led(!leds_off);
	LOG_INF("LEDs: %s (from prefs)", leds_off ? "disabled" : "enabled");

	/* Restore GPS state from persisted prefs.
	 * GPS hardware is powered at boot (bootloader/pull-up).
	 * First, ensure power state matches prefs (powers off if disabled).
	 * Then, if prefs say enabled, start the GPS state machine. */
	if (gps_is_available()) {
		/* Explicitly set power state at boot (handles disabled case) */
		gps_ensure_power_state(companion_mesh.prefs.gps_enabled);

		if (companion_mesh.prefs.gps_enabled) {
			LOG_INF("GPS: Restoring enabled state from prefs");
			gps_enable(true);
		}
	}

	/* Apply RX boost and duty cycle from prefs */
	lora_radio.setRxBoost(companion_mesh.prefs.rx_boost != 0);
	lora_radio.enableRxDutyCycle(companion_mesh.prefs.rx_duty_cycle != 0);

	/* Restore runtime ADC multiplier override (0 = keep DT default) */
	zephyr_board.setAdcMultiplier(companion_mesh.prefs.adc_multiplier);

	/* Initialize mesh event object */
	k_event_init(&mesh_events);

	/* Initialize UI mesh actions module (pass mesh objects for deferred actions) */
	ui_mesh_actions_init(&mesh_events, MESH_EVENT_UI_ACTION,
			     &companion_mesh, &data_store,
			     &lora_radio, &zephyr_board, &rtc_clock);
#endif

	/* Initialize BLE adapter (registers auth callbacks) */
	zephcore_ble_init(&ble_cbs);

	/*
	 * USB CDC ACM - only enabled when logging is enabled (debug builds).
	 * Production builds (CONFIG_LOG=n) skip USB entirely, saving ~2-5mA.
	 */
#if IS_ENABLED(CONFIG_LOG)
	zephcore_usb_companion_init(&mesh_events, &rx_process_work, MESH_EVENT_BLE_RX,
				   &zephyr_board);
#endif

	if (bt_enable(bt_ready) != 0) {
		LOG_ERR("bt_enable failed");
	}

	/*
	 * FULLY EVENT-DRIVEN architecture: main thread runs mesh event loop.
	 * Mesh loop wakes IMMEDIATELY on events - NO POLLING for TX completion!
	 *
	 * Event sources:
	 *   - MESH_EVENT_LORA_RX: LoRa packet received (from RX async callback)
	 *   - MESH_EVENT_LORA_TX_DONE: LoRa TX complete (from TX poll work -> callback)
	 *   - MESH_EVENT_BLE_RX: BLE frame received (from NUS write)
	 *   - MESH_EVENT_HOUSEKEEPING: Periodic maintenance (noise floor, etc.)
	 *
	 * Other processing still uses work queues:
	 *   - BLE TX: write_frame -> tx_drain_work (event-driven via notify callback)
	 *   - Contact iteration: contact_iter_work (on TX queue space)
	 *
	 * TX completion is now event-driven: radio polls at 5ms intervals internally
	 * and fires callback when done. Much better than 100ms main loop polling!
	 */
#ifdef ZEPHCORE_LORA
	mesh_event_loop();  /* Never returns */
#else
	for (;;) {
		k_sleep(K_FOREVER);
	}
#endif

	return 0;
}
