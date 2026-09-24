/*
 * SPDX-License-Identifier: MIT
 * ZephCore companion: composition root and mesh event loop.
 *
 * Transports: adapters/ble/ZephyrBLE.cpp, adapters/usb/ZephyrCompanionUSB.cpp.
 * UI to mesh actions: helpers/ui/ui_mesh_actions.cpp.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_main, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <ZephyrDataStore.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <adapters/clock/ZephyrRTCDiscover.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/hwinfo.h>
#include <helpers/boot_info.h>
#include <zephyr/sys/reboot.h>
#include <ZephyrSensorManager.h>
#include <helpers/time_sync.h>
#include "ui_task.h"
#include "ui_mesh_actions.h"
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
#include "display.h"
#endif
#include "oled_power.h"
#include "led_gate.h"
#include "buzzer_gate.h"
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_BUZZER)
#include "buzzer.h"
#endif

#include <ZephyrBLE.h>

/* The wired-companion stack (ZephyrCompanionUSB) is compiled when either
 * logging needs the CDC console (debug builds), the native-USB companion is
 * enabled (CONFIG_ZEPHCORE_COMPANION_USB, default-y on USB-capable boards), or
 * the plain-UART companion is enabled (CONFIG_ZEPHCORE_COMPANION_SERIAL, for
 * USB-UART-bridge / no-USB boards).  All three share the same frame parser,
 * TX ring, CLI, and BLE arbitration — only the byte backend differs. */
#define ZEPHCORE_USB_STACK \
	(IS_ENABLED(CONFIG_LOG) || IS_ENABLED(CONFIG_ZEPHCORE_COMPANION_USB) || \
	 IS_ENABLED(CONFIG_ZEPHCORE_COMPANION_SERIAL))

#if ZEPHCORE_USB_STACK
#include <ZephyrCompanionUSB.h>
#include <ZephyrUSBCDC.h>
#endif

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
#include <joystick_ui_task.h>
#include <joystick_ui_hooks.h>
#endif

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <src/RadioIncludes.h>
#ifdef ZEPHCORE_LORA
#include <app/CompanionMesh.h>
#include <helpers/CommonCLI.h>
#include <helpers/ClientACL.h>
#include <helpers/StatsFormatHelper.h>
#include <helpers/battery_curve.h>
#endif

/* Without this a BLE controller assert freezes the CPU at top IRQ priority
 * with no output; log the location and reboot. */
#if IS_ENABLED(CONFIG_BT_CTLR_ASSERT_HANDLER)
extern "C" void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	LOG_ERR("!!! BLE CONTROLLER ASSERT: %s:%u !!!", file ? file : "?", line);
	k_sleep(K_MSEC(100));  /* let the log drain */
	sys_reboot(SYS_REBOOT_COLD);
}
#endif

/* The mesh thread blocks on these; ISRs and callbacks post them. */
#define MESH_EVENT_LORA_RX       BIT(0)  /* LoRa packet received */
#define MESH_EVENT_LORA_TX_DONE  BIT(1)  /* LoRa TX complete */
#define MESH_EVENT_BLE_RX        BIT(2)  /* BLE/USB frame received */
#define MESH_EVENT_HOUSEKEEPING  BIT(3)  /* Periodic housekeeping (noise floor, etc.) */
#define MESH_EVENT_UI_ACTION     BIT(4)  /* Button action from UI (deferred to mesh thread) */
#define MESH_EVENT_GPS_ACTION    BIT(5)  /* GPS state change (main thread only) */
#define MESH_EVENT_TX_DRAIN      BIT(6)  /* Outbound packet delay expired, run checkSend */
#define MESH_EVENT_PREFS_DIRTY   BIT(8)  /* Prefs mutated off-main; main flushes to flash */
#define MESH_EVENT_RTC_SAVE      BIT(9)  /* Hardware-RTC write requested off-main */
#define MESH_EVENT_CONTACT_ITER  BIT(10) /* Continue contact-dump iteration on main thread */

#ifdef ZEPHCORE_LORA
static void save_prefs_to_flash(void);
static void vcontact_battery_alert_check(void);
#endif

/* Epoch for a deferred hardware-RTC write. gps_fix_callback runs on the GNSS
 * modem_chat thread, where several ms of blocking I2C would stall NMEA ingest,
 * so the main thread does the write (MESH_EVENT_RTC_SAVE); posts coalesce to
 * the latest time. */
static atomic_t pending_rtc_epoch = ATOMIC_INIT(0);

#define MESH_EVENT_BASE          (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | \
	MESH_EVENT_BLE_RX | MESH_EVENT_HOUSEKEEPING | MESH_EVENT_UI_ACTION |  \
	MESH_EVENT_GPS_ACTION | MESH_EVENT_TX_DRAIN | MESH_EVENT_PREFS_DIRTY | \
	MESH_EVENT_RTC_SAVE | MESH_EVENT_CONTACT_ITER)
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
#define MESH_EVENT_JOYSTICK_LOOP BIT(7)  /* Joystick UI loop tick (50 ms) */
#define MESH_EVENT_ALL           (MESH_EVENT_BASE | MESH_EVENT_JOYSTICK_LOOP)
#else
#define MESH_EVENT_ALL           MESH_EVENT_BASE
#endif

#define HOUSEKEEPING_INTERVAL_MS CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS

static struct k_event mesh_events;

/* See pending_rtc_epoch. */
static void request_rtc_save(uint32_t epoch)
{
	atomic_set(&pending_rtc_epoch, (atomic_val_t)epoch);
	k_event_post(&mesh_events, MESH_EVENT_RTC_SAVE);
}

static void process_companion_rx(void);   /* runs on MAIN thread (see ble_on_rx_frame) */
static void run_contact_iteration(void);  /* runs on MAIN thread (see MESH_EVENT_CONTACT_ITER) */
static void housekeeping_timer_fn(struct k_timer *timer);
#if ZEPHCORE_USB_STACK
static void companion_cli_run(const char *line);  /* main-thread text-CLI exec */
#endif

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
static JoystickUITask joystick_ui_task;

static void joystick_signal_refresh(void)
{
	k_event_post(&mesh_events, MESH_EVENT_JOYSTICK_LOOP);
}

static void joystick_signal_tx(void)
{
	k_event_post(&mesh_events, MESH_EVENT_TX_DRAIN);
}
#endif

/* Threading rule for everything below: the mesh state (packet pool,
 * dispatcher, contact table) belongs to the MAIN thread. Transport callbacks
 * run on sysworkq or in ISR context, so they only queue and post an event;
 * the work happens in the event loop. Breaking this caused the stuck
 * "Sending..." bug (a reply parsed on sysworkq raced loop()). */

#if ZEPHCORE_USB_STACK
/* Completed USB text-CLI lines, drained on MESH_EVENT_BLE_RX. */
#define CLI_LINE_BUF_SIZE 256
struct companion_cli_line { char buf[CLI_LINE_BUF_SIZE]; };
K_MSGQ_DEFINE(companion_cli_queue, sizeof(struct companion_cli_line), 4, 4);
#endif

#if ZEPHCORE_USB_STACK
/* USB TX ring drained (CDC TX interrupt context): resume the contact dump. */
static void usb_on_tx_drain(void)
{
	k_event_post(&mesh_events, MESH_EVENT_CONTACT_ITER);
}
#endif

K_TIMER_DEFINE(housekeeping_timer, housekeeping_timer_fn, NULL);

#ifdef ZEPHCORE_LORA
static CompanionMesh *companion_mesh_ptr;
#endif

/* ========== BLE callbacks → main ========== */

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

	k_event_post(&mesh_events, MESH_EVENT_BLE_RX);
}

/* BLE TX queue empty: resume the contact dump and the TX drain. */
static void ble_on_tx_idle(void)
{
	k_event_post(&mesh_events, MESH_EVENT_CONTACT_ITER);
	k_event_post(&mesh_events, MESH_EVENT_TX_DRAIN);
}

static void ble_on_connected(void)
{
	ui_notify(UI_EVENT_BLE_CONNECTED);
}

/* One deferred reply is enough: RX pauses until the transport has accepted
 * it. A msgq also lets the disconnect path purge it. */
struct pending_reply_frame {
	uint16_t len;
	uint8_t buf[MAX_FRAME_SIZE];
};
K_MSGQ_DEFINE(pending_reply, sizeof(struct pending_reply_frame), 1, 4);

/* Session end (BLE disconnect or USB close): drop the contact dump (a stale
 * PACKET_CONTACT_END would confuse the next session), keep the un-ACKed
 * message queued for re-send, and free an abandoned sign buffer. */
static void companion_session_cleanup(void)
{
	k_msgq_purge(&pending_reply);
#ifdef ZEPHCORE_LORA
	companion_mesh_ptr->cancelContactIterator();
	companion_mesh_ptr->cancelSyncPending();
	companion_mesh_ptr->cleanupSignState();
#endif
}

#if ZEPHCORE_USB_STACK
/* A USB session shows as "connected" too, as upstream does for serial. */
static void usb_on_session_start(void)
{
	ui_notify(UI_EVENT_BLE_CONNECTED);
}

static void usb_on_session_end(void)
{
	companion_session_cleanup();
	ui_notify(UI_EVENT_BLE_DISCONNECTED);
}
#endif

static void ble_on_disconnected(void)
{
#if ZEPHCORE_USB_STACK
	zephcore_usb_companion_reset_rx();
#endif
	companion_session_cleanup();
	ui_notify(UI_EVENT_BLE_DISCONNECTED);
}

#if IS_ENABLED(CONFIG_ZEPHCORE_BLE_DFU)
static void ble_on_dfu_request(void)
{
	mesh_reboot_to_ota_dfu();  /* GPREGRET 0xA8 + reset; never returns */
}
#endif

static const struct ble_callbacks ble_cbs = {
	.on_rx_frame = ble_on_rx_frame,
	.on_tx_idle = ble_on_tx_idle,
	.on_connected = ble_on_connected,
	.on_disconnected = ble_on_disconnected,
#if IS_ENABLED(CONFIG_ZEPHCORE_BLE_DFU)
	.on_dfu_request = ble_on_dfu_request,
#endif
};

/* ========== Frame send/receive (mesh ↔ BLE/USB) ========== */

static size_t transport_write_frame(const uint8_t *src, size_t len)
{
#if ZEPHCORE_USB_STACK
	/* While USB owns the interface the BLE queue is never drained. */
	if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
		/* A text-CLI session gets no binary frames; report success so the
		 * caller does not retry one we dropped on purpose. */
		if (zephcore_usb_companion_is_text_session()) {
			return len;
		}
		return zephcore_usb_companion_write_frame(src, len);
	}
#endif
	return zephcore_ble_send(src, (uint16_t)len);
}

static size_t write_frame(const uint8_t *src, size_t len)
{
	if (len == 0 || len > MAX_FRAME_SIZE ||
	    k_msgq_num_used_get(&pending_reply) != 0) {
		return 0;
	}
	if (transport_write_frame(src, len) == len) {
		return len;
	}
	/* Only replies are retained (pushes stay lossy). Success once we hold a
	 * copy, so retrying callers (contact/message sync) do not send it twice. */
	if (src[0] < 0x80) {
		struct pending_reply_frame f;
		f.len = len;
		memcpy(f.buf, src, len);
		if (k_msgq_put(&pending_reply, &f, K_NO_WAIT) == 0) {
			return len;
		}
	}
	return 0;
}

static void retry_pending_reply(void)
{
	struct pending_reply_frame f;
	if (k_msgq_peek(&pending_reply, &f) == 0 &&
	    transport_write_frame(f.buf, f.len) == f.len) {
		(void)k_msgq_get(&pending_reply, &f, K_NO_WAIT);
	}
}

/* A binary companion client is listening: BLE, or a USB session that is not
 * the text CLI (zephcore_ble_is_connected() alone dropped pushes over USB). */
static bool companion_transport_up(void)
{
	bool up = zephcore_ble_is_connected();
#if ZEPHCORE_USB_STACK
	up = up || (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB &&
		    !zephcore_usb_companion_is_text_session());
#endif
	return up;
}

/* [push_code][data...] */
static void push_callback(uint8_t code, const uint8_t *data, size_t len)
{
	if (!companion_transport_up()) return;

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

/* Inbound binary frames, on MESH_EVENT_BLE_RX. */
static void process_companion_rx(void)
{
	struct {
		uint16_t len;
		uint8_t buf[MAX_FRAME_SIZE];
	} f;
	bool handled = false;

	/* Process all queued frames */
	while (k_msgq_num_used_get(&pending_reply) == 0 &&
	       k_msgq_get(zephcore_ble_get_recv_queue(), &f, K_NO_WAIT) == 0) {
		handled = true;
#ifdef ZEPHCORE_LORA
		/* A contact dump survives commands in between (the app talks to us
		 * mid-sync); only CMD_APP_START or a disconnect ends it. */
		if (!companion_mesh_ptr->handleCmdFrame(f.buf, f.len)) {
			LOG_DBG("rx_process: unknown cmd 0x%02x len=%u", f.buf[0], (unsigned)f.len);
			uint8_t err_rsp[] = { 0x01, 0x01 };  /* PACKET_ERROR, ERR_UNSUPPORTED */
			write_frame(err_rsp, sizeof(err_rsp));
		}
#endif
	}
	ARG_UNUSED(handled);

#if ZEPHCORE_USB_STACK
	/* Over USB nothing else kicks the pump that paces multi-frame responses
	 * (the contact dump), so kick it here. Only after a handled frame: over
	 * USB the kick always posts MESH_EVENT_CONTACT_ITER, which calls this
	 * function, and an unconditional kick spun the main thread at 100% CPU
	 * and starved the LoRa TX-wait thread. */
	if (handled && zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
		zephcore_ble_kick_tx();
	}
#endif
}

/* The contact dump, as far as TX has room (MESH_EVENT_CONTACT_ITER). */
static void run_contact_iteration(void)
{
#ifdef ZEPHCORE_LORA
	if (!companion_mesh_ptr) {
		return;
	}

#if ZEPHCORE_USB_STACK
	if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
		/* Fill the USB TX ring, then stop; usb_on_tx_drain() brings us back
		 * when it is empty. tx_has_space() guarantees the next frame fits. */
		while (zephcore_usb_companion_tx_has_space(CONTACT_FRAME_SIZE)) {
			if (!companion_mesh_ptr->continueContactIteration()) {
				break;  /* iteration complete */
			}
		}
		return;
	}
#endif

	/* Not while BLE TX is congested, and not above a 2/3 high-water mark. */
	if (zephcore_ble_is_congested()) {
		return;
	}

	uint32_t used = k_msgq_num_used_get(zephcore_ble_get_send_queue());
	uint32_t queue_size = CONFIG_ZEPHCORE_BLE_QUEUE_SIZE;
	bool has_space = (used < (queue_size * 2 / 3));

	if (has_space && companion_mesh_ptr) {
		if (companion_mesh_ptr->continueContactIteration()) {
			zephcore_ble_kick_tx();  /* more to send; the drain brings us back */
		}
	}
#endif
}

/* The main thread: blocks until an event, plus the housekeeping tick. */
static void mesh_event_loop(void)
{
	LOG_INF("starting event-driven loop");

	k_timer_start(&housekeeping_timer, K_MSEC(HOUSEKEEPING_INTERVAL_MS),
		      K_MSEC(HOUSEKEEPING_INTERVAL_MS));

	for (;;) {
		uint32_t events = k_event_wait(&mesh_events, MESH_EVENT_ALL, false, K_FOREVER);
		k_event_clear(&mesh_events, events);

		/* TX-idle resumes both the retained reply and commands left in RX.
		 * Housekeeping is a fallback if a transport loses its drain kick. */
		retry_pending_reply();

#ifdef ZEPHCORE_LORA
		/* UI button actions (advert, pref saves) */
		if (events & MESH_EVENT_UI_ACTION) {
			mesh_handle_ui_actions();
		}

		/* GPS wake/timeout. GNSS configuration blocks on a semaphore the
		 * system work queue signals, so it would deadlock there. */
		if (events & MESH_EVENT_GPS_ACTION) {
			gps_process_event();
		}

		/* Inbound frames and CLI lines, before loop() drains what they
		 * queued. */
		if (events & (MESH_EVENT_BLE_RX | MESH_EVENT_CONTACT_ITER |
			      MESH_EVENT_HOUSEKEEPING)) {
			process_companion_rx();
#if ZEPHCORE_USB_STACK
			struct companion_cli_line c;
			while (k_msgq_get(&companion_cli_queue, &c, K_NO_WAIT) == 0) {
				companion_cli_run(c.buf);
			}
#endif
		}

		if (events & MESH_EVENT_CONTACT_ITER) {
			run_contact_iteration();
		}

		/* Packet processing only on radio/transport/TX events */
		if (companion_mesh_ptr &&
		    (events & (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE |
			       MESH_EVENT_BLE_RX | MESH_EVENT_TX_DRAIN))) {
			companion_mesh_ptr->loop();
		}

		if (events & MESH_EVENT_HOUSEKEEPING) {
			/* Noise floor, adaptive-CAD probe, RX watchdog: never on
			 * packet-driven events. */
			if (companion_mesh_ptr) {
				companion_mesh_ptr->maintenanceLoop();
			}

			/* Contact-dump watchdog: only the tx-idle callback pumps the
			 * dump, so one lost kick strands it. Re-kick only if the
			 * cursor did not move for a whole tick. */
			static int wd_last_iter_idx = -1;
			if (companion_mesh_ptr && companion_mesh_ptr->isContactIterActive()) {
				int idx = companion_mesh_ptr->getContactIterIdx();
				if (idx == wd_last_iter_idx) {
					LOG_WRN("contact dump watchdog: no progress at %d, resuming", idx);
					k_event_post(&mesh_events, MESH_EVENT_CONTACT_ITER);
				}
				wd_last_iter_idx = idx;
			} else {
				wd_last_iter_idx = -1;
			}

			/* Offline-queue watchdog (see msgWaitingWatchdog), only with
			 * a live client: a reconnect drains the queue anyway, and a
			 * prompt into a dead link would only delay the next one. */
			if (companion_mesh_ptr && companion_transport_up()) {
				companion_mesh_ptr->msgWaitingWatchdog();
			}

			/* Advertising watchdog: a transient bt_le_adv_start failure
			 * otherwise leaves the node undiscoverable until reboot. */
			if (zephcore_ble_is_enabled() && !zephcore_ble_is_connected() && !zephcore_ble_is_advertising()) {
				LOG_WRN("BLE adv watchdog: not advertising, re-enabling");
				zephcore_ble_set_enabled(true);
			}

			mesh_housekeeping_ui_refresh();

			if (companion_mesh_ptr) {
				companion_mesh_ptr->timeSyncTick();
			}

			/* Low-battery auto-shutdown; compiled out unless the board
			 * sets a threshold. */
			ui_auto_shutdown_check();

			/* The v-contact low-battery alert, separate because the
			 * shutdown check is a no-op on headless builds. */
			vcontact_battery_alert_check();
		}

		/* Prefs changed off-main: the flash write happens here, and posts
		 * coalesce into one write. */
		if ((events & MESH_EVENT_PREFS_DIRTY) && companion_mesh_ptr) {
			save_prefs_to_flash();
		}
#endif

		/* See pending_rtc_epoch. */
		if (events & MESH_EVENT_RTC_SAVE) {
			zephcore_rtc_save((uint32_t)atomic_get(&pending_rtc_epoch));
#ifdef ZEPHCORE_LORA
			/* GPS set the clock: arm the time-sync drift envelope and
			 * activate a deferred v-contact. */
			if (companion_mesh_ptr) {
				companion_mesh_ptr->noteGPSTimeSync();
				companion_mesh_ptr->vcontactClockSynced();
			}
#endif
		}

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
		if (events & MESH_EVENT_JOYSTICK_LOOP) {
			joystick_ui_task.loop();
		}
#endif
	}
}

static void housekeeping_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_HOUSEKEEPING);
}

#ifdef ZEPHCORE_LORA
/* ISR context */
static void lora_rx_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

/* Driver work queue, when an async TX finishes */
static void lora_tx_done_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_TX_DONE);
}

/* The dispatcher queued a delayed packet: wake when the delay expires. */
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

/* main() binds the radio to companion_mesh.prefs (setPrefs) before begin(). */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LoRaRadio lora_radio(lora_dev, zephyr_board);

static uint16_t get_battery_mv(void)
{
	return zephyr_board.getBattMilliVolts();
}

static mesh::ZephyrMillisecondClock ms_clock;
static mesh::ZephyrRNG zephyr_rng;
static SimpleMeshTables mesh_tables;
static StaticPoolPacketManager packet_mgr;
static CompanionMesh companion_mesh(lora_radio, ms_clock, zephyr_rng, rtc_clock,
	packet_mgr, mesh_tables, data_store);

static void save_prefs_to_flash(void)
{
	data_store.savePrefs(companion_mesh_ptr->prefs);
}

/* ========== Companion CLI ==========
 * The CommonCLI behind CompanionMesh::handleCommand, whose front-ends are the
 * app's CMD_RUN_CLI_COMMAND, the USB text console (the '<' sync byte tells V3
 * frames from text; BLE NUS has no such marker, so it has no text console),
 * the v-contact chat, and over-the-air TXT_TYPE_CLI_COMMAND. Not gated on
 * ZEPHCORE_USB_STACK. */

class CompanionCLICallbacks : public CommonCLICallbacks {
public:
	void savePrefs() override {
		data_store.savePrefs(companion_mesh.prefs);
	}
	const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
	const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
	const char* getRole() override { return "companion"; }
	/* CLI `erase`. Re-stamp the init marker as factoryReset() does, or the
	 * next boot sees no marker and formats a second time. */
	bool formatFileSystem() override {
		bool ok = data_store.formatFileSystem();
		if (ok) {
			data_store.writeInitMarker();
		}
		return ok;
	}

	/* Same path as the app's advert command; delay_millis is unused. */
	void sendSelfAdvertisement(int delay_millis, bool flood) override {
		(void)delay_millis;
		companion_mesh.sendSelfAdvert(flood);
	}
	void updateAdvertTimer() override {}
	void updateFloodAdvertTimer() override {}

	/* Reboot gate: true once the app has everything we owe it, including a
	 * held-back v-contact ack, so a `reboot` typed there waits for its ack. */
	bool transportTxIdle() override {
		if (companion_mesh.vcontactConfirmPending() ||
		    k_msgq_num_used_get(&pending_reply) != 0) {
			return false;
		}
#if ZEPHCORE_USB_STACK
		if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
			return zephcore_usb_companion_tx_idle();
		}
#endif
		return zephcore_ble_tx_idle();
	}

	/* No log file on a companion. */
	void setLoggingOn(bool enable) override { (void)enable; }
	void eraseLogFile() override {}
	void dumpLogFile() override {}

	/* No runtime TX-power API in the LoRa driver; log only, as the repeater. */
	void setTxPower(int8_t power_dbm) override {
		LOG_INF("TX power %d dBm requested (reboot to apply)", power_dbm);
	}

	bool setRxBoostedGain(bool enable) override {
		return lora_radio.setRxBoost(enable);
	}

	bool setFemRxGain(bool enable) override {
		return lora_radio.setFemRxEnable(enable);
	}

	bool configSideDetectors(const uint8_t* sfs, uint8_t num) override {
		return lora_radio.configSideDetectors(sfs, num);
	}

	/* Adaptive CAD */
	int formatFreqErrorStatus(char* buf, int cap) override {
		return lora_radio.formatFreqErrorStatus(buf, cap);
	}
	int formatCadStatus(char* buf, int cap) override {
		return lora_radio.formatCadStatus(buf, cap);
	}
	void applyCadPrefs() override {
		lora_radio.setCadParams(companion_mesh.prefs.cad_auto != 0,
					companion_mesh.prefs.cad_offset,
					companion_mesh.prefs.probe_interval,
					companion_mesh.prefs.cad_busycap,
					companion_mesh.prefs.cad_base);
		companion_mesh.prefs.cad_offset = lora_radio.getCadOffset();
		companion_mesh.prefs.cad_base = lora_radio.cadBasePeak();
	}
	void resetCadStats() override {
		lora_radio.resetCadStats();
	}

	mesh::LocalIdentity& getSelfId() override { return companion_mesh.self_id; }

	void saveIdentity(const mesh::LocalIdentity& new_id) override {
		companion_mesh.self_id = new_id;
		data_store.saveMainIdentity(new_id);
	}

	void clearStats() override {
		lora_radio.resetStats();
		lora_radio.resetDutyCycleTimeoutRestarts();
		companion_mesh.resetStats();
	}

	/* Duty-cycle false-preamble re-arm count.  Without these the
	 * CommonCLICallbacks default answers 0 forever, so `get dc.restarts`
	 * read clean on every companion regardless of what the radio was doing
	 * — and the companion is the role the duty cycle actually runs in.
	 * Mirrors RepeaterMesh::getDutyCycleTimeoutRestarts(). */
	uint32_t getDutyCycleTimeoutRestarts() const override {
		return lora_radio.getDutyCycleTimeoutRestarts();
	}

	void resetDutyCycleTimeoutRestarts() override {
		lora_radio.resetDutyCycleTimeoutRestarts();
	}

	/* Temp radio params — deferred; stub for now. */
	void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr,
				  int timeout_mins) override {
		(void)freq; (void)bw; (void)sf; (void)cr; (void)timeout_mins;
	}

	MeshTimeSync* getMeshTimeSync() override {
		return companion_mesh.getMeshTimeSync();
	}

	/* GPS, through the GPS manager as on the repeater. */
	bool setGpsEnabled(bool enabled) override {
		if (!gps_is_available()) return false;
		gps_enable(enabled);
		return true;
	}
	bool isGpsEnabled() const override {
		return gps_is_enabled();
	}
	void formatGpsStatsReply(char* reply) override {
		if (!gps_is_enabled()) {
			strcpy(reply, "off");
			return;
		}
		struct gps_state_info gsi;
		gps_get_state_info(&gsi);
		static const char* const state_str[] = { "off", "standby", "acquiring" };
		const char* state = gsi.state < 3 ? state_str[gsi.state] : "unknown";
		struct gps_position pos;
		bool has_pos = gps_get_last_known_position(&pos);
		if (has_pos) {
			snprintf(reply, CLI_REPLY_SIZE,
				"on state=%s sats=%u fix=%us ago lat=%.6f lon=%.6f",
				state, gsi.satellites, gsi.last_fix_age_s,
				pos.latitude_ndeg / 1e9, pos.longitude_ndeg / 1e9);
		} else if (gsi.next_search_s > 0) {
			snprintf(reply, CLI_REPLY_SIZE,
				"on state=%s sats=%u no fix next=%us",
				state, gsi.satellites, gsi.next_search_s);
		} else {
			snprintf(reply, CLI_REPLY_SIZE,
				"on state=%s sats=%u no fix",
				state, gsi.satellites);
		}
	}

	/* stats-core / stats-radio / stats-packets, as the repeater prints them. */
	void formatStatsReply(char* reply) override {
		StatsFormatHelper::formatCoreStats(reply, zephyr_board, ms_clock,
			companion_mesh.getErrFlags(), &packet_mgr);
	}
	void formatRadioStatsReply(char* reply) override {
		StatsFormatHelper::formatRadioStats(reply, &lora_radio, lora_radio,
			companion_mesh.getTotalAirTime(), companion_mesh.getReceiveAirTime());
	}
	void formatPacketStatsReply(char* reply) override {
		StatsFormatHelper::formatPacketStats(reply, lora_radio,
			companion_mesh.getNumSentFlood(), companion_mesh.getNumSentDirect(),
			companion_mesh.getNumRecvFlood(), companion_mesh.getNumRecvDirect());
	}
};

static CompanionCLICallbacks companion_cli_cbs;
static ClientACL companion_acl;  /* unused by CommonCLI but required by constructor */
static CommonCLI companion_cli(zephyr_board, rtc_clock, companion_acl,
			       &companion_mesh.prefs, &companion_cli_cbs);

/* CMD_SET_RADIO_PARAMS / CMD_SET_RADIO_TX_POWER (needs companion_cli, hence
 * here). Unlike the CLI's `set radio` this applies live, so the radio is
 * already on the new preset when the CAD state resets: preset_pending false. */
static void radio_reconfigure(bool preset_changed)
{
	lora_radio.reconfigure();

	if (preset_changed) {
		companion_cli.resetCadState(false);
	}
}

#if defined(CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS) && \
	CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS > 0
#define ZEPHCORE_HAS_AUTO_SHUTDOWN 1
#else
#define ZEPHCORE_HAS_AUTO_SHUTDOWN 0
#endif

#if ZEPHCORE_HAS_AUTO_SHUTDOWN
/* `get|set autoshutdown`: companion-only, so not in CommonCLI. True if the
 * line was one. */
static bool handle_autoshutdown_cli(const char *line, char *reply)
{
	if (strcmp(line, "get autoshutdown") == 0) {
		uint16_t mv = companion_mesh.prefs.auto_shutdown_mv;
		if (mv == 0) {
			strcpy(reply, "autoshutdown: off");
		} else {
			snprintf(reply, CLI_REPLY_SIZE, "autoshutdown: %u mV", mv);
		}
		return true;
	}
	if (strncmp(line, "set autoshutdown ", 17) == 0) {
		const char *arg = line + 17;
		while (*arg == ' ') {
			arg++;
		}
		/* Digits only, then nothing but whitespace. */
		char *end = NULL;
		long v = strtol(arg, &end, 10);
		while (*end == ' ' || *end == '\r' || *end == '\n' || *end == '\t') {
			end++;
		}
		if (arg[0] < '0' || arg[0] > '9' || *end != '\0') {
			strcpy(reply, "ERROR: numbers only (0 = off, 1-5000 mV)");
			return true;
		}
		if (v > 5000) {
			strcpy(reply, "ERROR: must be 0 (off) or 1-5000 mV");
			return true;
		}
		companion_mesh.prefs.auto_shutdown_mv = (uint16_t)v;
		/* Coalesced flash write from the event loop. */
		k_event_post(&mesh_events, MESH_EVENT_PREFS_DIRTY);
		ui_set_auto_shutdown_mv((uint16_t)v);
		if (v == 0) {
			strcpy(reply, "OK - autoshutdown off");
		} else {
			snprintf(reply, CLI_REPLY_SIZE, "OK - autoshutdown %ld mV", v);
		}
		return true;
	}
	return false;
}
#endif /* ZEPHCORE_HAS_AUTO_SHUTDOWN */

static void vcontact_battery_alert_rearm(void);

/* The v-contact battery-alert threshold in mV, 0 = off. The 0xFFFF default is
 * 200 mV above the auto-shutdown cutoff, so the alert beats the 90 s shutdown
 * confirm window; 3500 mV on boards without a cutoff. */
static uint16_t vcontact_battery_alert_threshold_mv(void)
{
	uint16_t pref = companion_mesh.prefs.v_battery_alert_mv;
	if (pref == 0) return 0;
	if (pref != 0xFFFF) return pref;
	uint16_t cutoff = companion_mesh.prefs.auto_shutdown_mv;
	return cutoff ? (uint16_t)(cutoff + 200) : 3500;
}

/* The `v.*` commands (v-contact settings) and `help`. True if the line was one. */
static bool handle_vcontact_cli(const char *line, char *reply)
{
	if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
		/* There is no global help: list the companion extras only. */
		strcpy(reply,
		       "Companion extras (standard set/get commands also work):\r\n"
#if ZEPHCORE_HAS_AUTO_SHUTDOWN
		       "  get|set autoshutdown <mV>     - low-batt cutoff, 0 = off\r\n"
#endif
		       "  get|set v.contact on|off      - loopback admin contact\r\n"
		       "  get|set v.batteryalert <mV>   - 0 = off, or default");
		return true;
	}
	if (strcmp(line, "get v.contact") == 0) {
		snprintf(reply, CLI_REPLY_SIZE, "v.contact: %s",
			 companion_mesh.prefs.v_contact_enabled ? "on" : "off");
		return true;
	}
	if (strncmp(line, "set v.contact ", 14) == 0) {
		const char *arg = line + 14;
		while (*arg == ' ') arg++;
		bool en;
		if (strcmp(arg, "on") == 0 || strcmp(arg, "1") == 0) {
			en = true;
		} else if (strcmp(arg, "off") == 0 || strcmp(arg, "0") == 0) {
			en = false;
		} else {
			strcpy(reply, "ERROR: use on|off|1|0");
			return true;
		}
		bool was = companion_mesh.prefs.v_contact_enabled != 0;
		companion_mesh.prefs.v_contact_enabled = en ? 1 : 0;
		k_event_post(&mesh_events, MESH_EVENT_PREFS_DIRTY);
		if (en && !was) {
			companion_mesh.vcontactPushAdvert();
		} else if (!en && was) {
			companion_mesh.vcontactPushDeleted();
			/* The app drops the contact, and its flags with it. */
			companion_mesh.prefs.v_contact_flags = 0;
		}
		snprintf(reply, CLI_REPLY_SIZE, "OK - v.contact %s", en ? "on" : "off");
		return true;
	}
	if (strcmp(line, "get v.batteryalert") == 0) {
		uint16_t pref = companion_mesh.prefs.v_battery_alert_mv;
		if (pref == 0) {
			strcpy(reply, "v.batteryalert: off");
		} else if (pref == 0xFFFF) {
			snprintf(reply, CLI_REPLY_SIZE, "v.batteryalert: default (%u mV)",
				 vcontact_battery_alert_threshold_mv());
		} else {
			snprintf(reply, CLI_REPLY_SIZE, "v.batteryalert: %u mV", pref);
		}
		return true;
	}
	if (strncmp(line, "set v.batteryalert ", 19) == 0) {
		const char *arg = line + 19;
		while (*arg == ' ') arg++;
		uint16_t v;
		if (strncmp(arg, "default", 7) == 0) {
			v = 0xFFFF;
		} else {
			/* Numbers only, same validation as autoshutdown. */
			char *end = NULL;
			long n = strtol(arg, &end, 10);
			while (*end == ' ' || *end == '\r' || *end == '\n' || *end == '\t') {
				end++;
			}
			if (arg[0] < '0' || arg[0] > '9' || *end != '\0' || n > 5000) {
				strcpy(reply, "ERROR: use 0 (off), 1-5000 mV, or default");
				return true;
			}
			v = (uint16_t)n;
		}
		companion_mesh.prefs.v_battery_alert_mv = v;
		k_event_post(&mesh_events, MESH_EVENT_PREFS_DIRTY);
		vcontact_battery_alert_rearm();
		if (v == 0) {
			strcpy(reply, "OK - v.batteryalert off");
		} else if (v == 0xFFFF) {
			snprintf(reply, CLI_REPLY_SIZE, "OK - v.batteryalert default (%u mV)",
				 vcontact_battery_alert_threshold_mv());
		} else {
			snprintf(reply, CLI_REPLY_SIZE, "OK - v.batteryalert %u mV", v);
		}
		return true;
	}
	return false;
}

/* Called just before a low-battery power-off. With an app connected, send a
 * v-contact notice and return true (the UI waits a short grace for it).
 * Otherwise store the reason in flash, reported on the next boot, since the
 * offline queue does not survive System OFF, and return false. */
static bool companion_shutdown_hook(int reason)
{
	const char *msg = (reason == UI_SHUTDOWN_LOW_BATTERY)
			  ? "Powering off: low battery"
			  : "Powering off";

	bool connected = zephcore_ble_is_connected();
#if ZEPHCORE_USB_STACK
	connected = connected ||
		(zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB &&
		 !zephcore_usb_companion_is_text_session());
#endif

	if (connected && companion_mesh.isVContactEnabled()) {
		companion_mesh.vcontactNotify(msg);
		return true;   /* deliver live — ask the UI for the grace delay */
	}

	data_store.saveShutdownReason((uint8_t)reason);
	return false;      /* nobody listening — flash marker, power off now */
}

/* The CompanionMesh::handleCommand callback (main thread). The v-contact and
 * autoshutdown commands never see a remote sender: handleCommand stops those. */
static_assert(COMPANION_CLI_REPLY_SIZE == CLI_REPLY_SIZE,
	      "companion CLI reply buffer must match CommonCLI reply size");

static void companion_cli_exec(const char *line, uint32_t sender_timestamp,
			       uint8_t reply_hdr_used, char *reply)
{
	reply[0] = '\0';
	if (handle_vcontact_cli(line, reply)) {
		return;
	}
#if ZEPHCORE_HAS_AUTO_SHUTDOWN
	if (handle_autoshutdown_cli(line, reply)) {
		return;
	}
#endif
	companion_cli.setReplyHeaderUsed(reply_hdr_used);
	companion_cli.handleCommand(sender_timestamp, line, reply);
}

/* ========== V-contact battery alert ==========
 * Not in ui_auto_shutdown_check(), which is a no-op on headless builds; same
 * pattern: a 30 s sample gate on the housekeeping tick, three strikes so a TX
 * sag cannot trigger it, skipped on external power. Once per discharge; it
 * re-arms at threshold + 150 mV or on a threshold change. */
static bool vcontact_batt_latched;
static uint8_t vcontact_batt_low_count;

static void vcontact_battery_alert_rearm(void)
{
	vcontact_batt_latched = false;
	vcontact_batt_low_count = 0;
}

static void vcontact_battery_alert_check(void)
{
	if (!companion_mesh.isVContactEnabled()) {
		return;
	}
	uint16_t thresh = vcontact_battery_alert_threshold_mv();
	if (thresh == 0) {
		return;
	}

	uint32_t now = k_uptime_get_32();
	static uint32_t next_check_ms;  /* 0 at boot → first tick samples */
	if (next_check_ms != 0 && (now - next_check_ms) < 30000) {
		return;
	}
	next_check_ms = now;

	uint16_t mv = zephyr_board.getBattMilliVolts();
	if (mv == 0) {  /* no battery hardware / no reading */
		vcontact_batt_low_count = 0;
		return;
	}
	if (zephyr_board.isExternalPowered() || mv >= thresh + 150) {
		/* charging or recovered — re-arm for the next discharge cycle */
		vcontact_battery_alert_rearm();
		return;
	}
	if (mv >= thresh) {
		vcontact_batt_low_count = 0;
		return;
	}
	if (vcontact_batt_latched) {
		return;
	}
	if (++vcontact_batt_low_count < 3) {
		return;
	}
	vcontact_batt_latched = true;
	char msg[48];
	snprintf(msg, sizeof(msg), "Battery low: %u mV (alert at %u mV)", mv, thresh);
	companion_mesh.vcontactNotify(msg);
}

#if ZEPHCORE_USB_STACK
/* A USB text-CLI line. The reply is framed like the repeater's serial CLI
 * ("\r\n  -> <reply>\r\n") so web serial consoles render it the same. */
static void companion_cli_run(const char *line)
{
	char reply[COMPANION_CLI_PREFIX_ROOM + CLI_REPLY_SIZE];
	companion_mesh.handleCommand(line, 0, reply);
	if (reply[0] != '\0') {
		zephcore_usb_companion_write_text("\r\n  -> ", 7);
		zephcore_usb_companion_write_text(reply, strlen(reply));
	}
	zephcore_usb_companion_write_text("\r\n", 2);
}

/* USB adapter callback (sysworkq): queue the line for companion_cli_run(). */
static void companion_cli_dispatch(const char *line)
{
	struct companion_cli_line c;
	strncpy(c.buf, line, sizeof(c.buf) - 1);
	c.buf[sizeof(c.buf) - 1] = '\0';
	if (k_msgq_put(&companion_cli_queue, &c, K_NO_WAIT) == 0) {
		k_event_post(&mesh_events, MESH_EVENT_BLE_RX);
	} else {
		zephcore_usb_companion_write_text("\r\n  -> busy\r\n", 13);
	}
}

#endif  /* ZEPHCORE_USB_STACK */

#endif

/* GPS on/off (persisted by the CMD_SET_CUSTOM_VAR handler) */
static void gps_enable_callback(bool enabled)
{
	LOG_INF("GPS %s", enabled ? "enabled" : "disabled");
	ui_set_gps_enabled(enabled);
}

/* A GPS state transition for the main thread (from the system work queue). */
static void gps_event_callback(void)
{
	k_event_post(&mesh_events, MESH_EVENT_GPS_ACTION);
}

/* A valid fix (GNSS modem_chat thread): set the clock and the node position. */
static void gps_fix_callback(double lat, double lon, int64_t utc_time)
{
	if (utc_time > 0) {
		LOG_INF("GPS fix: RTC sync time=%lld", utc_time);
		rtc_clock.setCurrentTime((uint32_t)utc_time);
		time_sync_report(TIME_SYNC_GPS);
		request_rtc_save((uint32_t)utc_time);
	}

#ifdef ZEPHCORE_LORA
	/* RAM only, as upstream: GPS jitter would otherwise rewrite prefs on
	 * nearly every fix. It reaches flash with the next real settings save. */
	if (lat != companion_mesh.prefs.node_lat || lon != companion_mesh.prefs.node_lon) {
		companion_mesh.prefs.node_lat = lat;
		companion_mesh.prefs.node_lon = lon;
		/* Zephyr LOG has no %f by default */
		int lat_deg = (int)lat;
		int lon_deg = (int)lon;
		int lat_frac = (int)((lat - lat_deg) * 1000000);
		int lon_frac = (int)((lon - lon_deg) * 1000000);
		if (lat_frac < 0) lat_frac = -lat_frac;
		if (lon_frac < 0) lon_frac = -lon_frac;
		LOG_INF("GPS fix: position updated lat=%d.%06d lon=%d.%06d",
			lat_deg, lat_frac, lon_deg, lon_frac);
	}

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

#if IS_ENABLED(CONFIG_BT)
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
	if (companion_mesh.prefs.ble_disabled) {
		zephcore_ble_set_enabled(false);
	}
}
#endif /* CONFIG_BT */

int main(void)
{
	zephyr_board.clearBootloaderMagic();

	/* USB up first so the host can enumerate; wait up to 1 s for the port to
	 * open (DTR), else the banner is buffered for a later attach. */
#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	!IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT) && \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))
	zephcore_usbd_init();
	zephcore_usbd_wait_dtr(1000);
#endif
	LOG_INF("=== ZephCore starting ===");

	/* Restart-reason notice for the v-contact (cause captured at POST_KERNEL by
	 * helpers/boot_info.c), queued once the mesh is up. A power-integrity
	 * breadcrumb; a debugger reset or a button wake is quiet. Size: "Restarted:"
	 * (10) + every label (238) + shutdown reason (29) + NUL. */
	char boot_cause_msg[280];
	boot_cause_msg[0] = '\0';
	{
		const uint32_t quiet = RESET_LOW_POWER_WAKE | RESET_DEBUG;

		if ((zephcore_boot_reset_cause_labelled() & ~quiet) != 0) {
			int n = snprintf(boot_cause_msg, sizeof(boot_cause_msg), "Restarted:");
			(void)zephcore_boot_reset_cause_str(boot_cause_msg + n,
							    sizeof(boot_cause_msg) - n, true);
		}
	}

	if (!ZephyrDataStore::mount()) {
		LOG_ERR("LittleFS mount failed");
	}
	data_store.begin();

	/* A low-battery shutdown with no app connected left its reason in flash
	 * (see companion_shutdown_hook); the hardware cause cannot tell. */
	{
		uint8_t sdr = data_store.takeShutdownReason();
		if (sdr == UI_SHUTDOWN_LOW_BATTERY) {
			size_t l = strlen(boot_cause_msg);
			snprintf(boot_cause_msg + l, sizeof(boot_cause_msg) - l,
				 "%sLast shutdown: low battery",
				 l ? " | " : "");
		}
	}

	/* First-boot migration: fix NVS (BLE bonds) before bt_enable() runs.
	 *
	 * nRF52 stores BLE bonds in storage_partition (NVS) at 0xD0000.  UF2
	 * flashing only writes pages covered by the binary, leaving whatever was
	 * there before.  Old firmware (Arduino MeshCore, ZephCore ≤1.16.1) used
	 * that region as app code; if those bytes accidentally pass Zephyr NVS
	 * sector validation, settings_load() hangs and BLE never advertises.
	 *
	 * A marker file /lfs/_zc_init is written after the first clean boot.
	 * If absent, we are on the first run of this ZephCore build:
	 *
	 *  • No prefs, or Arduino prefs (layout-incompatible): full format.
	 *    Covers fresh installs and Arduino MeshCore migrations.
	 *
	 *  • Valid ZephCore prefs + /lfs/settings present: NVS erase only.
	 *    The old file-based bonds backend (ZephCore ≤1.16.1) left this file;
	 *    0xD0000 is old app code → must erase.  Identity/prefs/contacts
	 *    are preserved; re-pair required (bonds were in /lfs/settings which
	 *    the NVS backend ignores anyway).
	 *
	 *  • Valid ZephCore prefs + no /lfs/settings: NVS was already initialized
	 *    by ZephCore ≥1.16.2 — skip format entirely, bonds survive. */
	if (!data_store.hasInitMarker()) {
		if (!data_store.hasPrefs() || data_store.prefsLookLikeArduino()) {
			LOG_WRN("First ZephCore boot (%s) — formatting LFS + NVS",
				data_store.hasPrefs() ? "Arduino prefs" : "no prefs");
			data_store.formatFileSystem();
			data_store.begin();
		} else if (data_store.hasOldSettingsFile()) {
			LOG_WRN("Pre-NVS ZephCore upgrade (found /lfs/settings) — erasing NVS");
			data_store.formatNVSOnly();
		} else {
			LOG_INF("ZephCore upgrade with valid NVS — skipping format, bonds preserved");
		}
		data_store.writeInitMarker();
	}

	sensor_manager_init();

	/* Wall-clock time from a battery-backed RTC, if one answers on I2C. */
	{
		uint32_t rtc_epoch;
		if (zephcore_rtc_restore(&rtc_epoch)) {
			rtc_clock.setCurrentTime(rtc_epoch);
		}
	}

	/* Buttons, buzzer, display; without a display UI, put any OLED to sleep. */
	ui_init();
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
	joystick_ui_hooks_register(&joystick_ui_task, joystick_signal_refresh, joystick_signal_tx);
#endif
#if !IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	oled_sleep();
#endif

	if (gps_is_available()) {
		LOG_INF("GPS available");
		gps_set_enable_callback(gps_enable_callback);
		gps_set_fix_callback(gps_fix_callback);
		gps_set_event_callback(gps_event_callback);
	}

	if (env_sensors_available()) {
		LOG_INF("Environment sensors available");
	}

#ifdef ZEPHCORE_LORA
	/* Defaults from initNodePrefs() only: a field defaulted here instead is
	 * missed by its other callers, and a missing default reads back as 0 on
	 * upgrade (that zeroed probe_interval and cad_auto once). The one override
	 * lets a board's Kconfig GPS duty win over the 300 s companion figure. */
	initNodePrefs(&companion_mesh.prefs);
	companion_mesh.prefs.gps_interval = CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC; /* 0 = always on */

	data_store.loadPrefs(companion_mesh.prefs);

	/* Saved BLE PIN (0 = Kconfig default) */
	if (companion_mesh.prefs.ble_pin >= 100000 && companion_mesh.prefs.ble_pin <= 999999) {
		zephcore_ble_set_passkey(companion_mesh.prefs.ble_pin);
		LOG_INF("BLE passkey loaded from prefs: %06u", companion_mesh.prefs.ble_pin);
	}

	/* The radio reads its preset through this from Radio::begin() on. */
	lora_radio.setPrefs(&companion_mesh.prefs);

	data_store.loadContacts(&companion_mesh);
	data_store.loadChannels(&companion_mesh);

	/* First boot: the Public channel */
	if (companion_mesh.getNumChannels() == 0) {
		// "izOH6cXN6mrJ5e26oRXNcg==" decoded
		static const uint8_t public_psk[16] = {
			0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
			0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72
		};
		companion_mesh.addChannel("Public", public_psk, 16);
		data_store.saveChannels(&companion_mesh);
		LOG_INF("Added default Public channel");
	}

	/* First boot generates the identity (ZephyrRNG::generateFirstBootIdentity). */
	mesh::LocalIdentity self_identity;
	if (!data_store.loadMainIdentity(self_identity)) {
		mesh::ZephyrRNG::generateFirstBootIdentity(self_identity);
		data_store.saveMainIdentity(self_identity);
	}
	companion_mesh.self_id = self_identity;

	/* Default node name: the first 4 bytes of the public key. */
	if (companion_mesh.prefs.node_name[0] == '\0') {
		snprintf(companion_mesh.prefs.node_name, sizeof(companion_mesh.prefs.node_name),
			 "%02X%02X%02X%02X",
			 self_identity.pub_key[0], self_identity.pub_key[1],
			 self_identity.pub_key[2], self_identity.pub_key[3]);
	}

	companion_mesh.setWriteFrameCallback(write_frame);
	companion_mesh.setPushCallback(push_callback);
	companion_mesh.setBatteryCallback(get_battery_mv);
	ui_set_battery_provider(get_battery_mv);
	ui_set_power_source_provider([]() { return zephyr_board.isExternalPowered(); });
	ui_set_auto_shutdown_mv(companion_mesh.prefs.auto_shutdown_mv);
	ui_set_shutdown_hook(companion_shutdown_hook);
	companion_mesh.setRadioReconfigureCallback(radio_reconfigure);
	companion_mesh.setPinChangeCallback([](uint32_t new_pin) {
		zephcore_ble_set_passkey(new_pin);
	});
	companion_mesh.setCLICallback(companion_cli_exec);
	companion_mesh_ptr = &companion_mesh;

	lora_radio.setRxCallback(lora_rx_callback, nullptr);
	lora_radio.setTxDoneCallback(lora_tx_done_callback, nullptr);
	companion_mesh.setTxQueuedCallback(tx_queued_callback, nullptr);

	companion_mesh.begin();

	/* After begin(), which derives the v-contact key. */
	if (boot_cause_msg[0] != '\0') {
		companion_mesh.vcontactNotify(boot_cause_msg);
	}

	/* Mounting orientation, as early as prefs allow (the splash may show
	 * upright first). A panel that cannot rotate stays native. */
	zephcore_input_set_flipped(companion_mesh.prefs.input_rotate != 0);
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	if (companion_mesh.prefs.display_rotate) {
		mc_display_set_rotated(true);
	}
#endif

	ui_set_node_name(companion_mesh.prefs.node_name);
	ui_set_radio_params(
		lora_radio.getActiveFrequencyHz(),
		lora_radio.getActiveSpreadingFactor(),
		lora_radio.getActiveBandwidthKHzX10(),
		lora_radio.getActiveCodingRate(),
		lora_radio.getConfiguredTxPower(),
		lora_radio.getNoiseFloor());
	ui_set_radio_runtime(
		lora_radio.getActiveSyncWord(),
		lora_radio.getActivePreambleLength(),
		lora_radio.isRxDutyCycleEnabled(),
		lora_radio.isRadioReady(),
		lora_radio.isInRecvMode(),
		lora_radio.isTxActive());
	ui_set_radio_stats(lora_radio.getPacketsRecv(),
			   lora_radio.getPacketsSent(),
			   lora_radio.getPacketsRecvErrors());
	ui_set_battery(zephyr_board.getBattMilliVolts(), 0);
	ui_set_gps_available(gps_is_available());
	ui_set_gps_enabled(companion_mesh.prefs.gps_enabled != 0);
	ui_set_ble_enabled(companion_mesh.prefs.ble_disabled != 1);  /* BLE starts advertising at boot */

	ui_set_offgrid_mode(companion_mesh.prefs.client_repeat != 0);
	LOG_INF("offgrid mode: %s (from prefs)",
		companion_mesh.prefs.client_repeat ? "on" : "off");

	/* buzzer_init() starts quiet, so the saved mode is always applied. */
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_BUZZER)
	{
		uint8_t bmode = zephcore_buzzer_mode_from_prefs(companion_mesh.prefs.buzzer_quiet);
		zephcore_buzzer_set_mode(bmode, false);
		ui_set_buzzer_mode(bmode);
		LOG_INF("buzzer: mode=%u (from prefs)", bmode);
	}
	ui_play_startup_chime();
#endif

	/* Straight to the LED gate: it also governs the TX LED and exists in every
	 * build, UI or not. */
	bool leds_off = companion_mesh.prefs.leds_disabled != 0;
	zephcore_leds_set_disabled(leds_off);
	zephcore_leds_set_radio_mode(companion_mesh.prefs.leds_radio_mode);
	zephcore_leds_set_hb_mode(companion_mesh.prefs.leds_hb_mode);
	ui_set_heartbeat_led(!leds_off);
	LOG_INF("LEDs: %s (from prefs)", leds_off ? "disabled" : "enabled");

	/* The GPS is powered at boot: match its power to prefs, then start the
	 * state machine if enabled. */
	if (gps_is_available()) {
		gps_set_poll_interval_sec(companion_mesh.prefs.gps_interval);
		gps_ensure_power_state(companion_mesh.prefs.gps_enabled);

		if (companion_mesh.prefs.gps_enabled) {
			LOG_INF("GPS: Restoring enabled state from prefs");
			gps_enable(true);
		}
	}

	/* Apply RX boost and duty cycle from prefs */
	lora_radio.setRxBoost(companion_mesh.prefs.rx_boost != 0);
	lora_radio.setFemRxEnable(companion_mesh.prefs.fem_rxgain != 0);
	lora_radio.enableRxDutyCycle(companion_mesh.prefs.rx_duty_cycle != 0);
	lora_radio.setCadParams(companion_mesh.prefs.cad_auto != 0,
				companion_mesh.prefs.cad_offset,
				companion_mesh.prefs.probe_interval,
				companion_mesh.prefs.cad_busycap,
				companion_mesh.prefs.cad_base);
	/* Persist the (offset, base) in force if setCadParams() re-anchored it
	 * across a base-table change, so the next upgrade has an anchor too. */
	if (companion_mesh.prefs.cad_base != lora_radio.cadBasePeak() ||
	    companion_mesh.prefs.cad_offset != lora_radio.getCadOffset()) {
		companion_mesh.prefs.cad_offset = lora_radio.getCadOffset();
		companion_mesh.prefs.cad_base = lora_radio.cadBasePeak();
		data_store.savePrefs(companion_mesh.prefs);
	}

	/* LR2021 side detectors (multi-SF RX). A set rejected for the current
	 * SF/BW leaves them off; other radios report them unsupported. */
	{
		uint8_t n = 0;
		while (n < EXTRA_SF_MAX && companion_mesh.prefs.extra_sf[n] != 0) n++;
		if (n > 0 && !lora_radio.configSideDetectors(companion_mesh.prefs.extra_sf, n)) {
			LOG_WRN("extra.sf %u SFs rejected for current SF/BW — side detectors off", n);
		}
	}
	ui_set_radio_runtime(
		lora_radio.getActiveSyncWord(),
		lora_radio.getActivePreambleLength(),
		lora_radio.isRxDutyCycleEnabled(),
		lora_radio.isRadioReady(),
		lora_radio.isInRecvMode(),
		lora_radio.isTxActive());

	/* ADC multiplier override (0 = devicetree default) */
	zephyr_board.setAdcMultiplier(companion_mesh.prefs.adc_multiplier);

	k_event_init(&mesh_events);

	ui_mesh_actions_init(&mesh_events, MESH_EVENT_UI_ACTION,
			     &companion_mesh, &data_store,
			     &lora_radio, &zephyr_board, &rtc_clock);

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
	joystick_ui_task.begin(&companion_mesh, &rtc_clock, &companion_mesh.prefs);
#endif
#endif

	zephcore_ble_init(&ble_cbs);

#if ZEPHCORE_USB_STACK
	zephcore_usb_companion_init(&mesh_events, MESH_EVENT_BLE_RX,
				   &zephyr_board);
	zephcore_usb_companion_set_session_start_cb(usb_on_session_start);
	zephcore_usb_companion_set_session_end_cb(usb_on_session_end);
	zephcore_usb_companion_set_tx_drain_cb(usb_on_tx_drain);
	/* The text CLI starts when the first byte is not '<'. */
	zephcore_usb_companion_set_cli_line_cb(companion_cli_dispatch);
#endif

#if IS_ENABLED(CONFIG_BT)
	if (bt_enable(bt_ready) != 0) {
		LOG_ERR("bt_enable failed");
	}
#else
	/* No BLE controller: LinuxTCPTransport.c provides zephcore_ble_start(). */
	zephcore_ble_start(companion_mesh.getDeviceName());
#endif

	/* The main thread becomes the mesh event loop (see MESH_EVENT_*). */
#ifdef ZEPHCORE_LORA
	mesh_event_loop();  /* Never returns */
#else
	for (;;) {
		k_sleep(K_FOREVER);
	}
#endif

	return 0;
}
