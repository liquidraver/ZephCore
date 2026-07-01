/*
 * SPDX-License-Identifier: MIT
 * ZephCore - CompanionMesh (Event-Driven)
 *
 * BLE stack is in adapters/ble/ZephyrBLE.cpp.
 * USB CDC transport is in adapters/usb/ZephyrUsbCompanion.cpp.
 * UI ↔ mesh actions are in helpers/ui/ui_mesh_actions.cpp.
 * This file handles: mesh event loop, LoRa, GPS, data storage, init.
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
#include <zephyr/sys/reboot.h>
#include <ZephyrSensorManager.h>
#include <helpers/time_sync.h>
#include "ui_task.h"
#include "ui_mesh_actions.h"
#include "oled_power.h"
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_BUZZER)
#include "buzzer.h"
#endif

#include <ZephyrBLE.h>

/* The USB CDC companion stack is compiled when either logging needs the CDC
 * console (debug builds) or the binary companion transport is explicitly
 * enabled (CONFIG_ZEPHCORE_COMPANION_USB, default-y on USB-capable boards). */
#define ZEPHCORE_USB_STACK \
	(IS_ENABLED(CONFIG_LOG) || IS_ENABLED(CONFIG_ZEPHCORE_COMPANION_USB))

#if ZEPHCORE_USB_STACK
#include <ZephyrCompanionUSB.h>
#include <ZephyrUSBCDC.h>
#endif

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
#include <joystick_ui_task.h>
#include <joystick_ui_hooks.h>
#endif

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <mesh/RadioIncludes.h>
#ifdef ZEPHCORE_LORA
#include <app/CompanionMesh.h>
#include <helpers/CommonCLI.h>
#include <helpers/ClientACL.h>
#include <helpers/battery_curve.h>
#endif

/*
 * BLE Controller assert handler — without this, a controller assert
 * silently freezes the CPU at highest IRQ priority with zero log output.
 * This prints the file/line and reboots so we can actually see what happened.
 */
#if IS_ENABLED(CONFIG_BT_CTLR_ASSERT_HANDLER)
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
#define MESH_EVENT_PREFS_DIRTY   BIT(8)  /* Prefs mutated off-main; main flushes to flash */
#define MESH_EVENT_RTC_SAVE      BIT(9)  /* Hardware-RTC write requested off-main */
#define MESH_EVENT_CONTACT_ITER  BIT(10) /* Continue contact-dump iteration on main thread */

#ifdef ZEPHCORE_LORA
/* Forward decl — data_store + companion_mesh_ptr statics are defined further
 * down in the file, so mesh_event_loop() can't reference them directly. */
static void save_prefs_to_flash(void);
#endif

/* Pending epoch for a deferred zephcore_rtc_save(). gps_fix_callback runs on
 * the GNSS modem_chat worker thread, where the RTC's blocking I2C transactions
 * (burst read/write, several ms each) would stall NMEA ingest — same hazard
 * documented for prefs flash writes below. We stash the latest epoch and let
 * the main thread perform the actual I2C write via MESH_EVENT_RTC_SAVE;
 * concurrent posts coalesce into one save of the latest time. */
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

/* Housekeeping interval - infrequent to preserve power savings */
#define HOUSEKEEPING_INTERVAL_MS CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS

/* Event-driven mesh loop - k_event for signaling from ISR/callbacks */
static struct k_event mesh_events;

/* Defer a hardware-RTC write to the main thread (see pending_rtc_epoch above
 * for why gps_fix_callback can't do this inline). Coalesces like prefs-dirty:
 * only the latest epoch survives if multiple fixes land before the main loop
 * services the event. */
static void request_rtc_save(uint32_t epoch)
{
	atomic_set(&pending_rtc_epoch, (atomic_val_t)epoch);
	k_event_post(&mesh_events, MESH_EVENT_RTC_SAVE);
}

/* Work items for event-driven processing */
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

#if ZEPHCORE_USB_STACK
/* Completed USB text-CLI lines are run on the MAIN thread (the USB adapter
 * assembles them on sysworkq).  CommonCLI::handleCommand mutates mesh state
 * shared with loop(), so it can't run on sysworkq — same hazard as the
 * binary protocol path.  Drained on MESH_EVENT_BLE_RX. */
#define CLI_LINE_BUF_SIZE 256
struct companion_cli_line { char buf[CLI_LINE_BUF_SIZE]; };
K_MSGQ_DEFINE(companion_cli_queue, sizeof(struct companion_cli_line), 4, 4);
#endif

#if ZEPHCORE_USB_STACK
/* USB TX-drained callback (mirrors BLE on_tx_idle): the TX ring emptied, so
 * resume the contact pump to queue the next batch. Runs in the CDC TX
 * interrupt-callback context — k_event_post is ISR-safe.  The contact dump
 * reads the contact table, which the main thread can mutate (addContact on
 * inbound advert), so it MUST run on the main thread, not here. */
static void usb_on_tx_drain(void)
{
	k_event_post(&mesh_events, MESH_EVENT_CONTACT_ITER);
}
#endif

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

	/* Wake the MAIN thread to parse the frame.  handleProtocolFrame()
	 * mutates the lock-free packet pool / dispatcher that loop() also
	 * touches, so it MUST run on the main thread — parsing on sysworkq
	 * races the main loop (the source of the stuck-"Sending…" bug when an
	 * inbound reply is processed while a send command is parsed). */
	k_event_post(&mesh_events, MESH_EVENT_BLE_RX);
}

/* BLE TX idle callback — called when TX queue is empty.
 * Continues contact iteration on the MAIN thread (the dump reads the contact
 * table, which the main thread can mutate — so it can't run on sysworkq). */
static void ble_on_tx_idle(void)
{
	k_event_post(&mesh_events, MESH_EVENT_CONTACT_ITER);
	k_event_post(&mesh_events, MESH_EVENT_TX_DRAIN);
}

/* BLE connected callback — notify UI, clear USB state if needed */
static void ble_on_connected(void)
{
	ui_notify(UI_EVENT_BLE_CONNECTED);
}

/* Per-session cleanup shared by BLE disconnect and USB session-end.
 * Silently cancels in-progress contact iteration (a reconnect would otherwise
 * send a stale PACKET_CONTACT_END to the new session and confuse the phone's
 * sync state machine), resets message sync (un-ACKed peeked message stays in
 * the queue and is re-sent on the next CMD_SYNC_NEXT_MESSAGE), and frees the
 * Ed25519 sign buffer if a sign op was interrupted (else an 8KB leak). */
static void companion_session_cleanup(void)
{
#ifdef ZEPHCORE_LORA
	companion_mesh_ptr->cancelContactIterator();
	companion_mesh_ptr->cancelSyncPending();
	companion_mesh_ptr->cleanupSignState();
#endif
}

#if ZEPHCORE_USB_STACK
/* USB session start — mirror ble_on_connected's UI notification so the
 * device shows "connected" during a USB session (matches Arduino, which
 * lights the indicator for serial transports too). */
static void usb_on_session_start(void)
{
	ui_notify(UI_EVENT_BLE_CONNECTED);
}

/* USB session end (host closed the port) — same cleanup + UI notification
 * as ble_on_disconnected. */
static void usb_on_session_end(void)
{
	companion_session_cleanup();
	ui_notify(UI_EVENT_BLE_DISCONNECTED);
}
#endif

/* BLE disconnected callback — clean up state and notify UI */
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

static size_t write_frame(const uint8_t *src, size_t len)
{
#if ZEPHCORE_USB_STACK
	/* When USB owns the interface, replies must go out the CDC port — the BLE
	 * send queue is never drained while USB is active (tx_drain no-ops on
	 * IFACE_USB), so routing through zephcore_ble_send would strand the frame. */
	if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
		/* A text-CLI session owns USB — swallow binary V3 output so the serial
		 * console never sees framed garbage. Report success so callers don't
		 * retry/back off on a frame we intentionally dropped. */
		if (zephcore_usb_companion_is_text_session()) {
			return len;
		}
		return zephcore_usb_companion_write_frame(src, len);
	}
#endif
	return zephcore_ble_send(src, (uint16_t)len);
}

/**
 * Push notification callback - sends push frames to BLE.
 * Push frames have format: [push_code] [data...]
 */
static void push_callback(uint8_t code, const uint8_t *data, size_t len)
{
	/* No point serializing if nobody is listening. A USB companion session
	 * is "connected" too, but zephcore_ble_is_connected() only reports the BLE
	 * link — without the USB check, pushes (new messages, etc.) are silently
	 * dropped on a USB-attached client. */
	bool transport_up = zephcore_ble_is_connected();
#if ZEPHCORE_USB_STACK
	/* A text-CLI session is not a binary companion — don't push V3 frames to it. */
	transport_up = transport_up ||
		(zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB &&
		 !zephcore_usb_companion_is_text_session());
#endif
	if (!transport_up) return;

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
/* Parse inbound BLE/USB binary frames on the MAIN thread.  Called from the
 * event loop on MESH_EVENT_BLE_RX (formerly a sysworkq work item — moved to
 * the main thread so handleProtocolFrame's mesh-state mutation can't race
 * loop()). */
static void process_companion_rx(void)
{
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

#if ZEPHCORE_USB_STACK
	/* Over USB, write_frame is synchronous and never kicks the BLE
	 * tx-drain/on_tx_idle pump that paces multi-frame responses (e.g. contact
	 * sync, which only emits PACKET_CONTACT_START in the command handler and
	 * relies on continueContactIteration() for the rest). Kick the pump here so
	 * any iteration started by the command(s) just handled actually proceeds;
	 * the USB branch of tx_drain fires on_tx_idle and the loop self-sustains. */
	if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
		zephcore_ble_kick_tx();
	}
#endif
}

/* Continue the contact-dump iteration when TX has space.  Runs on the MAIN
 * thread (driven by MESH_EVENT_CONTACT_ITER, posted from the BLE/USB tx-idle
 * callbacks) so reading the contact table never races a main-thread
 * addContact() from an inbound advert. */
static void run_contact_iteration(void)
{
#ifdef ZEPHCORE_LORA
	if (!companion_mesh_ptr) {
		return;
	}

#if ZEPHCORE_USB_STACK
	if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
		/* Event-driven pacing (no fixed delay): fill the USB TX ring as far
		 * as it'll hold, then stop. The CDC TX ISR drains it to the wire at
		 * full speed and calls usb_on_tx_drain() when empty, which re-kicks
		 * this work to queue the next batch — real backpressure, like BLE's
		 * on_tx_idle. tx_has_space() guarantees each continueContactIteration
		 * frame fits, so writeFrame never fails mid-dump. */
		while (zephcore_usb_companion_tx_has_space(CONTACT_FRAME_SIZE)) {
			if (!companion_mesh_ptr->continueContactIteration()) {
				break;  /* iteration complete */
			}
		}
		return;
	}
#endif

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

		/* Parse inbound BLE/USB frames + USB text-CLI lines HERE (main
		 * thread) before loop() drains any outbound they enqueued — keeps
		 * all mesh-state mutation on one thread (see ble_on_rx_frame). */
		if (events & MESH_EVENT_BLE_RX) {
			process_companion_rx();
#if ZEPHCORE_USB_STACK
			struct companion_cli_line c;
			while (k_msgq_get(&companion_cli_queue, &c, K_NO_WAIT) == 0) {
				companion_cli_run(c.buf);
			}
#endif
		}

		/* Continue contact-dump iteration on the main thread (see
		 * run_contact_iteration / MESH_EVENT_CONTACT_ITER). */
		if (events & MESH_EVENT_CONTACT_ITER) {
			run_contact_iteration();
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

			/* BLE advertising watchdog — if bt_le_adv_start failed
			 * transiently (HCI timeout, controller pacing) the device
			 * would silently stop advertising and be undiscoverable
			 * until next reboot.  Cheap to nudge it back here. */
			if (zephcore_ble_is_enabled() && !zephcore_ble_is_connected() && !zephcore_ble_is_advertising()) {
				LOG_WRN("BLE adv watchdog: not advertising, re-enabling");
				zephcore_ble_set_enabled(true);
			}

			mesh_housekeeping_ui_refresh();

			/* Low-battery auto-shutdown (companion only). Self-throttled
			 * and compiled out unless the board sets a threshold — no
			 * extra poll, just a cheap call on the existing tick. */
			ui_auto_shutdown_check();
		}

		/* Off-main pref mutators (e.g. gps_fix_callback in modem_chat
		 * context) post this bit and update _prefs in memory; we flush
		 * to flash here so the synchronous LittleFS write doesn't block
		 * the originating thread.  Multiple posts coalesce into one
		 * write of the latest _prefs values — desired behaviour. */
		if ((events & MESH_EVENT_PREFS_DIRTY) && companion_mesh_ptr) {
			save_prefs_to_flash();
		}
#endif

		/* Off-main RTC write request (gps_fix_callback runs in modem_chat
		 * context — see request_rtc_save()). Perform the blocking I2C
		 * write here on the main thread instead. */
		if (events & MESH_EVENT_RTC_SAVE) {
			zephcore_rtc_save((uint32_t)atomic_get(&pending_rtc_epoch));
		}

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
		if (events & MESH_EVENT_JOYSTICK_LOOP) {
			joystick_ui_task.loop();
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

/* Defined here (after data_store + companion_mesh_ptr statics) and
 * forward-declared near the top of the file so mesh_event_loop() can
 * call it without the static decls being in scope. */
static void save_prefs_to_flash(void)
{
	data_store.savePrefs(companion_mesh_ptr->prefs);
}

/* ========== Companion CLI (USB text sideband only) ==========
 * Text CLI lives on USB CDC only: its '<' sync byte cleanly separates binary
 * V3 frames from text, whereas BLE NUS frames have no prefix and V3 opcodes
 * overlap printable ASCII, so a BLE text sideband can't be disambiguated. */
#if ZEPHCORE_USB_STACK

class CompanionCLICallbacks : public CommonCLICallbacks {
public:
	void savePrefs() override {
		data_store.savePrefs(companion_mesh.prefs);
	}
	const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
	const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
	const char* getRole() override { return "companion"; }
	bool formatFileSystem() override { return data_store.formatFileSystem(); }

	/* Advert / timer controls — mesh-internal; stub for now. */
	void sendSelfAdvertisement(int delay_millis, bool flood) override {
		(void)delay_millis; (void)flood;
	}
	void updateAdvertTimer() override {}
	void updateFloodAdvertTimer() override {}

	/* Log control — no log file on companion. */
	void setLoggingOn(bool enable) override { (void)enable; }
	void eraseLogFile() override {}
	void dumpLogFile() override {}

	/* TX power — Zephyr LoRa driver has no runtime API; log only (matches repeater). */
	void setTxPower(int8_t power_dbm) override {
		LOG_INF("TX power %d dBm requested (reboot to apply)", power_dbm);
	}

#ifdef CONFIG_ZEPHCORE_APC
	int8_t getAPCReduction() const override {
		return companion_mesh.getAPCReduction();
	}
	float getAPCMargin() const override {
		return companion_mesh.getAPCMargin();
	}
	bool isAPCEnabled() const override {
		return companion_mesh.isAPCEnabled();
	}
	void setAPCEnabled(bool en) override {
		companion_mesh.setAPCEnabled(en);
	}
	uint8_t getAPCTargetMargin() const override {
		return companion_mesh.getAPCTargetMargin();
	}
	void setAPCTargetMargin(uint8_t margin_db) override {
		companion_mesh.setAPCTargetMargin(margin_db);
	}
#endif

	mesh::LocalIdentity& getSelfId() override { return companion_mesh.self_id; }

	void saveIdentity(const mesh::LocalIdentity& new_id) override {
		companion_mesh.self_id = new_id;
		data_store.saveMainIdentity(new_id);
	}

	void clearStats() override {
		lora_radio.resetStats();
		companion_mesh.resetStats();
	}

	/* Temp radio params — deferred; stub for now. */
	void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr,
				  int timeout_mins) override {
		(void)freq; (void)bw; (void)sf; (void)cr; (void)timeout_mins;
	}
};

static CompanionCLICallbacks companion_cli_cbs;
static ClientACL companion_acl;  /* unused by CommonCLI but required by constructor */
static CommonCLI companion_cli(zephyr_board, rtc_clock, companion_acl,
			       &companion_mesh.prefs, &companion_cli_cbs);

/* Dispatch a CLI text line from USB, reply back over USB CDC. Output mirrors
 * the repeater's serial CLI ("\r\n  -> <reply>\r\n", CRLF) so the
 * flasher.meshcore.io serial console renders companion replies identically. */
#if defined(CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS) && \
	CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS > 0
/* Companion-only CLI sideband for the low-battery auto-shutdown threshold.
 * Handled here (not in shared CommonCLI) so the pref persists via the
 * companion datastore and the command never appears on repeater builds.
 * Returns true if the line was a recognised autoshutdown command. */
static bool handle_autoshutdown_cli(const char *line, char *reply)
{
	if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
		/* No global CLI help exists; list only the companion-specific extras
		 * and make clear the standard set/get radio+mesh commands also work. */
		strcpy(reply,
		       "Auto-shutdown (plus the standard set/get commands):\r\n"
		       "  get autoshutdown      - show low-battery cutoff\r\n"
		       "  set autoshutdown <mV> - 0 = off, or 1-5000 mV");
		return true;
	}
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
		/* Numbers only: first char must be a digit (rejects sign, letters,
		 * empty), and nothing but trailing whitespace may follow. */
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
		/* Defer the flash write to the main thread (this runs on sysworkq via
		 * the USB RX work handler). A synchronous savePrefs here could race a
		 * concurrent main-thread save (e.g. GPS-fix MESH_EVENT_PREFS_DIRTY) on
		 * the same prefs .tmp file. Posting the event coalesces both onto main. */
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
#endif

/* Executes a text-CLI line — runs on the MAIN thread (drained from
 * companion_cli_queue in the event loop).  CommonCLI::handleCommand touches
 * mesh state shared with loop(), so it must not run on sysworkq. */
static void companion_cli_run(const char *line)
{
	char reply[CLI_REPLY_SIZE];
	reply[0] = '\0';
#if defined(CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS) && \
	CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS > 0
	if (handle_autoshutdown_cli(line, reply)) {
		zephcore_usb_companion_write_text("\r\n  -> ", 7);
		zephcore_usb_companion_write_text(reply, strlen(reply));
		zephcore_usb_companion_write_text("\r\n", 2);
		return;
	}
#endif
	companion_cli.handleCommand(0, line, reply);
	if (reply[0] != '\0') {
		zephcore_usb_companion_write_text("\r\n  -> ", 7);
		zephcore_usb_companion_write_text(reply, strlen(reply));
	}
	zephcore_usb_companion_write_text("\r\n", 2);
}

/* USB-adapter callback (runs on sysworkq).  Just queue the line and wake the
 * main thread — the actual handleCommand happens in companion_cli_run(). */
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
		time_sync_report(TIME_SYNC_GPS);
		/* Defer the hardware-RTC write to the main thread — see
		 * request_rtc_save()/pending_rtc_epoch: blocking I2C here would
		 * stall NMEA ingest, same hazard as the prefs flash write below. */
		request_rtc_save((uint32_t)utc_time);
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
		/* Defer flash write to main thread — gps_fix_callback runs in
		 * the GNSS modem_chat worker; a synchronous LittleFS write here
		 * (10-50 ms on nRF52 QSPI) would block NMEA ingest and risk
		 * UART buffer overflow / lost fixes.  Main handles the save
		 * via MESH_EVENT_PREFS_DIRTY; multiple fixes coalesce into one
		 * write (the latest prefs values are written). */
		k_event_post(&mesh_events, MESH_EVENT_PREFS_DIRTY);
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
	/* Clear any stale bootloader magic from previous sessions */
	zephyr_board.clearBootloaderMagic();

	/* USB CDC init up front so the host can enumerate, then block until the
	 * host opens the port (DTR-high) — event-driven via the shared usbd
	 * message callback.  Unplugged → 1 s timeout, banner is buffered for any
	 * later attach. */
#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	!IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT) && \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))
	zephcore_usbd_init();
	zephcore_usbd_wait_dtr(1000);
#endif
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

	/* Initialize sensor manager (GPS, environment sensors) */
	sensor_manager_init();

	/* Restore wall-clock time from a battery-backed hardware RTC if one is
	 * present on I2C. Shown tagged "L" (local) until the next GPS/app/CLI
	 * sync; no-op on boards without an RTC. */
	{
		uint32_t rtc_epoch;
		if (zephcore_rtc_restore(&rtc_epoch)) {
			rtc_clock.setCurrentTime(rtc_epoch);
		}
	}

	/* Initialize UI subsystem (buttons, buzzer, display).
	 * If display is present, ui_init() handles display init + auto-off.
	 * If no display, fall back to raw OLED sleep for power saving. */
	ui_init();
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
	joystick_ui_hooks_register(&joystick_ui_task, joystick_signal_refresh, joystick_signal_tx);
#endif
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
	companion_mesh.prefs.airtime_factor = 9.0f; /* Arduino formula: 100/(af+1) → 10% (EU 868 default) */
	companion_mesh.prefs.rx_duty_cycle = 0;     /* Default OFF: continuous RX */
	companion_mesh.prefs.rx_boost = 1;          /* Default: boosted RX (+3dB sensitivity, +2mA) */
	companion_mesh.prefs.apc_enabled = 0;       /* Default: APC off */
	companion_mesh.prefs.apc_margin = 20;       /* Companions: more conservative margin (mobile) */
	companion_mesh.prefs.auto_shutdown_mv = CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS; /* low-batt cutoff (0=off) */
	companion_mesh.prefs.gps_interval = CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC; /* 5-min duty cycle (0=always-on) */

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

	/* Load or generate identity. First-boot keygen runs the layered
	 * entropy mixer + Ed25519 derive + reserved-prefix guard inside
	 * ZephyrRNG::generateFirstBootIdentity. */
	mesh::LocalIdentity self_identity;
	if (!data_store.loadMainIdentity(self_identity)) {
		mesh::ZephyrRNG::generateFirstBootIdentity(self_identity);
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
	ui_set_battery_provider(get_battery_mv);
	ui_set_power_source_provider([]() { return zephyr_board.isExternalPowered(); });
	ui_set_auto_shutdown_mv(companion_mesh.prefs.auto_shutdown_mv);
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
		lora_radio.getActiveFrequencyHz(),
		lora_radio.getActiveSpreadingFactor(),
		lora_radio.getActiveBandwidthKHzX10(),
		lora_radio.getActiveCodingRate(),
		lora_radio.getConfiguredTxPower(),
		lora_radio.getNoiseFloor());
#ifdef CONFIG_ZEPHCORE_APC
	ui_set_radio_runtime(
		lora_radio.getEffectiveTxPower(),
		companion_mesh.isAPCEnabled(),
		companion_mesh.getAPCReduction(),
		(int16_t)(companion_mesh.getAPCMargin() * 10.0f),
		companion_mesh.getAPCTargetMargin(),
		lora_radio.getActiveSyncWord(),
		lora_radio.getActivePreambleLength(),
		lora_radio.isRxDutyCycleEnabled(),
		lora_radio.isRadioReady(),
		lora_radio.isInRecvMode(),
		lora_radio.isTxActive());
#else
	ui_set_radio_runtime(
		lora_radio.getEffectiveTxPower(),
		false, 0, 0, companion_mesh.prefs.apc_margin,
		lora_radio.getActiveSyncWord(),
		lora_radio.getActivePreambleLength(),
		lora_radio.isRxDutyCycleEnabled(),
		lora_radio.isRadioReady(),
		lora_radio.isInRecvMode(),
		lora_radio.isTxActive());
#endif
	ui_set_radio_stats(lora_radio.getPacketsRecv(),
			   lora_radio.getPacketsSent(),
			   lora_radio.getPacketsRecvErrors());
	ui_set_battery(zephyr_board.getBattMilliVolts(), 0);
	ui_set_gps_available(gps_is_available());
	ui_set_gps_enabled(companion_mesh.prefs.gps_enabled != 0);
	ui_set_ble_enabled(companion_mesh.prefs.ble_disabled != 1);  /* BLE starts advertising at boot */

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
		/* Apply persisted GPS duty interval (0 = always on) before the
		 * state machine starts. */
		gps_set_poll_interval_sec(companion_mesh.prefs.gps_interval);

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
#ifdef CONFIG_ZEPHCORE_APC
	ui_set_radio_runtime(
		lora_radio.getEffectiveTxPower(),
		companion_mesh.isAPCEnabled(),
		companion_mesh.getAPCReduction(),
		(int16_t)(companion_mesh.getAPCMargin() * 10.0f),
		companion_mesh.getAPCTargetMargin(),
		lora_radio.getActiveSyncWord(),
		lora_radio.getActivePreambleLength(),
		lora_radio.isRxDutyCycleEnabled(),
		lora_radio.isRadioReady(),
		lora_radio.isInRecvMode(),
		lora_radio.isTxActive());
#else
	ui_set_radio_runtime(
		lora_radio.getEffectiveTxPower(),
		false, 0, 0, companion_mesh.prefs.apc_margin,
		lora_radio.getActiveSyncWord(),
		lora_radio.getActivePreambleLength(),
		lora_radio.isRxDutyCycleEnabled(),
		lora_radio.isRadioReady(),
		lora_radio.isInRecvMode(),
		lora_radio.isTxActive());
#endif

	/* Restore runtime ADC multiplier override (0 = keep DT default) */
	zephyr_board.setAdcMultiplier(companion_mesh.prefs.adc_multiplier);

	/* Initialize mesh event object */
	k_event_init(&mesh_events);

	/* Initialize UI mesh actions module (pass mesh objects for deferred actions) */
	ui_mesh_actions_init(&mesh_events, MESH_EVENT_UI_ACTION,
			     &companion_mesh, &data_store,
			     &lora_radio, &zephyr_board, &rtc_clock);

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
	joystick_ui_task.begin(&companion_mesh, &rtc_clock, &companion_mesh.prefs);
#endif
#endif

	/* Initialize BLE adapter (registers auth callbacks) */
	zephcore_ble_init(&ble_cbs);

	/*
	 * USB CDC ACM companion transport.
	 * Enabled when logging is on (debug builds) OR when explicitly requested
	 * via CONFIG_ZEPHCORE_COMPANION_USB (e.g. prod builds on USB-capable boards).
	 */
#if ZEPHCORE_USB_STACK
	zephcore_usb_companion_init(&mesh_events, MESH_EVENT_BLE_RX,
				   &zephyr_board);
	/* Mirror BLE connect/disconnect UI + cleanup for USB sessions. */
	zephcore_usb_companion_set_session_start_cb(usb_on_session_start);
	zephcore_usb_companion_set_session_end_cb(usb_on_session_end);
	/* Resume the contact pump when the USB TX ring drains (≈ BLE on_tx_idle). */
	zephcore_usb_companion_set_tx_drain_cb(usb_on_tx_drain);
	/* Text CLI sideband — activates when first byte is not '<'. */
	zephcore_usb_companion_set_cli_line_cb(companion_cli_dispatch);
#endif

#if IS_ENABLED(CONFIG_BT)
	if (bt_enable(bt_ready) != 0) {
		LOG_ERR("bt_enable failed");
	}
#else
	/* No BLE controller — TCP companion transport starts itself.
	 * zephcore_ble_start() is provided by LinuxTCPTransport.c when
	 * CONFIG_ZEPHCORE_TRANSPORT_TCP=y. */
	zephcore_ble_start(companion_mesh.getDeviceName());
#endif

	/*
	 * FULLY EVENT-DRIVEN architecture: main thread runs mesh event loop.
	 * Mesh loop wakes IMMEDIATELY on events - NO POLLING for TX completion!
	 *
	 * Event sources:
	 *   - MESH_EVENT_LORA_RX: LoRa packet received (from RX async callback)
	 *   - MESH_EVENT_LORA_TX_DONE: LoRa TX complete (from TX poll work -> callback)
	 *   - MESH_EVENT_HOUSEKEEPING: Periodic maintenance (noise floor, etc.)
	 *   - MESH_EVENT_BLE_RX: BLE/USB frame (or USB text-CLI line) ready to parse.
	 *     The BLE callback / USB adapter only assemble + queue the frame off-main;
	 *     process_companion_rx() (and companion_cli_run()) parse it HERE on the
	 *     main thread so mesh-state mutation never races loop().
	 *   - MESH_EVENT_TX_DRAIN: outbound LoRa packet queued — wakes loop() to drain
	 *   - MESH_EVENT_UI_ACTION / GPS_ACTION / PREFS_DIRTY: deferred work
	 *     from non-main threads
	 *
	 * Other processing still uses work queues:
	 *   - BLE TX: write_frame -> tx_drain_work (event-driven via notify callback)
	 *   - Contact iteration: MESH_EVENT_CONTACT_ITER -> run_contact_iteration()
	 *     on the main thread, paced by the BLE/USB tx-idle callbacks
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
