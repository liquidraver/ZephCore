/*
 * SPDX-License-Identifier: MIT
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
#include "led_gate.h"

/* BLE controller assert handler — BT is compiled even for repeater (via zephcore_common.conf) */
#if IS_ENABLED(CONFIG_BT_CTLR_ASSERT_HANDLER)
extern "C" void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	LOG_ERR("!!! BLE CONTROLLER ASSERT: %s:%u !!!", file ? file : "?", line);
	k_sleep(K_MSEC(100));
	sys_reboot(SYS_REBOOT_COLD);
}
#endif

/* USB CDC ACM init + 1200-baud DFU + DTR callbacks (shared with companion).
 *
 * Gate on the CDC-ACM class driver, NOT on DT_HAS_COMPAT_STATUS_OKAY alone: a
 * board overlay may expose a cdc_acm_uart DT node unconditionally (the shared
 * esp32s3_usb_otg.dtsi does), so the node can be present while the class driver
 * — and therefore zephcore_usbd_* and the device object — is not compiled
 * (e.g. an ESP32-S3 repeater built without esp32s3_usb.conf). This mirrors the
 * repeater CMake condition that compiles ZephyrUSBCDC.cpp. */
#define ZEPHCORE_USB_STACK \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))

#if ZEPHCORE_USB_STACK && !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
#include <ZephyrUSBCDC.h>
#endif

#include <app/RepeaterDataStore.h>
#include <app/RepeaterMesh.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <adapters/clock/ZephyrRTCDiscover.h>
#include <ZephyrSensorManager.h>

/* UI subsystem (display, buttons, buzzer) */
#include "ui_task.h"

/* Headless repeaters link the weak no-op ui_* stubs (ui_headless_stubs.c), so
 * the periodic UI refresh in the maintenance pass is pure work for nothing on
 * them.  Guard the hot path on a real ui_* implementation being linked; the
 * one-shot calls at init and in the CLI reply path stay unguarded, matching
 * the rest of the file.
 *
 * This MUST mirror the CMake condition that selects the stubs, not the display
 * devicetree node: ZEPHCORE_UI_DESIGN_BUTTON is enabled by BUTTONS *or*
 * DISPLAY *or* BUZZER, so a board with buttons/buzzer and no panel still links
 * the real UI and still needs these updates. */
#define ZEPHCORE_HAS_UI (IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_BUTTON) || \
			 IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK))

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
#define MESH_EVENT_MAINTENANCE   BIT(3)  /* A maintenance deadline came due */
#define MESH_EVENT_GPS_ACTION    BIT(4)  /* GPS state change (must run on main thread!) */
#define MESH_EVENT_TX_DRAIN      BIT(5)  /* Outbound packet delay expired, run checkSend */
#define MESH_EVENT_RTC_SAVE      BIT(6)  /* Hardware-RTC write requested off-main */
#define MESH_EVENT_INIT_ADVERT   BIT(7)  /* Deferred boot advert — send on main thread */
#define MESH_EVENT_WAKE          BIT(8)  /* Off-main state set; run loop() promptly */
#define MESH_EVENT_ALL           (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | MESH_EVENT_CLI_RX | MESH_EVENT_MAINTENANCE | MESH_EVENT_GPS_ACTION | MESH_EVENT_TX_DRAIN | MESH_EVENT_RTC_SAVE | MESH_EVENT_INIT_ADVERT | MESH_EVENT_WAKE)

/* Maintenance is deadline-driven, not periodic: after every pass the loop asks
 * the mesh when its soonest pending deadline is (msUntilNextMaintenance) and
 * arms a single one-shot wake for exactly that moment.  An idle repeater with
 * nothing scheduled therefore sleeps until its next real deadline instead of
 * waking on a fixed cadence.
 *
 * MAINTENANCE_BACKSTOP_MS bounds that: it caps how long we will go without a
 * pass even when everything reports idle, so a deadline that is missed or
 * mis-reported degrades to the old behaviour instead of wedging.
 * MAINTENANCE_MIN_MS floors it, so an item that is due-but-blocked (radio
 * mid-packet, duty-cycle sleep window) re-arms shortly rather than spinning. */
#define MAINTENANCE_BACKSTOP_MS CONFIG_ZEPHCORE_MAINTENANCE_BACKSTOP_MS
#define MAINTENANCE_MIN_MS      50

/* Event object for mesh loop */
static struct k_event mesh_events;

/* Pending epoch for a deferred zephcore_rtc_save() — gps_fix_callback runs on
 * the GNSS modem_chat worker thread, where the RTC's blocking I2C transactions
 * would stall NMEA ingest. Stash the latest epoch and let the main thread
 * perform the write; concurrent posts coalesce into one save. */
static atomic_t pending_rtc_epoch = ATOMIC_INIT(0);

static void request_rtc_save(uint32_t epoch)
{
	atomic_set(&pending_rtc_epoch, (atomic_val_t)epoch);
	k_event_post(&mesh_events, MESH_EVENT_RTC_SAVE);
}

/* USB CDC state */
static const struct device *usb_dev;
static uint8_t usb_ring_buf_data[USB_RING_BUF_SIZE];
static struct ring_buf usb_ring_buf;
static char cli_line_buf[CLI_LINE_BUF_SIZE];
static char cli_reply_buf[256];
static uint16_t cli_line_idx;

/* Completed CLI lines are handed to the MAIN thread for execution.  Byte
 * assembly + echo (cli_rx_work_fn) runs on sysworkq and touches no mesh
 * state, but handleCommand() mutates the lock-free packet pool / dispatcher
 * that loop() also touches — running it on sysworkq races the main loop.
 * So we queue the finished line and let the event loop run handleCommand(). */
struct cli_cmd_line { char buf[CLI_LINE_BUF_SIZE]; };
K_MSGQ_DEFINE(cli_cmd_queue, sizeof(struct cli_cmd_line), 4, 4);

/* Work items for event-driven processing */
static void cli_rx_work_fn(struct k_work *work);
static void maintenance_timer_fn(struct k_timer *timer);
static void tx_drain_work_fn(struct k_work *work);
static void initial_advert_work_fn(struct k_work *work);
K_WORK_DEFINE(cli_rx_work, cli_rx_work_fn);
K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);
K_WORK_DELAYABLE_DEFINE(initial_advert_work, initial_advert_work_fn);

/* One-shot maintenance wake, re-armed after every loop pass (see
 * arm_maintenance_wake).  Not periodic — the period is the deadline. */
K_TIMER_DEFINE(maintenance_timer, maintenance_timer_fn, NULL);

/* Forward declarations */
#ifdef ZEPHCORE_LORA
static RepeaterMesh *repeater_mesh_ptr;
static void refresh_repeater_ui_radio_state(void);
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

				/* Hand the command to the main thread (see cli_cmd_queue).
				 * The reply + trailing newline are emitted there, exactly
				 * matching the previous inline output order. */
				struct cli_cmd_line c;
				strncpy(c.buf, cli_line_buf, sizeof(c.buf) - 1);
				c.buf[sizeof(c.buf) - 1] = '\0';
				if (k_msgq_put(&cli_cmd_queue, &c, K_NO_WAIT) == 0) {
					k_event_post(&mesh_events, MESH_EVENT_CLI_RX);
				} else {
					cli_print("\r\n  -> busy\r\n");
				}
				cli_line_idx = 0;
			} else {
				/* Empty line — just emit the newline (no command to run) */
				cli_print("\r\n");
			}
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

#ifdef ZEPHCORE_LORA
/* Run queued CLI commands on the MAIN thread (see cli_cmd_queue).  Output
 * order matches the old inline path exactly: "\r\n  -> reply" when there is
 * a reply, then a trailing "\r\n". */
static void process_cli_commands(void)
{
	struct cli_cmd_line c;
	while (repeater_mesh_ptr && k_msgq_get(&cli_cmd_queue, &c, K_NO_WAIT) == 0) {
		cli_reply_buf[0] = '\0';
		repeater_mesh_ptr->handleCommand(0, c.buf, cli_reply_buf);
		refresh_repeater_ui_radio_state();
		if (cli_reply_buf[0] != '\0') {
			cli_print("\r\n  -> ");
			cli_print(cli_reply_buf);
		}
		cli_print("\r\n");
	}
}
#endif

/* Maintenance deadline expired — wake the mesh loop to run the pass. */
static void maintenance_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_MAINTENANCE);
}

/* Ask the mesh for its soonest pending deadline and arm the one-shot for it.
 * Called after every loop pass, whatever woke us: any event may have created
 * or cleared a deadline (a queued advert, a CLI tempradio command, a CAD probe
 * that just ran), so the schedule is recomputed from scratch each time rather
 * than tracked incrementally.
 *
 * The backstop below is an unconditional CEILING on the wait, not a fallback
 * used only when everything reports idle.  That means it must stay well above
 * the shortest legitimate recurring deadline (the noise-floor sampler, and the
 * CAD probe) or it becomes the effective period and the deadline scheduling
 * buys nothing — see ZEPHCORE_MAINTENANCE_BACKSTOP_MS. */
static void arm_maintenance_wake(void)
{
	uint32_t delay = MAINTENANCE_BACKSTOP_MS;
	uint32_t next = MAINTENANCE_BACKSTOP_MS;

#ifdef ZEPHCORE_LORA
	if (repeater_mesh_ptr) {
		next = repeater_mesh_ptr->msUntilNextMaintenance();

		if (next < delay) {
			delay = next;
		}
	}
#endif

	if (delay < MAINTENANCE_MIN_MS) {
		delay = MAINTENANCE_MIN_MS;
	}

	/* Both figures, because which one is binding is the whole diagnosis:
	 *   next=15000 armed=15000  — a real deadline won; working as intended
	 *   next=15000 armed=5000   — the backstop is clamping (it must stay
	 *                             above the noise-floor/CAD intervals)
	 *   next=0     armed=50     — a deadline reports due but its state is
	 *                             not advancing; repeated = spin
	 * mesh::MAINTENANCE_IDLE (0x7FFFFFFF) as `next` means nothing pending. */
	LOG_DBG("maint: next=%u armed=%u", (unsigned)next, (unsigned)delay);

	k_timer_start(&maintenance_timer, K_MSEC(delay), K_NO_WAIT);
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

/* Deferred initial advertisement — gives GPS time to get a fix at boot.
 * Runs on sysworkq (the 10 s delay timer), so it must NOT send the advert
 * here: sendSelfAdvertisement() mutates the packet pool / dispatcher shared
 * with loop().  Post an event and let the main thread send it. */
static void initial_advert_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_event_post(&mesh_events, MESH_EVENT_INIT_ADVERT);
}

static void tx_queued_callback(uint32_t delay_ms, void *user_data)
{
	ARG_UNUSED(user_data);
	k_work_reschedule(&tx_drain_work, K_MSEC(delay_ms));
}

/* Off-main code (MQTT CONNACK, SNTP) set state that loop() must drain.  Post
 * only — this runs on the MQTT publisher / WiFi thread, and k_event_post is
 * the sole thing safe to do from there. */
static void wake_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_WAKE);
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
		/* Defer the hardware-RTC write to the main thread — blocking I2C
		 * here would stall NMEA ingest (gps_fix_callback runs in the
		 * GNSS modem_chat worker context). */
		request_rtc_save((uint32_t)utc_time);
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

static uint16_t get_battery_mv(void)
{
	return zephyr_board.getBattMilliVolts();
}

/* Radio is constructed with no prefs pointer; main() binds it to
 * repeater_mesh._prefs via setPrefs() before repeater_mesh.begin(). */

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
/* LR1110 via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LR1110Radio lora_radio(lora_dev, zephyr_board);
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_SX127X)
/* SX127x via Zephyr loramac-node driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX127xRadio lora_radio(lora_dev, zephyr_board);
#else
/* SX126x via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX126xRadio lora_radio(lora_dev, zephyr_board);
#endif

static mesh::ZephyrMillisecondClock ms_clock;
static mesh::ZephyrRNG zephyr_rng;
static mesh::SimpleMeshTables mesh_tables;

/* RepeaterMesh requires: board, radio, ms_clock, rng, rtc, tables */
static RepeaterMesh repeater_mesh(zephyr_board, lora_radio, ms_clock, zephyr_rng, rtc_clock, mesh_tables);

/* Strong override of the weak stub in ui_mesh_actions_stubs.c: keep the prefs
 * copy in step when the LEDs page toggles, so "get leds" reports what the node
 * is actually doing. RAM only — a repeater has no deferred-save path off the UI
 * thread, so a UI toggle lasts until reboot; "set leds" is what persists. */
extern "C" void mesh_set_leds_disabled(bool disabled)
{
	if (repeater_mesh_ptr) {
		repeater_mesh_ptr->getNodePrefs()->leds_disabled = disabled ? 1 : 0;
	}
}

static void refresh_repeater_ui_radio_state(void)
{
	if (!repeater_mesh_ptr) {
		return;
	}

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

	ui_set_radio_stats(
		lora_radio.getPacketsRecv(),
		lora_radio.getPacketsSent(),
		lora_radio.getPacketsRecvErrors());
}
#endif

/* Repeater event loop */
static void repeater_event_loop(void)
{
	LOG_INF("starting event-driven loop");

	/* Print startup banner (no prompt - Arduino style) */
	cli_print("\r\n=== ZephCore Repeater ===\r\n");

	/* Arm the first maintenance wake; every pass below re-arms it. */
	arm_maintenance_wake();

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
		/* Run queued CLI commands here (main thread) BEFORE loop() drains
		 * any outbound packets they enqueued — keeps all mesh-state
		 * mutation on the main thread (see cli_cmd_queue). */
		if (events & MESH_EVENT_CLI_RX) {
			process_cli_commands();
		}

		/* Deferred boot advert — sent on the main thread (see
		 * initial_advert_work_fn); loop() below drains the queued packet. */
		if (repeater_mesh_ptr && (events & MESH_EVENT_INIT_ADVERT)) {
			LOG_INF("Sending deferred initial advertisement");
			repeater_mesh_ptr->sendSelfAdvertisement(500, false);
		}

		/* Packet processing — only on radio/CLI/TX events */
		if (repeater_mesh_ptr &&
		    (events & (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE |
			       MESH_EVENT_CLI_RX | MESH_EVENT_TX_DRAIN |
			       MESH_EVENT_WAKE))) {
			repeater_mesh_ptr->loop();
		}
#endif

#ifdef ZEPHCORE_LORA
		/* Radio maintenance runs OPPORTUNISTICALLY, on every pass, whatever
		 * woke us — not only when its own timer fired.
		 *
		 * Every item inside is deadline-gated internally, so this is nearly
		 * free when nothing is due.  The point is what it does to a busy
		 * node: a hilltop repeater is already waking constantly for packets,
		 * so maintenance rides along on those wakes, its deadlines advance,
		 * and arm_maintenance_wake() below keeps pushing the timer out — the
		 * maintenance timer then almost never fires and costs no wakes at
		 * all.  Running it only on its own event (as this did originally)
		 * inverted that: the busiest nodes, which can least afford it, paid
		 * the full timer cadence on top of their packet wakes, and every
		 * blocked sample burned a retry against a channel that is busy for
		 * sustained reasons (real traffic in isReceiving(), the RSSI
		 * prefilter) rather than the transient duty-cycle sleep window the
		 * retries were sized for.
		 *
		 * Ordering matters: this sits AFTER packet processing so an inbound
		 * frame is handled before we spend SPI time on an RSSI sweep. */
		if (repeater_mesh_ptr) {
			repeater_mesh_ptr->maintenanceLoop();
		}
#endif

		/* A maintenance deadline came due — run the time-based work too. */
		if (events & MESH_EVENT_MAINTENANCE) {
#ifdef ZEPHCORE_LORA
			/* Drive loop() so time-based actions (advert timers,
			 * tempradio set/revert, contacts flush, uplink status)
			 * still fire when no LoRa/CLI traffic wakes the loop. */
			if (repeater_mesh_ptr) {
				repeater_mesh_ptr->loop();
			}
#endif

#if ZEPHCORE_HAS_UI
			ui_set_clock(rtc_clock.getCurrentTime());

#ifdef ZEPHCORE_LORA
			/* Refresh live radio state (noise floor, TX power
			 * reduction, RX/TX mode, packet counters).  Headless
			 * repeaters compile this out entirely — every ui_set_*
			 * below it is a weak no-op there (ui_headless_stubs.c),
			 * so the whole block was pure work for nothing. */
			refresh_repeater_ui_radio_state();

			/* Battery is now refreshed lazily from ui_pages_render() with
			 * a 30 s freshness guard — no periodic ADC fire here. */
#endif
#endif /* ZEPHCORE_HAS_UI */
		}

		/* Off-main RTC write request (gps_fix_callback runs in modem_chat
		 * context — see request_rtc_save()). Perform the blocking I2C
		 * write here on the main thread instead. */
		if (events & MESH_EVENT_RTC_SAVE) {
			zephcore_rtc_save((uint32_t)atomic_get(&pending_rtc_epoch));
#ifdef ZEPHCORE_LORA
			/* GPS just set the clock — arm the mesh time-sync drift envelope. */
			if (repeater_mesh_ptr) {
				repeater_mesh_ptr->noteGPSTimeSync();
			}
#endif
		}

		/* Re-arm for the soonest deadline this pass left behind.  Done for
		 * every wake, not just maintenance ones: a CLI command, an inbound
		 * packet or a GPS fix can all create or clear a deadline. */
		arm_maintenance_wake();
	}
}

int main(void)
{
#ifdef ZEPHCORE_LORA
	/* Clear any stale bootloader magic from previous sessions.
	 * Prevents nRF52 boards from re-entering bootloader after reboot. */
	zephyr_board.clearBootloaderMagic();
#endif

	/* USB CDC init up front so the host can enumerate, then wait for the
	 * host to open the port (DTR asserted) — event-driven via the usbd
	 * message callback.  Unplugged → 2 s timeout, no banner; attached →
	 * banner reaches the user the moment the port opens. */
#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	!IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT) && \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))
	zephcore_usbd_init();
	zephcore_usbd_wait_dtr(2000);
#endif
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

	/* Restore wall-clock time from a battery-backed hardware RTC if present
	 * (shown tagged "L" until the next GPS/CLI sync; no-op if no RTC). */
	{
		uint32_t rtc_epoch;
		if (zephcore_rtc_restore(&rtc_epoch)) {
			rtc_clock.setCurrentTime(rtc_epoch);
		}
	}

	/* Set GPS to repeater mode: power off now, wake every 48h for time sync only.
	 * This prevents GPS from draining power on boards that have it (e.g., Wio Tracker). */
	if (gps_is_available()) {
		gps_set_fix_callback(gps_fix_callback);
		gps_set_event_callback(gps_event_callback);
		/* Apply persisted GPS duty interval (repeater default 48h; 0 = always on) */
		gps_set_poll_interval_sec(repeater_mesh.getNodePrefs()->gps_interval);
		gps_set_repeater_mode(true);
	}

	/* Initialize UI (display + buttons).  Shows splash screen, then auto-
	 * transitions to STATUS page.  Display sleeps after auto-off timeout;
	 * user button is the only wake source for repeater. */
	ui_init();
#if !IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	oled_sleep();
#endif

	/* Apply the persisted LED master switch ("set leds on|off"). After ui_init()
	 * so the heartbeat cycle exists to be stopped; before the radio starts so the
	 * first transmit already honours it. */
	{
		bool leds_off = repeater_mesh.getNodePrefs()->leds_disabled != 0;
		zephcore_leds_set_disabled(leds_off);
		LOG_INF("LEDs: %s (from prefs)", leds_off ? "disabled" : "enabled");
	}

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
	repeater_mesh.setWakeCallback(wake_callback, nullptr);

	/* Load or generate identity BEFORE begin(). First-boot keygen runs
	 * the layered entropy mixer + Ed25519 derive + reserved-prefix
	 * guard inside ZephyrRNG::generateFirstBootIdentity. */
	mesh::LocalIdentity self_identity;
	if (!data_store.loadIdentity(self_identity)) {
		LOG_INF("No identity found, generating new keypair...");
		mesh::ZephyrRNG::generateFirstBootIdentity(self_identity);
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

	/* Load persisted prefs and bind the radio to _prefs BEFORE begin() — the
	 * radio reads freq/bw/sf/cr through this pointer during Mesh::begin() →
	 * Dispatcher::begin() → Radio::begin().  Without this, the radio would
	 * configure on NodePrefs defaults (869.618 MHz) regardless of saved
	 * settings: CLI readback looked correct but the hardware stayed on EU.
	 * Mirrors the temp_prefs pattern in main_companion.cpp. */
	data_store.loadPrefs(*repeater_mesh.getNodePrefs());
	lora_radio.setPrefs(repeater_mesh.getNodePrefs());

	/* Start mesh with data store - loads ACL, regions */
	repeater_mesh.begin(&data_store);

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
	lora_radio.setCadParams(prefs->cad_auto != 0, prefs->cad_offset,
				prefs->probe_interval, prefs->cad_busycap);

	/* Feed initial UI state from loaded prefs */
	ui_set_node_name(prefs->node_name);
	refresh_repeater_ui_radio_state();
	ui_set_battery_provider(get_battery_mv);
	ui_set_battery(zephyr_board.getBattMilliVolts(), 0);
	ui_set_gps_available(gps_is_available());

	/* Defer initial advertisement by 10s — gives GPS time for a quick fix.
	 * Advert payload (including coords) is built when the work fires. */
	LOG_INF("Initial advertisement scheduled in 10s");
	k_work_schedule(&initial_advert_work, K_SECONDS(10));
#endif

	/* USB CDC was initialized earlier (right after clearBootloaderMagic).
	 * Just acquire the device handle for the CLI's UART IRQ binding below. */
#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)
	usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
#else
	/* No CDC ACM class driver (e.g. ESP32 usb_serial, or an ESP32-S3 repeater
	 * built without esp32s3_usb.conf where the cdc_acm DT node exists but the
	 * class isn't compiled) — use the chosen console UART. */
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
