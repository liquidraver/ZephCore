/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Observer — listen-only LoRa node with WiFi+MQTT forwarding.
 *
 * Every received LoRa packet is published to an MQTT broker in
 * meshcoretomqtt-compatible JSON format.  All parameters (WiFi, MQTT,
 * IATA, radio) are runtime-configurable via serial CLI and stored in
 * LittleFS.  Nothing is hardcoded.
 *
 * Event loop:
 *   LORA_RX  → ObserverMesh::loop() → enqueuePacket() → mqtt_publisher_enqueue()
 *   CLI_RX   → ObserverMesh::handleCLI() (set/get commands)
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#define CLI_REPLY_SIZE 256

LOG_MODULE_REGISTER(zephcore_observer_main, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#include <app/RepeaterDataStore.h>
#include <app/ObserverMesh.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <mesh/RadioIncludes.h>
#include <ZephyrWiFiStation.h>
#include <ZephyrMQTTPublisher.h>
#include "observer_creds.h"

/* ========== LED (optional) ========== */

#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
#include <zephyr/drivers/gpio.h>
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

/* ========== Event loop bits ========== */

#define MESH_EVENT_LORA_RX   BIT(0)
#define MESH_EVENT_CLI_RX    BIT(1)
#define MESH_EVENT_STATUS    BIT(2)
#define MESH_EVENT_ALL       (MESH_EVENT_LORA_RX | MESH_EVENT_CLI_RX | MESH_EVENT_STATUS)

static struct k_event mesh_events;

/* ========== USB serial CLI ========== */

#define USB_RING_BUF_SIZE 512
#define CLI_LINE_BUF_SIZE 256

static const struct device *usb_dev;
static uint8_t  usb_ring_data[USB_RING_BUF_SIZE];
static struct   ring_buf usb_ring_buf;
static char     cli_line[CLI_LINE_BUF_SIZE];
static char     cli_reply[CLI_REPLY_SIZE];
static uint16_t cli_line_idx;

static void cli_print(const char *s)
{
	if (!usb_dev) return;
	while (*s) uart_poll_out(usb_dev, *s++);
}

static void cli_println(const char *s)
{
	cli_print(s);
	cli_print("\r\n");
}

static void cli_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int n = uart_fifo_read(dev, buf, sizeof(buf));
			if (n > 0) {
				ring_buf_put(&usb_ring_buf, buf, n);
				k_event_post(&mesh_events, MESH_EVENT_CLI_RX);
			}
		}
	}
}

/* ========== LoRa callbacks ========== */

static void lora_rx_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

static void status_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_STATUS);
}

K_TIMER_DEFINE(status_timer, status_timer_fn, NULL);

/* Observer never transmits — TX done callback not needed */

/* ========== Help banner ========== */

/* Forward references */
static mesh::ObserverMesh *s_mesh_ptr;
static struct ObserverCreds s_creds;

static void print_banner(void)
{
	char line[192];

	cli_println("\r\n=== ZephCore Observer ===");

	if (s_mesh_ptr) {
		const char *name = s_mesh_ptr->getNodePrefs()->node_name;
		const char *key  = s_mesh_ptr->getPubkeyHex();
		snprintf(line, sizeof(line), "Node:  %s", name);
		cli_println(line);
		snprintf(line, sizeof(line), "Key:   %.32s...", key);
		cli_println(line);

		NodePrefs *p = s_mesh_ptr->getNodePrefs();
		snprintf(line, sizeof(line),
			 "Radio: %.3f MHz  BW%.1f  SF%u  CR%u  TX%ddBm",
			 (double)p->freq, (double)p->bw,
			 p->sf, p->cr, p->tx_power_dbm);
		cli_println(line);
	}

	cli_println("");

	snprintf(line, sizeof(line), "WiFi:  %-14s (SSID: %s)",
		 zc_wifi_station_is_connected() ? "CONNECTED" : "DISCONNECTED",
		 s_creds.wifi_ssid[0] ? s_creds.wifi_ssid : "not set");
	cli_println(line);

	snprintf(line, sizeof(line), "MQTT:  %-14s (host: %s)",
		 mqtt_publisher_is_connected() ? "CONNECTED" : "DISCONNECTED",
		 s_creds.mqtt_host[0] ? s_creds.mqtt_host : "not set");
	cli_println(line);

	snprintf(line, sizeof(line), "IATA:  %s",
		 s_creds.mqtt_iata[0] ? s_creds.mqtt_iata : "not set");
	cli_println(line);

	cli_println("");
	cli_println("--- Configure ---");
	cli_println("set wifi.ssid <name>       WiFi network name");
	cli_println("set wifi.psk  <password>   WiFi password (empty = open)");
	cli_println("set mqtt.host <hostname>   MQTT broker hostname");
	cli_println("set mqtt.port <port>       MQTT broker port (default 8883)");
	cli_println("set mqtt.tls  <0|1>        TLS on/off (default 1)");
	cli_println("set mqtt.user <username>   MQTT username");
	cli_println("set mqtt.password <pass>   MQTT password");
	cli_println("set mqtt.iata <code>       Location code (e.g. BUD BTS VIE SEA)");
	cli_println("set name      <name>       Node display name");
	cli_println("set freq      <MHz|Hz>     LoRa frequency (e.g. 869.618 or 869618000)");
	cli_println("set sf        <7-12>       Spreading factor");
	cli_println("set bw        <idx>        Bandwidth: 3=62.5 0=125 1=250 2=500 kHz");
	cli_println("set cr        <5-8>        Coding rate");
	cli_println("");
	cli_println("--- Query ---");
	cli_println("get wifi.status            WiFi connection state");
	cli_println("get mqtt.status            MQTT connection state");
	cli_println("get radio                  LoRa radio parameters");
	cli_println("help                       Show this screen");
	cli_println("=========================");
}

/* ========== CLI RX processing ========== */

static void process_cli_rx(void)
{
	uint8_t byte;
	while (ring_buf_get(&usb_ring_buf, &byte, 1) == 1) {
		if (byte == '\r' || byte == '\n') {
			if (cli_line_idx > 0) {
				cli_line[cli_line_idx] = '\0';
				LOG_DBG("CLI: %s", cli_line);

				cli_reply[0] = '\0';
				bool want_banner = false;
				if (s_mesh_ptr) {
					want_banner = s_mesh_ptr->handleCLI(
						cli_line, cli_reply,
						sizeof(cli_reply));
				}

				if (want_banner) {
					print_banner();
				} else if (cli_reply[0] != '\0') {
					cli_print("\r\n  -> ");
					cli_println(cli_reply);
				}
				cli_line_idx = 0;
			}
			cli_print("\r\n");
		} else if (byte == 0x7F || byte == 0x08) {
			if (cli_line_idx > 0) {
				cli_line_idx--;
				uart_poll_out(usb_dev, '\b');
				uart_poll_out(usb_dev, ' ');
				uart_poll_out(usb_dev, '\b');
			}
		} else if (cli_line_idx < sizeof(cli_line) - 1) {
			uart_poll_out(usb_dev, byte);
			cli_line[cli_line_idx++] = (char)byte;
		}
	}
}

/* ========== Time sync callback ========== */

static mesh::ZephyrRTCClock s_rtc_clock;

static void time_sync_cb(uint32_t unix_ts)
{
	s_rtc_clock.setCurrentTime(unix_ts);
	LOG_INF("RTC synced from SNTP: %u", unix_ts);
}

/* ========== Global instances ========== */

static mesh::ZephyrBoard    s_board;
static mesh::ZephyrMillisecondClock s_ms_clock;
static mesh::ZephyrRNG      s_rng;

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* Radio prefs — observer-specific defaults set in main() */
static NodePrefs s_radio_prefs;

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
static mesh::LR1110Radio lora_radio(lora_dev, s_board, &s_radio_prefs);
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_SX127X)
static mesh::SX127xRadio lora_radio(lora_dev, s_board, &s_radio_prefs);
#else
static mesh::SX126xRadio lora_radio(lora_dev, s_board, &s_radio_prefs);
#endif

static mesh::ObserverMesh observer_mesh(lora_radio, s_ms_clock, s_rng, s_rtc_clock);
static RepeaterDataStore  data_store;

/* ========== main() ========== */

int main(void)
{
	/* Initialize radio prefs with observer-specific defaults */
	initNodePrefs(&s_radio_prefs);
	s_radio_prefs.cr           = 5;   /* CR 4/5 (initNodePrefs sets 8) */
	s_radio_prefs.tx_power_dbm = 0;   /* observer never TXes */
	strncpy(s_radio_prefs.node_name, "Observer",
		sizeof(s_radio_prefs.node_name) - 1);

	/* Brief boot delay — lets USB enumerate before first log */
	k_sleep(K_MSEC(1500));
	LOG_INF("=== ZephCore Observer starting ===");

	/* Configure LED */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
	if (gpio_is_ready_dt(&led0)) {
		gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	}
#endif

	/* Initialize LittleFS data store */
	if (!data_store.begin()) {
		LOG_ERR("RepeaterDataStore init failed");
	}

	/* Load observer credentials (WiFi, MQTT, IATA) */
	memset(&s_creds, 0, sizeof(s_creds));
	observer_creds_init(&s_creds);
	observer_creds_load(&s_creds, data_store.getBasePath());

	/* Init event object BEFORE any callbacks */
	k_event_init(&mesh_events);

	/* LoRa RX callback — observer never needs TX done */
	lora_radio.setRxCallback(lora_rx_callback, nullptr);

	/* Initialize and start mesh (loads prefs + identity from flash) */
	s_mesh_ptr = &observer_mesh;
	observer_mesh.begin(&data_store, &s_creds);

	/* Generate a default node name based on pubkey if still generic */
	NodePrefs *prefs = observer_mesh.getNodePrefs();
	if (strlen(prefs->node_name) == 0 ||
	    strcmp(prefs->node_name, "Observer") == 0 ||
	    strcmp(prefs->node_name, "Repeater") == 0) {
		/* Use first 4 bytes of pubkey for uniqueness */
		const char *hex = observer_mesh.getPubkeyHex();
		snprintf(prefs->node_name, sizeof(prefs->node_name),
			 "Observer-%.8s", hex);
		data_store.savePrefs(*prefs);
	}

	/* Initialize USB serial for CLI */
#if DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)
	usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
#else
	/* ESP32 native USB or other console UART */
	usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
	if (device_is_ready(usb_dev)) {
		ring_buf_init(&usb_ring_buf, sizeof(usb_ring_data), usb_ring_data);
		uart_irq_callback_set(usb_dev, cli_uart_isr);
		uart_irq_rx_enable(usb_dev);
		LOG_INF("Serial CLI ready: %s", usb_dev->name);
	} else {
		LOG_WRN("Serial device not ready — CLI unavailable");
		usb_dev = nullptr;
	}

	/* Print welcome banner */
	print_banner();

	/* Start WiFi (non-blocking — MQTT thread waits for WIFI_READY_BIT) */
	zc_wifi_station_start(&s_creds, time_sync_cb);

	/* Start MQTT publisher thread */
	char client_id[64];
	snprintf(client_id, sizeof(client_id), "%s", prefs->node_name);
	mqtt_publisher_start(&s_creds, client_id,
			     observer_mesh.getStatusTopic(),
			     observer_mesh.getPacketsTopic());

	/* Publish self-advert once on each MQTT connect so CoreScope can pin
	 * this observer on the map (requires lat/lon to be configured). */
	mqtt_publisher_set_connect_cb([]() {
		if (s_mesh_ptr) {
			s_mesh_ptr->publishStatus("online");
			s_mesh_ptr->publishSelfAdvert();
		}
	});
	k_timer_start(&status_timer, K_SECONDS(300), K_SECONDS(300));

	LOG_INF("Observer event loop running");

	/* ========== Event loop ========== */
	for (;;) {
		uint32_t ev = k_event_wait(&mesh_events, MESH_EVENT_ALL,
					   false, K_FOREVER);
		k_event_clear(&mesh_events, ev);

		if (ev & MESH_EVENT_LORA_RX) {
			observer_mesh.loop();
		}

		if (ev & MESH_EVENT_CLI_RX) {
			process_cli_rx();
		}
		if (ev & MESH_EVENT_STATUS) {
			observer_mesh.publishStatus("online");
		}
	}

	return 0;
}
