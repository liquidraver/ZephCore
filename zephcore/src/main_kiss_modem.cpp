/*
 * SPDX-License-Identifier: MIT
 * ZephCore - KISS modem
 *
 * Composition root of the KISS modem role (upstream examples/kiss_modem): raw
 * LoRa packets to and from a host over a serial port, plus MeshCore's
 * SetHardware extensions (KissModem). No mesh, no prefs: the host configures
 * the radio, the identity is the server roles' one.
 *
 * Event loop: serial input, LoRa RX/TX-done, host output drained and a one-shot
 * for the soonest modem or radio-maintenance deadline each wake KissModem::loop().
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_kiss_main, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#define ZEPHCORE_USB_STACK \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))

#if ZEPHCORE_USB_STACK && !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
#include <ZephyrUSBCDC.h>
#endif

/* The host port: the chosen `zephcore,kiss-uart`, else the CDC-ACM UART, else
 * the console. */
#if DT_HAS_CHOSEN(zephcore_kiss_uart)
#define KISS_HOST_NODE DT_CHOSEN(zephcore_kiss_uart)
#elif ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	!IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
#define KISS_HOST_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_cdc_acm_uart)
#else
#define KISS_HOST_NODE DT_CHOSEN(zephyr_console)
#endif

/* The console driver writing to the host port would corrupt the KISS stream. */
#if DT_HAS_CHOSEN(zephyr_console)
BUILD_ASSERT(!IS_ENABLED(CONFIG_UART_CONSOLE) ||
	     !DT_SAME_NODE(KISS_HOST_NODE, DT_CHOSEN(zephyr_console)),
	     "CONFIG_UART_CONSOLE must be off while the console UART is the KISS port");
#endif

#include <app/KissModem.h>
#include <app/RepeaterDataStore.h>
#include <src/RadioIncludes.h>
#include <src/mesh_events.h>
#include <helpers/boot_prefs.h>
#include <helpers/NodePrefs.h>
#include <adapters/transport/ZephyrUartStream.h>
#include <ZephyrSensorManager.h>
#include "../adapters/datastore/ZephyrFsFormat.h"
#include "oled_power.h"
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
#include "display.h"
#endif

/* With a PWM LED, led0 is the same pin and belongs to the PWM driver. */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios) && !IS_ENABLED(CONFIG_ZEPHCORE_LED_PWM)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#if DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif

#define MESH_EVENT_HOST_TX_IDLE  BIT(MESH_EVENT_ROLE_BASE)  /* host output drained */
#define MESH_EVENT_ALL (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | MESH_EVENT_CLI_RX | \
			MESH_EVENT_MAINTENANCE | MESH_EVENT_HOST_TX_IDLE)

static struct k_event mesh_events;

static void post_lora_rx(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

static void post_lora_tx_done(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_TX_DONE);
}

static void post_host_rx(void)
{
	k_event_post(&mesh_events, MESH_EVENT_CLI_RX);
}

static void post_host_tx_idle(void)
{
	k_event_post(&mesh_events, MESH_EVENT_HOST_TX_IDLE);
}

static void wake_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_MAINTENANCE);
}

K_TIMER_DEFINE(wake_timer, wake_timer_fn, NULL);

/* KissModem reaches FEM gain through the board's `get/set radio.fem.*` commands
 * (upstream's board CLI). Ours live in CommonCLI, so this board answers the RX one
 * from the radio; no ZephCore board has a switchable FEM PA, so no `txgain`. */
class KissBoard : public mesh::ZephyrBoard {
public:
	bool handleCommand(const char *command, uint32_t sender_timestamp, char *reply) override;
};

static KissBoard board;
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LoRaRadio radio(lora_dev, board);

/* Set at boot: the radio answers setFemRxEnable() only where the board wires an
 * LNA bypass line. RAM only, as every KISS setting. */
static bool fem_rx_supported;
static bool fem_rx_on;

bool KissBoard::handleCommand(const char *command, uint32_t sender_timestamp, char *reply)
{
	ARG_UNUSED(sender_timestamp);

	if (!fem_rx_supported) {
		return false;
	}
	if (strcmp(command, "get radio.fem.rxgain") == 0) {
		strcpy(reply, fem_rx_on ? "> on" : "> off");
		return true;
	}
	if (strncmp(command, "set radio.fem.rxgain ", 21) == 0) {
		const char *arg = &command[21];
		bool on = strcmp(arg, "on") == 0;

		if (!on && strcmp(arg, "off") != 0) {
			strcpy(reply, "Error: on or off");
		} else if (radio.setFemRxEnable(on)) {
			fem_rx_on = on;
			strcpy(reply, "OK");
		} else {
			strcpy(reply, "Error: not applied");
		}
		return true;
	}
	return false;
}
static mesh::ZephyrRNG rng;
static mesh::LocalIdentity identity;
static RepeaterDataStore data_store;
static ZephyrUartStream host;
/* RAM only: the radio reads its parameters here, the host sets them. */
static NodePrefs prefs;
static KissModem modem(host, identity, rng, radio, board, sensors);

static void apply_cad_prefs(void)
{
	radio.setCadParams(prefs.cad_auto != 0, prefs.cad_offset, prefs.probe_interval,
			   prefs.cad_busycap, prefs.cad_base);
}

static void on_set_radio(float freq, float bw, uint8_t sf, uint8_t cr)
{
	bool new_channel = freq != prefs.freq || bw != prefs.bw || sf != prefs.sf;

	prefs.freq = freq;
	prefs.bw = bw;
	prefs.sf = sf;
	prefs.cr = cr;
	/* The learned CAD offset belongs to the old channel, as `set radio`. */
	if (new_channel) {
		apply_cad_prefs();
	}
	radio.reconfigure();
}

static void on_set_tx_power(uint8_t power)
{
	/* The radio clamps to the board's limit when it next configures TX. */
	prefs.tx_power_dbm = (int8_t)MIN(power, 30);
}

static float on_get_current_rssi(void)
{
	return radio.getCurrentRSSI();
}

static void on_get_stats(uint32_t *rx, uint32_t *tx, uint32_t *errors)
{
	*rx = radio.getPacketsRecv();
	*tx = radio.getPacketsSent();
	*errors = radio.getPacketsRecvErrors();
}

/* Hands received packets to the modem while the host keeps up; the rest wait in
 * the driver until HOST_TX_IDLE. */
static void drain_rx(void)
{
	uint8_t rx_buf[256];
	int rx_len;

	while (!modem.isHostOutputBackedUp() &&
	       (rx_len = radio.recvRaw(rx_buf, sizeof(rx_buf))) > 0) {
		int8_t snr = (int8_t)(radio.getLastSNR() * 4);
		int8_t rssi = (int8_t)radio.getLastRSSI();

		modem.onPacketReceived(snr, rssi, rx_buf, rx_len);
	}
}

static bool on_set_rx_boosted_gain(bool enable)
{
	return radio.setRxBoost(enable);
}

static bool on_get_rx_boosted_gain(void)
{
	return radio.isRxBoostEnabled();
}

static void arm_wake(void)
{
	uint32_t next = MIN(modem.msUntilNextTx(), radio.msUntilNextMaintenance());

	if (next >= mesh::MAINTENANCE_IDLE) {
		k_timer_stop(&wake_timer);
		return;
	}
	k_timer_start(&wake_timer, K_MSEC(MAX(next, 1U)), K_NO_WAIT);
}

int main(void)
{
	board.clearBootloaderMagic();

	/* USB comes up even when the host is on another UART: the CDC port keeps
	 * the 1200-baud DFU touch and any console there. */
#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	!IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
	zephcore_usbd_init();
#endif
	const struct device *host_dev = DEVICE_DT_GET(KISS_HOST_NODE);

	/* No UI in this role: LEDs off, the display blanked as a server leaves
	 * it after its auto-off (the panel otherwise keeps its boot state). */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios) && !IS_ENABLED(CONFIG_ZEPHCORE_LED_PWM)
	if (gpio_is_ready_dt(&led0)) {
		gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
	if (gpio_is_ready_dt(&led1)) {
		gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	if (mc_display_init() == 0) {
		mc_display_off();
	}
#else
	oled_sleep();
#endif

	if (!data_store.hasRoleData()) {
		LOG_WRN("Volume holds no data for this role - formatting before first boot");
		if (!zephcore_fs_format_all(nullptr)) {
			LOG_ERR("First-boot format failed - /lfs is not mounted");
		}
	}
	if (!data_store.begin()) {
		LOG_ERR("RepeaterDataStore init failed");
	}
	if (!data_store.loadIdentity(identity)) {
		mesh::ZephyrRNG::generateFirstBootIdentity(identity);
		data_store.saveIdentity(identity);
	}

	initNodePrefs(&prefs);
	apply_boot_prefs(&prefs, false);
	gps_park();
	env_sensors_init();

	k_event_init(&mesh_events);
	radio.setRxCallback(post_lora_rx, nullptr);
	radio.setTxDoneCallback(post_lora_tx_done, nullptr);
	radio.setPrefs(&prefs);
	radio.begin();
	bool rx_boost_supported = radio.setRxBoost(prefs.rx_boost != 0);

	fem_rx_supported = radio.setFemRxEnable(prefs.fem_rxgain != 0);
	fem_rx_on = prefs.fem_rxgain != 0;
	apply_cad_prefs();

	if (!host.begin(host_dev, post_host_rx, post_host_tx_idle)) {
		LOG_ERR("Host serial not ready");
	}

	modem.setRadioCallback(on_set_radio);
	modem.setTxPowerCallback(on_set_tx_power);
	modem.setGetCurrentRssiCallback(on_get_current_rssi);
	modem.setGetStatsCallback(on_get_stats);
	modem.setPollRxCallback(drain_rx);
	if (rx_boost_supported) {
		modem.setRxBoostedGainCallbacks(on_set_rx_boosted_gain, on_get_rx_boosted_gain);
	}
	modem.begin();

	for (;;) {
		/* processTx() takes one state step per call; the second pass takes
		 * the step the first one made due (a new frame, a finished TX). */
		modem.loop();
		modem.loop();

		drain_rx();

		radio.triggerNoiseFloorCalibrate(0);
		radio.cadMaintenance();
		radio.radioMaintenance();

		arm_wake();
		uint32_t ev = k_event_wait(&mesh_events, MESH_EVENT_ALL, false, K_FOREVER);

		k_event_clear(&mesh_events, ev);
	}

	return 0;
}
