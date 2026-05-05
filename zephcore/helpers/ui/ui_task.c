/*
 * ZephCore - UI Task
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
 *
 * Integration layer that wires buttons, buzzer, and display together.
 * All event-driven via Zephyr input subsystem + k_work.
 *
 * Input flow (after longpress + multi-tap filter chain):
 *   KEY_1     → action_page_next()       (1 tap, 400ms delayed)
 *   KEY_LEFT  → action_page_prev()       (2 taps — RAK4631 / Pocket / Heltec V3–V4.3)
 *   KEY_B     → action_flood_advert()    (2 taps on extended multitap overlays)
 *   KEY_D     → action_buzzer_toggle()   (3 taps)
 *   KEY_C     → action_gps_toggle()      (4 taps, immediate)
 *   KEY_G     → GPS switch on/off        (hardware toggle, ThinkNode M1)
 *   KEY_POWER / KEY_F → action_deep_sleep() (long press — boards that emit these)
 *   KEY_ENTER → action_page_enter()      (long press — Pocket / Heltec; joystick center Wio)
 *   KEY_RIGHT → action_page_next()       (joystick, Wio Tracker)
 *
 * Notification flow:
 *   LoRa RX → CompanionMesh → ui_notify(UI_EVENT_CONTACT_MSG)
 *     → buzzer_play(MELODY_MSG_CONTACT)
 *     → mc_display_on() + render
 */

#include "ui_task.h"
#include "ui_pages.h"

#ifdef CONFIG_ZEPHCORE_UI_BUZZER
#include "buzzer.h"
#endif

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
#include "display.h"
#endif

#include <zephyr/kernel.h>

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
#include "doom_game.h"
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <string.h>

#ifdef CONFIG_POWEROFF
#include <zephyr/sys/poweroff.h>
#endif

#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
#include <hal/nrf_gpio.h>
#endif

#include <zephyr/sys/reboot.h>

/* GPS control (extern "C" in ZephyrSensorManager.h) */
#include <ZephyrSensorManager.h>

/* Mesh action wrappers (deferred to mesh event loop thread) */
#include "ui_mesh_actions.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ui_task, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* Tap-count feedback melodies.
 * b=200, d=16 → each chirp ~75ms, rest ~75ms = clear separation.
 *
 * 2 taps (flood advert):  chirp-chirp
 * 3 taps (buzzer toggle): chirp×3 + high(ON) or low(OFF)
 * 4 taps (GPS toggle):    chirp×4 + high(ON) or low(OFF)
 *
 * ON tail:  high E7 (~2637Hz) = "enabled"
 * OFF tail: low G5 (~784Hz)   = "disabled"  */
#define MELODY_BEEP_2     "b2:d=16,o=7,b=200:c,p,c"

#define MELODY_BUZZER_ON  "bon:d=16,o=7,b=200:c,p,c,p,c,p,p,8e"
#define MELODY_BUZZER_OFF "bof:d=16,o=7,b=200:c,p,c,p,c,p,p,8g5"

#define MELODY_GPS_ON     "gon:d=16,o=7,b=200:c,p,c,p,c,p,c,p,p,8e"
#define MELODY_GPS_OFF    "gof:d=16,o=7,b=200:c,p,c,p,c,p,c,p,p,8g5"

#define MELODY_LED_ON     "lon:d=16,o=7,b=200:c,p,c,p,c,p,c,p,c,p,p,8e"
#define MELODY_LED_OFF    "lof:d=16,o=7,b=200:c,p,c,p,c,p,c,p,c,p,p,8g5"

/* ========== LED Heartbeat ========== */
/* Match Arduino: 4s cycle, 20ms pulse (normal) or 200ms (unread messages).
 * Uses led0 or led1 alias — whichever exists in the board's DTS.
 * Disabled when OLED display is present (display makes LED redundant). */
/* Heartbeat LED — subtle pulse every 4s on led0 (or led1 fallback).
 * Works alongside displays; boards that want to disable it can
 * remove the led0 alias or override this with a Kconfig guard. */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
static const struct gpio_dt_spec heartbeat_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define HAS_HEARTBEAT_LED 1
#elif DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
static const struct gpio_dt_spec heartbeat_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#define HAS_HEARTBEAT_LED 1
#else
#define HAS_HEARTBEAT_LED 0
#endif

/* Second LED: unread pulse (companion only). Repeaters use led1 for LoRa TX via
 * lora-tx-led alias — no offline queue, so ui_task leaves led1 alone. */
#if HAS_HEARTBEAT_LED && DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios) && \
    DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios) && !defined(ZEPHCORE_REPEATER)
static const struct gpio_dt_spec msg_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#define HAS_MSG_LED 1
#else
#define HAS_MSG_LED 0
#endif

#define LED_CYCLE_MS      4000   /* Total heartbeat period */
#define LED_ON_MS         20     /* Normal pulse width */
#define LED_ON_MSG_MS     200    /* Pulse width when unread messages */

/* ========== Deep Sleep / System OFF ========== */
/* On nRF52840, sys_poweroff() = System OFF (~1µA).
 * Wake via reset button → full chip reset → boots fresh. */

/* When display is not available, maintain a local ui_state for setters
 * (data can still be used for logging or future features). */
#ifndef CONFIG_ZEPHCORE_UI_DISPLAY
static struct ui_state local_ui_state;
#endif

/* ========== Constants ========== */
#define SPLASH_DURATION_MS  3000
#define RENDER_DEBOUNCE_MS      50
#define RENDER_DEBOUNCE_EPD_MS  200   /* e-paper: coalesce rapid state updates, then refresh */

/* ========== State ========== */
static bool ui_initialized;
static bool splash_active;

/* ========== Doom Easter Egg Activation ========== */
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
/*
 * Activation sequence: double-click + single-click INPUT_KEY_ENTER on page 0.
 * That's 3 presses total: press-press (double) then press (single).
 * Deactivation: double-click INPUT_KEY_ENTER while Doom is running.
 *
 * State machine:
 *   IDLE → (1st press) → FIRST_PRESS
 *   FIRST_PRESS → (2nd press within 500ms) → DCLICK_DONE
 *   DCLICK_DONE → (3rd press within 500ms) → activate Doom
 *   Any state → (timeout) → IDLE
 */
enum doom_activate_state {
	DOOM_ACT_IDLE,
	DOOM_ACT_FIRST_PRESS,
	DOOM_ACT_DCLICK_DONE,
};

static enum doom_activate_state doom_act_state;
static uint32_t doom_act_time;
#define DOOM_ACT_TIMEOUT_MS  500
#endif

static inline struct ui_state *get_state(void)
{
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	return ui_pages_get_state();
#else
	return &local_ui_state;
#endif
}

/* Render work - debounced display update */
static struct k_work_delayable render_work;
/* Splash timeout work */
static struct k_work_delayable splash_work;
/* Deferred zero-hop advert — waits for possible double-press upgrade to flood */
static struct k_work_delayable advert_defer_work;

/* LED heartbeat only on boards with LED but no OLED (display makes it redundant) */
#if HAS_HEARTBEAT_LED
/* LED heartbeat: two one-shot works form a self-rescheduling cycle.
 * led_on_work turns LED on and schedules led_off_work after pulse width.
 * led_off_work turns LED off and schedules led_on_work after remainder. */
static struct k_work_delayable led_on_work;
static struct k_work_delayable led_off_work;
#endif

/* ========== Work Handlers ========== */

#if HAS_HEARTBEAT_LED
static void led_off_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	gpio_pin_set_dt(&heartbeat_led, 0);
#if HAS_MSG_LED
	gpio_pin_set_dt(&msg_led, 0);
#endif

	/* Schedule next ON after remainder of cycle */
	struct ui_state *s = get_state();
	uint16_t on_ms = (s->msg_count > 0) ? LED_ON_MSG_MS : LED_ON_MS;

	k_work_reschedule(&led_on_work, K_MSEC(LED_CYCLE_MS - on_ms));
}

static void led_on_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	gpio_pin_set_dt(&heartbeat_led, 1);
#if HAS_MSG_LED
	struct ui_state *s = get_state();
	if (s->msg_count > 0) {
		gpio_pin_set_dt(&msg_led, 1);
	}
	uint16_t on_ms = (s->msg_count > 0) ? LED_ON_MSG_MS : LED_ON_MS;
#else
	struct ui_state *s = get_state();
	uint16_t on_ms = (s->msg_count > 0) ? LED_ON_MSG_MS : LED_ON_MS;
#endif

	k_work_reschedule(&led_off_work, K_MSEC(on_ms));
}
#endif

static void render_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!ui_initialized) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	/* EPD: render even when backlight is off — content is always visible.
	 * OLED: only render when display is on (screen is black when off). */
	if ((mc_display_is_on() || mc_display_is_epd()) && !splash_active) {
		ui_pages_render();
	}
#endif
}

static void splash_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	splash_active = false;

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	mc_display_epd_full_reset();

	/* Transition from splash to home page (first active page for this role) */
#ifdef ZEPHCORE_REPEATER
	ui_pages_set(UI_PAGE_STATUS);
#else
	ui_pages_set(UI_PAGE_MESSAGES);
#endif
	k_work_reschedule(&render_work, K_NO_WAIT);
#endif
}

static void schedule_render(void);

static void advert_defer_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	/* Timer expired without second press — send zero-hop */
	LOG_INF("zero-hop advert requested (deferred)");
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(MELODY_BEEP_2);
#endif
	mesh_send_zerohop_advert();
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	ui_pages_advert_sent(false);
#endif
	schedule_render();
}

static void schedule_render(void)
{
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	uint32_t debounce = mc_display_is_epd() ? RENDER_DEBOUNCE_EPD_MS
						 : RENDER_DEBOUNCE_MS;
	k_work_reschedule(&render_work, K_MSEC(debounce));
#endif
}

/* ========== Button Action Functions ========== */
/* Each action checks capabilities internally — no #ifdef in the switch. */

static void action_page_next(void)
{
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	ui_pages_next();
#endif
	schedule_render();
}

static void action_page_prev(void)
{
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	ui_pages_prev();
#endif
	schedule_render();
}

/* Forward declarations for page-enter dispatch */
static void action_flood_advert(void);
static void action_gps_toggle(void);
static void action_buzzer_toggle(void);
static void action_leds_toggle(void);
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
static void action_ble_toggle(void);
static void action_enter_dfu(void);
#endif
static void action_deep_sleep(void);

static void action_page_enter(void)
{
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	enum ui_page page = ui_pages_current();

	LOG_DBG("ENTER on page %d", page);

	switch (page) {
	case UI_PAGE_BLUETOOTH:
		/* Toggle BLE on/off */
		action_ble_toggle();
		break;

	case UI_PAGE_ADVERT:
		/* Single press: defer 500ms then send zero-hop.
		 * Double press within 500ms: cancel deferred, send flood. */
		if (k_work_delayable_is_pending(&advert_defer_work)) {
			/* Second press — cancel deferred zero-hop, send flood */
			k_work_cancel_delayable(&advert_defer_work);
			action_flood_advert();
		} else {
			/* First press — start deferred zero-hop */
			k_work_reschedule(&advert_defer_work, K_MSEC(500));
		}
		break;

	case UI_PAGE_GPS:
		/* Toggle GPS (same as quad-tap) */
		action_gps_toggle();
		break;

	case UI_PAGE_BUZZER:
		/* Toggle buzzer mute (same as triple-tap) */
		action_buzzer_toggle();
		break;

	case UI_PAGE_LEDS:
		/* Toggle LED on/off */
		action_leds_toggle();
		break;

	case UI_PAGE_OFFGRID: {
		/* Double-press confirmation (CONFIRM_WINDOW_MS window) */
		struct ui_state *st_og = get_state();
		uint32_t now_og = k_uptime_get_32();

		if (st_og->offgrid_confirm_time != 0 &&
		    (now_og - st_og->offgrid_confirm_time) <= CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
			/* Confirmed — toggle offgrid mode */
			bool new_state = !st_og->offgrid_enabled;
			st_og->offgrid_enabled = new_state;
			st_og->offgrid_confirm_time = 0;
			mesh_set_offgrid_mode(new_state);
			LOG_INF("offgrid mode %s (button)", new_state ? "on" : "off");
		} else {
			/* First press — enter confirmation state */
			st_og->offgrid_confirm_time = now_og;
		}
		schedule_render();
		break;
	}

	case UI_PAGE_DFU: {
		/* Double-press confirmation (CONFIRM_WINDOW_MS window) */
		struct ui_state *st_dfu = get_state();
		uint32_t now_dfu = k_uptime_get_32();

		if (st_dfu->dfu_confirm_time != 0 &&
		    (now_dfu - st_dfu->dfu_confirm_time) <= CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
			/* Confirmed — reboot into BLE DFU */
			action_enter_dfu();
		} else {
			/* First press — enter confirmation state */
			st_dfu->dfu_confirm_time = now_dfu;
			schedule_render();
		}
		break;
	}

	case UI_PAGE_SHUTDOWN: {
		/* Double-press confirmation (CONFIRM_WINDOW_MS window) */
		struct ui_state *st = get_state();
		uint32_t now = k_uptime_get_32();

		if (st->shutdown_confirm_time != 0 &&
		    (now - st->shutdown_confirm_time) <= CONFIG_ZEPHCORE_UI_CONFIRM_WINDOW_MS) {
			/* Confirmed — shut down */
			action_deep_sleep();
		} else {
			/* First press — enter confirmation state */
			st->shutdown_confirm_time = now;
			schedule_render();
		}
		break;
	}

	case UI_PAGE_MESSAGES:
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	{
		/* Doom activation: 3 presses — double-click + single-click.
		 * Press 1 → FIRST_PRESS
		 * Press 2 (within 500ms) → DCLICK_DONE
		 * Press 3 (within 500ms) → activate Doom */
		uint32_t now_doom = k_uptime_get_32();

		switch (doom_act_state) {
		case DOOM_ACT_IDLE:
			doom_act_time = now_doom;
			doom_act_state = DOOM_ACT_FIRST_PRESS;
			break;
		case DOOM_ACT_FIRST_PRESS:
			if ((now_doom - doom_act_time) <= DOOM_ACT_TIMEOUT_MS) {
				/* Double-click detected, wait for single */
				doom_act_time = now_doom;
				doom_act_state = DOOM_ACT_DCLICK_DONE;
			} else {
				/* Timeout — restart as new first press */
				doom_act_time = now_doom;
				doom_act_state = DOOM_ACT_FIRST_PRESS;
			}
			break;
		case DOOM_ACT_DCLICK_DONE:
			if ((now_doom - doom_act_time) <= DOOM_ACT_TIMEOUT_MS) {
				/* Third press — activate Doom! */
				doom_act_state = DOOM_ACT_IDLE;
				LOG_DBG("Doom easter egg activated!");
				doom_game_start();
			} else {
				/* Timeout — restart */
				doom_act_time = now_doom;
				doom_act_state = DOOM_ACT_FIRST_PRESS;
			}
			break;
		}
		break;
	}
#endif
		/* fall through if doom not enabled */

	default:
		/* Other pages: no action on ENTER */
		break;
	}
#endif
}

static void action_flood_advert(void)
{
	LOG_INF("flood advert requested");
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(MELODY_BEEP_2);
#endif
	mesh_send_flood_advert();
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	ui_pages_advert_sent(true);
#endif
	schedule_render();
}

static void action_buzzer_toggle(void)
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	bool was_quiet = buzzer_is_quiet();

	if (was_quiet) {
		/* Unmuting: enable first, then play ascending confirmation */
		buzzer_set_quiet(false);
		buzzer_play(MELODY_BUZZER_ON);
	} else {
		/* Muting: play descending confirmation while still enabled.
		 * Use deferred mute so the "off" melody plays out fully
		 * before the quiet flag suppresses future sounds. */
		buzzer_play(MELODY_BUZZER_OFF);
		buzzer_set_quiet_deferred(true);
	}
	/* Persist mute state across reboots */
	mesh_set_buzzer_quiet(!was_quiet);
	get_state()->buzzer_quiet = !was_quiet;
	LOG_INF("buzzer %s", buzzer_is_quiet() ? "muted" : "unmuted");
#endif
	schedule_render();
}

static void action_leds_toggle(void)
{
	struct ui_state *s = get_state();
	bool new_disabled = !s->leds_disabled;

	s->leds_disabled = new_disabled;
	ui_set_heartbeat_led(!new_disabled);
	mesh_set_leds_disabled(new_disabled);
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(new_disabled ? MELODY_LED_OFF : MELODY_LED_ON);
#endif
	LOG_INF("LEDs %s (user toggle)", new_disabled ? "disabled" : "enabled");
	schedule_render();
}

static void action_gps_toggle(void)
{
	if (!gps_is_available()) {
		LOG_WRN("GPS toggle ignored — no GPS hardware");
		return;
	}

	bool now_enabled = !gps_is_enabled();

	LOG_INF("GPS toggle → %s", now_enabled ? "on" : "off");
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(now_enabled ? MELODY_GPS_ON : MELODY_GPS_OFF);
#endif
	/* Use persistent wrapper — same path as BLE CMD_SET_CUSTOM_VAR "gps" */
	mesh_gps_set_enabled(now_enabled);
	schedule_render();
}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
static void action_ble_toggle(void)
{
	struct ui_state *s = get_state();
	bool now_enabled = !s->ble_enabled;

	LOG_INF("BLE toggle → %s", now_enabled ? "on" : "off");
	s->ble_enabled = now_enabled;
	mesh_ble_set_enabled(now_enabled);
	schedule_render();
}

static void action_enter_dfu(void)
{
	LOG_INF("entering BLE DFU bootloader");

#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(MELODY_SHUTDOWN);
	while (buzzer_is_playing()) {
		k_sleep(K_MSEC(50));
	}
	buzzer_stop();
#endif

	mc_display_clear();
	mc_display_text(16, 28, "BLE DFU...", false);
	mc_display_finalize();
	k_sleep(K_MSEC(500));
	mc_display_off();

	/* Delegate to board adapter — handles GPREGRET + reset for any platform */
	mesh_reboot_to_ota_dfu();
	CODE_UNREACHABLE;
}
#endif /* CONFIG_ZEPHCORE_UI_DISPLAY */

static void action_deep_sleep(void)
{
#ifdef CONFIG_POWEROFF
	LOG_INF("deep sleep: shutting down...");

	/* 1. Stop LED heartbeat and msg indicator */
#if HAS_HEARTBEAT_LED
	k_work_cancel_delayable(&led_on_work);
	k_work_cancel_delayable(&led_off_work);
	gpio_pin_set_dt(&heartbeat_led, 0);
#endif
#if HAS_MSG_LED
	gpio_pin_set_dt(&msg_led, 0);
#endif

	/* 2. Play shutdown melody (blocking wait) */
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(MELODY_SHUTDOWN);
	while (buzzer_is_playing()) {
		k_sleep(K_MSEC(50));
	}
	buzzer_stop();
#endif

	/* 3. Turn display off */
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	mc_display_off();
#endif

	/* 4. Drive power-hungry enable pins LOW.
	 *
	 * nRF52840 GPIO output latches persist across System OFF and reset.
	 * If GPS_EN is HIGH, the GPS module stays powered during "sleep."
	 * Drive known power-enable GPIOs LOW via Zephyr's safe GPIO API.
	 *
	 * DO NOT touch BLE (bt_conn_disconnect / bt_le_adv_stop) — that
	 * corrupts BLE controller state and prevents clean reboot.
	 * DO NOT blank all GPIOs — that bricked the device previously. */
	gps_power_off_for_shutdown();
	mesh_disable_power_regulators();

	/* 5. Hold the LoRa radio in hardware reset.
	 *
	 * sys_poweroff() bypasses device PM — the SX126x hardware duty cycle
	 * would otherwise keep cycling autonomously (drawing mA) while the
	 * SoC is in System OFF (~1µA).  Drive RESET low; nRF52 GPIO output
	 * latches persist across System OFF so the chip stays in reset (0µA).
	 * On wakeup, the driver's sx126x_init() re-asserts then releases reset. */
#if DT_NODE_EXISTS(DT_ALIAS(lora0)) && DT_NODE_HAS_PROP(DT_ALIAS(lora0), reset_gpios)
	{
		static const struct gpio_dt_spec lora_reset =
			GPIO_DT_SPEC_GET(DT_ALIAS(lora0), reset_gpios);
		gpio_pin_configure_dt(&lora_reset, GPIO_OUTPUT_ACTIVE);
	}
#endif

	/* 6. Configure GPIO SENSE for button wakeup, then enter System OFF.
	 *
	 * The nRF GPIO driver does not implement GPIO_INT_WAKEUP, so the
	 * wakeup-source DTS property has no effect on nRF52. We must set
	 * SENSE bits directly via the nRF HAL. sys_poweroff() goes straight
	 * to nrf_power_system_off() with no device PM suspend, so these bits
	 * persist into System OFF and trigger a reset on next button press.
	 *
	 * Wait for button release first: if we enter System OFF while the
	 * button is still held (long-press shutdown), DETECT is already
	 * asserted and the chip cannot enter System OFF cleanly. */
#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF) && DT_NODE_EXISTS(DT_ALIAS(sw0))
	{
#define _SW0_NODE  DT_ALIAS(sw0)
#define _SW0_PORT  DT_PROP(DT_GPIO_CTLR(_SW0_NODE, gpios), port)
#define _SW0_PIN   DT_GPIO_PIN(_SW0_NODE, gpios)
#define _SW0_FLAGS DT_GPIO_FLAGS(_SW0_NODE, gpios)

		static const struct gpio_dt_spec sw0 =
			GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
		gpio_pin_configure_dt(&sw0, GPIO_INPUT);

		int64_t deadline = k_uptime_get() + 5000;
		while (gpio_pin_get_dt(&sw0) && k_uptime_get() < deadline) {
			k_sleep(K_MSEC(10));
		}

		nrf_gpio_cfg_sense_input(
			NRF_GPIO_PIN_MAP(_SW0_PORT, _SW0_PIN),
			(_SW0_FLAGS & GPIO_PULL_UP)   ? NRF_GPIO_PIN_PULLUP   :
			(_SW0_FLAGS & GPIO_PULL_DOWN) ? NRF_GPIO_PIN_PULLDOWN :
			                               NRF_GPIO_PIN_NOPULL,
			(_SW0_FLAGS & GPIO_ACTIVE_LOW) ? NRF_GPIO_PIN_SENSE_LOW
			                               : NRF_GPIO_PIN_SENSE_HIGH);
#undef _SW0_NODE
#undef _SW0_PORT
#undef _SW0_PIN
#undef _SW0_FLAGS
	}
#endif /* CONFIG_SOC_FAMILY_NORDIC_NRF && sw0 */

	LOG_INF("deep sleep: entering System OFF");
	sys_poweroff();
	CODE_UNREACHABLE;
#else
	LOG_WRN("deep sleep: CONFIG_POWEROFF not enabled");
#endif
}

/* ========== Input Event Handler ========== */

static void ui_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	/* When Doom is running, intercept ALL input (presses AND releases) */
	if (doom_game_is_running()) {
		/* Double-click ENTER to exit: detect two presses within 500ms */
		if (evt->code == INPUT_KEY_ENTER && evt->value) {
			static uint32_t doom_last_enter;
			uint32_t now = k_uptime_get_32();

			if (doom_last_enter != 0 &&
			    (now - doom_last_enter) <= 500) {
				/* Double-click — exit Doom */
				doom_last_enter = 0;
				doom_game_stop();
				schedule_render();
				return;
			}
			doom_last_enter = now;
		}

		/* Forward all key events (press + release) to Doom */
		doom_game_input(evt->code, evt->value);
		return;
	}
#endif

	/* GPS hardware switch (toggle switch, not momentary button).
	 * Needs both press (ON) and release (OFF) events,
	 * so handle before the release-event filter below. */
	if (evt->code == INPUT_KEY_G) {
		bool gps_on = (evt->value != 0);
		LOG_INF("GPS switch → %s", gps_on ? "on" : "off");
		if (gps_is_available()) {
			mesh_gps_set_enabled(gps_on);
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
			buzzer_play(gps_on ? MELODY_GPS_ON : MELODY_GPS_OFF);
#endif
			schedule_render();
		}
		return;
	}

	/* Only handle key press events (value=1), not releases (value=0) */
	if (!evt->value) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	/* If display is off, wake it and consume the event */
	if (!mc_display_is_on()) {
		mc_display_on();
		schedule_render();
		return;
	}

	/* Reset auto-off timer on any button press */
	mc_display_reset_auto_off();
#endif

	/* Dismiss splash screen on any button press */
	if (splash_active) {
		k_work_cancel_delayable(&splash_work);
		splash_active = false;
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
		mc_display_epd_full_reset();

#ifdef ZEPHCORE_REPEATER
		ui_pages_set(UI_PAGE_STATUS);
#else
		ui_pages_set(UI_PAGE_MESSAGES);
#endif
#endif
		schedule_render();
		return;
	}

	/* Map input key codes to UI actions.
	 *
	 * RAK4631 / WisMesh Pocket / Heltec V3–V4.3: 1 tap KEY_1, 2 taps KEY_LEFT;
	 * long press KEY_ENTER (page enter). Other boards: up to 4–5 tap codes
	 * (KEY_B/D/C/E) and KEY_POWER or KEY_F long → deep sleep.
	 * KEY_RIGHT/LEFT/ENTER/UP/DOWN: joystick (Wio Tracker)
	 *
	 * NOTE: KEY_A (raw short-press from longpress filter) is NOT handled
	 * here. It feeds into the multi-tap filter which emits the tap codes.
	 * Since INPUT_CALLBACK_DEFINE(NULL) sees events from all devices,
	 * the raw KEY_A events fall through to default: break.
	 */
	switch (evt->code) {
	/* ===== Multi-tap outputs ===== */
	case INPUT_KEY_1:
		/* Single tap (400ms delayed): page next */
		action_page_next();
		break;

	case INPUT_KEY_B:
		/* Double tap (400ms delayed): flood advert */
		action_flood_advert();
		break;

	case INPUT_KEY_D:
		/* Triple tap (400ms delayed): toggle buzzer mute */
		action_buzzer_toggle();
		break;

	case INPUT_KEY_C:
		/* Quadruple tap (400ms delayed): toggle GPS */
		action_gps_toggle();
		break;

	case INPUT_KEY_E:
		/* Quintuple tap (immediate): toggle LED heartbeat */
		action_leds_toggle();
		break;

	/* ===== Longpress output ===== */
	case INPUT_KEY_POWER:
	case INPUT_KEY_F:
		/* Long press (≥1s): deep sleep */
		action_deep_sleep();
		break;

	/* ===== Joystick (Wio Tracker) ===== */
	case INPUT_KEY_RIGHT:
		action_page_next();
		break;

	case INPUT_KEY_LEFT:
		action_page_prev();
		break;

	case INPUT_KEY_ENTER:
		action_page_enter();
		break;

	case INPUT_KEY_UP:
	case INPUT_KEY_DOWN:
		/* Joystick up/down - scroll within page (future) */
		break;

	default:
		break;
	}
}

/* Register input callback for ALL devices (no specific device filter) */
INPUT_CALLBACK_DEFINE(NULL, ui_input_cb, NULL);

/* ========== Public API ========== */

int ui_init(void)
{
	int ret;

	k_work_init_delayable(&render_work, render_work_handler);
	k_work_init_delayable(&splash_work, splash_work_handler);
	k_work_init_delayable(&advert_defer_work, advert_defer_handler);

	/* Initialize buzzer (optional - may not be present) */
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	ret = buzzer_init();
	if (ret == 0) {
		LOG_INF("buzzer ready");
	} else if (ret == -ENODEV) {
		LOG_INF("no buzzer hardware");
	} else {
		LOG_WRN("buzzer init failed: %d", ret);
	}
#endif

	/* Initialize display (optional - may not be present) */
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	ret = mc_display_init();
	if (ret == 0) {
		LOG_INF("display ready");

		/* Show splash screen */
		splash_active = true;
		ui_pages_render_splash();

		/* Start auto-off timer so display sleeps after timeout */
		mc_display_reset_auto_off();

		/* Schedule transition to home page */
		k_work_reschedule(&splash_work, K_MSEC(SPLASH_DURATION_MS));
	} else if (ret == -ENODEV) {
		LOG_INF("no display hardware");
	} else {
		LOG_WRN("display init failed: %d", ret);
	}
#endif

	/* Initialize LED heartbeat — only on boards WITHOUT a display.
	 * If there's an OLED, the heartbeat LED is redundant and wastes power. */
#if HAS_HEARTBEAT_LED
	if (gpio_is_ready_dt(&heartbeat_led)) {
		gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
		k_work_init_delayable(&led_on_work, led_on_work_handler);
		k_work_init_delayable(&led_off_work, led_off_work_handler);
		/* Start heartbeat cycle */
		k_work_reschedule(&led_on_work, K_NO_WAIT);
		LOG_INF("LED heartbeat started");
	}
#endif
#if HAS_MSG_LED
	if (gpio_is_ready_dt(&msg_led)) {
		gpio_pin_configure_dt(&msg_led, GPIO_OUTPUT_INACTIVE);
		LOG_INF("msg LED ready");
	}
#endif

	/* NOTE: startup chime is NOT played here. It's played from main()
	 * after loadPrefs() so we can respect the persisted buzzer_quiet setting.
	 * See ui_play_startup_chime(). */

	ui_initialized = true;
	LOG_INF("UI initialized");

	/* Suppress unused variable warning when both display and buzzer are disabled */
	(void)ret;

	return 0;
}

void ui_play_startup_chime(void)
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	if (!buzzer_is_quiet()) {
		buzzer_play(MELODY_STARTUP);
	}
#endif
}

void ui_notify(enum ui_event event)
{
	if (!ui_initialized) {
		return;
	}

	bool is_msg_event = false;

	switch (event) {
	case UI_EVENT_CONTACT_MSG:
		is_msg_event = true;
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
		/* Only buzz if no phone is connected to receive the message */
		if (!get_state()->ble_connected) {
			buzzer_play(MELODY_MSG_CONTACT);
		}
#endif
		break;

	case UI_EVENT_CHANNEL_MSG:
		is_msg_event = true;
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
		if (!get_state()->ble_connected) {
			buzzer_play(MELODY_MSG_CHANNEL);
		}
#endif
		break;

	case UI_EVENT_ROOM_MSG:
		is_msg_event = true;
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
		if (!get_state()->ble_connected) {
			buzzer_play(MELODY_MSG_CHANNEL);
		}
#endif
		break;

	case UI_EVENT_ACK:
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
		buzzer_play(MELODY_ACK);
#endif
		break;

	case UI_EVENT_BLE_CONNECTED:
		ui_set_ble_status(true, NULL);
		break;

	case UI_EVENT_BLE_DISCONNECTED:
		ui_set_ble_status(false, NULL);
		break;

	default:
		break;
	}

	/* On message events: flash heartbeat LED immediately (if not BLE connected
	 * and LEDs are enabled). Cancel the current cycle, turn on now, let
	 * led_off_work resume the normal heartbeat after LED_ON_MSG_MS. */
#if HAS_HEARTBEAT_LED
	if (is_msg_event && !get_state()->ble_connected && !get_state()->leds_disabled
	    && gpio_is_ready_dt(&heartbeat_led)) {
		k_work_cancel_delayable(&led_on_work);
		k_work_cancel_delayable(&led_off_work);
		gpio_pin_set_dt(&heartbeat_led, 1);
		k_work_reschedule(&led_off_work, K_MSEC(LED_ON_MSG_MS));
	}
#endif

	/* Wake display on non-message notifications (BLE connect/disconnect etc).
	 * Message notifications use buzzer + LED flash instead of waking the display. */
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
#ifndef ZEPHCORE_REPEATER
	if (!is_msg_event) {
		mc_display_on();
		schedule_render();
	}
#endif
#endif
}

void ui_set_msg_count(uint16_t count)
{
	struct ui_state *s = get_state();

	if (s->msg_count == count) {
		return;
	}

	s->msg_count = count;

	if (!ui_initialized) {
		return;
	}

	/* Always render when the display is already on (user is looking). */
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	if (mc_display_is_on()) {
		schedule_render();
		return;
	}
#endif

	/* EPD is bistable and readable without backlight. If the user is parked
	 * on the messages page, update the count silently via partial refresh
	 * without waking the backlight. Any other page: leave it for the next
	 * button press to avoid a pointless flash. */
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	if (mc_display_is_epd() && ui_pages_current() == UI_PAGE_MESSAGES) {
		schedule_render();
	}
#endif
}

void ui_set_ble_status(bool connected, const char *name)
{
	struct ui_state *s = get_state();

	s->ble_connected = connected;
	if (name) {
		strncpy(s->device_name, name, sizeof(s->device_name) - 1);
		s->device_name[sizeof(s->device_name) - 1] = '\0';
	}

	if (ui_initialized) {
		schedule_render();
	}
}

void ui_set_radio_params(uint32_t freq_hz, uint8_t sf, uint16_t bw_khz_x10,
			 uint8_t cr, int8_t tx_power, int16_t noise_floor)
{
	struct ui_state *s = get_state();

	s->lora_freq_hz = freq_hz;
	s->lora_sf = sf;
	s->lora_bw_khz_x10 = bw_khz_x10;
	s->lora_cr = cr;
	s->lora_tx_power = tx_power;
	s->lora_noise_floor = noise_floor;
}

void ui_set_gps_data(bool has_fix, uint8_t sats,
		     int32_t lat_mdeg, int32_t lon_mdeg, int32_t alt_mm)
{
	struct ui_state *s = get_state();

	s->gps_has_fix = has_fix;
	s->gps_satellites = sats;
	s->gps_lat_mdeg = lat_mdeg;
	s->gps_lon_mdeg = lon_mdeg;
	s->gps_alt_mm = alt_mm;
}

void ui_set_battery(uint16_t mv, uint8_t pct)
{
	struct ui_state *s = get_state();

	s->battery_mv = mv;
	s->battery_pct = pct;
}

void ui_set_clock(uint32_t epoch)
{
	struct ui_state *s = get_state();

	s->rtc_epoch = epoch;
}

void ui_add_recent(const char *name, int16_t rssi, uint32_t age_s)
{
	struct ui_state *s = get_state();

	/* Shift entries down if full */
	if (s->recent_count >= 4) {
		memmove(&s->recent[1], &s->recent[0], sizeof(s->recent[0]) * 3);
	} else {
		if (s->recent_count > 0) {
			memmove(&s->recent[1], &s->recent[0],
				sizeof(s->recent[0]) * s->recent_count);
		}
		s->recent_count++;
	}

	/* Add new entry at top — sanitize UTF-8 to Latin-1 for display font */
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	utf8_to_latin1(s->recent[0].name, name, sizeof(s->recent[0].name));
#else
	strncpy(s->recent[0].name, name, sizeof(s->recent[0].name) - 1);
	s->recent[0].name[sizeof(s->recent[0].name) - 1] = '\0';
#endif
	s->recent[0].rssi = rssi;
	s->recent[0].age_s = age_s;
}

void ui_clear_recent(void)
{
	struct ui_state *s = get_state();

	s->recent_count = 0;
	memset(s->recent, 0, sizeof(s->recent));
}

void ui_set_node_name(const char *name)
{
	struct ui_state *s = get_state();

	if (name) {
		strncpy(s->node_name, name, sizeof(s->node_name) - 1);
		s->node_name[sizeof(s->node_name) - 1] = '\0';
	}
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	ui_pages_set_node_name(name);
#endif
}

void ui_set_sensor_data(int16_t temp_c10, uint32_t pressure_pa,
			uint16_t humidity_rh10, uint16_t light_lux)
{
	struct ui_state *s = get_state();

	s->temperature_c10 = temp_c10;
	s->pressure_pa = pressure_pa;
	s->humidity_rh10 = humidity_rh10;
	s->light_lux = light_lux;
}

void ui_set_gps_available(bool available)
{
	struct ui_state *s = get_state();

	s->gps_available = available;
}

void ui_set_gps_enabled(bool enabled)
{
	struct ui_state *s = get_state();

	s->gps_enabled = enabled;
}

void ui_set_gps_state(uint8_t state, uint32_t last_fix_age_s, uint32_t next_search_s)
{
	struct ui_state *s = get_state();

	s->gps_state = state;
	s->gps_last_fix_age_s = last_fix_age_s;
	s->gps_next_search_s = next_search_s;
}

void ui_set_ble_enabled(bool enabled)
{
	struct ui_state *s = get_state();

	s->ble_enabled = enabled;
}

void ui_set_buzzer_quiet(bool quiet)
{
	struct ui_state *s = get_state();

	s->buzzer_quiet = quiet;
}

void ui_set_offgrid_mode(bool enabled)
{
	struct ui_state *s = get_state();

	s->offgrid_enabled = enabled;
}

void ui_set_leds_disabled(bool disabled)
{
	struct ui_state *s = get_state();

	s->leds_disabled = disabled;
}

void ui_set_heartbeat_led(bool enabled)
{
#if HAS_HEARTBEAT_LED
	if (enabled) {
		if (gpio_is_ready_dt(&heartbeat_led)) {
			k_work_reschedule(&led_on_work, K_NO_WAIT);
		}
	} else {
		k_work_cancel_delayable(&led_on_work);
		k_work_cancel_delayable(&led_off_work);
		gpio_pin_set_dt(&heartbeat_led, 0);
	}
#endif
}

void ui_refresh_display(void)
{
	if (!ui_initialized) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	/* EPD displays: skip periodic housekeeping renders.
	 * Each full e-paper refresh takes ~2s and causes visible flashing.
	 * All meaningful events (messages, BLE, GPS fix, button presses)
	 * already trigger renders via their own ui_set_*() → schedule_render().
	 * Housekeeping just updates slow-changing data (clock, contact ages)
	 * which will appear on the next event-driven render. */
	if (mc_display_is_epd()) {
		return;
	}

	schedule_render();
#endif
}
