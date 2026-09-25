/*
 * ZephCore - UI Task
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Integration layer that wires buttons, buzzer, and display together.
 * All event-driven via Zephyr input subsystem + k_work.
 *
 * Input flow (after longpress + multi-tap filter chain).  This file maps
 * key code → action only; which gesture emits which code is per-board, set by
 * the tap-codes array in that board's devicetree.  Tap counts below are the
 * common case, not a guarantee — e.g. Heltec T096 has no buzzer and gives the
 * 2-tap slot to page-prev, pushing KEY_B out to 3 taps.
 *
 *   KEY_1     → action_page_next()       (1 tap, 400ms delayed)
 *   KEY_LEFT  → action_page_prev()       (2 taps on boards with a screen)
 *   KEY_B     → action_leds_toggle()     (LED heartbeat; 2 taps stock)
 *   KEY_D     → action_buzzer_toggle()   (notification mode; 3 taps stock)
 *   KEY_C     → action_gps_toggle()      (4 taps)
 *   KEY_E     → action_flood_advert()    (5 taps)
 *   KEY_G     → GPS switch on/off        (hardware toggle, ThinkNode M1)
 *   KEY_POWER / KEY_F → action_deep_sleep() (long press — boards that emit these)
 *   KEY_ENTER → action_page_enter()      (long press — Pocket / Heltec; joystick center Wio)
 *   KEY_RIGHT → action_page_next()       (joystick, Wio Tracker)
 *
 * The last entry of a board's tap-codes fires immediately (no tap-delay-ms
 * wait) — the filter knows no further tap can extend it.
 *
 * Notification flow:
 *   LoRa RX → CompanionMesh → ui_notify(UI_EVENT_CONTACT_MSG)
 *     → buzzer_play(MELODY_MSG_CONTACT)
 *     → mc_display_on() + render
 */

#include "ui_task.h"
#include "ui_pages.h"
#include <helpers/buzzer_gate.h>
#include <time_sync.h>

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

#include <zephyr_poweroff.h>

#include <zephyr/sys/reboot.h>

/* GPS control */
#include <ZephyrGPSManager.h>

/* Mesh action wrappers (deferred to mesh event loop thread) */
#include "ui_mesh_actions.h"

/* LED helpers defined in ui_common.c */
extern void ui_led_flash_msg(void);

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ui_task, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* Action feedback melodies.
 * b=200, d=16 → each chirp ~75ms, rest ~75ms = clear separation.
 *
 * The chirp count identifies the action, not the tap count — the two are only
 * equal on boards using the stock tap-codes order, and LED already differs
 * there (2 taps, 5 chirps).  Do not "fix" a count to match a gesture.
 *
 * advert sent (zero-hop or flood): chirp-chirp
 * buzzer / notification mode:      chirp×3 + high(ON) or low(OFF)
 * GPS toggle:                      chirp×4 + high(ON) or low(OFF)
 * LED heartbeat toggle:            chirp×5 + high(ON) or low(OFF)
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

/* ========== Work Handlers ========== */

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

		/* Tiny panels flash the page title on a change, then need one
		 * more render to swap it for content once the flash expires. */
		uint32_t flash_ms = ui_pages_flash_remaining_ms();

		if (flash_ms > 0) {
			k_work_reschedule(&render_work, K_MSEC(flash_ms + 20));
		}
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
static void schedule_render_auto(void);
static void set_ble_connected(bool connected);

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
		/* Toggle GPS (same as the KEY_C tap action) */
		action_gps_toggle();
		break;

	case UI_PAGE_BUZZER:
		/* Cycle notification mode (same as the KEY_D tap action) */
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

/* Cycles sound -> vibrate -> silent -> sound. Boards with no motor skip the
 * middle step, so they keep the plain on/off toggle they always had. */
static void action_buzzer_toggle(void)
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	uint8_t mode = zephcore_buzzer_next_mode(get_state()->buzzer_mode);

	if (zephcore_buzzer_mode_audible(mode)) {
		/* Enable first, then play the ascending confirmation */
		zephcore_buzzer_set_mode(mode, false);
		buzzer_play(MELODY_BUZZER_ON);
	} else {
		/* Play the descending confirmation while still audible — the
		 * deferred mute lets it finish before sounds are suppressed. */
		buzzer_play(MELODY_BUZZER_OFF);
		zephcore_buzzer_set_mode(mode, true);
	}

	/* Persist across reboots */
	mesh_set_buzzer_mode(mode);
	get_state()->buzzer_mode = mode;
	LOG_INF("buzzer mode=%u", mode);
#endif
	schedule_render();
}

static void action_leds_toggle(void)
{
	struct ui_state *s = get_state();
	bool new_disabled = !s->leds_disabled;

	ui_set_leds_disabled(new_disabled);
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

	/* Shutdown melody — blocking wait is fine on this terminal path. */
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play(MELODY_SHUTDOWN);
	while (buzzer_is_playing()) {
		k_sleep(K_MSEC(50));
	}
	buzzer_stop();

	if (buzzer_is_quiet()) {
		ui_led_flash_shutdown();
	}
#endif

	/* The one power-off path (adapters/board/zephyr_poweroff.c), shared
	 * with the joystick UI and the CLI. */
	zephcore_shutdown_reason_save(ZC_SHUTDOWN_USER);
	zephcore_power_off();
#else
	LOG_WRN("deep sleep: CONFIG_POWEROFF not enabled");
#endif
}

/* ========== Input Event Handler ========== */

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
/* Set when a button press woke the display; consumed by the resolved action
 * code so the wake-up press does not also run its normal action.
 *
 * The display wakes on the raw press event (INPUT_KEY_0 from gpio-keys), but
 * the action it maps to is emitted later by the longpress / multi-tap filters
 * (up to tap-delay-ms or long-delay-ms afterwards).  By then the display
 * reports as on, so the resolved code used to fall through and run normally —
 * a single tap woke the screen and immediately paged forward.
 *
 * The flag is matched against real action codes rather than "the next event"
 * because the multi-tap filter passes an unhandled raw KEY_A through per tap
 * before it resolves; swallowing that would leave a double tap still acting.
 *
 * No expiry is needed, but the flag must only be armed when the wake press was
 * a raw code that still has an action code coming.  ZEPHCORE_UI_DESIGN_JOYSTICK
 * depends on ZEPHCORE_ROLE_COMPANION, so the repeater build of a joystick board
 * (wio_tracker_l1, gat562_30s — both shipped as repeater artifacts) falls back
 * to ZEPHCORE_UI_DESIGN_BUTTON and *does* reach this callback, with its
 * joystick GPIOs wired straight to INPUT_KEY_UP/DOWN/LEFT/RIGHT/ENTER and no
 * filter in between.  There the wake press is already an action code and is
 * consumed at the wake, so the flag stays clear and the next press acts.
 */
static bool display_woken_pending;

static bool is_ui_action_code(uint16_t code)
{
	switch (code) {
	case INPUT_KEY_1:
	case INPUT_KEY_B:
	case INPUT_KEY_D:
	case INPUT_KEY_C:
	case INPUT_KEY_E:
	case INPUT_KEY_POWER:
	case INPUT_KEY_F:
	case INPUT_KEY_RIGHT:
	case INPUT_KEY_LEFT:
	case INPUT_KEY_ENTER:
	case INPUT_KEY_UP:
	case INPUT_KEY_DOWN:
		return true;
	default:
		return false;
	}
}
#endif /* CONFIG_ZEPHCORE_UI_DISPLAY */

static void ui_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	/* Joystick axis flip for upside-down mounts.  Mapped into a local rather
	 * than written back into the event: the input event is shared with every
	 * other INPUT_CALLBACK_DEFINE consumer on the bus, and this remap is a
	 * UI-layer convention, not a hardware fact.  Non-directional codes pass
	 * through untouched, so the tap-code and longpress paths are unaffected. */
	const uint16_t code = zephcore_input_map_code(evt->code);

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	/* When Doom is running, intercept ALL input (presses AND releases) */
	if (doom_game_is_running()) {
		/* Double-click ENTER to exit: detect two presses within 500ms */
		if (code == INPUT_KEY_ENTER && evt->value) {
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
		doom_game_input(code, evt->value);
		return;
	}
#endif

	/* GPS hardware switch (toggle switch, not momentary button).
	 * Needs both press (ON) and release (OFF) events,
	 * so handle before the release-event filter below. */
	if (code == INPUT_KEY_G) {
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
	/* If display is off, wake it and consume the event.  The action this
	 * press resolves to arrives later from the longpress / multi-tap filter;
	 * display_woken_pending makes the switch below swallow it.
	 *
	 * Only arm that flag when this press really does resolve into a *later*
	 * action code, i.e. when the code we just consumed is a raw one.  Boards
	 * that wire an action code straight to the GPIO (joystick lines on
	 * wio_tracker_l1 / gat562_30s in repeater builds) already consumed their
	 * action here — arming the flag would make the next, genuinely separate
	 * press get swallowed instead. */
	if (!mc_display_is_on()) {
		mc_display_on();
		display_woken_pending = !is_ui_action_code(code);
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

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	/* This is the action the wake-up press resolved to — swallow it so the
	 * press only woke the display, on whatever page it was showing. */
	if (display_woken_pending && is_ui_action_code(code)) {
		display_woken_pending = false;
		return;
	}
#endif

	/* Map input key codes to UI actions.
	 *
	 * Two-code boards (RAK4631 / WisMesh Pocket / Heltec V3–V4.3 / T-Echo /
	 * ThinkNode M1 / T-Impulse): 1 tap KEY_1, 2 taps KEY_LEFT; long press
	 * KEY_ENTER (page enter).  Five-code boards emit KEY_B/D/C/E as well, and
	 * KEY_POWER or KEY_F long → deep sleep.  The gesture behind each code is
	 * the board's business (see the tap-codes note at the top of this file).
	 * KEY_RIGHT/LEFT/ENTER/UP/DOWN: joystick (Wio Tracker)
	 *
	 * NOTE: KEY_A (raw short-press from longpress filter) is NOT handled
	 * here. It feeds into the multi-tap filter which emits the tap codes.
	 * Since INPUT_CALLBACK_DEFINE(NULL) sees events from all devices,
	 * the raw KEY_A events fall through to default: break.
	 */
	switch (code) {
	/* ===== Multi-tap outputs ===== */
	case INPUT_KEY_1:
		/* First tap-codes entry (400ms delayed): page next */
		action_page_next();
		break;

	case INPUT_KEY_B:
		/* Toggle LED heartbeat (2 taps stock, 3 on T096) */
		action_leds_toggle();
		break;

	case INPUT_KEY_D:
		/* Cycle notification mode; no-op where UI_BUZZER=n */
		action_buzzer_toggle();
		break;

	case INPUT_KEY_C:
		/* Toggle GPS */
		action_gps_toggle();
		break;

	case INPUT_KEY_E:
		/* Flood advert (last tap-codes entry, so immediate) */
		action_flood_advert();
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

	ui_led_heartbeat_init();

	/* NOTE: startup chime is NOT played here. It's played from main()
	 * after loadPrefs() so we can respect the persisted buzzer_quiet setting.
	 * See ui_play_startup_chime(). */

	ui_initialized = true;
	LOG_INF("UI initialized");

	/* Suppress unused variable warning when both display and buzzer are disabled */
	(void)ret;

	return 0;
}

void ui_notify_contact_msg(uint8_t path_len, const char *from_name,
			   const char *text, uint16_t msg_count)
{
	(void)path_len; (void)from_name; (void)text;
	ui_set_msg_count(msg_count);
	ui_notify(UI_EVENT_CONTACT_MSG);
}

void ui_notify_channel_msg(const char *channel_name, const char *text,
			   uint32_t ts, uint8_t path_len, uint16_t msg_count)
{
	(void)channel_name; (void)text; (void)ts; (void)path_len;
	ui_set_msg_count(msg_count);
	ui_notify(UI_EVENT_CHANNEL_MSG);
}

void ui_notify_packet_sent(void)
{
	/* Button UI has no per-packet tracking */
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
		set_ble_connected(true);
		break;

	case UI_EVENT_BLE_DISCONNECTED:
		set_ble_connected(false);
		break;

	default:
		break;
	}

	if (is_msg_event && !get_state()->ble_connected) {
		ui_led_flash_msg();
	}

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

static void set_ble_connected(bool connected)
{
	struct ui_state *s = get_state();

	s->ble_connected = connected;

	if (ui_initialized) {
		schedule_render();
	}
}

void ui_set_radio_params(uint32_t freq_hz, uint8_t sf, uint16_t bw_khz_x10,
			 uint8_t cr, int8_t tx_power, int16_t noise_floor)
{
	struct ui_state *s = get_state();
	bool changed = s->lora_freq_hz != freq_hz || s->lora_sf != sf ||
		       s->lora_bw_khz_x10 != bw_khz_x10 || s->lora_cr != cr ||
		       s->lora_tx_power != tx_power ||
		       s->lora_noise_floor != noise_floor;

	s->lora_freq_hz = freq_hz;
	s->lora_sf = sf;
	s->lora_bw_khz_x10 = bw_khz_x10;
	s->lora_cr = cr;
	s->lora_tx_power = tx_power;
	s->lora_noise_floor = noise_floor;

	if (changed) {
		schedule_render_auto();
	}
}

void ui_set_radio_runtime(uint8_t sync_word,
			  uint16_t preamble_len, bool rx_duty_cycle,
			  bool radio_ready, bool in_rx, bool tx_active)
{
	struct ui_state *s = get_state();
	bool changed = s->lora_sync_word != sync_word ||
		       s->lora_preamble_len != preamble_len ||
		       s->lora_rx_duty_cycle != rx_duty_cycle ||
		       s->lora_radio_ready != radio_ready ||
		       s->lora_in_rx != in_rx ||
		       s->lora_tx_active != tx_active;

	s->lora_sync_word = sync_word;
	s->lora_preamble_len = preamble_len;
	s->lora_rx_duty_cycle = rx_duty_cycle;
	s->lora_radio_ready = radio_ready;
	s->lora_in_rx = in_rx;
	s->lora_tx_active = tx_active;

	if (changed) {
		schedule_render_auto();
	}
}

void ui_set_radio_stats(uint32_t packets_rx, uint32_t packets_tx,
			uint32_t packets_err)
{
	struct ui_state *s = get_state();
#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	bool activity_changed = packets_rx != s->lora_packets_rx ||
				packets_tx != s->lora_packets_tx;
#endif

	s->lora_packets_rx = packets_rx;
	s->lora_packets_tx = packets_tx;
	s->lora_packets_err = packets_err;

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY
	if (ui_initialized && activity_changed && mc_display_has_color() &&
	    ui_pages_current() == UI_PAGE_TRAFFIC) {
		schedule_render();
	}
#endif
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
	static enum time_sync_source last_src = TIME_SYNC_NONE;

	/* 1735689600 = 2025-01-01: the same "time is set" threshold the top-bar
	 * renderer uses. The clock only becomes visible once epoch crosses it. */
	bool was_valid = s->rtc_epoch > 1735689600;
	bool now_valid = epoch > 1735689600;

	s->rtc_epoch = epoch;

	/* Repaint the EPD (which only redraws on demand) when the visible top-bar
	 * clock/source-tag changes, so the user never has to change pages to see
	 * an update. Both transitions are picked up on the next housekeeping tick:
	 *   - clock first becomes valid  → boot RTC restore / first sync
	 *   - source tag changes         → L→A (app), →G (GPS fix), A→L (12h expiry)
	 * Steady state triggers nothing (epoch valid + source unchanged). */
	enum time_sync_source src = time_sync_get_source();
	bool source_changed = (src != last_src);
	last_src = src;

	if (now_valid && (!was_valid || source_changed)) {
		schedule_render();
	}
}

void ui_set_tz(int8_t hours)
{
	struct ui_state *s = get_state();

	if (s->tz_offset == hours) {
		return;
	}
	s->tz_offset = hours;

	/* EPD only redraws on demand, so a timezone change would otherwise not
	 * appear until something else forced a repaint. */
	schedule_render();
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

void ui_set_buzzer_mode(uint8_t mode)
{
	struct ui_state *s = get_state();

	s->buzzer_mode = mode;
}

void ui_set_offgrid_mode(bool enabled)
{
	struct ui_state *s = get_state();

	s->offgrid_enabled = enabled;
}

/* Schedule a redraw for an automatic (non-interactive) state update, e.g. a
 * live radio value changing. Skips EPD panels, which only repaint on real
 * events to avoid ~2s flashes. */
static void schedule_render_auto(void)
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

/* ========== LED heartbeat overrides (ui_common.c weak functions) ========== */

uint16_t ui_led_get_msg_count(void)
{
	return get_state()->msg_count;
}

void ui_led_on_disabled_changed(bool disabled)
{
	get_state()->leds_disabled = disabled;
}
