/*
 * ZephCore - LED master gate
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * One process-wide "are LEDs allowed" flag, consulted by every LED driver in
 * the firmware:
 *   - heartbeat / unread-message LEDs (helpers/ui/ui_common.c, UI builds only)
 *   - LoRa TX activity LED (adapters/board/ZephyrBoard.cpp, every role)
 *
 * It lives here rather than in ui_common.c because ui_common.c is only
 * compiled when a UI is enabled, while a repeater with no display still has a
 * blinking lora-tx-led that users want to be able to shut off.
 *
 * Set from persisted prefs at boot, and live via "set leds on|off" (all roles)
 * or the UI LED toggle page (companions with buttons/joystick).
 */

#ifndef ZEPHCORE_LED_GATE_H
#define ZEPHCORE_LED_GATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* true = every LED stays dark, including message and shutdown flashes. */
bool zephcore_leds_disabled(void);

/* Set the gate. Any LED currently lit is dealt with by zephcore_leds_ui_sync()
 * below; the momentary TX LED clears itself at the end of the transmit in
 * progress. Safe to call from any role, with or without a UI. */
void zephcore_leds_set_disabled(bool disabled);

/* Shared PWM brightness (0-100) for PWM-capable heartbeat/TX LEDs. Persisted
 * as prefs.led_brightness; the default only applies to a node with no saved
 * value. Independent of the on/off gate above. */
#define ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT 100

uint8_t zephcore_led_brightness_pct(void);
void zephcore_led_set_brightness_pct(uint8_t pct);

struct pwm_dt_spec;
/* Drive a PWM LED at the shared brightness when on, dark when off. */
void zephcore_led_pwm_write(const struct pwm_dt_spec *led, bool on);

/* Called by zephcore_leds_set_disabled() after the flag changes. Weak no-op in
 * led_gate.c; helpers/ui/ui_common.c overrides it to stop/restart the heartbeat
 * cycle and refresh the UI's LED page. Not meant to be called directly. */
void zephcore_leds_ui_sync(bool disabled);

/*
 * Mode values for the two per-LED settings (here, not in NodePrefs.h, because
 * the consumers are C). Value 0 is the historical behaviour in both, so an
 * absent or zero byte in an old prefs file decodes to it.
 */
#define LEDS_RADIO_TX   0   /* lit for the duration of each transmit (default) */
#define LEDS_RADIO_RX   1   /* short pulse on each packet received */
#define LEDS_RADIO_ALL  2   /* both of the above */
#define LEDS_RADIO_OFF  3   /* activity LED stays dark */
#define LEDS_RADIO_MAX  LEDS_RADIO_OFF

/*
 * leds.hb — what the heartbeat LED (`led0`, or `led1` on a board with no
 * `led0`) reacts to.  "unread" is not a separate blink: it is the existing
 * cycle widening its pulse from 20 ms to 200 ms, plus the second LED on boards
 * that have one.  So LEDS_HB_HB is "never widen", and LEDS_HB_UNREAD is "only
 * blink when it would have widened".
 */
#define LEDS_HB_ALL     0   /* liveness tick + unread indication (default) */
#define LEDS_HB_HB      1   /* liveness tick only, never widens */
#define LEDS_HB_UNREAD  2   /* dark unless there are unread messages */
#define LEDS_HB_OFF     3   /* heartbeat LED stays dark */
#define LEDS_HB_MAX     LEDS_HB_OFF

/*
 * Per-LED modes, below the master gate ("set leds off" wins). Lock-free
 * snapshots: the consumers run on other threads than the CLI that writes.
 */
uint8_t zephcore_leds_radio_mode(void);
void zephcore_leds_set_radio_mode(uint8_t mode);

uint8_t zephcore_leds_hb_mode(void);
void zephcore_leds_set_hb_mode(uint8_t mode);

/*
 * Shared-pin arbitration: where `lora-tx-led` is `led0`, the radio holds the
 * pin for a transmit or RX pulse and the heartbeat skips it meanwhile.
 * Compiled out where the pins differ (ZEPHCORE_LED_PIN_SHARED).
 */
bool zephcore_led_radio_holds_pin(void);
void zephcore_led_radio_hold_pin(bool held);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_LED_GATE_H */
