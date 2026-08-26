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

/* Shared PWM brightness (0-100) for boards whose heartbeat/TX LED is
 * PWM-capable (DT_ALIAS(heartbeat_pwm_led) / DT_ALIAS(lora_tx_pwm_led)).
 * Deliberately RAM-only, not persisted: resets to
 * ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT on every reboot regardless of any
 * "set led <pct>" done during a previous session. Independent of the on/off
 * gate above: brightness only matters while the gate is open. No-op on
 * boards without a PWM-capable LED (the getter still returns the compiled
 * default, simply unused). */
#define ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT 10

uint8_t zephcore_led_brightness_pct(void);
void zephcore_led_set_brightness_pct(uint8_t pct);

/* Called by zephcore_leds_set_disabled() after the flag changes. Weak no-op in
 * led_gate.c; helpers/ui/ui_common.c overrides it to stop/restart the heartbeat
 * cycle and refresh the UI's LED page. Not meant to be called directly. */
void zephcore_leds_ui_sync(bool disabled);

/*
 * Mode values for the two per-LED settings.  They live here rather than in
 * NodePrefs.h, where the rest of the prefs enums are, because both consumers of
 * the heartbeat modes are C (helpers/ui/ui_common.c) and NodePrefs.h is C++.
 * NodePrefs.h includes this header so the persisted fields and these values
 * still have exactly one definition between them.
 *
 * leds.radio — what the LoRa activity LED (the `lora-tx-led` DT alias) reacts
 * to.  TX is deliberately value 0: it is what every node did before the setting
 * existed, so an absent byte in an old prefs file, a short read, and a
 * zero-filled byte all decode to the historical behaviour.  Same reasoning
 * applies to LEDS_HB_ALL.  Boards without the alias have no activity LED at all
 * and ignore this setting whatever it says.
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
 * Per-LED modes, below the master gate: "set leds off" still wins over both.
 *
 * Kept here rather than read from NodePrefs directly because the consumers are
 * on different threads from the CLI that writes them: the heartbeat runs on the
 * system work queue and the activity LED on the radio's TX path, so both need a
 * lock-free snapshot rather than a pointer into a struct the main thread edits.
 */
uint8_t zephcore_leds_radio_mode(void);
void zephcore_leds_set_radio_mode(uint8_t mode);

uint8_t zephcore_leds_hb_mode(void);
void zephcore_leds_set_hb_mode(uint8_t mode);

/*
 * Shared-pin arbitration.  On 8 of the supported boards the `lora-tx-led` alias
 * IS `led0`, so the heartbeat cycle and the radio activity LED drive the same
 * physical GPIO from two different modules.  Without this the two clip each
 * other: the heartbeat's off-work clears the pin in the middle of a transmit,
 * and onAfterTransmit() truncates a heartbeat pulse.
 *
 * ZephyrBoard raises the hold for the transmit window and for each RX pulse;
 * ui_common.c's heartbeat handlers skip lighting and clearing the pin while it
 * is held, so radio activity wins and the liveness tick yields.  Boards whose
 * pins differ compile the check out entirely (see ZEPHCORE_LED_PIN_SHARED).
 */
bool zephcore_led_radio_holds_pin(void);
void zephcore_led_radio_hold_pin(bool held);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_LED_GATE_H */
