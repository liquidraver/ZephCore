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

#ifdef __cplusplus
extern "C" {
#endif

/* true = every LED stays dark, including message and shutdown flashes. */
bool zephcore_leds_disabled(void);

/* Set the gate. Any LED currently lit is dealt with by zephcore_leds_ui_sync()
 * below; the momentary TX LED clears itself at the end of the transmit in
 * progress. Safe to call from any role, with or without a UI. */
void zephcore_leds_set_disabled(bool disabled);

/* Called by zephcore_leds_set_disabled() after the flag changes. Weak no-op in
 * led_gate.c; helpers/ui/ui_common.c overrides it to stop/restart the heartbeat
 * cycle and refresh the UI's LED page. Not meant to be called directly. */
void zephcore_leds_ui_sync(bool disabled);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_LED_GATE_H */
