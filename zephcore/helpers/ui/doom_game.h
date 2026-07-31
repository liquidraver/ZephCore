/*
 * ZephCore - Doom Raycaster Easter Egg
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Hidden easter egg: Wolf3D-style raycaster on the OLED display.
 * Activated by double-click + single-click joystick center on page 0.
 * Deactivated by double-click joystick center.
 *
 * Build-time control: CONFIG_ZEPHCORE_EASTER_EGG_DOOM
 */

#ifndef ZEPHCORE_DOOM_GAME_H
#define ZEPHCORE_DOOM_GAME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM

/**
 * Start the Doom easter egg game.
 * Takes over display and input until doom_game_stop() is called.
 */
void doom_game_start(void);

/**
 * Stop the Doom easter egg game.
 * Restores display to normal firmware UI.
 */
void doom_game_stop(void);

/**
 * Check if Doom is currently running.
 * Used by display and input guards to redirect I/O.
 */
bool doom_game_is_running(void);

/**
 * Forward an input event to the Doom game.
 * Called from ui_input_cb() when game is running.
 *
 * @param code  Input key code (INPUT_KEY_UP, etc.)
 * @param value 1 = pressed, 0 = released
 */
void doom_game_input(uint16_t code, int32_t value);

#else /* !CONFIG_ZEPHCORE_EASTER_EGG_DOOM */

static inline void doom_game_start(void) {}
static inline void doom_game_stop(void) {}
static inline bool doom_game_is_running(void) { return false; }
static inline void doom_game_input(uint16_t code, int32_t value)
{
	(void)code;
	(void)value;
}

#endif /* CONFIG_ZEPHCORE_EASTER_EGG_DOOM */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_DOOM_GAME_H */
