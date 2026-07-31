/*
 * ZephCore - PWM Buzzer with RTTTL Melody Playback
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Non-blocking RTTTL melody playback using Zephyr PWM API.
 * Notes scheduled via k_work_delayable on a dedicated work queue.
 */

#ifndef ZEPHCORE_BUZZER_H
#define ZEPHCORE_BUZZER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Predefined RTTTL melodies */

#define MELODY_STARTUP     "Startup:d=4,o=5,b=160:16c6,16e6,8g6"
#define MELODY_SHUTDOWN    "Shutdown:d=4,o=5,b=100:8g5,16e5,16c5"
#define MELODY_MSG_CONTACT "MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7"
#define MELODY_MSG_CHANNEL "kerplop:d=16,o=6,b=120:32g#,32c#"
#define MELODY_ACK         "ack:d=32,o=8,b=120:c"

/**
 * Initialize buzzer from devicetree ('buzzer' alias → pwm-leds).
 * Optional 'buzzer-enable' alias for amplifier power gating.
 * @return 0 on success, -ENODEV if no buzzer in DT, negative errno on failure
 */
int buzzer_init(void);

/**
 * Play an RTTTL melody string (non-blocking). Stops any current melody.
 * No-op if quiet. String must remain valid until melody completes.
 */
void buzzer_play(const char *rtttl);

/** Stop current melody and silence the buzzer. */
void buzzer_stop(void);

/**
 * Set quiet mode. Stops current melody and gates amplifier power.
 * When quiet, buzzer_play() becomes a no-op.
 */
void buzzer_set_quiet(bool quiet);

/** @return true if muted */
bool buzzer_is_quiet(void);

/**
 * Set quiet mode without interrupting the current melody.
 * In-progress melody plays to completion; future plays suppressed.
 */
void buzzer_set_quiet_deferred(bool quiet);

/** @return true if a melody is currently playing */
bool buzzer_is_playing(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_BUZZER_H */
