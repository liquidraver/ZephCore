/*
 * ZephCore - Joystick UI Definitions
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Key codes and layout constants for the joystick UI.
 */

#pragma once

/* ===== Key codes =====
 *
 * These are the UI's INTERNAL codes, carried as a single char through
 * JoystickUITask::enqueueKey().  They are deliberately NOT the Zephyr
 * INPUT_KEY_* codes — joystick_ui_input_cb() translates between the two — and
 * they are never persisted or transmitted, so they can be renumbered freely.
 *
 * Key-space contract (keep this true):
 *   0x01-0x1F  control keys (navigation, enter, cancel)
 *   0x20-0x7E  RESERVED for printable characters typed on a keyboard
 *   0xF1-0xFF  long-press and global action keys
 *
 * The printable range is reserved so a keyboard board (ThinkNode M9's STC8H
 * matrix MCU at I2C 0x6C) can enqueue characters directly without colliding
 * with a control code.  The multi-tap action keys below used to sit at
 * 0x42-0x45 — i.e. on 'B'..'E' — which would have made typing "BCDE" fire a
 * flood advert, toggle GPS, mute the buzzer and kill the LED.
 *
 * Note KEY_ENTER and KEY_CANCEL are already ASCII CR and ESC, so a keyboard
 * that reports plain ASCII (the convention for this class of I2C matrix MCU)
 * produces correct enter/cancel with no translation at all.
 */
#define KEY_ENTER       0x0D   /* center/OK button click */
#define KEY_LEFT        0x01   /* joystick left */
#define KEY_RIGHT       0x02   /* joystick right */
#define KEY_UP          0x03   /* joystick up */
#define KEY_DOWN        0x04   /* joystick down */
#define KEY_CANCEL      0x1B   /* back/ESC button */
#define KEY_NEXT        0x05   /* single button next page */
#define KEY_PREV        0x06   /* single button prev page */
#define KEY_HOME        0x07   /* go to home screen */
#define KEY_SELECT      0x08   /* triple click / special */
/* Keys above 0x7F MUST be cast to char here.  They travel through the UI as a
 * plain `char` (enqueueKey / handleInput), and plain char is SIGNED on Xtensa
 * and RISC-V (ESP32-S3, ESP32-C6) while it is UNSIGNED on ARM.  Without the
 * cast, `char c = KEY_LED_TOGGLE` stores -8 but the bare constant promotes to
 * int 248, so every `c == KEY_LED_TOGGLE` compares -8 == 248 and silently
 * never matches on the ESP32 boards.  Casting makes both sides the same char
 * type, so the comparison holds under either signedness. */
#define KEY_ENTER_LONG  ((char)0xF1)   /* long press enter */
#define KEY_TO_TOP      ((char)0xF2)   /* long press up   → page up   */
#define KEY_TO_BOTTOM   ((char)0xF3)   /* long press down → page down */
#define KEY_LOCK        ((char)0xF4)   /* user button + joystick center held together → screen lock */

/* Global action keys emitted by multi tap filter, handled in loop().
 * Kept above 0xF0 so they stay clear of the printable range — see the
 * key-space contract above. */
#define KEY_FLOOD_ADVERT  ((char)0xF5)   /* INPUT_KEY_E: 5 taps → flood advert */
#define KEY_BUZZ_TOGGLE   ((char)0xF6)   /* INPUT_KEY_D: 3 taps → buzzer mute toggle */
#define KEY_GPS_TOGGLE    ((char)0xF7)   /* INPUT_KEY_C: 4 taps → GPS on/off */
#define KEY_LED_TOGGLE    ((char)0xF8)   /* INPUT_KEY_B: 2 taps → LED heartbeat toggle */

/* ===== Layout constants (calibrated for 128x64 OLED, 6x8 font) ===== */
/* All hard-coded offsets from old Arduino code are preserved here.
 * Resolution aware screens should use mc_display_width/height() directly. */
#define kHeaderSepY     11    /* y position of header separator line */
#define kContentY       14    /* y position where page content starts */
#define kMenuLineH      11    /* vertical spacing between menu items */
#define kBodyY          14    /* synonym for kContentY */
#define kLineH          9     /* compact line height */

/* Battery range for percentage calculation */
#define kBattMinMv      3000
#define kBattMaxMv      4200

/* Auto off timeout */
#define AUTO_OFF_MILLIS  30000UL   /* 30 seconds */

/* List view constants */
#define UI_RECENT_LIST_SIZE   4    /* max items visible in a scrollable list */

/* UI-only path_len status markers (no wire-format meaning beyond OUT_PATH_SENT,
 * which is exposed to the BLE app). These two encode the outcome of a local
 * channel send for the joystick UI's _ch_previews ring buffer:
 *   OUT_PATH_SENT_HEARD   — at least one neighbor repeated the flood
 *   OUT_PATH_SENT_UNHEARD — feedback window elapsed without hearing a repeat */
#define OUT_PATH_SENT_HEARD    0xFD
#define OUT_PATH_SENT_UNHEARD  0xFC
