/*
 * ZephCore — typed-character reporting over Zephyr's input subsystem
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * A keyboard has to deliver arbitrary characters, and INPUT_KEY_* has no
 * codes for "the letter the user typed". Characters are therefore reported as
 * INPUT_EV_KEY events with a code of ZEPHCORE_INPUT_ASCII_BASE + <ascii>.
 *
 * The base sits above KEY_MAX (0x2FF), so an ASCII code can never be confused
 * with a real key. Do NOT be tempted to report the bare ASCII value instead:
 * the printable range 0x20-0x7E overlaps INPUT_KEY_* heavily — LEFT is 105,
 * RIGHT 106, HOME 102, PAGEUP 104, PAGEDOWN 109, F1 59, F2 60 — so a consumer
 * testing the raw code would treat a joystick left as the letter 'i'. That
 * bug was written once already; this header exists so it is not written again.
 */

#ifndef ZEPHCORE_INPUT_ASCII_H
#define ZEPHCORE_INPUT_ASCII_H

#define ZEPHCORE_INPUT_ASCII_BASE 0x300
#define ZEPHCORE_INPUT_ASCII_MIN  (ZEPHCORE_INPUT_ASCII_BASE + 0x20)
#define ZEPHCORE_INPUT_ASCII_MAX  (ZEPHCORE_INPUT_ASCII_BASE + 0x7E)

#define ZEPHCORE_INPUT_IS_ASCII(code) \
	((code) >= ZEPHCORE_INPUT_ASCII_MIN && (code) <= ZEPHCORE_INPUT_ASCII_MAX)

#define ZEPHCORE_INPUT_TO_ASCII(code) ((char)((code) - ZEPHCORE_INPUT_ASCII_BASE))

#endif /* ZEPHCORE_INPUT_ASCII_H */
