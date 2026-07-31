/*
 * SPDX-License-Identifier: MIT
 * STM32 CMSIS peripheral-macro fixup for the portable mesh:: core.
 *
 * STM32 CMSIS device headers (e.g. stm32wle5xx.h, pulled in transitively via
 * <zephyr/kernel.h> -> soc.h) define object-like macros for every on-chip
 * peripheral instance: `RNG`, `AES`, `ADC`, `RTC`, `CRC`, ... Several collide
 * with generic identifiers in the portable mesh core -- most notably the
 * mesh::RNG class.
 *
 * The collision is order-sensitive: zephcore .cpp files include mesh headers
 * (which declare e.g. class RNG) and only afterwards include a Zephyr header
 * (e.g. <zephyr/logging/log.h>) that pulls CMSIS in. So a naive `#undef RNG`
 * placed next to the class declaration runs before the macro even exists.
 *
 * Fix: force the CMSIS device header to be processed NOW (via <soc.h>) so its
 * include guard trips, then undef the colliding instance macros. The later
 * pull through <zephyr/kernel.h> then re-enters a guarded, already-processed
 * header and is a no-op, so the macros stay undef'd for the rest of the
 * translation unit -- regardless of whether this header was reached before or
 * after the first Zephyr include.
 *
 * Only zephcore C++ TUs include this; Zephyr's own STM32 drivers (which use the
 * CMSIS peripheral pointers legitimately) are separate translation units and
 * are unaffected. No-op on every non-STM32 platform.
 */

#pragma once

#if defined(CONFIG_SOC_FAMILY_STM32)

#include <soc.h>

/* Peripheral-instance macros whose names can collide with portable identifiers.
 * #undef of an undefined macro is a harmless no-op, so no per-symbol guards. */
#undef RNG
#undef AES
#undef ADC
#undef DAC
#undef RTC
#undef CRC
#undef PWR
#undef PKA
#undef TAMP
#undef COMP

#endif /* CONFIG_SOC_FAMILY_STM32 */
