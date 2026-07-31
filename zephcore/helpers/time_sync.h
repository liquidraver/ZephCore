/*
 * SPDX-License-Identifier: MIT
 * Joystick UI: track which source last set the RTC (System > Info > Time).
 */

#pragma once

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

enum time_sync_source {
	TIME_SYNC_NONE = 0,
	TIME_SYNC_GPS,
	TIME_SYNC_APP,
	TIME_SYNC_WIFI,
	TIME_SYNC_CLI,
	TIME_SYNC_MESH,   /* mesh time-sync consensus step */
};

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK) || \
    IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_BUTTON)

void time_sync_report(enum time_sync_source src);

/* Current effective time source (prefers live GPS sync). TIME_SYNC_NONE
 * if the RTC has never been set. */
enum time_sync_source time_sync_get_source(void);

#else

static inline void time_sync_report(enum time_sync_source src)
{
	ARG_UNUSED(src);
}

static inline enum time_sync_source time_sync_get_source(void)
{
	return TIME_SYNC_NONE;
}

#endif /* CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK || _BUTTON */

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
const char *time_sync_display_label(void);
#endif

#ifdef __cplusplus
}
#endif
