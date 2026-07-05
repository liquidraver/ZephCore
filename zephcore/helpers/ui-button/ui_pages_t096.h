/*
 * ZephCore - Heltec T096 UI Page Renderer
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHCORE_UI_PAGES_T096_H
#define ZEPHCORE_UI_PAGES_T096_H

#include "ui_pages.h"

#include <stdbool.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_RENDERER_T096)
bool ui_pages_t096_render(enum ui_page page, const struct ui_state *state,
			  int page_index, int page_count);
bool ui_pages_t096_renderer_available(void);
bool ui_pages_t096_render_system_off_notice(void);
#else
static inline bool ui_pages_t096_render(enum ui_page page,
					const struct ui_state *state,
					int page_index, int page_count)
{
	(void)page;
	(void)state;
	(void)page_index;
	(void)page_count;
	return false;
}

static inline bool ui_pages_t096_renderer_available(void)
{
	return false;
}

static inline bool ui_pages_t096_render_system_off_notice(void)
{
	return false;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_UI_PAGES_T096_H */
