/*
 * SPDX-License-Identifier: Apache-2.0
 * WiFi OTA Firmware Update — Public API
 *
 * Starts a WiFi AP + HTTP server for browser-based firmware upload.
 * Mirrors Arduino MeshCore's ElegantOTA behavior.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start WiFi AP + HTTP OTA server.
 *
 * Creates WiFi AP "ZephCore-OTA" with static IP 192.168.100.1,
 * starts DHCP server and HTTP server with upload page at /update.
 * The mesh continues running — LoRa is independent of WiFi.
 *
 * @param node_name  Device name (shown on OTA page)
 * @param board_name Board/manufacturer name (shown on OTA page)
 * @return 0 on success, -EALREADY if active, negative errno on failure
 */
int wifi_ota_start(const char *node_name, const char *board_name);

/**
 * Stop WiFi AP + HTTP OTA server.
 * Tears down HTTP server, DHCP server, and WiFi AP.
 *
 * @return 0 on success, negative errno on failure
 */
int wifi_ota_stop(void);

/**
 * Check if OTA mode is currently active.
 */
bool wifi_ota_is_active(void);

/**
 * Confirm the current MCUboot image at startup.
 * Call early in main() before mesh init.
 * If the running image is unconfirmed (first boot after OTA),
 * marks it as confirmed so MCUboot keeps it on next reboot.
 */
void wifi_ota_confirm_image(void);

#ifdef __cplusplus
}
#endif
