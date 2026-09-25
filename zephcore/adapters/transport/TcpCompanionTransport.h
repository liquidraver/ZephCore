/*
 * SPDX-License-Identifier: MIT
 * ZephCore TCP companion transport (MeshCore SerialWifiInterface framing)
 *
 * A TCP server for one app at a time, over Zephyr sockets: the native-Linux
 * builds' companion link, and the WiFi companion's. CompanionInterfaces.h
 * wraps it as a BaseSerialInterface for the MultiSerialInterface.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "companion_framing.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Callbacks are raised from the listener thread and the system work queue:
 * post events only. */
void tcp_companion_init(const struct companion_link_cbs *link);

/* Start listening on `port` (once; later calls are no-ops). Works before the
 * network is up: accept() simply waits. */
void tcp_companion_start(uint16_t port);

size_t tcp_companion_send(const uint8_t *data, uint16_t len);
size_t tcp_companion_recv(uint8_t *dest);

/* Disabling drops the client and refuses new ones. */
void tcp_companion_set_enabled(bool enable);
bool tcp_companion_is_enabled(void);

bool tcp_companion_is_connected(void);
bool tcp_companion_is_write_busy(void);
bool tcp_companion_tx_idle(void);

#ifdef __cplusplus
}
#endif
