/*
 * SPDX-License-Identifier: MIT
 * Shared MeshCore companion serial framing — used by the non-BLE companion
 * transports (LinuxTCPTransport.c, SerialCompanionTransport.c).
 *
 * Wire format (MeshCore ArduinoSerialInterface):
 *   App  -> Node:  '<' (0x3C) | len_LSB | len_MSB | payload...
 *   Node -> App:   '>' (0x3E) | len_LSB | len_MSB | payload...
 *
 * `struct frame` is the unit stored in the ble_send_queue/ble_recv_queue; its
 * layout must match what main_companion.cpp puts on those queues (len + buf).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ZephyrBLE.h"   /* MAX_FRAME_SIZE */

#define COMPANION_FRAME_RX_SYNC '<'
#define COMPANION_FRAME_TX_SYNC '>'

/* Push codes >= 0x80 are lossy event signals (droppable under congestion);
 * protocol responses < 0x80 are lossless and must never be silently dropped. */
#define COMPANION_PUSH_CODE_BASE 0x80

struct frame {
	uint16_t len;
	uint8_t buf[MAX_FRAME_SIZE];
};

static inline bool companion_is_lossless_protocol_frame(const uint8_t *data,
							uint16_t len)
{
	return data != NULL && len > 0 && data[0] < COMPANION_PUSH_CODE_BASE;
}
