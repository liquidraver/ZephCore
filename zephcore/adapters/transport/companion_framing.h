/*
 * SPDX-License-Identifier: MIT
 * What every companion transport shares: the frame unit, the serial framing,
 * the lossless/lossy split, and the callbacks a transport raises to main.
 *
 * Wire format of the byte-stream transports (MeshCore ArduinoSerialInterface /
 * SerialWifiInterface):
 *   App  -> Node:  '<' (0x3C) | len_LSB | len_MSB | payload...
 *   Node -> App:   '>' (0x3E) | len_LSB | len_MSB | payload...
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Same value as upstream's BaseSerialInterface.h (+4 over 172 for transport
 * codes), so the two definitions never disagree. */
#define MAX_FRAME_SIZE  176   // +4 for transport codes (region scoping)

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

#ifdef __cplusplus
extern "C" {
#endif

/* Raised by a transport, from its own thread or work queue: they must only
 * post events. main supplies one set for every transport. */
struct companion_link_cbs {
	void (*on_rx)(void);            /* a frame is waiting in the transport's recv queue */
	void (*on_tx_idle)(void);       /* the transport's TX has drained */
	void (*on_connected)(void);     /* a client can now be reached */
	void (*on_disconnected)(void);  /* that client went away */
};

#ifdef __cplusplus
}
#endif
