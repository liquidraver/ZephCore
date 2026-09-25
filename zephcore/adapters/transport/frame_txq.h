/*
 * SPDX-License-Identifier: MIT
 * Companion frame TX queue, shared by the queued transports (BLE, TCP).
 *
 * The transport owns the drain (a GATT notify chain, a socket send); this owns
 * everything before it, identically for every transport:
 *   - a frame queue;
 *   - congestion: set when the queue is full, cleared at 1/3 (hysteresis), and
 *     reported through frame_txq_busy() so senders (the contact dump) hold off;
 *   - lossless protocol responses (< 0x80) are never dropped: when the queue is
 *     full the put fails and the caller retries;
 *   - one lossy push (>= 0x80) is parked in an overflow slot and re-queued every
 *     250 ms until it fits or the link goes down; a second one is dropped rather
 *     than overwrite it.
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include "companion_framing.h"

#ifdef __cplusplus
extern "C" {
#endif

struct frame_txq {
	struct k_msgq *q;
	uint32_t size;                 /* queue depth, for the water marks */
	const char *name;              /* log prefix */
	bool (*link_up)(void);         /* overflow is abandoned while false */
	void (*kick)(void);            /* start the transport's drain */
	bool congested;
	bool overflow_pending;
	struct frame overflow;
	struct k_work_delayable retry_work;
};

void frame_txq_init(struct frame_txq *t, struct k_msgq *q, uint32_t size, const char *name,
		    bool (*link_up)(void), void (*kick)(void));

/* Queue a frame and kick the drain. Returns len when queued (or parked in the
 * overflow slot), 0 when it was not accepted. */
size_t frame_txq_put(struct frame_txq *t, const uint8_t *data, uint16_t len);

/* Next frame for the drain: 0, or -EAGAIN when the queue is empty (congestion
 * is cleared then; the caller signals TX idle). Clears congestion at 1/3. */
int frame_txq_get(struct frame_txq *t, struct frame *f);

/* Senders should hold off: congested, or at the 2/3 high-water mark. */
bool frame_txq_busy(const struct frame_txq *t);

/* Nothing queued and nothing parked. */
bool frame_txq_empty(const struct frame_txq *t);

/* Drop everything (link down). */
void frame_txq_reset(struct frame_txq *t);

#ifdef __cplusplus
}
#endif
