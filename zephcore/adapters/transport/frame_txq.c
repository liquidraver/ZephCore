/*
 * SPDX-License-Identifier: MIT
 * Companion frame TX queue — see frame_txq.h.
 */

#include "frame_txq.h"

#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_txq, CONFIG_ZEPHCORE_BLE_LOG_LEVEL);

#define TXQ_OVERFLOW_RETRY_MS 250

static void retry_work_fn(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct frame_txq *t = CONTAINER_OF(dw, struct frame_txq, retry_work);

	if (!t->overflow_pending) {
		return;
	}

	/* Abandon the parked push if the link is gone */
	if (!t->link_up()) {
		t->overflow_pending = false;
		t->congested = false;
		LOG_INF("%s: overflow cleared (disconnected)", t->name);
		return;
	}

	if (k_msgq_put(t->q, &t->overflow, K_NO_WAIT) == 0) {
		t->overflow_pending = false;
		LOG_DBG("%s: overflow frame queued hdr=0x%02x, kicking drain", t->name,
			t->overflow.buf[0]);
		t->kick();
		/* Congestion is cleared by the drain at the low-water mark */
	} else {
		LOG_DBG("%s: overflow retry: queue still full, retry in %dms", t->name,
			TXQ_OVERFLOW_RETRY_MS);
		k_work_schedule(&t->retry_work, K_MSEC(TXQ_OVERFLOW_RETRY_MS));
	}
}

void frame_txq_init(struct frame_txq *t, struct k_msgq *q, uint32_t size, const char *name,
		    bool (*link_up)(void), void (*kick)(void))
{
	t->q = q;
	t->size = size;
	t->name = name;
	t->link_up = link_up;
	t->kick = kick;
	t->congested = false;
	t->overflow_pending = false;
	k_work_init_delayable(&t->retry_work, retry_work_fn);
}

size_t frame_txq_put(struct frame_txq *t, const uint8_t *data, uint16_t len)
{
	struct frame f;

	if (len == 0 || len > MAX_FRAME_SIZE) {
		LOG_WRN("%s: invalid len=%u", t->name, (unsigned)len);
		return 0;
	}
	f.len = len;
	memcpy(f.buf, data, len);

	if (k_msgq_put(t->q, &f, K_NO_WAIT) != 0) {
		/* Queue full: enter congestion so senders hold off. Blocking would
		 * stall LoRa; dropping would lose frames. */
		if (!t->congested) {
			LOG_WRN("%s: TX queue full (%u/%u), entering congestion", t->name,
				k_msgq_num_used_get(t->q), (unsigned)t->size);
			t->congested = true;
		}

		/* A lossless response is never parked: fail, the caller retries */
		if (companion_is_lossless_protocol_frame(data, len)) {
			LOG_DBG("%s: TX queue full for lossless frame hdr=0x%02x, retry later",
				t->name, data[0]);
			return 0;
		}

		/* One lossy push may wait in the overflow slot. A second is dropped
		 * rather than clobber it: most push codes carry per-event data.
		 * Chat messages are safe either way, in the offline queue. */
		if (t->overflow_pending) {
			LOG_WRN("%s: overflow full, dropping push hdr=0x%02x", t->name, data[0]);
			return 0;
		}
		t->overflow = f;
		t->overflow_pending = true;
		k_work_schedule(&t->retry_work, K_MSEC(TXQ_OVERFLOW_RETRY_MS));
		return len;
	}

	LOG_DBG("%s: queued len=%u hdr=0x%02x queue=%u", t->name, (unsigned)len, data[0],
		k_msgq_num_used_get(t->q));
	t->kick();
	return len;
}

int frame_txq_get(struct frame_txq *t, struct frame *f)
{
	if (k_msgq_get(t->q, f, K_NO_WAIT) != 0) {
		if (t->congested) {
			t->congested = false;
			LOG_INF("%s: congestion cleared (queue empty)", t->name);
		}
		return -EAGAIN;
	}

	/* Hysteresis: on at full, off at 1/3, for headroom before full again */
	if (t->congested) {
		uint32_t used = k_msgq_num_used_get(t->q);

		if (used <= t->size / 3) {
			t->congested = false;
			LOG_INF("%s: congestion cleared (queue=%u/%u)", t->name, used,
				(unsigned)t->size);
		}
	}
	return 0;
}

bool frame_txq_busy(const struct frame_txq *t)
{
	return t->congested || k_msgq_num_used_get(t->q) >= (t->size * 2 / 3);
}

bool frame_txq_empty(const struct frame_txq *t)
{
	return k_msgq_num_used_get(t->q) == 0 && !t->overflow_pending;
}

void frame_txq_reset(struct frame_txq *t)
{
	k_work_cancel_delayable(&t->retry_work);
	k_msgq_purge(t->q);
	t->overflow_pending = false;
	t->congested = false;
}
