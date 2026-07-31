/*
 * Copyright 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Multi-tap input filter.
 *
 * Counts rapid taps within a configurable window and emits a different
 * key code based on the tap count.  Unlike the old triple-tap filter
 * (which emitted immediately at count==3), this filter ALWAYS waits for
 * the tap window to expire before emitting — except when the maximum
 * tap count is reached, in which case it emits immediately.
 *
 * This means single-tap has ~tap-delay-ms latency, which is acceptable
 * for UI actions like page-next.
 */

#define DT_DRV_COMPAT zephcore_input_multi_tap

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(input_multi_tap, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

struct multi_tap_config {
	const struct device *input_dev;
	const uint16_t *input_codes;
	const uint16_t *tap_codes;
	uint32_t tap_delay_ms;
	uint8_t num_input_codes;
	uint8_t num_tap_codes; /* max taps recognized (length of tap_codes) */
};

struct multi_tap_data {
	const struct device *dev;
	struct k_work_delayable work;
	uint8_t tap_count;
};

static void multi_tap_emit(struct multi_tap_data *data)
{
	const struct device *dev = data->dev;
	const struct multi_tap_config *cfg = dev->config;

	if (data->tap_count > 0 && data->tap_count <= cfg->num_tap_codes) {
		uint16_t code = cfg->tap_codes[data->tap_count - 1];

		LOG_DBG("multi-tap: %u tap(s) -> emit code %u", data->tap_count, code);
		input_report_key(dev, code, 1, true, K_FOREVER);
		input_report_key(dev, code, 0, true, K_FOREVER);
	}
	data->tap_count = 0;
}

static void multi_tap_deferred(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct multi_tap_data *data =
		CONTAINER_OF(dwork, struct multi_tap_data, work);

	/* Window expired — emit code for the tap count we saw */
	multi_tap_emit(data);
}

static void __maybe_unused multi_tap_cb(struct input_event *evt, void *user_data)
{
	const struct device *dev = user_data;
	const struct multi_tap_config *cfg = dev->config;
	struct multi_tap_data *data = dev->data;

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	/* Only count key-press events (value=1), ignore releases */
	if (!evt->value) {
		return;
	}

	LOG_DBG("code=%u (watching %u codes, first=%u)",
		evt->code, cfg->num_input_codes,
		cfg->num_input_codes > 0 ? cfg->input_codes[0] : 0);

	/* Check if this is a watched input code */
	bool found = false;

	for (int i = 0; i < cfg->num_input_codes; i++) {
		if (evt->code == cfg->input_codes[i]) {
			found = true;
			break;
		}
	}
	if (!found) {
		return;
	}

	data->tap_count++;
	LOG_DBG("multi-tap: tap %u/%u", data->tap_count, cfg->num_tap_codes);

	if (data->tap_count >= cfg->num_tap_codes) {
		/* Reached max tap count — emit immediately, no point waiting */
		k_work_cancel_delayable(&data->work);
		multi_tap_emit(data);
	} else {
		/* Restart window timer */
		k_work_reschedule(&data->work, K_MSEC(cfg->tap_delay_ms));
	}
}

static int __maybe_unused multi_tap_init(const struct device *dev)
{
	const struct multi_tap_config *cfg = dev->config;
	struct multi_tap_data *data = dev->data;

	if (cfg->input_dev && !device_is_ready(cfg->input_dev)) {
		LOG_ERR("input device not ready");
		return -ENODEV;
	}

	data->dev = dev;
	data->tap_count = 0;
	k_work_init_delayable(&data->work, multi_tap_deferred);

	LOG_INF("multi-tap init: input_dev=%p, %u input codes, %u tap codes, delay=%u ms",
		cfg->input_dev, cfg->num_input_codes, cfg->num_tap_codes, cfg->tap_delay_ms);

	return 0;
}

#define INPUT_MULTI_TAP_DEFINE(inst)                                                              \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, input_codes) >= 1,                                   \
		     "input-codes must have at least 1 entry");                                    \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, tap_codes) >= 1,                                     \
		     "tap-codes must have at least 1 entry");                                      \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, tap_codes) <= 5,                                     \
		     "tap-codes must have at most 5 entries");                                     \
                                                                                                   \
	INPUT_CALLBACK_DEFINE_NAMED(                                                               \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, input),                                   \
			(DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, input))),                    \
			(NULL)),                                                                   \
		multi_tap_cb, (void *)DEVICE_DT_INST_GET(inst),                                    \
		multi_tap_cb_##inst);                                                              \
                                                                                                   \
	static const uint16_t multi_tap_input_codes_##inst[] =                                     \
		DT_INST_PROP(inst, input_codes);                                                   \
                                                                                                   \
	static const uint16_t multi_tap_codes_##inst[] =                                           \
		DT_INST_PROP(inst, tap_codes);                                                     \
                                                                                                   \
	static struct multi_tap_data multi_tap_data_##inst;                                        \
                                                                                                   \
	static const struct multi_tap_config multi_tap_config_##inst = {                           \
		.input_dev = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, input),                      \
			(DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, input))),                    \
			(NULL)),                                                                   \
		.input_codes = multi_tap_input_codes_##inst,                                       \
		.tap_codes = multi_tap_codes_##inst,                                               \
		.tap_delay_ms = DT_INST_PROP(inst, tap_delay_ms),                                  \
		.num_input_codes = DT_INST_PROP_LEN(inst, input_codes),                            \
		.num_tap_codes = DT_INST_PROP_LEN(inst, tap_codes),                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, multi_tap_init, NULL,                                          \
			      &multi_tap_data_##inst,                                              \
			      &multi_tap_config_##inst,                                            \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_MULTI_TAP_DEFINE)
