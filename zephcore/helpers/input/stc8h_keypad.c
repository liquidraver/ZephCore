/*
 * ZephCore — STC8H companion-MCU keypad driver
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Reports keypresses from an I2C helper MCU through Zephyr's input subsystem.
 * Written against the register interface documented in the binding; no vendor
 * driver code is reused.
 *
 * Board context (ThinkNode M9): the keypad is the ONLY input device — that
 * board has no user button at hardware level, confirmed against both upstream
 * firmwares and the V1.0 schematic. Without this driver the UI renders but
 * cannot be operated at all.
 *
 * Key codes emitted
 * -----------------
 * Printable ASCII (0x20-0x7E) has no INPUT_KEY_* expression, so it is reported
 * offset above the whole INPUT_KEY_* code space as ZEPHCORE_INPUT_ASCII_BASE +
 * <ascii> (see helpers/input/zephcore_input_ascii.h — that header explains why
 * the bare ASCII value must NOT be used). The joystick UI unwraps it back to a
 * plain char, which is what its key-space contract reserves 0x20-0x7E for (see
 * helpers/ui-joystick/joystick_defs.h). Everything else is a named key and is
 * translated here.
 *
 * The named codes below (arrows included) come from the stock keypad
 * firmware's protocol, as recovered from a linked reference-firmware ELF by a
 * second independent firmware — the register bodies there are the shipped
 * hardware truth, not header guesses. Any code this driver still does not
 * recognise is reported at INFO level with its hex value so it can be added
 * to key_map[].
 */

#define DT_DRV_COMPAT zephcore_stc8h_keypad

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include "zephcore_input_ascii.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stc8h_keypad, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* Register map — see dts/bindings/input/zephcore,stc8h-keypad.yaml.
 *
 * The key register is 0x01, NOT the 0x05 "matrix key" register that the
 * reference header documents: the recovered implementation bodies read the
 * key from 0x01, and the 0x05 define is dead code nothing consumes. Reading
 * 0x05 returns constant 0xFF — that was this driver's original bug (keys
 * fully dead, phantom 0xFF at init). The overlap with the battery registers
 * (0x01..0x04 little-endian) is odd but is what the shipped firmware does;
 * it is also why key reads are IRQ-gated below. */
#define STC8H_REG_BATTERY 0x01 /* 0x01..0x04, little-endian millivolts */
#define STC8H_REG_KEY     0x01
#define STC8H_REG_STATE   0x06
#define STC8H_STATE_SLEEP 0x01

#define STC8H_KEY_NONE    0x00
#define STC8H_KEY_INVALID 0x88 /* helper MCU's "invalid key" marker */
#define STC8H_KEY_ALL_ONES 0xFF /* error sentinel / floating-bus read */

struct stc8h_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec irq;
	uint32_t poll_interval_ms;
};

struct stc8h_data {
	const struct device *dev;
	struct gpio_callback irq_cb;
	struct k_work_delayable work;
	atomic_t irq_pending;
	uint8_t last_key;
};

/*
 * Named keys, as reported by the helper MCU. Codes from the recovered stock
 * keypad protocol (see file header). The two function keys are mapped to page
 * navigation because on a board with no other input they are the only way to
 * move between screens; the UI treats them as prev/next.
 */
static const struct {
	uint8_t raw;
	uint16_t code;
} key_map[] = {
	{ 0x0D, INPUT_KEY_ENTER },  /* enter / confirm                     */
	{ 0xB5, INPUT_KEY_UP },     /* arrow up                            */
	{ 0xB6, INPUT_KEY_DOWN },   /* arrow down                          */
	{ 0xB4, INPUT_KEY_LEFT },   /* arrow left                          */
	{ 0xB7, INPUT_KEY_RIGHT },  /* arrow right                         */
	{ 0x08, INPUT_KEY_BACK },   /* del                -> back / cancel */
	{ 0x89, INPUT_KEY_BACK },   /* del long-press     -> back / cancel */
	{ 0x86, INPUT_KEY_ESC },    /* back                                */
	{ 0x82, INPUT_KEY_HOME },   /* home                                */
	{ 0x83, INPUT_KEY_MENU },   /* context menu                        */
	{ 0x81, INPUT_KEY_PAGEUP }, /* left function key  -> previous page */
	{ 0x85, INPUT_KEY_PAGEDOWN },/* right function key -> next page    */
	{ 0x87, INPUT_KEY_F1 },     /* long-press map pin                  */
	{ 0xA3, INPUT_KEY_F2 },     /* long-press enter                    */
};

static int stc8h_read_reg(const struct stc8h_config *cfg, uint8_t reg, uint8_t *val)
{
	return i2c_write_read_dt(&cfg->i2c, &reg, 1, val, 1);
}

/* Battery millivolts as measured by the helper MCU. Not consumed yet — the
 * board reads its own cell through the host ADC — but the registers exist and
 * a second opinion is cheap to expose. */
int stc8h_keypad_battery_mv(const struct device *dev, uint16_t *mv)
{
	const struct stc8h_config *cfg = dev->config;
	uint8_t reg = STC8H_REG_BATTERY;
	uint8_t buf[4];
	int ret;

	if (!mv) {
		return -EINVAL;
	}
	ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}
	*mv = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
	return 0;
}

int stc8h_keypad_sleep(const struct device *dev)
{
	const struct stc8h_config *cfg = dev->config;
	const uint8_t cmd[2] = { STC8H_REG_STATE, STC8H_STATE_SLEEP };

	return i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
}

static void stc8h_report(const struct device *dev, uint8_t raw)
{
	/* Printable characters are offset out of the INPUT_KEY_* code space —
	 * see zephcore_input_ascii.h for why the bare ASCII value must not be
	 * used here. */
	if (raw >= 0x20 && raw <= 0x7E) {
		uint16_t code = ZEPHCORE_INPUT_ASCII_BASE + raw;

		input_report_key(dev, code, 1, false, K_NO_WAIT);
		input_report_key(dev, code, 0, true, K_NO_WAIT);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(key_map); i++) {
		if (key_map[i].raw == raw) {
			input_report_key(dev, key_map[i].code, 1, false, K_NO_WAIT);
			input_report_key(dev, key_map[i].code, 0, true, K_NO_WAIT);
			return;
		}
	}

	/* Deliberately loud: this is how the missing arrow codes get identified.
	 * See the file header. */
	LOG_INF("unmapped keypad code 0x%02X — add it to key_map[] to bind it", raw);
}

static bool stc8h_key_valid(uint8_t raw)
{
	return raw != STC8H_KEY_NONE && raw != STC8H_KEY_INVALID &&
	       raw != STC8H_KEY_ALL_ONES;
}

static void stc8h_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct stc8h_data *data = CONTAINER_OF(dwork, struct stc8h_data, work);
	const struct device *dev = data->dev;
	const struct stc8h_config *cfg = dev->config;
	bool irq_event = atomic_cas(&data->irq_pending, 1, 0);
	uint8_t key = STC8H_KEY_NONE;

	if (stc8h_read_reg(cfg, STC8H_REG_KEY, &key) == 0) {
		if (irq_event) {
			/* KB_INT edge latched: this read is a fresh keypress —
			 * report unconditionally. The register latches the
			 * *last* key (it does not return to 0 on release), so
			 * comparing against last_key here would swallow every
			 * repeated press of the same key. This mirrors the
			 * reference driver, which only ever reads the key
			 * register in response to the interrupt. */
			if (stc8h_key_valid(key)) {
				stc8h_report(dev, key);
			}
		} else if (cfg->irq.port == NULL) {
			/* No IRQ line: transition polling is all we have. */
			if (stc8h_key_valid(key) && key != data->last_key) {
				stc8h_report(dev, key);
			}
		} else if (stc8h_key_valid(key) && key != data->last_key &&
			   gpio_pin_get_dt(&cfg->irq) > 0) {
			/* Poll with an IRQ line = missed-edge safety net.
			 * Only report while the key is still held (INT level
			 * active) — the latched register would otherwise
			 * re-report the stale last key forever. */
			stc8h_report(dev, key);
		}
		data->last_key = key;
	}

	k_work_reschedule(&data->work, K_MSEC(cfg->poll_interval_ms));
}

static void stc8h_irq_handler(const struct device *port, struct gpio_callback *cb,
			      uint32_t pins)
{
	struct stc8h_data *data = CONTAINER_OF(cb, struct stc8h_data, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* I2C cannot be touched from an ISR — hand off to the work queue.
	 * The flag tells the handler this run is a real key edge, so it must
	 * report even when the latched register still holds the same code. */
	atomic_set(&data->irq_pending, 1);
	k_work_reschedule(&data->work, K_NO_WAIT);
}

static int stc8h_init(const struct device *dev)
{
	const struct stc8h_config *cfg = dev->config;
	struct stc8h_data *data = dev->data;
	uint8_t probe;

	data->dev = dev;
	data->last_key = STC8H_KEY_NONE;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* A keypad that does not answer is not fatal — the node still boots,
	 * it just has no input. Say so plainly rather than failing init, which
	 * on this board would look like a dead device for a missing keypad. */
	if (stc8h_read_reg(cfg, STC8H_REG_KEY, &probe) < 0) {
		LOG_WRN("keypad not responding at 0x%02X — no input available",
			cfg->i2c.addr);
		return 0;
	}

	k_work_init_delayable(&data->work, stc8h_work_handler);

	if (cfg->irq.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->irq)) {
			LOG_ERR("IRQ GPIO not ready");
			return -ENODEV;
		}
		gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
		gpio_init_callback(&data->irq_cb, stc8h_irq_handler,
				   BIT(cfg->irq.pin));
		gpio_add_callback(cfg->irq.port, &data->irq_cb);
		gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
	}

	/* Poll regardless: it is the only mechanism without an IRQ line, and
	 * the safety net for a missed edge with one. */
	k_work_reschedule(&data->work, K_MSEC(cfg->poll_interval_ms));

	LOG_INF("keypad at 0x%02X ready (%s)", cfg->i2c.addr,
		cfg->irq.port ? "interrupt + poll" : "poll only");
	return 0;
}

#define STC8H_INIT(n)                                                          \
	static const struct stc8h_config stc8h_cfg_##n = {                     \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                \
		.irq = GPIO_DT_SPEC_INST_GET_OR(n, irq_gpios, { 0 }),          \
		.poll_interval_ms = DT_INST_PROP(n, poll_interval_ms),         \
	};                                                                     \
	static struct stc8h_data stc8h_data_##n;                               \
	DEVICE_DT_INST_DEFINE(n, stc8h_init, NULL, &stc8h_data_##n,            \
			      &stc8h_cfg_##n, POST_KERNEL,                     \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(STC8H_INIT)
