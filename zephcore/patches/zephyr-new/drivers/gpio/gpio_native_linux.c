/*
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Zephyr GPIO controller that bridges to a Linux /dev/gpiochipX device
 * via the kernel's GPIO V2 character-device uAPI (direct ioctls, no
 * libgpiod). Runs under native_sim on a real Linux host (not in QEMU or
 * simulation).
 *
 * Each pin owns its own kernel line request, lazily (re)created when
 * pin_configure or pin_interrupt_configure changes its settings. A
 * dedicated k_thread polls() the fds of all edge-detection-enabled pins
 * and fires the registered Zephyr gpio_callbacks on each event.
 *
 * Host-side chardev ioctls live in gpio_native_linux_adapt.c (compiled
 * into the native_simulator INTERFACE) to keep host headers out of the
 * Zephyr translation unit.
 */

#define DT_DRV_COMPAT zephcore_gpio_native_linux

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>

#include <cmdline.h>
#include <posix_native_task.h>

#include "gpio_native_linux_adapt.h"

LOG_MODULE_REGISTER(gpio_native_linux, CONFIG_GPIO_LOG_LEVEL);

/* Max pins this driver tracks per chip. SX126x needs ~5; allow plenty. */
#define GNL_MAX_PINS 64

struct gnl_pin_state {
	gnl_line_t line;
	gpio_flags_t cfg_flags;
	enum gpio_int_mode int_mode;
	enum gpio_int_trig int_trig;
	bool requested;
};

struct gpio_native_linux_config {
	struct gpio_driver_config common;
	const char *chip_path;
	uint32_t ngpios;
};

struct gpio_native_linux_data {
	struct gpio_driver_data common;
	gnl_chip_t chip;
	struct gnl_pin_state pins[GNL_MAX_PINS];
	sys_slist_t callbacks;
	struct k_mutex mu;

	struct k_thread evt_thread;
	/* K_THREAD_STACK_MEMBER expands to a section attribute that is only
	 * valid at file scope — illegal inside a struct on native_sim.
	 * K_KERNEL_STACK_MEMBER is struct-safe. Use a literal 2048 rather
	 * than CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE, which evaluates to
	 * 24 bytes on a cross-compiled native_sim ARM build. */
	K_KERNEL_STACK_MEMBER(evt_thread_stack, 2048);
	bool evt_thread_started;
	struct k_sem reconfig_sem;
	const struct device *self;
};

/* Runtime cmdline override of the gpiochip path (applies to instance 0). */
static char *chip_path_cmd_opt;

static int translate_edge(enum gpio_int_trig trig)
{
	switch (trig) {
	case GPIO_INT_TRIG_LOW:  return GNL_EDGE_FALLING;
	case GPIO_INT_TRIG_HIGH: return GNL_EDGE_RISING;
	case GPIO_INT_TRIG_BOTH: return GNL_EDGE_BOTH;
	default:                 return GNL_EDGE_NONE;
	}
}

/* (Re)request a line based on cfg_flags + int settings. Releases prior. */
static int rebuild_line(const struct device *dev, gpio_pin_t pin)
{
	struct gpio_native_linux_data *data = dev->data;
	struct gnl_pin_state *p = &data->pins[pin];

	if (p->line != NULL) {
		gnl_line_release(p->line);
		p->line = NULL;
		p->requested = false;
	}

	gpio_flags_t f = p->cfg_flags;

	/* "Disconnected" means neither INPUT nor OUTPUT is requested.  Note
	 * GPIO_DISCONNECTED is 0, so the naive test (f & GPIO_DISCONNECTED)
	 * is always true and would skip requesting EVERY line -- mask the
	 * direction bits explicitly instead. */
	if ((f & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return 0;
	}

	bool active_low = (f & GPIO_ACTIVE_LOW) != 0;
	bool pull_up = (f & GPIO_PULL_UP) != 0;
	bool pull_down = (f & GPIO_PULL_DOWN) != 0;

	if ((f & GPIO_OUTPUT) != 0) {
		int init_val = -1;

		if ((f & GPIO_OUTPUT_INIT_HIGH) != 0) {
			init_val = 1;
		} else if ((f & GPIO_OUTPUT_INIT_LOW) != 0) {
			init_val = 0;
		} else {
			init_val = 0;
		}
		p->line = gnl_request_output(data->chip, pin, init_val, active_low);
	} else if ((f & GPIO_INPUT) != 0) {
		if (p->int_mode == GPIO_INT_MODE_DISABLED) {
			p->line = gnl_request_input(data->chip, pin,
						     pull_up, pull_down, active_low);
		} else {
			int edge = translate_edge(p->int_trig);

			if (edge == GNL_EDGE_NONE) {
				p->line = gnl_request_input(data->chip, pin,
							     pull_up, pull_down,
							     active_low);
			} else {
				p->line = gnl_request_input_edge(data->chip, pin,
								  edge,
								  pull_up,
								  pull_down,
								  active_low);
			}
		}
	}

	if (p->line == NULL) {
		LOG_ERR("Failed to request line %u on host chip", pin);
		return -EIO;
	}

	p->requested = true;
	return 0;
}

static int gnl_pin_configure(const struct device *dev, gpio_pin_t pin,
			      gpio_flags_t flags)
{
	const struct gpio_native_linux_config *cfg = dev->config;
	struct gpio_native_linux_data *data = dev->data;
	int ret;

	if (pin >= cfg->ngpios || pin >= GNL_MAX_PINS) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mu, K_FOREVER);
	data->pins[pin].cfg_flags = flags;
	ret = rebuild_line(dev, pin);
	k_mutex_unlock(&data->mu);

	/* Wake the event thread to re-collect fds. */
	k_sem_give(&data->reconfig_sem);

	return ret;
}

static int gnl_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_native_linux_config *cfg = dev->config;
	struct gpio_native_linux_data *data = dev->data;
	gpio_port_value_t v = 0;

	k_mutex_lock(&data->mu, K_FOREVER);
	for (uint32_t i = 0; i < cfg->ngpios && i < GNL_MAX_PINS; i++) {
		struct gnl_pin_state *p = &data->pins[i];

		if (!p->requested || p->line == NULL) {
			continue;
		}
		int x = gnl_line_get_value(p->line);

		if (x > 0) {
			v |= ((gpio_port_value_t)1U << i);
		}
	}
	k_mutex_unlock(&data->mu);

	*value = v;
	return 0;
}

static int gnl_port_set_masked_raw(const struct device *dev,
				    gpio_port_pins_t mask,
				    gpio_port_value_t value)
{
	const struct gpio_native_linux_config *cfg = dev->config;
	struct gpio_native_linux_data *data = dev->data;

	k_mutex_lock(&data->mu, K_FOREVER);
	for (uint32_t i = 0; i < cfg->ngpios && i < GNL_MAX_PINS; i++) {
		if ((mask & ((gpio_port_pins_t)1U << i)) == 0) {
			continue;
		}
		struct gnl_pin_state *p = &data->pins[i];

		if (!p->requested || p->line == NULL) {
			continue;
		}
		int bit = (value >> i) & 1U;

		(void)gnl_line_set_value(p->line, bit);
	}
	k_mutex_unlock(&data->mu);
	return 0;
}

static int gnl_port_set_bits_raw(const struct device *dev,
				  gpio_port_pins_t pins)
{
	return gnl_port_set_masked_raw(dev, pins, pins);
}

static int gnl_port_clear_bits_raw(const struct device *dev,
				    gpio_port_pins_t pins)
{
	return gnl_port_set_masked_raw(dev, pins, 0);
}

static int gnl_port_toggle_bits(const struct device *dev,
				 gpio_port_pins_t pins)
{
	gpio_port_value_t cur;
	int ret = gnl_port_get_raw(dev, &cur);

	if (ret != 0) {
		return ret;
	}
	return gnl_port_set_masked_raw(dev, pins, ~cur);
}

static int gnl_pin_interrupt_configure(const struct device *dev,
					gpio_pin_t pin,
					enum gpio_int_mode mode,
					enum gpio_int_trig trig)
{
	const struct gpio_native_linux_config *cfg = dev->config;
	struct gpio_native_linux_data *data = dev->data;
	int ret;

	if (pin >= cfg->ngpios || pin >= GNL_MAX_PINS) {
		return -EINVAL;
	}

	/* libgpiod v2 only supports edge detection on input lines. */
	if (mode == GPIO_INT_MODE_LEVEL && trig != GPIO_INT_TRIG_LOW &&
	    trig != GPIO_INT_TRIG_HIGH) {
		return -ENOTSUP;
	}
	if (mode == GPIO_INT_MODE_LEVEL) {
		/* Level interrupts not supported by libgpiod v2 edge events. */
		LOG_WRN("Level interrupts unsupported on pin %u; ignoring", pin);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mu, K_FOREVER);
	data->pins[pin].int_mode = mode;
	data->pins[pin].int_trig = trig;
	ret = rebuild_line(dev, pin);
	k_mutex_unlock(&data->mu);

	k_sem_give(&data->reconfig_sem);
	return ret;
}

static int gnl_manage_callback(const struct device *dev,
				struct gpio_callback *cb, bool set)
{
	struct gpio_native_linux_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, cb, set);
}

static DEVICE_API(gpio, gpio_native_linux_api) = {
	.pin_configure = gnl_pin_configure,
	.port_get_raw = gnl_port_get_raw,
	.port_set_masked_raw = gnl_port_set_masked_raw,
	.port_set_bits_raw = gnl_port_set_bits_raw,
	.port_clear_bits_raw = gnl_port_clear_bits_raw,
	.port_toggle_bits = gnl_port_toggle_bits,
	.pin_interrupt_configure = gnl_pin_interrupt_configure,
	.manage_callback = gnl_manage_callback,
};

/*
 * Event polling thread.
 *
 * Collects fds from all currently-edge-enabled pin requests, calls poll(),
 * and on each event drains it and fires the registered gpio_callbacks
 * (using the Zephyr gpio_fire_callbacks helper).
 *
 * Wakes on reconfig_sem whenever a pin's settings change so the fd set
 * stays in sync.
 */
static void gnl_evt_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct gpio_native_linux_data *data = dev->data;
	const struct gpio_native_linux_config *cfg = dev->config;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int fds[GNL_MAX_PINS];
	gpio_pin_t pin_for_fd[GNL_MAX_PINS];

	while (true) {
		size_t nfds = 0;

		k_mutex_lock(&data->mu, K_FOREVER);
		for (uint32_t i = 0; i < cfg->ngpios && i < GNL_MAX_PINS; i++) {
			struct gnl_pin_state *p = &data->pins[i];

			if (!p->requested || p->line == NULL) {
				continue;
			}
			if (p->int_mode != GPIO_INT_MODE_EDGE) {
				continue;
			}
			int fd = gnl_line_get_fd(p->line);

			if (fd < 0) {
				continue;
			}
			fds[nfds] = fd;
			pin_for_fd[nfds] = (gpio_pin_t)i;
			nfds++;
			if (nfds >= 32) {
				break;
			}
		}
		k_mutex_unlock(&data->mu);

		if (nfds == 0) {
			/* No edge-enabled pins: sleep properly so the Zephyr CPU
			 * is released (unlike blocking poll which holds it). */
			k_sleep(K_MSEC(10));
			while (k_sem_take(&data->reconfig_sem, K_NO_WAIT) == 0) {
			}
			continue;
		}

		/* Non-blocking poll: check for events instantly without holding
		 * the Zephyr CPU mutex (a blocking poll blocks ALL Zephyr threads
		 * for its duration, starving the SX126x work queue and causing
		 * CAD/TX-DONE timeouts).
		 *
		 * Then k_sleep(1ms): properly releases the Zephyr CPU to any
		 * thread (unlike k_yield which only yields to same-priority).
		 * During the 1ms, the SX126x work queue processes the IRQ and
		 * signals cad_sem/tx_done. GPIO event latency ≤ 1ms — well
		 * within the 200ms CAD and TX_DONE timeout budgets.
		 *
		 * TODO: replace with NSI interrupt model for true zero latency. */
		uint32_t ready_mask = gnl_poll_fds(fds, nfds, 0);

		/* Drain reconfig requests. */
		while (k_sem_take(&data->reconfig_sem, K_NO_WAIT) == 0) {
		}

		gpio_port_pins_t fired = 0;

		k_mutex_lock(&data->mu, K_FOREVER);
		for (size_t i = 0; i < nfds; i++) {
			if ((ready_mask & (1U << i)) == 0) {
				continue;
			}
			gpio_pin_t pin = pin_for_fd[i];
			struct gnl_pin_state *p = &data->pins[pin];

			if (p->line == NULL) {
				continue;
			}
			(void)gnl_line_drain_events(p->line);
			fired |= ((gpio_port_pins_t)1U << pin);
		}
		k_mutex_unlock(&data->mu);

		if (fired != 0) {
			gpio_fire_callbacks(&data->callbacks, dev, fired);
		}

		/* Sleep 1ms every loop iteration — properly releases the Zephyr
		 * CPU to any ready thread (SX126x work queue, mesh event loop,
		 * timers). k_yield() is insufficient: it only yields to threads
		 * of the same cooperative priority. */
		k_sleep(K_MSEC(1));
	}
}

static int gpio_native_linux_init(const struct device *dev)
{
	const struct gpio_native_linux_config *cfg = dev->config;
	struct gpio_native_linux_data *data = dev->data;
	const char *path;

	path = (chip_path_cmd_opt != NULL) ? chip_path_cmd_opt : cfg->chip_path;

	LOG_INF("Opening GPIO chip: %s", path);
	data->chip = gnl_chip_open(path);
	if (data->chip == NULL) {
		LOG_ERR("Failed to open chip %s", path);
		return -ENODEV;
	}

	k_mutex_init(&data->mu);
	k_sem_init(&data->reconfig_sem, 0, K_SEM_MAX_LIMIT);
	sys_slist_init(&data->callbacks);
	data->self = dev;
	memset(data->pins, 0, sizeof(data->pins));

	k_thread_create(&data->evt_thread, data->evt_thread_stack,
			2048, /* literal: K_KERNEL_STACK_SIZEOF unreliable on cross-compiled native_sim */
			gnl_evt_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(7), 0, K_NO_WAIT);
	data->evt_thread_started = true;

	LOG_INF("GPIO native_linux ready (%s, %u pins)",
		path, cfg->ngpios);
	return 0;
}

#define GPIO_NATIVE_LINUX_INIT(inst)							\
	static const struct gpio_native_linux_config					\
		gpio_native_linux_cfg_##inst = {					\
		.common = {								\
			.port_pin_mask =						\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),			\
		},									\
		.chip_path = DT_INST_PROP(inst, gpio_chip),				\
		.ngpios = DT_INST_PROP(inst, ngpios),					\
	};										\
											\
	static struct gpio_native_linux_data gpio_native_linux_data_##inst;		\
											\
	/* POST_KERNEL: k_thread_create with K_NO_WAIT requires the scheduler
	 * run queue to be initialized, which only happens by POST_KERNEL.
	 * PRE_KERNEL_1 crashes on native_sim (POSIX arch) because the dlist
	 * backing _kernel.ready_q is still NULL at that stage.
	 * GPIO_INIT_PRIORITY (40) < SPI_INIT_PRIORITY (70), both POST_KERNEL,
	 * so CS GPIO is still available when the SPI driver inits. */	\
	DEVICE_DT_INST_DEFINE(inst, gpio_native_linux_init, NULL,			\
			      &gpio_native_linux_data_##inst,				\
			      &gpio_native_linux_cfg_##inst,				\
			      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,			\
			      &gpio_native_linux_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_NATIVE_LINUX_INIT)

/* Command-line arg: --lora-gpio-chip=<path> overrides DT gpio-chip. */
static void gpio_native_linux_add_cmdline_opts(void)
{
	static struct args_struct_t gpio_native_options[] = {
		{
			.option = "lora-gpio-chip",
			.name = "path",
			.type = 's',
			.dest = (void *)&chip_path_cmd_opt,
			.descript = "Linux gpiochip device path (overrides DT gpio-chip)",
		},
		ARG_TABLE_ENDMARKER,
	};

	native_add_command_line_opts(gpio_native_options);
}

NATIVE_TASK(gpio_native_linux_add_cmdline_opts, PRE_BOOT_1, 12);
