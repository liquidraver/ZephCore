/*
 * Seeed Tracker T1000-E onboard analog sensors — NTC thermistor + photocell
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Ported from Arduino MeshCore variants/t1000-e/t1000e_sensors.cpp, which in
 * turn carries Seeed's own conversions. Both are reproduced here so a node
 * reports the same numbers it did on the stock firmware.
 */

#define DT_DRV_COMPAT seeed_t1000e_analog

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

LOG_MODULE_REGISTER(t1000e_analog, CONFIG_SENSOR_LOG_LEVEL);

/* Averaged per channel. The parts are slow and the rail is already paying a
 * 10 ms settle, so a few extra conversions are free noise rejection. */
#define T1000E_ADC_SAMPLES 4

/* NTC thermistor: resistance in ohms at each degree from -30 C (index 0) to
 * +105 C (index 135), a 10k-at-25C part. Do not replace the table by a beta
 * formula. */

#define NTC_TABLE_LEN   136
#define NTC_TABLE_T_MIN (-30)

static const uint32_t ntc_res[NTC_TABLE_LEN] = {
	113347, 107565, 102116, 96978, 92132, 87559, 83242, 79166, 75316, 71677,
	68237,  64991,  61919,  59011, 56258, 53650, 51178, 48835, 46613, 44506,
	42506,  40600,  38791,  37073, 35442, 33892, 32420, 31020, 29689, 28423,
	27219,  26076,  24988,  23951, 22963, 22021, 21123, 20267, 19450, 18670,
	17926,  17214,  16534,  15886, 15266, 14674, 14108, 13566, 13049, 12554,
	12081,  11628,  11195,  10780, 10382, 10000, 9634,  9284,  8947,  8624,
	8315,   8018,   7734,   7461,  7199,  6948,  6707,  6475,  6253,  6039,
	5834,   5636,   5445,   5262,  5086,  4917,  4754,  4597,  4446,  4301,
	4161,   4026,   3896,   3771,  3651,  3535,  3423,  3315,  3211,  3111,
	3014,   2922,   2834,   2748,  2666,  2586,  2509,  2435,  2364,  2294,
	2228,   2163,   2100,   2040,  1981,  1925,  1870,  1817,  1766,  1716,
	1669,   1622,   1578,   1535,  1493,  1452,  1413,  1375,  1338,  1303,
	1268,   1234,   1202,   1170,  1139,  1110,  1081,  1053,  1026,  999,
	974,    949,    925,    902,   880,   858,
};

/* Photocell: the divider voltage mapped onto 0-100 with a dead band at each
 * end, as Seeed's firmware. LIGHT_SPAN_MV is its own constant on purpose. */

#define LIGHT_MIN_MV  80
#define LIGHT_MAX_MV  2480
#define LIGHT_SPAN_MV 2400

struct t1000e_config {
	struct adc_dt_spec ntc;
	struct adc_dt_spec light;
	struct adc_dt_spec vcc;
	struct gpio_dt_spec power_gpio;
	const struct device *power_supply;
	uint32_t vcc_mv_multiplier;
	uint32_t vcc_max_mv;
	uint32_t ntc_series_ohms;
	uint16_t settle_time_ms;
};

struct t1000e_data {
	float temperature_c;
	float light_pct;
	bool temperature_valid;
	bool light_valid;
};

/* Averaged raw reading for one channel, or a negative errno. */
static int t1000e_read_raw(const struct adc_dt_spec *spec)
{
	int32_t total = 0;
	int valid = 0;

	for (int i = 0; i < T1000E_ADC_SAMPLES; i++) {
		int16_t sample = 0;
		struct adc_sequence seq = {
			.buffer = &sample,
			.buffer_size = sizeof(sample),
		};

		if (adc_sequence_init_dt(spec, &seq) < 0) {
			continue;
		}
		if (adc_read_dt(spec, &seq) == 0) {
			total += sample;
			valid++;
		}
	}

	if (valid == 0) {
		return -EIO;
	}

	total /= valid;

	/* A divider cannot swing below the rail's ground; a negative code is
	 * SAADC offset noise around zero, and would invert the conversions. */
	return (total < 0) ? 0 : (int)total;
}

/* Millivolts at the pin, or a negative errno. */
static int t1000e_read_mv(const struct adc_dt_spec *spec)
{
	int32_t mv = t1000e_read_raw(spec);

	if (mv < 0) {
		return mv;
	}
	if (adc_raw_to_millivolts_dt(spec, &mv) < 0) {
		return -EINVAL;
	}
	return (int)mv;
}

static float t1000e_ntc_temperature(const struct t1000e_config *cfg,
				    uint32_t vcc_mv, uint32_t ntc_mv)
{
	float rp = (float)cfg->ntc_series_ohms;
	float rt;
	int i;

	/* Divider is rail - NTC - node - Rp - ground, so
	 * Vnode = Vcc * Rp / (Rp + Rntc)  =>  Rntc = Rp * (Vcc - Vnode) / Vnode.
	 * A zero reading means an open NTC or an unpowered rail: infinitely
	 * cold on this curve, which the table floor turns into its low clamp. */
	if (ntc_mv == 0) {
		return (float)NTC_TABLE_T_MIN;
	}

	rt = rp * ((float)vcc_mv / (float)ntc_mv - 1.0f);

	/* Table is descending, so the first entry the resistance reaches or
	 * exceeds bounds it from above. */
	for (i = 0; i < NTC_TABLE_LEN; i++) {
		if (rt >= (float)ntc_res[i]) {
			break;
		}
	}

	/* Off either end of the curve. Seeed's loop indexes out of bounds in
	 * both of these cases; clamp instead. */
	if (i == 0) {
		return (float)NTC_TABLE_T_MIN;
	}
	if (i == NTC_TABLE_LEN) {
		return (float)(NTC_TABLE_T_MIN + NTC_TABLE_LEN - 1);
	}

	/* Linear interpolation between the bracketing entries, which are
	 * exactly one degree apart. The 0.05 is Seeed's rounding compensation:
	 * every consumer downstream truncates to a tenth of a degree. */
	return (float)(NTC_TABLE_T_MIN + i - 1) +
	       ((float)ntc_res[i - 1] - rt) /
		       (float)(ntc_res[i - 1] - ntc_res[i]) +
	       0.05f;
}

static float t1000e_light_percent(uint32_t light_mv)
{
	if (light_mv <= LIGHT_MIN_MV) {
		return 0.0f;
	}
	if (light_mv >= LIGHT_MAX_MV) {
		return 100.0f;
	}
	return 100.0f * (float)(light_mv - LIGHT_MIN_MV) / (float)LIGHT_SPAN_MV;
}

static int t1000e_power(const struct t1000e_config *cfg, bool on)
{
	int rc = 0;

	if (on) {
		/* The rail is shared with the battery divider (one devicetree node,
		 * two labels: sensor_power / vbat_enable). The regulator core
		 * refcounts, so if another holder already has it up there is no
		 * off->on transition to wait out — settling only costs time when we
		 * are the one turning it on. */
		bool already_on = (cfg->power_supply != NULL) &&
				  regulator_is_enabled(cfg->power_supply);

		if (cfg->power_supply != NULL) {
			rc = regulator_enable(cfg->power_supply);
			if (rc < 0) {
				return rc;
			}
		}
		if (cfg->power_gpio.port != NULL) {
			rc = gpio_pin_set_dt(&cfg->power_gpio, 1);
			if (rc < 0) {
				return rc;
			}
		}
		if (!already_on) {
			k_msleep(cfg->settle_time_ms);
		}
		return 0;
	}

	if (cfg->power_gpio.port != NULL) {
		(void)gpio_pin_set_dt(&cfg->power_gpio, 0);
	}
	if (cfg->power_supply != NULL) {
		(void)regulator_disable(cfg->power_supply);
	}
	return 0;
}

static int t1000e_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	const struct t1000e_config *cfg = dev->config;
	struct t1000e_data *data = dev->data;
	int ntc_mv, light_mv, vcc_raw;
	uint32_t vcc_mv;
	int rc;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP &&
	    chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	rc = t1000e_power(cfg, true);
	if (rc < 0) {
		LOG_ERR("sensor rail power-up failed: %d", rc);
		(void)t1000e_power(cfg, false);
		return rc;
	}

	ntc_mv = t1000e_read_mv(&cfg->ntc);
	light_mv = t1000e_read_mv(&cfg->light);
	vcc_raw = t1000e_read_raw(&cfg->vcc);

	(void)t1000e_power(cfg, false);

	/* The rail divider carries its own scaling, so it goes through the
	 * board's multiplier rather than the generic raw-to-millivolts helper. */
	if (vcc_raw < 0) {
		vcc_mv = cfg->vcc_max_mv;
	} else {
		vcc_mv = ((uint32_t)vcc_raw * cfg->vcc_mv_multiplier) / 4096u;
		if (vcc_mv > cfg->vcc_max_mv) {
			vcc_mv = cfg->vcc_max_mv;
		}
	}

	data->temperature_valid = (ntc_mv >= 0);
	if (data->temperature_valid) {
		data->temperature_c =
			t1000e_ntc_temperature(cfg, vcc_mv, (uint32_t)ntc_mv);
	}

	data->light_valid = (light_mv >= 0);
	if (data->light_valid) {
		data->light_pct = t1000e_light_percent((uint32_t)light_mv);
	}

	/* Millidegrees rather than a split integer/fraction pair: the naive
	 * split prints "-11.-75" below freezing, which is exactly where these
	 * readings most need checking. */
	LOG_DBG("ntc=%dmV light=%dmV vcc=%umV -> %d m°C, %d%%",
		ntc_mv, light_mv, vcc_mv,
		(int)(data->temperature_c * 1000.0f),
		(int)data->light_pct);

	if (!data->temperature_valid && !data->light_valid) {
		return -EIO;
	}
	return 0;
}

static int t1000e_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct t1000e_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		if (!data->temperature_valid) {
			return -ENODATA;
		}
		return sensor_value_from_float(val, data->temperature_c);

	case SENSOR_CHAN_LIGHT:
		if (!data->light_valid) {
			return -ENODATA;
		}
		return sensor_value_from_float(val, data->light_pct);

	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, t1000e_api) = {
	.sample_fetch = t1000e_sample_fetch,
	.channel_get = t1000e_channel_get,
};

static int t1000e_init(const struct device *dev)
{
	const struct t1000e_config *cfg = dev->config;
	const struct adc_dt_spec *chans[] = { &cfg->ntc, &cfg->light, &cfg->vcc };
	int rc;

	for (size_t i = 0; i < ARRAY_SIZE(chans); i++) {
		if (!adc_is_ready_dt(chans[i])) {
			LOG_ERR("ADC %s not ready", chans[i]->dev->name);
			return -ENODEV;
		}
		rc = adc_channel_setup_dt(chans[i]);
		if (rc < 0) {
			LOG_ERR("ADC channel %u setup failed: %d",
				chans[i]->channel_id, rc);
			return rc;
		}
	}

	if (cfg->power_supply != NULL && !device_is_ready(cfg->power_supply)) {
		LOG_ERR("sensor rail regulator not ready");
		return -ENODEV;
	}

	if (cfg->power_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->power_gpio)) {
			LOG_ERR("sensor enable GPIO not ready");
			return -ENODEV;
		}
		rc = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
		if (rc < 0) {
			LOG_ERR("sensor enable GPIO config failed: %d", rc);
			return rc;
		}
	}

	LOG_INF("T1000-E analog sensors ready (NTC + photocell)");
	return 0;
}

#define T1000E_POWER_SUPPLY(inst)                                             \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, power_supply),                \
		    (DEVICE_DT_GET(DT_INST_PHANDLE(inst, power_supply))),     \
		    (NULL))

#define T1000E_DEFINE(inst)                                                   \
	static struct t1000e_data t1000e_data_##inst;                         \
	static const struct t1000e_config t1000e_config_##inst = {            \
		.ntc = ADC_DT_SPEC_INST_GET_BY_NAME(inst, ntc),               \
		.light = ADC_DT_SPEC_INST_GET_BY_NAME(inst, light),           \
		.vcc = ADC_DT_SPEC_INST_GET_BY_NAME(inst, vcc),               \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),\
		.power_supply = T1000E_POWER_SUPPLY(inst),                    \
		.vcc_mv_multiplier = DT_INST_PROP(inst, vcc_mv_multiplier),   \
		.vcc_max_mv = DT_INST_PROP(inst, vcc_max_mv),                 \
		.ntc_series_ohms = DT_INST_PROP(inst, ntc_series_ohms),       \
		.settle_time_ms = DT_INST_PROP(inst, settle_time_ms),         \
	};                                                                    \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, t1000e_init, NULL,                 \
				     &t1000e_data_##inst,                     \
				     &t1000e_config_##inst,                   \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,\
				     &t1000e_api);

DT_INST_FOREACH_STATUS_OKAY(T1000E_DEFINE)

#endif /* DT_HAS_COMPAT_STATUS_OKAY */
