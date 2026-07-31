/*
 * ZephCore - PWM Buzzer with RTTTL Melody Playback
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Non-blocking RTTTL parser using Zephyr PWM API.
 * Each note is scheduled via k_work_delayable on a dedicated work queue
 * so flash/BLE/filesystem operations can't delay note timing.
 *
 * RTTTL Format: "Name:d=D,o=O,b=B:note,note,..."
 *   D = default duration (1,2,4,8,16,32)
 *   O = default octave (4-7)
 *   B = tempo in BPM
 *   note = [duration]<pitch>[#][.][octave]
 *   pitch = c,d,e,f,g,a,b,p (p = pause/rest)
 */

#include "buzzer.h"

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(buzzer, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* Dedicated work queue — high priority prevents flash/BLE/FS delays */
#define BUZZER_WQ_STACK_SIZE 512
#define BUZZER_WQ_PRIORITY   2  /* above default wq (~10+) */

K_THREAD_STACK_DEFINE(buzzer_wq_stack, BUZZER_WQ_STACK_SIZE);
static struct k_work_q buzzer_wq;

/* Safety watchdog: auto-silence if note handler stalls */
#define BUZZER_TONE_MAX_MS  2000

/* ========== Note Frequency Table ========== */
/* Frequencies for octave 4 (middle C = C4 = 262 Hz) */
/* Index: C=0, C#=1, D=2, D#=3, E=4, F=5, F#=6, G=7, G#=8, A=9, A#=10, B=11 */
static const uint16_t note_freq_o4[] = {
	262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494
};

/* ========== Buzzer Context ========== */
struct buzzer_ctx {
	struct pwm_dt_spec pwm;
	const struct device *enable_reg; /* Optional regulator for buzzer amp */
	struct k_work_delayable note_work;
	struct k_work_delayable safety_work; /* Auto-silence watchdog */

	/* RTTTL parser state */
	const char *melody;       /* Current position in RTTTL string */
	uint16_t default_dur;     /* Default note duration (1,2,4,8,16,32) */
	uint8_t  default_oct;     /* Default octave (4-7) */
	uint16_t bpm;             /* Beats per minute */
	uint32_t whole_note_ms;   /* Duration of a whole note in ms */

	bool quiet;
	bool playing;
	bool initialized;
};

static struct buzzer_ctx ctx;

/* ========== RTTTL Parser Helpers ========== */

static int parse_number(const char **p)
{
	int num = 0;

	while (**p >= '0' && **p <= '9') {
		num = num * 10 + (**p - '0');
		(*p)++;
	}
	return num;
}

static void skip_whitespace(const char **p)
{
	while (**p == ' ' || **p == '\t') {
		(*p)++;
	}
}

/**
 * Parse the RTTTL header: "Name:d=D,o=O,b=B:"
 * Sets defaults and advances pointer past the second colon.
 */
static bool parse_header(const char *rtttl)
{
	const char *p = rtttl;

	/* Skip name (everything before first ':') */
	while (*p && *p != ':') {
		p++;
	}
	if (!*p) {
		return false;
	}
	p++; /* skip ':' */

	/* Parse default values section */
	ctx.default_dur = 4;
	ctx.default_oct = 6;
	ctx.bpm = 63;

	while (*p && *p != ':') {
		skip_whitespace(&p);
		char key = tolower((unsigned char)*p);

		p++;
		if (*p == '=') {
			p++;
		}

		int val = parse_number(&p);

		switch (key) {
		case 'd':
			ctx.default_dur = val;
			break;
		case 'o':
			ctx.default_oct = val;
			break;
		case 'b':
			ctx.bpm = val;
			break;
		}

		if (*p == ',') {
			p++;
		}
	}

	if (!*p) {
		return false;
	}
	p++; /* skip second ':' */

	ctx.melody = p;
	/* Whole note = 4 beats. At B bpm, one beat = 60000/B ms */
	ctx.whole_note_ms = (60000UL * 4) / ctx.bpm;

	return true;
}

/**
 * Parse the next note from the melody string.
 * Returns frequency in Hz (0 for rest) and duration in ms.
 * Returns false if no more notes.
 */
static bool parse_next_note(uint16_t *freq_hz, uint32_t *dur_ms)
{
	const char *p = ctx.melody;

	skip_whitespace(&p);

	if (!*p) {
		return false;
	}

	/* Parse optional duration */
	uint16_t dur = parse_number(&p);

	if (dur == 0) {
		dur = ctx.default_dur;
	}

	/* Parse note letter */
	skip_whitespace(&p);
	char note = tolower((unsigned char)*p);

	if (!note) {
		return false;
	}
	p++;

	/* Map note letter to semitone index */
	int semitone = -1;
	bool is_rest = false;

	switch (note) {
	case 'c': semitone = 0;  break;
	case 'd': semitone = 2;  break;
	case 'e': semitone = 4;  break;
	case 'f': semitone = 5;  break;
	case 'g': semitone = 7;  break;
	case 'a': semitone = 9;  break;
	case 'b': semitone = 11; break;
	case 'p': is_rest = true; break;
	default:
		/* Unknown note, skip */
		while (*p && *p != ',' && *p != ':') {
			p++;
		}
		if (*p == ',') {
			p++;
		}
		ctx.melody = p;
		*freq_hz = 0;
		*dur_ms = ctx.whole_note_ms / dur;
		return true;
	}

	/* Check for sharp (#) */
	if (*p == '#') {
		semitone++;
		p++;
	}

	/* Check for dotted note */
	bool dotted = false;

	if (*p == '.') {
		dotted = true;
		p++;
	}

	/* Parse optional octave */
	uint8_t oct = 0;
	if (*p >= '0' && *p <= '9') {
		oct = parse_number(&p);
	} else {
		oct = ctx.default_oct;
	}

	/* Check for dotted note after octave too */
	if (*p == '.') {
		dotted = true;
		p++;
	}

	/* Calculate duration */
	uint32_t note_dur = ctx.whole_note_ms / dur;

	if (dotted) {
		note_dur += note_dur / 2;
	}
	*dur_ms = note_dur;

	/* Calculate frequency */
	if (is_rest) {
		*freq_hz = 0;
	} else {
		/* Base freq from octave 4, shift by octave difference */
		uint32_t f = note_freq_o4[semitone % 12];
		int oct_diff = (int)oct - 4;

		if (oct_diff > 0) {
			f <<= oct_diff;
		} else if (oct_diff < 0) {
			f >>= (-oct_diff);
		}
		*freq_hz = (uint16_t)f;
	}

	/* Skip comma separator */
	if (*p == ',') {
		p++;
	}
	ctx.melody = p;

	return true;
}

/* ========== Amplifier Enable/Disable ========== */

static void buzzer_amp_on(void)
{
	if (ctx.enable_reg) {
		regulator_enable(ctx.enable_reg);
	}
}

static void buzzer_amp_off(void)
{
	if (ctx.enable_reg) {
		regulator_disable(ctx.enable_reg);
	}
}

/* ========== PWM Control ========== */

static void buzzer_set_tone(uint16_t freq_hz)
{
	if (!ctx.initialized) {
		return;
	}

	if (freq_hz == 0) {
		/* Silence - set duty cycle to 0 */
		pwm_set_dt(&ctx.pwm, PWM_HZ(1000), 0);
	} else {
		/* Set PWM to desired frequency with 50% duty cycle */
		uint32_t period_ns = 1000000000UL / freq_hz;
		pwm_set_dt(&ctx.pwm, period_ns, period_ns / 2);
	}
}

static void buzzer_silence(void)
{
	buzzer_set_tone(0);
}

/* ========== Safety Watchdog ========== */

/**
 * Auto-silence handler: kills PWM if a note has been playing too long.
 * This is a safety net for crashes or workqueue stalls — the PWM hardware
 * is autonomous and keeps driving the pin until explicitly stopped.
 */
static void safety_work_handler(struct k_work *work)
{
	if (ctx.playing) {
		LOG_WRN("safety timeout — silencing stuck tone");
		ctx.playing = false;
		buzzer_silence();
		buzzer_amp_off();
	}
}

/* ========== Note Work Handler ========== */

static void note_work_handler(struct k_work *work)
{
	uint16_t freq;
	uint32_t dur_ms;

	if (!ctx.playing) {
		buzzer_silence();
		buzzer_amp_off();
		k_work_cancel_delayable(&ctx.safety_work);
		return;
	}

	if (!parse_next_note(&freq, &dur_ms)) {
		/* Melody complete */
		buzzer_silence();
		buzzer_amp_off();
		ctx.playing = false;
		k_work_cancel_delayable(&ctx.safety_work);
		return;
	}

	/* Play this note */
	buzzer_set_tone(freq);

	/* Reset safety watchdog — if the next note_work doesn't fire
	 * within BUZZER_TONE_MAX_MS, the safety handler kills the PWM. */
	k_work_reschedule_for_queue(&buzzer_wq, &ctx.safety_work,
				    K_MSEC(BUZZER_TONE_MAX_MS));

	/* Schedule next note after this note's duration */
	k_work_reschedule_for_queue(&buzzer_wq, &ctx.note_work,
				    K_MSEC(dur_ms));
}

/* ========== Public API ========== */

int buzzer_init(void)
{
	/* Check for buzzer alias in devicetree */
	const struct device *pwm_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pwm0));

	if (!pwm_dev || !device_is_ready(pwm_dev)) {
		LOG_INF("no PWM device found - buzzer disabled");
		return -ENODEV;
	}

	/* Get PWM spec from the buzzer node via alias */
	if (!DT_HAS_ALIAS(buzzer)) {
		LOG_INF("no buzzer alias in DT - buzzer disabled");
		return -ENODEV;
	}

	/* PWM spec from the pwm-leds buzzer node */
	ctx.pwm = (struct pwm_dt_spec)PWM_DT_SPEC_GET(DT_ALIAS(buzzer));

	if (!pwm_is_ready_dt(&ctx.pwm)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}

	/* Check for optional buzzer enable regulator (power gate for amplifier) */
	ctx.enable_reg = NULL;
	if (DT_HAS_ALIAS(buzzer_enable)) {
		const struct device *en_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(buzzer_enable));
		if (en_dev && device_is_ready(en_dev)) {
			ctx.enable_reg = en_dev;
			LOG_INF("buzzer enable regulator detected");
		}
	}

	/* Start dedicated buzzer work queue */
	k_work_queue_init(&buzzer_wq);
	k_work_queue_start(&buzzer_wq, buzzer_wq_stack,
			   K_THREAD_STACK_SIZEOF(buzzer_wq_stack),
			   BUZZER_WQ_PRIORITY, NULL);
	k_thread_name_set(&buzzer_wq.thread, "buzzer_wq");

	k_work_init_delayable(&ctx.note_work, note_work_handler);
	k_work_init_delayable(&ctx.safety_work, safety_work_handler);

	ctx.quiet = true;  /* Start quiet like Arduino */
	ctx.playing = false;
	ctx.initialized = true;

	/* Ensure buzzer is silent and amp off on init */
	buzzer_silence();
	buzzer_amp_off();

	LOG_INF("buzzer initialized (PWM, dedicated wq)");
	return 0;
}

void buzzer_play(const char *rtttl)
{
	if (!ctx.initialized) {
		return;
	}

	/* Stop any current melody */
	if (ctx.playing) {
		buzzer_stop();
	}

	if (ctx.quiet) {
		return;
	}

	if (!rtttl || !*rtttl) {
		return;
	}

	/* Parse RTTTL header */
	if (!parse_header(rtttl)) {
		LOG_WRN("invalid RTTTL format");
		return;
	}

	ctx.playing = true;

	/* Enable buzzer amplifier power */
	buzzer_amp_on();

	/* Start playing first note immediately on dedicated wq */
	k_work_reschedule_for_queue(&buzzer_wq, &ctx.note_work, K_NO_WAIT);
}

void buzzer_stop(void)
{
	if (!ctx.initialized) {
		return;
	}

	ctx.playing = false;
	k_work_cancel_delayable(&ctx.note_work);
	k_work_cancel_delayable(&ctx.safety_work);
	buzzer_silence();
	buzzer_amp_off();
}

void buzzer_set_quiet(bool quiet)
{
	ctx.quiet = quiet;

	if (quiet) {
		if (ctx.playing) {
			buzzer_stop();
		} else {
			/* Ensure amp is off even if not playing */
			buzzer_silence();
			buzzer_amp_off();
		}
	}

	LOG_INF("buzzer %s", quiet ? "muted" : "enabled");
}

void buzzer_set_quiet_deferred(bool quiet)
{
	/* Set the flag but don't stop the current melody.
	 * The melody will play out via note_work_handler (which doesn't
	 * check quiet). Future buzzer_play() calls will check ctx.quiet
	 * and become no-ops. */
	ctx.quiet = quiet;
	LOG_INF("buzzer %s (deferred)", quiet ? "muted" : "enabled");
}

bool buzzer_is_quiet(void)
{
	return ctx.quiet;
}

bool buzzer_is_playing(void)
{
	return ctx.playing;
}
