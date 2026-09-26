/*
 * ZephCore - Doom Raycaster Easter Egg
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Wolf3D-style raycasting engine merged into a single file.
 * Runs on k_work_delayable at 20 FPS on the system work queue.
 * Writes directly to display via display_write() (bypasses CFB).
 *
 * Two levels:
 *   L1 — kill 2 monsters (imp + demon); a portal spawns at a random open tile.
 *   L2 — open arena against a 300-HP boss that waits in a random corner.
 *
 * The L2 fight is not a damage race. The boss wears armour that soaks 7/8 of
 * every shot, and only drops it while its jaw is open — during the wind-up
 * before it spits a fireball, and on the frames its claws are out in melee.
 * Jaw open is drawn and computed from one predicate, so what you see is
 * exactly what you can hurt. Below half HP it enrages: twice the speed, a
 * shorter cast cycle. Shots and fireballs both need line of sight, which is
 * what makes the arena pillars and the centre block real cover.
 *
 * ~2.0 KB RAM, ~7 KB flash when enabled.
 */

#include "doom_game.h"
#include "display.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#ifdef CONFIG_ZEPHCORE_UI_BUZZER
#include "buzzer.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(doom_game, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ========== CONSTANTS ========== */

#define SCREEN_W      128
#define SCREEN_H      64
#define MAP_W         16
#define MAP_H         16
#define FB_SIZE       (SCREEN_W * (SCREEN_H / 8))

#define FP_SHIFT      16
#define FP_ONE        (1 << FP_SHIFT)
#define FP_HALF       (FP_ONE >> 1)

#define TARGET_FPS    20
#define FRAME_MS      (1000 / TARGET_FPS)
#define MOVE_SPEED    (FP_ONE / 8)   /* player */
#define ROT_SPEED     (FP_ONE / 12)
#define COLLISION_R   (FP_ONE / 4)
#define MAX_ENEMIES   8

#define ENEMY_SPEED   (FP_ONE / 16)  /* imp / demon — half the player's pace */
#define BOSS_SPEED      (FP_ONE / 32) /* boss phase 1 — a quarter of the player */
#define BOSS_SPEED_RAGE (FP_ONE / 16) /* boss phase 2 — doubles, still outrunnable */
#define BOSS_HP       300

#define SHOT_DAMAGE    20
/*
 * Armour divisor: shots land for SHOT_DAMAGE/8 = 2 while the jaw is shut.
 * Sized so that holding the trigger down cannot win. A held trigger lands
 * ~2.4 of its 14 shots per cast cycle inside the window by luck, for ~71 dmg
 * — 300 HP would take 4.2 cycles and 59 rounds, and the magazine holds ~45
 * by the time L2 starts. Playing the tell needs 15 rounds.
 */
#define BOSS_GUARD_DIV 8

/*
 * Boss cast cycle. Every fireball is telegraphed: the boss plants its feet and
 * opens its jaw for BOSS_WINDUP frames first, and that window is the only time
 * at range when it takes full damage.
 */
#define BOSS_WINDUP           12  /* 0.6 s of open jaw before the spit */
#define BOSS_CAST_PERIOD      70  /* 3.5 s between casts, phase 1 */
#define BOSS_CAST_PERIOD_RAGE 45  /* 2.25 s, phase 2 */
#define BOSS_CAST_MIN_DIST    (FP_ONE * 5 / 2)  /* nearer than this it swings */
/*
 * Frames the boss spends backing off after a melee swing. Without this it
 * closes once and camps in the player's face forever: the cast cycle is held
 * below BOSS_CAST_MIN_DIST, so the tell never fires again and the fight
 * collapses into a melee grind with nothing to read. Disengaging restores the
 * loop — close, swing, back out, cast, close again — and at phase-1 speed 50
 * frames buys back about 1.5 tiles, just past casting range.
 */
#define BOSS_RETREAT          50

#define MAX_FIREBALLS  3
#define FIREBALL_SPEED (FP_ONE / 6)  /* ~3.3 tiles/s — outruns a straight retreat */
#define FIREBALL_DMG   18
#define FIREBALL_LIFE  110           /* frames before it burns out */
#define FIREBALL_HIT_R FP_HALF

/* Input bitflags */
#define DINPUT_FWD    BIT(0)
#define DINPUT_BACK   BIT(1)
#define DINPUT_LEFT   BIT(2)
#define DINPUT_RIGHT  BIT(3)
#define DINPUT_FIRE   BIT(4)

/* ========== FIXED-POINT MATH ========== */

typedef int32_t fixed_t;

static inline fixed_t fp_mul(fixed_t a, fixed_t b)
{
	return (fixed_t)(((int64_t)a * b) >> FP_SHIFT);
}

static inline fixed_t fp_div(fixed_t a, fixed_t b)
{
	if (b == 0) return a > 0 ? INT32_MAX : INT32_MIN;
	return (fixed_t)(((int64_t)a << FP_SHIFT) / b);
}

static inline fixed_t fp_from_int(int x)
{
	return x << FP_SHIFT;
}

static inline int fp_to_int(fixed_t x)
{
	return x >> FP_SHIFT;
}

static inline fixed_t fp_abs(fixed_t x)
{
	return x < 0 ? -x : x;
}

/*
 * |(dx,dy)| by alpha-max-plus-beta-min (alpha 1, beta 3/8): ~7% worst-case
 * error, no sqrt and no division. Chase and aim vectors normalise with this.
 * The code this replaced divided by the *squared* distance, which made every
 * enemy crawl exponentially slower the further away it stood — the boss ran
 * at roughly a twentieth of its nominal speed at normal engagement range.
 */
static inline fixed_t fp_dist(fixed_t dx, fixed_t dy)
{
	fixed_t ax = fp_abs(dx);
	fixed_t ay = fp_abs(dy);
	fixed_t hi = ax > ay ? ax : ay;
	fixed_t lo = ax > ay ? ay : ax;

	return hi + (lo >> 2) + (lo >> 3);
}

/* ========== GAME TYPES ========== */

enum enemy_type {
	ENEMY_NONE = 0,
	ENEMY_IMP,
	ENEMY_DEMON,
	ENEMY_BOSS,
};

enum enemy_state {
	ESTATE_IDLE,
	ESTATE_CHASE,
	ESTATE_ATTACK,
	ESTATE_DYING,
	ESTATE_DEAD,
};

struct doom_player {
	fixed_t x, y;
	fixed_t dir_x, dir_y;
	fixed_t plane_x, plane_y;
	int health;
	int ammo;
	bool firing;
};

struct doom_enemy {
	enum enemy_type type;
	enum enemy_state state;
	fixed_t x, y;
	int health;
	int anim_tick;
	int cast_tick;    /* boss only: frames left in the current cast cycle */
	int retreat_tick; /* boss only: frames left disengaging after a swing */
};

struct doom_fireball {
	bool    active;
	fixed_t x, y;
	fixed_t vx, vy;
	int     life;
};

struct doom_state {
	struct doom_player player;
	struct doom_enemy enemies[MAX_ENEMIES];
	struct doom_fireball fireballs[MAX_FIREBALLS];
	int num_enemies;
	int level;
	int score;
	bool game_over;
	bool won;
	bool portal_active;
	int portal_map_x, portal_map_y;
	int portal_anim;
	uint32_t frame_count;
};

/* ========== STATIC STATE ========== */

static bool running;
static struct doom_state game;
static uint8_t render_fb[FB_SIZE];
static atomic_t input_state;
static fixed_t zbuffer[SCREEN_W];
static int fire_cooldown;

static struct k_work_delayable game_tick_work;

/* ========== SOUND ========== */

#if DT_HAS_ALIAS(buzzer)
static const struct pwm_dt_spec doom_buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer));
#define HAS_DOOM_BUZZER 1
#else
#define HAS_DOOM_BUZZER 0
#endif

struct tone {
	uint16_t freq;
	uint16_t dur_ms;
};

static const struct tone snd_shoot[] = {
	{800, 30}, {400, 30}, {200, 40}, {0, 0}
};
static const struct tone snd_hit[] = {
	{300, 50}, {150, 80}, {0, 0}
};
static const struct tone snd_death[] = {
	{500, 80}, {400, 80}, {300, 100}, {200, 120}, {100, 150}, {0, 0}
};
/* Rising arpeggio — plays when portal opens and again on victory */
static const struct tone snd_portal[] = {
	{400, 40}, {600, 40}, {800, 40}, {1100, 80}, {0, 0}
};
/* Boss spits — descending whoosh, the audio half of the tell */
static const struct tone snd_fireball[] = {
	{900, 25}, {700, 25}, {500, 35}, {0, 0}
};
/* Boss crosses half HP and enrages */
static const struct tone snd_rage[] = {
	{200, 60}, {160, 60}, {220, 60}, {160, 140}, {0, 0}
};

static const struct tone *current_sound;
static struct k_work_delayable sound_work;

static bool sound_allowed(void)
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	return !buzzer_is_quiet();
#else
	return true;
#endif
}

static void doom_sound_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

#if HAS_DOOM_BUZZER
	if (!current_sound || current_sound->freq == 0) {
		pwm_set_pulse_dt(&doom_buzzer, 0);
		current_sound = NULL;
		return;
	}

	uint32_t period = 1000000000U / current_sound->freq;
	pwm_set_dt(&doom_buzzer, period, period / 2);

	uint16_t dur = current_sound->dur_ms;
	current_sound++;

	k_work_reschedule(&sound_work, K_MSEC(dur));
#endif
}

static void doom_play_sound(const struct tone *seq)
{
#if HAS_DOOM_BUZZER
	if (!sound_allowed()) {
		return;
	}
	current_sound = seq;
	k_work_reschedule(&sound_work, K_NO_WAIT);
#else
	ARG_UNUSED(seq);
#endif
}

static void doom_sound_off(void)
{
#if HAS_DOOM_BUZZER
	k_work_cancel_delayable(&sound_work);
	if (pwm_is_ready_dt(&doom_buzzer)) {
		pwm_set_pulse_dt(&doom_buzzer, 0);
	}
	current_sound = NULL;
#endif
}

/* ========== MAPS ========== */

/*
 * Level 1: corridors and alcoves.
 * Player starts south-centre facing north.
 * Two monsters spawn NW and NE.
 */
static const uint8_t level_1[MAP_H][MAP_W] = {
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,2,2,2,0,0,0,0,3,3,3,0,0,1},
	{1,0,0,2,0,0,0,0,0,0,0,0,3,0,0,1},
	{1,0,0,2,0,0,0,0,0,0,0,0,3,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,4,4,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,4,4,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,3,0,0,0,0,0,0,0,0,2,0,0,1},
	{1,0,0,3,0,0,0,0,0,0,0,0,2,0,0,1},
	{1,0,0,3,3,3,0,0,0,0,2,2,2,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
};

/*
 * Level 2: open boss arena with corner pillars and a hollow centre block.
 * Long sight-lines for kiting. Player enters south-centre; boss spawns in one
 * of four far corners chosen pseudo-randomly.
 */
static const uint8_t level_2[MAP_H][MAP_W] = {
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,2,0,0,0,0,0,0,0,0,2,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,3,3,3,3,0,0,0,0,0,1},
	{1,0,0,0,0,0,3,0,0,3,0,0,0,0,0,1},
	{1,0,0,0,0,0,3,0,0,3,0,0,0,0,0,1},
	{1,0,0,0,0,0,3,3,3,3,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,2,0,0,0,0,0,0,0,0,2,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
};

static const uint8_t *current_map;

static void doom_map_init(void)
{
	current_map = (game.level == 2)
		? &level_2[0][0]
		: &level_1[0][0];
}

static uint32_t isqrt64(uint64_t v)
{
	uint64_t r = 0;
	uint64_t bit = 1ULL << 62;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return (uint32_t)r;
}

/* Turn by sin_r (signed) and put the view back on the unit circle.
 *
 * The rotation used to be cos = 1 - ROT/8, sin = ROT: a matrix with scale
 * ~0.993, plus truncation in every fp_mul, so each turn tick shrank dir and
 * plane. Half a view vector after ~1.3 full turns, 1/16 after ~5: walls read
 * as ever farther away (the level "stretches for kilometres", steps get
 * short), and around ten turns 1/ray_dir overflowed int32 in the raycaster
 * and a zero wall height divided by zero - a usage fault that rebooted the
 * node. Now: the small-angle cosine, then dir renormalised to exactly unit
 * length and plane rebuilt perpendicular to it at its 2/3 field of view. */
static void rotate_view(struct doom_player *p, fixed_t sin_r)
{
	fixed_t cos_r = FP_ONE - fp_mul(sin_r, sin_r) / 2;
	fixed_t dx = fp_mul(p->dir_x, cos_r) - fp_mul(p->dir_y, sin_r);
	fixed_t dy = fp_mul(p->dir_x, sin_r) + fp_mul(p->dir_y, cos_r);
	uint32_t len = isqrt64((uint64_t)((int64_t)dx * dx) + (uint64_t)((int64_t)dy * dy));

	if (len == 0) {
		dx = 0;
		dy = -FP_ONE;
		len = FP_ONE;
	}
	p->dir_x = (fixed_t)(((int64_t)dx << FP_SHIFT) / len);
	p->dir_y = (fixed_t)(((int64_t)dy << FP_SHIFT) / len);
	p->plane_x = -(p->dir_y * 2) / 3;
	p->plane_y = (p->dir_x * 2) / 3;
}

static uint8_t doom_map_get(int x, int y)
{
	if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) {
		return 1;
	}
	return current_map[y * MAP_W + x];
}

/*
 * True when nothing solid sits on the segment (x0,y0)-(x1,y1). Sampled every
 * 1/8 tile; walls are whole tiles, so the walk cannot tunnel one. Gates both
 * the player's hitscan and the boss's fireball — this is what promotes the L2
 * pillars and centre block from scenery to cover.
 */
static bool has_line_of_sight(fixed_t x0, fixed_t y0, fixed_t x1, fixed_t y1)
{
	fixed_t dx = x1 - x0;
	fixed_t dy = y1 - y0;
	int steps  = (int)(fp_dist(dx, dy) >> (FP_SHIFT - 3));

	if (steps <= 0) return true;
	if (steps > MAP_W * 8) steps = MAP_W * 8;

	fixed_t sx = dx / steps;
	fixed_t sy = dy / steps;
	fixed_t cx = x0;
	fixed_t cy = y0;

	for (int i = 0; i < steps; i++) {
		cx += sx;
		cy += sy;
		if (doom_map_get(fp_to_int(cx), fp_to_int(cy)) != 0) {
			return false;
		}
	}
	return true;
}

/* ========== WALL TEXTURES (8x8 1-bit) ========== */

static const uint8_t tex_brick[8] = {
	0xFF, 0x88, 0xFF, 0xFF, 0x22, 0xFF, 0xFF, 0x88,
};
static const uint8_t tex_stone[8] = {
	0xEE, 0xBB, 0xFF, 0x77, 0xFF, 0xDD, 0xBB, 0xFF,
};
static const uint8_t tex_metal[8] = {
	0xFF, 0x81, 0xA5, 0x81, 0x81, 0xA5, 0x81, 0xFF,
};
static const uint8_t tex_pillar[8] = {
	0xB6, 0xB6, 0xB6, 0xFF, 0x6D, 0x6D, 0x6D, 0xFF,
};

static const uint8_t *wall_textures[5] = {
	tex_brick, tex_brick, tex_stone, tex_metal, tex_pillar,
};

/* ========== BAYER 4x4 DITHERING ========== */

static const uint8_t bayer4x4[4][4] = {
	{ 0,  8,  2, 10},
	{12,  4, 14,  6},
	{ 3, 11,  1,  9},
	{15,  7, 13,  5},
};

static int dist_to_brightness(fixed_t dist)
{
	int d = fp_to_int(dist);
	if (d < 1) return 15;
	if (d >= 8) return 1;
	return 15 - (d * 2);
}

static bool textured_pixel(int screen_x, int screen_y,
			   int tex_x, int tex_y,
			   uint8_t wall_type, bool side_hit,
			   fixed_t perp_dist)
{
	const uint8_t *tex = wall_textures[wall_type < 5 ? wall_type : 0];
	bool tex_on = (tex[tex_y & 7] & (0x80 >> (tex_x & 7))) != 0;

	if (!tex_on) return false;

	int brightness = dist_to_brightness(perp_dist);
	if (side_hit) {
		brightness = (brightness * 3) / 4;
	}

	uint8_t threshold = bayer4x4[screen_y & 3][screen_x & 3];
	return brightness > threshold;
}

static bool floor_pixel(int x, int y)
{
	int dy = y - (SCREEN_H / 2);
	if (dy <= 0) return false;

	int brightness;
	if (dy < 4)       brightness = 6;
	else if (dy < 8)  brightness = 4;
	else if (dy < 16) brightness = 3;
	else              brightness = 2;

	uint8_t threshold = bayer4x4[y & 3][x & 3];
	return brightness > threshold;
}

static bool ceiling_pixel(int x, int y)
{
	int dy = (SCREEN_H / 2) - y;
	if (dy <= 0) return false;

	int brightness;
	if (dy < 4)       brightness = 3;
	else if (dy < 8)  brightness = 2;
	else              brightness = 1;

	uint8_t threshold = bayer4x4[y & 3][x & 3];
	return brightness > threshold;
}

/* ========== SPRITES (16x16 1-bit) ========== */

static const uint16_t spr_imp_idle[16] = {
	0x07C0, 0x0FE0, 0x1BB0, 0x1FF0, 0x0FE0, 0x07C0, 0x27C8, 0x77DC,
	0xFFFE, 0xEFEE, 0x47C4, 0x07C0, 0x06C0, 0x0EE0, 0x0C60, 0x1C70,
};
static const uint16_t spr_imp_attack[16] = {
	0x07C0, 0x0FE0, 0x1BB0, 0x1FF0, 0x0FE0, 0x07C0, 0x07C0, 0xFFFE,
	0xFFFE, 0xE7CE, 0x07C0, 0x07C0, 0x06C0, 0x0EE0, 0x0C60, 0x1C70,
};
static const uint16_t spr_demon_idle[16] = {
	0x4004, 0x600C, 0x783C, 0x7FFC, 0xFFFE, 0xFBBE, 0xFFFE, 0x7FFC,
	0x7FFC, 0x7FFC, 0xFFFE, 0xFFFE, 0xF83E, 0xE82E, 0xC826, 0xC826,
};
static const uint16_t spr_demon_attack[16] = {
	0x4004, 0x600C, 0x783C, 0x7FFC, 0xFFFE, 0xF03E, 0xFBBE, 0x787C,
	0x7FFC, 0x7FFC, 0xFFFE, 0xFFFE, 0xF83E, 0xE82E, 0xC826, 0xC826,
};

/*
 * Boss: hulking demon — very wide, almost fully filled, wide-open jaw on attack.
 * Distinct from the regular demon by the solid torso and broader silhouette.
 */
static const uint16_t spr_boss_idle[16] = {
	0x2004, 0x300C, 0x7FFE, 0xFFFF,
	0xE3C7, 0xFFFF, 0xDBBD, 0xFFFF,
	0xFFFF, 0xFFFF, 0x7FFE, 0x3FFC,
	0x1FF8, 0x0FF0, 0x0660, 0x0FF0,
};
static const uint16_t spr_boss_attack[16] = {
	0x2004, 0x300C, 0x7FFE, 0xFFFF,
	0xC003, 0xE7E7, 0x8001, 0xFFFF,
	0xFFFF, 0xFFFF, 0x7FFE, 0x3FFC,
	0x1FF8, 0x0FF0, 0x0660, 0x0FF0,
};

static const uint16_t spr_dying[16] = {
	0x0000, 0x0000, 0x0000, 0x0000, 0x0200, 0x1290, 0x0840, 0x27C8,
	0x1FF0, 0x3FF8, 0x7FFC, 0x7FFC, 0xFFFE, 0xFFFE, 0xFFFE, 0xFFFE,
};
static const uint16_t spr_dead[16] = {
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x4488, 0x2B54, 0x7FFC, 0xFFFE,
};

/*
 * Portal sprites — two frames that alternate every 8 ticks (~0.4 s each):
 *   frame A: diamond outline  (converging tips at top/bottom)
 *   frame B: X pattern        (diverging to corners)
 * The visual swap reads like a spinning / pulsing energy field.
 */
static const uint16_t spr_portal_a[16] = {
	0x0180, 0x0240, 0x0420, 0x0810, 0x1008, 0x2004, 0x4002, 0x8001,
	0x8001, 0x4002, 0x2004, 0x1008, 0x0810, 0x0420, 0x0240, 0x0180,
};
static const uint16_t spr_portal_b[16] = {
	0x8001, 0x4002, 0x2004, 0x1008, 0x0810, 0x0420, 0x0240, 0x0180,
	0x0180, 0x0240, 0x0420, 0x0810, 0x1008, 0x2004, 0x4002, 0x8001,
};

/*
 * Fireball — a solid ball confined to the middle 8 rows of the 16×16 cell, so
 * it renders at eye level, at roughly half an enemy's apparent size, through
 * the shared blitter at scale 100. Solid, unlike the outlined portal
 * diamond, so the two never read as the same thing.
 */
static const uint16_t spr_fireball[16] = {
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0180, 0x03C0, 0x07E0, 0x0FF0,
	0x0FF0, 0x07E0, 0x03C0, 0x0180,
	0x0000, 0x0000, 0x0000, 0x0000,
};

/* Boss phase 2: below half HP it moves twice as fast and casts more often. */
static inline bool boss_enraged(const struct doom_enemy *e)
{
	return e->type == ENEMY_BOSS && e->health <= BOSS_HP / 2;
}

/*
 * The boss's one tell. It is fully vulnerable exactly while its jaw is open:
 * during the fireball wind-up, and on the claws-out frames of a melee swing.
 * Both the renderer and the damage path call this, so the open-jaw sprite on
 * screen is a literal readout of the damage window rather than decoration.
 */
static bool boss_jaw_open(const struct doom_enemy *e)
{
	if (e->type != ENEMY_BOSS) return false;

	if (e->state == ESTATE_ATTACK) {
		return (e->anim_tick & 4) != 0;
	}
	if (e->state != ESTATE_CHASE) {
		return false;
	}
	return e->cast_tick > 0 && e->cast_tick <= BOSS_WINDUP;
}

static const uint16_t *get_sprite_16(const struct doom_enemy *e)
{
	if (e->state == ESTATE_DEAD)  return spr_dead;
	if (e->state == ESTATE_DYING) return spr_dying;

	switch (e->type) {
	case ENEMY_IMP:
		return (e->state == ESTATE_ATTACK && (e->anim_tick & 4))
			? spr_imp_attack : spr_imp_idle;
	case ENEMY_DEMON:
		return (e->state == ESTATE_ATTACK && (e->anim_tick & 4))
			? spr_demon_attack : spr_demon_idle;
	case ENEMY_BOSS:
		return boss_jaw_open(e) ? spr_boss_attack : spr_boss_idle;
	default:
		return spr_imp_idle;
	}
}

/* ========== RAYCASTER ========== */

/* The sprite texel under screen pixel (sc, sr) of a sprite drawn at
 * [x0, x0 + w) x [y0, y0 + h). */
static inline bool sprite_texel_on(const uint16_t *sprite, int sc, int sr,
				   int x0, int y0, int w, int h)
{
	int tex_x = (sc - x0) * 16 / w;
	int tex_y = (sr - y0) * 16 / h;

	if (tex_x > 15) tex_x = 15;
	if (tex_y > 15) tex_y = 15;
	return (sprite[tex_y] & (0x8000 >> tex_x)) != 0;
}

static inline void clear_px(int x, int y)
{
	render_fb[(y / 8) * SCREEN_W + x] &= (uint8_t)~(1U << (y & 7));
}

/*
 * Render one 16×16 sprite at world position (wx, wy), scale_pct percent of a
 * wall's height with its feet on the floor. Shared by the portal, the enemies
 * and the fireballs.
 *
 * Sprites are solid with a one-pixel black halo, not distance-dithered like
 * the walls. On a 1-bit panel a dithered sprite ORed over the dithered floor
 * band at the horizon has nothing to contrast against: the L2 boss, which
 * starts 8-12 tiles away, was a few dots at best and a grey smudge in the
 * floor's own pattern at 4 tiles. The halo pass clears first, then the fill
 * sets; both honour the zbuffer, so a nearer wall is never cut into.
 */
static void render_one_sprite(const uint16_t *sprite,
			      fixed_t wx, fixed_t wy,
			      fixed_t inv_det,
			      struct doom_player *p,
			      int scale_pct)
{
	fixed_t sx = wx - p->x;
	fixed_t sy = wy - p->y;

	fixed_t transform_x = fp_mul(inv_det,
		fp_mul(p->dir_y, sx) - fp_mul(p->dir_x, sy));
	fixed_t transform_y = fp_mul(inv_det,
		fp_mul(-p->plane_y, sx) + fp_mul(p->plane_x, sy));

	if (transform_y <= 0) return;

	int sprite_screen_x = (SCREEN_W / 2) +
		fp_to_int(fp_div(fp_mul(transform_x,
			fp_from_int(SCREEN_W)), transform_y));

	int base_h = fp_to_int(fp_abs(fp_div(
		fp_from_int(SCREEN_H), transform_y)));
	if (base_h > SCREEN_H * 2) base_h = SCREEN_H * 2;

	int sprite_h = base_h * scale_pct / 100;
	if (sprite_h < 3) return;

	int draw_end_y   = (SCREEN_H + base_h) / 2;  /* the floor line */
	int draw_start_y = draw_end_y - sprite_h;
	int draw_start_x = sprite_screen_x - sprite_h / 2;
	int draw_end_x   = sprite_screen_x + sprite_h / 2;
	int sprite_w     = draw_end_x - draw_start_x;

	int x_lo = draw_start_x < 0 ? 0 : draw_start_x;
	int x_hi = draw_end_x > SCREEN_W ? SCREEN_W : draw_end_x;
	int y_lo = draw_start_y < 0 ? 0 : draw_start_y;
	int y_hi = draw_end_y > SCREEN_H ? SCREEN_H : draw_end_y;

	for (int pass = 0; pass < 2; pass++) {
		for (int sc = x_lo; sc < x_hi; sc++) {
			if (transform_y >= zbuffer[sc]) continue;

			for (int sr = y_lo; sr < y_hi; sr++) {
				if (!sprite_texel_on(sprite, sc, sr,
						     draw_start_x, draw_start_y,
						     sprite_w, sprite_h)) {
					continue;
				}
				if (pass == 1) {
					render_fb[(sr / 8) * SCREEN_W + sc] |=
						(1U << (sr & 7));
					continue;
				}
				for (int hx = sc - 1; hx <= sc + 1; hx++) {
					if (hx < 0 || hx >= SCREEN_W ||
					    transform_y >= zbuffer[hx]) {
						continue;
					}
					for (int hy = sr - 1; hy <= sr + 1; hy++) {
						if (hy >= 0 && hy < SCREEN_H) {
							clear_px(hx, hy);
						}
					}
				}
			}
		}
	}
}

static void render_sprites(void)
{
	struct doom_player *p = &game.player;

	fixed_t det = fp_mul(p->plane_x, p->dir_y) -
		      fp_mul(p->dir_x, p->plane_y);
	if (det == 0) return;
	fixed_t inv_det = fp_div(FP_ONE, det);

	/* Portal renders behind enemies (drawn first, no depth-sort needed) */
	if (game.portal_active) {
		const uint16_t *pspr = (game.portal_anim & 8)
			? spr_portal_b : spr_portal_a;
		render_one_sprite(pspr,
			fp_from_int(game.portal_map_x) + FP_HALF,
			fp_from_int(game.portal_map_y) + FP_HALF,
			inv_det, p, 100);
	}

	/* Collect visible enemies and sort farthest-first */
	struct sprite_order {
		int idx;
		fixed_t dist;
	} order[MAX_ENEMIES];
	int visible = 0;

	for (int i = 0; i < game.num_enemies; i++) {
		struct doom_enemy *e = &game.enemies[i];
		if (e->type == ENEMY_NONE || e->state == ESTATE_DEAD) continue;

		fixed_t dx = e->x - p->x;
		fixed_t dy = e->y - p->y;
		order[visible].idx  = i;
		order[visible].dist = fp_mul(dx, dx) + fp_mul(dy, dy);
		visible++;
	}

	for (int i = 1; i < visible; i++) {
		struct sprite_order tmp = order[i];
		int j = i - 1;
		while (j >= 0 && order[j].dist < tmp.dist) {
			order[j + 1] = order[j];
			j--;
		}
		order[j + 1] = tmp;
	}

	for (int i = 0; i < visible; i++) {
		struct doom_enemy *e = &game.enemies[order[i].idx];
		render_one_sprite(get_sprite_16(e), e->x, e->y, inv_det, p,
				  e->type == ENEMY_BOSS ? 150 : 100);
	}

	/*
	 * Fireballs draw last, over the enemies. They fly toward the player so
	 * they are almost always the nearest sprite anyway, and the zbuffer
	 * still hides them behind walls — which is the part that matters for
	 * reading whether you are actually in cover.
	 */
	for (int i = 0; i < MAX_FIREBALLS; i++) {
		struct doom_fireball *f = &game.fireballs[i];
		if (!f->active) continue;
		render_one_sprite(spr_fireball, f->x, f->y, inv_det, p, 100);
	}
}

static void raycaster_render(void)
{
	struct doom_player *p = &game.player;

	for (int x = 0; x < SCREEN_W; x++) {
		fixed_t camera_x = fp_div(fp_from_int(2 * x - SCREEN_W),
					  fp_from_int(SCREEN_W));

		fixed_t ray_dir_x = p->dir_x + fp_mul(p->plane_x, camera_x);
		fixed_t ray_dir_y = p->dir_y + fp_mul(p->plane_y, camera_x);

		int map_x = fp_to_int(p->x);
		int map_y = fp_to_int(p->y);

		/* 1/|ray_dir|, capped: near an axis it exceeds int32 and the DDA
		 * below adds it up. 1024 tiles is far past any ray in a 16x16 map. */
		fixed_t delta_dist_x = (fp_abs(ray_dir_x) < (FP_ONE >> 10))
			? fp_from_int(1024)
			: fp_abs(fp_div(FP_ONE, ray_dir_x));
		fixed_t delta_dist_y = (fp_abs(ray_dir_y) < (FP_ONE >> 10))
			? fp_from_int(1024)
			: fp_abs(fp_div(FP_ONE, ray_dir_y));

		int step_x, step_y;
		fixed_t side_dist_x, side_dist_y;

		if (ray_dir_x < 0) {
			step_x = -1;
			side_dist_x = fp_mul(p->x - fp_from_int(map_x),
					     delta_dist_x);
		} else {
			step_x = 1;
			side_dist_x = fp_mul(fp_from_int(map_x + 1) - p->x,
					     delta_dist_x);
		}

		if (ray_dir_y < 0) {
			step_y = -1;
			side_dist_y = fp_mul(p->y - fp_from_int(map_y),
					     delta_dist_y);
		} else {
			step_y = 1;
			side_dist_y = fp_mul(fp_from_int(map_y + 1) - p->y,
					     delta_dist_y);
		}

		bool hit = false;
		bool side = false;
		uint8_t wall_type = 0;

		for (int i = 0; i < 64; i++) {
			if (side_dist_x < side_dist_y) {
				side_dist_x += delta_dist_x;
				map_x += step_x;
				side = false;
			} else {
				side_dist_y += delta_dist_y;
				map_y += step_y;
				side = true;
			}

			wall_type = doom_map_get(map_x, map_y);
			if (wall_type > 0) {
				hit = true;
				break;
			}
		}

		fixed_t perp_dist;
		if (!side) {
			perp_dist = side_dist_x - delta_dist_x;
		} else {
			perp_dist = side_dist_y - delta_dist_y;
		}

		if (perp_dist < (FP_ONE / 16)) {
			perp_dist = FP_ONE / 16;
		}

		zbuffer[x] = perp_dist;

		if (!hit) {
			for (int y = 0; y < SCREEN_H / 2; y++) {
				if (ceiling_pixel(x, y)) {
					render_fb[(y / 8) * SCREEN_W + x] |=
						(1U << (y & 7));
				}
			}
			for (int y = SCREEN_H / 2; y < SCREEN_H; y++) {
				if (floor_pixel(x, y)) {
					render_fb[(y / 8) * SCREEN_W + x] |=
						(1U << (y & 7));
				}
			}
			continue;
		}

		int line_height = fp_to_int(fp_div(fp_from_int(SCREEN_H),
						   perp_dist));
		/* A wall farther than SCREEN_H rounds to 0 lines, and the texture
		 * row below divides by this: a usage fault (divide by zero) that
		 * rebooted the node mid-game. */
		if (line_height < 1) {
			line_height = 1;
		}

		int draw_start = (SCREEN_H - line_height) / 2;
		int draw_end   = draw_start + line_height;

		fixed_t wall_x;
		if (!side) {
			wall_x = p->y + fp_mul(perp_dist, ray_dir_y);
		} else {
			wall_x = p->x + fp_mul(perp_dist, ray_dir_x);
		}
		wall_x -= fp_from_int(fp_to_int(wall_x));

		int tex_x = fp_to_int(fp_mul(wall_x, fp_from_int(8)));
		if (tex_x < 0) tex_x = 0;
		if (tex_x > 7) tex_x = 7;

		if ((!side && ray_dir_x > 0) || (side && ray_dir_y < 0)) {
			tex_x = 7 - tex_x;
		}

		int actual_start = draw_start < 0 ? 0 : draw_start;
		int actual_end   = draw_end >= SCREEN_H ? SCREEN_H - 1 : draw_end;

		for (int y = actual_start; y <= actual_end; y++) {
			int tex_y = ((y - draw_start) * 8) / line_height;
			if (tex_y < 0) tex_y = 0;
			if (tex_y > 7) tex_y = 7;

			if (textured_pixel(x, y, tex_x, tex_y,
					   wall_type, side, perp_dist)) {
				render_fb[(y / 8) * SCREEN_W + x] |=
					(1U << (y & 7));
			}
		}

		for (int y = 0; y < actual_start; y++) {
			if (ceiling_pixel(x, y)) {
				render_fb[(y / 8) * SCREEN_W + x] |=
					(1U << (y & 7));
			}
		}

		for (int y = actual_end + 1; y < SCREEN_H; y++) {
			if (floor_pixel(x, y)) {
				render_fb[(y / 8) * SCREEN_W + x] |=
					(1U << (y & 7));
			}
		}
	}

	render_sprites();
}

/* ========== PIXEL HELPERS ========== */

static inline void set_px(int x, int y)
{
	if (x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H) {
		render_fb[(y / 8) * SCREEN_W + x] |= (1U << (y & 7));
	}
}

/* ========== 4x5 DIGIT FONT ========== */

static const uint8_t digits[10][5] = {
	{0xF, 0x9, 0x9, 0x9, 0xF},
	{0x2, 0x6, 0x2, 0x2, 0x7},
	{0xF, 0x1, 0xF, 0x8, 0xF},
	{0xF, 0x1, 0xF, 0x1, 0xF},
	{0x9, 0x9, 0xF, 0x1, 0x1},
	{0xF, 0x8, 0xF, 0x1, 0xF},
	{0xF, 0x8, 0xF, 0x9, 0xF},
	{0xF, 0x1, 0x2, 0x4, 0x4},
	{0xF, 0x9, 0xF, 0x9, 0xF},
	{0xF, 0x9, 0xF, 0x1, 0xF},
};

static const uint8_t glyph_H[5] = {0x9, 0x9, 0xF, 0x9, 0x9};
static const uint8_t glyph_P[5] = {0xE, 0x9, 0xE, 0x8, 0x8};
static const uint8_t glyph_A[5] = {0x6, 0x9, 0xF, 0x9, 0x9};
static const uint8_t glyph_M[5] = {0x9, 0xF, 0xF, 0x9, 0x9};
static const uint8_t glyph_G[5] = {0x7, 0x8, 0xB, 0x9, 0x7};
static const uint8_t glyph_E[5] = {0xF, 0x8, 0xE, 0x8, 0xF};
static const uint8_t glyph_O[5] = {0x6, 0x9, 0x9, 0x9, 0x6};
static const uint8_t glyph_V[5] = {0x9, 0x9, 0x9, 0x6, 0x4};
static const uint8_t glyph_R[5] = {0xE, 0x9, 0xE, 0xC, 0x9};
static const uint8_t glyph_L[5] = {0x8, 0x8, 0x8, 0x8, 0xF};

static void draw_glyph(int dx, int dy, const uint8_t *glyph)
{
	for (int row = 0; row < 5; row++) {
		for (int col = 0; col < 4; col++) {
			if (glyph[row] & (8 >> col)) {
				set_px(dx + col, dy + row);
			}
		}
	}
}

static void draw_digit(int dx, int dy, int d)
{
	if (d < 0 || d > 9) return;
	draw_glyph(dx, dy, digits[d]);
}

static void draw_number(int x, int y, int val)
{
	if (val < 0) val = 0;
	if (val > 999) val = 999;
	if (val >= 100) {
		draw_digit(x, y, val / 100);
		x += 5;
	}
	if (val >= 10) {
		draw_digit(x, y, (val / 10) % 10);
		x += 5;
	}
	draw_digit(x, y, val % 10);
}

/* ========== HUD ========== */

static void draw_health_bar(int x, int y, int hp)
{
	int bar_w = 20;
	int filled = (hp * bar_w) / 100;
	if (filled < 0) filled = 0;
	if (filled > bar_w) filled = bar_w;

	for (int bx = x - 1; bx <= x + bar_w; bx++) {
		set_px(bx, y - 1);
		set_px(bx, y + 3);
	}
	for (int by = y; by < y + 3; by++) {
		set_px(x - 1, by);
		set_px(x + bar_w, by);
	}

	for (int bx = x; bx < x + filled; bx++) {
		for (int by = y; by < y + 3; by++) {
			set_px(bx, by);
		}
	}
}

static void draw_hud(void)
{
	memset(&render_fb[7 * SCREEN_W], 0, SCREEN_W);

	/* Separator line at y=57 */
	for (int x = 0; x < SCREEN_W; x++) {
		render_fb[7 * SCREEN_W + x] |= 0x01;
	}

	/* HP bar */
	draw_glyph(1, 58, glyph_H);
	draw_glyph(6, 58, glyph_P);
	draw_health_bar(12, 58, game.player.health);

	/* Ammo */
	draw_glyph(48, 58, glyph_A);
	draw_glyph(53, 58, glyph_M);
	draw_number(59, 58, game.player.ammo);

	/* Level indicator: "LV1" / "LV2" */
	draw_glyph(76, 58, glyph_L);
	draw_glyph(81, 58, glyph_V);
	draw_number(87, 58, game.level);

	/* Score */
	draw_number(100, 58, game.score);
}

/*
 * L2 only: the boss's HP as a two-pixel bar across the top of the viewport.
 * Drawn after the raycaster so it sits over the ceiling dither. It blinks once
 * the boss is enraged, which is the only other signal that phase 2 started.
 */
static void draw_boss_bar(void)
{
	if (game.level != 2 || game.num_enemies < 1) return;

	const struct doom_enemy *e = &game.enemies[0];
	if (e->type != ENEMY_BOSS || e->state >= ESTATE_DYING) return;

	/* Clear rows 0-2: the blank third row separates bar from ceiling */
	for (int x = 0; x < SCREEN_W; x++) {
		render_fb[x] &= (uint8_t)~0x07;
	}

	if (boss_enraged(e) && (game.frame_count & 4)) return;

	int hp = e->health < 0 ? 0 : e->health;
	int w  = (hp * (SCREEN_W - 4)) / BOSS_HP;

	for (int x = 2; x < 2 + w; x++) {
		render_fb[x] |= 0x03;
	}
}

/* ========== GUN SPRITE ========== */

static const uint16_t gun_idle[12] = {
	0x0180, 0x0180, 0x03C0, 0x03C0, 0x07E0, 0x0FF0,
	0x1FF8, 0x1FF8, 0x3FFC, 0x398C, 0x7186, 0x6186,
};
static const uint16_t gun_fire[12] = {
	0x0990, 0x1248, 0x07E0, 0x0FF0, 0x0FF0, 0x1FF8,
	0x3FFC, 0x3FFC, 0x7FFE, 0x798E, 0xF187, 0xE187,
};

static void draw_gun(bool firing)
{
	const uint16_t *sprite = firing ? gun_fire : gun_idle;
	int cx = SCREEN_W / 2 - 8;
	int by = SCREEN_H - 12 - 8;

	if (firing) by -= 1;

	for (int row = 0; row < 12; row++) {
		for (int col = 0; col < 16; col++) {
			if (sprite[row] & (0x8000 >> col)) {
				set_px(cx + col, by + row);
			}
		}
	}

	int ch_x = SCREEN_W / 2;
	int ch_y = SCREEN_H / 2 - 4;
	set_px(ch_x, ch_y - 2);
	set_px(ch_x, ch_y - 1);
	set_px(ch_x, ch_y + 1);
	set_px(ch_x, ch_y + 2);
	set_px(ch_x - 2, ch_y);
	set_px(ch_x - 1, ch_y);
	set_px(ch_x + 1, ch_y);
	set_px(ch_x + 2, ch_y);
}

/* ========== END SCREENS ========== */

static const uint8_t letter_D[8] = {
	0xFC, 0xFE, 0x86, 0x86, 0x86, 0x86, 0xFE, 0xFC
};
static const uint8_t letter_E[8] = {
	0xFE, 0xFE, 0x80, 0xF8, 0xF8, 0x80, 0xFE, 0xFE
};
static const uint8_t letter_A[8] = {
	0x7C, 0xFE, 0x86, 0x86, 0xFE, 0xFE, 0x86, 0x86
};
static const uint8_t letter_W[8] = {
	0x82, 0x82, 0x82, 0x92, 0xAA, 0xC6, 0x82, 0x00
};
static const uint8_t letter_I[8] = {
	0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE
};
static const uint8_t letter_N[8] = {
	0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x00
};

static void blit_letter(int lx, int ly, const uint8_t *data)
{
	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 8; col++) {
			if (data[row] & (0x80 >> col)) {
				int px = lx + col;
				int py = ly + row;
				if (px >= 0 && px < SCREEN_W &&
				    py >= 0 && py < SCREEN_H) {
					render_fb[(py/8)*SCREEN_W + px] |=
						(1U << (py & 7));
				}
			}
		}
	}
}

static void draw_game_over(void)
{
	memset(render_fb, 0, FB_SIZE);

	if (game.won) {
		/* "WIN" + score */
		int sx = (SCREEN_W - 29) / 2;
		int sy = 18;
		blit_letter(sx,      sy, letter_W);
		blit_letter(sx + 10, sy, letter_I);
		blit_letter(sx + 20, sy, letter_N);

		/* Sub-line: score */
		draw_number((SCREEN_W - 15) / 2, sy + 14, game.score);
	} else {
		/* "DEAD" */
		int sx = (SCREEN_W - 38) / 2;
		int sy = 20;
		blit_letter(sx,      sy, letter_D);
		blit_letter(sx + 10, sy, letter_E);
		blit_letter(sx + 20, sy, letter_A);
		blit_letter(sx + 30, sy, letter_D);

		int go_x = (SCREEN_W - 40) / 2;
		int go_y = sy + 14;
		draw_glyph(go_x,      go_y, glyph_G);
		draw_glyph(go_x +  5, go_y, glyph_A);
		draw_glyph(go_x + 10, go_y, glyph_M);
		draw_glyph(go_x + 15, go_y, glyph_E);
		draw_glyph(go_x + 22, go_y, glyph_O);
		draw_glyph(go_x + 27, go_y, glyph_V);
		draw_glyph(go_x + 32, go_y, glyph_E);
		draw_glyph(go_x + 37, go_y, glyph_R);
	}
}

/* ========== DISPLAY ========== */

static void doom_flush_fb(void)
{
	const struct device *dev = mc_display_get_device();
	if (!dev) return;

	struct display_buffer_descriptor desc = {
		.buf_size = FB_SIZE,
		.width = SCREEN_W,
		.height = SCREEN_H,
		.pitch = SCREEN_W,
	};

	display_write(dev, 0, 0, &desc, render_fb);
}

/* ========== GAME LOGIC ========== */

static void game_init(void)
{
	memset(&game, 0, sizeof(game));

	game.player.x      = fp_from_int(8) + FP_HALF;
	game.player.y      = fp_from_int(13) + FP_HALF;
	game.player.health = 100;
	game.player.ammo   = 50;
	game.player.dir_x  = 0;
	game.player.dir_y  = -FP_ONE;
	game.player.plane_x = (FP_ONE * 2) / 3;
	game.player.plane_y = 0;

	/* Level 1: exactly two monsters — one imp (NW), one demon (NE) */
	game.enemies[0] = (struct doom_enemy){
		.type   = ENEMY_IMP,
		.state  = ESTATE_IDLE,
		.x      = fp_from_int(4) + FP_HALF,
		.y      = fp_from_int(4) + FP_HALF,
		.health = 30,
	};
	/* x = 11, mirroring the imp: (12,4) is inside the NE wall block, where
	 * line of sight never reached the demon, so it could not be shot and the
	 * portal never opened. */
	game.enemies[1] = (struct doom_enemy){
		.type   = ENEMY_DEMON,
		.state  = ESTATE_IDLE,
		.x      = fp_from_int(11) + FP_HALF,
		.y      = fp_from_int(4) + FP_HALF,
		.health = 60,
	};
	game.num_enemies = 2;
	game.level       = 1;
	game.score       = 0;
	game.game_over   = false;
	game.won         = false;
	game.portal_active = false;

	fire_cooldown = 0;
	atomic_set(&input_state, 0);
	doom_map_init();
}

static bool can_move(fixed_t new_x, fixed_t new_y)
{
	int checks[][2] = {
		{fp_to_int(new_x + COLLISION_R), fp_to_int(new_y + COLLISION_R)},
		{fp_to_int(new_x - COLLISION_R), fp_to_int(new_y + COLLISION_R)},
		{fp_to_int(new_x + COLLISION_R), fp_to_int(new_y - COLLISION_R)},
		{fp_to_int(new_x - COLLISION_R), fp_to_int(new_y - COLLISION_R)},
	};

	for (int i = 0; i < 4; i++) {
		if (doom_map_get(checks[i][0], checks[i][1]) > 0) {
			return false;
		}
	}
	return true;
}

static void handle_movement(uint32_t input)
{
	struct doom_player *p = &game.player;

	if (input & DINPUT_FWD) {
		fixed_t nx = p->x + fp_mul(p->dir_x, MOVE_SPEED);
		fixed_t ny = p->y + fp_mul(p->dir_y, MOVE_SPEED);
		if (can_move(nx, p->y)) p->x = nx;
		if (can_move(p->x, ny)) p->y = ny;
	}

	if (input & DINPUT_BACK) {
		fixed_t nx = p->x - fp_mul(p->dir_x, MOVE_SPEED);
		fixed_t ny = p->y - fp_mul(p->dir_y, MOVE_SPEED);
		if (can_move(nx, p->y)) p->x = nx;
		if (can_move(p->x, ny)) p->y = ny;
	}

	if (input & DINPUT_LEFT) {
		rotate_view(p, ROT_SPEED);
	}

	if (input & DINPUT_RIGHT) {
		rotate_view(p, -ROT_SPEED);
	}
}

static void handle_shooting(uint32_t input)
{
	struct doom_player *p = &game.player;

	if (!(input & DINPUT_FIRE) || p->ammo <= 0) {
		p->firing = false;
		return;
	}

	if (p->firing) return;

	p->firing = true;
	p->ammo--;
	doom_play_sound(snd_shoot);

	for (int i = 0; i < game.num_enemies; i++) {
		struct doom_enemy *e = &game.enemies[i];
		if (e->type == ENEMY_NONE || e->state >= ESTATE_DYING) continue;

		fixed_t dx = e->x - p->x;
		fixed_t dy = e->y - p->y;

		if (fp_dist(dx, dy) > fp_from_int(10)) continue;

		fixed_t dot = fp_mul(dx, p->dir_x) + fp_mul(dy, p->dir_y);
		if (dot <= 0) continue;

		fixed_t cross = fp_abs(fp_mul(dx, p->dir_y) -
				       fp_mul(dy, p->dir_x));
		if (cross >= FP_HALF) continue;

		/* Walls stop bullets — cover works both ways now */
		if (!has_line_of_sight(p->x, p->y, e->x, e->y)) continue;

		bool was_above_half = e->health > BOSS_HP / 2;
		int  dmg            = SHOT_DAMAGE;

		if (e->type == ENEMY_BOSS && !boss_jaw_open(e)) {
			dmg /= BOSS_GUARD_DIV;
		}

		e->health -= dmg;
		doom_play_sound(snd_hit);

		if (e->health <= 0) {
			e->state     = ESTATE_DYING;
			e->anim_tick = 0;
			game.score  += (e->type == ENEMY_BOSS)  ? 1000 :
				       (e->type == ENEMY_DEMON) ? 200  : 100;

			if (e->type == ENEMY_BOSS) {
				/* Don't let a fireball already in the air rob
				 * a won run during the death animation */
				memset(game.fireballs, 0,
				       sizeof(game.fireballs));
			}
		} else {
			/*
			 * Only wake sleepers. Promoting unconditionally, as
			 * this did before, cancelled an in-progress swing —
			 * which under the new rules would also slam the jaw
			 * shut on the player's own damage window.
			 */
			if (e->state == ESTATE_IDLE) {
				e->state = ESTATE_CHASE;
			}
			if (e->type == ENEMY_BOSS && was_above_half &&
			    e->health <= BOSS_HP / 2) {
				doom_play_sound(snd_rage);
			}
		}
		break;
	}
}

/*
 * Scan the current map for open floor tiles that have open neighbours on all
 * four cardinal sides and are at least 4 tiles (squared) from the player.
 * Pick pseudo-randomly using frame_count as a seed.
 */
static void find_portal_spawn(int *out_x, int *out_y)
{
	int cand_x[32], cand_y[32];
	int count = 0;

	for (int y = 1; y < MAP_H - 1 && count < 32; y++) {
		for (int x = 1; x < MAP_W - 1 && count < 32; x++) {
			if (doom_map_get(x,     y    ) != 0) continue;
			if (doom_map_get(x - 1, y    ) != 0) continue;
			if (doom_map_get(x + 1, y    ) != 0) continue;
			if (doom_map_get(x,     y - 1) != 0) continue;
			if (doom_map_get(x,     y + 1) != 0) continue;

			int pdx = x - fp_to_int(game.player.x);
			int pdy = y - fp_to_int(game.player.y);
			if (pdx * pdx + pdy * pdy < 16) continue;

			cand_x[count] = x;
			cand_y[count] = y;
			count++;
		}
	}

	if (count == 0) {
		/* Fallback — should never trigger on the defined maps */
		*out_x = 8;
		*out_y = 2;
		return;
	}

	int idx = (int)(game.frame_count % (uint32_t)count);
	*out_x = cand_x[idx];
	*out_y = cand_y[idx];
}

/* If all L1 enemies are dead, open a portal at a random floor tile. */
static void check_level_complete(void)
{
	if (game.portal_active || game.level != 1) return;

	for (int i = 0; i < game.num_enemies; i++) {
		if (game.enemies[i].type != ENEMY_NONE &&
		    game.enemies[i].state != ESTATE_DEAD) {
			return;
		}
	}

	int px, py;
	find_portal_spawn(&px, &py);
	game.portal_active = true;
	game.portal_map_x  = px;
	game.portal_map_y  = py;
	game.portal_anim   = 0;
	doom_play_sound(snd_portal);
}

/*
 * Transition to level 2.
 * Player keeps current health and ammo.
 * Boss spawns in one of four far corners, chosen pseudo-randomly.
 */
static void advance_to_level2(void)
{
	game.level         = 2;
	game.portal_active = false;
	current_map        = &level_2[0][0];

	/* Player respawns south-centre, facing north */
	game.player.x       = fp_from_int(8) + FP_HALF;
	game.player.y       = fp_from_int(13) + FP_HALF;
	game.player.dir_x   = 0;
	game.player.dir_y   = -FP_ONE;
	game.player.plane_x = (FP_ONE * 2) / 3;
	game.player.plane_y = 0;

	static const int8_t boss_spots[4][2] = {
		{2, 2}, {13, 2}, {2, 7}, {13, 7}
	};
	int bi = (int)(game.frame_count % 4);

	memset(game.enemies, 0, sizeof(game.enemies));
	memset(game.fireballs, 0, sizeof(game.fireballs));
	game.enemies[0] = (struct doom_enemy){
		.type      = ENEMY_BOSS,
		.state     = ESTATE_CHASE,   /* immediately aggressive */
		.x         = fp_from_int(boss_spots[bi][0]) + FP_HALF,
		.y         = fp_from_int(boss_spots[bi][1]) + FP_HALF,
		.health    = BOSS_HP,
		.cast_tick = BOSS_CAST_PERIOD,
	};
	game.num_enemies = 1;
}

/*
 * Launch a fireball from the boss toward wherever the player stands at the
 * moment of release. It does not lead the target, so a sidestep started when
 * the jaw opens always clears it — standing still never does.
 */
static void boss_spit_fireball(const struct doom_enemy *e)
{
	struct doom_player *p = &game.player;

	for (int i = 0; i < MAX_FIREBALLS; i++) {
		struct doom_fireball *f = &game.fireballs[i];
		if (f->active) continue;

		fixed_t dx = p->x - e->x;
		fixed_t dy = p->y - e->y;
		fixed_t d  = fp_dist(dx, dy);

		if (d < FP_ONE / 8) d = FP_ONE / 8;

		f->vx     = fp_mul(fp_div(dx, d), FIREBALL_SPEED);
		f->vy     = fp_mul(fp_div(dy, d), FIREBALL_SPEED);
		f->x      = e->x + f->vx;
		f->y      = e->y + f->vy;
		f->life   = FIREBALL_LIFE;
		f->active = true;

		doom_play_sound(snd_fireball);
		return;
	}
}

/*
 * One frame of the boss's cast cycle, run while chasing. Returns true while
 * the boss is winding up — jaw open, feet planted, fully vulnerable — which
 * is the opening the fight is built around. It only arms at range and with
 * line of sight, so a player who closes to melee or ducks behind the centre
 * block fights the claws instead of the fire.
 */
static bool boss_run_cast_cycle(struct doom_enemy *e, fixed_t dist)
{
	struct doom_player *p = &game.player;
	int period = boss_enraged(e) ? BOSS_CAST_PERIOD_RAGE
				     : BOSS_CAST_PERIOD;

	if (dist < BOSS_CAST_MIN_DIST ||
	    !has_line_of_sight(e->x, e->y, p->x, p->y)) {
		/* Hold the timer above the tell so the jaw never opens dry */
		if (e->cast_tick <= BOSS_WINDUP) {
			e->cast_tick = BOSS_WINDUP + 1;
		}
		return false;
	}

	/* Enraging shortens the period; don't strand the timer above it */
	if (e->cast_tick > period) e->cast_tick = period;
	if (e->cast_tick > 0)      e->cast_tick--;

	if (e->cast_tick == 0) {
		boss_spit_fireball(e);
		e->cast_tick = period;
		return false;
	}

	return e->cast_tick <= BOSS_WINDUP;
}

static void update_fireballs(void)
{
	struct doom_player *p = &game.player;

	for (int i = 0; i < MAX_FIREBALLS; i++) {
		struct doom_fireball *f = &game.fireballs[i];
		if (!f->active) continue;

		f->x += f->vx;
		f->y += f->vy;

		if (--f->life <= 0 ||
		    doom_map_get(fp_to_int(f->x), fp_to_int(f->y)) != 0) {
			f->active = false;
			continue;
		}

		if (fp_dist(p->x - f->x, p->y - f->y) >= FIREBALL_HIT_R) {
			continue;
		}

		f->active   = false;
		p->health  -= FIREBALL_DMG;
		doom_play_sound(snd_hit);

		if (p->health <= 0) {
			game.game_over = true;
			doom_play_sound(snd_death);
		}
	}
}

static void update_enemies(void)
{
	struct doom_player *p = &game.player;

	for (int i = 0; i < game.num_enemies; i++) {
		struct doom_enemy *e = &game.enemies[i];
		if (e->type == ENEMY_NONE) continue;

		fixed_t spd = (e->type != ENEMY_BOSS) ? ENEMY_SPEED :
			      boss_enraged(e) ? BOSS_SPEED_RAGE : BOSS_SPEED;

		switch (e->state) {
		case ESTATE_IDLE: {
			if (fp_dist(e->x - p->x, e->y - p->y) <
			    fp_from_int(6)) {
				e->state = ESTATE_CHASE;
			}
			break;
		}
		case ESTATE_CHASE: {
			fixed_t dx   = p->x - e->x;
			fixed_t dy   = p->y - e->y;
			fixed_t dist = fp_dist(dx, dy);
			bool    back = false;

			if (e->retreat_tick > 0) {
				e->retreat_tick--;
				back = true;   /* disengaging after a swing */
			} else if (dist < FP_ONE + FP_HALF) {
				e->state     = ESTATE_ATTACK;
				e->anim_tick = 0;
				break;
			}

			/* A winding-up boss plants its feet for the tell */
			if (e->type == ENEMY_BOSS &&
			    boss_run_cast_cycle(e, dist)) {
				break;
			}

			/* Retreating skips the range check above, so floor the
			 * divisor rather than trust dist to be 1.5+ tiles */
			if (dist < FP_ONE / 4) dist = FP_ONE / 4;

			fixed_t step   = back ? -spd : spd;
			fixed_t move_x = fp_mul(fp_div(dx, dist), step);
			fixed_t move_y = fp_mul(fp_div(dy, dist), step);

			fixed_t nx = e->x + move_x;
			fixed_t ny = e->y + move_y;

			if (doom_map_get(fp_to_int(nx), fp_to_int(ny)) == 0) {
				e->x = nx;
				e->y = ny;
			}
			break;
		}
		case ESTATE_ATTACK:
			e->anim_tick++;
			if (e->anim_tick >= 20) {
				int dmg = (e->type == ENEMY_BOSS)  ? 25 :
					  (e->type == ENEMY_DEMON) ? 15 : 8;

				p->health -= dmg;
				doom_play_sound(snd_hit);

				if (p->health <= 0) {
					game.game_over = true;
					doom_play_sound(snd_death);
				}
				e->anim_tick = 0;
				e->state     = ESTATE_CHASE;

				/* Boss disengages after every swing so its
				 * ranged tell comes back around */
				if (e->type == ENEMY_BOSS) {
					e->retreat_tick = BOSS_RETREAT;
				}
			}
			break;
		case ESTATE_DYING:
			e->anim_tick++;
			if (e->anim_tick >= 10) {
				e->state = ESTATE_DEAD;
				if (e->type == ENEMY_BOSS) {
					game.game_over = true;
					game.won       = true;
					doom_play_sound(snd_portal);
				}
			}
			break;
		case ESTATE_DEAD:
			break;
		}
	}
}

/* ========== GAME TICK (work handler) ========== */

static void game_tick_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!running) return;

	uint32_t input = (uint32_t)atomic_get(&input_state);

	if (game.game_over) {
		draw_game_over();
		doom_flush_fb();

		if (input & DINPUT_FIRE) {
			game_init();
			doom_sound_off();
		}

		k_work_reschedule(&game_tick_work, K_MSEC(100));
		return;
	}

	handle_movement(input);

	if (fire_cooldown > 0) {
		fire_cooldown--;
	} else if (input & DINPUT_FIRE) {
		handle_shooting(input);
		fire_cooldown = 4;
	}
	if (!(input & DINPUT_FIRE)) {
		game.player.firing = false;
	}

	update_enemies();
	update_fireballs();

	/* L1: spawn portal once both monsters are dead */
	check_level_complete();

	/* Portal: flicker and check if player stepped into it */
	if (game.portal_active) {
		game.portal_anim++;

		fixed_t pdx = fp_from_int(game.portal_map_x) + FP_HALF
			      - game.player.x;
		fixed_t pdy = fp_from_int(game.portal_map_y) + FP_HALF
			      - game.player.y;

		/* Trigger within ~0.7 tiles (squared distance < 0.5) */
		if (fp_mul(pdx, pdx) + fp_mul(pdy, pdy) < (FP_ONE / 2)) {
			advance_to_level2();
		}
	}

	memset(render_fb, 0, FB_SIZE);
	raycaster_render();
	draw_boss_bar();
	draw_gun(game.player.firing && fire_cooldown > 2);
	draw_hud();
	doom_flush_fb();

	game.frame_count++;

	k_work_reschedule(&game_tick_work, K_MSEC(FRAME_MS));
}

/* ========== PUBLIC API ========== */

void doom_game_start(void)
{
	if (running) return;

	LOG_INF("Doom easter egg starting!");

	const struct device *dev = mc_display_get_device();
	if (dev) {
		display_blanking_off(dev);
	}

#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_stop();
#endif

	game_init();
	running = true;

	k_work_init_delayable(&game_tick_work, game_tick_handler);
	k_work_init_delayable(&sound_work, doom_sound_work_handler);

	k_work_reschedule(&game_tick_work, K_NO_WAIT);
}

void doom_game_stop(void)
{
	if (!running) return;

	LOG_INF("Doom easter egg stopping");

	running = false;
	k_work_cancel_delayable(&game_tick_work);
	doom_sound_off();

	const struct device *dev = mc_display_get_device();
	if (dev) {
		memset(render_fb, 0, FB_SIZE);
		struct display_buffer_descriptor desc = {
			.buf_size = FB_SIZE,
			.width = SCREEN_W,
			.height = SCREEN_H,
			.pitch = SCREEN_W,
		};
		display_write(dev, 0, 0, &desc, render_fb);
	}
}

bool doom_game_is_running(void)
{
	return running;
}

void doom_game_input(uint16_t code, int32_t value)
{
	uint32_t bit = 0;

	switch (code) {
	case INPUT_KEY_UP:
		bit = DINPUT_FWD;
		break;
	case INPUT_KEY_DOWN:
		bit = DINPUT_BACK;
		break;
	case INPUT_KEY_LEFT:
		bit = DINPUT_RIGHT;
		break;
	case INPUT_KEY_RIGHT:
		bit = DINPUT_LEFT;
		break;
	case INPUT_KEY_ENTER:
		/* Not INPUT_KEY_0: on the Wio that is the user button, whose tap is
		 * the way out (KEY_1 via the multi-tap filter); firing on it too
		 * shot on every exit. */
		bit = DINPUT_FIRE;
		break;
	default:
		return;
	}

	if (value) {
		atomic_or(&input_state, bit);
	} else {
		atomic_and(&input_state, ~bit);
	}
}
