/*
 * Copyright (c) 2020 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led_strip.h>
#include <math.h>
#include <stdlib.h>

#include <drivers/ext_power.h>
#include <zmk/rgb_underglow.h>
#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/workqueue.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)
#error "A zmk,underglow chosen node must be declared"
#endif

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)
#define LED_INDEX_OFFSET ((STRIP_NUM_PIXELS == 27) ? 29 : 0)
#define HUE_MAX 360
#define SAT_MAX 100
#define BRT_MAX 100
#define MAX_HITS 8
#define NO_LED_INDEX 0xFF

BUILD_ASSERT(STRIP_NUM_PIXELS == 29 || STRIP_NUM_PIXELS == 27,
             "Charybdis RGB underglow requires 29 or 27 LEDs");
BUILD_ASSERT(CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN <= CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX,
             "RGB underglow maximum brightness is less than minimum brightness");

enum rgb_underglow_effect {
    UNDERGLOW_EFFECT_SOLID,
    UNDERGLOW_EFFECT_BREATHE,
    UNDERGLOW_EFFECT_SPECTRUM,
    UNDERGLOW_EFFECT_SWIRL,
    UNDERGLOW_EFFECT_DEFAULT_ANIM,
    UNDERGLOW_EFFECT_GAME_ANIM,
    UNDERGLOW_EFFECT_GAY_ANIM,
    UNDERGLOW_EFFECT_NUMBER
};

struct rgb_underglow_state {
    struct zmk_led_hsb color;
    uint8_t animation_speed;
    uint8_t current_effect;
    uint16_t animation_step;
    bool on;
};

struct rgb_point { int16_t x; int16_t y; };
struct rgb_hit { struct rgb_point point; uint16_t tick; };

static const struct device *led_strip;
static struct led_rgb pixels[STRIP_NUM_PIXELS];
static struct rgb_underglow_state state;
static struct { uint8_t count; struct rgb_hit hits[MAX_HITS]; } last_hit_tracker;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *const ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
#endif

static const struct rgb_point led_points[56] = {
    [0]={80,0}, [1]={80,12}, [2]={80,24}, [3]={80,36},
    [4]={64,36}, [5]={64,24}, [6]={64,12}, [7]={64,0},
    [8]={48,0}, [9]={48,12}, [10]={48,24}, [11]={48,36},
    [12]={32,36}, [13]={32,24}, [14]={32,12}, [15]={32,0},
    [16]={16,0}, [17]={16,12}, [18]={16,24}, [19]={16,36},
    [20]={0,36}, [21]={0,24}, [22]={0,12}, [23]={0,0},
    [24]={112,64}, [25]={96,64}, [26]={80,52}, [27]={112,52}, [28]={96,52},
    [29]={144,0}, [30]={144,12}, [31]={144,24}, [32]={144,36},
    [33]={160,36}, [34]={160,24}, [35]={160,12}, [36]={160,0},
    [37]={176,0}, [38]={176,12}, [39]={176,24}, [40]={176,36},
    [41]={192,36}, [42]={192,24}, [43]={192,12}, [44]={192,0},
    [45]={208,0}, [46]={208,12}, [47]={208,24}, [48]={208,36},
    [49]={224,36}, [50]={224,24}, [51]={224,12}, [52]={224,0},
    [53]={112,52}, [54]={128,52}, [55]={112,64},
};

static const uint8_t position_to_led[56] = {
    23,16,15,8,7,0, 52,45,44,37,36,29,
    22,17,14,9,6,1, 51,46,43,38,35,30,
    21,18,13,10,5,2, 50,47,42,39,34,31,
    20,19,12,11,4,3, 49,48,41,40,33,32,
    26,25,24, 53,54, 28,27, 55,
};

static struct zmk_led_hsb hsb_scale_min_max(struct zmk_led_hsb hsb) {
    hsb.b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN +
            (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX - CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN) * hsb.b / BRT_MAX;
    return hsb;
}

static struct zmk_led_hsb hsb_scale_zero_max(struct zmk_led_hsb hsb) {
    hsb.b = hsb.b * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / BRT_MAX;
    return hsb;
}

static struct led_rgb hsb_to_rgb(struct zmk_led_hsb hsb) {
    float r=0, g=0, b=0;
    uint8_t i = hsb.h / 60;
    float v = hsb.b / (float)BRT_MAX;
    float s = hsb.s / (float)SAT_MAX;
    float f = hsb.h / (float)HUE_MAX * 6 - i;
    float p = v * (1-s), q = v * (1-f*s), t = v * (1-(1-f)*s);
    switch (i % 6) {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    case 5: r=v; g=p; b=q; break;
    }
    return (struct led_rgb){.r=r*255, .g=g*255, .b=b*255};
}

static void rgb_underglow_clear_hits(void) {
    last_hit_tracker.count = 0;
    for (int i=0; i<MAX_HITS; i++) last_hit_tracker.hits[i] = (struct rgb_hit){0};
}

static void rgb_underglow_register_hit(uint32_t position) {
    if (position >= ARRAY_SIZE(position_to_led)) return;
    uint8_t global = position_to_led[position];
    if (global == NO_LED_INDEX || global >= 56) return;
    if (global < LED_INDEX_OFFSET || global >= LED_INDEX_OFFSET + STRIP_NUM_PIXELS) return;

    for (int i=MAX_HITS-1; i>0; i--) last_hit_tracker.hits[i] = last_hit_tracker.hits[i-1];
    last_hit_tracker.hits[0].point = led_points[global];
    last_hit_tracker.hits[0].tick = state.animation_step;
    if (last_hit_tracker.count < MAX_HITS) last_hit_tracker.count++;

    LOG_DBG("RGB hit: position=%u global_led=%u local_led=%u x=%d y=%d",
            position, global, global - LED_INDEX_OFFSET,
            led_points[global].x, led_points[global].y);
}

static void zmk_rgb_underglow_effect_default_core(bool red_mode, bool gay_mode) {
    uint16_t step = state.animation_step;

    for (int i=0; i<STRIP_NUM_PIXELS; i++) {
        int global = LED_INDEX_OFFSET + i;
        struct zmk_led_hsb hsb = state.color;
        uint16_t wave = 0;

        for (uint8_t j=0; j<last_hit_tracker.count; j++) {
            int16_t dx = led_points[global].x - last_hit_tracker.hits[j].point.x;
            int16_t dy = led_points[global].y - last_hit_tracker.hits[j].point.y;
            uint16_t distance = (uint16_t)sqrtf((float)(dx*dx + dy*dy));
            uint16_t age = (uint16_t)(step - last_hit_tracker.hits[j].tick);
            if (age > 255) continue;

            int effect = 255 - (int)age * 3 - (int)distance * 5;
            if (effect < 0) effect = 0;
            wave = MIN(255, wave + effect);
        }

        uint8_t base = (uint8_t)(hsb.b * 255 / BRT_MAX);
        uint8_t background = base / 4;
        uint8_t brightness = MAX(background, (uint8_t)wave);

        if (!gay_mode) {
            if (red_mode) { hsb.h = 0; hsb.s = SAT_MAX; }
            hsb.b = brightness * BRT_MAX / 255;
        } else {
            hsb.h = (uint16_t)(((led_points[global].x * HUE_MAX) / 224 + step) % HUE_MAX);
            hsb.s = MIN(SAT_MAX, hsb.s + wave / 4);
            hsb.b = brightness * BRT_MAX / 255;
        }

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed;
}

static void zmk_rgb_underglow_effect_solid(void) {
    struct led_rgb color = hsb_to_rgb(hsb_scale_min_max(state.color));
    for (int i=0; i<STRIP_NUM_PIXELS; i++) pixels[i] = color;
}

static void zmk_rgb_underglow_effect_breathe(void) {
    for (int i=0; i<STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.b = abs(state.animation_step - 1200) / 12;
        pixels[i] = hsb_to_rgb(hsb_scale_zero_max(hsb));
    }
    state.animation_step += state.animation_speed * 10;
    if (state.animation_step > 2400) state.animation_step = 0;
}

static void zmk_rgb_underglow_effect_spectrum(void) {
    for (int i=0; i<STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
    state.animation_step = (state.animation_step + state.animation_speed) % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl(void) {
    for (int i=0; i<STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (HUE_MAX / STRIP_NUM_PIXELS * i + state.animation_step) % HUE_MAX;
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
    state.animation_step = (state.animation_step + state.animation_speed * 2) % HUE_MAX;
}

static void zmk_rgb_underglow_tick(struct k_work *work) {
    ARG_UNUSED(work);
    switch (state.current_effect) {
    case UNDERGLOW_EFFECT_SOLID: zmk_rgb_underglow_effect_solid(); break;
    case UNDERGLOW_EFFECT_BREATHE: zmk_rgb_underglow_effect_breathe(); break;
    case UNDERGLOW_EFFECT_SPECTRUM: zmk_rgb_underglow_effect_spectrum(); break;
    case UNDERGLOW_EFFECT_SWIRL: zmk_rgb_underglow_effect_swirl(); break;
    case UNDERGLOW_EFFECT_DEFAULT_ANIM: zmk_rgb_underglow_effect_default_core(false, false); break;
    case UNDERGLOW_EFFECT_GAME_ANIM: zmk_rgb_underglow_effect_default_core(true, false); break;
    case UNDERGLOW_EFFECT_GAY_ANIM: zmk_rgb_underglow_effect_default_core(false, true); break;
    default: zmk_rgb_underglow_effect_solid(); break;
    }

    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) LOG_ERR("Failed to update RGB strip: %d", err);
}

K_WORK_DEFINE(underglow_tick_work, zmk_rgb_underglow_tick);

static void zmk_rgb_underglow_tick_handler(struct k_timer *timer) {
    ARG_UNUSED(timer);
    if (state.on) k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_tick_work);
}

K_TIMER_DEFINE(underglow_tick, zmk_rgb_underglow_tick_handler, NULL);

#if IS_ENABLED(CONFIG_SETTINGS)
static int rgb_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(state)) return -EINVAL;
        int rc = read_cb(cb_arg, &state, sizeof(state));
        if (rc >= 0) {
            if (state.on) k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
            return 0;
        }
        return rc;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(rgb_underglow, "rgb/underglow", NULL, rgb_settings_set, NULL, NULL);

static void zmk_rgb_underglow_save_state_work(struct k_work *work) {
    ARG_UNUSED(work);
    settings_save_one("rgb/underglow/state", &state, sizeof(state));
}

static struct k_work_delayable underglow_save_work;
#endif

static int zmk_rgb_underglow_init(void) {
    led_strip = DEVICE_DT_GET(STRIP_CHOSEN);
    if (!device_is_ready
