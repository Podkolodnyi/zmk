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
    if (!device_is_ready(led_strip)) return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (!device_is_ready(ext_power)) return -ENODEV;
#endif

    state = (struct rgb_underglow_state){
        .color = {
            .h = CONFIG_ZMK_RGB_UNDERGLOW_HUE_START,
            .s = CONFIG_ZMK_RGB_UNDERGLOW_SAT_START,
            .b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_START,
        },
        .animation_speed = CONFIG_ZMK_RGB_UNDERGLOW_SPD_START,
        .current_effect = CONFIG_ZMK_RGB_UNDERGLOW_EFF_START,
        .animation_step = 0,
        .on = IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START),
    };

    rgb_underglow_clear_hits();

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&underglow_save_work, zmk_rgb_underglow_save_state_work);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    state.on = zmk_usb_is_powered();
#endif

    if (state.on) k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
    return 0;
}

int zmk_rgb_underglow_save_state(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    int ret = k_work_reschedule(&underglow_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}

int zmk_rgb_underglow_get_state(bool *on_off) {
    if (!led_strip) return -ENODEV;
    *on_off = state.on;
    return 0;
}

int zmk_rgb_underglow_on(void) {
    if (!led_strip) return -ENODEV;
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    ext_power_enable(ext_power);
#endif
    state.on = true;
    state.animation_step = 0;
    k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
    return zmk_rgb_underglow_save_state();
}

static void zmk_rgb_underglow_off_handler(struct k_work *work) {
    ARG_UNUSED(work);
    for (int i=0; i<STRIP_NUM_PIXELS; i++) pixels[i] = (struct led_rgb){0};
    led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
}

K_WORK_DEFINE(underglow_off_work, zmk_rgb_underglow_off_handler);

int zmk_rgb_underglow_off(void) {
    if (!led_strip) return -ENODEV;
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    ext_power_disable(ext_power);
#endif
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_off_work);
    k_timer_stop(&underglow_tick);
    state.on = false;
    rgb_underglow_clear_hits();
    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_calc_effect(int direction) {
    return (state.current_effect + UNDERGLOW_EFFECT_NUMBER + direction) % UNDERGLOW_EFFECT_NUMBER;
}

int zmk_rgb_underglow_select_effect(int effect) {
    if (!led_strip) return -ENODEV;
    if (effect < 0 || effect >= UNDERGLOW_EFFECT_NUMBER) return -EINVAL;
    state.current_effect = effect;
    state.animation_step = 0;
    rgb_underglow_clear_hits();
    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_cycle_effect(int direction) {
    return zmk_rgb_underglow_select_effect(zmk_rgb_underglow_calc_effect(direction));
}

int zmk_rgb_underglow_toggle(void) {
    return state.on ? zmk_rgb_underglow_off() : zmk_rgb_underglow_on();
}

int zmk_rgb_underglow_set_hsb(struct zmk_led_hsb color) {
    if (color.h > HUE_MAX || color.s > SAT_MAX || color.b > BRT_MAX) return -ENOTSUP;
    state.color = color;
    return 0;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_hue(int direction) {
    struct zmk_led_hsb color = state.color;
    color.h = (color.h + HUE_MAX + direction * CONFIG_ZMK_RGB_UNDERGLOW_HUE_STEP) % HUE_MAX;
    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_sat(int direction) {
    struct zmk_led_hsb color = state.color;
    color.s = CLAMP(color.s + direction * CONFIG_ZMK_RGB_UNDERGLOW_SAT_STEP, 0, SAT_MAX);
    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_brt(int direction) {
    struct zmk_led_hsb color = state.color;
    color.b = CLAMP(color.b + direction * CONFIG_ZMK_RGB_UNDERGLOW_BRT_STEP, 0, BRT_MAX);
    return color;
}

int zmk_rgb_underglow_change_hue(int direction) {
    if (!led_strip) return -ENODEV;
    state.color = zmk_rgb_underglow_calc_hue(direction);
    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_sat(int direction) {
    if (!led_strip) return -ENODEV;
    state.color = zmk_rgb_underglow_calc_sat(direction);
    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_brt(int direction) {
    if (!led_strip) return -ENODEV;
    state.color = zmk_rgb_underglow_calc_brt(direction);
    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_spd(int direction) {
    if (!led_strip) return -ENODEV;
    if (state.animation_speed == 1 && direction < 0) return 0;
    state.animation_speed += direction;
    if (state.animation_speed > 5) state.animation_speed = 5;
    return zmk_rgb_underglow_save_state();
}

static int rgb_underglow_key_hit_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || !ev->state) return ZMK_EV_EVENT_BUBBLE;
    rgb_underglow_register_hit(ev->position);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(rgb_underglow_key_hit, rgb_underglow_key_hit_listener);
ZMK_SUBSCRIPTION(rgb_underglow_key_hit, zmk_position_state_changed);

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) || IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
struct rgb_underglow_sleep_state { bool is_awake; bool rgb_state_before_sleeping; };

static int rgb_underglow_auto_state(bool wake) {
    static struct rgb_underglow_sleep_state sleep_state = {
        .is_awake = true,
        .rgb_state_before_sleeping = false,
    };

    if (wake == sleep_state.is_awake) return 0;
    sleep_state.is_awake = wake;

    if (wake) {
        return sleep_state.rgb_state_before_sleeping ?
            zmk_rgb_underglow_on() : zmk_rgb_underglow_off();
    }

    sleep_state.rgb_state_before_sleeping = state.on;
    return zmk_rgb_underglow_off();
}

static int rgb_underglow_event_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
    if (as_zmk_activity_state_changed(eh))
        return rgb_underglow_auto_state(zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
#endif
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    if (as_zmk_usb_conn_state_changed(eh))
        return rgb_underglow_auto_state(zmk_usb_is_powered());
#endif
    return -ENOTSUP;
}

ZMK_LISTENER(rgb_underglow, rgb_underglow_event_listener);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_activity_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_usb_conn_state_changed);
#endif

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
