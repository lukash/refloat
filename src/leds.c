// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "leds.h"

#include "conf/datatypes.h"
#include "led_driver.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Brightness change rate
#define BR_RATE (3.0f / LEDS_REFRESH_RATE)
// Footpad Sensor change rate
#define FS_RATE (10.0f / LEDS_REFRESH_RATE)
// Status Bar change rate
#define SB_RATE (5.0f / LEDS_REFRESH_RATE)

#define R(c) ((uint8_t) ((c) >> 16))
#define G(c) ((uint8_t) ((c) >> 8))
#define B(c) ((uint8_t) (c))
#define W(c) ((uint8_t) ((c) >> 24))
#define RGB(r, g, b) ((r) << 16 | (g) << 8 | (b))
#define RGBW(r, g, b, w) ((w) << 24 | (r) << 16 | (g) << 8 | (b))

static const uint32_t colors[] = {
    0x00000000,  // BLACK
    0xFFFFFFFF,  // WHITE_FULL
    0x00FFFFFF,  // WHITE_RGB
    0xFF000000,  // WHITE_SINGLE
    0x00FF0000,  // RED
    0x00FF3800,  // FERRARI
    0x00FF5000,  // FLAME
    0x00FF6040,  // CORAL
    0x00FF7830,  // SUNSET
    0x00FF9040,  // SUNRISE
    0x00FF8020,  // GOLD
    0x00FF7800,  // ORANGE
    0x00FFA000,  // YELLOW
    0x00FFB040,  // BANANA
    0x00FFFF00,  // LIME (yes, that's what it looks like on my SK6812)
    0x00A0FF00,  // ACID
    0x00A0FF50,  // SAGE
    0x0000FF00,  // GREEN
    0x0000FF50,  // MINT
    0x0000FFC0,  // TIFFANY
    0x0000FFFF,  // CYAN
    0x0090C0FF,  // STEEL
    0x0070D0FF,  // SKY
    0x0000A0FF,  // AZURE
    0x000070FF,  // SAPPHIRE
    0x000000FF,  // BLUE
    0x008000FF,  // VIOLET
    0x00A060FF,  // AMETHYST
    0x00FF00FF,  // MAGENTA
    0x00FF00C0,  // PINK
    0x00FF0070,  // FUCHSIA
    0x00FF70A0,  // LAVENDER
};

#define BATTERY_COLOR 0x00909090
#define DUTY_COLOR 0x00FFB030
#define FOOTPAD_SENSOR_COLOR 0x0000C0FF
#define RED_BAR_COLOR 0x00FF3828
#define BATTERY10_BAR_COLOR 0x00FF5038

static uint32_t color_blend(uint32_t color1, uint32_t color2, float blend) {
    if (blend <= 0.0f) {
        return color1;
    } else if (blend >= 1.0f) {
        return color2;
    }

    float blend1 = 1.0f - blend;

    uint8_t r = R(color1) * blend1 + R(color2) * blend;
    uint8_t g = G(color1) * blend1 + G(color2) * blend;
    uint8_t b = B(color1) * blend1 + B(color2) * blend;
    uint8_t w = W(color1) * blend1 + W(color2) * blend;

    return RGBW(r, g, b, w);
}

// Borrowed from WLED
static uint32_t color_wheel(uint8_t pos) {
    pos = 255 - pos;
    if (pos < 85) {
        return ((uint32_t) (255 - pos * 3) << 16) | ((uint32_t) 0 << 8) | (pos * 3);
    } else if (pos < 170) {
        pos -= 85;
        return ((uint32_t) 0 << 16) | ((uint32_t) (pos * 3) << 8) | (255 - pos * 3);
    } else {
        pos -= 170;
        return ((uint32_t) (pos * 3) << 16) | ((uint32_t) (255 - pos * 3) << 8) | 0;
    }
}

static void sattolo_shuffle(uint32_t seed, uint8_t *array, uint8_t length) {
    for (int16_t i = length - 1; i > 0; --i) {
        uint8_t j = rnd(seed + i) % i;
        uint8_t t = array[i];
        array[i] = array[j];
        array[j] = t;
    }
}

static void front_rear_animation_reset(Leds *leds, float current_time) {
    leds->animation_start = current_time;
}

static void full_animation_reset(Leds *leds, float current_time) {
    front_rear_animation_reset(leds, current_time);
    leds->status_idle_time = current_time;
    leds->status_on_front_idle_time = current_time;
}

static void led_set_color(
    Leds *leds, const LedStrip *strip, uint8_t i, uint32_t color, float brightness, float blend
) {
    if (blend <= 0.0f) {
        return;
    }

    uint8_t led = i;
    if (strip->reverse) {
        led = strip->length - i - 1;
    }
    led += strip->start;

    if (led >= leds->led_count) {
        return;
    }

    float br = brightness * leds->on_off_fade;

    uint8_t r = R(color) * br + 0.5f;
    uint8_t g = G(color) * br + 0.5f;
    uint8_t b = B(color) * br + 0.5f;
    uint8_t w = W(color) * br + 0.5f;

    if (blend < 1.0f) {
        uint32_t orig_color = leds->led_data[led];
        float orig_blend = 1.0f - blend;

        r = r * blend + R(orig_color) * orig_blend;
        g = g * blend + G(orig_color) * orig_blend;
        b = b * blend + B(orig_color) * orig_blend;
        w = w * blend + W(orig_color) * orig_blend;
    }

    leds->led_data[led] = RGBW(r, g, b, w);
}

static void strip_set_color(
    Leds *leds, const LedStrip *strip, uint32_t color, float brightness, float blend
) {
    for (uint8_t i = 0; i < strip->length; ++i) {
        led_set_color(leds, strip, i, color, brightness, blend);
    }
}

// Returns a cosine wave oscillating from 0 to 1, starting at 0, with a period of 2s.
static float cosine_progress(float time) {
    uint32_t rounded = lroundf(time);
    float x = (time - rounded) * M_PI;
    // Bhaskara cosine approximation, optimized to return: (1 - cos(x)) / 2
    float cos = 10 * x * x / (4 * x * x + 4 * M_PI * M_PI);
    if (rounded % 2 == 1) {
        cos = 1 - cos;
    }
    return cos;
}

static void anim_fade(Leds *leds, const LedStrip *strip, const LedBar *bar, float time) {
    float p = cosine_progress(time);
    uint32_t color = color_blend(colors[bar->color2], colors[bar->color1], p);
    strip_set_color(leds, strip, color, strip->brightness, 1.0f);
}

static void anim_strobe(Leds *leds, const LedStrip *strip, const LedBar *bar, float time) {
    uint32_t color = fmodf(time, 2.0f) >= 1.0f ? colors[bar->color2] : colors[bar->color1];
    strip_set_color(leds, strip, color, strip->brightness, 1.0f);
}

static void anim_pulse(
    Leds *leds, const LedStrip *strip, const LedBar *bar, float time, float center
) {
    float p = cosine_progress(time);

    float length = strip->length / 2.0f - center;
    float offset = length * (1.0f - p);
    float feather = strip->length / 4.0f;

    float fade = 1.0f;
    float ratio = center / length;
    if (time < ratio) {
        fade = time / ratio;
    }

    for (uint8_t i = 0; i < strip->length; ++i) {
        float dist1 = i - offset + 1.0f;
        float dist2 = strip->length - offset - i;
        float k1 = clampf(dist1 / feather, 0.0f, 1.0f);
        float k2 = clampf(dist2 / feather, 0.0f, 1.0f);

        uint32_t color =
            color_blend(colors[bar->color2], colors[bar->color1], fminf(k1, k2) * fade);
        led_set_color(leds, strip, i, color, strip->brightness, 1.0f);
    }
}

static void anim_knight_rider(Leds *leds, const LedStrip *strip, const LedBar *bar, float time) {
    const uint8_t tail = strip->length / 3 + 1;

    time *= 0.7f;
    float backlight = time > 0.3f ? 0.08f : 0.0f;
    float x1 = strip->length * fmodf(time, 2.0f) - 0.5f * strip->length - 1.0f;
    float x2 = 1.5f * strip->length - strip->length * fmodf(time - 1.0f, 2.0f);

    for (uint8_t i = 0; i < strip->length; ++i) {
        float k1 = backlight;
        float dist1 = fabsf(x1 - i);
        if (i <= x1) {
            if (dist1 <= tail) {
                k1 = (tail - dist1) / tail;
            }
        } else if (i < x1 + 1) {
            k1 = x1 - floorf(x1);
        }

        float k2 = backlight;
        float dist2 = fabsf(x2 - i);
        if (i >= x2) {
            if (dist2 <= tail) {
                k2 = (tail - dist2) / tail;
            }
        } else if (i > x2 - 1) {
            k2 = 1 - x2 + floorf(x2);
        }

        uint32_t color = color_blend(colors[bar->color2], colors[bar->color1], fmaxf(k1, k2));
        led_set_color(leds, strip, i, color, strip->brightness, 1.0f);
    }
}

static void led_strip_animate(Leds *leds, const LedStrip *strip, const LedBar *bar, float time) {
    time *= bar->speed;

    switch (bar->mode) {
    case LED_MODE_SOLID:
        strip_set_color(leds, strip, colors[bar->color1], strip->brightness, 1.0f);
        break;
    case LED_MODE_FADE:
        anim_fade(leds, strip, bar, time);
        break;
    case LED_MODE_PULSE:
        anim_pulse(leds, strip, bar, time, strip->length / 5.0f);
        break;
    case LED_MODE_STROBE:
        anim_strobe(leds, strip, bar, time);
        break;
    case LED_MODE_KNIGHT_RIDER:
        anim_knight_rider(leds, strip, bar, time);
        break;
    }
}

static void anim_fs_state(Leds *leds, const LedStrip *strip, bool reverse, float blend) {
    uint8_t offset = (strip->length + 1) / 2 - 1;
    uint8_t right_offset = strip->length - offset - 1;

    // need to reverse for displaying on the front bar
    float left_sensor = reverse ? leds->right_sensor : leds->left_sensor;
    float right_sensor = reverse ? leds->left_sensor : leds->right_sensor;

    for (uint8_t i = 0; i < strip->length; ++i) {
        uint32_t color = 0;
        float dim = 0.0f;

        if (left_sensor > 0.0f && i <= offset) {
            color = FOOTPAD_SENSOR_COLOR;
            if (i == offset) {
                dim = 0.6f * left_sensor;
            } else {
                dim = left_sensor;
            }
        }

        if (right_sensor > 0.0f && i >= right_offset) {
            color = FOOTPAD_SENSOR_COLOR;
            if (i == right_offset) {
                dim = fmaxf(dim, 0.6f * right_sensor);
            } else {
                dim = right_sensor;
            }
        }

        blend = fminf(blend, fmaxf(left_sensor, right_sensor));
        led_set_color(leds, strip, i, color, strip->brightness * dim, blend);
    }
}

static void anim_progress_bar(
    Leds *leds,
    const LedStrip *strip,
    float value,
    uint32_t color,
    bool color_end,
    bool reverse,
    float blend
) {
    float progress = strip->length * value;
    uint8_t offset = progress;
    // last tick at proportional brightness, need to lower it, otherwise it's hard to distinguish
    float remaining = (progress - floorf(progress)) * 0.7f;

    uint8_t red_offset = 0;
    uint8_t red_led_nr = roundf(strip->length * leds->cfg->status.red_bar_percentage);
    if (color_end) {
        red_offset = strip->length - red_led_nr;
    } else {
        if (ceilf(progress) <= red_led_nr) {
            color = RED_BAR_COLOR;
        }
    }

    for (uint8_t i = 0; i < strip->length; ++i) {
        uint32_t col = 0;
        float dim = 1.0f;
        if (i <= offset) {
            if (color_end && i >= red_offset) {
                col = RED_BAR_COLOR;
            } else {
                col = color;
            }
            if (i == offset) {
                dim = remaining;
            }
        }

        uint8_t led = reverse ? strip->length - i - 1 : i;
        led_set_color(leds, strip, led, col, strip->brightness * dim, blend);
    }
}

static void anim_disabled(Leds *leds, const LedStrip *strip, float time) {
    LedBar disabled_bar = {
        .brightness = 0.0f,
        .color1 = COLOR_RED,
        .color2 = COLOR_BLACK,
        .mode = LED_MODE_PULSE,
        .speed = 0.0f
    };
    anim_pulse(leds, strip, &disabled_bar, time / 2.0f, strip->length / 3.0f);
}

static void status_animate(
    Leds *leds, const LedStrip *strip, float current_time, float blend, float idle_blend
) {
    if (fabsf(VESC_IF->mc_get_rpm()) > ERPM_MOVING_THRESHOLD) {
        leds->status_idle_time = current_time;
    }

    float duty = 0;
    if (leds->state.state == STATE_RUNNING && leds->state.mode != MODE_FLYWHEEL) {
        duty = fminf(fabsf(VESC_IF->mc_get_duty_cycle_now() * 10.0f / 9.0f), 1.0f);
    }

    if (duty > leds->duty_threshold) {
        rate_limitf(&leds->status_duty_blend, 1.0f, SB_RATE);
    } else if (duty < leds->duty_threshold - 0.1f) {  // 10 percent hysteresis
        rate_limitf(&leds->status_duty_blend, 0.0f, SB_RATE);
    }

    if (idle_blend > 0.0f) {
        if (strip == &leds->front_strip) {
            strip_set_color(leds, strip, 0x00000000, strip->brightness, blend);
        } else {
            led_strip_animate(
                leds,
                &leds->status_strip,
                &leds->cfg->status_idle,
                current_time - leds->status_animation_start
            );
        }
    }

    if (idle_blend < 1.0f) {
        bool reverse = strip == &leds->front_strip;
        if (leds->status_duty_blend < 1.0f) {
            float battery = VESC_IF->mc_get_battery_level(NULL);
            anim_progress_bar(
                leds, strip, battery, BATTERY_COLOR, false, reverse, fminf(blend, 1.0f - idle_blend)
            );
        }

        if (leds->status_duty_blend > 0.0f) {
            anim_progress_bar(
                leds, strip, duty, DUTY_COLOR, true, reverse, leds->status_duty_blend
            );
        }

        if (leds->left_sensor > 0.0f || leds->right_sensor > 0.0f) {
            anim_fs_state(leds, strip, reverse, blend);
        }
    }
}

static void transition_reset(const Leds *leds, TransitionState *trans, LedStrip *strip) {
    switch (trans->transition) {
    case LED_TRANS_CIPHER:
    case LED_TRANS_MONO_CIPHER: {
        CipherData *data = &strip->trans_data.cipher;

        for (uint8_t i = 0; i < strip->length; ++i) {
            data->map[i] = i;
            data->map[i + strip->length] = i;
        }

        sattolo_shuffle(leds->animation_start, data->map, strip->length);
        sattolo_shuffle(leds->animation_start, data->map + strip->length, strip->length);
        break;
    }
    default:
        break;
    }
}

static uint32_t led_bar_to_color(const LedBar *bar) {
    return bar->mode == LED_MODE_SOLID ? bar->color1 : bar->color2;
}

static void trans_fade(Leds *leds, const LedStrip *strip, float progress, const LedBar *to_bar) {
    uint32_t to_color = colors[led_bar_to_color(to_bar)];
    float prog = (progress + 1.0f) / 2.0f;
    float brightness = strip->brightness + (to_bar->brightness - strip->brightness) * prog;
    strip_set_color(leds, strip, to_color, brightness, prog);
}

static void trans_fade_out_in(
    Leds *leds, const LedStrip *strip, float progress, const LedBar *to_bar
) {
    if (progress <= 0.0f) {
        float prog = progress + 1.0f;
        float brightness = strip->brightness + (to_bar->brightness - strip->brightness) * prog;
        strip_set_color(leds, strip, 0x00000000, brightness, prog);
    } else {
        uint32_t to_color = color_blend(0x00000000, colors[led_bar_to_color(to_bar)], progress);
        float brightness = strip->brightness + (to_bar->brightness - strip->brightness) * progress;
        strip_set_color(leds, strip, to_color, brightness, 1.0f);
    }
}

static void trans_cipher(
    Leds *leds,
    const LedStrip *strip,
    float progress,
    const LedBar *from_bar,
    const LedBar *to_bar,
    bool mono
) {
    const CipherData *data = &strip->trans_data.cipher;
    int8_t prog = progress * strip->length;

    uint32_t to_color = colors[led_bar_to_color(to_bar)];
    float mid_brightness = (strip->brightness + to_bar->brightness) / 2.0f;

    for (int8_t i = 1 - strip->length; i <= prog; ++i) {
        if (i <= 0) {
            uint8_t j = -i;
            uint8_t target_j = data->map[j];
            uint8_t r = rnd(j + target_j) % 256;
            uint32_t color;

            if (rnd(j + target_j + 17) % 8 < 3) {
                // make some percentage of LEDs black
                color = 0x00000000;
            } else {
                if (mono) {
                    color = color_blend(colors[from_bar->color1], to_color, r / 256.0f);
                } else {
                    // random fade to white
                    uint8_t wf = rnd(j + target_j + 23) % 128 + 80;
                    color = color_wheel(r) | RGB(wf, wf, wf);
                }
            }

            led_set_color(leds, strip, target_j, color, mid_brightness, 1.0f);
        } else {
            led_set_color(
                leds, strip, data->map[i - 1 + strip->length], to_color, to_bar->brightness, 1.0f
            );
        }
    }
}

static void led_strip_transition(
    Leds *leds,
    TransitionState *trans,
    const LedStrip *strip,
    const LedBar *from_bar,
    const LedBar *to_bar,
    bool reverse
) {
    float progress = reverse ? -trans->split : trans->split;

    switch (trans->transition) {
    case LED_TRANS_FADE:
        trans_fade(leds, strip, progress, to_bar);
        break;
    case LED_TRANS_FADE_OUT_IN:
        trans_fade_out_in(leds, strip, progress, to_bar);
        break;
    case LED_TRANS_CIPHER:
        trans_cipher(leds, strip, progress, from_bar, to_bar, false);
        break;
    case LED_TRANS_MONO_CIPHER:
        trans_cipher(leds, strip, progress, from_bar, to_bar, true);
        break;
    }
}

static void reset_led_bars(
    Leds *leds, const LedBar *front, const LedBar *rear, float current_time
) {
    leds->front_bar = front;
    leds->rear_bar = rear;
    leds->front_strip.brightness = front->brightness;
    leds->rear_strip.brightness = rear->brightness;
    front_rear_animation_reset(leds, current_time);
}

static bool headlights_should_be_on(const Leds *leds) {
    return (leds->state.state == STATE_RUNNING && leds->state.mode != MODE_FLYWHEEL) &&
        leds->cfg->headlights_on;
}

static const LedBar *target_bar(const Leds *leds, bool flip) {
    bool headlights = headlights_should_be_on(leds);
    if (leds->direction_forward) {
        if (headlights) {
            if (flip) {
                return &leds->cfg->taillights;
            } else {
                return &leds->cfg->headlights;
            }
        } else {
            if (flip) {
                return &leds->cfg->rear;
            } else {
                return &leds->cfg->front;
            }
        }
    } else {
        if (headlights) {
            if (flip) {
                return &leds->cfg->headlights;
            } else {
                return &leds->cfg->taillights;
            }
        } else {
            if (flip) {
                return &leds->cfg->rear;
            } else {
                return &leds->cfg->front;
            }
        }
    }
}

bool leds_init(Leds *leds, CfgHwLeds *hw_cfg, const CfgLeds *cfg, FootpadSensorState fs_state) {
    leds->status_strip.start = 0;
    leds->status_strip.length = hw_cfg->status.count;
    leds->status_strip.reverse = hw_cfg->status.reverse;
    if (cfg->headlights_on) {
        leds->status_strip.brightness = cfg->status.brightness_headlights_on;
    } else {
        leds->status_strip.brightness = cfg->status.brightness_headlights_off;
    }
    leds->front_strip.start = hw_cfg->status.count;
    leds->front_strip.length = hw_cfg->front.count;
    leds->front_strip.reverse = hw_cfg->front.reverse;
    leds->front_strip.brightness = cfg->front.brightness;
    leds->rear_strip.start = hw_cfg->status.count + hw_cfg->front.count;
    leds->rear_strip.length = hw_cfg->rear.count;
    leds->rear_strip.reverse = hw_cfg->rear.reverse;
    leds->rear_strip.brightness = cfg->rear.brightness;

    leds->cfg = cfg;

    leds->last_updated = 0.0f;
    state_init(&leds->state, false);
    leds->pitch = 0.0f;

    leds->left_sensor = 0.0f;
    leds->right_sensor = 0.0f;

    leds->on_off_fade = 0.0f;

    leds->duty_threshold = 0.0f;
    leds->status_duty_blend = 0.0f;
    leds->status_idle_blend = 0.0f;
    leds->status_idle_time = 0.0f;
    leds->status_animation_start = 0.0f;

    leds->status_on_front_blend = 0.0f;
    leds->status_on_front_idle_blend = 0.0f;
    leds->status_on_front_idle_time = 0.0f;
    leds->board_is_upright = false;

    leds->split_distance = 0.0f;
    leds->headlights_on = false;
    leds->direction_forward = true;
    leds->headlights_time = 0.0f;
    leds->animation_start = 0;

    leds->headlights_trans.transition = LED_TRANS_FADE;
    leds->headlights_trans.split = 1.0f;
    leds->dir_trans.transition = LED_TRANS_FADE;
    leds->dir_trans.split = 1.0f;
    front_rear_animation_reset(leds, 0);

    leds->front_bar = &cfg->front;
    leds->front_dir_target = &cfg->front;
    leds->front_time_target = &cfg->front;

    leds->rear_bar = &cfg->rear;
    leds->rear_dir_target = &cfg->rear;
    leds->rear_time_target = &cfg->rear;

    leds->led_count = leds->rear_strip.start + leds->rear_strip.length;

    bool driver_init = true;
    if (fs_state == FS_BOTH) {
        log_msg("Both sensors pressed, not initializing LEDs.");
        driver_init = false;
    }

    if (hw_cfg->front.count + hw_cfg->rear.count > LEDS_FRONT_AND_REAR_COUNT_MAX) {
        log_error("Front and rear LED counts exceed maximum.");
        driver_init = false;
    }

    if (driver_init) {
        driver_init = led_driver_init(
            &leds->led_driver, hw_cfg->pin, hw_cfg->type, hw_cfg->color_order, leds->led_count
        );
    }

    if (!driver_init) {
        leds->led_data = NULL;
        leds->led_count = 0;
        return false;
    }

    leds->led_data = VESC_IF->malloc(sizeof(uint32_t) * leds->led_count);
    if (!leds->led_data) {
        log_error("Failed to init LED data, out of memory.");
        led_driver_destroy(&leds->led_driver);
        return false;
    }

    memset(leds->led_data, 0, sizeof(uint32_t) * leds->led_count);

    leds_configure(leds, cfg);

    return true;
}

void leds_configure(Leds *leds, const CfgLeds *cfg) {
    leds->duty_threshold = fmaxf(cfg->status.duty_threshold, 0.15);

    leds->headlights_trans.transition = cfg->headlights_transition;
    leds->dir_trans.transition = cfg->direction_transition;

    float current_time = VESC_IF->system_time();
    leds->status_idle_time = current_time;
    leds->status_on_front_idle_time = current_time;
}

void leds_update(Leds *leds, const State *state, FootpadSensorState fs_state) {
    if (!leds->led_data) {
        return;
    }

    float current_time = VESC_IF->system_time();
    leds->last_updated = current_time;
    RunState old_state = leds->state.state;
    leds->state = *state;

    if (leds->state.state == STATE_STARTUP) {
        return;
    }

    if (leds->cfg->on) {
        if (leds->on_off_fade == 0.0f) {
            full_animation_reset(leds, current_time);
        }
        rate_limitf(&leds->on_off_fade, 1.0f, BR_RATE);
    } else {
        if (leds->on_off_fade == 0.0f) {
            return;
        }
        rate_limitf(&leds->on_off_fade, 0.0f, BR_RATE);
    }

    leds->pitch = rad2deg(VESC_IF->imu_get_pitch());

    if (fs_state != FS_NONE) {
        leds->status_idle_time = current_time;
        leds->status_on_front_idle_time = current_time;
    }

    if (!leds->board_is_upright && leds->pitch > 60) {
        leds->board_is_upright = true;
        leds->status_idle_time = current_time;
        leds->status_on_front_idle_time = current_time;
    } else if (leds->board_is_upright && leds->pitch < 50) {
        leds->board_is_upright = false;
        leds->status_idle_time = current_time;
        if (leds->cfg->lights_off_when_lifted) {
            front_rear_animation_reset(leds, current_time);
        }
    }

    bool status_on_front = leds->cfg->status_on_front_when_lifted &&
        leds->state.state == STATE_READY && leds->board_is_upright;

    if (leds->state.state != old_state) {
        if (old_state == STATE_STARTUP) {
            full_animation_reset(leds, current_time);
            if (leds->board_is_upright) {
                leds->status_on_front_blend = 1.0f;
            }
        } else if (old_state == STATE_DISABLED) {
            leds->on_off_fade = 0.0f;
        }

        if (leds->state.state == STATE_RUNNING && leds->headlights_time <= 0.0f) {
            if (leds->pitch < 0) {
                leds->dir_trans.split = -1.0f;
                leds->direction_forward = false;
            } else {
                leds->dir_trans.split = 1.0f;
                leds->direction_forward = true;
            }
        } else if (leds->state.state == STATE_DISABLED) {
            full_animation_reset(leds, current_time);
            leds->on_off_fade = 0.0f;
        }
    }

    // status brightness
    float status_brightness = leds->cfg->status.brightness_headlights_off;
    if (leds->cfg->headlights_on) {
        status_brightness = leds->cfg->status.brightness_headlights_on;
    }
    if (leds->status_idle_blend > 0.0f) {
        status_brightness = fminf(status_brightness, leds->cfg->status_idle.brightness);
    }
    rate_limitf(&leds->status_strip.brightness, status_brightness, BR_RATE);

    // front brightness
    if (status_on_front) {
        rate_limitf(&leds->front_strip.brightness, status_brightness, BR_RATE);
    } else {
        if (leds->board_is_upright && leds->cfg->lights_off_when_lifted) {
            rate_limitf(&leds->front_strip.brightness, 0.0f, BR_RATE);
        } else {
            rate_limitf(&leds->front_strip.brightness, leds->front_bar->brightness, BR_RATE);
        }
    }

    // rear brightness
    if (leds->board_is_upright && leds->cfg->lights_off_when_lifted) {
        rate_limitf(&leds->rear_strip.brightness, 0.0f, BR_RATE);
    } else {
        rate_limitf(&leds->rear_strip.brightness, leds->rear_bar->brightness, BR_RATE);
    }

    rate_limitf(&leds->status_on_front_blend, status_on_front ? 1.0f : 0.0f, BR_RATE);

    if (leds->state.state == STATE_DISABLED) {
        anim_disabled(leds, &leds->front_strip, current_time);
        anim_disabled(leds, &leds->rear_strip, current_time);
        anim_disabled(leds, &leds->status_strip, current_time);
        led_driver_paint(&leds->led_driver, leds->led_data, leds->led_count);
        return;
    }

    // footpad sensor indicator animation
    if (!leds->cfg->status.show_sensors_while_running && leds->state.state == STATE_RUNNING) {
        rate_limitf(&leds->left_sensor, 0.0f, FS_RATE);
        rate_limitf(&leds->right_sensor, 0.0f, FS_RATE);
    } else {
        if ((leds->state.state != STATE_RUNNING && fs_state & FS_LEFT) || fs_state == FS_LEFT) {
            rate_limitf(&leds->left_sensor, 1.0f, FS_RATE);
            // reset idle blend so that the idle animation doesn't pop back up after a short press
            if (leds->left_sensor >= 1.0f) {
                leds->status_idle_blend = 0.0f;
            }
        } else {
            rate_limitf(&leds->left_sensor, 0.0f, FS_RATE);
        }

        if ((leds->state.state != STATE_RUNNING && fs_state & FS_RIGHT) || fs_state == FS_RIGHT) {
            rate_limitf(&leds->right_sensor, 1.0f, FS_RATE);
            // reset idle blend so that the idle animation doesn't pop back up after a short press
            if (leds->right_sensor >= 1.0f) {
                leds->status_idle_blend = 0.0f;
            }
        } else {
            rate_limitf(&leds->right_sensor, 0.0f, FS_RATE);
        }
    }

    led_strip_animate(
        leds, &leds->front_strip, leds->front_bar, current_time - leds->animation_start
    );
    led_strip_animate(
        leds, &leds->rear_strip, leds->rear_bar, current_time - leds->animation_start
    );

    // headlights transition from off to on or vice versa
    bool headlights_should = headlights_should_be_on(leds);
    float hl_split = leds->headlights_trans.split;
    if (headlights_should != leds->headlights_on && leds->headlights_time <= 0.0f) {
        // when turning headlights on when running, reset the split to one or the other side
        if (leds->state.state == STATE_RUNNING && headlights_should) {
            if (leds->dir_trans.split > 0.0f) {
                leds->dir_trans.split = 1.0f;
                leds->direction_forward = true;
            } else {
                leds->dir_trans.split = -1.0f;
                leds->direction_forward = false;
            }
        }
        leds->front_time_target = target_bar(leds, false);
        leds->rear_time_target = target_bar(leds, true);
        transition_reset(leds, &leds->headlights_trans, &leds->front_strip);
        transition_reset(leds, &leds->headlights_trans, &leds->rear_strip);
        leds->headlights_trans.split = hl_split = -1.0f;
        leds->headlights_time = current_time;
    } else if (leds->headlights_time > 0.0f) {
        // transition split is in range of [-1, 1], we want a 1 second transition
        float time_diff = (current_time - leds->headlights_time) * 2;
        if (headlights_should == leds->headlights_on) {
            time_diff = -time_diff;
        }
        hl_split = leds->headlights_trans.split = clampf(hl_split + time_diff, -1.0f, 1.0f);
        leds->headlights_time = current_time;
    }

    float old_split = leds->dir_trans.split;
    float split = leds->dir_trans.split;
    if (leds->state.state == STATE_RUNNING && leds->headlights_time <= 0.0f) {
        float distance = VESC_IF->mc_get_distance();
        float distance_diff = distance - leds->split_distance;
        if (leds->state.darkride) {
            distance_diff = -distance_diff;
        }
        split = clampf(split + distance_diff * 2, -1.0f, 1.0f);
        leds->split_distance = distance;
        leds->dir_trans.split = split;
    }

    // direction transition
    if ((fabsf(split) < 1.0f || split != old_split) && leds->headlights_on) {
        if (fabsf(old_split) >= 1.0f) {
            transition_reset(leds, &leds->dir_trans, &leds->front_strip);
            transition_reset(leds, &leds->dir_trans, &leds->rear_strip);
            leds->front_dir_target = target_bar(leds, true);
            leds->rear_dir_target = target_bar(leds, false);
        }

        if (leds->direction_forward) {
            // transitioning to forward on the front strip
            led_strip_transition(
                leds,
                &leds->dir_trans,
                &leds->front_strip,
                leds->front_bar,
                leds->front_dir_target,
                true
            );
            led_strip_transition(
                leds,
                &leds->dir_trans,
                &leds->rear_strip,
                leds->rear_bar,
                leds->rear_dir_target,
                true
            );
        } else {
            // transitioning to backwards on the front strip
            led_strip_transition(
                leds,
                &leds->dir_trans,
                &leds->front_strip,
                leds->front_bar,
                leds->front_dir_target,
                false
            );
            led_strip_transition(
                leds,
                &leds->dir_trans,
                &leds->rear_strip,
                leds->rear_bar,
                leds->rear_dir_target,
                false
            );
        }

        if (split >= 1.0f) {
            leds->direction_forward = true;
            reset_led_bars(leds, target_bar(leds, false), target_bar(leds, true), current_time);
        } else if (split <= -1.0f) {
            leds->direction_forward = false;
            reset_led_bars(leds, target_bar(leds, false), target_bar(leds, true), current_time);
        }
    }

    // headlights transition
    if (leds->headlights_time > 0.0f) {
        led_strip_transition(
            leds,
            &leds->headlights_trans,
            &leds->front_strip,
            leds->front_bar,
            leds->front_time_target,
            false
        );
        led_strip_transition(
            leds,
            &leds->headlights_trans,
            &leds->rear_strip,
            leds->rear_bar,
            leds->rear_time_target,
            false
        );

        if (hl_split >= 1.0f || (leds->headlights_on == headlights_should && hl_split <= -1.0f)) {
            leds->headlights_time = 0.0f;
            leds->headlights_on = headlights_should;
            if (leds->state.state == STATE_RUNNING) {
                // reset direction transition
                leds->split_distance = VESC_IF->mc_get_distance();
            }
            reset_led_bars(leds, target_bar(leds, false), target_bar(leds, true), current_time);
        }
    }

    if (leds->status_strip.length > 0) {
        float idle_timeout = leds->cfg->status.idle_timeout;
        if (idle_timeout > 0.0f && current_time - leds->status_idle_time > idle_timeout) {
            if (leds->status_idle_blend == 0.0f) {
                leds->status_animation_start = current_time;
            }
            rate_limitf(&leds->status_idle_blend, 1.0f, BR_RATE);
        } else {
            rate_limitf(&leds->status_idle_blend, 0.0f, BR_RATE);
        }

        status_animate(leds, &leds->status_strip, current_time, 1.0f, leds->status_idle_blend);
    }

    if (leds->cfg->status_on_front_when_lifted && leds->status_on_front_blend > 0.0f) {
        if (leds->cfg->lights_off_when_lifted &&
            current_time - leds->status_on_front_idle_time > 3.0f) {
            rate_limitf(&leds->status_on_front_idle_blend, 1.0f, BR_RATE);
        } else {
            rate_limitf(&leds->status_on_front_idle_blend, 0.0f, BR_RATE);
        }

        status_animate(
            leds,
            &leds->front_strip,
            current_time,
            leds->status_on_front_blend,
            leds->status_on_front_idle_blend
        );
    }

    led_driver_paint(&leds->led_driver, leds->led_data, leds->led_count);
}

void leds_destroy(Leds *leds) {
    led_driver_destroy(&leds->led_driver);

    if (leds->led_data) {
        VESC_IF->free(leds->led_data);
        leds->led_data = NULL;
    }
    leds->led_count = 0;
}
