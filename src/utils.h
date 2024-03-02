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

#pragma once

#include "vesc_c_if.h"

#include <stdint.h>

#define log_msg(fmt, ...)                                                                          \
    do {                                                                                           \
        if (!VESC_IF->app_is_output_disabled()) {                                                  \
            float t = VESC_IF->system_time();                                                      \
            uint32_t decimals = (uint32_t) ((t - (uint32_t) t) * 1000000);                         \
            VESC_IF->printf(                                                                       \
                "%d.%.6d [refloat] " fmt, (uint32_t) t, decimals __VA_OPT__(, ) __VA_ARGS__        \
            );                                                                                     \
        }                                                                                          \
    } while (0)

#define log_error(fmt, ...) log_msg("Error: " fmt __VA_OPT__(, ) __VA_ARGS__)

// Declaration for the SEMD_APP_DATA macro, definition needs to be in main.c.
void send_app_data_overflow_terminate();

/**
 * DRY macro to check the buffer didn't overflow and send the app data.
 */
#define SEND_APP_DATA(buffer, buf_size, ind)                                                       \
    do {                                                                                           \
        if (ind > buf_size) {                                                                      \
            log_error("%s: App data buffer overflow, terminating.", __func__);                     \
            /* terminate the main thread, the memory has just been corrupted by buffer overflow */ \
            send_app_data_overflow_terminate();                                                    \
        }                                                                                          \
        VESC_IF->send_app_data(buffer, ind);                                                       \
    } while (0)

#define sign(x) (((x) < 0) ? -1 : 1)

#define deg2rad(deg) ((deg) * (M_PI / 180.0f))
#define rad2deg(rad) ((rad) * (180.0f / M_PI))

#define min(a, b)                                                                                  \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _a < _b ? _a : _b;                                                                         \
    })

#define max(a, b)                                                                                  \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _a > _b ? _a : _b;                                                                         \
    })

// See @rate_limitf
#define rate_limit(value, target, step)                                                            \
    do {                                                                                           \
        if (abs(target - *value) < step) {                                                         \
            *value = target;                                                                       \
        } else if (target - *value > 0) {                                                          \
            *value += step;                                                                        \
        } else {                                                                                   \
            *value -= step;                                                                        \
        }                                                                                          \
    } while (0)

uint32_t rnd(uint32_t seed);

float clampf(float value, float min, float max);

/**
 * Rate-limits @p value towards @p target by an amount of maximum value of @p step.
 *
 * If the difference between @p value and @p target is greated than step, @p
 * value is increased or decreased (if @p target is greater or less than @p
 * value respectively) by @p step. Otherwise, @p value is set to @p target.
 *
 * @param value A pointer to value to rate-limit towards @p target.
 * @param target A target to rate-limit @p value towards.
 * @param step A maximum unit of change of @p value.
 */
void rate_limitf(float *value, float target, float step);
