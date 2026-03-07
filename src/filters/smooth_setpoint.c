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

#include "smooth_setpoint.h"

#include "ema.h"
#include "lib/utils.h"

#include <math.h>

void smooth_setpoint_init(SmoothSetpoint *st) {
    st->on_speed_up = 0.0f;
    st->off_speed_up = 0.0f;
    st->on_speed_down = 0.0f;
    st->off_speed_down = 0.0f;

    st->alpha = 0.0f;
    st->on_speed_alpha = 0.0f;
    st->off_speed_alpha = 0.0f;
    st->winddown_alpha = 0.0f;

    smooth_setpoint_reset(st);
}

void smooth_setpoint_configure(
    SmoothSetpoint *st,
    float time_constant,
    float on_speed_time_constant,
    float off_speed_time_constant,
    float winddown_time_constant,
    float on_speed_up,
    float off_speed_up,
    float on_speed_down,
    float off_speed_down,
    float frequency
) {
    st->on_speed_up = on_speed_up;
    st->off_speed_up = off_speed_up;
    st->on_speed_down = on_speed_down;
    st->off_speed_down = off_speed_down;

    // alpha is used in a 2nd order EMA filter, 2.146 is the multiplier for the
    // calculated first order EMA alpha to maintain the time constant
    st->alpha = 2.146f * ema_calculate_alpha_time_constant(time_constant, frequency);
    st->on_speed_alpha = ema_calculate_alpha_time_constant(on_speed_time_constant, frequency);
    st->off_speed_alpha = ema_calculate_alpha_time_constant(off_speed_time_constant, frequency);
    st->winddown_alpha = ema_calculate_alpha_time_constant(winddown_time_constant, frequency);
}

void smooth_setpoint_reset(SmoothSetpoint *st) {
    st->is_winddown = false;
    st->v1 = 0.0f;
    st->step = 0.0f;
    st->value = 0.0f;
}

void smooth_setpoint_update(SmoothSetpoint *st, float target, bool forward, float mult, float dt) {
    if (st->is_winddown) {
        st->is_winddown = false;
        st->v1 = st->value;
        st->step = 0;
    }

    bool is_up = (st->value >= 0.0f) == forward;

    st->v1 += mult * st->alpha * (target - st->v1);
    float delta = mult * st->alpha * (st->v1 - st->value);

    if (fabsf(delta) > fabsf(st->step) || sign(delta) != sign(st->step)) {
        if (sign(st->value) == sign(delta)) {
            st->step += mult * st->on_speed_alpha * (delta - st->step);
        } else {
            st->step += mult * st->off_speed_alpha * (delta - st->step);
        }
    } else {
        st->step = delta;
    }

    float speed_limit;
    const float on_speed = is_up ? st->on_speed_up : st->on_speed_down;
    const float off_speed = is_up ? st->off_speed_up : st->off_speed_down;
    if (st->value * target < 0) {
        speed_limit = max(on_speed, off_speed);
    } else if (fabsf(st->value) > fabsf(target)) {
        speed_limit = off_speed;
    } else {
        speed_limit = on_speed;
    }

    st->value += sign(st->step) * min(fabsf(st->step), mult * speed_limit * dt);
}

void smooth_setpoint_winddown(SmoothSetpoint *st) {
    st->is_winddown = true;
    st->value *= 1.0f - st->winddown_alpha;
}
