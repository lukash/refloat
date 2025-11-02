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
    st->in_alpha_away = 0.0f;
    st->in_alpha_back = 0.0f;

    smooth_setpoint_reset(st, 0.0f);
}

void smooth_setpoint_configure(
    SmoothSetpoint *st,
    float cutoff,
    float in_cutoff_away,
    float in_cutoff_back,
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

    st->alpha = ema_calculate_alpha(cutoff, frequency);
    st->in_alpha_away = ema_calculate_alpha(in_cutoff_away, frequency);
    st->in_alpha_back = ema_calculate_alpha(in_cutoff_back, frequency);
}

static float strength_to_cutoff(float strength) {
    return powf(10.0f, 2.0f - strength);
}

void smooth_setpoint_configure_strengths(
    SmoothSetpoint *st,
    float strength,
    float on_ease_in_strength,
    float off_ease_in_strength,
    float on_speed_up,
    float off_speed_up,
    float on_speed_down,
    float off_speed_down,
    float frequency
) {
    smooth_setpoint_configure(
        st,
        strength_to_cutoff(strength),
        strength_to_cutoff(on_ease_in_strength),
        strength_to_cutoff(off_ease_in_strength),
        on_speed_up,
        off_speed_up,
        on_speed_down,
        off_speed_down,
        frequency
    );
}

void smooth_setpoint_reset(SmoothSetpoint *st, float value) {
    st->v1 = 0;
    st->step = 0;
    st->value = value;
}

void smooth_setpoint_update(SmoothSetpoint *st, float target, float dt, bool forward) {
    bool is_up = (st->value >= 0.0f) == forward;

    st->v1 += st->alpha * (target - st->v1);
    float delta = st->alpha * (st->v1 - st->value);

    if (fabsf(delta) > fabsf(st->step) || sign(delta) != sign(st->step)) {
        if (sign(st->value) == sign(delta)) {
            st->step += st->in_alpha_away * (delta - st->step);
        } else {
            st->step += st->in_alpha_back * (delta - st->step);
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

    st->value += sign(st->step) * min(fabsf(st->step), speed_limit * dt);
}
