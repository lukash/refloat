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

#include "smooth_target.h"

#include "utils.h"

#include <math.h>

void smooth_target_configure(
    SmoothTarget *st, const CfgTargetFilter *cfg, float on_speed, float off_speed, float hertz
) {
    st->cfg = *cfg;
    st->on_speed = on_speed / hertz;
    st->off_speed = off_speed / hertz;
}

void smooth_target_reset(SmoothTarget *st, float value) {
    st->v1 = 0;
    st->step = 0;
    st->value = value;
}

void smooth_target_update(SmoothTarget *st, float target) {
    st->v1 += st->cfg.alpha * (target - st->v1);

    float delta = st->cfg.alpha * (st->v1 - st->value);

    if (fabsf(delta) > fabsf(st->step) || sign(delta) != sign(st->step)) {
        if (sign(st->value) == sign(delta)) {
            st->step += st->cfg.in_alpha_away * (delta - st->step);
        } else {
            st->step += st->cfg.in_alpha_back * (delta - st->step);
        }
    } else {
        st->step = delta;
    }

    float speed_limit;
    if ((st->value * target >= 0) && (fabsf(st->value) > fabsf(target))) {
        // Moving towards smaller angle of same sign or zero
        speed_limit = st->off_speed;
    } else {
        // Moving towards larger angle of same sign or crossing zero
        speed_limit = st->on_speed;
    }

    st->value += sign(st->step) * min(fabsf(st->step), speed_limit);
}
