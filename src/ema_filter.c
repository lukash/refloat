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

#include "ema_filter.h"

#include "utils.h"

#include <math.h>

void ema_filter_configure(
    EMAFilter *filter, const CfgTargetFilter *cfg, float on_speed, float off_speed
) {
    filter->cfg = *cfg;
    filter->k = 0.693f / max(cfg->ema_half_time, 0.001f);
    filter->on_speed = on_speed;
    filter->off_speed = off_speed;
}

void ema_filter_reset(EMAFilter *filter, float value, float speed) {
    filter->value = value;
    filter->speed = speed;
    filter->accel = 0.0f;
}

void ema_filter_update(EMAFilter *filter, float target, float dt) {
    float k = filter->k;
    if (fabsf(target) < fabsf(filter->value)) {
        k *= filter->cfg.ema_return_multiplier;
    }

    // Coefficients chosen to approximate Gaussian filter and preserve half time
    float speed = 1.34f * k * (target - filter->value);
    float accel = 3.62f * k * (speed - filter->speed);
    float jerk = 9.77f * k * (accel - filter->accel);

    filter->accel += dt * jerk;
    filter->speed += dt * filter->accel;

    uint16_t speed_limit;
    if ((filter->value * target >= 0) && (fabsf(filter->value) > fabsf(target))) {
        // Moving towards smaller angle of same sign or zero
        speed_limit = filter->off_speed;
    } else {
        // Moving towards larger angle of same sign or crossing zero
        speed_limit = filter->on_speed;
    }
    filter->value += dt * sign(filter->speed) * min(fabsf(filter->speed), speed_limit);
}
