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

#include "booster.h"

#include "utils.h"

#include <math.h>

void booster_init(Booster *b) {
    booster_reset(b);
}

void booster_reset(Booster *b) {
    b->current = 0.0f;
}

void booster_update(
    Booster *b, const MotorData *md, const RefloatConfig *config, float proportional
) {
    float current;
    float angle;
    float ramp;
    if (md->braking) {
        current = config->brkbooster_current;
        angle = config->brkbooster_angle;
        ramp = config->brkbooster_ramp;
    } else {
        current = config->booster_current;
        angle = config->booster_angle;
        ramp = config->booster_ramp;
    }

    // Make booster a bit stronger at higher speed (up to 2x stronger when braking)
    const int boost_min_erpm = 3000;
    if (md->abs_erpm > boost_min_erpm) {
        float speedstiffness = fminf(1, (md->abs_erpm - boost_min_erpm) / 10000);
        if (md->braking) {
            // use higher current at speed when braking
            current += current * speedstiffness;
        } else {
            // when accelerating, we reduce the booster start angle as we get faster
            // strength remains unchanged
            float angledivider = 1 + speedstiffness;
            angle /= angledivider;
        }
    }

    float abs_proportional = fabsf(proportional);
    if (abs_proportional > angle) {
        if (abs_proportional - angle < ramp) {
            current *= sign(proportional) * (abs_proportional - angle) / ramp;
        } else {
            current *= sign(proportional);
        }
    } else {
        current = 0;
    }

    // No harsh changes in booster current (effective delay <= 100ms)
    b->current = 0.01 * current + 0.99 * b->current;
}
