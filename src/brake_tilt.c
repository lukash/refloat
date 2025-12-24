// Copyright 2022 Dado Mista
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

#include "brake_tilt.h"

#include "utils.h"

#include <math.h>

void brake_tilt_init(BrakeTilt *bt) {
    bt->factor = 0.0f;
    brake_tilt_reset(bt);
}

void brake_tilt_reset(BrakeTilt *bt) {
    bt->ramped_step_size = 0.0f;
    bt->target = 0.0f;
    bt->setpoint = 0.0f;
}

void brake_tilt_configure(BrakeTilt *bt, const RefloatConfig *config) {
    if (config->braketilt_strength == 0) {
        bt->factor = 0;
    } else {
        // incorporate negative sign into braketilt factor instead of adding it each balance loop
        bt->factor = -(0.5f + (20 - config->braketilt_strength) / 5.0f);
    }
}

void brake_tilt_update(
    BrakeTilt *bt,
    const MotorData *motor,
    const ATR *atr,
    const RefloatConfig *config,
    float balance_offset
) {
    // braking also should cause setpoint change lift, causing a delayed lingering nose lift
    if (bt->factor < 0 && motor->braking && motor->abs_erpm > 2000) {
        // negative currents alone don't necessarily constitute active braking, look at
        // proportional:
        if (sign(balance_offset) != motor->erpm_sign) {
            float downhill_damper = 1;
            // if we're braking on a downhill we don't want braking to lift the setpoint quite as
            // much
            if ((motor->erpm > 1000 && atr->accel_diff < -1) ||
                (motor->erpm < -1000 && atr->accel_diff > 1)) {
                downhill_damper += fabsf(atr->accel_diff) / 2;
            }
            bt->target = balance_offset / bt->factor / downhill_damper;
            if (downhill_damper > 2) {
                // steep downhills, we don't enable this feature at all!
                bt->target = 0;
            }
        }
    } else {
        bt->target = 0;
    }

    float braketilt_step_size = atr->off_step_size / max(config->braketilt_lingering, 1);
    if (fabsf(bt->target) > fabsf(bt->setpoint)) {
        braketilt_step_size = atr->on_step_size * 1.5;
    } else if (motor->abs_erpm < 800) {
        braketilt_step_size = atr->on_step_size;
    }

    if (motor->abs_erpm < 500) {
        braketilt_step_size /= 2;
    }

    // Smoothen changes in tilt angle by ramping the step size
    smooth_rampf(&bt->setpoint, &bt->ramped_step_size, bt->target, braketilt_step_size, 0.05, 1.5);
}

void brake_tilt_winddown(BrakeTilt *bt) {
    bt->setpoint *= 0.995;
    bt->target *= 0.99;
}
