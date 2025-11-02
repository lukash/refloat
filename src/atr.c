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

#include "atr.h"

#include "lib/utils.h"

#include <math.h>

void atr_init(ATR *atr) {
    atr->speed_boost_mult = 0.0f;
    atr->ad_alpha1 = 0.0f;
    atr->ad_alpha2 = 0.0f;
    atr->ad_alpha3 = 0.0f;

    smooth_setpoint_init(&atr->setpoint);

    atr_reset(atr);
}

void atr_reset(ATR *atr) {
    atr->accel_diff = 0.0f;
    atr->speed_boost = 0.0f;

    atr->target = 0.0f;
    smooth_setpoint_reset(&atr->setpoint, 0.0f);
}

void atr_configure(ATR *atr, const RefloatConfig *config, float frequency) {
    atr->speed_boost_mult = 1.0f / 3000.0f;
    if (fabsf(config->atr_speed_boost) > 0.4f) {
        // above 0.4 we add 500erpm for each extra 10% of speed boost, so at
        // most +6000 for 100% speed boost
        atr->speed_boost_mult = 1.0f / ((fabsf(config->atr_speed_boost) - 0.4f) * 5000 + 3000.0f);
    }

    atr->ad_alpha3 = ema_calculate_alpha(10.0f, frequency);
    atr->ad_alpha2 = ema_calculate_alpha(6.0f, frequency);
    atr->ad_alpha1 = ema_calculate_alpha(1.0f, frequency);

    smooth_setpoint_configure_strengths(
        &atr->setpoint,
        config->atr.filter.strength,
        config->atr.filter.on_ease_in_strength,
        config->atr.filter.off_ease_in_strength,
        config->atr_on_speed,
        config->atr_off_speed,
        config->atr_on_speed,
        config->atr_off_speed,
        frequency
    );
}

void atr_update(ATR *atr, const MotorData *motor, const RefloatConfig *config, float dt) {
    float abs_torque = fabsf(motor->filt_current.value);
    float torque_offset = 8;  // hard-code to 8A for now (shouldn't really be changed much anyways)
    float atr_threshold = motor->braking ? config->atr_threshold_down : config->atr_threshold_up;
    float accel_factor =
        motor->braking ? config->atr_amps_decel_ratio : config->atr_amps_accel_ratio;
    float accel_factor2 = accel_factor * 1.3;

    // compare measured acceleration to expected acceleration
    float measured_acc = clampf(motor->acceleration.value * LOOP_HERTZ_COMPAT_RECIP, -5.0f, 5.0f);

    // expected acceleration is proportional to current (minus an offset, required to
    // balance/maintain speed)
    float expected_acc;
    if (abs_torque < 25) {
        expected_acc =
            (motor->filt_current.value - motor->erpm_sign * torque_offset) / accel_factor;
    } else {
        // primitive linear approximation of non-linear torque-accel relationship
        int torque_sign = sign(motor->filt_current.value);
        expected_acc = (torque_sign * 25 - motor->erpm_sign * torque_offset) / accel_factor;
        expected_acc += torque_sign * (abs_torque - 25) / accel_factor2;
    }

    float new_accel_diff = expected_acc - measured_acc;
    if (motor->abs_erpm > 250) {
        float alpha = atr->ad_alpha3;
        if (motor->abs_erpm > 2000) {
            alpha = atr->ad_alpha1;
        } else if (motor->abs_erpm > 1000) {
            alpha = atr->ad_alpha2;
        }
        atr->accel_diff += alpha * (new_accel_diff - atr->accel_diff);
    } else {
        atr->accel_diff = 0;
    }

    // atr->accel_diff | > 0  | <= 0
    // -------------+------+-------
    //         forward | up   | down
    //        !forward | down | up
    float atr_strength = motor->forward == (atr->accel_diff > 0) ? config->atr_strength_up
                                                                 : config->atr_strength_down;

    // from 3000 to 6000..9000 erpm gradually crank up the torque response
    if (motor->abs_erpm > 3000 && !motor->braking) {
        float speed_boost_mult = (motor->abs_erpm - 3000.0f) * atr->speed_boost_mult;
        // configured speedboost can now also be negative (-1..1)
        // -1 brings it to 0 (if erpm exceeds 9000)
        // +1 doubles it     (if erpm exceeds 9000)
        atr->speed_boost = fminf(1, speed_boost_mult) * config->atr_speed_boost;
        atr_strength += atr_strength * atr->speed_boost;
    } else {
        atr->speed_boost = 0.0f;
    }

    // now ATR target is purely based on gap between expected and actual acceleration
    float new_atr_target = atr_strength * atr->accel_diff;
    if (fabsf(new_atr_target) < atr_threshold) {
        new_atr_target = 0;
    } else {
        new_atr_target -= sign(new_atr_target) * atr_threshold;
    }

    atr->target = atr->target * 0.95 + 0.05 * new_atr_target;
    atr->target = fminf(atr->target, config->atr_angle_limit);
    atr->target = fmaxf(atr->target, -config->atr_angle_limit);

    smooth_setpoint_update(&atr->setpoint, atr->target, dt, motor->forward);
}

void atr_winddown(ATR *atr) {
    // atr->setpoint *= 0.995;
    atr->target *= 0.99;
}
