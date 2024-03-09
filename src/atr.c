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

#include "utils.h"

#include <math.h>

void atr_reset(ATR *atr) {
    atr->accel_diff = 0;
    atr->speed_boost = 0;
    atr->target_offset = 0;
    atr->offset = 0;
    atr->braketilt_target_offset = 0;
    atr->braketilt_offset = 0;
}

void atr_configure(ATR *atr, const RefloatConfig *config) {
    atr->on_step_size = config->atr_on_speed / config->hertz;
    atr->off_step_size = config->atr_off_speed / config->hertz;

    atr->speed_boost_mult = 1.0f / 3000.0f;
    if (fabsf(config->atr_speed_boost) > 0.4f) {
        // above 0.4 we add 500erpm for each extra 10% of speed boost, so at
        // most +6000 for 100% speed boost
        atr->speed_boost_mult = 1.0f / ((fabsf(config->atr_speed_boost) - 0.4f) * 5000 + 3000.0f);
    }

    if (config->braketilt_strength == 0) {
        atr->braketilt_factor = 0;
    } else {
        // incorporate negative sign into braketilt factor instead of adding it each balance loop
        atr->braketilt_factor = -(0.5f + (20 - config->braketilt_strength) / 5.0f);
    }
}

static void atr_update(ATR *atr, const MotorData *motor, const RefloatConfig *config) {
    float abs_torque = fabsf(motor->atr_filtered_current);
    float torque_offset = 8;  // hard-code to 8A for now (shouldn't really be changed much anyways)
    float atr_threshold = motor->braking ? config->atr_threshold_down : config->atr_threshold_up;
    float accel_factor =
        motor->braking ? config->atr_amps_decel_ratio : config->atr_amps_accel_ratio;
    float accel_factor2 = accel_factor * 1.3;

    // compare measured acceleration to expected acceleration
    float measured_acc = fmaxf(motor->acceleration, -5);
    measured_acc = fminf(measured_acc, 5);

    // expected acceleration is proportional to current (minus an offset, required to
    // balance/maintain speed)
    float expected_acc;
    if (abs_torque < 25) {
        expected_acc =
            (motor->atr_filtered_current - motor->erpm_sign * torque_offset) / accel_factor;
    } else {
        // primitive linear approximation of non-linear torque-accel relationship
        int torque_sign = sign(motor->atr_filtered_current);
        expected_acc = (torque_sign * 25 - motor->erpm_sign * torque_offset) / accel_factor;
        expected_acc += torque_sign * (abs_torque - 25) / accel_factor2;
    }

    bool forward = motor->erpm > 0;
    if (motor->abs_erpm < 250 && abs_torque > 30) {
        forward = (expected_acc > 0);
    }

    float new_accel_diff = expected_acc - measured_acc;
    if (motor->abs_erpm > 2000) {
        atr->accel_diff = 0.9f * atr->accel_diff + 0.1f * new_accel_diff;
    } else if (motor->abs_erpm > 1000) {
        atr->accel_diff = 0.95f * atr->accel_diff + 0.05f * new_accel_diff;
    } else if (motor->abs_erpm > 250) {
        atr->accel_diff = 0.98f * atr->accel_diff + 0.02f * new_accel_diff;
    } else {
        atr->accel_diff = 0;
    }

    // atr->accel_diff | > 0  | <= 0
    // -------------+------+-------
    //         forward | up   | down
    //        !forward | down | up
    float atr_strength =
        forward == (atr->accel_diff > 0) ? config->atr_strength_up : config->atr_strength_down;

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

    atr->target_offset = atr->target_offset * 0.95 + 0.05 * new_atr_target;
    atr->target_offset = fminf(atr->target_offset, config->atr_angle_limit);
    atr->target_offset = fmaxf(atr->target_offset, -config->atr_angle_limit);

    float response_boost = 1;
    if (motor->abs_erpm > 2500) {
        response_boost = config->atr_response_boost;
    }
    if (motor->abs_erpm > 6000) {
        response_boost *= config->atr_response_boost;
    }

    // Key to keeping the board level and consistent is to determine the appropriate step size!
    // We want to react quickly to changes, but we don't want to overreact to glitches in
    // acceleration data or trigger oscillations...
    float atr_step_size = 0;
    const float TT_BOOST_MARGIN = 2;
    if (forward) {
        if (atr->offset < 0) {
            // downhill
            if (atr->offset < atr->target_offset) {
                // to avoid oscillations we go down slower than we go up
                atr_step_size = atr->off_step_size;
                if ((atr->target_offset > 0) &&
                    ((atr->target_offset - atr->offset) > TT_BOOST_MARGIN) &&
                    motor->abs_erpm > 2000) {
                    // boost the speed if tilt target has reversed (and if there's a significant
                    // margin)
                    atr_step_size = atr->off_step_size * config->atr_transition_boost;
                }
            } else {
                // ATR is increasing
                atr_step_size = atr->on_step_size * response_boost;
            }
        } else {
            // uphill or other heavy resistance (grass, mud, etc)
            if ((atr->target_offset > -3) && (atr->offset > atr->target_offset)) {
                // ATR winding down (current ATR is bigger than the target)
                // normal wind down case: to avoid oscillations we go down slower than we go up
                atr_step_size = atr->off_step_size;
            } else {
                // standard case of increasing ATR
                atr_step_size = atr->on_step_size * response_boost;
            }
        }
    } else {
        if (atr->offset > 0) {
            // downhill
            if (atr->offset > atr->target_offset) {
                // to avoid oscillations we go down slower than we go up
                atr_step_size = atr->off_step_size;
                if ((atr->target_offset < 0) &&
                    ((atr->offset - atr->target_offset) > TT_BOOST_MARGIN) &&
                    motor->abs_erpm > 2000) {
                    // boost the speed if tilt target has reversed (and if there's a significant
                    // margin)
                    atr_step_size = atr->off_step_size * config->atr_transition_boost;
                }
            } else {
                // ATR is increasing
                atr_step_size = atr->on_step_size * response_boost;
            }
        } else {
            // uphill or other heavy resistance (grass, mud, etc)
            if ((atr->target_offset < 3) && (atr->offset < atr->target_offset)) {
                // normal wind down case: to avoid oscillations we go down slower than we go up
                atr_step_size = atr->off_step_size;
            } else {
                // standard case of increasing torquetilt
                atr_step_size = atr->on_step_size * response_boost;
            }
        }
    }

    if (motor->abs_erpm < 500) {
        atr_step_size /= 2;
    }

    rate_limitf(&atr->offset, atr->target_offset, atr_step_size);
}

static void braketilt_update(
    ATR *atr, const MotorData *motor, const RefloatConfig *config, float proportional
) {
    // braking also should cause setpoint change lift, causing a delayed lingering nose lift
    if (atr->braketilt_factor < 0 && motor->braking && motor->abs_erpm > 2000) {
        // negative currents alone don't necessarily consitute active braking, look at proportional:
        if (sign(proportional) != motor->erpm_sign) {
            float downhill_damper = 1;
            // if we're braking on a downhill we don't want braking to lift the setpoint quite as
            // much
            if ((motor->erpm > 1000 && atr->accel_diff < -1) ||
                (motor->erpm < -1000 && atr->accel_diff > 1)) {
                downhill_damper += fabsf(atr->accel_diff) / 2;
            }
            atr->braketilt_target_offset = proportional / atr->braketilt_factor / downhill_damper;
            if (downhill_damper > 2) {
                // steep downhills, we don't enable this feature at all!
                atr->braketilt_target_offset = 0;
            }
        }
    } else {
        atr->braketilt_target_offset = 0;
    }

    float braketilt_step_size = atr->off_step_size / config->braketilt_lingering;
    if (fabsf(atr->braketilt_target_offset) > fabsf(atr->braketilt_offset)) {
        braketilt_step_size = atr->on_step_size * 1.5;
    } else if (motor->abs_erpm < 800) {
        braketilt_step_size = atr->on_step_size;
    }

    if (motor->abs_erpm < 500) {
        braketilt_step_size /= 2;
    }

    rate_limitf(&atr->braketilt_offset, atr->braketilt_target_offset, braketilt_step_size);
}

void atr_and_braketilt_update(
    ATR *atr, const MotorData *motor, const RefloatConfig *config, float proportional
) {
    atr_update(atr, motor, config);
    braketilt_update(atr, motor, config, proportional);
}

void atr_and_braketilt_winddown(ATR *atr) {
    atr->offset *= 0.995;
    atr->target_offset *= 0.99;
    atr->braketilt_offset *= 0.995;
    atr->braketilt_target_offset *= 0.99;
}
