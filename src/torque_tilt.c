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

#include "torque_tilt.h"

#include "utils.h"

#include <math.h>

void torque_tilt_reset(TorqueTilt *tt) {
    tt->offset = 0;
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *config) {
    tt->on_step_size = config->torquetilt_on_speed / config->hertz;
    tt->off_step_size = config->torquetilt_off_speed / config->hertz;
    tt->ramped_step_size = 0;
}

void torque_tilt_update(TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config) {
    float strength =
        motor->braking ? config->torquetilt_strength_regen : config->torquetilt_strength;

    // Take abs motor current, subtract start offset, and take the max of that
    // with 0 to get the current above our start threshold (absolute). Then
    // multiply it by "power" to get our desired angle, and min with the limit
    // to respect boundaries. Finally multiply it by motor current sign to get
    // directionality back.
    float target_offset =
        fminf(
            fmaxf((fabsf(motor->atr_filtered_current) - config->torquetilt_start_current), 0) *
                strength,
            config->torquetilt_angle_limit
        ) *
        sign(motor->atr_filtered_current);

    float step_size = 0;
    if ((tt->offset - target_offset > 0 && target_offset > 0) ||
        (tt->offset - target_offset < 0 && target_offset < 0)) {
        step_size = tt->off_step_size;
    } else {
        step_size = tt->on_step_size;
    }

    if (motor->abs_erpm < 500) {
        step_size /= 2;
    }

    // Smoothen changes in tilt angle by ramping the step size
    if (config->inputtilt_smoothing_factor > 0) {
        float smoothing_factor = 0.04;
        // Sets the angle away from Target that step size begins ramping down
        float smooth_center_window = 1.5;
        float tiltback_target_diff = target_offset - tt->offset;

        // Within X degrees of Target Angle, start ramping down step size
        if (fabsf(tiltback_target_diff) < smooth_center_window) {
            // Target step size is reduced the closer to center you are (needed for smoothly
            // transitioning away from center)
            tt->ramped_step_size = (smoothing_factor * step_size * (tiltback_target_diff / 2)) +
                ((1 - smoothing_factor) * tt->ramped_step_size);
            // Linearly ramped down step size is provided as minimum to prevent overshoot
            float centering_step_size =
                fminf(fabsf(tt->ramped_step_size), fabsf(tiltback_target_diff / 2) * step_size) *
                sign(tiltback_target_diff);
            if (fabsf(tiltback_target_diff) < fabsf(centering_step_size)) {
                tt->offset = target_offset;
            } else {
                tt->offset += centering_step_size;
            }
        } else {
            // Ramp up step size until the configured tilt speed is reached
            tt->ramped_step_size = (smoothing_factor * step_size * sign(tiltback_target_diff)) +
                ((1 - smoothing_factor) * tt->ramped_step_size);
            tt->offset += tt->ramped_step_size;
        }
    } else {
        rate_limitf(&tt->offset, target_offset, step_size);
    }
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->offset *= 0.995;
}
