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
    tt->setpoint = 0;
    tt->ramped_step_size = 0;
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *config) {
    tt->on_step_size = config->torquetilt_on_speed / config->hertz;
    tt->off_step_size = config->torquetilt_off_speed / config->hertz;
}

void torque_tilt_update(TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config) {
    float strength =
        motor->braking ? config->torquetilt_strength_regen : config->torquetilt_strength;

    // Take abs motor current, subtract start offset, and take the max of that
    // with 0 to get the current above our start threshold (absolute). Then
    // multiply it by "power" to get our desired angle, and min with the limit
    // to respect boundaries. Finally multiply it by motor current sign to get
    // directionality back.
    float target =
        fminf(
            fmaxf((fabsf(motor->filt_current) - config->torquetilt_start_current), 0) * strength,
            config->torquetilt_angle_limit
        ) *
        sign(motor->filt_current);

    float step_size = 0;
    if ((tt->setpoint - target > 0 && target > 0) || (tt->setpoint - target < 0 && target < 0)) {
        step_size = tt->off_step_size;
    } else {
        step_size = tt->on_step_size;
    }

    if (motor->abs_erpm < 500) {
        step_size /= 2;
    }

    // Smoothen changes in tilt angle by ramping the step size
    smooth_rampf(&tt->setpoint, &tt->ramped_step_size, target, step_size, 0.04, 1.5);
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->setpoint *= 0.995;
}
