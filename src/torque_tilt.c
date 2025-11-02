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

#include "lib/utils.h"

#include <math.h>

void torque_tilt_init(TorqueTilt *tt) {
    smooth_setpoint_init(&tt->setpoint);

    torque_tilt_reset(tt);
}

void torque_tilt_reset(TorqueTilt *tt) {
    smooth_setpoint_reset(&tt->setpoint, 0.0f);
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *config, float frequency) {
    smooth_setpoint_configure_strengths(
        &tt->setpoint,
        config->torque_tilt.filter.strength,
        config->torque_tilt.filter.on_ease_in_strength,
        config->torque_tilt.filter.off_ease_in_strength,
        config->torquetilt_on_speed,
        config->torquetilt_off_speed,
        config->torquetilt_on_speed,
        config->torquetilt_off_speed,
        frequency
    );
}

void torque_tilt_update(
    TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config, float dt
) {
    float strength =
        motor->braking ? config->torquetilt_strength_regen : config->torquetilt_strength;

    // Take abs motor current, subtract start offset, and take the max of that
    // with 0 to get the current above our start threshold (absolute). Then
    // multiply it by "power" to get our desired angle, and min with the limit
    // to respect boundaries. Finally multiply it by motor current sign to get
    // directionality back.
    float target =
        fminf(
            fmaxf((fabsf(motor->filt_current.value) - config->torquetilt_start_current), 0) *
                strength,
            config->torquetilt_angle_limit
        ) *
        sign(motor->filt_current.value);

    smooth_setpoint_update(&tt->setpoint, target, dt, motor->forward);
}

void torque_tilt_winddown(TorqueTilt *tt) {
    // tt->setpoint *= 0.995;
}
