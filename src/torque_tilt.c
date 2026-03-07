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
    tt->target = 0.0f;
    smooth_setpoint_reset(&tt->setpoint);
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *config, float frequency) {
    smooth_setpoint_configure(
        &tt->setpoint,
        config->torque_tilt.filter.time_constant,
        config->torque_tilt.filter.on_speed_time_constant,
        config->torque_tilt.filter.off_speed_time_constant,
        0.2f,
        config->torquetilt_on_speed,
        config->torquetilt_off_speed,
        config->torquetilt_on_speed_downhill,
        config->torquetilt_off_speed_downhill,
        frequency
    );
}

static void calculate_torque_tilt_target(
    TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config
) {
    float strength =
        (motor->braking ? config->torquetilt_strength_regen : config->torquetilt_strength) *
        TORQUE_CONSTANT_COMPAT;

    float torque_base = fmaxf(
        (fabsf(motor->torque) - config->torquetilt_start_current * TORQUE_CONSTANT_COMPAT), 0
    );
    tt->target =
        fminf(torque_base * strength, config->torquetilt_angle_limit) * sign(motor->torque);
}

void torque_tilt_update(
    TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config, bool wheelslip, float dt
) {
    if (!wheelslip) {
        calculate_torque_tilt_target(tt, motor, config);
        smooth_setpoint_update(&tt->setpoint, tt->target, motor->forward, dt);
    } else {
        smooth_setpoint_winddown(&tt->setpoint);
    }
}
