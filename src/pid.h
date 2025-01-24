// Copyright 2025 Lukas Hrazky
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

#pragma once

#include "conf/datatypes.h"
#include "imu.h"
#include "motor_data.h"
#include "state.h"

typedef struct {
    float p;
    float i;
    float rate_p;  // used instead of d, works better with high Mahony KP

    // PID brake scaling
    float kp_brake_scale;
    float kp2_brake_scale;
    float kp_accel_scale;
    float kp2_accel_scale;
} PID;

void pid_init(PID *pid);

void pid_update(
    PID *pid,
    float setpoint,
    const MotorData *md,
    const IMU *imu,
    const State *state,
    const RefloatConfig *config
);

void pid_reset_integral(PID *pid);
