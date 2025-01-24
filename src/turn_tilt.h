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

#include "atr.h"
#include "conf/datatypes.h"
#include "imu.h"
#include "motor_data.h"

typedef struct {
    float step_size;
    float boost_per_erpm;

    float last_yaw_angle;
    float last_yaw_change;
    float yaw_change;
    float abs_yaw_change;
    float yaw_aggregate;

    float target;
    float setpoint;
} TurnTilt;

void turn_tilt_reset(TurnTilt *tt);

void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *config);

void turn_tilt_aggregate(TurnTilt *tt, const IMU *imu);

void turn_tilt_update(
    TurnTilt *tt,
    const MotorData *md,
    const ATR *atr,
    float balance_pitch,
    float noseangling,
    const RefloatConfig *config
);
