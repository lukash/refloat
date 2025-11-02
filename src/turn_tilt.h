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
#include "filters/smooth_setpoint.h"
#include "imu.h"
#include "motor_data.h"

typedef struct {
    float boost_per_erpm;

    float last_yaw_angle;
    EMA yaw_change;
    float yaw_aggregate;

    float target;
    SmoothSetpoint setpoint;
} TurnTilt;

void turn_tilt_init(TurnTilt *tt);

void turn_tilt_reset(TurnTilt *tt);

void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *config, float frequency);

void turn_tilt_aggregate(TurnTilt *tt, const IMU *imu, float dt);

void turn_tilt_update(TurnTilt *tt, const MotorData *md, const RefloatConfig *config, float dt);

void turn_tilt_winddown(TurnTilt *tt);
