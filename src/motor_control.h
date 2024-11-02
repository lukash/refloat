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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "conf/datatypes.h"
#include "state.h"

typedef struct {
    bool current_requested;
    float requested_current;

    uint8_t click_counter;
    float brake_timeout;
    bool parking_brake_active;

    float brake_current;
    float click_current;
    ParkingBrakeMode parking_brake_mode;
} MotorControl;

void motor_control_init(MotorControl *mc);

void motor_control_configure(MotorControl *mc, const RefloatConfig *config);

void motor_control_request_current(MotorControl *mc, float current);

void motor_control_apply(MotorControl *mc, float abs_erpm, RunState state, float time);
