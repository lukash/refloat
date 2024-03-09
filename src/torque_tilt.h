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

#pragma once

#include "conf/datatypes.h"
#include "motor_data.h"

typedef struct {
    float on_step_size;
    float off_step_size;

    float offset;  // rate-limited setpoint offset
} TorqueTilt;

void torque_tilt_reset(TorqueTilt *tt);

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *config);

void torque_tilt_update(TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config);

void torque_tilt_winddown(TorqueTilt *tt);
