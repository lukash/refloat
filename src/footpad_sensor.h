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

typedef enum {
    FS_NONE = 0,
    FS_LEFT = 1,
    FS_RIGHT = 2,
    FS_BOTH = 3
} FootpadSensorState;

typedef struct {
    float adc1, adc2;
    FootpadSensorState state;
} FootpadSensor;

void footpad_sensor_init(FootpadSensor *fs);

void footpad_sensor_update(FootpadSensor *fs, const RefloatConfig *config);

int footpad_sensor_state_to_switch_compat(FootpadSensorState v);
