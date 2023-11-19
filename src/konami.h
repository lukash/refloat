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

#include "footpad_sensor.h"

typedef struct {
    float time;
    uint8_t state;
    const FootpadSensorState *sequence;
    uint8_t sequence_size;
} Konami;

void konami_init(Konami *konami, const FootpadSensorState *sequence, uint8_t sequence_size);

bool konami_check(Konami *konami, const FootpadSensor *fs, float current_time);
