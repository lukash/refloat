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

#include "filters/ema.h"
#include "time.h"

#include <stdbool.h>

typedef struct {
    float start_setpoint;  // absolute setpoint from which we started
    float target_setpoint;  // absolute target setpoint, either 0 or TARGET_STOP_ANGLE
    float start_distance;  // absolute distance from which we started
    float target_distance;  // relative target distance to go, negative when going into reverse stop
    float current_distance;  // relative distance we're at right now
    EMA progress;

    time_t timer;
} ReverseStop;

void reverse_stop_init(ReverseStop *rs);

void reverse_stop_reset(ReverseStop *rs, float distance);

void reverse_stop_configure(ReverseStop *rs, float frequency);

void reverse_stop_update(
    ReverseStop *rs, float distance, float setpoint, const Time *time, bool enabled
);

float reverse_stop_setpoint(ReverseStop *rs);

bool reverse_stop_active(ReverseStop *rs);

bool reverse_stop_stop(ReverseStop *rs, const Time *time);
