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

typedef struct {
    float on_speed_up;
    float off_speed_up;
    float on_speed_down;
    float off_speed_down;

    float alpha;
    float in_alpha_away;
    float in_alpha_back;

    float v1;
    float step;
    float value;
} SmoothSetpoint;

void smooth_setpoint_init(SmoothSetpoint *st);

void smooth_setpoint_configure(
    SmoothSetpoint *st,
    float cutoff,
    float in_cutoff_away,
    float in_cutoff_back,
    float on_speed_up,
    float off_speed_up,
    float on_speed_down,
    float off_speed_down,
    float frequency
);

void smooth_setpoint_configure_strengths(
    SmoothSetpoint *st,
    float strength,
    float on_ease_in_strength,
    float off_ease_in_strength,
    float on_speed_up,
    float off_speed_up,
    float on_speed_down,
    float off_speed_down,
    float frequency
);

void smooth_setpoint_reset(SmoothSetpoint *st, float value);

void smooth_setpoint_update(SmoothSetpoint *st, float target, float dt, bool forward);
