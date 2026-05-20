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
#include "state.h"
#include "time.h"

typedef struct {
    float input;
    SmoothSetpoint setpoint;
    time_t command_input_time;

    float move_speed;  // speed for the remote (wheel) move; NAN means no remote control
    float move_pid_i;
    time_t move_idle_time;
} Remote;

void remote_init(Remote *remote, const Time *time);

void remote_reset(Remote *remote, const Time *time);

void remote_configure(Remote *remote, const RefloatConfig *config, float frequency);

void remote_input(Remote *remote, const Time *time, const RefloatConfig *config);

void remote_command_input(
    Remote *remote, float value, const Time *time, const RefloatConfig *config
);

float remote_get_move_torque(Remote *remote, float speed, float dt);

void remote_update(Remote *remote, const State *state, const RefloatConfig *config, float dt);
