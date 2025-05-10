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
#include "state.h"

typedef struct {
    float step_size;

    float input;
    float ramped_step_size;

    float setpoint;
} Remote;

void remote_init(Remote *remote);

void remote_reset(Remote *remote);

void remote_configure(Remote *remote, const RefloatConfig *config);

void remote_input(Remote *remote, const RefloatConfig *config);

void remote_update(Remote *remote, const State *state, const RefloatConfig *config);
