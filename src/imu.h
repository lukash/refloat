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

#include "balance_filter.h"
#include "state.h"

typedef struct {
    float pitch;
    float balance_pitch;
    float roll;
    float yaw;
    float pitch_rate;

    float flywheel_pitch_offset;
    float flywheel_roll_offset;
} IMU;

void imu_init(IMU *imu);

void imu_update(IMU *imu, const BalanceFilterData *bf, const State *state);

void imu_set_flywheel_offsets(IMU *imu);
