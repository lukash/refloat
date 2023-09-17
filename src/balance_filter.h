// Copyright 2023 - 2024 Lukas Hrazky
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

#ifndef BALANCE_FILTER_h
#define BALANCE_FILTER_h

#include "conf/datatypes.h"

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float acc_mag;

    // parameters
    float acc_confidence_decay;
    float kp_pitch;
    float kp_roll;
    float kp_yaw;
} BalanceFilterData;

void balance_filter_init(BalanceFilterData *data);

void balance_filter_configure(BalanceFilterData *data, const RefloatConfig *config);

void balance_filter_update(BalanceFilterData *data, float *gyro_xyz, float *accel_xyz, float dt);

float balance_filter_get_roll(BalanceFilterData *data);
float balance_filter_get_pitch(BalanceFilterData *data);
float balance_filter_get_yaw(BalanceFilterData *data);

#endif
