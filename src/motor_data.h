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

#include "biquad.h"

#include <stdbool.h>
#include <stdint.h>

#define ACCEL_ARRAY_SIZE 40

typedef struct {
    float erpm;
    float abs_erpm;
    float last_erpm;
    int8_t erpm_sign;

    float current;
    bool braking;

    float duty_cycle;
    float duty_smooth;

    // an average calculated over last ACCEL_ARRAY_SIZE values
    float acceleration;
    float accel_history[ACCEL_ARRAY_SIZE];
    uint8_t accel_idx;

    bool atr_filter_enabled;
    Biquad atr_current_biquad;
    float atr_filtered_current;
} MotorData;

void motor_data_reset(MotorData *m);

void motor_data_configure(MotorData *m, float frequency);

void motor_data_update(MotorData *m);
