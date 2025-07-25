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

#include "alert_tracker.h"
#include "biquad.h"

#include <stdbool.h>
#include <stdint.h>

#define ACCEL_ARRAY_SIZE 40

typedef struct {
    float erpm;
    float abs_erpm;
    float abs_erpm_smooth;
    float last_erpm;
    int8_t erpm_sign;

    float speed;

    float current;  //  "regular" motor current (positive = accelerating, negative = braking)
    float dir_current;  // directional current (sign represents direction of torque generation)
    float filt_current;  // filtered directional current
    bool braking;

    float duty_cycle;
    float duty_raw;

    // an average calculated over last ACCEL_ARRAY_SIZE values
    float acceleration;

    float batt_current;
    float batt_voltage;

    float mosfet_temp;
    float motor_temp;

    // The following values are periodically updated from the aux thread
    float current_min;
    float current_max;
    float battery_current_min;
    float battery_current_max;
    float mosfet_temp_max;
    float motor_temp_max;
    float duty_max_with_margin;
    float lv_threshold;
    float hv_threshold;

    float accel_history[ACCEL_ARRAY_SIZE];
    uint8_t accel_idx;

    bool current_filter_enabled;
    Biquad current_biquad;
} MotorData;

void motor_data_init(MotorData *m);

void motor_data_reset(MotorData *m);

void motor_data_refresh_motor_config(MotorData *m, float lv_threshold, float hv_threshold);

void motor_data_configure(MotorData *m, float frequency);

void motor_data_update(MotorData *m);

void motor_data_evaluate_alerts(const MotorData *m, AlertTracker *at, const Time *time);

float motor_data_get_current_saturation(const MotorData *m);
