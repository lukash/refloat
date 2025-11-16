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
#include "filters/biquad.h"
#include "filters/ema.h"
#include "filters/sma.h"

#include <stdbool.h>
#include <stdint.h>

#define ACCEL_ARRAY_SIZE 40

typedef struct {
    float erpm;
    float abs_erpm;
    float last_erpm;
    int8_t erpm_sign;
    EMA abs_erpm_smooth;

    float speed;
    float distance;

    float current;  //  "regular" motor current (positive = accelerating, negative = braking)
    float dir_current;  // directional current (sign represents direction of torque generation)
    Biquad filt_current;  // filtered directional current
    bool braking;
    bool forward;

    float duty_raw;
    EMA duty_cycle;

    SMA acceleration;

    EMA batt_current;
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
} MotorData;

void motor_data_init(MotorData *m);

void motor_data_destroy(MotorData *m);

void motor_data_reset(MotorData *m);

void motor_data_refresh_motor_config(MotorData *m, float lv_threshold, float hv_threshold);

void motor_data_configure(MotorData *m, float current_cutoff_freq, float frequency);

void motor_data_update(MotorData *m, float dt);

void motor_data_evaluate_alerts(const MotorData *m, AlertTracker *at, const Time *time);

float motor_data_get_current_saturation(const MotorData *m);
