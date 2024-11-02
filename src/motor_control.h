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
#include <stdint.h>

#include "conf/datatypes.h"
#include "state.h"
#include "time.h"

typedef struct {
    bool disabled;
    bool current_requested;
    float requested_current;

    uint8_t click_counter;
    time_t brake_timer;
    bool parking_brake_active;

    uint8_t tone_ticks;
    uint8_t tone_counter;
    bool tone_high;
    float tone_intensity;

    float brake_current;
    float click_current;
    ParkingBrakeMode parking_brake_mode;
    uint16_t main_freq;
} MotorControl;

void motor_control_init(MotorControl *mc);

void motor_control_configure(MotorControl *mc, const RefloatConfig *config);

void motor_control_request_current(MotorControl *mc, float current);

void motor_control_apply(MotorControl *mc, float abs_erpm, RunState state, const Time *time);

void motor_control_play_tone(MotorControl *mc, uint16_t frequency, float intensity);

void motor_control_stop_tone(MotorControl *mc);

void motor_control_play_click(MotorControl *mc);
