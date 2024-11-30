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

#include "atr.h"
#include "booster.h"
#include "brake_tilt.h"
#include "charging.h"
#include "data_record.h"
#include "footpad_sensor.h"
#include "haptic_feedback.h"
#include "imu.h"
#include "konami.h"
#include "lcm.h"
#include "leds.h"
#include "motor_control.h"
#include "motor_data.h"
#include "pid.h"
#include "remote.h"
#include "state.h"
#include "time.h"
#include "torque_tilt.h"
#include "turn_tilt.h"

#include <vesc_c_if.h>

typedef struct {
    lib_thread main_thread;
    lib_thread aux_thread;

    RefloatConfig float_conf;

    // Firmware version, passed in from Lisp
    int fw_version_major, fw_version_minor, fw_version_beta;

    Time time;
    MotorData motor;
    IMU imu;
    PID pid;
    MotorControl motor_control;
    TorqueTilt torque_tilt;
    ATR atr;
    BrakeTilt brake_tilt;
    TurnTilt turn_tilt;
    Booster booster;
    Remote remote;

    // Beeper
    int beep_num_left;
    int beep_duration;
    int beep_countdown;
    int beep_reason;
    bool beeper_enabled;

    Leds leds;

    // Lights Control Module - external lights control
    LcmData lcm;

    Charging charging;

    // Config values
    uint32_t loop_time_us;
    float startup_pitch_trickmargin, startup_pitch_tolerance;
    float startup_step_size;
    float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size,
        tiltback_return_step_size;
    float tiltback_variable, tiltback_variable_max_erpm, noseangling_step_size;
    float mc_max_temp_fet, mc_max_temp_mot;
    bool duty_beeping;

    // IMU data for the balancing filter
    BalanceFilterData balance_filter;

    float max_duty_with_margin;

    FootpadSensor footpad;

    // Rumtime state values
    State state;

    float balance_current;

    float setpoint, setpoint_target, setpoint_target_interpolated;
    float noseangling_interpolated, inputtilt_interpolated;
    time_t nag_timer;
    float idle_voltage;
    time_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer,
        fault_switch_half_timer;
    time_t wheelslip_timer, tb_highvoltage_timer;
    float switch_warn_beep_erpm;
    bool traction_control;

    // Darkride aka upside down mode:
    bool is_upside_down_started;  // dark ride has been engaged
    bool enable_upside_down;  // dark ride mode is enabled (10 seconds after fault)
    time_t upside_down_fault_timer;

    // Feature: Flywheel
    bool flywheel_abort;

    // Feature: Reverse Stop
    float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
    time_t reverse_timer;

    // Feature: Soft Start
    float softstart_pid_limit, softstart_ramp_step_size;

    uint64_t odometer;

    // Feature: RC Move (control via app while idle)
    int rc_steps;
    int rc_counter;
    float rc_current_target;
    float rc_current;

    HapticFeedback haptic_feedback;
    DataRecord data_record;

    Konami flywheel_konami;
    Konami headlights_on_konami;
    Konami headlights_off_konami;
} Data;
