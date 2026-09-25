// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    INPUTTILT_NONE = 0,
    INPUTTILT_UART,
    INPUTTILT_PPM
} FLOAT_INPUTTILT_REMOTE_TYPE;

typedef enum {
    PARKING_BRAKE_ALWAYS = 0,
    PARKING_BRAKE_IDLE,
    PARKING_BRAKE_NEVER
} ParkingBrakeMode;

typedef enum {
    LED_Type_None = 0,
    LED_Type_RGB,
    LED_Type_RGBW,
    LED_Type_External_Module,
} LEDType;

typedef struct {
    uint8_t led_type;
    uint8_t led_status_count;
    uint8_t led_forward_count;
    uint8_t led_rear_count;
    uint8_t led_brightness;
    uint8_t led_brightness_idle;
    uint8_t led_mode;
    uint8_t led_mode_idle;
    uint8_t led_status_brightness;
    uint8_t led_status_mode;
} CfgLeds;

typedef struct {
    bool swap_footpad_adcs;
} CfgHardware;

typedef struct {
    uint16_t frequency;
    float strength;
} CfgHapticTone;

typedef struct {
    CfgHapticTone duty;
    CfgHapticTone error;
    CfgHapticTone vibrate;
    float min_strength;
    float strength_curvature;
    float max_strength_speed;
    float duty_solid_offset;
    float current_threshold;
} CfgHapticFeedback;

typedef struct {
    float time_constant;
    float on_speed_time_constant;
    float off_speed_time_constant;
    float on_speed_limit;
    float off_speed_limit;
} CfgSetpointFilter;

typedef struct {
    float time_constant;
} CfgSetpointFilterSimple;

typedef struct {
    CfgSetpointFilter filter;
} CfgTorqueTilt;

typedef struct {
    CfgSetpointFilter filter;
    float transition_boost;
} CfgATR;

typedef struct {
    CfgSetpointFilterSimple filter;
} CfgTurnTilt;

typedef struct {
    CfgSetpointFilterSimple filter;
    uint8_t max_move_speed;
} CfgRemote;

typedef struct {
    bool enabled;
    float cell_lv_threshold;
    float cell_hv_threshold;
    float cell_balance_threshold;
    int8_t cell_lt_threshold;
    int8_t cell_ht_threshold;
    int8_t bms_ht_threshold;
} CfgBMS;

typedef struct {
    bool is_default;
} CfgMeta;

typedef struct {
    bool disabled;
    float kp;
    float ki;
    float kp2;
    float mahony_kp;
    float mahony_kp_roll;
    float kp_brake;
    float kp2_brake;
    float fault_pitch;
    float fault_roll;
    float fault_adc1;
    float fault_adc2;
    uint16_t fault_delay_pitch;
    uint16_t fault_delay_roll;
    uint16_t fault_delay_switch_half;
    uint16_t fault_delay_switch_full;
    uint16_t fault_adc_half_erpm;
    bool fault_is_dual_switch;
    bool fault_moving_fault_disabled;
    bool enable_quickstop;
    bool fault_darkride_enabled;
    bool fault_reversestop_enabled;
    float tiltback_duty_angle;
    float tiltback_duty_speed;
    float tiltback_duty;
    uint8_t tiltback_speed;
    float tiltback_hv_angle;
    float tiltback_hv_speed;
    float tiltback_hv;
    float tiltback_lv_angle;
    float tiltback_lv_speed;
    float tiltback_lv;
    float tiltback_return_speed;
    bool persistent_fatal_error;
    float tiltback_constant;
    uint16_t tiltback_constant_erpm;
    float tiltback_variable;
    float tiltback_variable_max;
    uint16_t tiltback_variable_erpm;
    FLOAT_INPUTTILT_REMOTE_TYPE inputtilt_remote_type;
    float inputtilt_angle_limit;
    bool inputtilt_invert_throttle;
    float inputtilt_deadband;
    float remote_throttle_grace_period;
    float noseangling_speed;
    float startup_pitch_tolerance;
    float startup_roll_tolerance;
    float startup_speed;
    float startup_click_current;
    bool startup_simplestart_enabled;
    bool startup_pushstart_enabled;
    bool startup_dirtylandings_enabled;
    ParkingBrakeMode parking_brake_mode;
    float brake_current;
    float ki_limit;
    float booster_angle;
    float booster_ramp;
    float booster_current;
    float brkbooster_angle;
    float brkbooster_ramp;
    float brkbooster_current;
    float torquetilt_start_current;
    float torquetilt_angle_limit;
    float torquetilt_strength;
    float torquetilt_strength_regen;
    float atr_strength_up;
    float atr_strength_down;
    float atr_threshold_up;
    float atr_threshold_down;
    float atr_speed_boost;
    float atr_angle_limit;
    float atr_filter;
    float atr_amps_accel_ratio;
    float atr_amps_decel_ratio;
    float braketilt_strength;
    float braketilt_lingering;
    float turntilt_strength;
    float turntilt_angle_limit;
    float turntilt_start_angle;
    uint16_t turntilt_start_erpm;
    uint16_t turntilt_erpm_boost;
    uint16_t turntilt_erpm_boost_end;
    int turntilt_yaw_aggregate;
    bool is_beeper_enabled;
    bool is_dutybeep_enabled;
    bool is_footbeep_enabled;

    CfgTorqueTilt torque_tilt;
    CfgATR atr;
    CfgTurnTilt turn_tilt;
    CfgRemote remote;

    CfgHapticFeedback haptic;
    CfgBMS bms;
    CfgLeds leds;
    CfgHardware hardware;

    CfgMeta meta;
} RefloatConfig;

// DATATYPES_H_
#endif
