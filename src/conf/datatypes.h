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
    LED_MODE_OFF = 0,
    LED_MODE_INTERNAL = 0x1,
    LED_MODE_EXTERNAL = 0x2,
    LED_MODE_BOTH = 0x3,
} LedMode;

typedef enum {
    LED_PIN_B6 = 0,
    LED_PIN_B7,
    LED_PIN_C9,
    LED_PIN_LAST = LED_PIN_C9
} LedPin;

typedef enum {
    LED_PIN_CFG_PULLUP_TO_5V = 0,
    LED_PIN_CFG_NO_PULLUP
} LedPinConfig;

typedef enum {
    LED_COLOR_GRB = 0,
    LED_COLOR_GRBW,
    LED_COLOR_RGB,
    LED_COLOR_WRGB
} LedColorOrder;

typedef enum {
    LED_STRIP_ORDER_NONE = 0,
    LED_STRIP_ORDER_1ST,
    LED_STRIP_ORDER_2ND,
    LED_STRIP_ORDER_3RD
} LedStripOrder;

typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE_FULL,
    COLOR_WHITE_RGB,
    COLOR_WHITE_SINGLE,
    COLOR_RED,
    COLOR_FERRARI,
    COLOR_FLAME,
    COLOR_CORAL,
    COLOR_SUNSET,
    COLOR_SUNRISE,
    COLOR_GOLD,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_BANANA,
    COLOR_LIME,
    COLOR_ACID,
    COLOR_SAGE,
    COLOR_GREEN,
    COLOR_MINT,
    COLOR_TIFFANY,
    COLOR_CYAN,
    COLOR_STEEL,
    COLOR_SKY,
    COLOR_AZURE,
    COLOR_SAPPHIRE,
    COLOR_BLUE,
    COLOR_VIOLET,
    COLOR_AMETHYST,
    COLOR_MAGENTA,
    COLOR_PINK,
    COLOR_FUCHSIA,
    COLOR_LAVENDER,
} LedColor;

typedef enum {
    LED_ANIM_SOLID = 0,
    LED_ANIM_FADE,
    LED_ANIM_PULSE,
    LED_ANIM_STROBE,
    LED_ANIM_KNIGHT_RIDER,
    LED_ANIM_FELONY,
    LED_ANIM_RAINBOW_CYCLE,
    LED_ANIM_RAINBOW_FADE,
    LED_ANIM_RAINBOW_ROLL,
} LedAnimMode;

typedef enum {
    LED_TRANS_FADE = 0,
    LED_TRANS_FADE_OUT_IN,
    LED_TRANS_CIPHER,
    LED_TRANS_MONO_CIPHER,
} LedTransition;

typedef struct {
    float brightness;
    LedColor color1;
    LedColor color2;
    LedAnimMode mode;
    float speed;
} LedBar;

typedef struct {
    uint16_t idle_timeout;
    float duty_threshold;
    float red_bar_percentage;
    bool show_sensors_while_running;
    float brightness_headlights_on;
    float brightness_headlights_off;
} StatusBar;

typedef struct {
    bool on;
    bool headlights_on;

    LedTransition headlights_transition;
    LedTransition direction_transition;

    bool lights_off_when_lifted;
    bool status_on_front_when_lifted;

    LedBar headlights;
    LedBar taillights;
    LedBar front;
    LedBar rear;
    StatusBar status;
    LedBar status_idle;
} CfgLeds;

typedef struct {
    LedStripOrder order;
    uint8_t count;
    LedColorOrder color_order;
    bool reverse;
} CfgLedStrip;

typedef struct {
    LedMode mode;
    LedPin pin;
    LedPinConfig pin_config;
    CfgLedStrip status;
    CfgLedStrip front;
    CfgLedStrip rear;
} CfgHwLeds;

typedef struct {
    CfgHwLeds leds;
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
    uint16_t hertz;
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
    float inputtilt_speed;
    float inputtilt_angle_limit;
    bool inputtilt_invert_throttle;
    float inputtilt_deadband;
    float remote_throttle_current_max;
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
    float torquetilt_on_speed;
    float torquetilt_off_speed;
    float torquetilt_strength;
    float torquetilt_strength_regen;
    float atr_strength_up;
    float atr_strength_down;
    float atr_threshold_up;
    float atr_threshold_down;
    float atr_speed_boost;
    float atr_angle_limit;
    float atr_on_speed;
    float atr_off_speed;
    float atr_response_boost;
    float atr_transition_boost;
    float atr_filter;
    float atr_amps_accel_ratio;
    float atr_amps_decel_ratio;
    float braketilt_strength;
    float braketilt_lingering;
    float turntilt_strength;
    float turntilt_angle_limit;
    float turntilt_start_angle;
    uint16_t turntilt_start_erpm;
    float turntilt_speed;
    uint16_t turntilt_erpm_boost;
    uint16_t turntilt_erpm_boost_end;
    int turntilt_yaw_aggregate;
    bool is_beeper_enabled;
    bool is_dutybeep_enabled;
    bool is_footbeep_enabled;

    CfgHapticFeedback haptic;
    CfgBMS bms;
    CfgLeds leds;
    CfgHardware hardware;

    CfgMeta meta;
} RefloatConfig;

// DATATYPES_H_
#endif
