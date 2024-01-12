/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	INPUTTILT_NONE = 0,
	INPUTTILT_UART,
	INPUTTILT_PPM
} FLOAT_INPUTTILT_REMOTE_TYPE;

typedef struct {
	float float_version;
	float float_disable;
	float kp;
	float ki;
	float kp2;
	float mahony_kp;
	float kp_brake;
	float kp2_brake;
	uint16_t kp_brake_erpm;
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
	bool fault_darkride_enabled;
	bool fault_reversestop_enabled;
	float tiltback_duty_angle;
	float tiltback_duty_speed;
	float tiltback_duty;
	float tiltback_hv_angle;
	float tiltback_hv_speed;
	float tiltback_hv;
	float tiltback_lv_angle;
	float tiltback_lv_speed;
	float tiltback_lv;
	float tiltback_return_speed;
	float tiltback_constant;
	uint16_t tiltback_constant_erpm;
	float tiltback_variable;
	float tiltback_variable_max;
	uint16_t tiltback_variable_erpm;
	FLOAT_INPUTTILT_REMOTE_TYPE inputtilt_remote_type;
	float inputtilt_speed;
	float inputtilt_angle_limit;
	uint16_t inputtilt_smoothing_factor;
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
	float atr_torque_offset;
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
	float dark_pitch_offset;
	bool is_buzzer_enabled;
	bool is_dutybuzz_enabled;
	bool is_footbuzz_enabled;
	bool is_surgebuzz_enabled;
	float surge_duty_start;
	float surge_angle;
} float_config;

// DATATYPES_H_
#endif
