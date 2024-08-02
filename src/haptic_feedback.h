// Copyright 2024 Lukas Hrazky, Syler Clayton, Dado Mista
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
#include "conf/datatypes.h"
#include "state.h"
#include "utils.h"
#include <math.h>

typedef struct {
    float applied_haptic_current, haptic_timer;
    int haptic_counter, haptic_mode;
    HAPTIC_FEEDBACK_TYPE haptic_type;
    bool haptic_tone_in_progress;
    int haptic_feedback_intensity;
    int haptic_feedback_min;
    HAPTIC_FEEDBACK_TYPE haptic_feedback_duty;
    HAPTIC_FEEDBACK_TYPE haptic_feedback_hv;
    HAPTIC_FEEDBACK_TYPE haptic_feedback_lv;
    HAPTIC_FEEDBACK_TYPE haptic_feedback_temp;
} HapticFeedback;

void haptic_feedback_configure(HapticFeedback *haptic_feedback, RefloatConfig *float_conf);

void haptic_feedback_reset(HapticFeedback *haptic_feedback, float current_time);

void haptic_tone(
    HapticFeedback *haptic_feedback,
    float *pid_value,
    float motor_timeout_s,
    float current_time,
    float abs_erpm,
    float startup_click_current,
    float note_duration,
    bool disengage
);

void haptic_feedback_update(
    HapticFeedback *haptic_feedback,
    State *state,
    float *pid_value,
    float motor_timeout_s,
    float current_time,
    float abs_erpm,
    float startup_click_current,
    float note_duration,
    bool disengage
);
