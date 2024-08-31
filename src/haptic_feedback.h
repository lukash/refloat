// Copyright 2024 Syler Clayton
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

typedef enum {
    HAPTIC_FEEDBACK_NONE = 0,
    HAPTIC_FEEDBACK_BEEP,
    HAPTIC_FEEDBACK_DUTY,
    HAPTIC_FEEDBACK_ERROR
} HAPTIC_FEEDBACK_TYPE;

typedef struct {
    HAPTIC_FEEDBACK_TYPE type;
    float volt;
    int freq, peroid;
} HapticFeedbackConfig;

typedef struct {
    HapticFeedbackConfig duty, error, haptic_feedback_state;
    float vibrate_volt, duty_solid;
    int vibrate_freq;
    float timer, alt_timer, note_duration;
    bool tone_in_progress;
    int counter, beep_num;
} HapticFeedback;

void haptic_feedback_configure(HapticFeedback *haptic_feedback, RefloatConfig *float_conf);

void haptic_feedback_reset(HapticFeedback *haptic_feedback, float current_time);

void haptic_feedback_play_single_tone(float freq, float volt, int channel);

void haptic_feedback_play_two_tones(HapticFeedback *haptic_feedback);

void haptic_feedback_check_tiltback(
    HapticFeedback *haptic_feedback, State *state, float current_time, float duty_cycle
);

void haptic_feedback_update(HapticFeedback *haptic_feedback, float current_time);

void haptic_feedback_beep(
    HapticFeedback *haptic_feedback,
    State *state,
    float current_time,
    float freq,
    float volt,
    float duration,
    int beep_num
);
