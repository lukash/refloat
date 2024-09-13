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

#include "conf/datatypes.h"
#include "state.h"

typedef enum {
    HAPTIC_FEEDBACK_NONE = 0,
    HAPTIC_FEEDBACK_DUTY,
    HAPTIC_FEEDBACK_DUTY_CONTINUOUS,
    HAPTIC_FEEDBACK_ERROR_TEMPERATURE,
    HAPTIC_FEEDBACK_ERROR_VOLTAGE,
} HapticFeedbackType;

typedef struct {
    const CfgHapticFeedback *cfg;

    HapticFeedbackType type_playing;
    float start_time;
    bool is_playing;
} HapticFeedback;

void haptic_feedback_init(HapticFeedback *hf);

void haptic_feedback_configure(HapticFeedback *hf, const CfgHapticFeedback *cfg);

void haptic_feedback_update(
    HapticFeedback *hf, const State *state, float duty_cycle, float current_time
);
