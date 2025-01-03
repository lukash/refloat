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

#include "haptic_feedback.h"

#include "vesc_c_if.h"

#include <math.h>

void haptic_feedback_init(HapticFeedback *hf) {
    hf->type_playing = HAPTIC_FEEDBACK_NONE;
    hf->start_time = 0.0f;
    hf->is_playing = false;
}

void haptic_feedback_configure(HapticFeedback *hf, const CfgHapticFeedback *cfg) {
    hf->cfg = cfg;
}

static HapticFeedbackType state_to_haptic_type(const State *state) {
    if (state->state != STATE_RUNNING) {
        return HAPTIC_FEEDBACK_NONE;
    }

    switch (state->sat) {
    case SAT_PB_DUTY:
        return HAPTIC_FEEDBACK_DUTY;
    case SAT_PB_TEMPERATURE:
        return HAPTIC_FEEDBACK_ERROR_TEMPERATURE;
    case SAT_PB_LOW_VOLTAGE:
    case SAT_PB_HIGH_VOLTAGE:
        return HAPTIC_FEEDBACK_ERROR_VOLTAGE;
    default:
        return HAPTIC_FEEDBACK_NONE;
    }
}

// Returns the number of "beats" per period of a given tone. Tones are played
// on even beats and if there are more than two beats, the last beat is
// skipped, giving a certain number of "beeps" followed by a pause.
static uint8_t get_beats(HapticFeedbackType type) {
    switch (type) {
    case HAPTIC_FEEDBACK_DUTY:
        return 2;
    case HAPTIC_FEEDBACK_DUTY_CONTINUOUS:
        return 0;
    case HAPTIC_FEEDBACK_ERROR_TEMPERATURE:
        return 6;
    case HAPTIC_FEEDBACK_ERROR_VOLTAGE:
        return 8;
    case HAPTIC_FEEDBACK_NONE:
        break;
    }

    return 0;
}

static const CfgHapticTone *get_haptic_tone(const HapticFeedback *hf) {
    switch (hf->type_playing) {
    case HAPTIC_FEEDBACK_DUTY:
    case HAPTIC_FEEDBACK_DUTY_CONTINUOUS:
        return &hf->cfg->duty;
    case HAPTIC_FEEDBACK_ERROR_TEMPERATURE:
    case HAPTIC_FEEDBACK_ERROR_VOLTAGE:
        return &hf->cfg->error;
    case HAPTIC_FEEDBACK_NONE:
        break;
    }

    return 0;
}

void haptic_feedback_update(
    HapticFeedback *hf, const State *state, float duty_cycle, float current_time
) {
    if (!VESC_IF->foc_play_tone) {
        return;
    }

    HapticFeedbackType type_to_play = state_to_haptic_type(state);
    if (type_to_play == HAPTIC_FEEDBACK_DUTY && duty_cycle > hf->cfg->duty_solid_threshold) {
        type_to_play = HAPTIC_FEEDBACK_DUTY_CONTINUOUS;
    }

    float tone_length = hf->cfg->tone_length * 0.001f;

    if (type_to_play != hf->type_playing && current_time - hf->start_time > tone_length) {
        hf->type_playing = type_to_play;
        hf->start_time = current_time;
    }

    bool should_be_playing = false;
    if (hf->type_playing != HAPTIC_FEEDBACK_NONE) {
        uint8_t beats = get_beats(hf->type_playing);
        if (beats == 0) {
            should_be_playing = true;
        } else {
            float period = tone_length * beats;
            float time = fmodf(current_time - hf->start_time, period);
            uint8_t beat = floorf(time / tone_length);
            uint8_t off_beat = beats > 2 ? beats - 2 : 0;

            should_be_playing = beat % 2 == 0 && (off_beat == 0 || beat != off_beat);
        }
    }

    if (hf->is_playing && !should_be_playing) {
        VESC_IF->foc_play_tone(0, 1, 0.0f);
        VESC_IF->foc_play_tone(1, 1, 0.0f);
        hf->is_playing = false;
    } else if (!hf->is_playing && should_be_playing) {
        const CfgHapticTone *tone = get_haptic_tone(hf);
        VESC_IF->foc_play_tone(0, tone->frequency, tone->voltage);
        VESC_IF->foc_play_tone(1, hf->cfg->vibrate.frequency, hf->cfg->vibrate.voltage);
        hf->is_playing = true;
    }
}
