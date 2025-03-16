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

#include "conf/datatypes.h"
#include "utils.h"
#include "vesc_c_if.h"

#include <math.h>

#define TONE_LENGTH 0.1f

void haptic_feedback_init(HapticFeedback *hf) {
    hf->type_playing = HAPTIC_FEEDBACK_NONE;
    hf->tone_timer = 0;
    hf->is_playing = false;
}

void haptic_feedback_configure(HapticFeedback *hf, const RefloatConfig *cfg) {
    hf->cfg = &cfg->haptic;
    hf->duty_solid_threshold = cfg->tiltback_duty + hf->cfg->duty_solid_offset;

    // pre-calculate the coefficients of the polynomial given the configured max strength speed
    float m = hf->cfg->max_strength_speed > 0 ? hf->cfg->max_strength_speed : 1;
    float a = hf->cfg->min_strength;
    hf->str_poly_b = (1 - hf->cfg->strength_curvature) * (1 - a) / m;
    hf->str_poly_c = (1 - a - hf->str_poly_b * m) / (m * m);
}

static HapticFeedbackType haptic_feedback_get_type(
    const HapticFeedback *hf, const State *state, const MotorData *md
) {
    // TODO: Ideally we don't even do pushback in handtest, as it can be confusing
    if (state->state != STATE_RUNNING || state->mode == MODE_HANDTEST) {
        return HAPTIC_FEEDBACK_NONE;
    }

    switch (state->sat) {
    case SAT_PB_DUTY:
        if (md->duty_cycle > hf->duty_solid_threshold) {
            return HAPTIC_FEEDBACK_DUTY_CONTINUOUS;
        } else {
            return HAPTIC_FEEDBACK_DUTY;
        }
    case SAT_PB_TEMPERATURE:
        return HAPTIC_FEEDBACK_ERROR_TEMPERATURE;
    case SAT_PB_LOW_VOLTAGE:
    case SAT_PB_HIGH_VOLTAGE:
        return HAPTIC_FEEDBACK_ERROR_VOLTAGE;
    default:
        break;
    }

    if (hf->cfg->current_threshold > 0.0f &&
        motor_data_get_current_saturation(md) > hf->cfg->current_threshold) {
        return HAPTIC_FEEDBACK_DUTY_CONTINUOUS;
    }

    return HAPTIC_FEEDBACK_NONE;
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

static inline float strength_scale(const HapticFeedback *hf, float speed) {
    return min(hf->cfg->min_strength + hf->str_poly_b * speed + hf->str_poly_c * speed * speed, 1);
}

static inline void foc_play_tone(int channel, float freq, float voltage) {
    if (!VESC_IF->foc_play_tone) {
        return;
    }

    VESC_IF->foc_play_tone(channel, freq, voltage);
}

void haptic_feedback_update(
    HapticFeedback *hf, MotorControl *mc, const State *state, const MotorData *md, const Time *time
) {
    HapticFeedbackType type_to_play = haptic_feedback_get_type(hf, state, md);

    if (type_to_play != hf->type_playing && timer_older(time, hf->tone_timer, TONE_LENGTH)) {
        hf->type_playing = type_to_play;
        timer_refresh(time, &hf->tone_timer);
    }

    bool should_be_playing = false;
    if (hf->type_playing != HAPTIC_FEEDBACK_NONE) {
        uint8_t beats = get_beats(hf->type_playing);
        if (beats == 0) {
            should_be_playing = true;
        } else {
            float period = TONE_LENGTH * beats;
            float tone_time = fmodf(timer_age(time, hf->tone_timer), period);
            uint8_t beat = floorf(tone_time / TONE_LENGTH);
            uint8_t off_beat = beats > 2 ? beats - 2 : 0;

            should_be_playing = beat % 2 == 0 && (off_beat == 0 || beat != off_beat);
        }
    }

    if (hf->is_playing && !should_be_playing) {
        foc_play_tone(0, 1, 0.0f);
        motor_control_stop_tone(mc);
        hf->is_playing = false;
    } else if (should_be_playing) {
        const CfgHapticTone *tone = get_haptic_tone(hf);
        if (tone->strength > 0.0f) {
            foc_play_tone(0, tone->frequency, tone->strength * strength_scale(hf, md->speed));
        }

        if (hf->cfg->vibrate.strength > 0.0f) {
            motor_control_play_tone(
                mc,
                hf->cfg->vibrate.frequency,
                hf->cfg->vibrate.strength * strength_scale(hf, md->speed)
            );
        }

        hf->is_playing = true;
    }
}
