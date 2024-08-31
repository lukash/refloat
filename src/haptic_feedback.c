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

#include "haptic_feedback.h"

void haptic_feedback_configure(HapticFeedback *haptic_feedback, RefloatConfig *float_conf) {
    haptic_feedback->duty_solid = float_conf->haptic_duty_solid;
    haptic_feedback->duty.volt = float_conf->haptic_duty_volt;
    haptic_feedback->duty.freq = float_conf->haptic_duty_freq;
    haptic_feedback->error.volt = float_conf->haptic_error_volt;
    haptic_feedback->error.freq = float_conf->haptic_error_freq;
    haptic_feedback->vibrate_volt = float_conf->haptic_vibrate_volt;
    haptic_feedback->vibrate_freq = float_conf->haptic_vibrate_freq;
}

void haptic_feedback_reset(HapticFeedback *haptic_feedback, float current_time) {
    if (haptic_feedback->tone_in_progress && VESC_IF->foc_stop_audio) {
        VESC_IF->foc_stop_audio(true);
    }
    haptic_feedback->tone_in_progress = false;
    haptic_feedback->timer = current_time;
    haptic_feedback->alt_timer = current_time;
    haptic_feedback->haptic_feedback_state.type = HAPTIC_FEEDBACK_NONE;
    haptic_feedback->counter = 0;
    haptic_feedback->duty.peroid = 200;  // 200 ms
    haptic_feedback->error.peroid = 200;  // 200 ms
    haptic_feedback->note_duration = 0;
    haptic_feedback->beep_num = 0;
}

void haptic_feedback_play_single_tone(float freq, float volt, int channel) {
    if (!VESC_IF->foc_play_tone) {
        return;
    }
    if ((volt > 0) && (freq > 0.0)) {
        VESC_IF->foc_play_tone(channel, freq, volt);
    }
}

void haptic_feedback_play_two_tones(HapticFeedback *haptic_feedback) {
    haptic_feedback_play_single_tone(
        haptic_feedback->haptic_feedback_state.freq, haptic_feedback->haptic_feedback_state.volt, 0
    );
    haptic_feedback_play_single_tone(
        haptic_feedback->vibrate_freq, haptic_feedback->vibrate_volt, 1
    );
}

void haptic_feedback_check_tiltback(
    HapticFeedback *haptic_feedback, State *state, float current_time, float duty_cycle
) {
    if (!(VESC_IF->foc_play_tone && VESC_IF->foc_stop_audio)) {
        return;
    }

    if (state->mode == MODE_FLYWHEEL) {
        haptic_feedback_reset(haptic_feedback, current_time);
        return;
    }

    if (state->sat >= SAT_PB_DUTY) {
        HAPTIC_FEEDBACK_TYPE prev = haptic_feedback->haptic_feedback_state.type;
        if (state->sat == SAT_PB_DUTY) {
            haptic_feedback->haptic_feedback_state = haptic_feedback->duty;
            if ((duty_cycle >= haptic_feedback->duty_solid)) {
                if (haptic_feedback->counter % 2 == 0) {
                    haptic_feedback->alt_timer = current_time;
                }
            }
        } else {
            haptic_feedback->haptic_feedback_state = haptic_feedback->error;
            switch (state->sat) {
            case SAT_PB_TEMPERATURE:
                // Careful
                if (haptic_feedback->haptic_feedback_state.peroid > 0) {
                    haptic_feedback->haptic_feedback_state.peroid /= 2;
                }
                __attribute__((fallthrough));
            case SAT_PB_LOW_VOLTAGE:
                __attribute__((fallthrough));
            case SAT_PB_HIGH_VOLTAGE:
                if (haptic_feedback->counter == 3) {
                    haptic_feedback->haptic_feedback_state.peroid *= 2;
                } else if (haptic_feedback->counter == 4) {
                    haptic_feedback->counter = 0;
                }
                break;
            default:
                break;
            }
        }
        // Reset state if a different type of event is coming in
        if (haptic_feedback->haptic_feedback_state.type != prev) {
            haptic_feedback_reset(haptic_feedback, current_time);
        }

        if (!haptic_feedback->tone_in_progress) {
            haptic_feedback_play_two_tones(haptic_feedback);
            haptic_feedback->alt_timer = current_time;
            haptic_feedback->tone_in_progress = true;
            haptic_feedback->note_duration = 0.3;
        }
        haptic_feedback->timer = current_time;
    }
}

void haptic_feedback_update(HapticFeedback *haptic_feedback, float current_time) {
    if (!(VESC_IF->foc_play_tone && VESC_IF->foc_stop_audio)) {
        return;
    }

    if (haptic_feedback->tone_in_progress) {
        if ((fabsf(haptic_feedback->timer - current_time) > haptic_feedback->note_duration) ||
            (haptic_feedback->haptic_feedback_state.type == HAPTIC_FEEDBACK_BEEP &&
             haptic_feedback->beep_num == 0)) {
            haptic_feedback_reset(haptic_feedback, current_time);
        } else if ((haptic_feedback->haptic_feedback_state.peroid > 0) &&
                   (fabsf(haptic_feedback->alt_timer - current_time) >
                    haptic_feedback->haptic_feedback_state.peroid / 1000.0)) {
            haptic_feedback->alt_timer = current_time;
            // update for special modes
            haptic_feedback->counter++;
            if (haptic_feedback->counter % 2 == 0) {
                if (haptic_feedback->haptic_feedback_state.type == HAPTIC_FEEDBACK_BEEP) {
                    haptic_feedback_play_single_tone(
                        haptic_feedback->haptic_feedback_state.freq,
                        haptic_feedback->haptic_feedback_state.volt,
                        0
                    );
                    haptic_feedback->beep_num--;
                } else {
                    haptic_feedback_play_two_tones(haptic_feedback);
                }
            } else {
                VESC_IF->foc_stop_audio(false);
            }
        }
    }
}

void haptic_feedback_beep(
    HapticFeedback *haptic_feedback,
    State *state,
    float current_time,
    float freq,
    float volt,
    float duration,
    int beep_num
) {
    if (!VESC_IF->foc_play_tone) {
        return;
    }
    if (state->mode == MODE_FLYWHEEL) {
        haptic_feedback_reset(haptic_feedback, current_time);
        return;
    }
    if (!haptic_feedback->tone_in_progress) {
        haptic_feedback->haptic_feedback_state.freq = freq;
        haptic_feedback->haptic_feedback_state.volt = volt;
        haptic_feedback->haptic_feedback_state.peroid = duration * 1000;
        haptic_feedback->haptic_feedback_state.type = HAPTIC_FEEDBACK_BEEP;
        haptic_feedback->note_duration = duration * beep_num * 2;
        haptic_feedback->beep_num = beep_num * 2;
        haptic_feedback_play_single_tone(freq, volt, 0);
        haptic_feedback->tone_in_progress = true;
        haptic_feedback->timer = current_time;
        haptic_feedback->alt_timer = current_time;
    }
}
