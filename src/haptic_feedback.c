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

#include "haptic_feedback.h"

void haptic_feedback_configure(HapticFeedback *haptic_feedback, RefloatConfig *float_conf) {
    haptic_feedback->haptic_feedback_intensity = float_conf->haptic_feedback_intensity;
    haptic_feedback->haptic_feedback_min = float_conf->haptic_feedback_min;
    haptic_feedback->haptic_feedback_duty = float_conf->haptic_feedback_duty;
    haptic_feedback->haptic_feedback_hv = float_conf->haptic_feedback_hv;
    haptic_feedback->haptic_feedback_lv = float_conf->haptic_feedback_lv;
    haptic_feedback->haptic_feedback_temp = float_conf->haptic_feedback_temp;
}

void haptic_feedback_reset(HapticFeedback *haptic_feedback, float current_time) {
    haptic_feedback->haptic_tone_in_progress = false;
    haptic_feedback->haptic_timer = current_time;
    haptic_feedback->applied_haptic_current = 0;
    haptic_feedback->haptic_mode = 0;
    haptic_feedback->haptic_counter = 0;
}

void haptic_tone(
    HapticFeedback *haptic_feedback,
    float *pid_value,
    float motor_timeout_s,
    float current_time,
    float abs_erpm,
    float startup_click_current,
    float note_duration,
    bool disengage
) {
    if (haptic_feedback->haptic_tone_in_progress || disengage) {
        haptic_feedback->haptic_counter += 1;

        float haptic_current = fminf(20, haptic_feedback->haptic_feedback_intensity);
        // small periods (1,2) produce audible tone, higher periods produce vibration
        int haptic_period = haptic_feedback->haptic_type;
        if (haptic_feedback->haptic_type == HAPTIC_FEEDBACK_ALTERNATING) {
            haptic_period = 1;
        }

        // alternate frequencies, depending on "mode"
        haptic_period += haptic_feedback->haptic_mode;

        if (disengage) {
            // This is to emulate the equivalent of "stop click"
            haptic_current = fmaxf(3, startup_click_current * 0.8);
            haptic_current = fminf(10, haptic_current);
            haptic_period = 0;
        } else if ((abs_erpm < 10000) && (haptic_current > 5)) {
            // scale high currents down to as low as 5A for lower erpms
            haptic_current =
                fmaxf(haptic_feedback->haptic_feedback_min, abs_erpm / 10000 * haptic_current);
        }

        if (haptic_feedback->haptic_counter > haptic_period) {
            haptic_feedback->haptic_counter = 0;
        }

        if (haptic_feedback->haptic_counter == 0) {
            if (haptic_feedback->applied_haptic_current > 0) {
                haptic_feedback->applied_haptic_current = -haptic_current;
            } else {
                haptic_feedback->applied_haptic_current = haptic_current;
            }

            if (fabsf(haptic_feedback->haptic_timer - current_time) > note_duration) {
                haptic_feedback->haptic_tone_in_progress = false;
                if (disengage) {
                    haptic_feedback->haptic_mode += 1;
                } else {
                    if (haptic_feedback->haptic_type == HAPTIC_FEEDBACK_ALTERNATING) {
                        haptic_feedback->haptic_mode = 5 - haptic_feedback->haptic_mode;
                    } else {
                        haptic_feedback->haptic_mode = 1 - haptic_feedback->haptic_mode;
                    }
                }

                haptic_feedback->haptic_timer = current_time;
            }
        }
    } else {
        haptic_feedback->haptic_mode = 0;
        haptic_feedback->haptic_counter = 0;
        haptic_feedback->haptic_timer = current_time;
        haptic_feedback->applied_haptic_current = 0;
    }

    // if (VESC_IF->foc_stop_audio && (!haptic_feedback->haptic_tone_in_progress || disengage)) {
    //     VESC_IF->foc_stop_audio(true);
    // }

    if (disengage) {
        // For now we'll keep the disengage click the same functionality
        if (startup_click_current > 0) {
            set_current(motor_timeout_s, haptic_feedback->applied_haptic_current);
        }
    } else {
        //  check which mode is being used here based on if the firmware is >= 6.05. Note: We're
        //  gonna have to maybe call haptic_feedback_update from main differently too, to signal to
        //  stop playing a tone (for 6.05 API) if the board is in an not running state?
        // if (VESC_IF->foc_play_tone && haptic_feedback->haptic_tone_in_progress) {
        //    // Map HAPTIC_FEEDBACK_TYPE to specific frequencies, handle haptic_mode for
        //    // switching between alternating freq, pick out voltage between 1.5-3v (dado said max
        //    // he's tried is 6v)
        //    VESC_IF->foc_play_tone(0, 495.0, 3.0);
        //} else {
        // Modulate the current onto the pid
        *pid_value += haptic_feedback->applied_haptic_current;
        //}
    }
}

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
) {
    if (state->mode == MODE_FLYWHEEL) {
        return;
    }
    if (((state->sat > SAT_NONE) && (state->state == STATE_RUNNING))) {

        if (state->sat == SAT_PB_DUTY) {
            haptic_feedback->haptic_type = haptic_feedback->haptic_feedback_duty;
        } else if (state->sat == SAT_PB_HIGH_VOLTAGE) {
            haptic_feedback->haptic_type = haptic_feedback->haptic_feedback_hv;
        } else if (state->sat == SAT_PB_LOW_VOLTAGE) {
            haptic_feedback->haptic_type = haptic_feedback->haptic_feedback_lv;
        } else if (state->sat == SAT_PB_TEMPERATURE) {
            haptic_feedback->haptic_type = haptic_feedback->haptic_feedback_temp;
        } else {
            haptic_feedback->haptic_type = HAPTIC_FEEDBACK_NONE;
        }

        // This kicks it off till at least one ~300ms tone is completed
        if (haptic_feedback->haptic_type != HAPTIC_FEEDBACK_NONE) {
            haptic_feedback->haptic_tone_in_progress = true;
        }
    }

    haptic_tone(
        haptic_feedback,
        pid_value,
        motor_timeout_s,
        current_time,
        abs_erpm,
        startup_click_current,
        note_duration,
        disengage
    );
}
