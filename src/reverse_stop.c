// Copyright 2025 Lukas Hrazky
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

#include "reverse_stop.h"

#include "lib/transitions.h"
#include "lib/utils.h"
#include <math.h>

#define REVERSE_STOP_DISTANCE 0.5f
#define TARGET_STOP_ANGLE 15.0f
#define DISTANCE_PER_DEGREE (REVERSE_STOP_DISTANCE / TARGET_STOP_ANGLE)
#define TIMER_ANGLE_THRESHOLD (TARGET_STOP_ANGLE / 2)

// Distance of travel for one motor step, could be calculated, for the purposes
// of Reverse Stop an approximate value is enough. Calculated as: wheel_dia *
// pi / (3 * motor_poles)
#define MOTOR_STEP_DIST 0.01f

void reverse_stop_init(ReverseStop *rs) {
    ema_init(&rs->progress);
    reverse_stop_reset(rs, 0.0f);
}

void reverse_stop_reset(ReverseStop *rs, float distance) {
    rs->start_setpoint = 0.0f;
    rs->target_setpoint = 0.0f;
    rs->start_distance = distance;
    rs->current_distance = 0.0f;
    rs->target_distance = 0.0f;
    ema_reset(&rs->progress, 1.0f);
}

void reverse_stop_configure(ReverseStop *rs, float frequency) {
    ema_configure(&rs->progress, 1.0f, frequency);
}

void reverse_stop_update(
    ReverseStop *rs, float distance, float setpoint, const Time *time, bool enabled
) {
    if (!enabled && rs->progress.value >= 1.0f) {
        rs->start_distance = distance;
        return;
    }

    float new_distance = distance - rs->start_distance;
    float distance_diff = (new_distance - rs->current_distance) * sign(rs->target_distance);

    if (distance_diff < -2 * MOTOR_STEP_DIST) {
        rs->target_setpoint = rs->target_setpoint > 0.0f ? 0.0f : TARGET_STOP_ANGLE;
        rs->start_setpoint = setpoint;
        rs->start_distance = distance;
        rs->current_distance = 0.0f;
        rs->target_distance = fabsf(rs->start_setpoint - rs->target_setpoint) * DISTANCE_PER_DEGREE;
        // determine sign according to target_setpoint for the case we somehow go into
        // a reverse stop from a negative setpoint and then start going back out of it
        if (rs->target_setpoint > 0.0f) {
            rs->target_distance *= -1;
        }

        float new_progress = 0.0f;
        if (fabsf(rs->target_distance) < MOTOR_STEP_DIST) {
            new_progress = 1.0f;
            rs->target_distance = 0.0f;
        }
        ema_reset(&rs->progress, new_progress);

        return;
    }

    if (rs->progress.value >= 1.0f) {
        if (distance_diff > 0.0f) {
            rs->start_distance = distance;
        }
        return;
    }

    if (distance_diff > 0.0f) {
        rs->current_distance = new_distance;
    }

    ema_update(&rs->progress, rs->current_distance / rs->target_distance);

    if (rs->progress.value >= 1.0f) {
        rs->target_distance = 0.0f;
        rs->current_distance = 0.0f;
    }

    // update the timer if returning from reverse or the angle is above threshold
    if (rs->target_setpoint == 0.0f || setpoint < TIMER_ANGLE_THRESHOLD) {
        timer_refresh(time, &rs->timer);
    }
}

float reverse_stop_setpoint(ReverseStop *rs) {
    float prog = smoothstep(rs->progress.value);
    return rs->start_setpoint + prog * (rs->target_setpoint - rs->start_setpoint);
}

bool reverse_stop_active(ReverseStop *rs) {
    return rs->target_setpoint > 0.0f || rs->progress.value < 1.0f;
}

bool reverse_stop_stop(ReverseStop *rs, const Time *time) {
    float progress = rs->progress.value;

    // the timer only starts aging below a certain setpoint angle threshold
    // the further we get progress-wise, the sooner we disengage: 3s at 0%, 1s at 100%
    float time_threshold = 3.0f - 2 * progress;
    if (timer_older(time, rs->timer, time_threshold)) {
        return true;
    }

    return rs->target_setpoint > 0.0f && progress >= 1.0f;
}
