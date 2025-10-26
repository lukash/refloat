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

#include "utils.h"

#include <math.h>

uint32_t rnd(uint32_t seed) {
    return seed * 1664525u + 1013904223u;
}

void rate_limitf(float *value, float target, float step) {
    if (fabsf(target - *value) < step) {
        *value = target;
    } else if (target - *value > 0) {
        *value += step;
    } else {
        *value -= step;
    }
}

// Smoothen changes in tilt angle by ramping the step size
// smooth_center_window: Sets the angle away from Target that step size begins ramping down
void smooth_rampf(
    float *value,
    float *ramped_step,
    float target,
    float step,
    float smoothing_factor,
    float smooth_center_window
) {
    float tiltback_target_diff = target - *value;

    // Within X degrees of Target Angle, start ramping down step size
    if (fabsf(tiltback_target_diff) < smooth_center_window) {
        // Target step size is reduced the closer to center you are (needed for smoothly
        // transitioning away from center)
        *ramped_step = (smoothing_factor * step * (tiltback_target_diff / 2)) +
            ((1 - smoothing_factor) * *ramped_step);
        // Linearly ramped down step size is provided as minimum to prevent overshoot
        float centering_step = fminf(fabsf(*ramped_step), fabsf(tiltback_target_diff / 2) * step) *
            sign(tiltback_target_diff);
        if (fabsf(tiltback_target_diff) < fabsf(centering_step)) {
            *value = target;
        } else {
            *value += centering_step;
        }
    } else {
        // Ramp up step size until the configured tilt speed is reached
        *ramped_step = (smoothing_factor * step * sign(tiltback_target_diff)) +
            ((1 - smoothing_factor) * *ramped_step);
        *value += *ramped_step;
    }
}

float clampf(float value, float min, float max) {
    const float m = value < min ? min : value;
    return m > max ? max : m;
}
