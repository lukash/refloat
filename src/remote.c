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

#include "remote.h"

#include "utils.h"

void remote_init(Remote *remote) {
    remote->input = 0;
    remote->ramped_step_size = 0;
    remote->setpoint = 0;
}

void remote_configure(Remote *remote, const RefloatConfig *config) {
    remote->step_size = config->inputtilt_speed / config->hertz;
}

void remote_input(Remote *remote, const RefloatConfig *config) {
    bool connected = false;
    float value = 0;

    switch (config->inputtilt_remote_type) {
    case (INPUTTILT_PPM):
        value = VESC_IF->get_ppm();
        connected = VESC_IF->get_ppm_age() < 1;
        break;
    case (INPUTTILT_UART): {
        remote_state remote = VESC_IF->get_remote_state();
        value = remote.js_y;
        connected = remote.age_s < 1;
        break;
    }
    case (INPUTTILT_NONE):
        break;
    }

    if (!connected) {
        remote->input = 0;
        return;
    }

    float deadband = config->inputtilt_deadband;
    if (fabsf(value) < deadband) {
        value = 0.0;
    } else {
        value = sign(value) * (fabsf(value) - deadband) / (1 - deadband);
    }

    if (config->inputtilt_invert_throttle) {
        value = -value;
    }

    remote->input = value;
}

void remote_update(Remote *remote, const State *state, const RefloatConfig *config) {
    float target = remote->input * config->inputtilt_angle_limit;

    if (state->darkride) {
        target = -target;
    }

    float target_diff = target - remote->setpoint;

    // Smoothen changes in tilt angle by ramping the step size
    const float smoothing_factor = 0.02;

    // Within X degrees of Target Angle, start ramping down step size
    if (fabsf(target_diff) < 2.0f) {
        // Target step size is reduced the closer to center you are (needed for smoothly
        // transitioning away from center)
        remote->ramped_step_size = smoothing_factor * remote->step_size * target_diff / 2 +
            (1 - smoothing_factor) * remote->ramped_step_size;
        // Linearly ramped down step size is provided as minimum to prevent overshoot
        float centering_step_size =
            fminf(fabsf(remote->ramped_step_size), fabsf(target_diff / 2) * remote->step_size) *
            sign(target_diff);
        if (fabsf(target_diff) < fabsf(centering_step_size)) {
            remote->setpoint = target;
        } else {
            remote->setpoint += centering_step_size;
        }
    } else {
        // Ramp up step size until the configured tilt speed is reached
        remote->ramped_step_size = smoothing_factor * remote->step_size * sign(target_diff) +
            (1 - smoothing_factor) * remote->ramped_step_size;
        remote->setpoint += remote->ramped_step_size;
    }
}
