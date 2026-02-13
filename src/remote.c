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

void remote_reset(Remote *remote) {
    remote->setpoint = 0;
    remote->ramped_step_size = 0;

    smooth_target_reset(&remote->smooth_target, 0.0f);
}

void remote_configure(Remote *remote, const RefloatConfig *config) {
    remote->step_size = config->inputtilt_speed / config->hertz;

    // Hardcoded target filter for remote
    static const CfgTargetFilter remote_target_filter = {
        .alpha = 0.020, .in_alpha_away = 0.050, .in_alpha_back = 0.015
    };

    smooth_target_configure(
        &remote->smooth_target,
        &remote_target_filter,
        config->inputtilt_speed,
        config->inputtilt_speed,
        config->hertz
    );
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

void remote_update(Remote *remote, const State *state, const RefloatConfig *config, float dt) {
    float target = remote->input * config->inputtilt_angle_limit;

    if (state->darkride) {
        target = -target;
    }

    smooth_target_update(&remote->smooth_target, target);
    remote->setpoint = remote->smooth_target.value;
}
