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

#include "lib/utils.h"

void remote_init(Remote *remote) {
    remote->input = 0;
    smooth_setpoint_init(&remote->setpoint);

    remote_reset(remote);
}

void remote_reset(Remote *remote) {
    smooth_setpoint_reset(&remote->setpoint, 0.0f);
}

void remote_configure(Remote *remote, const RefloatConfig *config, float frequency) {
    smooth_setpoint_configure(
        &remote->setpoint,
        config->remote.filter.strength,
        config->remote.filter.on_ease_in_strength,
        config->remote.filter.off_ease_in_strength,
        config->inputtilt_speed,
        config->inputtilt_speed,
        config->inputtilt_speed,
        config->inputtilt_speed,
        frequency
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

    // The `forward` argument doesn't matter, as up and down speeds are the same
    smooth_setpoint_update(&remote->setpoint, target, dt, true);
}
