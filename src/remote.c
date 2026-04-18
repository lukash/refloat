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

#define MOVE_KP 1.2f
#define MOVE_KI 1.0f
#define MOVE_TORQUE_LIMIT 10.0f

#define REMOTE_TIMEOUT 0.5f

void remote_init(Remote *remote, const Time *time) {
    remote->input = 0;
    smooth_setpoint_init(&remote->setpoint);

    remote->command_input_time = time->now - REMOTE_TIMEOUT;

    remote_reset(remote);
}

void remote_reset(Remote *remote) {
    smooth_setpoint_reset(&remote->setpoint);
    remote->move_speed = NAN;
    remote->move_pid_i = 0.0f;
}

void remote_configure(Remote *remote, const RefloatConfig *config, float frequency) {
    smooth_setpoint_configure(
        &remote->setpoint,
        config->remote.filter.time_constant,
        config->remote.filter.on_speed_time_constant,
        config->remote.filter.off_speed_time_constant,
        0.2f,
        config->inputtilt_speed,
        config->inputtilt_speed,
        config->inputtilt_speed,
        config->inputtilt_speed,
        frequency
    );
}

void remote_input(Remote *remote, const Time *time, const RefloatConfig *config) {
    if (!timer_older(time, remote->command_input_time, REMOTE_TIMEOUT)) {
        // input is being set from a package command
        return;
    }

    bool connected = false;
    float value = 0;

    switch (config->inputtilt_remote_type) {
    case (INPUTTILT_PPM):
        value = VESC_IF->get_ppm();
        connected = VESC_IF->get_ppm_age() < REMOTE_TIMEOUT;
        break;
    case (INPUTTILT_UART): {
        remote_state remote = VESC_IF->get_remote_state();
        value = remote.js_y;
        connected = remote.age_s < REMOTE_TIMEOUT;
        break;
    }
    case (INPUTTILT_NONE):
        break;
    }

    if (!connected) {
        remote->input = 0;
        remote->move_speed = NAN;
        return;
    }

    float deadband = config->inputtilt_deadband;
    float value_abs = fabsf(value);
    if (value_abs < deadband) {
        value = 0.0;
    } else {
        value = sign(value) * (value_abs - deadband) / (1 - deadband);
    }

    // remote move logic
    if (value == 0.0f) {
        remote->move_speed = NAN;
    } else {
        if (config->remote.max_move_speed > 0 &&
            time_elapsed(time, disengage, config->remote_throttle_grace_period)) {
            remote->move_speed = value * config->remote.max_move_speed;
        }
    }

    if (config->inputtilt_invert_throttle) {
        value = -value;
    }

    remote->input = value;
}

void remote_command_input(
    Remote *remote, float value, const Time *time, const RefloatConfig *config
) {
    remote->input = value;
    if (time_elapsed(time, disengage, 2.0f)) {
        // default to a limit of 5 km/h if limit of 0 is configured for the remote
        float speed_max = config->remote.max_move_speed > 0 ? config->remote.max_move_speed : 5;
        remote->move_speed = value * speed_max;
    }

    timer_refresh(time, &remote->command_input_time);
}

float remote_get_move_torque(Remote *remote, float speed, float dt) {
    if (!isnan(remote->move_speed)) {
        float error = remote->move_speed - speed;

        remote->move_pid_i += MOVE_KI * error * dt;
        remote->move_pid_i = clampf(remote->move_pid_i, -MOVE_TORQUE_LIMIT, MOVE_TORQUE_LIMIT);

        return clampf(MOVE_KP * error + remote->move_pid_i, -MOVE_TORQUE_LIMIT, MOVE_TORQUE_LIMIT);
    } else {
        remote->move_pid_i = 0.0f;
        return NAN;
    }
}

void remote_update(Remote *remote, const State *state, const RefloatConfig *config, float dt) {
    float target = remote->input * config->inputtilt_angle_limit;

    if (state->darkride) {
        target = -target;
    }

    // The `forward` argument doesn't matter, as up and down speeds are the same
    smooth_setpoint_update(&remote->setpoint, target, true, 1.0f, dt);
}
