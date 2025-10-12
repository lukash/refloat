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

#include "pid.h"

#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void pid_init(PID *pid) {
    pid->p = 0;
    pid->i = 0;
    pid->rate_p = 0;

    pid->kp_brake_scale = 1.0;
    pid->kp2_brake_scale = 1.0;
    pid->kp_accel_scale = 1.0;
    pid->kp2_accel_scale = 1.0;
}

void pid_update(
    PID *pid,
    float setpoint,
    const MotorData *md,
    const IMU *imu,
    const State *state,
    const RefloatConfig *config
) {
    pid->p = setpoint - imu->balance_pitch;
    pid->i = pid->i + pid->p * config->ki;

    // I term filter
    if (config->ki_limit > 0 && fabsf(pid->i) > config->ki_limit) {
        pid->i = config->ki_limit * sign(pid->i);
    }
    // quickly ramp down integral component during reverse stop
    if (state->sat == SAT_REVERSESTOP) {
        pid->i *= 0.9;
    }

    // brake scale coefficient smoothing
    if (md->abs_erpm < 500) {
        // all scaling should roll back to 1.0 when near a stop for smooth transitions
        pid->kp_brake_scale = 0.01 + 0.99 * pid->kp_brake_scale;
        pid->kp2_brake_scale = 0.01 + 0.99 * pid->kp2_brake_scale;
        pid->kp_accel_scale = 0.01 + 0.99 * pid->kp_accel_scale;
        pid->kp2_accel_scale = 0.01 + 0.99 * pid->kp2_accel_scale;
    } else if (md->erpm > 0) {
        // rolling forward - brakes transition to scaled values
        pid->kp_brake_scale = 0.01 * config->kp_brake + 0.99 * pid->kp_brake_scale;
        pid->kp2_brake_scale = 0.01 * config->kp2_brake + 0.99 * pid->kp2_brake_scale;
        pid->kp_accel_scale = 0.01 + 0.99 * pid->kp_accel_scale;
        pid->kp2_accel_scale = 0.01 + 0.99 * pid->kp2_accel_scale;
    } else {
        // rolling backward, NEW brakes (we use kp_accel) transition to scaled values
        pid->kp_brake_scale = 0.01 + 0.99 * pid->kp_brake_scale;
        pid->kp2_brake_scale = 0.01 + 0.99 * pid->kp2_brake_scale;
        pid->kp_accel_scale = 0.01 * config->kp_brake + 0.99 * pid->kp_accel_scale;
        pid->kp2_accel_scale = 0.01 * config->kp2_brake + 0.99 * pid->kp2_accel_scale;
    }

    pid->p *= config->kp * (pid->p > 0 ? pid->kp_accel_scale : pid->kp_brake_scale);

    pid->rate_p = -imu->pitch_rate * config->kp2;
    pid->rate_p *= pid->rate_p > 0 ? pid->kp2_accel_scale : pid->kp2_brake_scale;
}

void pid_reset_integral(PID *pid) {
    pid->i = 0;
}
