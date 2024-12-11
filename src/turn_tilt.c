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

#include "turn_tilt.h"

#include "lib/utils.h"

#include <math.h>

void turn_tilt_init(TurnTilt *tt) {
    tt->boost_per_erpm = 0.0f;
    ema_init(&tt->yaw_change);
    smooth_setpoint_init(&tt->setpoint);

    turn_tilt_reset(tt);
}

void turn_tilt_reset(TurnTilt *tt) {
    tt->last_yaw_angle = 0.0f;
    ema_reset(&tt->yaw_change, 0.0f);
    tt->yaw_aggregate = 0.0f;

    tt->target = 0.0f;
    smooth_setpoint_reset(&tt->setpoint, 0.0f);
}

void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *config, float frequency) {
    ema_configure(&tt->yaw_change, 25.0f, frequency);

    tt->boost_per_erpm =
        (float) config->turntilt_erpm_boost / 100.0 / config->turntilt_erpm_boost_end;

    smooth_setpoint_configure_strengths(
        &tt->setpoint,
        config->turn_tilt.filter.strength,
        config->turn_tilt.filter.on_ease_in_strength,
        config->turn_tilt.filter.off_ease_in_strength,
        config->turntilt_speed,
        config->turntilt_speed,
        config->turntilt_speed,
        config->turntilt_speed,
        frequency
    );
}

void turn_tilt_aggregate(TurnTilt *tt, const IMU *imu, float dt) {
    float new_change = imu->yaw - tt->last_yaw_angle;
    if (new_change < -180.0f) {
        new_change += 360.0f;
    } else if (new_change > 180.0f) {
        new_change -= 360.0f;
    }

    tt->last_yaw_angle = imu->yaw;

    // limit change to avoid overreactions at low speed
    ema_update(&tt->yaw_change, clampf(new_change / dt, -72.0f, 72.0f));

    // clear the aggregate yaw whenever we change direction
    if (sign(tt->yaw_change.value) != sign(tt->yaw_aggregate)) {
        tt->yaw_aggregate = 0;
    }

    // don't count tiny yaw changes towards aggregate
    if (fabsf(tt->yaw_change.value) > 30.0f) {
        tt->yaw_aggregate += new_change;
    }
}

static void calculate_turn_tilt_target(
    TurnTilt *tt, const MotorData *md, const RefloatConfig *config
) {
    if (config->turntilt_strength == 0) {
        return;
    }

    float abs_yaw_change = fabsf(tt->yaw_change.value);
    float abs_yaw_aggregate = fabsf(tt->yaw_aggregate);

    // Minimum threshold based on
    // a) minimum degrees per second (yaw/turn increment)
    // b) minimum yaw aggregate (to filter out wiggling on uneven road)
    if (abs_yaw_aggregate < config->turntilt_start_angle || abs_yaw_change < 30.0f) {
        tt->target = 0;
    } else {
        // Calculate desired angle
        tt->target = abs_yaw_change * LOOP_HERTZ_COMPAT_RECIP * config->turntilt_strength;

        // Apply speed scaling
        float boost;
        if (md->abs_erpm < config->turntilt_erpm_boost_end) {
            boost = 1.0 + md->abs_erpm * tt->boost_per_erpm;
        } else {
            boost = 1.0 + (float) config->turntilt_erpm_boost / 100.0;
        }
        tt->target *= boost;

        // Increase turntilt based on aggregate yaw change (at most: double it)
        float aggregate_damper = 1.0;
        if (md->abs_erpm < 2000) {
            aggregate_damper = 0.5;
        }
        boost = 1 + aggregate_damper * abs_yaw_aggregate / config->turntilt_yaw_aggregate;
        boost = fminf(boost, 2);
        tt->target *= boost;

        tt->target =
            clampf(tt->target, -config->turntilt_angle_limit, config->turntilt_angle_limit);

        // Disable below erpm threshold otherwise add directionality
        if (md->abs_erpm < config->turntilt_start_erpm) {
            tt->target = 0;
        } else {
            tt->target *= md->erpm_sign;
        }
    }
}

void turn_tilt_update(
    TurnTilt *tt, const MotorData *motor, const RefloatConfig *config, bool wheelslip, float dt
) {
    if (!wheelslip) {
        calculate_turn_tilt_target(tt, motor, config);
    } else {
        tt->target *= 0.99;
    }

    smooth_setpoint_update(&tt->setpoint, tt->target, dt, motor->forward);
}
