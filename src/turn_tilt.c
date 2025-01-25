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

#include "utils.h"

#include <math.h>

void turn_tilt_reset(TurnTilt *tt) {
    tt->last_yaw_angle = 0;
    tt->last_yaw_change = 0;
    tt->yaw_change = 0;
    tt->yaw_aggregate = 0;

    tt->target = 0;
    tt->setpoint = 0;
}

void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *config) {
    tt->step_size = config->turntilt_speed / config->hertz;
    tt->boost_per_erpm =
        (float) config->turntilt_erpm_boost / 100.0 / config->turntilt_erpm_boost_end;
}

void turn_tilt_aggregate(TurnTilt *tt, const IMU *imu) {
    float new_change = imu->yaw - tt->last_yaw_angle;
    bool unchanged = false;

    // exact 0's only happen when the IMU is not updating between loops,
    // yaw flips signs at 180, ignore those changes
    if (new_change == 0 || fabsf(new_change) > 100) {
        new_change = tt->last_yaw_change;
        unchanged = true;
    }
    tt->last_yaw_change = new_change;
    tt->last_yaw_angle = imu->yaw;

    // limit change to avoid overreactions at low speed
    tt->yaw_change = 0.8 * tt->yaw_change + 0.2 * clampf(new_change, -0.10, 0.10);

    // clear the aggregate yaw whenever we change direction
    if (sign(tt->yaw_change) != sign(tt->yaw_aggregate)) {
        tt->yaw_aggregate = 0;
    }

    tt->abs_yaw_change = fabsf(tt->yaw_change);
    // don't count tiny yaw changes towards aggregate
    if (tt->abs_yaw_change > 0.04 && !unchanged) {
        tt->yaw_aggregate += tt->yaw_change;
    }
}

void turn_tilt_update(
    TurnTilt *tt,
    const MotorData *md,
    const ATR *atr,
    float balance_pitch,
    float noseangling,
    const RefloatConfig *config
) {
    if (config->turntilt_strength == 0) {
        return;
    }

    float abs_yaw_aggregate = fabsf(tt->yaw_aggregate);

    // Minimum threshold based on
    // a) minimum degrees per second (yaw/turn increment)
    // b) minimum yaw aggregate (to filter out wiggling on uneven road)
    if (abs_yaw_aggregate < config->turntilt_start_angle || tt->abs_yaw_change < 0.04) {
        tt->target = 0;
    } else {
        // Calculate desired angle
        tt->target = tt->abs_yaw_change * config->turntilt_strength;

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

        // Limit angle to max angle
        if (tt->target > 0) {
            tt->target = fminf(tt->target, config->turntilt_angle_limit);
        } else {
            tt->target = fmaxf(tt->target, -config->turntilt_angle_limit);
        }

        // Disable below erpm threshold otherwise add directionality
        if (md->abs_erpm < config->turntilt_start_erpm) {
            tt->target = 0;
        } else {
            tt->target *= md->erpm_sign;
        }

        // ATR interference: Reduce target during moments of high torque response
        float atr_min = 2;
        float atr_max = 5;
        if (sign(atr->target) != sign(tt->target)) {
            // further reduced turntilt during moderate to steep downhills
            atr_min = 1;
            atr_max = 4;
        }
        if (fabsf(atr->target) > atr_min) {
            // Start scaling turntilt when ATR>2, down to 0 turntilt for ATR > 5 degrees
            float atr_scaling = (atr_max - fabsf(atr->target)) / (atr_max - atr_min);
            if (atr_scaling < 0) {
                atr_scaling = 0;
                // during heavy torque response clear the yaw aggregate too
                tt->yaw_aggregate = 0;
            }
            tt->target *= atr_scaling;
        }
        if (fabsf(balance_pitch - noseangling) > 4) {
            // no setpoint changes during heavy acceleration or braking
            tt->target = 0;
            tt->yaw_aggregate = 0;
        }
    }

    // Move towards target limited by max speed
    rate_limitf(&tt->setpoint, tt->target, tt->step_size);
}
