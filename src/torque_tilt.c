// Copyright 2022 Dado Mista
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

#include "torque_tilt.h"

#include "utils.h"

#include <math.h>

void torque_tilt_reset(TorqueTilt *tt) {
    tt->offset = 0;
    tt->ramped_step_size = 0;

    smooth_target_reset(&tt->smooth_target, 0.0f);
    ema_filter_reset(&tt->ema_target, 0.0f, 0.0f);
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *config) {
    tt->on_step_size = config->torquetilt_on_speed / config->hertz;
    tt->off_step_size = config->torquetilt_off_speed / config->hertz;

    smooth_target_configure(
        &tt->smooth_target,
        &config->target_filter,
        config->torquetilt_on_speed,
        config->torquetilt_off_speed,
        config->hertz
    );
    ema_filter_configure(
        &tt->ema_target,
        &config->target_filter,
        config->torquetilt_on_speed,
        config->torquetilt_off_speed
    );
}

void torque_tilt_update(
    TorqueTilt *tt, const MotorData *motor, const RefloatConfig *config, float dt
) {
    float strength =
        motor->braking ? config->torquetilt_strength_regen : config->torquetilt_strength;

    // Take abs motor current, subtract start offset, and take the max of that
    // with 0 to get the current above our start threshold (absolute). Then
    // multiply it by "power" to get our desired angle, and min with the limit
    // to respect boundaries. Finally multiply it by motor current sign to get
    // directionality back.
    float target_offset =
        fminf(
            fmaxf((fabsf(motor->atr_filtered_current) - config->torquetilt_start_current), 0) *
                strength,
            config->torquetilt_angle_limit
        ) *
        sign(motor->atr_filtered_current);

    float step_size = 0;
    if ((tt->offset - target_offset > 0 && target_offset > 0) ||
        (tt->offset - target_offset < 0 && target_offset < 0)) {
        step_size = tt->off_step_size;
    } else {
        step_size = tt->on_step_size;
    }

    if (motor->abs_erpm < 500) {
        step_size /= 2;
    }

    if (config->target_filter.type == SFT_NONE) {
        rate_limitf(&tt->offset, target_offset, step_size);
    } else if (config->target_filter.type == SFT_EMA3) {
        ema_filter_update(&tt->ema_target, target_offset, dt);
        tt->offset = tt->ema_target.value;
    } else if (config->target_filter.type == SFT_THREE_STAGE) {
        smooth_target_update(&tt->smooth_target, target_offset);
        tt->offset = tt->smooth_target.value;
    } else {
        // Smoothen changes in tilt angle by ramping the step size
        smooth_rampf(&tt->offset, &tt->ramped_step_size, target_offset, step_size, 0.04, 1.5);
    }
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->offset *= 0.995;
}
