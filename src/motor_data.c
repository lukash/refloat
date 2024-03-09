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

#include "motor_data.h"

#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void motor_data_reset(MotorData *m) {
    m->duty_smooth = 0;

    m->acceleration = 0;
    m->accel_idx = 0;
    for (int i = 0; i < 40; i++) {
        m->accel_history[i] = 0;
    }

    biquad_reset(&m->atr_current_biquad);
}

void motor_data_configure(MotorData *m, float frequency) {
    if (frequency > 0) {
        biquad_configure(&m->atr_current_biquad, BQ_LOWPASS, frequency);
        m->atr_filter_enabled = true;
    } else {
        m->atr_filter_enabled = false;
    }
}

void motor_data_update(MotorData *m) {
    m->erpm = VESC_IF->mc_get_rpm();
    m->abs_erpm = fabsf(m->erpm);
    m->erpm_sign = sign(m->erpm);

    m->current = VESC_IF->mc_get_tot_current_directional_filtered();
    m->braking = m->abs_erpm > 250 && sign(m->current) != m->erpm_sign;

    m->duty_cycle = fabsf(VESC_IF->mc_get_duty_cycle_now());
    m->duty_smooth = m->duty_smooth * 0.9f + m->duty_cycle * 0.1f;

    float current_acceleration = m->erpm - m->last_erpm;
    m->last_erpm = m->erpm;

    m->acceleration += (current_acceleration - m->accel_history[m->accel_idx]) / ACCEL_ARRAY_SIZE;
    m->accel_history[m->accel_idx] = current_acceleration;
    m->accel_idx = (m->accel_idx + 1) % ACCEL_ARRAY_SIZE;

    if (m->atr_filter_enabled) {
        m->atr_filtered_current = biquad_process(&m->atr_current_biquad, m->current);
    } else {
        m->atr_filtered_current = m->current;
    }
}
