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
    m->abs_erpm_smooth = 0;
    m->duty_raw = 0;

    m->acceleration = 0;
    m->accel_idx = 0;
    for (int i = 0; i < 40; i++) {
        m->accel_history[i] = 0;
    }

    biquad_reset(&m->current_biquad);
}

void motor_data_configure(MotorData *m, float frequency) {
    if (frequency > 0) {
        biquad_configure(&m->current_biquad, BQ_LOWPASS, frequency);
        m->current_filter_enabled = true;
    } else {
        m->current_filter_enabled = false;
    }

    // min motor current is a positive value here!
    m->current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
    m->current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    m->battery_current_min = VESC_IF->get_cfg_float(CFG_PARAM_l_in_current_min);
    m->battery_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_in_current_max);
}

void motor_data_update(MotorData *m) {
    m->erpm = VESC_IF->mc_get_rpm();
    m->abs_erpm = fabsf(m->erpm);
    m->abs_erpm_smooth = m->abs_erpm_smooth * 0.9 + m->abs_erpm * 0.1;
    m->erpm_sign = sign(m->erpm);

    // TODO mc_get_speed() calculates speed from erpm using the full formula,
    // including four divisions. In theory multiplying by a single constant is
    // enough, we just need to calculate the constant (and keep it up to date
    // when motor config changes, there's no way to know, we'll have to poll).
    m->speed = VESC_IF->mc_get_speed() * 3.6;

    m->current = VESC_IF->mc_get_tot_current_filtered();
    m->dir_current = VESC_IF->mc_get_tot_current_directional_filtered();
    m->braking = m->current < 0;

    m->duty_raw = fabsf(VESC_IF->mc_get_duty_cycle_now());
    m->duty_cycle += 0.01f * (m->duty_raw - m->duty_cycle);

    float current_acceleration = m->erpm - m->last_erpm;
    m->last_erpm = m->erpm;

    m->acceleration += (current_acceleration - m->accel_history[m->accel_idx]) / ACCEL_ARRAY_SIZE;
    m->accel_history[m->accel_idx] = current_acceleration;
    m->accel_idx = (m->accel_idx + 1) % ACCEL_ARRAY_SIZE;

    if (m->current_filter_enabled) {
        m->filt_current = biquad_process(&m->current_biquad, m->dir_current);
    } else {
        m->filt_current = m->dir_current;
    }

    m->batt_current += 0.01f * (VESC_IF->mc_get_tot_current_in_filtered() - m->batt_current);
    m->batt_voltage = VESC_IF->mc_get_input_voltage_filtered();

    m->mosfet_temp = VESC_IF->mc_temp_fet_filtered();
    m->motor_temp = VESC_IF->mc_temp_motor_filtered();
}

float motor_data_get_current_saturation(const MotorData *m) {
    float motor_saturation =
        fabsf(m->filt_current) / (m->braking ? m->current_min : m->current_max);
    float battery_saturation =
        m->batt_current / (m->batt_current < 0 ? m->battery_current_min : m->battery_current_max);

    return max(motor_saturation, battery_saturation);
}
