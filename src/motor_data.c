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

#include "lib/utils.h"

#include "vesc_c_if.h"

#include <math.h>

void motor_data_init(MotorData *m) {
    m->erpm = 0.0f;
    m->abs_erpm = 0.0f;
    m->last_erpm = 0.0f;
    m->erpm_sign = 1;
    ema_init(&m->abs_erpm_smooth);

    m->speed = 0.0f;

    m->current = 0.0f;
    m->dir_current = 0.0f;
    biquad_init(&m->filt_current);
    m->braking = false;
    m->forward = true;

    m->duty_raw = 0.0f;
    ema_init(&m->duty_cycle);
    sma_init(&m->acceleration);

    ema_init(&m->batt_current);
    m->batt_voltage = 0.0f;

    m->mosfet_temp = 0.0f;
    m->motor_temp = 0.0f;

    m->current_min = 0.0f;
    m->current_max = 0.0f;
    m->battery_current_min = 0.0f;
    m->battery_current_max = 0.0f;
    m->mosfet_temp_max = 0.0f;
    m->motor_temp_max = 0.0f;
    m->duty_max_with_margin = 0.0f;
    m->lv_threshold = 0.0f;
    m->hv_threshold = 0.0f;

    motor_data_reset(m);
}

void motor_data_destroy(MotorData *m) {
    sma_destroy(&m->acceleration);
}

void motor_data_reset(MotorData *m) {
    ema_reset(&m->duty_cycle, 0.0f);
    sma_reset(&m->acceleration);
    biquad_reset(&m->filt_current);
}

void motor_data_refresh_motor_config(MotorData *m, float lv_threshold, float hv_threshold) {
    uint8_t battery_cells = VESC_IF->get_cfg_int(CFG_PARAM_si_battery_cells);
    if (battery_cells > 0) {
        if (lv_threshold < 10) {
            lv_threshold *= battery_cells;
        }
        if (hv_threshold < 10) {
            hv_threshold *= battery_cells;
        }
    }

    m->lv_threshold = lv_threshold;
    m->hv_threshold = hv_threshold;

    // min motor current is a positive value here!
    m->current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
    m->current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    m->battery_current_min = VESC_IF->get_cfg_float(CFG_PARAM_l_in_current_min);
    m->battery_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_in_current_max);
    m->mosfet_temp_max = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3;
    m->motor_temp_max = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3;
    m->duty_max_with_margin = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty) - 0.05;
}

void motor_data_configure(MotorData *m, float current_cutoff_freq, float frequency) {
    ema_configure(&m->abs_erpm_smooth, 10.0f, frequency);

    // setting cutoff freq to 0 used to turn off the filtering
    if (current_cutoff_freq < 1) {
        current_cutoff_freq = 20;
    }
    biquad_configure(&m->filt_current, BQ_LOWPASS, current_cutoff_freq, frequency);

    ema_configure(&m->duty_cycle, 1.0f, frequency);
    sma_configure(&m->acceleration, 8.0f, frequency);
    ema_configure(&m->batt_current, 1.0f, frequency);
}

void motor_data_update(MotorData *m, float dt) {
    m->erpm = VESC_IF->mc_get_rpm();
    m->abs_erpm = fabsf(m->erpm);
    m->erpm_sign = sign(m->erpm);
    ema_update(&m->abs_erpm_smooth, m->abs_erpm);

    // TODO mc_get_speed() calculates speed from erpm using the full formula,
    // including four divisions. In theory multiplying by a single constant is
    // enough, we just need to calculate the constant (and keep it up to date
    // when motor config changes, there's no way to know, we'll have to poll).
    // And it's only possible on 6.05+.
    m->speed = VESC_IF->mc_get_speed() * 3.6;

    m->current = VESC_IF->mc_get_tot_current_filtered();
    m->dir_current = VESC_IF->mc_get_tot_current_directional_filtered();
    m->braking = m->current < 0;
    if (m->abs_erpm > 250 || m->filt_current.value < 30.0f) {
        m->forward = m->erpm >= 0.0f;
    } else {
        m->forward = m->filt_current.value >= 0.0f;
    }

    m->duty_raw = fabsf(VESC_IF->mc_get_duty_cycle_now());
    ema_update(&m->duty_cycle, m->duty_raw);

    sma_update(&m->acceleration, (m->erpm - m->last_erpm) / dt);
    m->last_erpm = m->erpm;

    biquad_update(&m->filt_current, m->dir_current);

    ema_update(&m->batt_current, VESC_IF->mc_get_tot_current_in_filtered());
    m->batt_voltage = VESC_IF->mc_get_input_voltage_filtered();

    m->mosfet_temp = VESC_IF->mc_temp_fet_filtered();
    m->motor_temp = VESC_IF->mc_temp_motor_filtered();
}

void motor_data_evaluate_alerts(const MotorData *m, AlertTracker *at, const Time *time) {
    unused(m);

    mc_fault_code fault_code = VESC_IF->mc_get_fault();
    if (fault_code != FAULT_CODE_NONE) {
        alert_tracker_add(at, time, ALERT_FW_FAULT, fault_code);
    }
}

float motor_data_get_current_saturation(const MotorData *m) {
    float motor_saturation =
        fabsf(m->filt_current.value) / (m->braking ? m->current_min : m->current_max);
    float battery_saturation = m->batt_current.value /
        (m->batt_current.value < 0 ? m->battery_current_min : m->battery_current_max);

    return max(motor_saturation, battery_saturation);
}
