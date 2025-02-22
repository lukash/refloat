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

#include "motor_control.h"

#include "conf/datatypes.h"
#include "utils.h"
#include "vesc_c_if.h"

void motor_control_init(MotorControl *mc) {
    mc->disabled = false;
    mc->current_requested = false;
    mc->requested_current = 0.0f;
    mc->click_counter = 0;
    mc->brake_timeout = 0.0f;
    mc->parking_brake_active = false;
}

void motor_control_configure(MotorControl *mc, const RefloatConfig *config) {
    mc->brake_current = config->brake_current;
    mc->click_current = config->startup_click_current;
    mc->parking_brake_mode = config->parking_brake_mode;
}

void motor_control_request_current(MotorControl *mc, float current) {
    mc->current_requested = true;
    mc->requested_current = current;
}

void motor_control_apply(MotorControl *mc, float abs_erpm, RunState state, float time) {
    if (state == STATE_DISABLED) {
        if (!mc->disabled) {
            // set 0A only once to reset any previously-set current, then stop touching the motor
            VESC_IF->mc_set_current(0.0f);
            mc->disabled = true;
        }
        return;
    } else {
        mc->disabled = false;
    }

    if (mc->parking_brake_mode == PARKING_BRAKE_ALWAYS ||
        (mc->parking_brake_mode == PARKING_BRAKE_IDLE && state != STATE_RUNNING && abs_erpm < 50)) {
        mc->parking_brake_active = true;
    } else if (mc->parking_brake_mode == PARKING_BRAKE_NEVER || state == STATE_RUNNING) {
        mc->parking_brake_active = false;
    }

    if (mc->click_counter) {
        mc->current_requested = true;

        // Generate alternate pulses to produce distinct "click"
        mc->click_counter--;
        if ((mc->click_counter & 0x1) == 0) {
            mc->requested_current -= mc->click_current;
        } else {
            mc->requested_current += mc->click_current;
        }
    }

    // Reset VESC Firmware safety timeout
    VESC_IF->timeout_reset();

    // BEWARE: Some sort of motor control must always be set before returning from this function
    if (mc->current_requested) {
        // Keep modulation on for 50ms in case we request close-to-0 current
        VESC_IF->mc_set_current_off_delay(0.05f);
        VESC_IF->mc_set_current(mc->requested_current);
    } else {
        // Brake logic
        if (abs_erpm > ERPM_MOVING_THRESHOLD) {
            mc->brake_timeout = time + 1.0f;
        }

        if (time > mc->brake_timeout) {
            // Release the motor by setting zero current
            VESC_IF->mc_set_current(0.0f);
            return;
        }

        if (mc->parking_brake_active && abs_erpm < 2000) {
            // Duty Cycle mode has better holding power (phase-shorting on 6.05)
            VESC_IF->mc_set_duty(0);
        } else {
            // Use brake current over certain ERPM to avoid MOSFET overcurrent
            VESC_IF->mc_set_brake_current(mc->brake_current);
        }
    }

    mc->current_requested = false;
    mc->requested_current = 0.0f;
}
