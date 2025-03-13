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

#include "state.h"

void state_init(State *state) {
    state->state = STATE_STARTUP;
    state->mode = MODE_NORMAL;
    state->sat = SAT_NONE;
    state->stop_condition = STOP_NONE;
    state->charging = false;
    state->wheelslip = false;
    state->darkride = false;
}

void state_stop(State *state, StopCondition stop_condition) {
    state->state = STATE_READY;
    state->stop_condition = stop_condition;
    state->wheelslip = false;
}

void state_engage(State *state) {
    state->state = STATE_RUNNING;
    state->sat = SAT_CENTERING;
    state->stop_condition = STOP_NONE;
}

void state_set_disabled(State *state, bool disabled) {
    if (state->state != STATE_RUNNING && disabled) {
        state->state = STATE_DISABLED;
    } else if (state->state == STATE_DISABLED && !disabled) {
        state->state = STATE_STARTUP;
    }
}

uint8_t state_compat(const State *state) {
    if (state->charging) {
        return 14;  // CHARGING
    }

    switch (state->state) {
    case STATE_DISABLED:
        return 15;  // DISABLED
    case STATE_STARTUP:
        return 0;  // STARTUP
    case STATE_READY:
        switch (state->stop_condition) {
        case STOP_NONE:
            return 11;  // FAULT_STARTUP
        case STOP_PITCH:
            return 6;  // FAULT_ANGLE_PITCH
        case STOP_ROLL:
            return 7;  // FAULT_ANGLE_ROLL
        case STOP_SWITCH_HALF:
            return 8;  // FAULT_SWITCH_HALF
        case STOP_SWITCH_FULL:
            return 9;  // FAULT_SWITCH_FULL
        case STOP_REVERSE_STOP:
            return 12;  // FAULT_REVERSE
        case STOP_QUICKSTOP:
            return 13;  // FAULT_QUICKSTOP
        }
        return 11;  // FAULT_STARTUP
    case STATE_RUNNING:
        if (state->sat > SAT_PB_DUTY) {
            return 2;  // RUNNING_TILTBACK
        } else if (state->wheelslip) {
            return 3;  // RUNNING_WHEELSLIP
        } else if (state->darkride) {
            return 4;  // RUNNING_UPSIDEDOWN
        } else if (state->mode == MODE_FLYWHEEL) {
            return 5;  // RUNNING_FLYWHEEL
        }
        return 1;  // RUNNING
    }
    return 0;  // STARTUP
}

uint8_t sat_compat(const State *state) {
    switch (state->sat) {
    case SAT_CENTERING:
        return 0;  // CENTERING
    case SAT_REVERSESTOP:
        return 1;  // REVERSESTOP
    case SAT_NONE:
        return 2;  // TILTBACK_NONE
    case SAT_PB_DUTY:
        return 3;  // TILTBACK_DUTY
    case SAT_PB_HIGH_VOLTAGE:
    case SAT_PB_BMS_CONNECTION:
        return 4;  // TILTBACK_HV
    case SAT_PB_LOW_VOLTAGE:
        return 5;  // TILTBACK_LV
    case SAT_PB_TEMPERATURE:
        return 6;  // TILTBACK_TEMP
    }
    return 0;  // CENTERING
}
