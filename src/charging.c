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

#include "charging.h"

#include "conf/buffer.h"

#include "vesc_c_if.h"

void charging_init(Charging *charging) {
    charging->timer = 0.0f;
    charging->voltage = 0.0f;
    charging->current = 0.0f;
}

void charging_timeout(Charging *charging, State *state) {
    // Timeout the charging state after 5 seconds if the charging module didn't update
    if (VESC_IF->system_time() - charging->timer > 5.0f) {
        state->charging = false;
    }
}

void charging_state_request(Charging *charging, uint8_t *buffer, size_t len, State *state) {
    // Expecting 6 bytes:
    // -magic nr: must be 151
    // -charging: 1/0 aka true/false
    // -voltage: 16bit float divided by 10
    // -current: 16bit float divided by 10
    if (len < 6) {
        return;
    }
    int32_t idx = 0;

    uint8_t magicnr = buffer[idx++];
    if (magicnr != 151) {
        return;
    }

    state->charging = buffer[idx++] > 0;
    charging->timer = VESC_IF->system_time();

    if (state->charging) {
        charging->voltage = buffer_get_float16(buffer, 10, &idx);
        charging->current = buffer_get_float16(buffer, 10, &idx);
    } else {
        charging->voltage = 0;
        charging->current = 0;
    }
}
