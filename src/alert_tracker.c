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

#include "alert_tracker.h"

static uint32_t alert_id_to_mask(AlertId alert_id) {
    return 1 << (alert_id - 1);
}

static const AlertProperties alert_properties[] = {
    [ALERT_FW_FAULT] = {.type = ATYPE_FATAL},
};

const AlertProperties *alert_tracker_properties(AlertId alert_id) {
    return &alert_properties[alert_id];
}

void alert_tracker_init(AlertTracker *at) {
    at->persistent_fatal_error = true;

    at->active_alert_mask = 0;
    at->new_active_alert_mask = 0;
    at->fw_fault_code = FAULT_CODE_NONE;
    at->fatal_error = false;

    circular_buffer_init(
        &at->alert_buffer, sizeof(AlertRecord), ALERT_TRACKER_SIZE, &at->alert_buffer_array
    );
}

void alert_tracker_configure(AlertTracker *at, const RefloatConfig *config) {
    at->persistent_fatal_error = config->persistent_fatal_error;
}

void alert_tracker_add(AlertTracker *at, const Time *time, uint8_t id, uint8_t code) {
    uint32_t mask = alert_id_to_mask(ALERT_FW_FAULT);
    bool already_active = at->active_alert_mask & mask;
    if (id == ALERT_FW_FAULT) {
        already_active = already_active && code == at->fw_fault_code;
        at->fw_fault_code = code;
    }

    if (!already_active) {
        AlertRecord alert = {.time = time->now, .id = id, .code = code, .active = true};
        circular_buffer_push(&at->alert_buffer, &alert);
    }

    if (alert_tracker_properties(id)->type == ATYPE_FATAL) {
        at->fatal_error = true;
    }

    at->new_active_alert_mask |= mask;
}

void alert_tracker_finalize(AlertTracker *at, const Time *time) {
    // Check for alerts that ended and create a record of it
    bool clear_fatal = !at->persistent_fatal_error;
    for (uint8_t id = 1; id <= ALERT_LAST; ++id) {
        uint32_t mask = alert_id_to_mask(id);
        if (at->active_alert_mask & mask && !(at->new_active_alert_mask & mask)) {
            AlertRecord alert = {.time = time->now, .id = id, .code = 0, .active = false};
            circular_buffer_push(&at->alert_buffer, &alert);
            if (id == ALERT_FW_FAULT) {
                at->fw_fault_code = 0;
            }
        }

        if (at->active_alert_mask & mask && alert_tracker_properties(id)->type == ATYPE_FATAL) {
            clear_fatal = false;
        }
    }

    at->active_alert_mask = at->new_active_alert_mask;
    at->new_active_alert_mask = 0;

    at->fatal_error &= !clear_fatal;
}

bool alert_tracker_is_alert_active(AlertTracker *at, AlertId alert) {
    return at->active_alert_mask & alert_id_to_mask(alert);
}

void alert_tracker_clear_fatal(AlertTracker *at) {
    at->fatal_error = false;
}
