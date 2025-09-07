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

#pragma once

#include "conf/datatypes.h"
#include "lib/circular_buffer.h"
#include "time.h"

#define ALERT_TRACKER_SIZE 20

typedef enum {
    ATYPE_FATAL = 1,
    ATYPE_ERROR = 2,
    ATYPE_WARNING = 3,
} AlertType;

typedef enum {
    ALERT_NONE = 0,
    ALERT_FW_FAULT = 1,
    ALERT_LAST = ALERT_FW_FAULT
} AlertId;

typedef struct {
    uint8_t type;
} AlertProperties;

typedef struct {
    time_t time;
    uint8_t id;
    bool active;
    uint8_t code;
} AlertRecord;

typedef struct {
    bool persistent_fatal_error;

    uint32_t active_alert_mask;
    uint32_t new_active_alert_mask;
    mc_fault_code fw_fault_code;
    bool fatal_error;

    uint8_t alert_buffer_array[ALERT_TRACKER_SIZE * sizeof(AlertRecord)];
    CircularBuffer alert_buffer;
} AlertTracker;

const AlertProperties *alert_tracker_properties(AlertId alert_id);

void alert_tracker_init(AlertTracker *at);

void alert_tracker_configure(AlertTracker *at, const RefloatConfig *config);

void alert_tracker_add(AlertTracker *at, const Time *time, uint8_t id, uint8_t data);

void alert_tracker_finalize(AlertTracker *at, const Time *time);

bool alert_tracker_is_alert_active(AlertTracker *at, AlertId alert);

void alert_tracker_clear_fatal(AlertTracker *at);
