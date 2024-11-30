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

#pragma once

#include "rt_data.h"
#include "time.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    time_t time;
    uint8_t flags;
    uint16_t values[ITEMS_COUNT_REC(RT_DATA_ALL_ITEMS)];  // values encoded as float16
} Sample;

typedef struct {
    Sample *buffer;
    size_t size;
    size_t head;
    size_t tail;
    bool empty;
    bool recording;
    bool autostart;
    bool autostop;
} DataRecord;
