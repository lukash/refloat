// Copyright 2024 Syler Clayton
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

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    NONE = 0,
    BMS_CONNECTION = 1,
    BMS_OVER_TEMP = 2,
    BMS_CELL_OVER_VOLTAGE = 3,
    BMS_CELL_UNDER_VOLTAGE = 4,
    BMS_CELL_OVER_TEMP = 5,
    BMS_CELL_UNDER_TEMP = 6,
    BMS_CELL_BALANCE = 7
} BMSFaultCode;

bool bms_is_fault_set(uint32_t fault_mask, BMSFaultCode fault_code);
