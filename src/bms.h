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
    BMSF_NONE = 0,
    BMSF_CONNECTION = 1,
    BMSF_OVER_TEMP = 2,
    BMSF_CELL_OVER_VOLTAGE = 3,
    BMSF_CELL_UNDER_VOLTAGE = 4,
    BMSF_CELL_OVER_TEMP = 5,
    BMSF_CELL_UNDER_TEMP = 6,
    BMSF_CELL_BALANCE = 7
} BMSFaultCode;

bool bms_get_fault(uint32_t fault_mask, BMSFaultCode fault_code);

void bms_set_fault(uint32_t *fault_mask, BMSFaultCode fault_code);
