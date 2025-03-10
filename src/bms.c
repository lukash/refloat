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

#include "bms.h"

bool bms_get_fault(uint32_t fault_mask, BMSFaultCode fault_code) {
    return (fault_mask & (1U << (fault_code - 1))) != 0;
}

void bms_set_fault(uint32_t *fault_mask, BMSFaultCode fault_code) {
    if (fault_code == BMSF_NONE) {
        *fault_mask = BMSF_NONE;
        return;
    }

    *fault_mask |= (1U << (fault_code - 1));
}
