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

#include <stdint.h>

/** Simple (Arithmetic) Moving Average
 */
typedef struct {
    uint8_t n;
    uint8_t allocated_n;
    uint8_t new_n;
    uint8_t idx;
    float *array;
    float value;
} SMA;

void sma_init(SMA *sma);

void sma_destroy(SMA *sma);

/**
 * Configures the filter. If the array for averaging is not allocated yet,
 * allocates it. The array is allocated with 20% more items to allow for
 * increasing the n later, if the sampling frequency is found to be greater
 * than previously. Note if the frequency is ever raised after the allocation,
 * the array length will be capped by the allocated size and so will the cutoff
 * frequency.
 *
 * If the array is already allocated and the n calculated from the desired
 * cutoff/update frequencies differs from the previous one, it stores the new n
 * to the new_n member variable. After the new_n is set, the sma_update()
 * function will switch to it (set n = new_n) once it reaches the end of the
 * array (and would just rotate to the beginning). This ensures a smooth
 * transition. The transition has a nontrivial overhead, as it needs to update
 * the live value if shortening the array and it needs to fill up any new slots
 * if enlarging the array.
 */
void sma_configure(SMA *sma, float cutoff_freq, float update_freq);

void sma_reset(SMA *sma);

void sma_update(SMA *sma, float target);
