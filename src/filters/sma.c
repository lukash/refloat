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

#include "sma.h"

#include "lib/utils.h"
#include "vesc_c_if.h"

#include <math.h>

// https://dsp.stackexchange.com/questions/9966
static uint8_t sma_calculate_n(float cutoff_freq, float update_freq) {
    float fc = cutoff_freq / update_freq;
    return min(sqrtf(0.196202f + fc * fc) / fc, 255);
}

void sma_init(SMA *sma) {
    sma->n = 0;
    sma->allocated_n = 0;
    sma->new_n = 0;
    sma->array = 0;

    sma_reset(sma);
}

void sma_destroy(SMA *sma) {
    if (sma->array) {
        VESC_IF->free(sma->array);
    }
}

void sma_configure(SMA *sma, float cutoff_freq, float update_freq) {
    uint8_t n = min(sma_calculate_n(cutoff_freq, update_freq), 255);
    if (sma->array == 0) {
        // Allocate with a 20% leeway to be able to increase the n later if needed
        sma->allocated_n = min(1.2 * n, 255);
        sma->array = VESC_IF->malloc(sma->allocated_n * sizeof(float));
        if (!sma->array) {
            sma->allocated_n = 0;
            log_error("Failed to allocate SMA array.");
            return;
        }
        log_msg(
            "Allocated SMA total: %u N: %u (freq: %d)", sma->allocated_n, n, (int32_t) update_freq
        );

        sma_reset(sma);
        sma->n = n;
    } else if ((n = min(n, sma->allocated_n)) != sma->n && sma->new_n == 0) {
        sma->new_n = n;
        log_msg("Changed N from: %u to: %u (freq: %d)", sma->n, n, (int32_t) update_freq);
    }
}

void sma_reset(SMA *sma) {
    sma->value = 0.0f;

    sma->idx = 0;
    for (size_t i = 0; i < sma->n; i++) {
        sma->array[i] = 0.0f;
    }
}

void sma_update(SMA *sma, float target) {
    sma->value += (target - sma->array[sma->idx]) / sma->n;
    sma->array[sma->idx] = target;

    if (sma->new_n > 0 && sma->idx == sma->n - 1) {
        // This is ugly... to change the N of the filter, we need to correct
        // the live value according to items being removed from the array and
        // we need to insert the current value to newly added slots.
        if (sma->new_n < sma->n) {
            float subt = 0.0f;
            for (uint8_t i = sma->new_n; i < sma->n; ++i) {
                subt += sma->array[i];
            }
            sma->value -= subt / sma->n;
            sma->value *= (float) sma->n / sma->new_n;
        } else if (sma->new_n > sma->n) {
            for (uint8_t i = sma->n; i < sma->new_n; ++i) {
                sma->array[i] = sma->value;
            }
        }

        sma->n = sma->new_n;
        log_msg("Used new N: %u", sma->new_n);
        sma->new_n = 0;
    }

    if (++sma->idx >= sma->n) {
        sma->idx = 0;
    }
}
