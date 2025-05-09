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

#include "ema.h"

#include <math.h>

float ema_calculate_alpha(float cutoff_freq, float update_freq) {
    float omega = 2.0f * M_PI * cutoff_freq / update_freq;
    // second order Taylor series approximation of aplha = 1 - e^-omega
    return omega - 0.5f * omega * omega;
}

void ema_init(EMA *ema) {
    ema->alpha = 0.0f;

    ema_reset(ema, 0.0f);
}

void ema_configure(EMA *ema, float cutoff_freq, float update_freq) {
    ema->alpha = ema_calculate_alpha(cutoff_freq, update_freq);
}

void ema_reset(EMA *ema, float value) {
    ema->value = value;
}

extern inline void ema_update(EMA *ema, float target);
