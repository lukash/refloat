// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#include "biquad.h"

#include <math.h>

void biquad_init(Biquad *biquad) {
    biquad->a0 = 0.0f;
    biquad->a1 = 0.0f;
    biquad->a2 = 0.0f;
    biquad->b1 = 0.0f;
    biquad->b2 = 0.0f;

    biquad_reset(biquad);
}

void biquad_configure(Biquad *biquad, BiquadType type, float cutoff_freq, float update_freq) {
    float k = tanf(M_PI * cutoff_freq / update_freq);
    float q = 0.707;  // maximum sharpness (0.5 = maximum smoothness)
    float norm = 1 / (1 + k / q + k * k);
    if (type == BQ_LOWPASS) {
        biquad->a0 = k * k * norm;
        biquad->a1 = 2 * biquad->a0;
        biquad->a2 = biquad->a0;
    } else if (type == BQ_HIGHPASS) {
        biquad->a0 = 1 * norm;
        biquad->a1 = -2 * biquad->a0;
        biquad->a2 = biquad->a0;
    }
    biquad->b1 = 2 * (k * k - 1) * norm;
    biquad->b2 = (1 - k / q + k * k) * norm;
}

void biquad_reset(Biquad *biquad) {
    biquad->z1 = 0.0f;
    biquad->z2 = 0.0f;
    biquad->value = 0.0f;
}

void biquad_update(Biquad *biquad, float target) {
    biquad->value = target * biquad->a0 + biquad->z1;
    biquad->z1 = target * biquad->a1 + biquad->z2 - biquad->b1 * biquad->value;
    biquad->z2 = target * biquad->a2 - biquad->b2 * biquad->value;
}
