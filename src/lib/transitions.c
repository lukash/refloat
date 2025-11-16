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

#include "transitions.h"

#define check_and_return(x)                                                                        \
    do {                                                                                           \
        if (x < 0.0f) {                                                                            \
            return 0.0f;                                                                           \
        } else if (x > 1.0f) {                                                                     \
            return 1.0f;                                                                           \
        }                                                                                          \
    } while (0)

float smoothstep(float x) {
    check_and_return(x);
    return x * x * (3.0f - 2.0f * x);
}

float smootherstep(float x) {
    check_and_return(x);
    return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f);
}
