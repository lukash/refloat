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

#include <stdint.h>

#define STRIP_COUNT 3
#define LEDS_FRONT_AND_REAR_COUNT_MAX 60

typedef struct {
    uint8_t map[LEDS_FRONT_AND_REAR_COUNT_MAX];
} CipherData;

typedef union {
    CipherData cipher;
} TransitionData;

typedef struct {
    uint32_t *data;
    uint8_t length;
    LedColorOrder color_order;
    bool reverse;
    float brightness;
    TransitionData trans_data;
} LedStrip;

void led_strip_init(LedStrip *strip);

void led_strip_configure(LedStrip *strip, const CfgLedStrip *cfg);
