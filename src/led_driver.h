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

#pragma once

#include "conf/datatypes.h"
#include "led_strip.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t *bitbuffer;
    uint32_t bitbuffer_length;
    LedPin pin;
    const LedStrip *strips[STRIP_COUNT];
    uint16_t *strip_bitbuffs[STRIP_COUNT];
} LedDriver;

void led_driver_init(LedDriver *driver);

bool led_driver_setup(LedDriver *driver, LedPin pin, const LedStrip **led_strips);

void led_driver_paint(LedDriver *driver);

void led_driver_destroy(LedDriver *driver);
