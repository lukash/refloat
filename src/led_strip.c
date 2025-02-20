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

#include "led_strip.h"

#include <string.h>

void led_strip_init(LedStrip *strip) {
    strip->data = NULL;
    strip->length = 0;
    strip->color_order = LED_COLOR_GRB;
    strip->reverse = false;
}

void led_strip_configure(LedStrip *strip, const CfgLedStrip *cfg) {
    strip->length = cfg->count;
    strip->color_order = cfg->color_order;
    strip->reverse = cfg->reverse;
}
