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

typedef struct {
    CfgTargetFilter cfg;
    float on_speed;
    float off_speed;

    float value;
    float speed;
    float accel;
    float k;
    float dt;
} EMAFilter;

void ema_filter_configure(
    EMAFilter *filter, const CfgTargetFilter *cfg, float on_speed, float off_speed
);

void ema_filter_reset(EMAFilter *filter, float value, float speed);

void ema_filter_update(EMAFilter *filter, float target, float dt);
