// Copyright 2022 Dado Mista
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

#include "atr.h"
#include "conf/datatypes.h"
#include "motor_data.h"

typedef struct {
    float factor;
    float target;
    float setpoint;
} BrakeTilt;

void brake_tilt_reset(BrakeTilt *bt);

void brake_tilt_configure(BrakeTilt *bt, const RefloatConfig *config);

void brake_tilt_update(
    BrakeTilt *bt,
    const MotorData *motor,
    const ATR *atr,
    const RefloatConfig *config,
    float balance_offset
);

void brake_tilt_winddown(BrakeTilt *bt);
