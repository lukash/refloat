// Copyright 2026 Lukas Hrazky
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

#include <stdbool.h>

/**
 * Compensation of the accelerometer data for the lever arm effect.
 *
 * An IMU mounted at an offset from the axle measures, on top of the
 * acceleration of the board itself, rotational effects at its position: a
 * tangential component from angular acceleration and a centripetal component
 * from angular rate. The balance filter can't tell these from gravity and
 * they skew its attitude estimate whenever the board rotates rapidly.
 *
 * Given the IMU position r relative to the axle, the rotational components
 * are computed from gyro data and subtracted from the accelerometer data,
 * yielding the values an IMU placed at the axle would measure:
 *
 *   acc_axle = acc_imu - alpha x r - omega x (omega x r)
 *
 * (omega: angular rate, alpha: its time derivative, x: cross product)
 */
typedef struct {
    // IMU position relative to the axle center [m], in the IMU frame (see
    // lever_arm_configure for the mapping from the config offset)
    float offset[3];
    // last gyro sample [rad/s] for differentiation
    float gyro[3];
    bool initialized;
} LeverArm;

void lever_arm_init(LeverArm *la);

void lever_arm_configure(LeverArm *la, const RefloatConfig *config);

/**
 * Computes the lever arm correction from gyro [rad/s] and subtracts it from
 * acc [g], storing the result in acc_out.
 */
void lever_arm_update(LeverArm *la, const float *gyro, const float *acc, float *acc_out, float dt);
