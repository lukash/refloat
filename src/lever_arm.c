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

#include "lever_arm.h"

#define STANDARD_GRAVITY 9.80665f

static inline void cross(const float *a, const float *b, float *out) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void lever_arm_init(LeverArm *la) {
    for (int i = 0; i < 3; ++i) {
        la->offset[i] = 0.0f;
        la->gyro[i] = 0.0f;
    }
    la->initialized = false;
}

void lever_arm_configure(LeverArm *la, const RefloatConfig *config) {
    // Config offsets are in a rider frame: X positive towards the nose, Y
    // positive towards the left (looking towards the nose), Z positive up.
    // The IMU frame has +X pointing towards the tail: the balance filter's
    // pitch convention (positive = nose up, increased by positive gyro Y)
    // leaves no other option for a right-handed frame with Z up. Mapping to
    // it negates X and Y (a 180 degree rotation about Z).
    //
    // Only the X offset is exposed in the config, Y/Z offsets have proven
    // insignificant.
    la->offset[0] = -config->imu.offset_x / 1000.0f;
    la->offset[1] = 0.0f;
    la->offset[2] = 0.0f;
}

void lever_arm_update(LeverArm *la, const float *gyro, const float *acc, float *acc_out, float dt) {
    float ang_accel[3] = {0.0f, 0.0f, 0.0f};
    if (la->initialized && dt > 0.0f) {
        for (int i = 0; i < 3; ++i) {
            ang_accel[i] = (gyro[i] - la->gyro[i]) / dt;
        }
    }
    la->initialized = true;

    float tangential[3], rate_x_offset[3], centripetal[3];
    cross(ang_accel, la->offset, tangential);
    cross(gyro, la->offset, rate_x_offset);
    cross(gyro, rate_x_offset, centripetal);

    for (int i = 0; i < 3; ++i) {
        la->gyro[i] = gyro[i];
        acc_out[i] = acc[i] - (tangential[i] + centripetal[i]) / STANDARD_GRAVITY;
    }
}
