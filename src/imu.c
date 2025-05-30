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

#include "imu.h"

#include "utils.h"

#include "vesc_c_if.h"

void imu_init(IMU *imu) {
    imu->pitch = 0.0f;
    imu->balance_pitch = 0.0f;
    imu->roll = 0.0f;
    imu->yaw = 0.0f;
    imu->gyro_y = 0.0f;

    imu->flywheel_pitch_offset = 0.0f;
    imu->flywheel_roll_offset = 0.0f;
}

void imu_update(IMU *imu, const BalanceFilterData *bf, const State *state) {
    float roll_rad = VESC_IF->imu_get_roll();  // in Radians

    imu->pitch = rad2deg(VESC_IF->imu_get_pitch());
    imu->roll = rad2deg(roll_rad);
    imu->yaw = rad2deg(VESC_IF->imu_get_yaw());
    imu->balance_pitch = rad2deg(balance_filter_get_pitch(bf));

    float gyro[3];
    VESC_IF->imu_get_gyro(gyro);

    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);

    // Rotated to diminish influence of Yaw Change on Gyro Y when board is rolled
    // (Estimates Pitch Rate solely due to rider input, without influence from board turning)
    imu->gyro_y = cos_roll * cos_roll * gyro[1] + sin_roll * cos_roll * gyro[2];

    if (state->mode == MODE_FLYWHEEL) {
        imu->pitch = imu->flywheel_pitch_offset - imu->pitch;
        imu->balance_pitch = imu->pitch;
        imu->roll -= imu->flywheel_roll_offset;
        if (imu->roll < -200) {
            imu->roll += 360;
        } else if (imu->roll > 200) {
            imu->roll -= 360;
        }
    }
}

void imu_set_flywheel_offsets(IMU *imu) {
    imu->flywheel_pitch_offset = imu->pitch;
    imu->flywheel_roll_offset = imu->roll;
}
