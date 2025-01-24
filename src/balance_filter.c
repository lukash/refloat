// Copyright 2023 - 2024 Lukas Hrazky
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

//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimized for reduced CPU load
// 26/01/2014   Benjamin V      Adaption to our platform
// 20/02/2017   Benjamin V      Added Madgwick algorithm and refactoring
// 17/09/2023   Lukas Hrazky    Adopted from vedderb/bldc, modified for self-balancing skateboard
//
//=====================================================================================================

#include "balance_filter.h"

#include "vesc_c_if.h"

#include <math.h>

static inline float inv_sqrt(float x) {
    return 1.0 / sqrtf(x);
}

static float calculate_acc_confidence(float new_acc_mag, BalanceFilterData *data) {
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity
    data->acc_mag = data->acc_mag * 0.9 + new_acc_mag * 0.1;

    // Hard-coded accelerometer confidence decay of 0.02
    float confidence = 1.0 - (0.02 * sqrtf(fabsf(data->acc_mag - 1.0f)));

    return confidence > 0 ? confidence : 0;
}

void balance_filter_init(BalanceFilterData *data) {
    // Init with internal filter orientation, otherwise the AHRS would need a while to stabilize
    float quat[4];
    VESC_IF->imu_get_quaternions(quat);
    data->q0 = quat[0];
    data->q1 = quat[1];
    data->q2 = quat[2];
    data->q3 = quat[3];
    data->acc_mag = 1.0;
}

void balance_filter_configure(BalanceFilterData *data, const RefloatConfig *config) {
    data->kp_pitch = config->mahony_kp;
    data->kp_roll = config->mahony_kp_roll;
    // Use middle value between Pitch KP and Roll KP. Yaw KP seems to have
    // negligible effect on balancing and the middle value should skew the
    // filter the least.
    data->kp_yaw = (config->mahony_kp + config->mahony_kp_roll) / 2.0f;
}

void balance_filter_update(BalanceFilterData *data, float *gyro_xyz, float *accel_xyz, float dt) {
    float gx = gyro_xyz[0];
    float gy = gyro_xyz[1];
    float gz = gyro_xyz[2];

    float ax = accel_xyz[0];
    float ay = accel_xyz[1];
    float az = accel_xyz[2];

    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    // Compute feedback only if accelerometer abs(vector)is not too small to avoid a division
    // by a small number
    if (accel_norm > 0.01) {
        float accel_confidence = calculate_acc_confidence(accel_norm, data);
        float two_kp_pitch = 2.0 * data->kp_pitch * accel_confidence;
        float two_kp_roll = 2.0 * data->kp_roll * accel_confidence;
        float two_kp_yaw = 2.0 * data->kp_yaw * accel_confidence;

        // Normalize accelerometer measurement
        float recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        float halfvx = data->q1 * data->q3 - data->q0 * data->q2;
        float halfvy = data->q0 * data->q1 + data->q2 * data->q3;
        float halfvz = data->q0 * data->q0 - 0.5f + data->q3 * data->q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        float halfex = (ay * halfvz - az * halfvy);
        float halfey = (az * halfvx - ax * halfvz);
        float halfez = (ax * halfvy - ay * halfvx);

        // Apply proportional feedback
        gx += two_kp_roll * halfex;
        gy += two_kp_pitch * halfey;
        gz += two_kp_yaw * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);  // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = data->q0;
    float qb = data->q1;
    float qc = data->q2;
    data->q0 += (-qb * gx - qc * gy - data->q3 * gz);
    data->q1 += (qa * gx + qc * gz - data->q3 * gy);
    data->q2 += (qa * gy - qb * gz + data->q3 * gx);
    data->q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    float recip_norm = inv_sqrt(
        data->q0 * data->q0 + data->q1 * data->q1 + data->q2 * data->q2 + data->q3 * data->q3
    );
    data->q0 *= recip_norm;
    data->q1 *= recip_norm;
    data->q2 *= recip_norm;
    data->q3 *= recip_norm;
}

float balance_filter_get_roll(const BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return -atan2f(q0 * q1 + q2 * q3, 0.5 - (q1 * q1 + q2 * q2));
}

float balance_filter_get_pitch(const BalanceFilterData *data) {
    float sin = -2.0 * (data->q1 * data->q3 - data->q0 * data->q2);

    if (sin < -1) {
        return -M_PI / 2;
    } else if (sin > 1) {
        return M_PI / 2;
    }

    return asinf(sin);
}

float balance_filter_get_yaw(const BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return -atan2f(q0 * q3 + q1 * q2, 0.5 - (q2 * q2 + q3 * q3));
}
