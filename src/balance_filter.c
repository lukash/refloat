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
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
// 26/01/2014   Benjamin V      Adaption to our platform
// 20/02/2017   Benjamin V      Added Madgwick algorithm and refactoring
// 17/09/2023   Lukas Hrazky    Adopted from vedderb/bldc
//
//=====================================================================================================

#include "balance_filter.h"

#include "vesc_c_if.h"

#include <math.h>

static inline float inv_sqrt(float x) {
    return 1.0 / sqrtf(x);
}

static float calculate_acc_confidence(float accMag, float *accMagP, float acc_confidence_decay) {
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity

    float confidence;

    accMag = *accMagP * 0.9f + accMag * 0.1f;
    *accMagP = accMag;

    confidence = 1.0 - (acc_confidence_decay * sqrtf(fabsf(accMag - 1.0f)));

    return confidence > 0 ? confidence : 0;
}

void balance_filter_init(BalanceFilterData *data) {
    // Init with internal filter orientation, otherwise the AHRS would need a while to stabilise
    float quat[4];
    VESC_IF->imu_get_quaternions(quat);
    data->q0 = quat[0];
    data->q1 = quat[1];
    data->q2 = quat[2];
    data->q3 = quat[3];
    data->integralFBx = 0.0;
    data->integralFBy = 0.0;
    data->integralFBz = 0.0;
    data->accMagP = 1.0;
}

void balance_filter_configure(BalanceFilterData *data, const RefloatConfig *config) {
    data->acc_confidence_decay = config->bf_accel_confidence_decay;
    data->kp = config->mahony_kp;
    data->ki = 0;
}

void balance_filter_update(float *gyroXYZ, float *accelXYZ, float dt, BalanceFilterData *data) {
    float accelNorm, recipNorm;
    float qa, qb, qc;

    float gx = gyroXYZ[0];
    float gy = gyroXYZ[1];
    float gz = gyroXYZ[2];

    float ax = accelXYZ[0];
    float ay = accelXYZ[1];
    float az = accelXYZ[2];

    accelNorm = sqrtf(ax * ax + ay * ay + az * az);

    // Compute feedback only if accelerometer abs(vector)is not too small to avoid a division
    // by a small number
    if (accelNorm > 0.01) {
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float accelConfidence;

        volatile float twoKp = 2.0 * data->kp;
        volatile float twoKi = 2.0 * data->ki;

        accelConfidence =
            calculate_acc_confidence(accelNorm, &data->accMagP, data->acc_confidence_decay);
        twoKp *= accelConfidence;
        twoKi *= accelConfidence;

        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = data->q1 * data->q3 - data->q0 * data->q2;
        halfvy = data->q0 * data->q1 + data->q2 * data->q3;
        halfvz = data->q0 * data->q0 - 0.5f + data->q3 * data->q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            data->integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
            data->integralFBy += twoKi * halfey * dt;
            data->integralFBz += twoKi * halfez * dt;
            gx += data->integralFBx;  // apply integral feedback
            gy += data->integralFBy;
            gz += data->integralFBz;
        } else {
            data->integralFBx = 0.0f;  // prevent integral windup
            data->integralFBy = 0.0f;
            data->integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);  // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = data->q0;
    qb = data->q1;
    qc = data->q2;
    data->q0 += (-qb * gx - qc * gy - data->q3 * gz);
    data->q1 += (qa * gx + qc * gz - data->q3 * gy);
    data->q2 += (qa * gy - qb * gz + data->q3 * gx);
    data->q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = inv_sqrt(
        data->q0 * data->q0 + data->q1 * data->q1 + data->q2 * data->q2 + data->q3 * data->q3
    );
    data->q0 *= recipNorm;
    data->q1 *= recipNorm;
    data->q2 *= recipNorm;
    data->q3 *= recipNorm;
}

float balance_filter_get_roll(BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return -atan2f(q0 * q1 + q2 * q3, 0.5 - (q1 * q1 + q2 * q2));
}

float balance_filter_get_pitch(BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return asinf(-2.0 * (q1 * q3 - q0 * q2));
}

float balance_filter_get_yaw(BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return -atan2f(q0 * q3 + q1 * q2, 0.5 - (q2 * q2 + q3 * q3));
}
