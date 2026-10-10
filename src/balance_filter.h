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

#ifndef BALANCE_FILTER_h
#define BALANCE_FILTER_h

#include "conf/datatypes.h"
#include "filters/ema.h"

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    // confidence state: mean magnitude (Decay algo), squared-deviation
    // envelope (Deviation Envelope algo)
    EMA acc_mag;
    EMA acc_dev_sq;
    // gyro bias estimated by the integral feedback [rad/s]
    float integral_fb_x;
    float integral_fb_y;
    float integral_fb_z;
    // confidence pipeline telemetry for realtime data
    float deviation;  // instantaneous |acc| - 1 [g]
    float sigma;  // deviation envelope (RMS) [g]
    float confidence;  // the confidence weight applied

    // parameters
    AccelConfidenceAlgo confidence_algo;
    // 1 / half_dev^2 of the Deviation Envelope algo, 0 disables
    float half_dev_recip_sq;
    // signal clamp on the deviation entering the envelope, 0 disables
    float acc_dev_clamp;
    float kp_pitch;
    float kp_roll;
    float kp_yaw;
    float ki;
} BalanceFilterData;

void balance_filter_init(BalanceFilterData *data);

void balance_filter_configure(
    BalanceFilterData *data, const RefloatConfig *config, float frequency
);

void balance_filter_update(BalanceFilterData *data, float *gyro_xyz, float *accel_xyz, float dt);

float balance_filter_get_roll(const BalanceFilterData *data);
float balance_filter_get_pitch(const BalanceFilterData *data);
float balance_filter_get_yaw(const BalanceFilterData *data);

#endif
