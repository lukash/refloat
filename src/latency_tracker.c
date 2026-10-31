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

#include "latency_tracker.h"

#include "vesc_c_if.h"

#include <math.h>

// rate of TIM5 backing timer_time_now(), not exposed by the C interface
#define TIMER_HZ 14000000.0f

void latency_tracker_init(LatencyTracker *lt) {
    lt->latency = 0.0f;
    lt->latency_min = INFINITY;
    lt->excess = 0.0f;
    lt->last_stamp = 0;
    lt->first = true;
}

void latency_tracker_update(LatencyTracker *lt, float dt) {
    // An IMU restart would otherwise add its gap to the latency permanently
    if (!VESC_IF->imu_startup_done()) {
        latency_tracker_init(lt);
        return;
    }

    uint32_t now = VESC_IF->timer_time_now();
    if (lt->first) {
        lt->first = false;
        lt->last_stamp = now;
        return;
    }

    float interval = (now - lt->last_stamp) / TIMER_HZ;
    lt->last_stamp = now;

    lt->latency += interval - dt;
    if (lt->latency < lt->latency_min) {
        lt->latency_min = lt->latency;
    }
    lt->excess = (lt->latency - lt->latency_min) * 1000.0f;
}
