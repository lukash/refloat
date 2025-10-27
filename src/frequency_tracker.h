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

#pragma once

#include "filters/ema.h"
#include "time.h"

/**
 * Serves for tracking the dt and frequency of a loop. The stored dt is mainly
 * as a metric for tracking in realtime data. The frequency is updated in a
 * slow filter. If it changes significantly (over 3%) from the frequency
 * configured for the filters (filter_frequency), a recalculation of all the
 * filters dependent on that frequency is triggered.
 */
typedef struct {
    float dt;
    EMA frequency;
    float filter_frequency;
    time_t filter_last_update;
    bool first;
    bool running;
    uint32_t recalcs;
} FrequencyTracker;

void frequency_tracker_init(FrequencyTracker *ft, float frequency, const Time *time);

/**
 * Updates the tracked dt and filtered frequency.
 */
void frequency_tracker_update(FrequencyTracker *ft, float dt);

/**
 * Checks if the frequency is within 3% of the current filter frequency and if
 * not, triggers a reconfiguration of the filters. The amount of changes is
 * throttled to once per second and is only changed when running, because the
 * current modulation has a big impact on CPU load and subsequently the
 * scheduling.
 */
void frequency_tracker_check(
    FrequencyTracker *ft, bool running, const Time *time, void (*reconf_cb)(float)
);
