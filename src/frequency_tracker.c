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

#include "frequency_tracker.h"

#include "lib/utils.h"

#include <math.h>

// Time constant of the frequency filter [s] (corresponds to a 1 Hz cutoff)
#define FREQUENCY_TIME_CONSTANT 0.16f
// Period of reading the frequency for the reconfiguration check [s]. A change
// of frequency has to last about two periods to trigger a reconfiguration.
#define READING_PERIOD 0.5f
// Max relative difference of two consecutive readings of a stable frequency
#define STABLE_TOLERANCE 0.01f
// Relative difference from the filter frequency that triggers a reconfiguration
#define RECONFIGURE_THRESHOLD 0.03f

static void restart_readings(FrequencyTracker *ft, const Time *time) {
    timer_refresh(time, &ft->last_reading_time);
    ft->last_reading = 0.0f;
}

void frequency_tracker_init(FrequencyTracker *ft, float frequency, const Time *time) {
    ft->dt = 0.0f;
    ft->frequency = frequency;
    ft->filter_frequency = frequency;
    restart_readings(ft, time);
    ft->calibrating = true;
    ft->running = false;
    ft->recalcs = 0;
}

void frequency_tracker_recalibrate(FrequencyTracker *ft, float frequency, const Time *time) {
    ft->frequency = frequency;
    restart_readings(ft, time);
    ft->calibrating = true;
}

void frequency_tracker_update(FrequencyTracker *ft, float dt) {
    ft->dt = dt * 1000.0f;
    // A first order filter of 1 / dt with each sample weighted by the time it
    // covers (alpha = dt / time constant). The dt cancels out, so every sample
    // counts once no matter how short its dt is and the result is the number
    // of samples per time, unbiased by jitter (unlike the mean of 1 / dt). The
    // max() keeps the filter stable for a dt longer than the time constant.
    ft->frequency += (1.0f - ft->frequency * dt) / max(FREQUENCY_TIME_CONSTANT, dt);
}

void frequency_tracker_check(
    FrequencyTracker *ft, bool running, const Time *time, void (*reconf_cb)(float)
) {
    if (!ft->running && running) {
        // Readings from before engaging don't count, the load changes the frequency.
        // The reset last_reading can't pass as stable, so a reconfiguration takes two
        // readings after engaging: a change of frequency has to last about a second.
        restart_readings(ft, time);
    }
    ft->running = running;

    if (!timer_older(time, ft->last_reading_time, READING_PERIOD)) {
        return;
    }
    timer_refresh(time, &ft->last_reading_time);

    float frequency = ft->frequency;
    bool stable = fabsf(frequency - ft->last_reading) < STABLE_TOLERANCE * frequency;
    ft->last_reading = frequency;

    if (!stable) {
        return;
    }

    if ((running || ft->calibrating) &&
        fabsf(1 - frequency / ft->filter_frequency) > RECONFIGURE_THRESHOLD) {
        reconf_cb(frequency);
        ft->filter_frequency = frequency;
        ++ft->recalcs;
    }
    ft->calibrating = false;
}
