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

#include <math.h>

void frequency_tracker_init(FrequencyTracker *ft, float frequency, const Time *time) {
    ft->dt = 0.0f;
    ema_init(&ft->frequency);
    ema_configure(&ft->frequency, 1.0f, frequency);
    ema_reset(&ft->frequency, frequency);
    ft->filter_frequency = frequency;
    timer_refresh(time, &ft->filter_last_update);
    ft->first = true;
    ft->running = false;
    ft->recalcs = 0;
}

void frequency_tracker_update(FrequencyTracker *ft, float dt) {
    ft->dt = dt * 1000.0f;
    ema_update(&ft->frequency, 1.0f / dt);
}

void frequency_tracker_check(
    FrequencyTracker *ft, bool running, const Time *time, void (*reconf_cb)(float)
) {
    if (!ft->running && running) {
        // wait a second after engaging for the frequency to settle
        timer_refresh(time, &ft->filter_last_update);
    }
    ft->running = running;

    if ((running || ft->first) && timer_older(time, ft->filter_last_update, 1.0f) &&
        fabsf(1 - ft->frequency.value / ft->filter_frequency) > 0.03f) {
        reconf_cb(ft->frequency.value);
        ft->filter_frequency = ft->frequency.value;
        ema_configure(&ft->frequency, 1.0f, ft->filter_frequency);
        timer_refresh(time, &ft->filter_last_update);
        ft->first = false;
        ++ft->recalcs;
    }
}
