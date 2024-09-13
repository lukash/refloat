// Copyright 2024 Lukas Hrazky
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

#include "state.h"

#include "vesc_c_if.h"

typedef uint32_t time_t;

typedef struct {
    time_t now;
    time_t engage_timer;
    time_t disengage_timer;
    time_t idle_timer;
} Time;

void time_init(Time *t);

void time_update(Time *t, RunState state);

inline void time_refresh_idle(Time *t) {
    t->idle_timer = t->now;
}

inline void timer_refresh(const Time *t, time_t *timer) {
    *timer = t->now;
}

inline bool timer_older(const Time *t, time_t timer, float seconds) {
    return t->now - timer > (time_t) (seconds * SYSTEM_TICK_RATE_HZ);
}

inline bool timer_older_ms(const Time *t, time_t timer, float seconds) {
    return t->now - timer > (time_t) (seconds * SYSTEM_TICK_RATE_HZ / 1000);
}

inline float timer_age(const Time *t, time_t timer) {
    return (t->now - timer) * (1.0f / SYSTEM_TICK_RATE_HZ);
}

#define time_elapsed(t, event, seconds)                                                            \
    ({                                                                                             \
        const Time *_t = (t);                                                                      \
        timer_older(_t, _t->event##_timer, seconds);                                               \
    })
