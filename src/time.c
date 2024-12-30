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

#include "time.h"

static inline void time_now(Time *t) {
    if (VESC_IF->system_time_ticks) {
        t->now = VESC_IF->system_time_ticks();
    } else {
        t->now = VESC_IF->system_time() * SYSTEM_TICK_RATE_HZ;
    }
}

void time_init(Time *t) {
    time_now(t);
    t->engage_timer = t->now;
    // Workaround: After startup (assume the time is very close to 0), we don't
    // want anything to be thinking we have just disengaged. Set disengage time
    // a minute into the past to prevent this.
    t->disengage_timer = t->now - 60;
    time_refresh_idle(t);
}

void time_update(Time *t, RunState state) {
    time_now(t);
    if (state == STATE_RUNNING) {
        t->disengage_timer = t->now;
        time_refresh_idle(t);
    }
}
