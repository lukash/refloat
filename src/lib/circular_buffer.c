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

#include "circular_buffer.h"

#include <string.h>

void circular_buffer_clear(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->empty = true;
}

void circular_buffer_init(CircularBuffer *cb, size_t item_size, size_t item_number, void *buffer) {
    cb->buffer = buffer;
    cb->length = item_number;
    cb->item_size = item_size;
    circular_buffer_clear(cb);
}

static inline void increment(const CircularBuffer *cb, size_t *i) {
    ++(*i);
    if (*i >= cb->length) {
        *i -= cb->length;
    }
}

size_t circular_buffer_size(const CircularBuffer *cb) {
    size_t size = cb->head - cb->tail;
    if (size > cb->length) {
        return size + cb->length;
    } else if (size == 0 && !cb->empty) {
        return cb->length;
    }
    return size;
}

void circular_buffer_push(CircularBuffer *cb, const void *item) {
    if (!cb->empty && cb->head == cb->tail) {
        increment(cb, &cb->tail);
    }

    memcpy(cb->buffer + cb->head * cb->item_size, item, cb->item_size);
    increment(cb, &cb->head);

    cb->empty = false;
}

bool circular_buffer_get(const CircularBuffer *cb, size_t i, void *item) {
    if (i >= circular_buffer_size(cb)) {
        return false;
    }

    size_t offset = cb->tail + i;
    if (offset >= cb->length) {
        offset -= cb->length;
    }
    memcpy(item, cb->buffer + offset * cb->item_size, cb->item_size);

    return true;
}

bool circular_buffer_pop(CircularBuffer *cb, size_t i, void *item) {
    if (!circular_buffer_get(cb, i, item)) {
        return false;
    }

    increment(cb, &cb->tail);
    if (cb->tail == cb->head) {
        cb->empty = true;
    }

    return true;
}

void circular_buffer_iterate(
    const CircularBuffer *cb, void (*callback)(const void *item, void *data), void *data
) {
    if (cb->empty) {
        return;
    }

    size_t i = cb->tail;
    do {
        callback(&cb->buffer[i], data);
        increment(cb, &i);
    } while (i != cb->head);
}
