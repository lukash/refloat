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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Note: The circular buffer is not synchronized. Provide your own locking, or
// at least insure it's only being written from a single thread and know that
// data may not be consistent on reading.

typedef struct {
    uint8_t *buffer;
    size_t length;
    size_t item_size;
    size_t head;
    size_t tail;
    bool empty;
} CircularBuffer;

void circular_buffer_init(CircularBuffer *cb, size_t item_size, size_t item_number, void *buffer);

void circular_buffer_clear(CircularBuffer *cb);

size_t circular_buffer_size(const CircularBuffer *cb);

/**
 * Pushes an item into the buffer. In case it's full, replaces the last item.
 */
void circular_buffer_push(CircularBuffer *cb, const void *item);

/**
 * Gets an item at index i (a position relative to the current tail) from the
 * buffer without removing it. If there's no item at that index, leaves item
 * unchanged and returns false, otherwise returns true.
 */
bool circular_buffer_get(const CircularBuffer *cb, size_t i, void *item);

/**
 * Pops an item at index i (a position relative to the current tail) from the
 * buffer. If there's no item at that index, leaves item unchanged and returns
 * false, otherwise returns true.
 */
bool circular_buffer_pop(CircularBuffer *cb, size_t i, void *item);

/**
 * Iterates over the buffer, calling the callback function on each item.
 */
void circular_buffer_iterate(
    const CircularBuffer *cb, void (*callback)(const void *item, void *data), void *data
);
