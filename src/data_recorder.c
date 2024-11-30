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

#include "data_recorder.h"
#include "conf/buffer.h"
#include "utils.h"
#include "vesc_c_if.h"

static void circular_buffer_reset(DataRecord *dr) {
    dr->head = 0;
    dr->tail = 0;
    dr->empty = true;
}

static inline void increment(const DataRecord *buffer, size_t *i) {
    ++(*i);
    if (*i >= buffer->size) {
        *i -= buffer->size;
    }
}

static void circular_buffer_push(DataRecord *buffer, const Sample *sample) {
    buffer->buffer[buffer->head] = *sample;

    if (!buffer->empty && buffer->head == buffer->tail) {
        increment(buffer, &buffer->tail);
    }
    increment(buffer, &buffer->head);

    buffer->empty = false;
}

static void circular_buffer_iterate(
    const DataRecord *buffer, void (*cb)(const Sample *sample, void *data), void *data
) {
    if (buffer->empty) {
        return;
    }

    size_t i = buffer->tail;
    do {
        cb(&buffer->buffer[i], data);
        increment(buffer, &i);
    } while (i != buffer->head);
}

static void start_recording(DataRecord *dr) {
    circular_buffer_reset(dr);
    dr->recording = true;
}

static void stop_recording(DataRecord *dr) {
    dr->recording = false;
}

typedef struct {
    uint32_t magic;
    uint8_t *buffer;
    size_t length;
} DataBufferInfo;

void data_recorder_init(DataRecord *dr) {
    // fetch information about the data buffer, it's stored at the end of the
    // VESC interface memory area
    DataBufferInfo *buffer_info = (DataBufferInfo *) ((uint8_t *) VESC_IF + 2036);
    if (buffer_info->magic == 0xcafe1111) {
        size_t size = buffer_info->length;
        size_t sample_nr = size / sizeof(Sample);
        log_msg("Data Record buffer size: %uB (%u samples)", size, sample_nr);
        dr->buffer = (Sample *) buffer_info->buffer;
        dr->size = sample_nr;
    } else {
        dr->buffer = 0;
        dr->size = 0;
    }

    dr->head = 0;
    dr->tail = 0;
    dr->empty = true;
    dr->recording = false;
    dr->autostart = true;
    dr->autostop = true;
}

bool data_recorder_has_capability(const DataRecord *dr) {
    return dr->buffer != NULL;
}

void data_recorder_trigger(DataRecord *dr, bool engage) {
    if (!dr->buffer) {
        return;
    }

    if (dr->autostart && engage) {
        start_recording(dr);
    } else if (dr->autostop && !engage) {
        stop_recording(dr);
    }
}

void data_recorder_sample(DataRecord *dr, const Data *d) {
    if (!dr->buffer || !dr->recording) {
        return;
    }

    uint8_t flags = d->state.sat << 4 | d->footpad.state << 2;
    flags |= d->state.wheelslip << 1 | (d->state.state == STATE_RUNNING);

    Sample p =
        {.time = d->time.now,
         .flags = flags,
         .values = {
#define ARRAY_VALUE(id) to_float16(d->id),
             VISIT_REC(RT_DATA_ALL_ITEMS, ARRAY_VALUE)
#undef ARRAY_VALUE
         }};
    circular_buffer_push(dr, &p);
}

static void send_point_vt_experiment(const Sample *sample, void *data) {
    unused(data);

    for (uint8_t i = 0; i < ITEMS_COUNT_REC(RT_DATA_ALL_ITEMS); ++i) {
        VESC_IF->plot_set_graph(i);
        VESC_IF->plot_send_points(sample->time, sample->values[i]);
    }
}

void data_recorder_send_experiment_plot(DataRecord *dr) {
    if (!dr->buffer) {
        return;
    }

    VESC_IF->plot_init("t", "v");

#define ADD_GRAPH(id) VESC_IF->plot_add_graph(#id);
    VISIT_REC(RT_DATA_ALL_ITEMS, ADD_GRAPH);
#undef ADD_GRAPH

    circular_buffer_iterate(dr, &send_point_vt_experiment, 0);
}

typedef enum {
    COMMAND_DATA_RECORD_HEADER = 42,
    COMMAND_DATA_RECORD_DATA = 43,
} DataRecordCommands;

static void send_header(DataRecord *dr) {
    static const int bufsize = 256;
    uint8_t buf[bufsize];
    int32_t ind = 0;

    buf[ind++] = 101;  // Package ID
    buf[ind++] = COMMAND_DATA_RECORD_HEADER;

    size_t size = 0;
    if (!dr->empty) {
        size = (dr->head - dr->tail) % dr->size;
        if (size == 0) {
            size = dr->size;
        }
    }

    buffer_append_uint32(buf, size, &ind);

    buf[ind++] = ITEMS_COUNT_REC(RT_DATA_ALL_ITEMS);
#define ADD_ID(id) buffer_append_string(buf, #id, &ind);
    VISIT_REC(RT_DATA_ALL_ITEMS, ADD_ID);
#undef ADD_ID

    SEND_APP_DATA(buf, bufsize, ind);
}

static void send_data(const DataRecord *dr, size_t offset) {
    if (dr->empty) {
        return;
    }

    static const int bufsize = SEND_BUF_MAX_SIZE;
    uint8_t buf[bufsize];
    int32_t ind = 0;

    buf[ind++] = 101;  // Package ID
    buf[ind++] = COMMAND_DATA_RECORD_DATA;

    buffer_append_uint32(buf, offset, &ind);

    bool start = true;
    for (size_t i = (dr->tail + offset) % dr->size; start || i != dr->head; increment(dr, &i)) {
        // 4 bytes for time, 1 byte for flags, 2 bytes for rest of the values
        if (ind + 4 + 1 + 2 * ITEMS_COUNT_REC(RT_DATA_ALL_ITEMS) > bufsize) {
            break;
        }

        const Sample *sample = &dr->buffer[i];
        buffer_append_uint32(buf, sample->time, &ind);
        buf[ind++] = sample->flags;

        for (size_t i = 0; i < ITEMS_COUNT_REC(RT_DATA_ALL_ITEMS); ++i) {
            buffer_append_uint16(buf, sample->values[i], &ind);
        }

        start = false;
    }

    SEND_APP_DATA(buf, bufsize, ind);
}

void data_recorder_request(DataRecord *dr, uint8_t *buffer, size_t len) {
    if (!dr->buffer) {
        log_error("Data Record not supported.");
        return;
    }

    if (len < 2) {
        log_error("Data Record request missing data.");
        return;
    }

    int32_t ind = 0;
    uint8_t mode = buffer[ind++];
    uint8_t sub_mode = buffer[ind++];
    if (mode == 1) {  // control
        if (len < 3) {
            log_error("Data Record request missing value, length: %u", len);
            return;
        }
        uint8_t value = buffer[ind++];
        if (sub_mode == 1) {  // start/stop recording
            if (value > 0) {
                start_recording(dr);
            } else {
                stop_recording(dr);
            }
        } else if (sub_mode == 2) {  // set autostart on engage
            dr->autostart = value;
        } else if (sub_mode == 3) {  // set autostop on disengage
            dr->autostop = value;
        }
    } else if (mode == 2) {  // send
        if (sub_mode == 1) {  // header
            // pause recording; in theory a race, but the delay between sending the header
            // and getting a data request should be enough for any writing to end
            stop_recording(dr);
            send_header(dr);
        } else if (sub_mode == 2) {  // data
            if (len < 6) {
                log_error("Data Record request missing offset, length: %u", len);
                return;
            }

            size_t offset = buffer_get_uint32(buffer, &ind);
            send_data(dr, offset);
        }
    }
}
