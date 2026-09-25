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

#include "lcm.h"

#include "vesc_c_if.h"

#include "conf/buffer.h"
#include "lib/utils.h"

#include <math.h>

void lcm_init(LcmData *lcm, const CfgLeds *cfg) {
    lcm->cfg = cfg;
    lcm->enabled = cfg->led_type == LED_Type_External_Module;
    lcm->brightness = 0;
    lcm->brightness_idle = 0;
    lcm->status_brightness = 0;
    lcm->name[0] = '\0';
    lcm->payload_size = 0;
    lcm->lights_off_when_lifted = true;
}

void lcm_configure(LcmData *lcm, const Leds *leds) {
    const CfgLeds *cfg = leds->cfg;
    lcm->cfg = cfg;
    const LedsRuntimeStatus *status = leds_get_runtime_status(leds);

    if (!status->enabled) {
        lcm->brightness = 0.0f;
        lcm->brightness_idle = 0.0f;
        lcm->status_brightness = 0.0f;
    } else {
        lcm->brightness = status->headlights_enabled ? cfg->led_brightness : 0;
        lcm->brightness_idle = status->headlights_enabled ? cfg->led_brightness_idle : 0;
        lcm->status_brightness = cfg->led_status_brightness;
    }
    lcm->lights_off_when_lifted = true;
}

void lcm_poll_request(LcmData *lcm, uint8_t *buffer, size_t len) {
    if (!lcm->enabled) {
        return;
    }

    // Optionally pass in LCM name and version in a single string
    if (len > 0) {
        size_t name_len = min(len, sizeof(lcm->name) - 1);
        for (size_t i = 0; i < name_len; ++i) {
            lcm->name[i] = buffer[i];
        }
        lcm->name[name_len] = '\0';
    }
}

void lcm_poll_response(
    LcmData *lcm,
    const State *state,
    FootpadSensorState fs_state,
    const MotorData *motor,
    const float pitch
) {
    static const int bufsize = 20 + MAX_LCM_PAYLOAD_LENGTH;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_POLL;

    if (lcm->enabled) {
        uint8_t send_state = state_compat(state) & 0xF;
        send_state += fs_state << 4;
        if (state->mode == MODE_HANDTEST) {
            send_state |= 0x80;
        }

        buffer[ind++] = send_state;
        buffer[ind++] = VESC_IF->mc_get_fault();

        if (state->state == STATE_RUNNING) {
            buffer[ind++] = fminf(100, fabsf(motor->duty_cycle.value * 100));
        } else {
            // pitch is a value between -180 and +180, so abs(pitch) fits into uint8
            buffer[ind++] = lcm->lights_off_when_lifted ? fabsf(pitch) : 0;
        }

        buffer_append_float16(buffer, motor->erpm, 1e0, &ind);
        buffer_append_float16(buffer, VESC_IF->mc_get_tot_current_in(), 1e0, &ind);
        buffer_append_float16(buffer, VESC_IF->mc_get_input_voltage_filtered(), 1e1, &ind);

        // LCM control info
        buffer[ind++] = lcm->brightness;
        buffer[ind++] = lcm->brightness_idle;
        buffer[ind++] = lcm->status_brightness;

        // Relay any generic byte pairs set by cmd_light_ctrl
        for (uint8_t i = 0; i < lcm->payload_size; ++i) {
            buffer[ind++] = lcm->payload[i];
        }
        lcm->payload_size = 0;  // Message has been processed, clear it
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_light_info_response(const LcmData *lcm) {
    static const int bufsize = 15;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_LIGHT_INFO;

    buffer[ind++] = lcm->cfg->led_type;

    if (lcm->cfg->led_type != LED_Type_None) {
        buffer[ind++] = lcm->brightness;
        buffer[ind++] = lcm->brightness_idle;
        buffer[ind++] = lcm->status_brightness;

        buffer[ind++] = lcm->cfg->led_mode;
        buffer[ind++] = lcm->cfg->led_mode_idle;
        buffer[ind++] = lcm->cfg->led_status_mode;
        buffer[ind++] = lcm->cfg->led_status_count;
        buffer[ind++] = lcm->cfg->led_forward_count;
        buffer[ind++] = lcm->cfg->led_rear_count;
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_device_info_response(const LcmData *lcm) {
    static const int bufsize = MAX_LCM_NAME_LENGTH + 2;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_DEVICE_INFO;

    if (lcm->enabled) {
        // Write LCM firmware name into the buffer
        for (int i = 0; i < MAX_LCM_NAME_LENGTH; i++) {
            buffer[ind++] = lcm->name[i];
            if (lcm->name[i] == '\0') {
                break;
            }
        }
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_get_battery_response(const LcmData *lcm) {
    static const int bufsize = 10;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_GET_BATTERY;

    if (lcm->enabled) {
        buffer_append_float32_auto(buffer, VESC_IF->mc_get_battery_level(NULL), &ind);
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_light_ctrl_request(LcmData *lcm, unsigned char *cfg, int len) {
    if (!lcm->enabled || len < 3) {
        return;
    }

    lcm->payload_size = 0;
    int32_t idx = 0;

    lcm->brightness = min(cfg[idx++], 100u);
    lcm->brightness_idle = min(cfg[idx++], 100u);
    lcm->status_brightness = min(cfg[idx++], 100u);

    if (len > 3) {
        lcm->payload_size = min(len - idx, MAX_LCM_PAYLOAD_LENGTH);
        for (int i = 0; i < lcm->payload_size; i++) {
            lcm->payload[i] = cfg[idx + i];
        }
    }
}
