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
#include "utils.h"

#include <math.h>

void lcm_init(LcmData *lcm, CfgHwLeds *hw_cfg) {
    lcm->enabled = hw_cfg->type == LED_TYPE_EXTERNAL;
    lcm->brightness = 0;
    lcm->brightness_idle = 0;
    lcm->status_brightness = 0;
    lcm->name[0] = '\0';
    lcm->payload_size = 0;
}

void lcm_configure(LcmData *lcm, const CfgLeds *cfg) {
    if (!cfg->on) {
        lcm->brightness = 0.0f;
        lcm->brightness_idle = 0.0f;
        lcm->status_brightness = 0.0f;
    } else {
        if (cfg->headlights_on) {
            lcm->brightness = cfg->headlights.brightness * 100;
            lcm->status_brightness = cfg->status.brightness_headlights_on * 100;
        } else {
            lcm->brightness = cfg->front.brightness * 100;
            lcm->status_brightness = cfg->status.brightness_headlights_off * 100;
        }
        lcm->brightness_idle = cfg->front.brightness * 100;
    }
}

void lcm_poll_request(LcmData *lcm, uint8_t *buffer, size_t len) {
    if (!lcm->enabled) {
        return;
    }

    // Optionally pass in LCM name and version in a single string
    if (len > 0) {
        for (size_t i = 0; i < MAX_LCM_NAME_LENGTH; i++) {
            if (i > len || i > MAX_LCM_NAME_LENGTH - 1 || buffer[i] == '\0') {
                lcm->name[i] = '\0';
                break;
            }
            lcm->name[i] = buffer[i];
        }
    }
}

void lcm_poll_response(
    LcmData *lcm, const State *state, FootpadSensorState fs_state, const MotorData *motor
) {
    if (!lcm->enabled) {
        return;
    }

    static const int bufsize = 20 + MAX_LCM_PAYLOAD_LENGTH;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_POLL;

    uint8_t send_state = state_compat(state) & 0xF;
    send_state += fs_state << 4;
    if (state->mode == MODE_HANDTEST) {
        send_state |= 0x80;
    }

    buffer[ind++] = send_state;
    buffer[ind++] = VESC_IF->mc_get_fault();

    buffer[ind++] = fminf(100, fabsf(motor->duty_cycle * 100));
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

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_light_info_response(const LcmData *lcm) {
    if (!lcm->enabled) {
        return;
    }

    static const int bufsize = 15;
    uint8_t buffer[15];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_LIGHT_INFO;

    // Lights control for Refloat is not compatible with this interface; Send 3
    // for LCM (3 is he identifier for external led module in Float), otherwise 0.
    buffer[ind++] = lcm->enabled ? 3 : 0;

    buffer[ind++] = lcm->brightness;
    buffer[ind++] = lcm->brightness_idle;
    buffer[ind++] = lcm->status_brightness;

    // Don't send Float-specific configuration.
    buffer[ind++] = 0;  // led_mode
    buffer[ind++] = 0;  // mode_idle
    buffer[ind++] = 0;  // status_mode
    buffer[ind++] = 0;  // status_count
    buffer[ind++] = 0;  // forward_count
    buffer[ind++] = 0;  // rear_count

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_device_info_response(const LcmData *lcm) {
    if (!lcm->enabled) {
        return;
    }

    static const int bufsize = MAX_LCM_NAME_LENGTH + 2;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_DEVICE_INFO;

    // Write LCM firmware name into the buffer
    for (int i = 0; i < MAX_LCM_NAME_LENGTH; i++) {
        buffer[ind++] = lcm->name[i];
        if (lcm->name[i] == '\0') {
            break;
        }
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_get_battery_response(const LcmData *lcm) {
    if (!lcm->enabled) {
        return;
    }

    static const int bufsize = 10;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LCM_GET_BATTERY;

    buffer_append_float32_auto(buffer, VESC_IF->mc_get_battery_level(NULL), &ind);

    SEND_APP_DATA(buffer, bufsize, ind);
}

void lcm_light_ctrl_request(LcmData *lcm, unsigned char *cfg, int len) {
    if (!lcm->enabled || len < 3) {
        return;
    }

    int32_t idx = 0;

    lcm->brightness = cfg[idx++];
    lcm->brightness_idle = cfg[idx++];
    lcm->status_brightness = cfg[idx++];

    if (len > 3) {
        if (lcm->enabled) {
            // Copy rest of payload into data for LCM to pull
            lcm->payload_size = len - idx;
            for (int i = 0; i < lcm->payload_size; i++) {
                lcm->payload[i] = cfg[idx + i];
            }
        } else {
            if (len > 5) {
                // d->float_conf.led_mode = cfg[idx++];
                // d->float_conf.led_mode_idle = cfg[idx++];
                // d->float_conf.led_status_mode = cfg[idx++];
            }
        }
    }
}
