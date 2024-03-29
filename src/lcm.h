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

#include "footpad_sensor.h"
#include "motor_data.h"
#include "state.h"

#include <stddef.h>

typedef enum {
    COMMAND_LCM_POLL = 24,  // this should only be called by external light modules
    COMMAND_LCM_LIGHT_INFO = 25,  // to be called by apps to get lighting info
    COMMAND_LCM_LIGHT_CTRL = 26,  // to be called by apps to change light settings
    COMMAND_LCM_DEVICE_INFO = 27,  // to be called by apps to check lighting controller firmware
    COMMAND_LCM_GET_BATTERY = 29,

    COMMAND_LCM_DEBUG = 99,  // reserved for external debug purposes
} LcmCommands;

#define MAX_LCM_NAME_LENGTH 20
#define MAX_LCM_PAYLOAD_LENGTH 64

typedef struct {
    bool enabled;
    uint8_t brightness;
    uint8_t brightness_idle;
    uint8_t status_brightness;
    bool lights_off_when_lifted;

    char name[MAX_LCM_NAME_LENGTH];
    uint8_t payload[MAX_LCM_PAYLOAD_LENGTH];
    uint8_t payload_size;
} LcmData;

void lcm_init(LcmData *lcm, CfgHwLeds *hw_cfg);

void lcm_configure(LcmData *lcm, const CfgLeds *cfg);

/**
 * Poll request from LCM with any data that need to be passed to the package.
 */
void lcm_poll_request(LcmData *lcm, uint8_t *buffer, size_t len);

/**
 * Response to the LCM poll request to get data from the package.
 */
void lcm_poll_response(
    LcmData *lcm,
    const State *state,
    FootpadSensorState fs_state,
    const MotorData *motor,
    const float pitch
);

/**
 * Command for apps to call to get info about lighting.
 */
void lcm_light_info_response(const LcmData *lcm);

/**
 * Command for apps to call to get LCM hardware info (if any).
 */
void lcm_device_info_response(const LcmData *lcm);

/**
 * No idea why this exists. Seems it should have been part of lcm_poll_response().
 */
void lcm_get_battery_response(const LcmData *lcm);

/**
 * Command for apps to call to control LCM details (lights, behavior, etc).
 */
void lcm_light_ctrl_request(LcmData *lcm, unsigned char *cfg, int len);
