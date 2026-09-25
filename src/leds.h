#pragma once

#include "conf/datatypes.h"
#include "footpad_sensor.h"
#include "led.h"
#include "motor_data.h"
#include "state.h"

#define LEDS_REFRESH_RATE 20

typedef struct {
    bool enabled;
    bool headlights_enabled;
} LedsRuntimeStatus;

typedef struct {
    LEDData data;
    const CfgLeds *cfg;
    LedsRuntimeStatus runtime_status;
    float confirm_until;
} Leds;

void leds_init(Leds *leds);
void leds_setup(Leds *leds, const CfgLeds *cfg);
void leds_configure(Leds *leds, const CfgLeds *cfg);
const LedsRuntimeStatus *leds_get_runtime_status(const Leds *leds);
void leds_set_enabled(Leds *leds, bool value);
void leds_set_headlights_enabled(Leds *leds, bool value);
void leds_update(Leds *leds, const State *state, const MotorData *motor,
                 FootpadSensorState fs_state, float fault_adc_half_erpm);
void leds_status_confirm(Leds *leds);
void leds_destroy(Leds *leds);
