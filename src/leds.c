#include "leds.h"

#include "vesc_c_if.h"

#include <math.h>
#include <string.h>

void leds_init(Leds *leds) {
    memset(leds, 0, sizeof(*leds));
    leds->runtime_status.enabled = true;
    leds->runtime_status.headlights_enabled = true;
}

void leds_setup(Leds *leds, const CfgLeds *cfg) {
    leds->cfg = cfg;
    led_init(&leds->data, cfg);
}

void leds_configure(Leds *leds, const CfgLeds *cfg) {
    leds->cfg = cfg;
}

const LedsRuntimeStatus *leds_get_runtime_status(const Leds *leds) {
    return &leds->runtime_status;
}

void leds_set_enabled(Leds *leds, bool value) {
    leds->runtime_status.enabled = value;
}

void leds_set_headlights_enabled(Leds *leds, bool value) {
    leds->runtime_status.headlights_enabled = value;
}

void leds_update(Leds *leds, const State *state, const MotorData *motor,
                 FootpadSensorState fs_state, float fault_adc_half_erpm) {
    if (leds->cfg->led_type != LED_Type_RGB && leds->cfg->led_type != LED_Type_RGBW) {
        return;
    }

    CfgLeds runtime_cfg = *leds->cfg;
    if (!leds->runtime_status.enabled) {
        runtime_cfg.led_brightness = 0;
        runtime_cfg.led_brightness_idle = 0;
        runtime_cfg.led_status_brightness = 0;
    } else if (!leds->runtime_status.headlights_enabled) {
        runtime_cfg.led_brightness = 0;
        runtime_cfg.led_brightness_idle = 0;
    }

    float now = VESC_IF->system_time();
    int float_state = state_compat(state);
    led_update(&leds->data, &runtime_cfg, now, motor->erpm,
               fabsf(motor->duty_cycle.value), footpad_sensor_state_to_switch_compat(fs_state),
               float_state, fault_adc_half_erpm);

    if (now < leds->confirm_until && leds->data.RGBdata) {
        led_strip_set_color(&leds->data, 0, leds->data.led_status_count,
                            0x00FFFFFF, runtime_cfg.led_status_brightness, false);
    }
}

void leds_status_confirm(Leds *leds) {
    leds->confirm_until = VESC_IF->system_time() + 0.2f;
}

void leds_destroy(Leds *leds) {
    led_stop(&leds->data);
}
