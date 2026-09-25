#ifndef FLOATLED_H_
#define FLOATLED_H_

#include "conf/datatypes.h"

typedef struct {
    float led_last_updated;
    uint8_t led_previous_brightness;
    bool led_latching_direction;
    uint8_t led_type;
	uint8_t led_status_count;
	uint8_t led_forward_count;
	uint8_t led_rear_count;
    int ledbuf_len;
    int bitbuf_len;
    uint16_t* bitbuffer;
    uint32_t* RGBdata;
} LEDData;

void led_init(LEDData* led_data, const CfgLeds* float_conf);
void led_update(LEDData* led_data, const CfgLeds* float_conf, float current_time, float erpm, float abs_duty_cycle, int switch_state, int float_state, float fault_adc_half_erpm);
void led_stop(LEDData* led_data);
void led_strip_set_color(LEDData* led_data, int offset, int length, uint32_t color,
                         uint32_t brightness, bool fade);

#endif /* FLOATLED_H_ */
