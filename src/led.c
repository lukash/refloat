#include "led.h"

// Lighting implementation adapted from Float 2.0c (surfdado/vesc_pkg).
#include "st_types.h"
#include "vesc_c_if.h"

#include "conf/datatypes.h"

#include <math.h>
#include <stdlib.h>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#define WS2812_CLK_HZ		800000
#define TIM_PERIOD			(((168000000 / 2 / WS2812_CLK_HZ) - 1))
#define WS2812_ZERO			(TIM_PERIOD * 0.3)
#define WS2812_ONE			(TIM_PERIOD * 0.7)
#define BITBUFFER_PAD		1000

uint32_t led_rgb_to_local(uint32_t color, uint8_t brightness, bool rgbw) {
    uint32_t w = (color >> 24) & 0xFF;
    uint32_t r = (color >> 16) & 0xFF;
    uint32_t g = (color >> 8) & 0xFF;
    uint32_t b = color & 0xFF;

    r = (r * brightness) / 100;
    g = (g * brightness) / 100;
    b = (b * brightness) / 100;
    w = (w * brightness) / 100;

    if (rgbw) {
        return (g << 24) | (r << 16) | (b << 8) | w;
    } else {
        return (g << 16) | (r << 8) | b;
    }
}

int stepTowards(int from, int to, int step_size){
    if (from < to) {
        if (from + step_size > to) {
            return to;
        } else {
            return from + step_size;
        }
    } else if (from > to) {
        if (from - step_size < to) {
            return to;
        } else {
            return from - step_size;
        }
    }
    return to;
}

uint32_t led_fade_color(uint32_t from, uint32_t to) {
    uint32_t fw = (from >> 24) & 0xFF;
    uint32_t fr = (from >> 16) & 0xFF;
    uint32_t fg = (from >> 8) & 0xFF;
    uint32_t fb = from & 0xFF;

    uint8_t tw = (to >> 24) & 0xFF;
    uint8_t tr = (to >> 16) & 0xFF;
    uint8_t tg = (to >> 8) & 0xFF;
    uint8_t tb = to & 0xFF;

    fw = stepTowards(fw, tw, 12);
    fr = stepTowards(fr, tr, 12);
    fg = stepTowards(fg, tg, 12);
    fb = stepTowards(fb, tb, 12);

    return (fw << 24) | (fr << 16) | (fg << 8) | fb;
}

void led_init_dma(LEDData* led_data) {
    // Init
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    TIM_TypeDef* tim;
    DMA_Stream_TypeDef* dma_stream;
    uint32_t dma_ch;

    // Always GPIOB PIN7 #dealwithit
    tim = TIM4;
    dma_stream = DMA1_Stream3;
    dma_ch = DMA_Channel_2;
    VESC_IF->set_pad_mode(GPIOB, 7, PAL_MODE_ALTERNATE(2) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1);

    TIM_DeInit(tim);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(dma_stream);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&tim->CCR2;

    DMA_InitStructure.DMA_Channel = dma_ch;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(led_data->bitbuffer);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = led_data->bitbuf_len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(dma_stream, &DMA_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = led_data->bitbuffer[0];
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(tim, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(tim, ENABLE);

    TIM_Cmd(tim, ENABLE);

    DMA_Cmd(dma_stream, ENABLE);

    TIM_DMACmd(tim, TIM_DMA_CC2, ENABLE);
}

void led_init(LEDData* led_data, const CfgLeds* float_conf) {
    if ((float_conf->led_type == LED_Type_None) || (float_conf->led_type == LED_Type_External_Module)) {
        led_data->led_type = float_conf->led_type;
        return;
    }

    // De-init
    led_stop(led_data);

    // Copy params that require a reboot
    led_data->led_type = float_conf->led_type;
    led_data->led_status_count = float_conf->led_status_count;
    led_data->led_forward_count = float_conf->led_forward_count;
    led_data->led_rear_count = float_conf->led_rear_count;

    // Init
    int bits = 0;
    if (led_data->led_type == LED_Type_RGB) {
        bits = 24;
    } else if (led_data->led_type == LED_Type_RGBW) {
        bits = 32;
    } else {
        return;
    }

    led_data->ledbuf_len = led_data->led_status_count + led_data->led_forward_count + led_data->led_rear_count + 1;
    led_data->bitbuf_len = bits * led_data->ledbuf_len + BITBUFFER_PAD;

    bool ok = false;
    led_data->bitbuffer = VESC_IF->malloc(sizeof(uint16_t) * led_data->bitbuf_len);
    led_data->RGBdata = VESC_IF->malloc(sizeof(uint32_t) * led_data->ledbuf_len);
    ok = led_data->bitbuffer != NULL && led_data->RGBdata != NULL;
    if (!ok) {
        led_stop(led_data);
        VESC_IF->printf("LED setup failed, out of memory\n");
        return;
    }

    led_data->led_last_updated = 0;
    led_data->led_previous_brightness = 0;
    led_data->led_latching_direction = true;

    // Default LED values
    for (int i = 0;i < led_data->ledbuf_len;i++) {
        led_data->RGBdata[i] = 0;
    }
    for (int i = 0;i < led_data->ledbuf_len;i++) {
        uint32_t tmp_color = led_rgb_to_local(0x00000000, 0x00, bits == 32);

        for (int bit = 0;bit < bits;bit++) {
            if (tmp_color & (1u << (bits - 1))) {
                led_data->bitbuffer[bit + i * bits] = WS2812_ONE;
            } else {
                led_data->bitbuffer[bit + i * bits] = WS2812_ZERO;
            }
            tmp_color <<= 1;
        }
    }

    // Fill the rest of the buffer with zeros to give the LEDs a chance to update
    // after sending all bits
    for (int i = 0;i < BITBUFFER_PAD;i++) {
        led_data->bitbuffer[led_data->bitbuf_len - BITBUFFER_PAD + i] = 0;
    }

    led_init_dma(led_data);
    return;
}

void led_set_color(LEDData* led_data, int led, uint32_t color, uint32_t brightness, bool fade) {
    if ((led_data->led_type == LED_Type_None) || (led_data->led_type == LED_Type_External_Module)) {
        return;
    }
    if (led >= 0 && led < led_data->ledbuf_len) {
        if (fade) {
            color = led_fade_color(led_data->RGBdata[led], color);
        }
        led_data->RGBdata[led] = color;

        color = led_rgb_to_local(color, brightness, led_data->led_type == 2);

        int bits = 0;
        if (led_data->led_type == 1) {
            bits = 24;
        } else {
            bits = 32;
        }

        int bit;
        for (bit = 0;bit < bits;bit++) {
            if (color & (1u << (bits - 1))) {
                led_data->bitbuffer[bit + led * bits] = WS2812_ONE;
            } else {
                led_data->bitbuffer[bit + led * bits] = WS2812_ZERO;
            }
            color <<= 1;
        }
    }
}

void led_strip_set_color(LEDData* led_data, int offset, int length, uint32_t color, uint32_t brightness, bool fade) {
    for(int led= offset; led < offset+length; led++){
        led_set_color(led_data, led, color, brightness, fade);
    }
}

void led_display_battery(LEDData* led_data, int brightness, int strip_offset, int strip_length, bool fade) {
    float batteryLevel = VESC_IF->mc_get_battery_level(NULL);
    int batteryLeds = (int)(batteryLevel * strip_length);
    int batteryColor = 0x0000FF00;
    if (batteryLevel < .4) {
        batteryColor = 0x00FFFF00;
    }
    if (batteryLevel < .2) {
        batteryColor = 0x00FF0000;
    }
    for (int i = strip_offset; i < strip_offset + strip_length; i++) {
        if (i < strip_offset + batteryLeds) {
            led_set_color(led_data, i, batteryColor, brightness, fade);
        } else {
            led_set_color(led_data, i, 0x00000000, brightness, fade);
        }
    }
}

void led_float_disabled(LEDData* led_data, int brightness, int first, int count) {
    // show red LEDs in the center of the light bar
    int start = count / 4;
    int end = count * 3 / 4;
    if (end < 3) { // 3 or fewer LEDs in total? Show red lights across the entire light bar
        end = count;
    }
    for (int i = first; i < first + count; i++) {
        int bright = brightness;
        if ((end > 4) && ((i == first+start) || (i == first+end-1))) // first and last red led at half brightness
            bright /= 2;
        if ((i < first+start) || (i > first+end)) // outer LEDs are off
            bright = 0;
        led_set_color(led_data, i, 0x00FF0000, bright, false);
    }
}

void led_update(LEDData* led_data, const CfgLeds* float_conf, float current_time, float erpm, float abs_duty_cycle, int switch_state, int float_state, float fault_adc_half_erpm) {
    ///////////////////////
    // Status LED Logic //
    /////////////////////
    if ((float_conf->led_type == LED_Type_None) || (float_conf->led_type == LED_Type_External_Module)) {
        return;
    }
    if (current_time - led_data->led_last_updated < 0.05) {
        return;
    }
    led_data->led_last_updated = current_time;

    if (led_data->led_status_count > 0) {
        int statusBrightness = (int)(float_conf->led_status_brightness);
        if (float_state == 15) {
            led_float_disabled(led_data, statusBrightness, 0, led_data->led_status_count);
        }
        else if (fabsf(erpm) < fault_adc_half_erpm) {
            // Display status LEDs
            if (switch_state == 0) {
                led_display_battery(led_data, statusBrightness, 0, led_data->led_status_count, false);
            } else if (switch_state == 1) {
                for (int i = 0; i < led_data->led_status_count; i++) {
                    if (i < led_data->led_status_count / 2) {
                        led_set_color(led_data, i, 0x000000FF, statusBrightness, false);
                    } else {
                        led_set_color(led_data, i, 0x00000000, 0, false);
                    }
                }
            } else {
                for (int i = 0; i < led_data->led_status_count; i++) {
                    led_set_color(led_data, i, 0x000000FF, statusBrightness, false);
                }
            }
        } else {
            // Display duty cycle when riding
            int dutyLeds = (int)(fminf((abs_duty_cycle * 1.1112), 1) * led_data->led_status_count);
            uint32_t dutyColor = 0x00FFFF00;
            if (abs_duty_cycle > 0.85) {
                dutyColor = 0x00FF0000;
            } else if (abs_duty_cycle > 0.7) {
                dutyColor = 0x00FF8800;
            }

            for (int i = 0; i < led_data->led_status_count; i++) {
                if (i < dutyLeds) {
                    led_set_color(led_data, i, dutyColor, statusBrightness, false);
                } else {
                    led_set_color(led_data, i, 0x00000000, 0, false);
                }
            }
        }
    }

    int brightness = float_conf->led_brightness;
    if (float_state > 5) {
        // board is disengaged
        brightness = float_conf->led_brightness_idle;
    }
    if (brightness > led_data->led_previous_brightness) {
        led_data->led_previous_brightness += 5;
        if (led_data->led_previous_brightness > brightness) {
            led_data->led_previous_brightness = brightness;
        }
    } else if (brightness < led_data->led_previous_brightness) {
        led_data->led_previous_brightness -= 5;
        if (led_data->led_previous_brightness < brightness) {
            led_data->led_previous_brightness = brightness;
        }
    }
    brightness = led_data->led_previous_brightness;

    //////////////////////////
    // Pattern strip logic //
    ////////////////////////

    // calculate directonality
    int forwardOffsetDirectional = led_data->led_status_count;
    int forwardlengthDirectional = led_data->led_forward_count;
    int rearOffsetDirectional = led_data->led_status_count + led_data->led_forward_count;
    int rearlengthDirectional = led_data->led_rear_count;
    if (float_state == 4) { // RUNNING_UPSIDEDOWN aka drark ride => flip erpm sign
        erpm = -erpm;
    }

    if (erpm > 100) {
        led_data->led_latching_direction = true;
    } else if (erpm < -100) {
        led_data->led_latching_direction = false;
    }
    if (led_data->led_latching_direction == false) {
        forwardOffsetDirectional = led_data->led_status_count + led_data->led_forward_count;
        forwardlengthDirectional = led_data->led_rear_count;
        rearOffsetDirectional = led_data->led_status_count;
        rearlengthDirectional = led_data->led_forward_count;
    }

    // Find color
    int led_mode = float_state < 5 ? float_conf->led_mode : float_conf->led_mode_idle;
    if (float_state == 15) { // disabled board? front leds show red in center, rear leds off
        led_float_disabled(led_data, brightness, led_data->led_status_count, led_data->led_forward_count);
        led_strip_set_color(led_data, led_data->led_status_count + led_data->led_forward_count, led_data->led_rear_count, 0, 0, true);
        return;
    }else if (led_mode == 0) { // Red/White
        led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0xFFFFFFFF, brightness, true);
        led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FF0000, brightness, true);
        return;
    } else if (led_mode == 1) { //Battery Meter
        led_display_battery(led_data, brightness, led_data->led_status_count, led_data->led_forward_count, true);
        led_display_battery(led_data, brightness, led_data->led_status_count + led_data->led_forward_count, led_data->led_rear_count, true);
        return;
    } else if (led_mode == 2) { // Cyan/Magenta
        led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x0000FFFF, brightness, true);
        led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FF00FF, brightness, true);
        return;
    } else if (led_mode == 3) { // Blue/Green
        led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x000000FF, brightness, true);
        led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x0000FF00, brightness, true);
        return;
    } else if (led_mode == 4) { // Yellow/Green
        led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x00FFFF00, brightness, true);
        led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x0000FF00, brightness, true);
        return;
    } else if (led_mode == 5) { // RGB Fade
        uint16_t time = (unsigned long)(current_time * 1000) % 3000;
        uint32_t color;
        if (time <= 1000) {
            color =  (uint32_t)(255 * (1 - (time / 1000.0))) << 16 | (uint8_t)(255 * (time / 1000.0));
        } else if (time <= 2000) {
            color = (uint16_t)(255 * ((time - 1000) / 1000.0)) << 8 | (uint8_t)(255 * (1 - ((time - 1000) / 1000.0)));
        } else {
            color =  (uint32_t)(255 * ((time - 2000) / 1000.0)) << 16 | (uint16_t)(255 * (1 - ((time - 2000) / 1000.0))) << 8 ;
        }
        led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, color, brightness, false);
        led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, color, brightness, false);
        return;
    } else if (led_mode == 6) { // Strobe
        if ((unsigned long)(current_time * 1000) % 100 < 50) {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x00000000, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00000000, brightness, false);
        } else {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0xFFFFFFFF, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0xFFFFFFFF, brightness, false);
        }
        return;
    } else if (led_mode == 7) { // Rave
        uint16_t time = (unsigned long)(current_time * 1000) % 300;
        if (time < 50) {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x00FFFF00, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FFFF00, brightness, false);
        } else if (time < 100) {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x0000FF00, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x0000FF00, brightness, false);
        } else if (time < 150) {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x0000FFFF, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x0000FFFF, brightness, false);
        } else if (time < 200) {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x000000FF, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x000000FF, brightness, false);
        } else if (time < 250) {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x00FF00FF, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FF00FF, brightness, false);
        } else {
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0x00FF0000, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FF0000, brightness, false);
        }
        return;
    } else if (led_mode == 8) { // Mullet
        led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional, 0xFFFFFFFF, brightness, true);
        uint16_t time = (unsigned long)(current_time * 1000) % 300;
        if (time < 50) {
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FFFF00, brightness, false);
        } else if (time < 100) {
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x0000FF00, brightness, false);
        } else if (time < 150) {
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x0000FFFF, brightness, false);
        } else if (time < 200) {
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x000000FF, brightness, false);
        } else if (time < 250) {
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FF00FF, brightness, false);
        } else {
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional, 0x00FF0000, brightness, false);
        }
        return;
    } else if (led_mode == 9){ // Knight Rider
        int centerPosition = (fmod(current_time, 1) * forwardlengthDirectional) + forwardOffsetDirectional;
        if(fmod(current_time, 2) > 1){
            centerPosition = forwardOffsetDirectional + forwardlengthDirectional - centerPosition + forwardOffsetDirectional;
        }
        for(int i = forwardOffsetDirectional; i < forwardOffsetDirectional + forwardlengthDirectional; i++){
            uint32_t intensity = (((forwardlengthDirectional/2)-min(abs(i-centerPosition), forwardlengthDirectional/2))
            /(forwardlengthDirectional/2.0)) * 255;
            led_set_color(led_data, i, intensity << 16, brightness, false);
        }
        centerPosition = (fmod(current_time, 1) * rearlengthDirectional) + rearOffsetDirectional;
        if(fmod(current_time, 2) > 1){
            centerPosition = rearOffsetDirectional + rearlengthDirectional - centerPosition + rearOffsetDirectional;
        }
        for(int i = rearOffsetDirectional; i < rearOffsetDirectional + rearlengthDirectional; i++){
            uint32_t intensity = (((rearlengthDirectional/2)-min(abs(i-centerPosition), rearlengthDirectional/2))/(rearlengthDirectional/2.0)) * 255;
            led_set_color(led_data, i, intensity << 16, brightness, false);
        }
        return;
    } else if (led_mode == 10){ // Felony
        uint8_t state = 0;
        if((unsigned long)(current_time * 1000) % 150 < 50){
            state = 1;
        }else if((unsigned long)(current_time * 1000) % 150 < 100){
            state = 2;
        }
        if(state == 0){
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional / 2, 0x00000000, brightness, false);
            led_strip_set_color(led_data, forwardOffsetDirectional + forwardlengthDirectional / 2, forwardlengthDirectional, 0x00FF0000, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional / 2, 0x00000000, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional + rearlengthDirectional / 2, rearlengthDirectional, 0x00FF0000, brightness, false);
        } else if (state == 1){
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional / 2, 0x00FF0000, brightness, false);
            led_strip_set_color(led_data, forwardOffsetDirectional + forwardlengthDirectional / 2, forwardlengthDirectional, 0x000000FF, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional / 2, 0x00FF0000, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional + rearlengthDirectional / 2, rearlengthDirectional, 0x000000FF, brightness, false);
        }else{
            led_strip_set_color(led_data, forwardOffsetDirectional, forwardlengthDirectional / 2, 0x000000FF, brightness, false);
            led_strip_set_color(led_data, forwardOffsetDirectional + forwardlengthDirectional / 2, forwardlengthDirectional, 0x00000000, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional, rearlengthDirectional / 2, 0x000000FF, brightness, false);
            led_strip_set_color(led_data, rearOffsetDirectional + rearlengthDirectional / 2, rearlengthDirectional, 0x00000000, brightness, false);
        }
        return;
    }
}

void led_stop(LEDData* led_data) {
    if (led_data->led_type == LED_Type_None || led_data->led_type == LED_Type_External_Module) {
        return;
    }

    TIM_DeInit(TIM4);
    DMA_DeInit(DMA1_Stream3);

    led_data->ledbuf_len = 0;
    led_data->bitbuf_len = 0;
    if (led_data->bitbuffer) {
        VESC_IF->free(led_data->bitbuffer);
        led_data->bitbuffer = NULL;
    }
    if (led_data->RGBdata) {
        VESC_IF->free(led_data->RGBdata);
        led_data->RGBdata = NULL;
    }
    led_data->led_type = LED_Type_None;
}
