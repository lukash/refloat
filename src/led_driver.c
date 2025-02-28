// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#include "led_driver.h"

#include "st_types.h"

#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>
#include <string.h>

#define WS2812_CLK_HZ 800000
#define TIM_PERIOD ((168000000 / 2 / WS2812_CLK_HZ) - 1)
#define WS2812_ZERO (((uint32_t) TIM_PERIOD) * 0.3)
#define WS2812_ONE (((uint32_t) TIM_PERIOD) * 0.7)
#define BITBUFFER_PAD 1000

static DMA_Stream_TypeDef *get_dma_stream(LedPin pin) {
    if (pin == LED_PIN_B6) {
        return DMA1_Stream0;
    } else {
        return DMA1_Stream3;
    }
}

static void init_dma(LedPin pin, uint16_t *buffer, uint32_t length) {
    TIM_TypeDef *tim = TIM4;
    uint32_t dma_ch = DMA_Channel_2;
    DMA_Stream_TypeDef *dma_stream = get_dma_stream(pin);

    // settings for different pins in one place, can be extended if more pins are added
    uint8_t pin_nr;
    uint32_t CCR_address;
    uint16_t DMA_source;
    if (pin == LED_PIN_B6) {
        pin_nr = 6;
        CCR_address = (uint32_t) &tim->CCR1;
        DMA_source = TIM_DMA_CC1;
    } else {
        pin_nr = 7;
        CCR_address = (uint32_t) &tim->CCR2;
        DMA_source = TIM_DMA_CC2;
    }

    VESC_IF->set_pad_mode(
        GPIOB, pin_nr, PAL_MODE_ALTERNATE(2) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1
    );

    TIM_DeInit(tim);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(dma_stream);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = CCR_address;
    DMA_InitStructure.DMA_Channel = dma_ch;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (buffer);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = length;
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

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = buffer[0];
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    if (CCR_address == (uint32_t) &tim->CCR1) {
        TIM_OC1Init(tim, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    } else {
        TIM_OC2Init(tim, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(tim, ENABLE);

    TIM_Cmd(tim, ENABLE);

    DMA_Cmd(dma_stream, ENABLE);

    TIM_DMACmd(tim, DMA_source, ENABLE);
}

static void deinit_dma(LedPin pin) {
    TIM_DeInit(TIM4);
    DMA_DeInit(get_dma_stream(pin));
}

inline static uint8_t color_order_bits(LedColorOrder order) {
    switch (order) {
    case LED_COLOR_GRBW:
    case LED_COLOR_WRGB:
        return 32;
    case LED_COLOR_GRB:
    case LED_COLOR_RGB:
        return 24;
    }

    // the switch above should be exhaustive, just silence the warning
    return 24;
}

void led_driver_init(LedDriver *driver) {
    driver->bitbuffer_length = 0;
    driver->bitbuffer = NULL;
}

bool led_driver_setup(LedDriver *driver, LedPin pin, const LedStrip **led_strips) {
    driver->bitbuffer_length = 0;

    size_t offsets[3] = {0};
    for (size_t i = 0; i < STRIP_COUNT; ++i) {
        const LedStrip *strip = led_strips[i];
        if (!strip) {
            driver->strips[i] = NULL;
            driver->strip_bitbuffs[i] = NULL;
            continue;
        }

        driver->strips[i] = strip;
        offsets[i] = driver->bitbuffer_length;
        driver->bitbuffer_length += color_order_bits(strip->color_order) * strip->length;
    }

    uint32_t padding_offset = driver->bitbuffer_length;

    driver->bitbuffer_length += BITBUFFER_PAD;
    driver->bitbuffer = VESC_IF->malloc(sizeof(uint16_t) * driver->bitbuffer_length);
    driver->pin = pin;

    if (!driver->bitbuffer) {
        log_error("Failed to init LED driver, out of memory.");
        return false;
    }

    for (size_t i = 0; i < STRIP_COUNT; ++i) {
        if (driver->strips[i]) {
            driver->strip_bitbuffs[i] = driver->bitbuffer + offsets[i];
        }
    }

    for (uint32_t i = 0; i < padding_offset; ++i) {
        driver->bitbuffer[i] = WS2812_ZERO;
    }

    memset(driver->bitbuffer + padding_offset, 0, sizeof(uint16_t) * BITBUFFER_PAD);
    init_dma(pin, driver->bitbuffer, driver->bitbuffer_length);
    return true;
}

inline static uint8_t cgamma(uint8_t c) {
    return (c * c + c) / 256;
}

static uint32_t color_grb(uint8_t w, uint8_t r, uint8_t g, uint8_t b) {
    unused(w);
    return (g << 16) | (r << 8) | b;
}

static uint32_t color_grbw(uint8_t w, uint8_t r, uint8_t g, uint8_t b) {
    return (g << 24) | (r << 16) | (b << 8) | w;
}

static uint32_t color_rgb(uint8_t w, uint8_t r, uint8_t g, uint8_t b) {
    unused(w);
    return (r << 16) | (g << 8) | b;
}

static uint32_t color_wrgb(uint8_t w, uint8_t r, uint8_t g, uint8_t b) {
    return (w << 24) | (r << 16) | (g << 8) | b;
}

void led_driver_paint(LedDriver *driver) {
    if (!driver->bitbuffer) {
        return;
    }

    for (size_t i = 0; i < STRIP_COUNT; ++i) {
        const LedStrip *strip = driver->strips[i];
        if (!strip) {
            break;
        }

        uint32_t (*color_conv)(uint8_t, uint8_t, uint8_t, uint8_t);
        switch (strip->color_order) {
        case LED_COLOR_GRB:
            color_conv = color_grb;
            break;
        case LED_COLOR_GRBW:
            color_conv = color_grbw;
            break;
        case LED_COLOR_RGB:
            color_conv = color_rgb;
            break;
        case LED_COLOR_WRGB:
            color_conv = color_wrgb;
            break;
        }

        uint8_t bits = color_order_bits(strip->color_order);
        uint16_t *strip_bitbuffer = driver->strip_bitbuffs[i];

        for (uint32_t j = 0; j < strip->length; ++j) {
            uint32_t color = strip->data[j];
            uint8_t w = cgamma((color >> 24) & 0xFF);
            uint8_t r = cgamma((color >> 16) & 0xFF);
            uint8_t g = cgamma((color >> 8) & 0xFF);
            uint8_t b = cgamma(color & 0xFF);

            color = color_conv(w, r, g, b);

            for (int8_t bit = bits - 1; bit >= 0; --bit) {
                strip_bitbuffer[bit + j * bits] = color & 0x1 ? WS2812_ONE : WS2812_ZERO;
                color >>= 1;
            }
        }
    }
}

void led_driver_destroy(LedDriver *driver) {
    if (driver->bitbuffer) {
        // only touch the timer/DMA if we inited it - something else could be using it
        deinit_dma(driver->pin);

        VESC_IF->free(driver->bitbuffer);
        driver->bitbuffer = NULL;
    }
    driver->bitbuffer_length = 0;
}
