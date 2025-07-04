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

#include <string.h>

#define WS2812_CLK_HZ 800000
#define TIM_PERIOD ((168000000 / 2 / WS2812_CLK_HZ) - 1)
#define WS2812_ZERO (((uint32_t) TIM_PERIOD) * 0.3)
#define WS2812_ONE (((uint32_t) TIM_PERIOD) * 0.7)

typedef struct {
    uint8_t pin_nr;
    DMA_Stream_TypeDef *dma_stream;
    uint16_t dma_source;
    uint32_t ccr_address;
} PinHwConfig;

static PinHwConfig get_pin_hw_config(LedPin pin) {
    PinHwConfig cfg;
    switch (pin) {
    case LED_PIN_B6:
        cfg.pin_nr = 6;
        cfg.dma_stream = DMA1_Stream0;
        cfg.dma_source = TIM_DMA_CC1;
        cfg.ccr_address = (uint32_t) &TIM4->CCR1;
        break;
    case LED_PIN_B7:
        cfg.pin_nr = 7;
        cfg.dma_stream = DMA1_Stream3;
        cfg.dma_source = TIM_DMA_CC2;
        cfg.ccr_address = (uint32_t) &TIM4->CCR2;
        break;
    }

    return cfg;
}

static void reset_tim4() {
    RCC->APB1RSTR |= RCC_APB1Periph_TIM4;
    RCC->APB1RSTR &= ~RCC_APB1Periph_TIM4;
}

static void init_tim4(bool ccr1) {
    // enable the Low Speed APB (APB1) peripheral clock for TIM4
    RCC->APB1ENR |= RCC_APB1Periph_TIM4;

    // init TIM4 registers
    TIM4->CR1 |= TIM_CounterMode_Up;
    TIM4->ARR = TIM_PERIOD;

    if (ccr1) {
        TIM4->CCMR1 |= TIM_OCMode_PWM1;
        TIM4->CCER |= TIM_OCPolarity_High | TIM_OutputState_Enable;
        TIM4->CCMR1 |= TIM_OCPreload_Enable;
    } else {
        TIM4->CCMR1 |= TIM_OCMode_PWM1 << 8;
        TIM4->CCER |= TIM_OCPolarity_High << 4 | TIM_OutputState_Enable << 4;
        TIM4->CCMR1 |= TIM_OCPreload_Enable << 8;
    }

    // enable TIM4 peripheral Preload register on ARR and enable TIM4
    TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void disable_dma_stream(DMA_Stream_TypeDef *dma_stream) {
    dma_stream->CR &= ~DMA_SxCR_EN;
}

static void enable_dma_stream(DMA_Stream_TypeDef *dma_stream) {
    dma_stream->CR |= DMA_SxCR_EN;
}

// Only DMA_Stream0 and DMA_Stream3 supported as of now.
static void init_dma_stream(
    DMA_Stream_TypeDef *dma_stream, uint16_t *buf, uint32_t buf_len, uint32_t ccr_address
) {
    // enable the AHB1 peripheral clock for DMA1
    RCC->AHB1ENR |= RCC_AHB1Periph_DMA1;

    disable_dma_stream(dma_stream);

    dma_stream->M1AR = 0;

    const uint32_t dma_stream0_it_mask =
        DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0;
    if (dma_stream == DMA1_Stream0) {
        DMA1->LIFCR = dma_stream0_it_mask;
    } else {
        DMA1->LIFCR = dma_stream0_it_mask << 22;
    }

    dma_stream->CR = DMA_Channel_2 | DMA_DIR_MemoryToPeripheral | DMA_MemoryInc_Enable |
        DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord | DMA_Mode_Normal |
        DMA_Priority_High | DMA_MemoryBurst_Single | DMA_PeripheralBurst_Single;

    dma_stream->FCR = 0x00000020 | DMA_FIFOThreshold_Full;

    dma_stream->M0AR = (uint32_t) buf;
    dma_stream->NDTR = buf_len;
    dma_stream->PAR = ccr_address;

    enable_dma_stream(dma_stream);
}

static void reset_dma_stream_transfer_complete(DMA_Stream_TypeDef *dma_stream) {
    if (dma_stream == DMA1_Stream0) {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
    } else if (dma_stream == DMA1_Stream3) {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
    }
}

static void disable_tim4_dma(uint16_t dma_source) {
    TIM4->DIER &= ~dma_source;
}

static void enable_tim4_dma(uint16_t dma_source) {
    TIM4->DIER |= dma_source;
}

static void init_hw(LedPin pin, uint16_t *buffer, uint32_t length) {
    PinHwConfig cfg = get_pin_hw_config(pin);

    VESC_IF->set_pad_mode(
        GPIOB, cfg.pin_nr, PAL_MODE_ALTERNATE(2) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1
    );

    reset_tim4();
    init_dma_stream(cfg.dma_stream, buffer, length, cfg.ccr_address);
    init_tim4(cfg.ccr_address == (uint32_t) &TIM4->CCR1);
    enable_tim4_dma(cfg.dma_source);
}

static void deinit_hw(LedPin pin) {
    reset_tim4();
    disable_dma_stream(get_pin_hw_config(pin).dma_stream);
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

    // An extra array item to set the output to 0 PWM
    ++driver->bitbuffer_length;
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

    for (uint32_t i = 0; i < driver->bitbuffer_length - 1; ++i) {
        driver->bitbuffer[i] = WS2812_ZERO;
    }
    driver->bitbuffer[driver->bitbuffer_length - 1] = 0;

    init_hw(pin, driver->bitbuffer, driver->bitbuffer_length);
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

    PinHwConfig cfg = get_pin_hw_config(driver->pin);
    disable_tim4_dma(cfg.dma_source);
    disable_dma_stream(cfg.dma_stream);
    reset_dma_stream_transfer_complete(cfg.dma_stream);
    enable_dma_stream(cfg.dma_stream);
    enable_tim4_dma(cfg.dma_source);
}

void led_driver_destroy(LedDriver *driver) {
    if (driver->bitbuffer) {
        // only touch the timer/DMA if we inited it - something else could be using it
        deinit_hw(driver->pin);

        VESC_IF->free(driver->bitbuffer);
        driver->bitbuffer = NULL;
    }
    driver->bitbuffer_length = 0;
}
