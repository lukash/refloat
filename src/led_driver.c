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

#include "utils.h"

#include "vesc_c_if.h"

#include <string.h>

#define WS2812_CLK_HZ 800000
#define TIM_PERIOD ((168000000 / 2 / WS2812_CLK_HZ) - 1)
#define WS2812_ZERO (((uint32_t) TIM_PERIOD) * 0.3)
#define WS2812_ONE (((uint32_t) TIM_PERIOD) * 0.7)

static const PinHwConfig pin_hw_configs[] = {
    {
        .pin_port = GPIOB,
        .pin_nr = 6,
        .timer = TIM4,
        .ccr_address = (uint32_t) &TIM4->CCR1,
        .rcc_apb1_periph = RCC_APB1Periph_TIM4,
        .timer_ccmr = &TIM4->CCMR1,
        .timer_ccmr_shift = 0,
        .timer_ccer_shift = 0,
        .dma_stream = DMA1_Stream0,
        .dma_channel = DMA_Channel_2,
        .dma_if_shift = 0,
        .dma_source = TIM_DMA_CC1,
    },
    {
        .pin_port = GPIOB,
        .pin_nr = 7,
        .timer = TIM4,
        .ccr_address = (uint32_t) &TIM4->CCR2,
        .rcc_apb1_periph = RCC_APB1Periph_TIM4,
        .timer_ccmr = &TIM4->CCMR1,
        .timer_ccmr_shift = 8,
        .timer_ccer_shift = 4,
        .dma_stream = DMA1_Stream3,
        .dma_channel = DMA_Channel_2,
        .dma_if_shift = 22,
        .dma_source = TIM_DMA_CC2,
    },
    {
        .pin_port = GPIOC,
        .pin_nr = 9,
        .timer = TIM3,
        .ccr_address = (uint32_t) &TIM3->CCR4,
        .rcc_apb1_periph = RCC_APB1Periph_TIM3,
        .timer_ccmr = &TIM3->CCMR2,
        .timer_ccmr_shift = 8,
        .timer_ccer_shift = 12,
        .dma_stream = DMA1_Stream2,
        .dma_channel = DMA_Channel_5,
        .dma_if_shift = 16,
        .dma_source = TIM_DMA_CC4,
    }
};

static void reset_timer(const PinHwConfig *pin_hw_cfg) {
    RCC->APB1RSTR |= pin_hw_cfg->rcc_apb1_periph;
    RCC->APB1RSTR &= ~pin_hw_cfg->rcc_apb1_periph;
}

static void init_timer(const PinHwConfig *pin_hw_cfg) {
    // enable the Low Speed APB (APB1) peripheral clock
    // see: RCC_APB1PeriphClockCmd()
    RCC->APB1ENR |= pin_hw_cfg->rcc_apb1_periph;

    // init timer registers
    // see: TIM_TimeBaseInit()
    pin_hw_cfg->timer->CR1 |= TIM_CounterMode_Up;
    pin_hw_cfg->timer->ARR = TIM_PERIOD;

    // see: TIM_OC<N>Init() and TIM_OC<N>PreloadConfig()
    *pin_hw_cfg->timer_ccmr |= TIM_OCMode_PWM1 << pin_hw_cfg->timer_ccmr_shift;
    pin_hw_cfg->timer->CCER |= (TIM_OCPolarity_High | TIM_OutputState_Enable)
        << pin_hw_cfg->timer_ccer_shift;
    *pin_hw_cfg->timer_ccmr |= TIM_OCPreload_Enable << pin_hw_cfg->timer_ccmr_shift;

    // enable timer peripheral Preload register on ARR and enable the timer
    // see: RCC_APB1PeriphClockCmd()
    pin_hw_cfg->timer->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void disable_dma_stream(const PinHwConfig *pin_hw_cfg) {
    pin_hw_cfg->dma_stream->CR &= ~DMA_SxCR_EN;
}

static void enable_dma_stream(const PinHwConfig *pin_hw_cfg) {
    pin_hw_cfg->dma_stream->CR |= DMA_SxCR_EN;
}

static void init_dma_stream(const PinHwConfig *pin_hw_cfg, uint16_t *buf, uint32_t buf_len) {
    // enable the AHB1 peripheral clock for DMA1
    // see: RCC_AHB1PeriphClockCmd()
    RCC->AHB1ENR |= RCC_AHB1Periph_DMA1;

    // see: DMA_Init()
    disable_dma_stream(pin_hw_cfg);

    pin_hw_cfg->dma_stream->M1AR = 0;

    const uint32_t dma_stream0_it_mask =
        DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0;
    DMA1->LIFCR = dma_stream0_it_mask << pin_hw_cfg->dma_if_shift;

    pin_hw_cfg->dma_stream->CR = pin_hw_cfg->dma_channel | DMA_DIR_MemoryToPeripheral |
        DMA_MemoryInc_Enable | DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord |
        DMA_Mode_Normal | DMA_Priority_High | DMA_MemoryBurst_Single | DMA_PeripheralBurst_Single;

    pin_hw_cfg->dma_stream->FCR = 0x00000020 | DMA_FIFOThreshold_Full;

    pin_hw_cfg->dma_stream->M0AR = (uint32_t) buf;
    pin_hw_cfg->dma_stream->NDTR = buf_len;
    pin_hw_cfg->dma_stream->PAR = pin_hw_cfg->ccr_address;

    enable_dma_stream(pin_hw_cfg);
}

static void reset_dma_stream_transfer_complete(const PinHwConfig *pin_hw_cfg) {
    DMA1->LIFCR |= DMA_LIFCR_CTCIF0 << pin_hw_cfg->dma_if_shift;
}

static void disable_timer_dma(const PinHwConfig *pin_hw_cfg) {
    pin_hw_cfg->timer->DIER &= ~pin_hw_cfg->dma_source;
}

static void enable_timer_dma(const PinHwConfig *pin_hw_cfg) {
    pin_hw_cfg->timer->DIER |= pin_hw_cfg->dma_source;
}

static void init_hw(
    const PinHwConfig *cfg, LedPinConfig pin_config, uint16_t *buffer, uint32_t length
) {
    uint32_t pin_mode = PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_MID1;
    pin_mode |= pin_config == LED_PIN_CFG_PULLUP_TO_5V ? PAL_STM32_OTYPE_OPENDRAIN
                                                       : PAL_STM32_OTYPE_PUSHPULL;
    VESC_IF->set_pad_mode(cfg->pin_port, cfg->pin_nr, pin_mode);

    reset_timer(cfg);
    init_dma_stream(cfg, buffer, length);
    init_timer(cfg);
    enable_timer_dma(cfg);
}

static void deinit_hw(const PinHwConfig *cfg) {
    reset_timer(cfg);
    disable_dma_stream(cfg);
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

bool led_driver_setup(
    LedDriver *driver, LedPin pin, LedPinConfig pin_config, const LedStrip **led_strips
) {
    if (pin > LED_PIN_LAST) {
        log_error("Invalid LED pin configured: %u", pin);
        return false;
    }
    driver->pin_hw_config = &pin_hw_configs[pin];

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

    init_hw(driver->pin_hw_config, pin_config, driver->bitbuffer, driver->bitbuffer_length);
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

    const PinHwConfig *cfg = driver->pin_hw_config;
    disable_timer_dma(cfg);
    disable_dma_stream(cfg);
    reset_dma_stream_transfer_complete(cfg);
    enable_dma_stream(cfg);
    enable_timer_dma(cfg);
}

void led_driver_destroy(LedDriver *driver) {
    if (driver->bitbuffer) {
        // only touch the timer/DMA if we inited it - something else could be using it
        deinit_hw(driver->pin_hw_config);

        VESC_IF->free(driver->bitbuffer);
        driver->bitbuffer = NULL;
    }
    driver->bitbuffer_length = 0;
}
