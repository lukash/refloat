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

#pragma once

#include "conf/datatypes.h"
#include "led_strip.h"

#include "st_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    void *pin_port;
    TIM_TypeDef *timer;
    uint32_t ccr_address;
    uint32_t rcc_apb1_periph;
    volatile uint32_t *timer_ccmr;
    DMA_Stream_TypeDef *dma_stream;
    uint32_t dma_channel;
    uint16_t dma_source;
    uint8_t dma_if_shift;
    uint8_t timer_ccmr_shift;
    uint8_t timer_ccer_shift;
    uint8_t pin_nr;
} PinHwConfig;

typedef struct {
    uint16_t *bitbuffer;
    uint32_t bitbuffer_length;
    LedPin pin;
    const PinHwConfig *pin_hw_config;
    const LedStrip *strips[STRIP_COUNT];
    uint16_t *strip_bitbuffs[STRIP_COUNT];
} LedDriver;

void led_driver_init(LedDriver *driver);

bool led_driver_setup(
    LedDriver *driver, LedPin pin, LedPinConfig pin_config, const LedStrip **led_strips
);

void led_driver_paint(LedDriver *driver);

void led_driver_destroy(LedDriver *driver);
