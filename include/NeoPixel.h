// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// * NeoPixel (WS2812B) file, PB11 = TIM2_CH4 PWM driven by DMA
// *****************************************************************

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include "stm32f4xx.h"

#define NUM_LEDS 4  // NeoPixels D9, D10, D11, D12

// Sets up PB11 (TIM2_CH4), TIM2 for an 800 kHz PWM period, and DMA1 Stream 7 to feed it
void NeoPixel_Init_PB11(void);

// Writes one LED's color into the RAM buffer (nothing is sent until NeoPixel_Show_PB11)
void NeoPixel_SetColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

// Writes the same color to every LED in the RAM buffer
void NeoPixel_SetAll(uint8_t red, uint8_t green, uint8_t blue);

// Sends the RAM buffer out on PB11 once using TIM2 and DMA
void NeoPixel_Show_PB11(void);

#endif
