// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// * NeoPixel (WS2812B) file, PB11 = TIM2_CH4 PWM driven by DMA
// *****************************************************************

#include "NeoPixel.h"

#define BITS_PER_LED  24
#define RESET_SLOTS   50   // 50 zero entries x 1.25 us = about 62 us low, longer than the 50 us latch time
#define PWM_0_VAL     36   // about 0.40 us high pulse at the 90 MHz TIM2 clock
#define PWM_1_VAL     72   // about 0.80 us high pulse at the 90 MHz TIM2 clock
#define TIM2_ARR_800K 111  // 90 MHz / 112 ticks = 800 kHz, a 1.25 us period

#define BUFFER_LEN ((NUM_LEDS * BITS_PER_LED) + RESET_SLOTS)

// One 32-bit compare value per bit, because TIM2 CCR4 is a 32-bit register; the last 50 entries stay 0 for the latch
static uint32_t dma_pwm_buffer[BUFFER_LEN];

// Converts RGB to a GRB bit stream (MSB first) and stores one PWM value per bit in the buffer
void NeoPixel_SetColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (index >= NUM_LEDS) return;

    uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
    uint32_t start_idx = index * BITS_PER_LED;

    for (int i = 0; i < 24; i++) {
        if (grb & (1U << (23 - i))) {
            dma_pwm_buffer[start_idx + i] = PWM_1_VAL;
        } else {
            dma_pwm_buffer[start_idx + i] = PWM_0_VAL;
        }
    }
}

// Sets every LED in the buffer to the same color
void NeoPixel_SetAll(uint8_t red, uint8_t green, uint8_t blue) {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        NeoPixel_SetColor(i, red, green, blue);
    }
}

// Enables clocks, sets PB11 to TIM2_CH4 (AF1), sets TIM2 to PWM mode 1 at 800 kHz, and configures DMA1 Stream 7
void NeoPixel_Init_PB11(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // An off pixel is still sent as 24 zero bits, not as a zero compare value
    for (uint32_t i = 0; i < NUM_LEDS * BITS_PER_LED; i++) {
        dma_pwm_buffer[i] = PWM_0_VAL;
    }

    // PB11 as alternate function AF1 (TIM2_CH4), high speed for clean edges
    GPIOB->MODER   &= ~(3U << (11 * 2));
    GPIOB->MODER   |=  (2U << (11 * 2));
    GPIOB->OSPEEDR |=  (3U << (11 * 2));
    GPIOB->AFR[1]  &= ~(0xFU << ((11 - 8) * 4));
    GPIOB->AFR[1]  |=  (0x1U << ((11 - 8) * 4));

    // TIM2 channel 4 in PWM mode 1 with preload, output enabled
    TIM2->PSC  = 0;
    TIM2->ARR  = TIM2_ARR_800K;
    TIM2->CCR4 = 0;
    TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;
    TIM2->CCMR2 |=  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER  |=  TIM_CCER_CC4E;
    TIM2->DIER  &= ~TIM_DIER_CC4DE; // DMA is paced by the update event, enabled in Show

    // DMA1 Stream 7 Channel 3 (TIM2_UP request): memory to peripheral, 32-bit, memory increment, normal mode
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream7->CR & DMA_SxCR_EN) { }

    DMA1_Stream7->PAR  = (uint32_t)&(TIM2->CCR4);
    DMA1_Stream7->M0AR = (uint32_t)dma_pwm_buffer;
    DMA1_Stream7->NDTR = BUFFER_LEN;
    DMA1_Stream7->CR   = (3U << DMA_SxCR_CHSEL_Pos) |
                         DMA_SxCR_DIR_0 |
                         DMA_SxCR_MINC |
                         DMA_SxCR_MSIZE_1 |
                         DMA_SxCR_PSIZE_1;
}

// Stops and restarts the DMA transfer cleanly, then streams the buffer once
void NeoPixel_Show_PB11(void) {
    // Stop the timer and DMA
    TIM2->CR1 &= ~TIM_CR1_CEN;
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream7->CR & DMA_SxCR_EN) { }

    // Force CCR4 to 0 (output low) without raising a DMA request
    TIM2->DIER &= ~TIM_DIER_UDE;
    TIM2->CCR4 = 0;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;

    // Reload the DMA count and clear all stream 7 flags
    DMA1_Stream7->M0AR = (uint32_t)dma_pwm_buffer;
    DMA1_Stream7->NDTR = BUFFER_LEN;
    DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |
                  DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;

    // Each update event (one per bit period) makes DMA write the next compare value
    DMA1_Stream7->CR |= DMA_SxCR_EN;
    TIM2->CNT = 0;
    TIM2->DIER |= TIM_DIER_UDE;
    TIM2->CR1 |= TIM_CR1_CEN;
}
