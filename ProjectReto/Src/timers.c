/*
 * timers.c
 *
 *  Created on: May 13, 2024
 *      Author: richywarrior
 */

#include "timers.h"
#include "main.h"
#include <math.h>

uint16_t tim2_get_prescaler(float time) {
    return (uint16_t)(ceil((time / (T_HCLK * ((65535 + 1) - 0))) - 1));
}

uint16_t tim2_get_init_count(float time, uint16_t prescaler) {
    return (uint16_t)((65535 + 1) - (round(time / (T_HCLK * (prescaler + 1)))));
}

void tim2_delay(float time) {
    uint16_t prescaler = tim2_get_prescaler(time);
    uint16_t init_count = tim2_get_init_count(time, prescaler);

    // Enable Timer 2 clock
    RCC->APB1ENR |= (0x1 << 0U);

    RCC->APB2ENR    |=  ( 0x1UL <<  4U );

    // Clear control register 1 of Timer 2 to reset configuration
    TIM2->CR1 &= ~(0x72);

    // Clear status register of Timer 2 to reset any pending flags
    TIM2->SR &= ~(0x1UL << 0U);

    // Set auto-reload register of Timer 2 to maximum value for longest delay
    TIM2->ARR = 65535;

    // Set counter register of Timer 2 to initial value for desired delay
    TIM2->CNT = init_count;

    // Set prescaler register of Timer 2 for desired time base
    TIM2->PSC = prescaler;

    // Start Timer 2 by setting control register 1
    TIM2->CR1 |= (0x1 << 0U);

    // Wait until Timer 2 update event occurs
    while ((TIM2->SR & (0x1UL << 0U)) == 0);

}
