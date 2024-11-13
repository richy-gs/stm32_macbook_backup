/*
 * main.c
 *
 *  Created on: Apr 11, 2024
 *      Author: richywarrior
 */

/* Libraries, Definitions and Global Declarations */
#include <stdint.h>
#include "main.h"

void USER_RCC_ClockEnable(void);
void USER_GPIO_Init(void);
void delay(void);

/* Superloop structure */
int main(void) {
    /* Declarations and Initializations */
    USER_RCC_ClockEnable();
    USER_GPIO_Init();

    for (;;) {
        GPIOA->BSRR = (0x1UL << 5U); // Value to reset pin 5 of port A (Turn-ON LD2)
        delay();
        GPIOA->BSRR = (0x1UL << 21U); // Value to set pin 5 of port A (Turn-OFF LD2)
        delay();
    }
}

void USER_RCC_ClockEnable(void) {
    // RCC_APB2ENR modified to IO port A clock enable
    RCC->APB2ENR = RCC->APB2ENR | (0x1UL << 2U); // RCC_APB2ENR actual value | (mask) IOPAEN bit
}

void USER_GPIO_Init(void) {
    // GPIOx_BSRR modified to reset pin 5 of port A (LD2 is connected to PA5)
    GPIOA->BSRR = (0x1UL << 21U); // Immediate value

    // GPIOx_CRL modified to configure pin 5 as output
    GPIOA->CRL = GPIOA->CRL & ~(0x3UL << 22U) & ~(0x2UL << 20U); // GPIOx_CRL actual value & (to clear) (mask) CNF5[1:0] bits & (to clear) (mask) MODE5_1 bit

    // GPIOx_CRL modified to select pin 5 max speed of 10MHz
    GPIOA->CRL = GPIOA->CRL | (0x1UL << 20U); // GPIOx_CRL actual value | (to set) (mask) MODE5_0 bit
}

void delay(void) {
    // Enable Timer 2 clock
    RCC->APB1ENR |= (0x1 << 0U);

    // Clear control register 1 of Timer 2 to reset configuration
    TIM2->CR1 &= ~(0x72);

    // Clear status register of Timer 2 to reset any pending flags
    TIM2->SR &= ~(0x1UL << 0U);

    // Set auto-reload register of Timer 2 to maximum value for longest delay
    TIM2->ARR = 65535;

    // Set counter register of Timer 2 to initial value for desired delay
    TIM2->CNT = 495;

    // Set prescaler register of Timer 2 for desired time base
    TIM2->PSC = 122;

    // Start Timer 2 by setting control register 1
    TIM2->CR1 |= (0x1 << 0U);

    // Wait until Timer 2 update event occurs
    while ((TIM2->SR & (0x1UL << 0U)) == 0);
}

