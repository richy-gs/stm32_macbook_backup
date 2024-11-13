/*
 * main.h
 *
 *  Created on: Apr 11, 2024
 *      Author: richywarrior
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

/* Reset and Clock Control (RCC) registers */
typedef struct {
    volatile uint32_t CR;       // Control Register
    volatile uint32_t CFGR;     // Configuration Register
    volatile uint32_t CIR;      // Clock Interrupt Register
    volatile uint32_t APB2RSTR; // APB2 Peripheral Reset Register
    volatile uint32_t APB1RSTR; // APB1 Peripheral Reset Register
    volatile uint32_t AHBENR;   // AHB Peripheral Clock Enable Register
    volatile uint32_t APB2ENR;  // APB2 Peripheral Clock Enable Register
    volatile uint32_t APB1ENR;  // APB1 Peripheral Clock Enable Register
    volatile uint32_t BDCR;     // Backup Domain Control Register
    volatile uint32_t CSR;      // Control/Status Register
} RCC_TypeDef;

/* General Purpose Input/Output (GPIO) registers */
typedef struct {
    volatile uint32_t CRL;  // Port Configuration Register Low
    volatile uint32_t CRH;  // Port Configuration Register High
    volatile uint32_t IDR;  // Input Data Register
    volatile uint32_t ODR;  // Output Data Register
    volatile uint32_t BSRR; // Bit Set/Reset Register
    volatile uint32_t BRR;  // Bit Reset Register
    volatile uint32_t LCKR; // Lock Register
} GPIO_TypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter (USART) registers */
typedef struct {
    volatile uint32_t SR;   // Status Register
    volatile uint32_t DR;   // Data Register
    volatile uint32_t BRR;  // Baud Rate Register
    volatile uint32_t CR1;  // Control Register 1
    volatile uint32_t CR2;  // Control Register 2
    volatile uint32_t CR3;  // Control Register 3
    volatile uint32_t GTPR; // Guard Time and Prescaler Register
} USART_TypeDef;

/* Timer registers */
typedef struct {
    volatile uint32_t CR1;  // Control Register 1
    volatile uint32_t CR2;  // Control Register 2
    volatile uint32_t SMCR; // Slave Mode Control Register
    volatile uint32_t DIER; // DMA/Interrupt Enable Register
    volatile uint32_t SR;   // Status Register
    volatile uint32_t EGR;  // Event Generation Register
    volatile uint32_t CCMR1;// Capture/Compare Mode Register 1
    volatile uint32_t CCMR2;// Capture/Compare Mode Register 2
    volatile uint32_t CCER; // Capture/Compare Enable Register
    volatile uint32_t CNT;  // Counter
    volatile uint32_t PSC;  // Prescaler
    volatile uint32_t ARR;  // Auto-Reload Register
    volatile uint32_t RCR;  // Repetition Counter
    volatile uint32_t CCR1; // Capture/Compare Register 1
    volatile uint32_t CCR2; // Capture/Compare Register 2
    volatile uint32_t CCR3; // Capture/Compare Register 3
    volatile uint32_t CCR4; // Capture/Compare Register 4
    volatile uint32_t BDTR; // Break and Dead-Time Register
    volatile uint32_t DCR;  // DMA Control Register
    volatile uint32_t DMAR; // DMA Address Register
} TIM_TypeDef;

#define RCC_BASE    0x40021000UL  // RCC base address
#define GPIOA_BASE  0x40010800UL  // GPIO Port A base address
#define USART_BASE  0x40013800UL  // USART1 base address
#define TIM_BASE    0x40000000UL

#define RCC         ((RCC_TypeDef   *) RCC_BASE)
#define GPIOA       ((GPIO_TypeDef  *) GPIOA_BASE)
#define USART1      ((USART_TypeDef *) USART_BASE)
#define TIM2        ((TIM_TypeDef   *) TIM_BASE)

#endif /* MAIN_H_ */
