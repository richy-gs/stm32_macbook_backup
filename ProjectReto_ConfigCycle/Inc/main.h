/*
 * main.h
 *
 *  Created on: Apr 18, 2024
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

typedef struct {
	volatile uint32_t ISER[3U];
	volatile uint32_t RESERVED0[29U];
	volatile uint32_t ICER[3U];
	volatile uint32_t RESERVED1[29U];
	volatile uint32_t ISPR[3U];
	volatile uint32_t RESERVED2[29U];
	volatile uint32_t ICPR[3U];
	volatile uint32_t RESERVED3[29U];
	volatile uint32_t IABR[3U];
	volatile uint32_t RESERVED4[61U];
	volatile uint32_t IPR[84U];
	volatile uint32_t RESERVED5[683U];
	volatile uint32_t STIR;
} NVIC_TypeDef;

/* Flash Memory interface registers*/
typedef struct{
	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t AR;
	volatile uint32_t reserved;
	volatile uint32_t OBR;
	volatile uint32_t WRPR;
} FLASH_TypeDef;

/* ADC Registers */
typedef struct {
	volatile uint32_t SR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	volatile uint32_t JOFR1;
	volatile uint32_t JOFR2;
	volatile uint32_t JOFR3;
	volatile uint32_t JOFR4;
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR1;
	volatile uint32_t SQR2;
	volatile uint32_t SQR3;
	volatile uint32_t JSQR;
	volatile uint32_t JDR1;
	volatile uint32_t JDR2;
	volatile uint32_t JDR3;
	volatile uint32_t JDR4;
	volatile uint32_t DR;
} ADC_TypeDef;


#define RCC_BASE    0x40021000UL  // RCC base address
#define GPIOA_BASE  0x40010800UL  // GPIO Port A base address
#define GPIOB_BASE	0x40010C00UL//	GPIO Port B base address
#define GPIOC_BASE	0x40011000UL//	GPIO Port C base address
#define USART_BASE  0x40013800UL  // USART1 base address
#define TIM2_BASE   0x40000000UL	//TIM2 base address
#define TIM4_BASE   0x40000800UL	//TIM4 base address
#define NVIC_BASE	0xE000E100UL // NVIC base address
#define FLASH_BASE  0x40022000UL // FLASH base address
#define ADC1_BASE  	0x40012400UL // ADC1 base address

#define RCC         ((RCC_TypeDef   *) RCC_BASE)
#define GPIOA       ((GPIO_TypeDef  *) GPIOA_BASE)
#define GPIOB		((GPIO_TypeDef *)  GPIOB_BASE)
#define GPIOC		((GPIO_TypeDef *)  GPIOC_BASE)
#define USART1      ((USART_TypeDef *) USART_BASE)
#define TIM2        ((TIM_TypeDef   *) TIM2_BASE)
#define TIM4        ((TIM_TypeDef   *) TIM4_BASE)
#define	NVIC		((NVIC_TypeDef  *) NVIC_BASE)
#define FLASH		((FLASH_TypeDef *) FLASH_BASE)
#define ADC1		((ADC_TypeDef *) ADC1_BASE)

#endif /* MAIN_H_ */
