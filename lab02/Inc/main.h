/*
 * main.h
 *
 *  Created on: Apr 1, 2024
 *      Author: richywarrior
 */

#ifndef MAIN_H_
#define MAIN_H_

/* Reset and Clock Control registers */
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
} RCC_TypeDef;

/* General Purpose I/O registers */
typedef struct
{
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} GPIO_TypeDef;

#define RCC_BASE	0x40021000UL//	RCC base address
#define GPIOA_BASE	0x40010800UL//	GPIO Port A base address
#define GPIOB_BASE	0x40010C00UL//	GPIO Port B base address
#define GPIOC_BASE	0x40011000UL//	GPIO Port C base address
#define RCC         ((RCC_TypeDef *)RCC_BASE)
#define GPIOA		((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB		((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC		((GPIO_TypeDef *)GPIOC_BASE)

#endif /* MAIN_H_ */
