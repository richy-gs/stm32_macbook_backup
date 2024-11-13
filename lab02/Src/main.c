/**
 ******************************************************************************
 * @file           : helloLED.c
 * @author         : rahu7p
 * @board          : NUCLEO-F103RB
 ******************************************************************************
 *
 * C code to Turn-ON and Turn-OFF the LD2 Led (w/o delay) in bare metal
 *
 ******************************************************************************
 */

/* ***************************** START **************************************** */
/* Libraries, Definitions and Global Declarations */
#include <stdint.h>
#include "main.h"

#define BUTTON ( GPIOA -> IDR & (0x1UL << 10U))
void USER_Delay( void );
void USER_Debounce ( void );
void USER_RCC_ClockEnable( void );
void USER_GPIO_Init( void );
void USER_GPIO_e1( void );
void USER_GPIO_e2( void );
void USER_GPIO_e3( void );
uint16_t USER_GPIO_e4( void );
void USER_GPIO_e5( void );

/* Superloop structure */
int main(void)
{
	/* Declarations and Initializations */
	USER_RCC_ClockEnable( );
	USER_GPIO_Init( );

//	for(;;){
//		if( !(GPIOA->IDR & ( 0x1UL << 10U)) ){
//			GPIOA->ODR ^= ( 0x1UL << 5U);
//		}
//	}

	for(;;){
		USER_Debounce();
	}


}

void USER_RCC_ClockEnable( void ){
	// RCC_APB2ENR modified to IO port A clock enable
	RCC->APB2ENR	=	RCC->APB2ENR//		RCC_APB2ENR actual value
				|//			to set
				( 0x1UL <<  2U );//	(mask) IOPAEN bit
}

void USER_GPIO_Init( void ){
	// GPIOx_BSRR modified to reset pin5 of port A (LD2 is connected to PA5)
	GPIOA->BSRR	=	( 0x1UL << 21U );//	immediate value

	// GPIOx_CRL modified to configure pin5 as output
	GPIOA->CRL	=	GPIOA->CRL//		GPIOx_CRL actual value
				&//			to clear
				~( 0x3UL << 22U )//	(mask) CNF5[1:0] bits
				&//			to clear
				~( 0x2UL << 20U );//	(mask) MODE5_1 bit

	// GPIOx_CRL modified to select pin5 max speed of 10MHz
	GPIOA->CRL	=	GPIOA->CRL//		GPIOx_CRL actual value
				|//			to set
				( 0x1UL << 20U );//	(mask) MODE5_0 bit

	//pinPA10 as input floating
//	GPIOA->CRH		&=		~( 0x2UL << 10U )
//					&		~( 0x3UL << 8U  );

	//pinPA10 as input pull-up
	GPIOA->CRH		&=		~( 0x1UL << 10U )	// Se borra el bit 10 (CNF10) del registro CRH de GPIOA para desactivar la salida
					&		~( 0x3UL << 8U  );	// Se borran los bits 9 y 8 (MODE10[1:0]) del registro CRH de GPIOA para desactivar la resistencia de pull-up/pull-down
	GPIOA->CRH		|=		 ( 0x2UL << 10U );	// Se establece el bit 10 (CNF10) del registro CRH de GPIOA para activar la resistencia de pull-up
	GPIOA->ODR		|=		 ( 0x1UL << 10U );	// Se establece el bit 10 (ODR10) del registro ODR de GPIOA para establecer el pin en un nivel alto (activando la resistencia de pull-up)
}


void USER_GPIO_e1(void) {
    // Configure PB0 and PB4 as output
    GPIOB->CRL &= ~((0x3UL << 0U) | (0x3UL << 16U));
    GPIOB->CRL |= ((0x1UL << 0U) | (0x1UL << 16U));

    // Set PB0 and PB4 to high level
    GPIOB->ODR	 = (0x1UL << 0U) | (0x1UL << 4U);
}

void USER_GPIO_e2(void) {
    // Configure PB9 and PB12 as output
    GPIOB->CRH &= ~((0x3UL << 4U) | (0x3UL << 16U));
    GPIOB->CRH |= ((0x1UL << 4U) | (0x1UL << 16U));

    // Clear PB9 and PB12 to low level
    GPIOB->ODR	 = (0x0UL << 9U) | (0x0UL << 12U);
}

void USER_GPIO_e3(void) {
    // Configure odd pins of port A as input with pull-up
    GPIOA->CRL &= ~((0x3UL << 4U) | (0x3UL << 12U) | (0x3UL << 20U) | (0x3UL << 28U));
    GPIOA->CRL &= ~((0x3UL << 6U) | (0x3UL << 14U) | (0x3UL << 22U) | (0x3UL << 30U));

    GPIOA->CRL |= ((0x1UL << 6U) | (0x1UL << 14U) | (0x1UL << 22U) | (0x1UL << 30U));

    GPIOA->CRH &= ~((0x3UL << 4U) | (0x3UL << 12U) | (0x3UL << 20U) | (0x3UL << 28U));
    GPIOA->CRH &= ~((0x3UL << 6U) | (0x3UL << 14U) | (0x3UL << 22U) | (0x3UL << 30U));

    GPIOA->CRH |= ((0x1UL << 6U) | (0x1UL << 14U) | (0x1UL << 22U) | (0x1UL << 30U));
    GPIOA->ODR |= (0xAAAAUL); // Set odd pins of port A to high level
}

uint16_t USER_GPIO_e4(void) {

	GPIOC -> CRH 	&= ~(0x3UL << 8U)
					&  ~(0x2UL << 10U)
					&  ~(0x3UL << 12U)
					&  ~(0x2UL << 14U);
	GPIOC -> CRH	|=  (0x1UL << 10U)
					|	(0X1UL << 14U);
	uint16_t result = (GPIOC->IDR & (0X3UL << 10U) >> 10U);
	return result;
}


void USER_GPIO_e5(void) {
    // Perform a sweep of '1s' from bit 0 to bit 15 in Port A
    for (int i = 0; i < 16; i++) {
        GPIOA->BSRR = (0x1UL << i); // Set bit i in BSRR register
    }
}


void USER_Delay( void ){
	__asm("			ldr	r0, =20000	");
	__asm("	again:	sub	r0, r0, #1	");
	__asm("			cmp r0, #0		");
	__asm("			bne again		");
}

void USER_Debounce ( void ){
	if( !BUTTON ){
		USER_Delay();		// Dejar pasar 10ms
		if( !BUTTON ){
			GPIOA->ODR ^= ( 0x1UL << 5U); // Cambiar el estado de LED
			while( !BUTTON ){ }
			USER_Delay();	// Dejar pasar 10ms
		}
	}
}
