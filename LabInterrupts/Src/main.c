/*
 * main.c
 *
 *  Created on: Apr 11, 2024
 *      Author: richywarrior
 */

/* Libraries, Definitions and Global Declarations */
#include <stdint.h>
#include "main.h"
#include "uart.h"

// Define for the button
#define BUTTON ( GPIOC->IDR & (0x1UL << 13U))

// Sets the UIF interrupt flag
#define TIM_SR_UIF ( 1 << 0UL )
#define TOGGLE_LD2 ( GPIOA->ODR ^ 1<<5UL)
#define TIM2_INIT_COUNT (230) //

/* Function prototypes */
void USER_RCC_ClockEnable( void );
void USER_GPIO_Init( void );
void delay(void);
void USER_Delay( void );

uint8_t msg[] = "Hello World!\r\n";

void TIM2_IRQHandler(void){
    // Handle Timer 2 interrupt
    if( TIM2->SR  & TIM_SR_UIF ){
        // Toggle LD2 LED
        GPIOA-> ODR = TOGGLE_LD2;
        // Clear UIF flag
        TIM2-> SR &= ~TIM_SR_UIF;
        // Reset Timer 2 count
        TIM2-> CNT = (uint16_t)TIM2_INIT_COUNT;
    }
}

/* Superloop structure */
int main(void)
{

	/* Declarations and Initializations */
    USER_RCC_ClockEnable( );
    USER_GPIO_Init( );
    USER_USART1_Init( );

    // Initialize Timer 2 delay
    delay();
    /* Repetitive block */

    for(;;){
        // Check button state
        if( !BUTTON ){
            // Delay for debouncing
            USER_Delay();       // Dejar pasar 10ms
            // Check button state again
            if( !BUTTON ){
                // Transmit message via USART1
                //USER_USART1_Transmit( msg, sizeof( msg ));
                printf("Vehicle Speed: \r\n");
                // Toggle LD2 LED while button pressed
                // GPIOA-> ODR = TOGGLE_LD2;
                // Wait until button is released
                while( !BUTTON );
                // Delay for debouncing
                USER_Delay();   // Dejar pasar 10ms
            }
        }
    }
}

void USER_RCC_ClockEnable( void ){
    // Enable clock for required peripherals
    RCC->APB2ENR    |=  ( 0x1UL <<  2U )   // IO port A clock enable
                    |   ( 0x1UL << 14U ); // USART 1 clock enable
    RCC->APB2ENR    |=  ( 0x1UL <<  4U );  // IO port C clock enable
}

void USER_GPIO_Init( void ){
    // Configure USART1_TX pin (PA9) as alternate function output push-pull, max speed 10MHz
    GPIOA->CRH  &=  ~( 0x1UL <<  6U )
                &   ~( 0x2UL <<  4U );
    GPIOA->CRH  |=  ( 0x2UL <<  6U )
                |   ( 0x1UL <<  4U );

    // Reset pin5 of port A (LD2 is connected to PA5)
    GPIOA->BSRR =   ( 0x1UL << 21U ); // Immediate value

    // Configure pin5 as output
    GPIOA->CRL  =   GPIOA->CRL          // Actual value
                &   ~( 0x3UL << 22U )   // Clear CNF5[1:0] bits
                &   ~( 0x2UL << 20U );  // Clear MODE5_1 bit

    // Select pin5 max speed of 10MHz
    GPIOA->CRL  =   GPIOA->CRL          // Actual value
                |   ( 0x1UL << 20U );  // Set MODE5_0 bit

    // Configure PC13 pin as input with pull-up
    GPIOC-> CRH &=  ~( 0x3UL << 22U )
                 &  ~( 0x3UL << 20U );
    GPIOC-> CRH |=  ( 0x1UL << 22U );
}

void delay(void) {
    // Enable Timer 2 clock
    RCC->APB1ENR |= (0x1 << 0U);

    // Reset Timer 2 configuration
    TIM2->CR1 &= ~(0x72);
    TIM2->SR &= ~(0x1UL << 0U);

    // Set Timer 2 parameters for delay
    TIM2->ARR = 65535;
    TIM2->CNT = 230;
    TIM2->PSC = 244;

    // Start Timer 2
    TIM2->CR1 |= (0x1 << 0U);

    // Enable Timer 2 update interrupt
    TIM2->DIER |=  1 << 0UL;
    // Enable Timer 2 interrupt in NVIC
    NVIC->ISER[0] |=  1 << 28UL;
}

void USER_Delay( void ){
    // Simple delay function
    __asm("            ldr r0, =20000   ");
    __asm("    again:  sub r0, r0, #1   ");
    __asm("            cmp r0, #0       ");
    __asm("            bne again        ");
}
