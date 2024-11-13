// /**
//  ******************************************************************************
//  * @file           : helloLED.c
//  * @author         : rahu7p
//  * @board          : NUCLEO-F103RB
//  ******************************************************************************
//  *
//  * C code to Turn-ON and Turn-OFF the LD2 Led (w/o delay) in bare metal
//  *
//  ******************************************************************************
//  */

// /* ***************************** START **************************************** */
// /* Libraries, Definitions and Global Declarations */
// #include <stdint.h>

// /* Reset and Clock Control registers */
// typedef struct
// {
// 	volatile uint32_t CR;
// 	volatile uint32_t CFGR;
// 	volatile uint32_t CIR;
// 	volatile uint32_t APB2RSTR;
// 	volatile uint32_t APB1RSTR;
// 	volatile uint32_t AHBENR;
// 	volatile uint32_t APB2ENR;
// 	volatile uint32_t APB1ENR;
// 	volatile uint32_t BDCR;
// 	volatile uint32_t CSR;
// } RCC_TypeDef;

// /* General Purpose I/O registers */
// typedef struct
// {
// 	volatile uint32_t CRL;
// 	volatile uint32_t CRH;
// 	volatile uint32_t IDR;
// 	volatile uint32_t ODR;
// 	volatile uint32_t BSRR;
// 	volatile uint32_t BRR;
// 	volatile uint32_t LCKR;
// } GPIO_TypeDef;

// #define RCC_BASE	0x40021000UL//	RCC base address
// #define GPIOA_BASE	0x40010800UL//	GPIO Port A base address
// #define RCC         	((RCC_TypeDef *)RCC_BASE)
// #define GPIOA		((GPIO_TypeDef *)GPIOA_BASE)

// void USER_RCC_ClockEnable( void );
// void USER_GPIO_Init( void );


// #define BUTTON ( GPIOA -> IDR & (0x1UL << 10U))
// void USER_Delay( void );
// void USER_Debounce ( void );

// /* Superloop structure */
// int main(void)
// {
// 	/* Declarations and Initializations */
// 	USER_RCC_ClockEnable( );
// 	USER_GPIO_Init( );

// 	/*for(;;){
// 			GPIOA->BSRR	=	( 0x1UL <<  5U );//	value to reset pin5 of port A (Turn-OFF LD2)
// 			GPIOA->BSRR	=	( 0x1UL << 21U );//	value to set pin5 of port A (Turn-ON LD2)
// 	} */

// /*
// 	int mode = 1;
// 	for(;;){
// 		if(!(GPIOA -> IDR & (0x1UL << 10U))){
// 			if(mode == 1){
// 				GPIOA->BSRR	=	( 0x1UL << 21U );//	value to set pin5 of port A (Turn-ON LD2)
// 				mode = 0;
// 			} else {
// 				GPIOA->BSRR	=	( 0x1UL <<  5U );//	value to reset pin5 of port A (Turn-OFF LD2)
// 				mode = 1;
// 			}
// 		}
// 	}
// */

// 	for(;;){
// 		if( !(GPIOA->IDR & ( 0x1UL << 10U)) ){
// 			GPIOA->ODR ^= ( 0x1UL << 5U);
// 		}
// 	}

// /*
// 	for(;;){
// 		USER_Debounce();
// 	}
// */

// }

// void USER_RCC_ClockEnable( void ){
// 	// RCC_APB2ENR modified to IO port A clock enable
// 	RCC->APB2ENR	=	RCC->APB2ENR//		RCC_APB2ENR actual value
// 				|//			to set
// 				( 0x1UL <<  2U );//	(mask) IOPAEN bit
// }

// void USER_GPIO_Init( void ){
// 	// GPIOx_BSRR modified to reset pin5 of port A (LD2 is connected to PA5)
// 	GPIOA->BSRR	=	( 0x1UL << 21U );//	immediate value

// 	// GPIOx_CRL modified to configure pin5 as output
// 	GPIOA->CRL	=	GPIOA->CRL//		GPIOx_CRL actual value
// 				&//			to clear
// 				~( 0x3UL << 22U )//	(mask) CNF5[1:0] bits
// 				&//			to clear
// 				~( 0x2UL << 20U );//	(mask) MODE5_1 bit

// 	// GPIOx_CRL modified to select pin5 max speed of 10MHz
// 	GPIOA->CRL	=	GPIOA->CRL//		GPIOx_CRL actual value
// 				|//			to set
// 				( 0x1UL << 20U );//	(mask) MODE5_0 bit

// 	//pinPA10 as input floating
// 	GPIOA->CRH		&=		~( 0x2UL << 10U )
// 					&		~( 0x3UL << 8U  );
// 	GPIOA->CRH		|=		 ( 0x1UL << 10U );

// }


// void USER_Delay( void ){
// 	__asm("			ldr	r0, =20000	");
// 	__asm("	again:	sub	r0, r0, #1	");
// 	__asm("			cmp r0, #0		");
// 	__asm("			bne again		");
// }



// void USER_Debounce ( void ){
// 	if( !BUTTON ){
// 		USER_Delay();		// Dejar pasar 10ms
// 		if( !BUTTON ){
// 			GPIOA->ODR ^= ( 0x1UL << 5U); // Cambiar el estado de LED
// 			while( !BUTTON ){ }
// 			USER_Delay();	// Dejar pasar 10ms
// 		}
// 	}
// }
