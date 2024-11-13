/*
 * uart.c
 *
 *  Created on: Apr 25, 2024
 *      Author: richywarrior
 */

#include <stdint.h>
#include "main.h"
#include "uart.h"

#define USARTDIV            0X22B		//USARTDIV for 64MHz in 115200
//#define USARTDIV            0x1A0B		//USARTDIV for 64MHz in 115200

static void USER_USART1_Send_8bit( uint8_t Data );

int _write(int  file, char *ptr, int len){
	// Redirect stdout to UART
	//	freopen(NULL, "w", stdout);

	int DataIdx;
	for(DataIdx = 0; DataIdx < len ; DataIdx++){
		while(!( USART1->SR & USART_SR_TXE ));
		USART1->DR = *ptr++;
	}
	return len;
}

void USER_USART1_Init( void ){
	USART1->CR1	|=	 USART_CR1_UE;//	Step 1 Usart enabled
	USART1->CR1	&=	~USART_CR1_M;//		Step 2 8 Data bits
	USART1->CR2	&=	~USART_CR2_STOP;//	Step 3 1 Stop bit
	USART1->BRR	 =	 USARTDIV;//		Step 5 Desired baud rate
	USART1->CR1	|= 	 USART_CR1_TE;//	Step 6 Transmitter enabled
	USART1->CR1	|= 	 USART_CR1_RE;//	Step 7 Receiver enabled
}

void USER_USART1_Transmit( uint8_t *pData, uint16_t size ){
	for( int i = 0; i < size; i++ ){
		USER_USART1_Send_8bit( *pData++ );
	}
}

//void USER_USART1_Receive( uint8_t *pData, uint16_t size ){
//	for( int i = 0; i < size; i++ ){
//		USER_USART1_Read_8bit( *pData++ );
//	}
//}

static void USER_USART1_Send_8bit( uint8_t Data ){
	while(!( USART1->SR & USART_SR_TXE ));//	wait until next data can be written
	USART1->DR = Data;//				Step 7 Data to send
}

uint8_t USER_USART1_Receive( void ){
	if(USART1->SR & USART_SR_RXNE) {
		return (uint8_t) USART1->DR;
	} else {
		return 'n';
	}
}

//uint8_t USER_USART1_Read_8bit(){
//	while( !(USART1->SR & USART_SR_RNXE) );
//	return USART1->DR;
//}
