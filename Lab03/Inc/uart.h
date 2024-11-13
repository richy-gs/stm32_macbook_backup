/*
 * uart.h
 *
 *  Created on: Apr 4, 2024
 *      Author: richywarrior
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>

int _write(int file, char *ptr, int len);

//#define USARTDIV          0x1D4C//				    9600 baud rate with a 72MHz of FPCLK
//#define USARTDIV          0x341//				    9600 baud rate with a 72MHz of FPCLK

#define USARTDIV          0x45


#define USART_CR1_UE    ( 0x1UL << 13U )
#define USART_CR1_M     ( 0x1UL << 12U )
#define USART_CR1_TE    ( 0x1UL <<  3U )
#define USART_CR1_RE    ( 0x1UL <<  2U )
#define USART_CR2_STOP  ( 0x3UL << 12U )
#define USART_SR_TXE    ( 0x1UL <<  7U )
#define USART_SR_RNXE   ( 0x1UL <<  5U )

void USER_USART1_Init( void );
void USER_USART1_Transmit( uint8_t *pData, uint16_t size );
void USER_USART1_Receive( uint8_t *pData, uint16_t size );
uint8_t USER_USART1_Read_8bit();

#endif /* UART_H_ */
