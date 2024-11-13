/*
 * timers.h
 *
 *  Created on: May 13, 2024
 *      Author: richywarrior
 */

#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

#define T_HCLK              (1.0f / 64000000.0f)
//#define T_HCLK              (1.0f / 32000000.0f)

#define TIM2_TIME_1S        1.00000f
#define TIM2_TIME_200MS     0.20000f
#define TIM2_TIME_10MS      0.01000f
#define TIM2_TIME_50MS      0.05000f
#define TIM2_TIME_5MS       0.00500f
#define TIM2_TIME_500MS     0.50000f
#define TIM2_TIME_100MS     0.10000f
#define TIM2_TIME_100uS     0.000100f
#define TIM2_TIME_10uS      0.000010f
#define TIM2_TIME_1MS       0.00100f

uint16_t tim2_get_prescaler(float time);
uint16_t tim2_get_init_count(float time, uint16_t prescaler);
void tim2_delay(float time);

//void USER_TIM4_Init_Timer( void );

#endif /* DELAY_H */
