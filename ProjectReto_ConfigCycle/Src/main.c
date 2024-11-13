#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>              /* This ert_main.c example uses printf/fflush */
#include "EngTrModel.h"         /* Model's header file */
#include "rtwtypes.h"
#include "main.h"
#include "uart.h"
#include "lcd.h"
#include "timers.h"
#include "keypad.h"

// Matrix Keyboard
volatile char pressedButton;

// LCD
volatile uint8_t col = 16;

#define USER_B1		( GPIOC->IDR & ( 0x1UL << 13U ))

uint16_t USE_ADC1_MODULE(void);

// Variables Globales
uint16_t readValue = 0;
int ThrottleValue = 0;
char *direccion = "Front";

void Task1_Init( void );
void Task2_Init( void );
void Task3_Init( void );
void Task4_Init( void );

void Task1( void );
void Task2( void );
void Task3( void );
void Task4( void );


float time;
uint16_t start, end, total;

int main(void)
{

//.............. TASK INIT .....................
	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task1_Init();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task2_Init();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task3_Init();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task4_Init();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task1();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task2();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task3();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo

	TIM4->CNT = 0;   // Reiniciar el contador
	start = TIM4->CNT;  // Leer el contador al inicio de la tarea
	Task4();   // Ejecutar la tarea
	end = TIM4->CNT;  // Leer el contador al final de la tarea
	total = end - start;  // Calcular el tiempo total transcurrido
	time = T_HCLK * total * (TIM4->PSC + 1);  // Calcular el tiempo en segundos
	printf("Time is: %f\r\n", time);  // Imprimir el tiempo


//	Task1_Init();
//	Task2_Init();
//	Task3_Init();
//	Task4_Init();

//.............. TASK ......................

//	for(;;)
//	{
//		Task1();
//		Task2();
//		Task3();
//		Task4();
//	}
}

void Task1_Init (void){
	/*System Clock (SYSCLK) Configuration or */
	FLASH->ACR		&=	~( 0X5UL <<  0U); // two wait states latency, if SYSCLK > 48MHz
	FLASH->ACR		|=	 ( 0X2UL <<  0U); // two wait states latency, if SYSCLK > 48MHz
	RCC->CFGR		&=	~( 0x1UL << 16U)  // PLL HSI Clock /2 selected as PLL input clock
					&	~( 0x7UL << 11U)  // APB2 prescaler /1
					&	~( 0x3UL <<  8U)  // APB1 prescaler /2 (APB1 must not exceeed 36Mhz)
					&	~( 0xFUL <<  4U); // AHB perscaler /1
	RCC->CFGR		|=	 ( 0xFUL << 18U)  // PLL input clock x 16 (PLLMUL bits)
					|	 ( 0x4UL <<  8U); // APB1 Prescaler /2
	RCC->CR			|= 	 ( 0x1UL << 24U); // PLL2 ON
	while( !(RCC->CR & ~(0x1UL << 25U )));// Wait until PLL is locked
	RCC->CFGR 		&=  ~( 0x1UL <<  0U); // PLL used as system clock (SW bits)
	RCC->CFGR		|=	 ( 0x2UL <<  0U); // PLL used as system clock (SW bits)
	while( 0x8UL != (RCC->CFGR & 0xCUL)); // wait until PLL is switched

//-------------------------------------------------------------------------------

	// Enable clock for required peripherals
	RCC->APB2ENR    |=  ( 0x1UL <<  2U )    // IO port A clock enable
					|   ( 0x1UL << 14U ); 	// USART 1 clock enable
	RCC->APB2ENR    |=  ( 0x1UL <<  4U );   // IO port C clock enable
	RCC->APB2ENR    |=  ( 0x1UL <<  9U );   // ADC1 clock enable

//-------------------------------------------------------------------------------

	// Configure ADC1 clock peripheral
	RCC->CFGR		|= 	( 0x3  <<  14);	// PCLK2 divided by 8


    // CONFIGURE THE ADC1 PIN3 TO INPUT THE VOLTAGE
    // GPIOx_BSRR modified to reset pin 0 of port A
	GPIOA->BSRR |= (0x1UL << 16U); // Immediate value
	// GPIOx_CRL modified to configure pin 0 as input analog
	GPIOA->CRL  &=  ~(0x3UL << 2U) & ~(0x3UL << 0U);


	// INITIALIZE AND ENABLE THE ADC1 PERIPHERAL
	ADC1->CR1		&=	~(0xF << 16UL);
	// DETERMINE THE RESULT FORMAT
	ADC1->CR2		&= ~(0x1  << 11UL)  // Right Alignment
					|	(0x1  <<  1UL); // Continuos conversion mode

	// DETERMINE THE SAMPLE TIME
	// ¿Cómo se determina el Sample time adecuado?
	ADC1->SMPR2		&= ~(0x7  <<  0UL); // 1.5 Cycles
	// THE SECUENCES AND/OR THE NUMBER OF CONVERSACIONS FOR THE ADC REGULAR CHANNELS
	ADC1->SQR1		&= ~(0xF  <<  20UL);// 1 Conversion for regular channels
	// SELECT THE CHANNEL FOR THE FIRST ADC CONVERSATION
	ADC1->SQR3		&= ~(0x1F  << 0UL);// first conversion in channel 0

	// ENABLE THE ADC MODULE
	ADC1->CR2		|=  (0x1  <<  0UL); // ADC Enable and start conversion
	// ADC Calibration
	ADC1->CR2		|=  (0x1  <<  2UL); // Start the calibration
    // Wait until after calibration is complete
    while((ADC1->CR2 & (0x1UL << 2U)) == 0){}

//-------------------------------------------------------------------------------

	// Reset pin11 of port B
	GPIOB->BSRR = (0x1UL << 27U); // Immediate value

	// Configure pin11 as output
	GPIOB->CRH = GPIOB->CRH         // Actual value
			   & ~(0x3UL << 12U)    // Clear CNF11[1:0] bits
			   & ~(0x2UL << 10U);   // Clear MODE11_1 bit

	// Select pin11 max speed of 10MHz
	GPIOB->CRH = GPIOB->CRH         // Actual value
			   | (0x1UL << 10U);    // Set MODE11_0 bit

	// Reset pin12 of port B
	GPIOB->BSRR = (0x1UL << 28U); // Immediate value

	// Configure pin12 as output
	GPIOB->CRH = GPIOB->CRH         // Actual value
			   & ~(0x3UL << 16U)    // Clear CNF12[1:0] bits
			   & ~(0x2UL << 14U);   // Clear MODE12_1 bit

	// Select pin12 max speed of 10MHz
	GPIOB->CRH = GPIOB->CRH         // Actual value
			   | (0x1UL << 14U);    // Set MODE12_0 bit

//-------------------------------------------------------------------------------

}

void Task2_Init (void){

	KEYPAD_Init();
	EngTrModel_initialize();
}

void Task3_Init (void){
	// Configure USART1_TX pin (PA9) as alternate function output push-pull, max speed 10MHz
    GPIOA->CRH  &=  ~( 0x1UL <<  6U )
                &   ~( 0x2UL <<  4U );
    GPIOA->CRH  |=  ( 0x2UL <<  6U )
                |   ( 0x1UL <<  4U );

	USER_USART1_Init();
}

void Task4_Init (void){
	LCD_Init();
}

void Task1 (void){
	readValue = USE_ADC1_MODULE();
}

void Task2 (void){

	ThrottleValue = ((readValue*100)/4095);

	if (ThrottleValue < 5){
		ThrottleValue = 5;
	}

	EngTrModel_U.Throttle = ThrottleValue;

	pressedButton = KEYPAD_ReadKey();

	if(pressedButton == '2')
	{
		EngTrModel_U.BrakeTorque = 100.0;
		direccion = "Break";
	}
	else if(pressedButton == '1')
	{
		direccion = "Left";
	}
	else if(pressedButton == '3')
	{
		direccion = "Right";
	}
	else
	{
		EngTrModel_U.BrakeTorque = 0.0;
	}

	// Update the values into the vehicle model
	EngTrModel_step();
}

void Task3 (void){
	// Send the output values
	printf("%f,%f,%f,%s\n\r", EngTrModel_Y.VehicleSpeed, EngTrModel_Y.Gear ,EngTrModel_Y.EngineSpeed, direccion);
	tim2_delay(TIM2_TIME_50MS);
}

void Task4 (void){
	LCD_Clear();

//	LCD_Set_Cursor( 1, 1 );
//	LCD_Put_Str( "RPM:" );
//	CD_Put_Num( EngTrModel_B.EngineRPM );
//	tim2_delay(TIM2_TIME_200MS);

	LCD_Set_Cursor( 1, 1 );
	LCD_Put_Str( "VS:" );
	LCD_Put_Num( EngTrModel_Y.VehicleSpeed );
	LCD_Put_Str( "   G:");
	LCD_Put_Num( EngTrModel_Y.Gear );
	LCD_Set_Cursor( 2, 1 );
	LCD_Put_Str( "ES:");
	LCD_Put_Num( EngTrModel_Y.EngineSpeed );
	LCD_Put_Str( " D: ");
	LCD_Put_Str( direccion );
	tim2_delay(TIM2_TIME_200MS);
}

uint16_t USE_ADC1_MODULE(void){
	ADC1->CR2	|=  (0x1  <<  0UL); // ADC Enable and start conversion

	// Wait until conversation is done (SR reg EOC bit)
    while ((ADC1->SR & (0x1UL << 1U)) == 0);

    // Read the value in DR(15:0) of ADC1
    uint16_t readValue = ADC1->DR & 0xFFFF;

    return readValue;
}

