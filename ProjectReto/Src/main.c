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

//#define T_HCLK              (1.0f / 64000000.0f)
#define T_HCLK              (1.0f / 32000000.0f)

#define TIM2_TIME_1S        1.00000f
#define TIM2_TIME_200MS     0.20000f
#define TIM2_TIME_10MS      0.01000f

#define TIM2_PRESC_1S       ((uint16_t)( ceil((TIM2_TIME_1S / (T_HCLK * ((65535 + 1) - 0))) - 1)))
#define TIM2_PRESC_200MS    ((uint16_t)( ceil((TIM2_TIME_200MS / (T_HCLK * ((65535 + 1) - 0))) - 1)))
#define TIM2_PRESC_10MS     ((uint16_t)( ceil((TIM2_TIME_10MS / (T_HCLK * ((65535 + 1) - 0))) - 1)))

#define TIM2_INIT_COUNT_1S  	((uint16_t)( (65535 + 1) - (round(TIM2_TIME_1S / (T_HCLK * (TIM2_PRESC_1S + 1))))))
#define TIM2_INIT_COUNT_200MS	((uint16_t)( (65535 + 1) - (round(TIM2_TIME_200MS / (T_HCLK * (TIM2_PRESC_200MS + 1))))))
#define TIM2_INIT_COUNT_10MS	((uint16_t)( (65535 + 1) - (round(TIM2_TIME_10MS / (T_HCLK * (TIM2_PRESC_10MS + 1))))))

#define GPIO_BSRR_BR_11 (1U << 27)
#define GPIO_BSRR_BS_11 (1U << 11) // 0x0800

// Matrix Keyboard
volatile char pressedButton;

// LCD
volatile uint8_t col = 16;

#define USER_B1		( GPIOC->IDR & ( 0x1UL << 13U ))

void USER_RCC_Init(void);
void USER_GPIO_Init(void);

void USER_TIM2_Delay(uint16_t , uint16_t);

uint16_t USE_ADC1_MODULE(void);

void turnOnLED(uint16_t);
void turnOffLED(uint16_t);

int main(void)
{
	USER_RCC_Init();
	USER_GPIO_Init();
  	USER_USART1_Init();
  	USE_ADC1_MODULE();
	EngTrModel_initialize();

	LCD_Init();

	uint16_t readValue = 0;
	int ThrottleValue = 0;

	for(;;)
	{

// -------------- CAMBIAR EL ESTADO DEL LED MANUALMENTE ----------------

//		GPIOA->BSRR = (0x1UL << 5U); // Value to reset pin 5 of port A (Turn-ON LD2)
//		USER_TIM2_Delay(TIM2_PRESC_1S, TIM2_INIT_COUNT_1S);
//		GPIOA->BSRR = (0x1UL << 21U); // Value to set pin 5 of port A (Turn-OFF LD2)
//		USER_TIM2_Delay(TIM2_PRESC_1S, TIM2_INIT_COUNT_1S);

// -------------- CAMBIAR EL ESTADO DEL LED  ----------------

//		GPIOA->ODR ^= (0x1UL << 5U);
//		USER_TIM2_Delay(TIM2_PRESC_1S, TIM2_INIT_COUNT_1S);



// -------------- PRACTICA TECLADO MATRICIAL  ----------------
//		KEYPAD_Init();
//		char pressedButton = KEYPAD_ReadKey();
//
//		switch (pressedButton){
//			case '1':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '2':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '3':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '4':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '5':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '6':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '7':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '8':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '9':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '*':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '0':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			case '#':
//				printf("n: %c\r\n",pressedButton);
//				break;
//			default:
//				printf("n: %i\r\n",-1);
//				break;
//		}

		// ESPERAR A SCANEAR EL TECLADO MATRICIAL
//		USER_TIM2_Delay(TIM2_PRESC_10MS, TIM2_INIT_COUNT_10MS);//  10ms


// -------------- PRACTICA PANTALLA LCD  ----------------
//		LCD_Clear();
//		LCD_Set_Cursor( 1, 1 );
//		LCD_Put_Str( "TE" );
//		LCD_Put_Num( 2003 );
//		LCD_Put_Char( 'B' );
//		LCD_Put_Str( " SoC" );
//		LCD_Set_Cursor( 2, col2-- );
//		LCD_Put_Str( "Prueba de LCD");
//		LCD_BarGraphic( 0, 64 );
//
//		tim2_delay(TIM2_TIME_200MS);// 200ms
//
//		if( col2 == 0 ){
//			tim2_delay(TIM2_TIME_500MS);//	500ms
//			col2 = 16;
//		}

//		LCD_Clear();
//		LCD_Set_Cursor( 1, 1 );
//		LCD_Put_Str( "Velocidad:" );
//		LCD_Put_Num( 2003 );
//		LCD_BarGraphic( 0, 64 );
//		tim2_delay(TIM2_TIME_200MS);// 200ms

// -------------- PRACTICA DE POTENCIOMETRO -------------------

//		PRINT THE DIGITAL VALUE OF ADC1
//		readValue = USE_ADC1_MODULE();
//		ThrottleValue = ((readValue*100)/4095);
//		printf("P: %i\r\n",ThrottleValue);

// --------------MANDAR LOS DATOS A LA RASPBERRY PI ----------------

		// PRINT THE DIGITAL VALUE OF ADC1
//		readValue = USE_ADC1_MODULE();
//		ThrottleValue = ((readValue*100)/4095);
//		EngTrModel_U.Throttle = ThrottleValue;
//
//		if(!USER_B1){
//			USER_TIM2_Delay(TIM2_PRESC_10MS, TIM2_INIT_COUNT_10MS);//  10ms
//			if(!USER_B1){
//				EngTrModel_U.BrakeTorque = 100.0;
//			}
//		}
//		else{
//			EngTrModel_U.BrakeTorque = 0.0;
//		}
//		EngTrModel_step();
//		printf("Vehicle Speed: %f\r\n", EngTrModel_Y.VehicleSpeed);
//		printf("Engine Speed: %f\r\n", EngTrModel_Y.EngineSpeed);
//		printf("Gear: %f\r\n", EngTrModel_Y.Gear);
//		USER_TIM2_Delay(TIM2_PRESC_200MS, TIM2_INIT_COUNT_200MS);//  200ms

//// -------------- PRACTICA PANTALLLA LCD Y DATOS  ----------------

		char *direccion = "Front";

		KEYPAD_Init();
		pressedButton = KEYPAD_ReadKey();


		readValue = USE_ADC1_MODULE();
		ThrottleValue = ((readValue*100)/4095);

		if (ThrottleValue < 5){
			ThrottleValue = 5;
		}

		EngTrModel_U.Throttle = ThrottleValue;


	    if(pressedButton == '2')
	    {
	    	EngTrModel_U.BrakeTorque = 100.0;
	    	direccion = "Break";
	    	turnOnLED(11);
	    	turnOnLED(12);
	    }
	    else if(pressedButton == '1')
	    {
	    	direccion = "Left";
	    	turnOnLED(11);
	    	turnOffLED(12);
	    }
	    else if(pressedButton == '3')
		{
	    	direccion = "Right";
	    	turnOnLED(12);
	    	turnOffLED(11);
		}
	    else
	    {
	    	turnOffLED(11);
	    	turnOffLED(12);
	    	EngTrModel_U.BrakeTorque = 0.0;
	    }

	    // Update the values into the vehicle model
	    EngTrModel_step();

	    // Send the output values
	    printf("%f,%f,%f,%s\n\r", EngTrModel_Y.VehicleSpeed, EngTrModel_Y.Gear ,EngTrModel_Y.EngineSpeed, direccion);
	    tim2_delay(TIM2_TIME_50MS);

		LCD_Clear();
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
}

void USER_RCC_Init(void){
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

	// Enable clock for required peripherals
	RCC->APB2ENR    |=  ( 0x1UL <<  2U )    // IO port A clock enable
	                |   ( 0x1UL << 14U ); 	// USART 1 clock enable
	  // RCC_APB2ENR modified to IO port B clock enable
	RCC->APB2ENR |= (0x1UL << (3U));	// To set IOPBEN bit

	RCC->APB2ENR    |=  ( 0x1UL <<  4U );   // IO port C clock enable
	RCC->APB2ENR    |=  ( 0x1UL <<  9U );   // ADC1 clock enable


	// Configure ADC1 clock peripheral
	RCC->CFGR		|= 	( 0x3  <<  14);	// PCLK2 divided by 8
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
    while ((ADC1->CR2 & (0x1UL << 2U)) == 0);
}


uint16_t USE_ADC1_MODULE(void){
	ADC1->CR2	|=  (0x1  <<  0UL); // ADC Enable and start conversion

	// Wait until conversation is done (SR reg EOC bit)
    while ((ADC1->SR & (0x1UL << 1U)) == 0);

    // Read the value in DR(15:0) of ADC1
    uint16_t readValue = ADC1->DR & 0xFFFF;

    return readValue;
}



void USER_GPIO_Init(void) {
	// Configure USART1_TX pin (PA9) as alternate function output push-pull, max speed 10MHz
    GPIOA->CRH  &=  ~( 0x1UL <<  6U )
                &   ~( 0x2UL <<  4U );
    GPIOA->CRH  |=  ( 0x2UL <<  6U )
                |   ( 0x1UL <<  4U );


	// GPIOx_BSRR modified to reset pin 5 of port A (LD2 is connected to PA5)
    GPIOA->BSRR &= (0x1UL << 21U); // Immediate value
    // GPIOx_CRL modified to configure pin 5 as output
    GPIOA->CRL = GPIOA->CRL & ~(0x3UL << 22U) & ~(0x2UL << 20U); // GPIOx_CRL actual value & (to clear) (mask) CNF5[1:0] bits & (to clear) (mask) MODE5_1 bit
    // GPIOx_CRL modified to select pin 5 max speed of 10MHz
    GPIOA->CRL = GPIOA->CRL | (0x1UL << 20U); // GPIOx_CRL actual value | (to set) (mask) MODE5_0 bit

    // CONFIGURE THE ADC1 PIN3 TO INPUT THE VOLTAGE
    // GPIOx_BSRR modified to reset pin 0 of port A
	GPIOA->BSRR |= (0x1UL << 16U); // Immediate value
	// GPIOx_CRL modified to configure pin 0 as input analog
	GPIOA->CRL  &=  ~(0x3UL << 2U) & ~(0x3UL << 0U);


//	Configuración del PIN PB11 como salida push-pull
	GPIOB->CRH &= ~((0xF << 12));  // Limpiar bits de configuración para el PIN 11
	GPIOB->CRH |=  (0x1 << 12);    // Configurar como salida de 10 MHz (MODE11[1:0] = 01, CNF11[1:0] = 00)

	// Configuración del PIN PB12 como salida push-pull
	GPIOB->CRH &= ~((0xF << 16));  // Limpiar bits de configuración para el PIN 12
	GPIOB->CRH |=  (0x1 << 16);    // Configurar como salida de 10 MHz (MODE12[1:0] = 01, CNF12[1:0] = 00)
}

void USER_TIM2_Delay(uint16_t presc, uint16_t count){
    // Enable Timer 2 clock
    RCC->APB1ENR 	|=  (0x1 << 0U);
    RCC->APB2ENR    |=  ( 0x1UL <<  4U );

    // Clear control register 1 of Timer 2 to reset configuration
    TIM2->CR1 &= ~(0x72);

    // Clear status register of Timer 2 to reset any pending flags
    TIM2->SR &= ~(0x1UL << 0U);

    // Set auto-reload register of Timer 2 to maximum value for longest delay
    TIM2->ARR = 65535;

    // Set counter register of Timer 2 to initial value for desired delay
    TIM2->CNT = count;

    // Set prescaler register of Timer 2 for desired time base
    TIM2->PSC = presc;

    // Start Timer 2 by setting control register 1
    TIM2->CR1 |= (0x1 << 0U);

    // Wait until Timer 2 update event occurs
    while ((TIM2->SR & (0x1UL << 0U)) == 0);
}

void turnOnLED(uint16_t pin) {
    GPIOB->ODR |= (1 << pin);  // Establecer el pin correspondiente (alta)
}

void turnOffLED(uint16_t pin) {
    GPIOB->ODR &= ~(1 << pin);  // Restablecer el pin correspondiente (baja)
}
