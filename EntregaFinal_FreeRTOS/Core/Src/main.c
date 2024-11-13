/* Includes ------------------------------------------------------------------*/
#include <delay.h>
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "EngTrModel.h"
#include "rtwtypes.h"
#include "lcd.h"
#include "keypad.h"
#include "uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD_T1	30		// Potenciometro
#define PERIOD_T2	60		// Keypad - Modelo Motor
#define PERIOD_T3	150		// UART
#define PERIOD_T4	150		// LCD

//#define PERIOD_T1	40
//#define PERIOD_T2	60
//#define PERIOD_T3	120
//#define PERIOD_T4	160

#define TICK_DIFF_T1	(osKernelSysTick() - (PERIOD_T1 * counter++))
#define TICK_DIFF_T2	(osKernelSysTick() - (PERIOD_T2 * counter++))
#define TICK_DIFF_T3	(osKernelSysTick() - (PERIOD_T3 * counter++))
#define TICK_DIFF_T4	(osKernelSysTick() - (PERIOD_T4 * counter++))

#define GPIO_BSRR_BR_11 (1U << 27)
#define GPIO_BSRR_BS_11 (1U << 11) // 0x0800

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef struct{
	char *direccion;
} DataMotorUART;

typedef struct{
	uint16_t analogicValue;
	uint8_t character;
} DataRasp;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;

osMessageQId Queue1Handle; // Send data from the potentiometer
osMessageQId Queue2Handle; // Send data to UART
//osMessageQId Queue3Handle; // Send data to the LCD

osMutexId mutex1Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void StartTask1(void const* argument);
void StartTask2(void const* argument);
void StartTask3(void const* argument);

uint16_t USE_ADC1_MODULE(void);
void turnOnLED(uint16_t);
void turnOffLED(uint16_t);
void Init_Task1 (void);

uint32_t startTick, endTick;

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  USER_USART1_Init();

  // Redirecciona stdout a UART
  setvbuf(stdout, NULL, _IONBF, 0);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(mutex1);
  mutex1Handle = osMutexCreate(osMutex(mutex1));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
//  osMessageQDef(Queue1, 100, uint16_t);
  osMessageQDef(Queue1, 100, DataRasp);
  Queue1Handle = osMessageCreate(osMessageQ(Queue1), NULL);

  osMessageQDef(Queue2, 100, DataMotorUART);
  Queue2Handle = osMessageCreate(osMessageQ(Queue2), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 384);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* definition and creation of Task2 */

  osThreadDef(Task1, StartTask1, osPriorityHigh, 0, 384);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  osThreadDef(Task2, StartTask2, osPriorityAboveNormal, 0, 384);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  osThreadDef(Task3, StartTask3, osPriorityBelowNormal, 0, 384);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartTask1(void const * argument){
	Init_Task1();
//    uint32_t counter = 0;
    uint16_t readValue = 0;
    uint8_t characterRasp;

    for(;;){
        startTick = osKernelSysTick();

    	readValue = USE_ADC1_MODULE();
    	characterRasp = USER_USART1_Receive();
    	DataRasp Rasp;
    	Rasp.analogicValue = readValue;
    	Rasp.character = characterRasp;

    	/* Send the value to Queue */
//    	osStatus status = osMessagePut(Queue1Handle, (uint16_t) readValue, osWaitForever);
    	osStatus status = osMessagePut(Queue1Handle, (uint32_t) &Rasp, osWaitForever);
    	if (status != osOK) {
    	    if (status == osErrorTimeoutResource) {
    	        printf("Error: Queue is full\r\n");
    	    } else {
    	    	printf("Error in osMessagePut Task 1\r\n");
    	    }
    	}

//        /* Record the end tick */
        endTick = osKernelSysTick();
//        /* Calculate the time difference */
        uint32_t executionTime = endTick - startTick + 1;
//        /* Print or log the execution time */
//        printf("Task 1 Execution Time: %lu ticks\r\n", executionTime);

    	osDelay(PERIOD_T1 - executionTime);
//    	osDelay(29);
    }
}

void StartTask2(void const * argument){

//	uint32_t counter = 0;
	uint16_T readValue = 0;
	int ThrottleValue = 0;
	volatile char pressedButton;
	char *direccion = "Front";
	uint8_t character;
	DataMotorUART M1;

	for(;;){
        startTick = osKernelSysTick();

        /*Recieve value from Queue*/
		osEvent analogicValue = osMessageGet(Queue1Handle, 0);
        if (analogicValue.status != osEventMessage) {
        	printf("Error in osMessageGet Task 2 \r\n");
        }

		readValue = ((DataRasp *)analogicValue.value.p)->analogicValue;
		character = ((DataRasp *)analogicValue.value.p)->character;
		ThrottleValue = ((readValue*100)/4095);

		if (ThrottleValue < 5){
			ThrottleValue = 5;
		}

		EngTrModel_U.Throttle = ThrottleValue;

		pressedButton = KEYPAD_ReadKey();
		direccion = "Front";

		if(pressedButton == '2' || character == 'B')
		{
			EngTrModel_U.BrakeTorque = 100.0;
			direccion = "Break";
			turnOnLED(11);
			turnOnLED(12);
		}
		else if(pressedButton == '1' || character == 'L')
		{
			direccion = "Left";
			turnOnLED(11);
			turnOffLED(12);
		}
		else if(pressedButton == '3' || character == 'R')
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

//		/* Send Data to TASK 3*/
//		M1.eS = EngTrModel_Y.EngineSpeed;
//		M1.gear = EngTrModel_Y.Gear;
//		M1.vS = EngTrModel_Y.VehicleSpeed;
		M1.direccion = direccion;
//
//		M2.eS = EngTrModel_Y.EngineSpeed;
//		M2.gear = EngTrModel_Y.Gear;
//		M2.vS = EngTrModel_Y.VehicleSpeed;

		osStatus status1;
		status1 = osMessagePut(Queue2Handle,(uint32_t) &M1, osWaitForever);
		if (status1 != osOK) {
			if (status1 == osErrorTimeoutResource) {
				printf("Error: Queue is full\r\n");
			} else {
				printf("Error in osMessagePut Task 2\r\n");
			}
		}


//		/* Record the end tick */
		endTick = osKernelSysTick();
//		/* Calculate the time difference */
		uint32_t executionTime = endTick - startTick;
//		/* Print or log the execution time */
//		printf("Task 2 Execution Time: %lu ticks\r\n", executionTime);

//		osDelay(PERIOD_T2 - TICK_DIFF_T2);
		osDelay(PERIOD_T2 - executionTime);
//		osDelay(59);
	}
}

void StartTask3(void const * argument){
//	uint32_t counter = 0;
	char *direccion  = "";

	for(;;){
		startTick = osKernelSysTick();

		/*Receive value from Queue*/
		osEvent valuesMotor = osMessageGet(Queue2Handle, 0);
		if (valuesMotor.status != osEventMessage) {
			printf("Error in osMessageGet Task 3\r\n");
		}

		direccion = ((DataMotorUART *)valuesMotor.value.p)->direccion;

		// Send the output values
		osMutexWait(mutex1Handle, osWaitForever);

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
		tim2_delay(TIM2_TIME_100MS);

		osMutexRelease(mutex1Handle);

//		/* Record the end tick */
		endTick = osKernelSysTick();
//		/* Calculate the time difference */
		uint32_t executionTime = endTick - startTick;
//		/* Print or log the execution time */
//		printf("Task 3 Execution Time: %lu ticks\r\n", executionTime);

//		osDelay(PERIOD_T3 - TICK_DIFF_T3);
		osDelay(PERIOD_T3 - executionTime);
//		osDelay(128);
	}
}

void Init_Task1 (void){
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

    //	Configuración del PIN PB11 como salida push-pull
	GPIOB->CRH &= ~((0xF << 12));  // Limpiar bits de configuración para el PIN 11
	GPIOB->CRH |=  (0x1 << 12);    // Configurar como salida de 10 MHz (MODE11[1:0] = 01, CNF11[1:0] = 00)

	// Configuración del PIN PB12 como salida push-pull
	GPIOB->CRH &= ~((0xF << 16));  // Limpiar bits de configuración para el PIN 12
	GPIOB->CRH |=  (0x1 << 16);    // Configurar como salida de 10 MHz (MODE12[1:0] = 01, CNF12[1:0] = 00)

//-------------------------------------------------------------------------------

	KEYPAD_Init();
	EngTrModel_initialize();
	LCD_Init();
}

uint16_t USE_ADC1_MODULE(void){
	ADC1->CR2	|=  (0x1  <<  0UL); // ADC Enable and start conversion
	// Wait until conversation is done (SR reg EOC bit)
    while ((ADC1->SR & (0x1UL << 1U)) == 0);
    // Read the value in DR(15:0) of ADC1
    uint16_t readValue = ADC1->DR & 0xFFFF;
    return readValue;
}

void turnOnLED(uint16_t pin) {
    GPIOB->ODR |= (1 << pin);  // Establecer el pin correspondiente (alta)
}

void turnOffLED(uint16_t pin) {
    GPIOB->ODR &= ~(1 << pin);  // Restablecer el pin correspondiente (baja)
}

/* Read USART1 To String */
void ReadUSART1ToString(void)
{
    uint8_t rxBuffer[100];
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
        // Null-terminate the received string
        rxBuffer[sizeof(rxBuffer) - 1] = '\0';
        // Print the received string
        printf("Received: %s\n", rxBuffer);
    }
    else
    {
        // Handle error
        printf("Receive error: %d\n", status);
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
