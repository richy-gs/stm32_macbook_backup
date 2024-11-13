/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD_T1	4000
#define PERIOD_T2	6000
#define PERIOD_T3	12000
#define TICK_DIFF_T1	(osKernelSysTick() - (PERIOD_T1 * counter++))
#define TICK_DIFF_T2	(osKernelSysTick() - (PERIOD_T2 * counter++))
#define TICK_DIFF_T3	(osKernelSysTick() - (PERIOD_T3 * counter++))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;

osThreadId ReceiverHandle;
osThreadId Sender1Handle;
osThreadId Sender2Handle;
osMessageQId Queue1Handle;

osMutexId myMutex01Handle;
osMutexId myMutex02Handle;
osMutexId MutexXHandle;
osMutexId MutexYHandle;
osSemaphoreId BinarySem1Handle;
osSemaphoreId BinarySem2Handle;

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
void StartTask4(void const* argument);

void StartReceiver(void const * argument);
void StartSender1(void const * argument);
void StartSender2(void const * argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(myMutex01);
  myMutex01Handle =  osMutexCreate(osMutex(myMutex01));

  osMutexDef(myMutex02);
  myMutex02Handle =  osMutexCreate(osMutex(myMutex02));

//  osMutexDef(MutexX);
//  MutexXHandle =  osMutexCreate(osMutex(MutexX));
//
//  osMutexDef(MutexY);
//  MutexYHandle =  osMutexCreate(osMutex(MutexY));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(BinarySem1);
  BinarySem1Handle = osSemaphoreCreate(osSemaphore(BinarySem1), 1);

  osSemaphoreDef(BinarySem2);
  BinarySem2Handle = osSemaphoreCreate(osSemaphore(BinarySem2), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
//  osMessageQDef(Queue1, 1, uint32_t);
//  Queue1Handle = osMessageCreate(osMessageQ(Queue1), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

//  printf("Inicializando Scheduler\r\n");

  /* definition and creation of Task2 */
  osThreadDef(Task2, StartTask2, osPriorityNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

//  /* definition and creation of Task2 */
//  osThreadDef(Task4, StartTask4, osPriorityHigh, 0, 128);
//  Task4Handle = osThreadCreate(osThread(Task4), NULL);
//
//  /* definition and creation of Task2 */
//  osThreadDef(Task3, StartTask3, osPriorityAboveNormal, 0, 128);
//  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* definition and creation of Task1 */
  osThreadDef(Task1, StartTask1, osPriorityBelowNormal, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Sender1 */
//  osThreadDef(Sender1, StartSender1, osPriorityNormal, 0, 128);
//  Sender1Handle = osThreadCreate(osThread(Sender1), NULL);

  /* definition and creation of Sender2 */
//  osThreadDef(Sender2, StartSender2, osPriorityNormal, 0, 128);
//  Sender2Handle = osThreadCreate(osThread(Sender2), NULL);

  /* definition and creation of Receiver */
//  osThreadDef(Receiver, StartReceiver, osPriorityAboveNormal, 0, 128);
//  ReceiverHandle = osThreadCreate(osThread(Receiver), NULL);



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

void StartReceiver(void const * argument){
	osEvent retValue;
	printf("Receiver\r\n");
	/* Infinite loop */
	for(;;) {
		retValue = osMessageGet(Queue1Handle, 0);
		if(retValue.status == osEventMessage)
			printf("Message received: %ld\r\n", retValue.value.v);
		else
			printf("Queue is empty\r\n");
	}

////	uint32_t counter = 0;
//	osEvent retValue;
//	for(;;) {
//		/* Get item from Queue with a 0 timeout */
//		retValue = osMessageGet(Queue1Handle, 1000);
//		if(retValue.status == osEventMessage){
//			if(((Data *)retValue.value.p)->source == 1){
//				printf("Receiver receive msg from Sender 1\r\n");
//			} else {
//				printf("Receiver receive msg from Sender 2\r\n");
//			}
//			printf("Msg: 0x%x\r\n",((Data *)retValue.value.p)->value);
//		} else
//			printf("Queue is empty\r\n");
////		osDelay(999 - (osKernelSysTick() - (999 * counter++)));
//	}

}

void StartSender1(void const * argument){
	osStatus status;
	uint32_t message = 7;

	/* Infinite loop */
	for(;;)
	{
		printf("Sending the message\r\n");
		status = osMessagePut(Queue1Handle, message, 0);
		if(status != osOK)
			printf("Queue is Full\r\n");
		osDelay(1000);
	}
//	uint32_t counter = 0;
//	osStatus status;
////	uint32_t message = 7;
//	Data DataToSend1 = {0x2023, 1};
//	for(;;) {
//		/* Put a message into Queue with a 0 timeout */
////		status = osMessagePut(Queue1Handle, message, 0);
//		status = osMessagePut(Queue1Handle, &DataToSend1, 0);
//		if(status != osOK)
//			printf("Full for S1\r\n");
//		osDelay(1000 - (osKernelSysTick() - (1000 * counter++)));
//	}
}

void StartSender2(void const * argument){
//	uint32_t counter = 0;
//	osStatus status;
////	uint32_t message = 4;
//	Data DataToSend2 = {0x2024, 2};
//	for(;;) {
//		/* Put a message into Queue with a 0 timeout */
////		status = osMessagePut(Queue1Handle, message, 0);
//		status = osMessagePut(Queue1Handle, &DataToSend2, 0);
//		if(status != osOK)
//			printf("Full for S2\r\n");
//		osDelay(1000 - (osKernelSysTick() - (1000 * counter++)));
//	}
}

void StartTask1(void const * argument)
{
	/* Infinite loop */
	for(;;)
	{
		printf("Task 1\r\n");
		printf("Taking X by Task 1\r\n");
		osMutexWait(MutexXHandle, osWaitForever);
		printf("Task 1 using X\r\n");
		osDelay(1000);
		printf("Taking Y by Task 1\r\n");
		osMutexWait(MutexYHandle, osWaitForever);
		printf("Task 1 using Y\r\n");
		osDelay(1000);
		printf("Releasing Y by Task 1\r\n");
		osMutexRelease(MutexYHandle);
		printf("Releasing X by Task 1\r\n");
		osMutexRelease(MutexXHandle);
		osDelay(1000);
	}


//	/* Infinite loop */
//	for(;;)
//	{
//		printf("Task 1\r\n");
//		HAL_Delay(1000);
//		printf("Taking Q by Task 1\r\n");
////		osSemaphoreWait(BinarySem1Handle, osWaitForever);
//		osMutexWait(myMutex01Handle, osWaitForever);
//		printf("Task 1 using\r\n");
//		HAL_Delay(1000);
//		printf("Task 1 using\r\n");
//		HAL_Delay(1000);
//		printf("Task 1 using\r\n");
//		HAL_Delay(1000);
//		printf("Task 1 using\r\n");
//		HAL_Delay(1000);
//		printf("Releasing Q by Task 1\r\n");
////		osSemaphoreRelease(BinarySem1Handle);
//		osMutexRelease(myMutex01Handle);
//		printf("Task 1\r\n");
//		HAL_Delay(1000);
//		printf("Task 1 finished in: %ld s\r\n", osKernelSysTick());
//		osThreadTerminate(Task1Handle);
//	}
}

void StartTask2(void const * argument){

	/* Infinite loop */
	for(;;)
	{
//		printf("Task 2\r\n");
		printf("Taking X by Task 2\r\n");
		osMutexWait(MutexXHandle, osWaitForever);
		printf("Task 2 using X\r\n");
		osDelay(1000);
		printf("Taking Y by Task 2\r\n");
		osMutexWait(MutexYHandle, osWaitForever);
		printf("Task 2 using Y\r\n");
		osDelay(1000);
		printf("Releasing Y by Task 2\r\n");
		osMutexRelease(MutexYHandle);
		printf("Releasing X by Task 2\r\n");
		osMutexRelease(MutexXHandle);
		osDelay(1000);
	}

//	/* Infinite loop */
//	osDelay(2000);
//	for(;;)
//	{
//		printf("Task 2\r\n");
//		HAL_Delay(1000);
//		printf("Task 2\r\n");
//		HAL_Delay(1000);
//		printf("Task 2 finished in: %ld s\r\n", osKernelSysTick());
//		osThreadTerminate(Task2Handle);
//	}

}

void StartTask3(void const * argument)
{
	osDelay(2000);
	/* Infinite loop */
	for(;;)
	{
		printf("Task 3\r\n");
		HAL_Delay(1000);
		printf("Taking V by Task 3\r\n");
//		osSemaphoreWait(BinarySem2Handle, osWaitForever);
		osMutexWait(myMutex02Handle, osWaitForever);
		printf("Task 3 using V\r\n");
		HAL_Delay(1000);
		printf("Task 3 using V\r\n");
		HAL_Delay(1000);
		printf("Releasing V by Task 3\r\n");
//		osSemaphoreRelease(BinarySem2Handle);
		osMutexRelease(myMutex02Handle);
		printf("Task 3\r\n");
		HAL_Delay(1000);
		printf("Task 3 finished in: %ld s\r\n", osKernelSysTick());
		osThreadTerminate(Task3Handle);
	}
}

void StartTask4(void const * argument)
{
	osDelay(4000);
	/* Infinite loop */
	for(;;)
	{
		printf("Task 4\r\n");
		HAL_Delay(1000);
		printf("Task 4\r\n");
		HAL_Delay(1000);
		printf("Taking Q by Task 4\r\n");
//		osSemaphoreWait(BinarySem1Handle, osWaitForever);
		osMutexWait(myMutex01Handle, osWaitForever);
		printf("Task 4 using Q\r\n");
		HAL_Delay(1000);
		printf("Releasing Q by Task 4\r\n");
//		osSemaphoreRelease(BinarySem1Handle);
		osMutexRelease(myMutex01Handle);
		printf("Taking V by Task 4\r\n");
//		osSemaphoreWait(BinarySem2Handle, osWaitForever);
		osMutexWait(myMutex02Handle, osWaitForever);
		printf("Task 4 using V\r\n");
		HAL_Delay(1000);
		printf("Releasing V by Task 4\r\n");
//		osSemaphoreRelease(BinarySem2Handle);
		osMutexRelease(myMutex02Handle);
		printf("Task 4\r\n");
		HAL_Delay(1000);
		printf("Task 4 finished in: %ld s\r\n", osKernelSysTick());
		osThreadTerminate(Task4Handle);
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
