/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"
#include "CO2_helper.h"

/**************************************************************/
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  osMutexDef(Counter_Data_Protect);
  Counter_Data_ProtectHandle = osMutexCreate(osMutex(Counter_Data_Protect));

  /* Create the semaphores(s) */
  osSemaphoreDef(usart2_sem);
  usart2_semHandle = osSemaphoreCreate(osSemaphore(usart2_sem), 1);
  osSemaphoreDef(usart3_sem);
  usart3_semHandle = osSemaphoreCreate(osSemaphore(usart3_sem), 1);
  osSemaphoreDef(usart3_Tx1_sem);
  usart3_Tx1_semHandle = osSemaphoreCreate(osSemaphore(usart3_Tx1_sem), 1);
  osSemaphoreDef(usart3_Tx2_sem);
  usart3_Tx2_semHandle = osSemaphoreCreate(osSemaphore(usart3_Tx2_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of HMI_ISR_usart3_Q1 */
  osMessageQDef(HMI_ISR_usart3_Q1, 1, uint8_t);
  HMI_ISR_usart3_Q1Handle = osMessageCreate(osMessageQ(HMI_ISR_usart3_Q1), NULL);

  /* definition and creation of CO2_ISR_usart6_Q1 */
  osMessageQDef(CO2_ISR_usart6_Q1, 1, uint8_t);
  CO2_ISR_usart6_Q1Handle = osMessageCreate(osMessageQ(CO2_ISR_usart6_Q1), NULL);

  /* definition and creation of Data_for_HMI_Q1 */
  osMessageQDef(Data_for_HMI_Q1, 1, uint16_t);
  Data_for_HMI_Q1Handle = osMessageCreate(osMessageQ(Data_for_HMI_Q1), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
//  usart2_queue = xQueueCreate(USART2_QUEUE_LENGTH, USART2_QUEUE_ITEM_SIZE);
//  if (usart2_queue == NULL)
//  {
//	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
//	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
//	  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
//  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myLED_Task */
  osThreadDef(myLED_Task, StartLED_Task, osPriorityLow, 0, 128);
  myLED_TaskHandle = osThreadCreate(osThread(myLED_Task), NULL);

  /* definition and creation of myButton_Task */
  osThreadDef(myButton_Task, StartButton_Task, osPriorityLow, 0, 128);
  myButton_TaskHandle = osThreadCreate(osThread(myButton_Task), NULL);

  /* definition and creation of counter_1s_Task */
  osThreadDef(counter_1s_Task, Startcounter_1s_Task, osPriorityLow, 0, 128);
  counter_1s_TaskHandle = osThreadCreate(osThread(counter_1s_Task), NULL);

  /* definition and creation of uart2_Rx_handle */
  osThreadDef(uart2_Rx_handle, Start_uart2_Rx_handle, osPriorityNormal, 0, 128);
  uart2_Rx_handleHandle = osThreadCreate(osThread(uart2_Rx_handle), NULL);

  /* definition and creation of uart3_Rx_handle */
  osThreadDef(uart3_Rx_handle, Start_uart3_Rx_handle, osPriorityNormal, 0, 128);
  uart3_Rx_handleHandle = osThreadCreate(osThread(uart3_Rx_handle), NULL);

  /* definition and creation of uart3_Tx1_handle */
  osThreadDef(uart3_Tx1_handle, Start_uart3_Tx1_handle, osPriorityBelowNormal, 0, 128);
  uart3_Tx1_handleHandle = osThreadCreate(osThread(uart3_Tx1_handle), NULL);

  /* definition and creation of uart3_Tx2_handle */
  osThreadDef(uart3_Tx2_handle, Start_uart3_Tx2_handle, osPriorityBelowNormal, 0, 128);
  uart3_Tx2_handleHandle = osThreadCreate(osThread(uart3_Tx2_handle), NULL);

  /* definition and creation of CO2_u6_Rx_handl */
  osThreadDef(CO2_u6_Rx_handl, Start_CO2_Rx_handle, osPriorityNormal, 0, 128);
  CO2_u6_Rx_handlHandle = osThreadCreate(osThread(CO2_u6_Rx_handl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

/**************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t usart2_Rx_data = 0x00;

	HAL_UART_IRQHandler(&huart2);
	usart2_Rx_data = (uint8_t) huart2.Instance->DR;
}

/**************************************************************/
void USART3_IRQHandler(void)
{

	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t usart3_Rx_data = 0x00;

	HAL_UART_IRQHandler(&huart3);
	usart3_Rx_data = (uint8_t) huart3.Instance->DR;
	if (pdPASS == xQueueSendToBackFromISR(HMI_ISR_usart3_Q1Handle, &usart3_Rx_data, &xHigherPriorityTaskWoken))
	{
//		HAL_GPIO_WritePin(GPIOE, USART3_RT_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	}
}

/**************************************************************/
void USART6_IRQHandler(void)
{

	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t usart6_Rx_data = 0x00;

	HAL_UART_IRQHandler(&huart6);
	usart6_Rx_data = (uint8_t) huart6.Instance->DR;
//	usart3_counter++;

	if (pdPASS == xQueueSendToBackFromISR(CO2_ISR_usart6_Q1Handle, &usart6_Rx_data, &xHigherPriorityTaskWoken))
	{
		HAL_GPIO_WritePin(GPIOC, USART6_RT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
	}
}
/**************************************************************/
