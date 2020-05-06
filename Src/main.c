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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t	data;
} uart2_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	USART2_QUEUE_LENGTH	1
#define	USART2_QUEUE_ITEM_SIZE	sizeof(uart2_data)

#define MB_FRAME_RX_IDLE						0X00
#define MB_FRAME_RX_STARTED						0X01
#define	MB_FRAME_RX_READ_COILS_FRAME 			0X02
#define	MB_FRAME_RX_READ_INREG_FRAME 			0X03
#define	MB_FRAME_RX_COILS_FRAME_FINISHED_OK		0x09
#define	MB_FRAME_RX_INREG_FRAME_FINISHED_OK		0x0A

#define HMI_SLAVE_ADDR_U3				0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
osThreadId myLED_TaskHandle;
osThreadId myButton_TaskHandle;
osThreadId uart2_Rx_handleHandle;
osThreadId uart3_Rx_handleHandle;
osThreadId CO2_u6_Rx_handlHandle;
osMessageQId HMI_ISR_usart3_Q1Handle;
osMessageQId CO2_ISR_usart6_Q1Handle;
osSemaphoreId usart2_semHandle;
osSemaphoreId usart3_semHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartLED_Task(void const * argument);
void StartButton_Task(void const * argument);
void Start_uart2_Rx_handle(void const * argument);
void Start_uart3_Rx_handle(void const * argument);
void Start_CO2_Rx_handle(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of usart2_sem */
  osSemaphoreDef(usart2_sem);
  usart2_semHandle = osSemaphoreCreate(osSemaphore(usart2_sem), 1);
  osSemaphoreDef(usart3_sem);
  usart3_semHandle = osSemaphoreCreate(osSemaphore(usart3_sem), 1);

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

  /* definition and creation of uart2_Rx_handle */
  osThreadDef(uart2_Rx_handle, Start_uart2_Rx_handle, osPriorityNormal, 0, 128);
  uart2_Rx_handleHandle = osThreadCreate(osThread(uart2_Rx_handle), NULL);

  /* definition and creation of uart3_Rx_handle */
  osThreadDef(uart3_Rx_handle, Start_uart3_Rx_handle, osPriorityNormal, 0, 512);
  uart3_Rx_handleHandle = osThreadCreate(osThread(uart3_Rx_handle), NULL);

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */



  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* Enable Rx Interrupt */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* Enable Rx Interrupt */
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|USART3_RT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART6_RT_GPIO_Port, USART6_RT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin USART3_RT_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|USART3_RT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin USART6_RT_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|USART6_RT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
#ifdef SOFTWARE_DEBUG
	  osDelay(2000);
	  freemem = xPortGetFreeHeapSize();
	  vTaskList (pcWriteBuffer);
#endif
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartLED_Task */
/**
* @brief Function implementing the myLED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void const * argument)
{
  /* USER CODE BEGIN StartLED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED_Task */
}

/* USER CODE BEGIN Header_StartButton_Task */
/**
* @brief Function implementing the myButton_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButton_Task */
void StartButton_Task(void const * argument)
{
  /* USER CODE BEGIN StartButton_Task */
  /* Infinite loop */
  for(;;)
  {

	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	  osDelay(150);
  }
  /* USER CODE END StartButton_Task */
}


void USART2_IRQHandler(void)
{
	uint8_t usart2_Rx_data = 0x00;

	HAL_UART_IRQHandler(&huart2);
	usart2_Rx_data = (uint8_t) huart2.Instance->DR;
}


/* USER CODE BEGIN Header_Start_uart2_Rx_handle */
/**
* @brief Function implementing the uart2_Rx_handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_uart2_Rx_handle */
void Start_uart2_Rx_handle(void const * argument)
{
  /* USER CODE BEGIN Start_uart2_Rx_handle */

  /* Infinite loop */
  for(;;)
  {
	  if  (pdPASS == xSemaphoreTake(usart2_semHandle, portMAX_DELAY))
	  {
		  xSemaphoreTake(usart2_semHandle, portMAX_DELAY);
		  //	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//		  uint8_t uart2_Tx_result = HAL_UART_Transmit(&huart2, uart2_Tx_buf, (sizeof(uart2_Tx_buf)/sizeof(uart2_Tx_buf[0])), 1000);
//		  if (HAL_OK == uart2_Tx_result)
//		  {
//			  //			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
//			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//		  }
	  }
  }
  /* USER CODE END Start_uart2_Rx_handle */
}


void USART3_IRQHandler(void)
{

	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t usart3_Rx_data = 0x00;

	HAL_UART_IRQHandler(&huart3);
	usart3_Rx_data = (uint8_t) huart3.Instance->DR;
	if (pdPASS == xQueueSendToBackFromISR(HMI_ISR_usart3_Q1Handle, &usart3_Rx_data, &xHigherPriorityTaskWoken))
	{
		HAL_GPIO_WritePin(GPIOE, USART3_RT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	}

}

/* USER CODE BEGIN Header_Start_uart3_Rx_handle */
/**
* @brief Function implementing the uart3_Rx_handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_uart3_Rx_handle */
void Start_uart3_Rx_handle(void const * argument)
{
  /* USER CODE BEGIN Start_uart3_Rx_handle */
	static uint8_t	usart3_counter = 0x00;
	static uint8_t	HMI_LAMP_blinking_counter = 0x00;
	static uint8_t	Modbus_Rx_status =  MB_FRAME_RX_IDLE;
//	static uint8_t	N_bytes_to_finish_Rx = 0x00;
	const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

	uint8_t Rx_buffer_from_ISR;
	uint8_t Tx_buf_1 [] = {0x01, 0x01, 0x02, 0x00, 0x04, 0xB8, 0x3F};
	uint8_t Tx_buf_2 [] = {0x01, 0x01, 0x02, 0x00, 0x00, 0xB9, 0xFC};
	uint8_t Tx_buf_3 [] = {'O', 'k'};
	uint8_t Tx_buf_4 [] = {'B', 'a', 'd'};
	uint8_t uart3_Tx_result = 0x00;
  /* Infinite loop */
	for(;;)
	{
		if(pdPASS == xQueueReceive(HMI_ISR_usart3_Q1Handle, &Rx_buffer_from_ISR, portMAX_DELAY )) /* xTicksToWait */
		{
			usart3_counter++;

//			switch (Modbus_Rx_status)
//			{
//
//				case MB_FRAME_RX_IDLE:
//						if ((HMI_SLAVE_ADDR_U3 == Rx_buffer_from_ISR) && (0x01 == usart3_counter))
//						{
//							Modbus_Rx_status = MB_FRAME_RX_STARTED;
//							uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//						}
//						else
//						{
//							usart3_counter = 0x00;
//							uart3_Tx_result = HAL_UART_Transmit(&huart3, Tx_buf_4, (uint16_t)(sizeof(Tx_buf_4)/sizeof(Tx_buf_4[0])), 1000);
//						}
//				break;
//				case MB_FRAME_RX_STARTED:
//						if ((0x01 == Rx_buffer_from_ISR) && (0x02 == usart3_counter))
//						{
//							Modbus_Rx_status = MB_FRAME_RX_READ_COILS_FRAME;
//							uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//						}
//						else if ((0x04 == Rx_buffer_from_ISR) && (0x02 == usart3_counter))
//						{
//							Modbus_Rx_status = MB_FRAME_RX_READ_INREG_FRAME;
//							uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//						}
//						else
//						{
//							usart3_counter = 0x00;
//							Modbus_Rx_status = MB_FRAME_RX_IDLE;
//							uart3_Tx_result = HAL_UART_Transmit(&huart3, Tx_buf_4, (uint16_t)(sizeof(Tx_buf_4)/sizeof(Tx_buf_4[0])), 1000);
//						}
//				break;
//				case MB_FRAME_RX_READ_COILS_FRAME:
//							if(0x08 == usart3_counter)
//							{
//								/* Check CRC*/
//								Modbus_Rx_status = MB_FRAME_RX_COILS_FRAME_FINISHED_OK;
//								uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//							}
//							else
//							{
//								uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//								uart3_Tx_result = HAL_UART_Transmit(&huart3, &Modbus_Rx_status, 1, 1000);
//							}
//
//				break;
//				case MB_FRAME_RX_READ_INREG_FRAME:
//							if(0x08 == usart3_counter)
//							{
//								/* Check CRC*/
//								Modbus_Rx_status = MB_FRAME_RX_INREG_FRAME_FINISHED_OK;
//								uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//							}
//							else
//							{
//								uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//								uart3_Tx_result = HAL_UART_Transmit(&huart3, &Modbus_Rx_status, 1, 1000);
//							}
//				break;
//				default:
//					Modbus_Rx_status = MB_FRAME_RX_IDLE;
//					usart3_counter = 0x00;
//					uart3_Tx_result = HAL_UART_Transmit(&huart3, &usart3_counter, 1, 1000);
//
//			}

//			if (((MB_FRAME_RX_COILS_FRAME_FINISHED_OK == Modbus_Rx_status) || (MB_FRAME_RX_INREG_FRAME_FINISHED_OK == Modbus_Rx_status)) && (0x08 == usart3_counter))
			if (0x08 == usart3_counter)
			{

				usart3_counter = 0x00;
				Modbus_Rx_status = MB_FRAME_RX_IDLE;

				if (0x04 > HMI_LAMP_blinking_counter)
				{
					HMI_LAMP_blinking_counter++;
					uart3_Tx_result = HAL_UART_Transmit(&huart3, Tx_buf_1, (uint16_t)(sizeof(Tx_buf_1)/sizeof(Tx_buf_1[0])), 1000);
					if (HAL_OK != uart3_Tx_result)
					{
						Modbus_Rx_status = MB_FRAME_RX_IDLE;
					}
				}
				else if ((HMI_LAMP_blinking_counter < 0x07) && (HMI_LAMP_blinking_counter >= 0x04))
				{
					HMI_LAMP_blinking_counter++;
					uart3_Tx_result = HAL_UART_Transmit(&huart3, Tx_buf_2, (uint16_t)(sizeof(Tx_buf_2)/sizeof(Tx_buf_2[0])), 1000);
					if (HAL_OK != uart3_Tx_result)
					{
						Modbus_Rx_status = MB_FRAME_RX_IDLE;
					}
				}
				else if ((0x07 == HMI_LAMP_blinking_counter) && (0x07 <= HMI_LAMP_blinking_counter))
				{
					HMI_LAMP_blinking_counter = 0x00;
					uart3_Tx_result = HAL_UART_Transmit(&huart3, Tx_buf_2, (uint16_t)(sizeof(Tx_buf_2)/sizeof(Tx_buf_2[0])), 1000);
					if (HAL_OK != uart3_Tx_result)
					{
						Modbus_Rx_status = MB_FRAME_RX_IDLE;
					}
				}
				else
				{

				}


			}
			else
			{


			}


			HAL_GPIO_WritePin(GPIOE, USART3_RT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

		}
		else
		{
			usart3_counter = 0x00;
			Modbus_Rx_status = MB_FRAME_RX_IDLE;
			HAL_GPIO_WritePin(GPIOE, USART3_RT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}

	}
  /* USER CODE END Start_uart3_Rx_handle */
}


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


/* USER CODE BEGIN Header_Start_CO2_Rx_handle */
/**
* @brief Function implementing the CO2_u6_Rx_handl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CO2_Rx_handle */
void Start_CO2_Rx_handle(void const * argument)
{
  /* USER CODE BEGIN Start_CO2_Rx_handle */
	uint8_t Rx_buffer_from_ISR;
	uint8_t Tx_buf_1 [] = {'O', 'k'};
	uint8_t Tx_buf_2 [] = {'B', 'a', 'd'};
	uint8_t	uart6_Tx_result = 0x00;

  /* Infinite loop */
  for(;;)
  {
	 if(pdPASS == xQueueReceive(CO2_ISR_usart6_Q1Handle, &Rx_buffer_from_ISR, portMAX_DELAY))
	 {
		 if (0x01 == Rx_buffer_from_ISR)
		 {
			 uart6_Tx_result = HAL_UART_Transmit(&huart6, &Tx_buf_1, (uint16_t)(sizeof(Tx_buf_1)/sizeof(Tx_buf_1[0])), 1000);
			 if (HAL_OK == uart6_Tx_result)
			 {
				 HAL_GPIO_WritePin(GPIOC, USART6_RT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
			 }
		 }
		 else
		 {
			 uart6_Tx_result = HAL_UART_Transmit(&huart6, Tx_buf_2, (uint16_t)(sizeof(Tx_buf_2)/sizeof(Tx_buf_2[0])), 1000);
			 if (HAL_OK == uart6_Tx_result)
			 {
				 HAL_GPIO_WritePin(GPIOC, USART6_RT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
			 }
		 }
	 }
	 else
	 {

	 }
  }
  /* USER CODE END Start_CO2_Rx_handle */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
