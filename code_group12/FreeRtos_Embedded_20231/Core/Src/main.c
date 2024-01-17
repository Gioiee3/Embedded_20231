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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "lcd_16x2.h"

#include "ssd1306_conf_template.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T1 1000
#define T2 3000
#define T3 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for wait_LCD */
osMutexId_t wait_LCDHandle;
const osMutexAttr_t wait_LCD_attributes = {
  .name = "wait_LCD"
};
/* Definitions for wait_UART */
osMutexId_t wait_UARTHandle;
const osMutexAttr_t wait_UART_attributes = {
  .name = "wait_UART"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
void StartSensorTask(void *argument);
void StartUartTask(void *argument);
void StartLCDTask(void *argument);

/* USER CODE BEGIN PFP */

void sensor_read(void);
void Lcd_display(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar (int ch)
{
   HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
   return ch;
}

uint8_t do_C[8]={0x0,0x06,0x06,0x00,0x00,0x00,0x00,0x00};
uint16_t data[1];
uint16_t nhietdo;
//float nhietdo;
uint8_t* buff_temp[20];
char temp_char[20];

uint32_t tick = 0;
uint8_t time_tick[20];
uint8_t sp[] = "\n";

uint32_t period = 3000;
uint16_t temp_limit = 20;
uint8_t period_char[5], temp_limit_char[2] = "20";

uint8_t buffer[8];
uint8_t RX_buff[8] = "M1";

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t count = 0;
uint8_t oled_on = 1, lcd_on = 0;

void sensor_read(void)
{
	//hieu chinh cam bien
	nhietdo = (330*data[0])/4095.0 + 6;
	sprintf(temp_char,"%d", nhietdo); // @suppress("Float formatting support")
}

void Lcd_display(void)
{
	// hien thị nhiet do len lcd
	if (lcd_on)
	{
		Lcd_clear_display();
		sprintf(temp_char,"%d", nhietdo); // @suppress("Float formatting support")
		Lcd_gotoxy(0, 0);
		Lcd_write_string("Temp: ");
		Lcd_gotoxy(6, 0);
		Lcd_write_string(temp_char);
		Lcd_create_custom_char(0, do_C);
		Lcd_write_custom_char(9, 0, 0);
		Lcd_gotoxy(10, 0);
		Lcd_write_string("C");

		if(nhietdo > temp_limit)
			{
				Lcd_gotoxy(0,1);
				Lcd_write_string("Over temp limit");
			}
	}
	else
	{
		Lcd_clear_display();
		Lcd_gotoxy(5, 0);
		Lcd_write_string("LCD_OFF");
	}
}

void oled_display(void)
{
	if (oled_on)
	{
		sprintf(temp_char,"%d", nhietdo); // @suppress("Float formatting support")
		ssd1306_Fill(Black);
		ssd1306_SetCursor(4,10);
		ssd1306_WriteString("T: ", Font_11x18, White);
		ssd1306_WriteString(temp_char, Font_11x18, White);
		ssd1306_WriteString(" C", Font_11x18, White);
		ssd1306_UpdateScreen();
		if(nhietdo > temp_limit)
			{
				Lcd_gotoxy(0,1);
				ssd1306_SetCursor(4, 40);
				ssd1306_WriteString("Over Temp Limit !", Font_7x10, White);
				ssd1306_UpdateScreen();
			}
	}
	else
		{
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0,25);
			ssd1306_WriteString("OLED OFF", Font_16x26, White);
			ssd1306_UpdateScreen();
		}
}

void uart_temp_display(void)
{
	printf("\n\rTemperature: %d \xB0\C", nhietdo); // @suppress("Float formatting support")
	printf("\n\rTemperature limit: %d \xB0\C\n",temp_limit);
	if(nhietdo > temp_limit)
	{
		printf("->Over temperature limit!!!\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(huart->Instance == USART1)
	 {
		 strcpy(RX_buff, buffer);
	 }

	 // periods
	 if(RX_buff[2] == 80)
	 {
		 for(int i=0;i<5;i++)
		 {
			 period_char[i] = RX_buff[i+3];
		 }
		 printf("\nSet period: %d ms\n", atoi(period_char));
		 period = atoi(period_char);
		 oled_on = 1;
	 }

	 //Temp limit: M1LT--xx
	 if(RX_buff[3] == 'T')
	 {
		 for(int i=0;i<2;i++)
		 {
			 temp_limit_char[i] = RX_buff[i+6];
		 }

		 temp_limit = atoi(temp_limit_char);
		 printf("\nUpdate temp limit: %d", temp_limit);
		 oled_on = 1;

	 }

	 // hien thi mode 2
	 if(RX_buff[1] == 50)
	 {
		 HAL_UART_Transmit(&huart1, (uint8_t*)"\nMode 2\n", sizeof("\nMode 2\n"), 100);
		 //M2LCD---
		 if(RX_buff[3] == 'C')
		 {
			 // hien thi nhiet do len LCD
			 HAL_UART_Transmit(&huart1, (uint8_t*)"Display LCD\n", sizeof("Display LCD\n"), 100);
			 lcd_on = 1;
			 oled_on = 0;
		 }

		 //M2OLED--
		 if ((RX_buff[2] == 'O')&& (RX_buff[3] == 'L') && (RX_buff[4] == 'E'))
		 {
			 // hien thi nhiet do len oled
			 HAL_UART_Transmit(&huart1, (uint8_t*)"Display Oled\n", sizeof("Display Oled\n"), 100);
			 oled_on = 1;
			 lcd_on = 0;
		 }
		 //M2BOTH--
		 if(RX_buff[2] == 'B')
		 {
			 // hien thi len ca hai man hinh
			 HAL_UART_Transmit(&huart1, (uint8_t*)"Display All\n", sizeof("Display All\n"), 100);
			 oled_on = 1;
			 lcd_on = 1;
		 }
		 //M2NONE---
		 if(RX_buff[2] == 'N')
		 {
			 // hien thi len ca hai man hinh
			 HAL_UART_Transmit(&huart1, (uint8_t*)"OFF All\n", sizeof("OFF All\n"), 100);
			 oled_on = 0;
			 lcd_on = 0;
		 }
	 }

	 HAL_UART_Receive_IT(&huart1, buffer, 8);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	currentMillis = HAL_GetTick();
	if ((GPIO_Pin == GPIO_PIN_15) && ((currentMillis - previousMillis) > 40))
	{
		count++;
		if (count == 1)
		{
			period = T1;
			HAL_UART_Transmit(&huart1,(uint8_t*)"\n\rSet Periods: 1s", sizeof("\n\rSet Periods: 1s"), 100);
		}

		if (count == 2)
		{
			period = T2;
			HAL_UART_Transmit(&huart1, (uint8_t*)"\n\rSet Periods: 3s",sizeof("\n\rSet Periods: 3s") , 100);
		}

		if (count == 3)
		{
			period = T3;
			HAL_UART_Transmit(&huart1, (uint8_t*)"\n\rSet Periods: 5s", sizeof("\n\rSet Periods: 1s"), 100);
			count = 0;
		}

		if (count > 3) count = 0;

		previousMillis = currentMillis;
	}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) data, 1);
  HAL_UART_Receive_IT(&huart1, buffer, 8);
  Lcd_Init();
  ssd1306_Init();
  Lcd_gotoxy(5, 0);
  Lcd_write_string("START");

  ssd1306_Fill(Black);
  ssd1306_SetCursor(25,25);
  ssd1306_WriteString("START", Font_16x26, White);
  ssd1306_UpdateScreen();
	uint16_t sum = 0;
	for(int i = 0; i<300; i++)
	{
		sum+= data[0];
	}
	nhietdo = (3.3*100*sum/300.0)/4095.0;
	sum = 0;

  HAL_Delay(2000-666);
  Lcd_clear_display();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of wait_LCD */
  wait_LCDHandle = osMutexNew(&wait_LCD_attributes);

  /* creation of wait_UART */
  wait_UARTHandle = osMutexNew(&wait_UART_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &LCDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |EN_Pin|RW_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           EN_Pin RW_Pin RS_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |EN_Pin|RW_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(wait_UARTHandle, osWaitForever);
	  osMutexAcquire(wait_LCDHandle, osWaitForever);
	  //printf("Start sensor\n");
	  sensor_read();
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  osMutexRelease(wait_UARTHandle);
	  osMutexRelease(wait_LCDHandle);
	  osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(wait_UARTHandle, osWaitForever);
	  tick = HAL_GetTick();
	  printf("\nTime: %d",tick);
	  uart_temp_display();

	  osMutexRelease(wait_UARTHandle);
	  osDelay(period-37);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
* @brief Function implementing the LCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(wait_LCDHandle, osWaitForever);
	  Lcd_display();
	  oled_display();
	  osMutexRelease(wait_LCDHandle);

	  osDelay(period-550);
  }
  /* USER CODE END StartLCDTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
