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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_16x2.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t do_C[8] = {0x0,0x06,0x06,0x00,0x00,0x00,0x00,0x00};

uint16_t data[1];
uint32_t nhietdo;

//uint8_t* buff_temp[20];
char temp_char[6];

uint32_t period = 1000;
uint16_t temp_limit = 20;
uint8_t period_char[5], temp_limit_char[2] = "20";

uint8_t buffer[8];
uint8_t rx_buff[8] = "M1";

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t count = 0;
uint32_t tick;
uint8_t oled_on = 1, lcd_on = 0;

int __io_putchar (int ch)
{
   HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
   return ch;
}

void sensors_read(void)
{
	/********Nhiet do**********/
	nhietdo = (3.3*100*data[0])/4095.0 + 5;
	sprintf(temp_char,"%d", nhietdo);
}

void Lcd_display(void)
{
	// hien thị nhiet do len lcd
	  if (lcd_on)
	  {
		  Lcd_clear_display();
		  sprintf(temp_char,"%d", nhietdo);
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
			 Lcd_write_string("Over temp limit!!!");
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
		sprintf(temp_char,"%d", nhietdo);
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
	printf("\nTemperature: %d \xB0\C", nhietdo);
	printf("\nTemperature limit: %d \xB0\C\n",temp_limit);
	if(nhietdo > temp_limit)
	{
		printf("->Over temperature limit!!!\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) strcpy(rx_buff, buffer);

	// periods: M1Pxxxxx
	if(rx_buff[2] == 80)
	{
		for(int i=0;i<5;i++)
		{
			period_char[i] = rx_buff[i+3];
		}
		printf("\nSet period: %d ms", atoi(period_char));
		period = atoi(period_char);
		oled_on = 1;
	}

	//Temp limit: M1LT00xx
	if ((rx_buff[2] == 'L') && (rx_buff[3] == 84))
	{
		for(int i=0;i<2;i++)
		{
			temp_limit_char[i] = rx_buff[i+6];
		}
		temp_limit = atoi(temp_limit_char);
		printf("\nUpdate temperature limit: %d", temp_limit);
		oled_on = 1;
	}

	// hien thi mode 2
	if(rx_buff[1] == 50)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"\nMode 2\n", sizeof("\nMode 2\n"), 100);
		//M2LCD---
			if ((rx_buff[3] == 'C')&& (rx_buff[4] == 'D'))
			{
				// hien thi nhiet do len LCD
				HAL_UART_Transmit(&huart1, (uint8_t*)"Display LCD\n", sizeof("Display LCD\n"), 100);
				lcd_on = 1;
				oled_on = 0;
			}

		//M2OLED--
			if(rx_buff[2] == 'O')
			{
				// hien thi nhiet do len oled
				HAL_UART_Transmit(&huart1, (uint8_t*)"Display Oled\n", sizeof("Display Oled\n"), 100);
				oled_on = 1;
				lcd_on = 0;
			}
		//M2BOTH--
			if(rx_buff[2] == 'B')
			{
				// hien thi len ca hai man hinh
				HAL_UART_Transmit(&huart1, (uint8_t*)"Display All\n", sizeof("Display All\n"), 100);
				oled_on = 1;
				lcd_on = 1;
			}
		//M2NONE---
			if((rx_buff[2] == 'N') &&(rx_buff[3] == 'O'))
			{
				// hien thi len ca hai man hinh
				HAL_UART_Transmit(&huart1, (uint8_t*)"OFF All\n", sizeof("OFF All\n"), 100);
				oled_on = 0;
				lcd_on = 0;
			}
	}

	HAL_UART_Receive_IT(&huart1, buffer, 8);
}
//Button thay doi trang thai man hinh LCD & OLED
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	currentMillis = HAL_GetTick();
	if ((GPIO_Pin == GPIO_PIN_15) && ((currentMillis - previousMillis) > 40))
	{
		count++;
		if (count == 1)
		{
			HAL_UART_Transmit(&huart1,(uint8_t*)"\nDisplay OLED & LCD", sizeof("\nDisplay OLED & LCD"), 100);
			oled_on = 1;
			lcd_on = 1;
		}

		if (count == 2)
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)"\nOff OLED & LCD",sizeof("\nOff OLED & LCD") , 100);
			oled_on = 0;
			lcd_on = 0;
			count = 0;
		}

		if (count > 2) count = 0;
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)data, 1);
  	HAL_UART_Receive_IT(&huart1, buffer, 8);
  	Lcd_Init();
    ssd1306_Init();
    Lcd_gotoxy(5, 0);
    Lcd_write_string("START");
    HAL_UART_Transmit(&huart1, (uint8_t*)"START", sizeof("START"), 100);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(25,25);
    ssd1306_WriteString("START", Font_16x26, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000 - 350);
    Lcd_clear_display();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //read sensors
	  sensors_read();

	  //UART
	  tick = HAL_GetTick();
	  printf("\nTick: %d ms",tick);
	  uart_temp_display();

	  //display LCD, OLED
	  Lcd_display();
	  oled_display();

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  //Delay period
	  if ((oled_on == 1) && (lcd_on == 1)) HAL_Delay(period-394);
	  if ((oled_on == 0) && (lcd_on == 1)) HAL_Delay(period-263);
	  if ((oled_on == 1) && (lcd_on == 0)) HAL_Delay(period-259);
	  if ((oled_on == 0) && (lcd_on == 0)) HAL_Delay(period-160);
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
