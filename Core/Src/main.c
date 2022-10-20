/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button_debounce.h"
#include "leds.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart2_SSDisplay(void);

//VARIABLES
//uint16_t timer_var;
const int max_speed_limit = 10000 - 1;
const int lower_speed_limit = 200;
uint16_t speed = max_speed_limit;

uint8_t main_cmd[] = "Enter command:\r\n";
uint8_t type_cmd[] = "Type display command:\r\n";
uint8_t error_msg[] = "Invalid command!\r\n";
uint8_t newline[] = "\r\n";
uint8_t cmd0[] = "Display set 0";
uint8_t cmd1[] = "Display set 1";
uint8_t cmd2[] = "Display set 2";
uint8_t cmd3[] = "Display set 3";
uint8_t cmd4[] = "Display set 4";
uint8_t cmd5[] = "Display set 5";
uint8_t cmd6[] = "Display set 6";
uint8_t cmd7[] = "Display set 7";
uint8_t cmd8[] = "Display set 8";
uint8_t cmd9[] = "Display set 9";
uint8_t UART2_rxBuffer[50];
uint8_t segment_value;
uint8_t display;

void uart2_SSDisplay(void)
{

	  HAL_UART_Transmit(&huart2, type_cmd, sizeof(type_cmd), 1000);
	  HAL_UART_Receive(&huart2, UART2_rxBuffer, sizeof(UART2_rxBuffer), 10000);

//	  HAL_UART_Transmit(&huart2, newline, sizeof(newline), 500);
	  for (int i = 0; i < 50; i++)
	  {
		  if (UART2_rxBuffer[i] == '\r')
		  {
			  UART2_rxBuffer[i] = '\0';
			  break;
		  }
	  }

	  if (strcmp(UART2_rxBuffer, cmd0) == 0)
	  {
		  segment_value = 0;
	  }
	  else if(strcmp(UART2_rxBuffer, cmd1) == 0)
	  {
		  segment_value = 1;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd2) == 0)
	  {
		  segment_value = 2;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd3) == 0)
	  {
		  segment_value = 3;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd4) == 0)
	  {
		  segment_value = 4;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd5) == 0)
	  {
		  segment_value = 5;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd6) == 0)
	  {
		  segment_value = 6;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd7) == 0)
	  {
		  segment_value = 7;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd8) == 0)
	  {
		  segment_value = 8;
	  }
	  else if (strcmp(UART2_rxBuffer, cmd9) == 0)
	  {
		  segment_value = 9;
	  }
	  else if (strcmp(UART2_rxBuffer, "exit") == 0)
	  {
		  display = 1;
		  segment_value = 36;			//any radnom number except 0-9
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart2, error_msg, sizeof(error_msg), 1000);
	  }
	  seven_segment_display(segment_value);


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
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);

//  HAL_TIM_Base_Start_IT(&htim10);

  //  timer_var = __HAL_TIM_GET_COUNTER(&htim10);
/*
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char store_buffer[50];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_UART_Transmit(&huart2, main_cmd, sizeof(main_cmd), 1000);
	  HAL_UART_Receive(&huart2, UART2_rxBuffer, sizeof(UART2_rxBuffer), 10000);
	  for (int k = 0; k < 50; k++)
	  {
		  store_buffer[k] = UART2_rxBuffer[k];
		  if (UART2_rxBuffer[k] == '\r')
	 	 {
			  UART2_rxBuffer[k] = '\0';
			  store_buffer[k] = '\0';
			  break;
		 }

	  }
	  display = strcmp(UART2_rxBuffer, "Display");
	  if (display == 0)
	  {
		  while(display == 0)
		  {
		  uart2_SSDisplay();
		  }
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart2, error_msg, sizeof(error_msg), 1000);
	  }


//	  button_debounce2();



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1680 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16800 - 1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000 - 1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pin_e_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin
                          |pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : pin_e_Pin pin_f_Pin pin_g_Pin pin_dp_Pin
                           pin_a_Pin pin_b_Pin pin_c_Pin pin_d_Pin */
  GPIO_InitStruct.Pin = pin_e_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin
                          |pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//USART2 interrupt
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	uint8_t result;
	HAL_UART_Transmit(&huart2, UART2_rxBuffer, 4, 50);
	HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 4);

	result = strcmp(UART2_rxBuffer, "led1");
	if (result == 1) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	}else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	}


}
*/
//Button interrupt callback
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_0) {
		read_button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	}
}
*/

//Callback: timer has reset
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim10)
	{
		timer_LEDs();
	}

}
*/
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
