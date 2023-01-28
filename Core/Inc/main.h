/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define button_IT_Pin GPIO_PIN_0
#define button_IT_GPIO_Port GPIOA
#define button_IT_EXTI_IRQn EXTI0_IRQn
#define US_trig_Pin GPIO_PIN_14
#define US_trig_GPIO_Port GPIOE
#define lcd_RW_Pin GPIO_PIN_11
#define lcd_RW_GPIO_Port GPIOB
#define lcd_RS_Pin GPIO_PIN_12
#define lcd_RS_GPIO_Port GPIOB
#define lcd_E_Pin GPIO_PIN_13
#define lcd_E_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_8
#define D7_GPIO_Port GPIOD
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOD
#define D5_Pin GPIO_PIN_10
#define D5_GPIO_Port GPIOD
#define D4_Pin GPIO_PIN_11
#define D4_GPIO_Port GPIOD
#define Latch_Pin GPIO_PIN_6
#define Latch_GPIO_Port GPIOC
#define DIG0_Pin GPIO_PIN_4
#define DIG0_GPIO_Port GPIOB
#define DIG1_Pin GPIO_PIN_5
#define DIG1_GPIO_Port GPIOB
#define DIG2_Pin GPIO_PIN_7
#define DIG2_GPIO_Port GPIOB
#define DIG3_Pin GPIO_PIN_8
#define DIG3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
