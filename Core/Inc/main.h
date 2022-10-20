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
#define pin_e_Pin GPIO_PIN_11
#define pin_e_GPIO_Port GPIOB
#define pin_f_Pin GPIO_PIN_12
#define pin_f_GPIO_Port GPIOB
#define pin_g_Pin GPIO_PIN_13
#define pin_g_GPIO_Port GPIOB
#define pin_dp_Pin GPIO_PIN_14
#define pin_dp_GPIO_Port GPIOB
#define pin_a_Pin GPIO_PIN_4
#define pin_a_GPIO_Port GPIOB
#define pin_b_Pin GPIO_PIN_5
#define pin_b_GPIO_Port GPIOB
#define pin_c_Pin GPIO_PIN_7
#define pin_c_GPIO_Port GPIOB
#define pin_d_Pin GPIO_PIN_8
#define pin_d_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
