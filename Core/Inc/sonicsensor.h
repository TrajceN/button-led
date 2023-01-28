/*
 * sonicsensor.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Nenad
 */

#ifndef INC_SONICSENSOR_H_
#define INC_SONICSENSOR_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdio.h"
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

extern uint8_t tx_distance[20];

extern void delay_uS(uint16_t microsec);
void trig_sensor(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* INC_SONICSENSOR_H_ */
