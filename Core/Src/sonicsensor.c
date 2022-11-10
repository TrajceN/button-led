/*
 * sonicsensor.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Nenad
 */
#include "sonicsensor.h"

void trig_sensor(void)
{
	HAL_GPIO_WritePin(US_trig_GPIO_Port, US_trig_Pin, GPIO_PIN_SET);
	delay_uS(10);
	HAL_GPIO_WritePin(US_trig_GPIO_Port, US_trig_Pin, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
	HAL_UART_Transmit_IT(&huart2, tx_distance, sizeof(tx_distance));
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) 	//if interrupt happened from channel_3
	{
		if (first_val_capture == 0)
		{
			IC_value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);		//read first value
			first_val_capture = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (first_val_capture == 1)
		{
			IC_value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);		//read second value
			__HAL_TIM_SET_COUNTER(&htim1, 0);
			if (IC_value2 > IC_value1)
			{
				time_difference = IC_value2 - IC_value1;
			}
			else if (IC_value2 < IC_value1)
			{
				time_difference = (65535 - IC_value1) + IC_value2;
			}
			distance = time_difference * (0.034/2);
			sprintf((char*)tx_distance, "Distance: %3dcm\r\n", distance);
			first_val_capture = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
		}
	}
}
