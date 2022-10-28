/*
 * uart.c
 *
 *  Created on: 11 Oct 2022
 *      Author: Nenad
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "main.h"
#include "leds.h"

//Variables
int var = 0;
int compare = 0;

/* --------------------Functions---------------------------------------- */
#if 0
void seven_segment_display(unsigned int SSD_numbers){

	HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin|pin_e_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 0);
	switch (SSD_numbers)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOB, pin_b_Pin|pin_c_Pin|pin_dp_Pin, 1);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_g_Pin|pin_e_Pin|pin_d_Pin|pin_dp_Pin, 1);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin|pin_g_Pin|pin_dp_Pin, 1);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOB, pin_b_Pin|pin_c_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 1);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_c_Pin|pin_d_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 1);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_c_Pin|pin_d_Pin|pin_e_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 1);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_dp_Pin, 1);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin|pin_e_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 1);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 1);
			break;
		case 0:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin|pin_e_Pin|pin_f_Pin|pin_dp_Pin, 1);
			break;
		default:
			HAL_GPIO_WritePin(GPIOB, pin_a_Pin|pin_b_Pin|pin_c_Pin|pin_d_Pin|pin_e_Pin|pin_f_Pin|pin_g_Pin|pin_dp_Pin, 0);
			break;

	}



}

#endif

void LEDs(void) {
 		 switch (var)
 		 {
 		 	case 0:
 		 		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
 		 		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
 		 		var++;
 		 	break;
			case 1:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
				var++;
			break;
			case 2:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
				var++;
			break;
			default:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
				var=0;
			break;
 		 }
}

//Light dimmer using PWM
void PWM_LEDs(void){
		  for (compare = 0; compare < 1000; compare++) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compare);
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, compare);
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare);
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, compare);
			HAL_Delay(1);
		}
		  for (compare = 1000; compare > 0; compare--) {
		  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compare);
//		  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, compare);
//		  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare);
//		  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, compare);
		  	HAL_Delay(1);
		  	}

}

//Timer function
void timer_LEDs(void){
//	if ((__HAL_TIM_GET_COUNTER(&htim10) - timer_var) >= speed){
		  switch (var) {
		  	  case 0:
		  		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13);
		  		var++;
		  		break;
		  	  case 1:
		  		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14);
		  		var++;
		  		break;
		  	  case 2:
		  		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15);
		  		var++;
		   		break;

		  	  default:
		  		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15|GPIO_PIN_12);
		  		var = 0;
		  		break;
		  }
	}
