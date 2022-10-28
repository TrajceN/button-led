/*
 * button_debounce.c
 *
 *  Created on: 13 Oct 2022
 *      Author: Nenad
 */

//Includes
#include <leds.h>
#include "stm32f4xx_hal.h"
#include "button_debounce.h"
#include "stdbool.h"

//Variables of button_debounce2 function----------BOLJA OPCIJA
bool read_button_state;
bool buttonP = 0;
int buttonP_confidencelvl = 0;
int buttonR_confidencelvl = 0;
int confidence_threshold = 500;	//sto je vise instrukcija potrebno izvrsiti, threshold se mora povecati
bool LED_status = 0;	//opciono
uint8_t newline[] = "\r\n";


extern UART_HandleTypeDef huart2;

//Functions
void button_debounce2(uint8_t *adc_ptr){
	read_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if (read_button_state == 1)
	{
		if (buttonP == 0)
		{

			if (buttonP_confidencelvl > confidence_threshold)
			{
/*				if (LED_status == 0)
				{
					LED_status = 1;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
			}
				else
				{
					LED_status = 0;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
				}

 */		//Add some functions to work with

				HAL_UART_Transmit(&huart2, adc_ptr, sizeof(adc_ptr), 50);
				HAL_UART_Transmit(&huart2, newline, sizeof(newline), 50);
				buttonP = 1;
			}
			else
			{
				buttonP_confidencelvl++;
				buttonR_confidencelvl = 0;
			}

		}
	}
	else
	{
		if (buttonP == 1)
		{

			if (buttonR_confidencelvl > confidence_threshold)
			{
				buttonP = 0;
			}
			else
			{
				buttonR_confidencelvl++;
				buttonP_confidencelvl = 0;
			}

		}

	}

}

//Variables of Button_debounce function
bool read_button;
bool button_state_0;
bool button_state_1;

//Druga opcija debounce
void Button_debounce(void) {

	read_button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if (read_button != button_state_0) {
	//	HAL_Delay(20);
		if (read_button != button_state_1)
		  	{
		  			button_state_1 = read_button;

		  			if (button_state_1 == true)
		  			{
		  				LEDs();
		  			}

		  	}
	  }
	  button_state_0 = read_button;
}
