/*
 * spi_7seg.c
 *
 *  Created on: Nov 3, 2022
 *      Author: Nenad
 */

#include "spi_7seg.h"




void displayDigit(uint8_t display, uint8_t digit)
{
	HAL_GPIO_WritePin(GPIOB, DIG0_Pin|DIG1_Pin|DIG2_Pin|DIG3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, &digits[digit], 1);
	HAL_GPIO_WritePin(GPIOB, displaySelect[display], GPIO_PIN_SET);

	if (display == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
}

void displayDigits(uint16_t digit_to_display)
{
	static uint8_t displaySelected = 0;

	switch (displaySelected)
	{
	case 0:
		digit_to_display = digit_to_display%10;
		break;
	case 1:
		digit_to_display = (digit_to_display%100)/10;
		break;
	case 2:
		digit_to_display = (digit_to_display/100)%10;
		break;
	case 3:
		digit_to_display = digit_to_display/1000;
		break;
	default:
		break;
	}

	displayDigit(displaySelected, digit_to_display);
	displaySelected++;
	if (displaySelected > 3)
	{
		displaySelected = 0;
	}
}
