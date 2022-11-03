/*
 * spi_7seg.h
 *
 *  Created on: Nov 3, 2022
 *      Author: Nenad
 */

#ifndef INC_SPI_7SEG_H_
#define INC_SPI_7SEG_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

volatile uint16_t tick_cnt;
uint16_t display_cnt;


//SPI1 variables
const uint8_t digits[10] = {			//Common cathode
	0B01000000,		//digit 0	0B10111111
	0B01111001,		//digit 1	0B10000110
	0B00100100,		//digit 2	0B11011011
	0B00110000,		//digit 3	0B11001111
	0B00011001,		//digit 4	0B11100110
	0B00010010,		//digit 5	0B11101101
	0B00000010,		//digit 6	0B11111101
	0B01111000,		//digit 7	0B10000111
	0B00000000,		//digit 8	0B11111111
	0B00010000		//digit 9	0B11101111
};//  dpgfedcba

const uint16_t displaySelect[] = {
	DIG0_Pin,
	DIG1_Pin,
	DIG2_Pin,
	DIG3_Pin
};

void displayDigits(uint16_t digit_to_display);
void displayDigit(uint8_t display, uint8_t digit);

#endif /* INC_SPI_7SEG_H_ */
