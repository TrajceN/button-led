/*
 * tmp75.h
 *
 *  Created on: 16 Nov 2022
 *      Author: Nenad
 */

#ifndef INC_TMP75_H_
#define INC_TMP75_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c3;

/*---Temperature data format for TMP75----*/
//Positive temperature
#define TMP_128 0x7FF
#define TMP_100 0x640
#define TMP_80 	0x500
#define TMP_75	0x4B0
#define TMP_50	0x320
#define TMP_25	0x190
#define TMP_0	0x000
//Negative temperature
#define TMP_N25	0xE70
#define TMP_N55	0xC90

//TMP75 Data registers
#define TEMP_REG 		0x00	//Temperature register - Read only
#define CONFIG_REG 	 	0x01	//Configuration register - R/W
#define T_LOW_REG		0x02	//T_low register - R/W
#define T_HIGH_REG 	 	0x03	//T_high register - R/W

//TMP75 RESOLUTION
typedef enum {
	res_9bits = 0x00,	//0.5 degrees celsius
	res_10bits = 0x20,	//0.25 degrees celsius
	res_11bits = 0x40,	//0.125 degrees celsius
	res_12bits = 0x60	//0.0625 degrees celsius
}TMP75_ResolutionDef;

//TMP75 address
#define TMP75_ADDR0 0x48 << 1

extern uint8_t tmp_buf[12];
extern uint16_t display_temp;

float tmp75_ReturnTemperature(int16_t);
void tmp75_read_temp(TMP75_ResolutionDef);


#endif /* INC_TMP75_H_ */
