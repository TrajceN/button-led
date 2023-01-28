/*
 * tmp75.c
 *
 *  Created on: 16 Nov 2022
 *      Author: Nenad
 */
#include "tmp75.h"

/*-----------------------TMP75 temperature sensor--------------------------*/


float tmp75_ReturnTemperature(int16_t temperature)
{
	const float res = 0.0625;
	if (temperature > TMP_128) {
		temperature |= 0xF000;				//2's complement is used as temperature can be negative
	}
	return temperature * res;
}
//extern void SysTick_Handler(void);
void tmp75_read_temp(TMP75_ResolutionDef resolution)
{
	HAL_StatusTypeDef addr_check;
	uint8_t tx_error[] = "SCL or SDA pin is disconnected!\r\n";

	HAL_I2C_Mem_Write(&hi2c3, TMP75_ADDR0, CONFIG_REG, 1, &resolution, 1, 50);
	addr_check = HAL_I2C_Master_Transmit(&hi2c3, TMP75_ADDR0, TEMP_REG, 1, 50);
	if (addr_check == HAL_OK) {
		HAL_I2C_Master_Receive(&hi2c3, TMP75_ADDR0, tmp_buf, 2, 50);

		int16_t temp_dec = ((int16_t)tmp_buf[0]<<4) | (tmp_buf[1]>>4);
		float temp_C = tmp75_ReturnTemperature(temp_dec);
		if ((resolution == res_9bits) || (resolution == res_10bits)) {
			sprintf((char*)tmp_buf, "%.2fC", temp_C);
		}else if ((resolution == res_11bits) || (resolution == res_12bits)) {
			sprintf((char*)tmp_buf, "%.3fC", temp_C);
		}
		display_temp = temp_C * 100;
		HAL_UART_Transmit(&huart2, tmp_buf, sizeof(tmp_buf), 10);
	}
	else{
		HAL_UART_Transmit(&huart2, tx_error, sizeof(tx_error), 10);
	}
}
//void tmp75_Registers_Write(void){
//	HAL_I2C_Mem_Write(&hi2c3, TMP75_ADDR0, CONFIG_REG, 1, &resolution, 1, 50);
//	HAL_I2C_Mem_Write(&hi2c3, TMP75_ADDR0, T_LOW_REG, 1, &tlow_val, 2, 50);
//}
//	HAL_I2C_Mem_Write(&hi2c3, addr, T_LOW_REG, 1, &tlow_val, 1, 50);
//	addr_check = HAL_I2C_Master_Transmit(&hi2c3, addr, T_LOW_REG, 1, 50);
//	if (addr_check == HAL_OK) {
//		HAL_I2C_Master_Receive(&hi2c3, addr, buffer, 2, 50);
//		temp_C = tmp75_set_resolution(resolution);
//		sprintf((char*)buffer, "%.2fC\r\n", temp_C);
//		HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 10);
//	}
//	uint8_t debug_buf[11] = {0};
//	sprintf((char*)debug_buf, "0x%X\r\n", (unsigned int)SysTick_Handler);
