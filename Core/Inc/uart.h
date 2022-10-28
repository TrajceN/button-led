/*
 * uart.h
 *
 *  Created on: Oct 21, 2022
 *      Author: Nenad
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"
#include "string.h"
#include "leds.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

size_t uart2_rxbuf_pos = 0;

uint8_t UART2_rxBuffer[50];
uint8_t command_received = 0;
uint8_t rx_byte = 0;
uint8_t cmd_state = 0;

void uart2_buzzer(void);
void RX_uart_cmd(void);

#endif /* INC_UART_H_ */
