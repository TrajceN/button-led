/*
 * uart.c
 *
 *  Created on: Oct 21, 2022
 *      Author: Nenad
 */
#include "uart.h"
#include "main.h"



void uart2_buzzer(void)
{
//	 HAL_UART_Transmit(&huart2, buzzer_cmd, sizeof(buzzer_cmd), 1000);
//	 HAL_UART_Receive(&huart2, UART2_rxBuffer, sizeof(UART2_rxBuffer), 10000);
//
//		  for (int i = 0; i < 50; i++)
//		  {
//			  if (UART2_rxBuffer[i] == '\r')
//			  {
//				  UART2_rxBuffer[i] = '\0';
//				  break;
//			  }
//		  }
		  if (strcmp((char *)UART2_rxBuffer, "ON") == 0)
		  {
			  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		  }
		  else if (strcmp((char *)UART2_rxBuffer, "OFF") == 0)
		  {
			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		  }

}


void RX_uart_cmd(void)
{
	size_t set_cmd = 3;
	HAL_UART_Receive(&huart2, &rx_byte, 1, 5);
		  if (rx_byte != 0)
		  {
			  UART2_rxBuffer[uart2_rxbuf_pos++] = rx_byte;
			  if (rx_byte == '\r' || rx_byte == '\n')
			  {
				  UART2_rxBuffer[uart2_rxbuf_pos-1] = '\0';
				  rx_byte = 0;
				  command_received = 1;
			  }

			  if (uart2_rxbuf_pos > sizeof(UART2_rxBuffer))
			  {
				  uart2_rxbuf_pos = 0;
				  memset(UART2_rxBuffer, 0, sizeof(UART2_rxBuffer));
			  }
		  }

		  if (command_received)
		  {
			  command_received = 0;
			  if (strncmp((char *)UART2_rxBuffer, "Display", uart2_rxbuf_pos) == 0)
			  {
				  cmd_state = 1;
			  }
			  else if (strncmp((char *)UART2_rxBuffer, "Buzzer", uart2_rxbuf_pos) == 0)
			  {
				  cmd_state = 2;
			  }

			  switch (cmd_state)
			  {
			  case 1:
				  if (strncmp((char *)UART2_rxBuffer, "set", set_cmd) == 0)
				  {
					  seven_segment_display(UART2_rxBuffer[set_cmd+1]-'0');
				  }
				  break;
			  case 2:
				  uart2_buzzer();
				  break;
			  default:
				  cmd_state = 0;
				  break;
			  }
			  uart2_rxbuf_pos = 0;
			  memset(UART2_rxBuffer, 0, sizeof(UART2_rxBuffer));
		  }

}
