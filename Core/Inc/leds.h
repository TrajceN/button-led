/*
 * uart.h
 *
 *  Created on: 11 Oct 2022
 *      Author: Nenad
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;

void seven_segment_display();
void LEDs(void);
void PWM_LEDs(void);
void timer_LEDs(void);



#endif /* INC_LEDS_H_ */
