/*
 * button_debouce.h
 *
 *  Created on: 13 Oct 2022
 *      Author: Nenad
 */

#ifndef INC_BUTTON_DEBOUNCE_H_
#define INC_BUTTON_DEBOUNCE_H_

#include "lcd.h"

extern Lcd_HandleTypeDef LCD_ret;
extern uint8_t tx_distance[20];
extern uint8_t lcd_page;

//Declaration of functions
void button_debounce2(void);
void Button_debounce(void);


#endif /* INC_BUTTON_DEBOUNCE_H_ */
