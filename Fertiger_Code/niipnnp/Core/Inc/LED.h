/*
 * LED.h
 *
 *  Created on: 25.06.2020
 *      Author: Felix
 */

#ifndef INC_LED_H_
#define INC_LED_H_
#include "stm32g0xx_hal.h"

#define LED_active 0

#define LEDPORT1 GPIOA
#define LEDPIN1 GPIO_PIN_13
#define LEDPORT2 GPIOB
#define LEDPIN2 GPIO_PIN_9

void initLED();
void LEDRED();
void LEDGREEN();
void LEDYELLOW();
void LEDCLEAN();

#endif /* INC_LED_H_ */
