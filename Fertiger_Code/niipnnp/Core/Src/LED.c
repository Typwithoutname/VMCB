/*
 * LED.c
 *
 *  Created on: 25.06.2020
 *      Author: Felix
 */

#include "LED.h"
void initLED(){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

void LEDYELLOW(){
	GPIO_InitTypeDef GPIO_InitStruct1 = {0};

		HAL_GPIO_WritePin(LEDPORT1, LEDPIN1, GPIO_PIN_RESET);
		GPIO_InitStruct1.Pin = LEDPIN1;
		GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct1.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LEDPORT1, &GPIO_InitStruct1);


		GPIO_InitStruct1.Pin = LEDPIN2;
		GPIO_InitStruct1.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct1.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LEDPORT2, &GPIO_InitStruct1);

		/* Insert delay 100 ms */
		//HAL_Delay(100);
}
void LEDRED(){
	GPIO_InitTypeDef GPIO_InitStruct2 = {0};

		HAL_GPIO_WritePin(LEDPORT2, LEDPIN2, GPIO_PIN_RESET);

		GPIO_InitStruct2.Pin = LEDPIN2;
		GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct2.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LEDPORT2, &GPIO_InitStruct2);

		GPIO_InitStruct2.Pin = LEDPIN1;
		GPIO_InitStruct2.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct2.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LEDPORT1, &GPIO_InitStruct2);

		/* Insert delay 100 ms */
		//HAL_Delay(100);

}
void LEDGREEN(){
	GPIO_InitTypeDef GPIO_InitStruct3 = {0};

		HAL_GPIO_WritePin(LEDPORT1, LEDPIN1, GPIO_PIN_SET);

		GPIO_InitStruct3.Pin = LEDPIN1;
		GPIO_InitStruct3.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct3.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LEDPORT1, &GPIO_InitStruct3);

		GPIO_InitStruct3.Pin = LEDPIN2;
		GPIO_InitStruct3.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct3.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LEDPORT2, &GPIO_InitStruct3);

		//HAL_Delay(100);

}

void LEDCLEAN(){
	HAL_GPIO_DeInit(LEDPORT1, LEDPIN1);
	HAL_GPIO_DeInit(LEDPORT2, LEDPIN2);
}

