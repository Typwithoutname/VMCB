/*
 * CapacitiveSensor.cpp
 *
 *  Created on: Apr 18, 2020
 *      Author: Felix
 */
#include "stm32g0xx_hal.h"
#include "CapacitiveSensor.h"
/*

 CapacitiveSense.h - Capacitive Sensing Library for 'duino / Wiring
 https://github.com/PaulStoffregen/CapacitiveSensor
 http://www.pjrc.com/teensy/td_libs_CapacitiveSensor.html
 http://playground.arduino.cc/Main/CapacitiveSensor
 Copyright (c) 2009 Paul Bagder
 Updates for other hardare by Paul Stoffregen, 2010-2016
 vim: set ts=4:


 Permission is hereby granted, free of charge, to any person obtaining a
 copy of this software and associated documentation files (the "Software"),
 to deal in the Software without restriction, including without limitation
 the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.


 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 DEALINGS IN THE SOFTWARE.
*/

#define LPMODE
int sleepmodevar=0;

void CapacitiveSensorinit()
{
	// initialize this instance's variables
	// Serial.begin(9600);		// for debugging
	error = 1;
	f_CPU=16000000;
	loopTimingFactor = 310;		// determined empirically -  a hack
	CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)f_CPU) / 16000000;
	CS_AutocaL_Millis = 20000;


	DIRECT_MODE_OUTPUT(0,sBit);						// sendpin to OUTPUT
	DIRECT_MODE_INPUT(0,rBit);						// receivePin to INPUT
	DIRECT_WRITE_LOW(0,sBit);

	// get pin mapping and port for receive Pin - from digital pin functions in Wiring.c
	leastTotal = 0x0FFFFFFFL;   // input large value for autocalibrate begin
	//lastCal = millis();         // set millis for start
}


long capacitiveSensorRaw(unsigned char samples)
{
	/*sleepmodevar=checksleepMode();
	if(sleepmodevar==10){
		return -10;
	}*/
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;                  // bad pin - this appears not to work
	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
	}
	return total;
}



int SenseOneCycle(void)
{
    noInterrupts();
	DIRECT_WRITE_LOW(sendPin, sBit);	// sendPin Register low
	DIRECT_MODE_INPUT(receivePin, rBit);	// receivePin to input (pullups are off)
	DIRECT_MODE_OUTPUT(receivePin, rBit); // receivePin to OUTPUT
	DIRECT_WRITE_LOW(receivePin, rBit);	// pin is now LOW AND OUTPUT
	delayMicroseconds(10);
	//HAL_Delay(1000);
	DIRECT_MODE_INPUT(receivePin, rBit);	// receivePin to input (pullups are off)
	DIRECT_WRITE_HIGH(sendPin, sBit);	// sendPin High
    interrupts();


	while ( !DIRECT_READ(receivePin, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is LOW AND total is positive value
		total++;
	}
	//Serial.print("SenseOneCycle(1): ");
	//Serial.println(total);


	if (total > CS_Timeout_Millis) {
		return -2;         //  total variable over timeout
	}

	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V
    noInterrupts();
	DIRECT_WRITE_HIGH(receivePin, rBit);
	DIRECT_MODE_OUTPUT(receivePin, rBit);  // receivePin to OUTPUT - pin is now HIGH AND OUTPUT
	DIRECT_WRITE_HIGH(receivePin, rBit);
	DIRECT_MODE_INPUT(receivePin, rBit);	// receivePin to INPUT (pullup is off)
	DIRECT_WRITE_LOW(sendPin, sBit);	// sendPin LOW
    interrupts();

	while ( DIRECT_READ(receivePin, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is HIGH  AND total is less than timeout
		total++;
	}

	//Serial.print("SenseOneCycle(2): ");
	//Serial.println(total)
	if (total >= CS_Timeout_Millis) {
		return -2;     // total variable over timeout
	} else {
		return 1;
	}
}

void delayMicroseconds(volatile unsigned int microsec){

	for(int i=0;i<microsec*160;i++){}

}


void SystemClock_Config3(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void SystemClock_Decrease2(void){
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Select HSI as system clock source a */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Modify HSI to HSI DIV8 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV8;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void MX_GPIO_Deinit(){
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
	//LEDCLEAN();
}

void MX_GPIO_Init(void)   //funktion aus Main einfach reinkopieren
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  /*Configure GPIO pin : PA12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  delayMicroseconds(10);
}

void initTaster(){

	  HAL_GPIO_WritePin(LP_PORT, LP_pin, GPIO_PIN_RESET);
	  /* PA.11 will be used to exit from Low Power Run mode */
	  GPIO_InitTypeDef      GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = LP_pin;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	  /* Enable GPIOA clock */
	  __HAL_RCC_GPIOA_CLK_ENABLE();									//////////////////////könnte zu Fehler führen

	  HAL_GPIO_Init(LP_PORT, &GPIO_InitStruct);
}

void deinitTaster(){
	HAL_GPIO_DeInit(LP_PORT, LP_pin);
}



void NextState(enum States* current){
	switch (*current){
	case OFF:
		*current=GREEN;
		LEDGREEN();
	break;
	case GREEN:
		*current=YELLOW;
		LEDYELLOW();
	break;
	case YELLOW:
		*current=RED;
		LEDRED();
	break;
	case RED:
		*current=OFF;
		LEDCLEAN();
	break;
	default:
	break;
	}
}

void sleep(enum States* current){
	MX_GPIO_Deinit();
	initTaster();
	//LEDCLEAN();
	SystemClock_Decrease2();
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
	HAL_PWREx_EnableLowPowerRunMode();
	while(HAL_GPIO_ReadPin(LP_PORT, LP_pin) == GPIO_PIN_SET)
		     {
		     }
	HAL_PWREx_DisableLowPowerRunMode();
	SystemClock_Config();
	deinitTaster();
	initLED();
	MX_GPIO_Init();
	CapacitiveSensorinit();
}

void NextStateLongPressed(enum States* current){
	switch (*current){

	break;
	default:
	break;
	}
}

int checksleepMode(){
	initTaster();
	if(HAL_GPIO_ReadPin(LP_PORT, LP_pin)==GPIO_PIN_SET){
								//Button pressed
		return 10;
	}
	else{
		deinitTaster();
		CapacitiveSensorinit();							//Button not pressed
		return 0;
	}
}

