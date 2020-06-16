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

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

void CapacitiveSensorinit(unsigned char sendPin, unsigned char receivePin)
{
	// initialize this instance's variables
	// Serial.begin(9600);		// for debugging
	error = 1;
	f_CPU=16000000;
	loopTimingFactor = 310;		// determined empirically -  a hack
	CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)f_CPU) / 16000000;
	CS_AutocaL_Millis = 20000;


	DIRECT_MODE_OUTPUT(0,sendPin);						// sendpin to OUTPUT
	DIRECT_MODE_INPUT(0,receivePin);						// receivePin to INPUT
	DIRECT_WRITE_LOW(0,sendPin);

	sBit = (sendPin);					// get send pin's ports and bitmask
	sReg = 0;					// get pointer to output register
	rBit = (receivePin);				// get receive pin's ports and bitmask
	rReg = 0;

	// get pin mapping and port for receive Pin - from digital pin functions in Wiring.c
	leastTotal = 0x0FFFFFFFL;   // input large value for autocalibrate begin
	//lastCal = millis();         // set millis for start
}


long capacitiveSensorRaw(unsigned char samples)
{
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
	DIRECT_WRITE_LOW(sReg, sBit);	// sendPin Register low
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to input (pullups are off)
	DIRECT_MODE_OUTPUT(rReg, rBit); // receivePin to OUTPUT
	DIRECT_WRITE_LOW(rReg, rBit);	// pin is now LOW AND OUTPUT
	delayMicroseconds(10);
	//HAL_Delay(1000);
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to input (pullups are off)
	DIRECT_WRITE_HIGH(sReg, sBit);	// sendPin High
    interrupts();


	while ( !DIRECT_READ(rReg, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is LOW AND total is positive value
		total++;
	}
	//Serial.print("SenseOneCycle(1): ");
	//Serial.println(total);


	if (total > CS_Timeout_Millis) {
		return -2;         //  total variable over timeout
	}

	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V
    noInterrupts();
	DIRECT_WRITE_HIGH(rReg, rBit);
	DIRECT_MODE_OUTPUT(rReg, rBit);  // receivePin to OUTPUT - pin is now HIGH AND OUTPUT
	DIRECT_WRITE_HIGH(rReg, rBit);
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to INPUT (pullup is off)
	DIRECT_WRITE_LOW(sReg, sBit);	// sendPin LOW
    interrupts();

	while ( DIRECT_READ(rReg, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is HIGH  AND total is less than timeout
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
#ifdef LPMODE
void SystemClock_Config2(void){
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
    //Error_Handler();
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

void SystemClock_Decrease(void)
{
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
#endif

void NextState(enum States* current){
	switch (*current){
	case OFF:
		*current=GREEN;
	break;
	case GREEN:
		*current=RED;
	break;
	case RED:
		HAL_GPIO_TogglePin (GPIOA,GPIO_PIN_12);
		HAL_Delay(2000);
		HAL_GPIO_TogglePin (GPIOA,GPIO_PIN_12);
		*current=OFF;
	break;
	default:
	break;
	}
}

void NextStateLongPressed(enum States* current){
	switch (*current){
#ifdef LPMODE
	case OFF:
		*current=LPMode;
		SystemClock_Decrease();
	    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

		HAL_PWREx_EnableLowPowerRunMode();

	break;
	case LPMode:
		*current=OFF;
		HAL_PWREx_DisableLowPowerRunMode();
	    SystemClock_Config2();
	break;
#endif
	default:
	break;
	}
}

