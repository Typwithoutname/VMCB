/*
 * CapacitiveSensor.h
 *
 *  Created on: Apr 18, 2020
 *      Author: Felix
 *
 *  Originally Created by PaulStoffregen
 *  https://github.com/PaulStoffregen/CapacitiveSensor
 */
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

#ifndef SRC_CAPACITIVESENSOR_H_
#define SRC_CAPACITIVESENSOR_H_

#include "stm32g0xx_hal.h"



#define DIRECT_READ(base, pin)          (GPIOB->IDR>>pin)&0x01
#define DIRECT_MODE_INPUT(base, pin)    GPIOB->MODER&=~(0x11<<(pin*2));						//pinMode(pin,INPUT)
#define DIRECT_MODE_OUTPUT(base, pin)   GPIOB->MODER|=(0x01<<(pin*2));
#define DIRECT_WRITE_LOW(base, pin)     GPIOB->BRR|=0x01<<(pin)
#define DIRECT_WRITE_HIGH(base, pin)    GPIOB->ODR|=0x01<<(pin)
#define interrupts()						//Need to be changed in final version
#define noInterrupts()
#define IO_REG_TYPE unsigned char


		// enums
#ifdef	LPMODE
enum States {OFF,RED,GREEN,LPMode};
#else
enum States {OFF,RED,GREEN};
#endif
		void CapacitiveSensorinit(unsigned char sendPin, unsigned char receivePin);
		long capacitiveSensorRaw(unsigned char samples);
		void delayMicroseconds(unsigned int value);
		void NextState(enum States* current);
		void NextStateLongPressed(enum States* current);
	  // library-accessible "private" interface

	  // variables
		int error;
		unsigned long  leastTotal;
		unsigned int   loopTimingFactor;
		unsigned long  CS_Timeout_Millis;
		unsigned long  CS_AutocaL_Millis;
		unsigned long  lastCal;
		unsigned long  total;
		unsigned long  f_CPU;
		IO_REG_TYPE sBit;   // send pin's ports and bitmask
		volatile IO_REG_TYPE *sReg;
		IO_REG_TYPE rBit;    // receive pin's ports and bitmask
		volatile IO_REG_TYPE *rReg;
	  // methods
		int SenseOneCycle(void);



#endif /* SRC_CAPACITIVESENSOR_H_ */






