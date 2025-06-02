/*
 * test_adc.cpp
 *
 *  Created on: 17 févr. 2020
 *      Author: jfezande
 */

#include "Arduino.h"

#include "../_appDirectives.h"

#include "stm32l0_adc.h"
#include "stm32l0_gpio.h"
#include "test_adc.h"

void test_adc_read(void){

	const unsigned int ulPin = A3;

	int _readPeriod = 20;
	uint32_t data, _loopDelay;
	uint16_t count = 0;



	// {loopDelay, readPeriod}
	const uint32_t params[6][2] = { {10, 2}, {10, 20}, {10, 10}, {50, 2}, {50, 20}, {100, 20}};
	const uint8_t paramsMax = 4;
	uint8_t params_index = 0;

	uint64_t accData = 0;
	uint32_t nbData = 0;
	uint32_t min = 0xFFFFFFFF;
	uint32_t max = 0;

	Serial.begin(9600);

	delay(10000);

	_loopDelay = params[params_index][0];
	_readPeriod = (int)params[params_index][1];

	LOGLN("Period;Loop Delay;Moy;Min;Max");
	pinMode(9, OUTPUT);
	digitalWrite(9, LOW);
//	LOG("Period = ");LOGLN(_readPeriod);
//	LOG("Loop delay = ");LOGLN(_loopDelay);
	LOG(_readPeriod);LOG(";");LOG(_loopDelay);LOG(";");
	while(1){
		delay(_loopDelay);
		digitalWrite(9, HIGH);
		stm32l0_gpio_pin_configure(g_APinDescription[ulPin].pin, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_MODE_ANALOG));

		stm32l0_adc_enable();

		data = stm32l0_adc_read(g_APinDescription[ulPin].adc_channel, _readPeriod );

		stm32l0_adc_disable();

		//adcValue = mapResolution(data, 12, _readResolution);

		digitalWrite(9, LOW);

		accData += data;
		nbData++;
		if(data < min) min = data;
		if(data > max) max = data;
		LOGLN(data);

		/*if(count++ > 100){
			count = 0;
			//LOG("Moy = ");LOG((uint32_t)(accData/nbData));LOG(" ,Min = ");LOG(min);LOG(" ,Max = ");LOGLN(max);
			LOG((uint32_t)(accData/nbData));LOG(";");LOG(min);LOG(";");LOGLN(max);
			delay(2000);
			//LOGLN("New series");
			//LOG("Period = ");LOGLN(_readPeriod);
			//LOG("Loop delay = ");LOGLN(_loopDelay);
			LOG(_readPeriod);LOG(";");LOG(_loopDelay);LOG(";");


			accData = 0;
			nbData = 0;
			min = 0xFFFFFFFF;
			max = 0;

			params_index++;
			if(params_index == paramsMax) params_index = 0;

			_loopDelay = params[params_index][0];
			_readPeriod = (int)params[params_index][1];

		}*/

	}
}
