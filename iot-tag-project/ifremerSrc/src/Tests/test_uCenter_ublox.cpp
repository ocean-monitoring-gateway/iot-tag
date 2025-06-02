/*
 * test_uCenter_ublox.cpp
 *
 *  Created on: 30 juil. 2020
 *      Author: jfezande
 */

#include "Arduino.h"
#include "wiring_private.h"
//permet de connecter la balise à uCEnter en renvoyant les donnees USB vers le serial du GPS


void test_uCenter_launch(void){
	uint8_t buf[512];
	uint8_t buf1[512];

	int size;
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_ENABLE, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_ENABLE, 1);

	pinMode(A0, OUTPUT);   // power for MAX M8Q RTC backup
	digitalWrite(A0, HIGH);



	Serial1.begin(9600); //port serie du GPS

	Serial.begin(115200);

	delay(5000);


	while(1){
		memset(buf,0,512);
		memset(buf1,0,512);

		//lire l'entrée USB
		size = Serial.available();
		Serial.read(buf, size);
		Serial1.print((char*)buf);


		size = Serial1.available();
		Serial1.read(buf1, size);
		Serial.print((char*)buf1);

		delay(10);
	}


}
