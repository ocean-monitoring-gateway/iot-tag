/*
 * shell.cpp
 *
 *  Created on: 7 févr. 2020
 *      Author: jfezande
 */



#include "Arduino.h"
#include "shell.h"


void shell_receive(void){
	uint8_t buf[10];
	int size = Serial.available();

	if(size>0){

		int readSize = (size>10) ? 10 : size;



		Serial.read(buf, readSize);

		Serial.print("S receive : ");
		Serial.println((char*)buf);

	}



}
