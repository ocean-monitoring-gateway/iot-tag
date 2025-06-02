/*
 * test_pressure.cpp
 *
 *  Created on: 29 janv. 2020
 *      Author: jfezande
 */


#include "Arduino.h"
#include "../Manager/_manager.h"
#include "test_pressure.h"




void test_pressure(void){
	Serial.begin(9600);
	delay(5000);
	
	int32_t depth_value30BA, depth_value1BA;
	manager_depth_init();

	
	while(1){

		manager_depth_get_mm(pressureSensor_1BA, &depth_value1BA);
		Serial.print("1BA value : ");Serial.print(depth_value1BA);Serial.println(" mm");

		manager_depth_get_mm(pressureSensor_30BA, &depth_value30BA);
		Serial.print("30BA value : ");Serial.print(depth_value30BA);Serial.println(" mm");

		delay(1000);

		








	}



}

