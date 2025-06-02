/*
 * test_pressureStability.cpp
 *
 *  Created on: 17 févr. 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"
#include "Arduino.h"
#include "RTC.h"
#include "TimerMillis.h"
#include "test_accelero_1.h" //for SetDefaultRTC
#include "../Manager/_manager.h"

#define TICK_PERIOD (5000)

volatile bool tickFlag = false;

//const char *build_date = __DATE__ ;
//const char *build_time = __TIME__ ;

void _TickCallback(void){
	tickFlag = true;
}


void test_pressureStability_run(void){

	manager_rtc_setDefaultTime();
	Serial.begin(9600);

	delay(10000);

	TimerMillis _mainTimer;
	int32_t depth1BA,depth30BA;
	uint32_t ts;
	LOGLN("time;1BA;30BA");
	manager_depth_init();

	_mainTimer.start(_TickCallback,TICK_PERIOD,TICK_PERIOD);

	while(1){
		if(tickFlag){
			tickFlag = false;

			manager_depth_readPressure_mPa_1BA(&depth1BA);
			manager_depth_readPressure_mPa_30BA(&depth30BA);
			ts = RTC.getEpoch();

			LOG(ts);LOG(";");LOG(depth1BA);LOG(";");LOGLN(depth30BA);

		}
	}
}
