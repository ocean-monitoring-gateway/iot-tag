/*
 * manager_lowpower.cpp
 *
 *  Created on: 22 janv. 2020
 *      Author: jfezande
 */


#include "STM32L0.h"
#include "manager_lowpower.h"

static uint8_t needSleep = 0;

#define SLEEP_MAXTIMEOUT	(10*60*1000UL) //10min


void manager_lowpower_needSleep(void){

	needSleep++;

}
void manager_lowpower_releaseSleep(void){

	if(needSleep > 0)
		needSleep --;
}




void manager_lowpower_manage(void){
//#warning GERER LE MODE LOWPOWER
	//return;

	if(needSleep != 0)
		STM32L0.sleep(SLEEP_MAXTIMEOUT);
	else
		STM32L0.stop(SLEEP_MAXTIMEOUT);

}
