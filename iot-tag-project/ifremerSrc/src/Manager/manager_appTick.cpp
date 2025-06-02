/*
 * manager_apppTick.cpp
 *
 *  Created on: 10 août 2020
 *      Author: jfezande
 */

#include "manager_appTick.h"

static volatile uint32_t _tick;

void manager_appTick_init(void){
	_tick = 0;
}

void manager_appTick_tick(void){
	_tick++;
}

uint32_t manager_appTick_get(void){
	return _tick;

}




