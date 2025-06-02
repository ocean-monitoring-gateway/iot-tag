/*
 * manager_imu.cpp
 *
 *  Created on: 2 mars 2020
 *      Author: jfezande
 */


#include "Arduino.h"
#include "../_appDirectives.h"
#include "../_board.h"

#include "manager_imu.h"



manager_imu::manager_imu(void){

}


void manager_imu::init(void){
	_accelero_int1_callback = NULL;
	_accelero_int2_callback = NULL;
	_mangeto_callback = NULL;

}

imuMove_t manager_imu::getCurrentMove(void){
	return imuM_moving;
}

void manager_imu::_handler_accelero1(void){
	if(_accelero_int1_callback != NULL) _accelero_int1_callback();
}

void manager_imu::_handler_accelero2(void){
	if(_accelero_int2_callback != NULL) _accelero_int2_callback();
}
void manager_imu::_handler_magneto(void){
	if(_mangeto_callback != NULL) _mangeto_callback();
}



