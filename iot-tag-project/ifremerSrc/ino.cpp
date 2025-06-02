/*
 * ino.cpp
 *
 *  Created on: 17 janv. 2020
 *      Author: jfezande
 */


#include "Arduino.h"

#include "src/_appDirectives.h"
#include "src/_board.h"

#include "src/_application.h"
#include "src/Tests/_tests.h"



void setup(void){

	tests_launch(); //add your test in _test.h
	application_init();
}


void loop(void){

	application_process();
}
