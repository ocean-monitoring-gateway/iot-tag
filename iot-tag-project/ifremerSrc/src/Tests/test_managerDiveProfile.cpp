/*
 * test_managerDiveProfile.cpp
 *
 *  Created on: 10 août 2020
 *      Author: jfezande
 */

#include "Arduino.h"
#include "string.h"
#include "../_appDirectives.h"
#include "../Manager/_manager.h"
#include "test_managerDiveProfile.h"


void _logProfile(manager_diveProfile_result_t *p){

	LOG("[");
	for(uint8_t i =0; i<9; i++){
		LOG(p->profile[i]);LOG(",");
	}

	LOG(p->profile[9]);LOG("]");

	LOG(" with : "); LOGLN(p->timeStep);


}

void _logDiveManager(manager_diveProfile_t *p){

	LOG("Current Step : ");LOGLN(p->currentStep);
	LOG("Nb Point : ");LOGLN(p->meanIndex+1);

	LOG("[");

	for(uint8_t i = 0; i<p->meanIndex ; i++){
		LOG(p->means[i]);LOG(",");
	}
	LOG(p->means[p->meanIndex]);LOGLN("]");


}

void test_managerDiveProfile_launch(void){
	//on simule le tick en mode très rapide

	LOGINIT();

	delay(10000);
	for(uint8_t i = 0; i<20; i++){
		LOG(".");
		delay(500);
	}

	LOGLN("!");

	LOGLN("Start Dive Profile test");

	uint8_t profilResult[10];
	uint16_t i;


	//test 1 : remplissage simple sur 20 périodes
	// test allant de 10cm à 2m
	manager_diveProfile_t divePr;
	manager_diveProfile_result_t res;


	LOGLN("");

	manager_diveProfile_start(&divePr);


	for(i=0; i< 20; i++){
		uint16_t value = i;
		value *= 100;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);

	//test 2 même série avec un des valeurs allalnt de 1 à 20m
	manager_diveProfile_start(&divePr);


	for(i=0; i< 15; i++){
		uint16_t value = i;
		value *= 1000;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);


	//test 3 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 20; i++){
		uint16_t value = i;
		value *= 1000;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);

	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 35; i++){
		uint16_t value = i;
		value *= 1000;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);


	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 45; i++){
		uint16_t value = i;
		value *= 1000;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);


	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 55; i++){
		uint16_t value = i;
		value *= 1000;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);


	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 55; i++){
		uint16_t value = i;
		value *= 100;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);


	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 65; i++){
		uint16_t value = i;
		value *= 100;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");

	_logDiveManager(&divePr);
	_logProfile(&res);



	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 143; i++){
		uint16_t value = i;
		value *= 100;

		for(uint8_t j = 0; j<30; j++){

			manager_diveProfile_addPoint(&divePr, value);
		}
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);



	//test 4 même série avec un des valeurs allalnt de 1 à 20m
	LOGLN("");
	manager_diveProfile_start(&divePr);


	for(i=0; i< 46; i++){


			manager_diveProfile_addPoint(&divePr, i*10);
	}





	manager_diveProfile_end(&divePr);
	manager_diveProfile_get(&divePr, &res);

	LOG("Test with : ");LOG(i);LOGLN("pts");
	_logDiveManager(&divePr);
	_logProfile(&res);

}
