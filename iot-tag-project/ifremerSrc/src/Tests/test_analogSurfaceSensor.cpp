/*
 * test_analogSurfaceSensor.c
 *
 *  Created on: 9 mars 2020
 *      Author: jfezande
 */

#include "Arduino.h"
#include "TimerMillis.h"
#include "STM32L0.h"

#include "../_board.h"
#include "../_appDirectives.h"

#include "../Manager/_manager.h"

#include "test_analogSurfaceSensor.h"


//test simulé de l'application avec capteur de surface avec un nombre d'état minimal



#define DELTA_SURFACESENSOR	500
volatile bool timerFlagAnalogSurfaceSensor = false;
volatile bool timerFlagAnalogSurfaceSensor_app = false;

TimerMillis timerS;
uint16_t wetValue = 0;

uint16_t lastNValue[3];
uint16_t lastValue;

void analogSurfaceSensor_cb(void){
	timerFlagAnalogSurfaceSensor = true;
	STM32L0.wakeup();
}

void analogSurfaceSensor_app_cb(void){
	timerFlagAnalogSurfaceSensor_app = true;
	STM32L0.wakeup();
}


void _startSurfSensor(void){
	timerS.start(analogSurfaceSensor_cb, 0, 500);
	wetValue = 0; //reset de la valeur
}
void _initWetValue(void){
	wetValue = lastValue;
	LOG("WET VALUE = ");LOGLN(wetValue);


}



void test_analogSurfaceSensor(void){

	const uint8_t measTabSize = 5;

	TimerMillis timerS_app;
	bool _surfaceDetected = false;
	uint8_t _intState = 0;

	Serial.begin(9600);
	delay(5000);
	LOGLN("Let s go");
	//timerS.start(analogSurfaceSensor_cb, 0, 500);
	timerS_app.start(analogSurfaceSensor_app_cb,0,1000);
	uint32_t acc;


	pinMode(switchPin, OUTPUT);
	digitalWrite(switchPin, LOW);

	manager_depth_init();


	while(1){

		if(timerFlagAnalogSurfaceSensor){
			timerFlagAnalogSurfaceSensor = false;


			digitalWrite(switchPin, HIGH);

			acc = 0;

			for(uint8_t i =0; i<measTabSize; i++){

				acc += 10*analogRead(sensorPin);  //x10 -> precision 0.1 sans utiliser de float
			}
			//Serial.println(current_measure);
			digitalWrite(switchPin, LOW);

			acc /= measTabSize;
			lastValue = (uint16_t)acc;
			LOG("surface sensor : ");LOGLN(acc);

			if(wetValue !=0){
				if(lastValue >= (wetValue + DELTA_SURFACESENSOR)){
					_surfaceDetected = true;
					timerS.stop();
					timerFlagAnalogSurfaceSensor_app = true;
				}

			}



		}


		if(timerFlagAnalogSurfaceSensor_app){
			int32_t pressureValue;
			timerFlagAnalogSurfaceSensor_app = false;
			switch(_intState){
			case 0: //état d'inititlisation en surface //attente de plongé
				LOGLN("-- en surface --");
				manager_depth_get_mm(pressureSensor_1BA,&pressureValue);

				if(pressureValue> 150)
					_intState = 1; //on plonge

				break;

			case 1:
				LOGLN("-- strat sensor --");

				_startSurfSensor();
				_intState = 2;
				break;
			case 2:
				LOGLN("-- INIT WET --");

				_initWetValue();
				_intState = 3;
				break;
			case 3:

				_intState = 4; //attente de 1s
				break;
			case 4:
				LOGLN("-- DIVE --");
				if(_surfaceDetected){
					LOGLN("En Surface");
					_intState = 0;
				}

				break;




			}



		}


		STM32L0.stop((10*60*1000UL));
	}





}

