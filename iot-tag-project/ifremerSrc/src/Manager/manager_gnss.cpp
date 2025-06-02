/*
 * manager_gnss.cpp
 *
 *  Created on: 4 févr. 2020
 *      Author: jfezande
 */

#include "Arduino.h"
#include "GNSS.h"
#include "manager_gnss.h"
#include "wiring_private.h"

// MAX M8Q GNSS configuration
#define GNSS_en      5     // enable for GNSS 3.0 V LDO
#define pps          4     // 1 Hz fix pulse
#define GNSS_backup A0     // RTC backup for MAX M8Q

bool _gnssState = false;
bool _gnssNeedState = false;

void manager_gnss_init(void){

	pinMode(GNSS_backup, OUTPUT);   // power for MAX M8Q RTC backup
	digitalWrite(GNSS_backup, HIGH);

	//GNSS init
	#warning Add timeout to busy wait
	GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ);
	while (GNSS.busy());
	GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS);
	while (GNSS.busy());
	GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL); // GNSS.ANTENNA_INTERNAL or GNSS.ANTENNA_EXTERNAL
	while (GNSS.busy()) { } // wait for set to complete

	GNSS.enableWakeup();
	while (GNSS.busy()) { } // wait for set to complete

	GNSS.suspend();
}


void manager_gnss_setState(bool state){

	_gnssNeedState = state;
	manager_gnss_processState();

}

bool manager_gnss_getState(void){
	return _gnssState;
}


void manager_gnss_processState(void){

	if(_gnssNeedState != _gnssState){

		if(_gnssNeedState){ //
			if(GNSS.resume()){
				_gnssState = _gnssNeedState;
				Serial.println("GNSS ON");
				while(GNSS.busy())
					delay(1);
			}
		}
		else{
			if(GNSS.suspend()){
				_gnssState = _gnssNeedState;
				Serial.println("GNSS OFF");
				while(GNSS.busy())
					delay(1);
			}
		}
	}

}

void manager_gnss_Vdd_En(bool state){
	if(state)
		stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_ENABLE, 1);
	else
	    stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_ENABLE, 0);


}
void manager_gnss_Backup_En(bool state){
	if(state)
		digitalWrite(GNSS_backup, HIGH);
	else
		digitalWrite(GNSS_backup, LOW);

}



void manager_gnss_logSattelites(void){

	GNSSSatellites mySatellites;
	GNSS.satellites(mySatellites);

	uint8_t nbSat = mySatellites.count();

	Serial.print("Nb Sat : ");Serial.println(nbSat);

	for (uint8_t index = 0; index < nbSat; index++){

        Serial.print(": SNR=");
        Serial.print(mySatellites.snr(index));
        Serial.print(", ELEVATION=");
        Serial.print(mySatellites.elevation(index));
        Serial.print(", AZIMUTH=");
        Serial.print(mySatellites.azimuth(index));
        if (mySatellites.unhealthy(index)) {
            Serial.print(", UNHEALTHY");
        }

        if (mySatellites.almanac(index)) {
            Serial.print(", ALMANAC");
        }

        if (mySatellites.ephemeris(index)) {
            Serial.print(", EPHEMERIS");
        }

        if (mySatellites.autonomous(index)) {
            Serial.print(", AUTONOMOUS");
        }

        if (mySatellites.correction(index)) {
            Serial.print(", CORRECTION");
        }

        if (mySatellites.acquired(index)) {
            Serial.print(", ACQUIRED");
        }

        if (mySatellites.locked(index)) {
            Serial.print(", LOCKED");
        }

        if (mySatellites.navigating(index)) {
            Serial.print(", NAVIGATING");
        }

        Serial.println();



	}



}
