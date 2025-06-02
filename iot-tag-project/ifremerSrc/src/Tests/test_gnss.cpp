/*
 * test_gnss.cpp
 *
 *  Created on: 16 janv. 2020
 *      Author: jfezande
 */

#include "Arduino.h"
#include "GNSS.h"
#include "TimerMillis.h"
#include "RTC.h"
#include "LoRaWAN.h"

#include "../_appDirectives.h"
#include "../LoraApplication/ifremerLoraMsg.h"
#include "../Manager/_manager.h"

GNSSLocation myLocation;
TimerMillis timer;

#define TICKMAIN	1000
#define RESTART_DELAY	60000


volatile bool flagTick = false;

bool isFixGPS(void){

  return (myLocation.fixType() != GNSSLocation::TYPE_NONE) && (myLocation.fixType() != GNSSLocation::TYPE_TIME);

}

void callbackTick(void){

	flagTick = true;
}


bool gnnsRun = false;

void test_gnss_ttff(void){

	uint32_t startTime, fixTime;
	uint8_t firstFix = 0; //1 for first Fix

	data_TTFF_t ttff;
	data_gnssTracking_t gnssPoint;
	bool gnssRun = false;

	const float EHPE = 20;

	manager_gnss_init();

	Serial.begin(9600);



	LoRaWAN.begin(EU868);
	LoRaWAN.setADR(false);
	LoRaWAN.setDataRate(DATARATE);
	LoRaWAN.setTxPower(10);
	LoRaWAN.setSubBand(1);
	LoRaWAN.setDutyCycle(false);

	char _DevEUI[17];

	delay(5000);
	Serial.println("start");

	LoRaWAN.getDevEui(_DevEUI,17);
	Serial.println(_DevEUI);


	while(!LoRaWAN.joined()){
		if(!LoRaWAN.busy()){

			LoRaWAN.joinOTAA(APPEUI, APPKEY, _DevEUI);
			Serial.println("Join attempt");

		}
		else
			Serial.println("Lora busy");

		delay(1000);
	}


	LOG("Join Success");



    delay(20000);
    manager_gnss_setState(GNSS_ON);
    while(manager_gnss_getState() != GNSS_ON)
    	manager_gnss_processState();

    gnssRun = true;
    startTime = RTC.getEpoch();

    LOG("Start Timer");
    timer.start(callbackTick, TICKMAIN, TICKMAIN);




	while(1){//loop forever

		if(flagTick){
			flagTick = false;
			LOG(".");

			if(!gnssRun){
				LOG("Sarting GPS");
				manager_gnss_setState(GNSS_ON);
				    while(manager_gnss_getState() != GNSS_ON)
				    	manager_gnss_processState();

				    startTime = RTC.getEpoch();

				    gnssRun = true;
				    timer.restart(TICKMAIN, TICKMAIN);
					LOG("Launch Main Tick");


			}

			else{
			GNSS.location(myLocation);
				LOG("Try to fix");
				manager_gnss_logSattelites();
			if (isFixGPS()){

				LOG("EHPE");
				LOG(myLocation.ehpe() );
				if(myLocation.ehpe() < EHPE){
					uint32_t ctime = RTC.getEpoch();

					gnssPoint.lat = myLocation.latitude();
					gnssPoint.lng = myLocation.longitude();
					gnssPoint.ehpe = myLocation.ehpe();
					gnssPoint.ttf = ctime - startTime;

					LOG("Lora Send");

					LoRaWAN.sendPacket(loraPortNumber[ilp_GnssTracking], (uint8_t*)&gnssPoint, sizeof(data_gnssTracking_t));

					//stop the GPS
				    manager_gnss_setState(GNSS_OFF);
				    while(manager_gnss_getState() != GNSS_OFF)
				    	manager_gnss_processState();

				    gnssRun = false;
				    timer.restart(RESTART_DELAY, RESTART_DELAY);

				    LOG("DELAY TIMER");


				}


				/*if(firstFix == 0){
					firstFix = 1;

					fixTime = RTC.getEpoch() - startTime;

					ttff.ttff = (uint16_t)fixTime;

					ttff.ehpe[0] = (uint16_t)(myLocation.ehpe() * 10 + 0.5);
					timer.stop();
					timer.start(callbackTick, 10000, 10000);
				}
				else if( (firstFix > 0) && (firstFix<7)){
					//read the ehpe each 10s
					ttff.ehpe[firstFix] = (uint16_t)(myLocation.ehpe() * 10 + 0.5);
					firstFix++;

					if(firstFix>6){
					    manager_gnss_setState(GNSS_OFF);

					    while(manager_gnss_getState() != GNSS_OFF)
					    	manager_gnss_processState();

					    timer.restart(10 * 60000, 10 *60000);
					}
				}else{
				    //GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ);
					timer.restart(2000, 2000);

				}*/





			}


		}

	}



	}


}



