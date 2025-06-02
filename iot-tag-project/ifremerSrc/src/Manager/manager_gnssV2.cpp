/*
 * manager_gnssV2.cpp
 *
 *  Created on: 17 juil. 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"

#include "Arduino.h"
#include "GNSS.h"
#include "wiring_private.h"
#include "RTC.h"

#include "../_board.h"
#include "../_manager_gnss_config.h"
#include "manager_appTick.h"
#include "manager_gnssV2.h"


static const char *fixTypeString[] = {
		"NONE",
		"TIME",
		"2D",
		"3D",
};


void _busyWait(int timeout_ms){




}


void _managerON(manager_gnss_t *pManager){
	pManager->state = GNSS_ON;
	//when a new session start
	pManager->lastEHPE = 10000 * 1000;
	pManager->startTime = manager_appTick_get();
	pManager->noFixCount++;


	pManager->nbSat = 0;
	pManager->nbSatPowered = 0;

	LOGLN("[gnssV2] ON");

}

void _managerOFF(manager_gnss_t *pManager){
	pManager->state = GNSS_OFF;


	uint32_t sessionUseTime;

	sessionUseTime = manager_appTick_get() - pManager->startTime;

	pManager->dailyUseTime += sessionUseTime;
	pManager->totalUseTime += sessionUseTime;
	LOG("[gnssV2] Session time ");LOGLN(sessionUseTime);
	LOG("[gnssV2] Daily time ");LOGLN(pManager->dailyUseTime);
	LOG("[gnssV2] Total time ");LOGLN(pManager->totalUseTime);

	LOGLN("[gnssV2] OFF");

}

void _processState(manager_gnss_t *pManager){
	if(pManager->neededState != pManager->state){

		if(pManager->neededState){ //
			//digitalWrite(GNSS_backup, HIGH);

#ifdef GNSS_FIXDELAYLOCK
			if(pManager->lastFixTime > 0){
				// at least a first good fix
				if((manager_appTick_get() - pManager->lastFixTime) < FIXDELAYLOCK_DURATION_S ){
					//cancel the GNSS start sequence
					LOGLN("[gnssV2] FixDelay Lock : cancel start");
					pManager->fixDelayLocked++;
					pManager->neededState = GNSS_OFF;
					return;
				}
			}
#endif

			if(GNSS.resume()){
				//pManager->state = pManager->neededState;
				_managerON(pManager);
				uint16_t i = 0;
				LOG("GNSS ON");
				while(GNSS.busy()){
					delay(1);
					i++;
				}
				LOG(" after ");LOG(i);LOGLN("ms");
			}
		}
		else{
			if(GNSS.suspend()){
				_managerOFF(pManager);
				//digitalWrite(GNSS_backup, LOW);

				//				pManager->state = pManager->neededState;
				LOG("GNSS OFF");
				uint16_t i = 0;
				while(GNSS.busy()){
					delay(1);
					i++;
				}
				LOG(" after ");LOG(i);LOGLN("ms");

			}
		}
	}


}


void _logSattelites(GNSSSatellites *pSat){


	uint8_t nbSat = pSat->count();

	LOG("Nb Sat : ");LOGLN(nbSat);

	for (uint8_t index = 0; index < nbSat; index++){
		LOG(pSat->svid(index));

		LOG(" : SNR=");
		LOG(pSat->snr(index));
		LOG(", ELEVATION=");
		LOG(pSat->elevation(index));
		LOG(", AZIMUTH=");
		LOG(pSat->azimuth(index));
		if (pSat->unhealthy(index)) {
			LOG(", UNHEALTHY");
		}

		if (pSat->almanac(index)) {
			LOG(", ALMANAC");
		}

		if (pSat->ephemeris(index)) {
			LOG(", EPHEMERIS");
		}

		if (pSat->autonomous(index)) {
			LOG(", AUTONOMOUS");
		}

		if (pSat->correction(index)) {
			LOG(", CORRECTION");
		}

		if (pSat->acquired(index)) {
			LOG(", ACQUIRED");
		}

		if (pSat->locked(index)) {
			LOG(", LOCKED");
		}

		if (pSat->navigating(index)) {
			LOG(", NAVIGATING");
		}

		LOGLN();



	}



}






void manager_gnssV2_init(manager_gnss_t *pManager){

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

	pManager->noFixCount = 0;
	pManager->zeroSatTimeoutCount = 0;
	pManager->dailyUseTime = 0;
	pManager->totalUseTime = 0;

	pManager->lastFixTime = 0;

	pManager->state = GNSS_OFF;



}


void manager_gnssV2_setON(manager_gnss_t *pManager){
	pManager->neededState = GNSS_ON;
	_processState(pManager);
}

void manager_gnssV2_setOFF(manager_gnss_t *pManager){
	pManager->neededState = GNSS_OFF;
	_processState(pManager);
}

void manager_gnssV2_process(manager_gnss_t *pManager){
	_processState(pManager);



	if(pManager->state == GNSS_ON){
		uint32_t currentTime = manager_appTick_get();
		//uint32_t currentOnDuration = manager_appTick_get() - pManager->startTime;
		LOG("[gnssV2] Sat On since ");LOG(currentTime - pManager->startTime);LOGLN("s");

		GNSSSatellites mySatellites;
		GNSS.satellites(mySatellites);

		_logSattelites(&mySatellites);

		//check the number of satellite
		uint16_t sat = mySatellites.count();
		if(sat > pManager->nbSat) pManager->nbSat = sat;

		//count the number of sat with a minimum of SNR
		uint16_t satPwd = 0;
		for(uint16_t i = 0; i< sat; i++ ){
			if( mySatellites.snr(i) >= GNSS_SNR_LEVEL) satPwd++;

		}
		if(satPwd > pManager->nbSatPowered) pManager->nbSatPowered = satPwd;

		////// NoSat TimeOut /////
		#ifdef GNSS_ZEROSATTIMEOUT
		if((pManager->nbSatPowered < 6) && (satPwd == 0)){
			LOGLN("[gnssV2] No Sat");

			if((currentTime - pManager->startTime) >= GNSS_ZEROSATTIMEOUT_S){

				LOGLN("[gnssV2] Zero Sat");
				pManager->zeroSatTimeoutCount++;
				manager_gnssV2_setOFF(pManager);
			}
		}
		else{
			LOG("[gnssV2] Sat : ");LOGLN(mySatellites.count());

		}
#endif


		GNSSLocation _location;
		//check the GNSS status
		GNSS.location(_location);

		if( (_location.fixType() == GNSSLocation::TYPE_2D) || (_location.fixType() == GNSSLocation::TYPE_3D) ){
			uint32_t _ehpe = _location.ehpe() * 1000;

			LOG("[gnssV2] Fixed with ehpe : ");LOGLN( (_ehpe + 500)/1000);
			if(_ehpe < pManager->lastEHPE){
				//new best value
				int32_t value;
				_location.latitude(value);
				pManager->latitude = value;

				_location.longitude(value);
				pManager->longitude = value;

				pManager->hdop = (uint32_t)(_location.hdop() * 1000);
				pManager->ehpe = _ehpe;
				pManager->lastEHPE = _ehpe;

				pManager->noFixCount = 0; //reset the noFixCount
				pManager->zeroSatTimeoutCount = 0;
				pManager->fixDelayLocked = 0;

				pManager->ttf = currentTime - pManager->startTime;

#ifdef GNSS_FIXDELAYLOCK
				if(_ehpe <= (FIXDELAYLOCK_EHPE * 1000)){
					// the fix is good enough
					pManager->lastFixTime = currentTime;
				}
#endif



				LOG("[gnssV2] Nes Best Value : lat=");LOG(pManager->latitude);LOG(",long=");LOGLN(pManager->longitude);
			}


			uint32_t ehpe_limit = EHPE_LIMIT(currentTime - pManager->startTime) * 1000;

			if(pManager->ehpe <= ehpe_limit){ //accept GNSS constraint
				LOG("[gnssV2] EHPE under limit ");LOGLN(pManager->ehpe);
				manager_gnssV2_setOFF(pManager);

			}
		}


		if(currentTime - pManager->startTime >= GNSS_TIMEOUT_S){
			LOGLN("[gnssV2] Timeout");
			manager_gnssV2_setOFF(pManager);
		}


	}


}



