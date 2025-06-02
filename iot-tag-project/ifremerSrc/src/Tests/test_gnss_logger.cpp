/*
 * test_ping_GW_lora.cpp
 *
 *  Created on: 22 sept. 2020
 *      Author: Andrea
 */


#include "../_appDirectives.h"
#include "../Manager/_manager.h"
#include "../LoraApplication/_LoraApplication.h"

#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "STM32L0.h"

//const char DEVADDR[8]	= "5128695A";
//const char NWKSKEY[32] =   "8BC53FBA7DC260826A3BE36574AF36DC";
//const char APPSKEY[32]	= "4FBC853AD2E7AE38B548320AF261BC1E";

// gnss_tracker_0
//#define DEVADDR "1F3EB9C1"
//#define NWKSKEY "1474FDD4523EBCF47F143C43E1C0D2FE"
//#define APPSKEY "3227D1D324CF666822472DD30B5E76D3"

// gnss_tracker_1
//#define DEVADDR "1ff0fe9c"
//#define NWKSKEY "dff2eb8062fe3f5ab6058c657476b3d1"
//#define APPSKEY "51f15591ceaca3821252c3af62539683"

// gnss_tracker_2
#define DEVADDR "1f3eb9c2"
#define NWKSKEY "1474fdd4523ebcf47f143c43e1c0d2ff"
#define APPSKEY "ca644b5c0679cd8509b0e1d246b0dc23"


// gnss_solar_3_panneaux
//#define DEVADDR "1F3EB9C5"
//#define NWKSKEY "5474FDD4523EBCF47F143C43E1C0D2FF"
//#define APPSKEY "ca644b5c0679cd8509b0e1d246b0dc25"

// gnss_tracker_reunion
#define DEVADDR "00d00eda"
#define NWKSKEY "eb8b059ce907e7e1c284446973e8719e"
#define APPSKEY "b3d28f909e4abc8cae3d5513dddd6db7"

volatile bool flagGnssLoggerTick;


TimerMillis appGnssLoggerTimer;

void appTickGnssLogger(void){

	//may be tick
	flagGnssLoggerTick = true;
	STM32L0.wakeup();
}


void test_gnss_logger_launch(void){
	manager_gnss_t gnssManager;

	uint16_t currentTimeCount = 0;

	uint16_t time_relaunch_gnss = 10;  // seconds
	uint16_t time_gnss_scan     = 10;  // seconds

	manager_rtc_setDefaultTime();
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	LOGINIT();

	delay(5000);

	LOGLN("GNSS LOGGER Start");

	manager_gnssV2_init(&gnssManager);


	//LoraWan init
	LoRaWAN.begin(EU868);
	LoRaWAN.setADR(false);
	LoRaWAN.setDataRate(0);//always join in SF12 first
	LoRaWAN.setTxPower(10);
	LoRaWAN.setSubBand(1);
	LoRaWAN.setDutyCycle(false);
	LoRaWAN.setSaveSession(true);

	safeWhile(LoRaWAN.busy(), 50,LOGLN("Lora Busy");delay(1000));


	//try to join the lora server
	while(LoRaWAN.joined() == false){
		LOGLN("Try to join Lora Network");
		uint8_t count = 150;
		while(count--){
			if(!LoRaWAN.busy()) break;
			LOG("Lora Busy ");LOGLN(count);
			delay(5000);
		}

		char _DevEUI[17];

		LoRaWAN.getDevEui(_DevEUI,17);
		LOGLN(_DevEUI);
		if(LoRaWAN.joinABP(DEVADDR, NWKSKEY, APPSKEY)  == 0){   //LoRaWAN.joinOTAA(APPEUI, APPKEY, _DevEUI)
			LOGLN("Join func Error");
			delay(1000);
		}
		else{
			delay(6000);
			count = 15;
			while(count--){
				delay(1000);
				LOG("Check join : ");LOGLN(count);
				if(LoRaWAN.joined() == true) break;
			}
			LOGLN("Lora join Denied");
		}
	}

	safeWhile(LoRaWAN.setDataRate(DATARATE) == 0,20,LOGLN("Retry Set Datarate");delay(1000));

	if(LoRaWAN.joined()) {
		LOGLN("Lora joined");
		const char *strHello = "GNSS";
		LOGLN(strHello);
		LoRaWAN.sendPacket(loraPortNumber[ilp_StringMsg], (uint8_t*)strHello, strlen(strHello), false);



		safeWhile(LoRaWAN.busy(), 50,LOGLN("Lora Busy");delay(1000));

		//send the lora session keys
		LoRaWANSession sess;
		sess = LoRaWAN.getSession();

		LoRaWAN.sendPacket(loraPortNumber[ilp_keys], (uint8_t*)&sess.DevAddr, 36, false);

		/*LOG("DevAddr :");LOGLN(sess.DevAddr);

		const char hexStr[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
		LOG("NwkSKey :");
		for(uint8_t i=0; i<16;i++){
			LOG(hexStr[0x0F & (sess.NwkSKey[i]>>4)]);
			LOG(hexStr[0x0F & sess.NwkSKey[i]]);
		}
		LOGLN("");

		LOG("AppSKey :");
		for(uint8_t i=0; i<16;i++){
			LOG(hexStr[0x0F & (sess.AppSKey[i]>>4)]);
			LOG(hexStr[0x0F & sess.AppSKey[i]]);
		}*/
		LOGLN("");

	}



	//preload GNSS Data
	LOGLN(" ");
	LOGLN("BOOT ----  PRELOAD GPS DATA");
	manager_gnssV2_setON(&gnssManager);
	//Start GNSS for 1 minute
	LOG(time_gnss_scan);
	for(uint8_t i = 0;i<time_gnss_scan;i++){
		LOG("-");LOG(time_gnss_scan-i);
		if(i%10 == 0) LOGLN("");
		delay(1000);
	}
	manager_gnssV2_process(&gnssManager);


	appGnssLoggerTimer.start(appTickGnssLogger, 0, 1000); //tick 1s
	digitalWrite(LED_BUILTIN, LOW);
	manager_gnssV2_setON(&gnssManager);
	LOGLN("GNSS RESTART");
	
	while(true){

		if(flagGnssLoggerTick){
			flagGnssLoggerTick = false;

			//if(currentTimeCount>= timeSeries[timeSeriesIndex]){
			if((currentTimeCount>= time_gnss_scan)){



				LOGLN("GNSS STOP");
				digitalWrite(LED_BUILTIN, HIGH);
				LOGLN("Current position :");
				LOGLN("Latitude  : "); LOGLN(gnssManager.latitude);
				LOGLN("Longitude : "); LOGLN(gnssManager.longitude);
				LOGLN("EHPE :"); LOGLN((gnssManager.ehpe + 500)/1000);

				currentTimeCount = 0;


				data_gnssTrackingV2_t msg;

				msg.lat = gnssManager.latitude;
				msg.lng = gnssManager.longitude;
				msg.ehpe = gnssManager.ehpe;
				msg.hdop = gnssManager.hdop;
				msg.ttf = gnssManager.ttf;
				msg.nbSat = gnssManager.nbSat;
				msg.noFixCount = gnssManager.noFixCount;
				msg.zeroSatTimeout = gnssManager.zeroSatTimeoutCount;

				LoRaWAN.setDutyCycle(false);
				LoRaWAN.sendPacket(loraPortNumber[ilp_GnssTrackingV2], (uint8_t*)&msg, sizeof(data_gnssTrackingV2_t));
				// ilp_GnssTrackingV2
				// wait X minutes before GNSS relaunch
				//manager_gnssV2_setOFF(&gnssManager);
				LOG("WAIT "); LOG(time_relaunch_gnss); LOG(" S BEFORE RESTART ...");
				delay(time_relaunch_gnss*1000);
				
				digitalWrite(LED_BUILTIN, LOW);
				//manager_gnssV2_setON(&gnssManager);
				LOGLN("GNSS RESTART");




			}

			manager_gnssV2_process(&gnssManager);
			currentTimeCount++;

		}


		manager_lowpower_manage();

	}





}
