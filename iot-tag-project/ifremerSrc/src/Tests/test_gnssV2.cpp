/*
 * test_gnssV2.cpp
 *
 *  Created on: 21 juil. 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"
#include "../Manager/_manager.h"
#include "../LoraApplication/_LoraApplication.h"

#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "STM32L0.h"

volatile bool flagGnssTick;


TimerMillis appGnssTimer;

void appTickGnss(void){

	//may be tick
	flagGnssTick = true;
	STM32L0.wakeup();
}


void test_gnssV2_launch(void){
	manager_gnss_t gnssManager;
	static const uint16_t timeSeries[] = { 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 3600, 3600, 3600, 3600, 7200, 7200 };
	uint8_t timeSeriesSize = sizeof(timeSeries) / sizeof(uint16_t);
	uint8_t timeSeriesIndex = 0;
	uint16_t currentTimeCount = 0;

	manager_rtc_setDefaultTime();
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	LOGINIT();

	delay(5000);

	LOGLN("Test GNSS V2 Start");

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
		//safeWhile(LoRaWAN.busy(), 30,LOGLN("Lora Busy");delay(5000));
		/*while(LoRaWAN.busy()){
				LOGLN("Lora Busy");
				delay(500);
			}
		 */
		char _DevEUI[17];

		LoRaWAN.getDevEui(_DevEUI,17);
		LOGLN(_DevEUI);
		if( LoRaWAN.joinOTAA(APPEUI, APPKEY, _DevEUI) == 0){
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
		const char *strHello = "Hello";
		LOGLN(strHello);
		LoRaWAN.sendPacket(loraPortNumber[ilp_StringMsg], (uint8_t*)strHello, strlen(strHello), false);



		safeWhile(LoRaWAN.busy(), 50,LOGLN("Lora Busy");delay(1000));

		//send the lora session keys
		LoRaWANSession sess;
		sess = LoRaWAN.getSession();

		LoRaWAN.sendPacket(loraPortNumber[ilp_keys], (uint8_t*)&sess.DevAddr, 36, false);

		LOG("DevAddr :");LOGLN(sess.DevAddr);

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
		}
		LOGLN("");

	}

	for(uint8_t i=0; i<10; i++){
		LOGLN("Start gnss");
		manager_gnssV2_setON(&gnssManager);
		manager_gnssV2_process(&gnssManager);
		delay(60000);
		manager_gnssV2_process(&gnssManager);
		manager_gnssV2_setOFF(&gnssManager);
		LOGLN("Stop gnss");
		delay(1000);
		manager_gnssV2_process(&gnssManager);

	}


	appGnssTimer.start(appTickGnss, 0, 1000); //tick 1s

	while(true){

		if(flagGnssTick){
			flagGnssTick = false;

			//if(currentTimeCount>= timeSeries[timeSeriesIndex]){
			if(currentTimeCount>= (60)){
				LOGLN("End of Series");
				currentTimeCount = 0;

				timeSeriesIndex++;
				if(timeSeriesIndex >= timeSeriesSize) timeSeriesIndex = 0;


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

				manager_gnssV2_setON(&gnssManager);




			}

			manager_gnssV2_process(&gnssManager);
			currentTimeCount++;

		}


		manager_lowpower_manage();

	}





}
