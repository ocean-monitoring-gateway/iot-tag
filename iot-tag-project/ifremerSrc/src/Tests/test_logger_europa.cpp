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


volatile bool flagLoggerEuropaTick;

data_histo_t msg;

TimerMillis appLoggerEuropaTimer;

void appTickLoggerEuropa(void){

	//may be tick
	flagLoggerEuropaTick = true;
	STM32L0.wakeup();
}


void test_logger_europa_launch(void){

	uint16_t currentTimeCount = 0;

	uint16_t time_relaunch_tag = 30;  // seconds

	manager_rtc_setDefaultTime();
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	#define myVBat_en  2 // enable VBat read
	#define myVBat    A1 // VBat analog read pin

	pinMode(myVBat_en, OUTPUT);
	digitalWrite(myVBat_en, LOW); // start with battery voltage monirot off
	pinMode(myVBat, INPUT);

	LOGINIT();

	delay(5000);

	LOGLN("Europa Logger Start");



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


	message_histo_init(&msg);

	LOGLN(" ");
	LOGLN("LOGGER START ----  ");



	appLoggerEuropaTimer.start(appTickLoggerEuropa, 0, 1000); //tick 1s

	while(true){

		if(flagLoggerEuropaTick){
			flagLoggerEuropaTick = false;

			//if(currentTimeCount>= timeSeries[timeSeriesIndex]){
			if((currentTimeCount>= time_relaunch_tag)){


				float VDDA = STM32L0.getVDDA();
				float _vbat = 0;
				digitalWrite(myVBat_en, HIGH);
				_vbat = 1.27f * VDDA * analogRead(myVBat) / 1024.0f;
				digitalWrite(myVBat_en, LOW);
				LOG("BAT = ");LOGLN(_vbat);

				msg.battLevel_mV = (uint16_t)(_vbat * 1000);

				for(uint8_t i =0;i< DIVEPROFILE_OUTPUTSIZE;i++)
						msg.profile[i] = 0;

				msg.temperature = 0; // manager_depthCalibration_getTemperature(&calibratedDepthHandle);
				msg.latitude = 0;
				msg.longitude = 0;
				msg.ehpe = 0;
				msg.gnssNoFixCount = 0;
				msg.gnssZeroSatTimeout = 0;
				msg.gnssUseTime = 0;
				msg.ttf = 0;

				msg.gnssNbSat = 0;
				msg.gnssNbSatPowered = 0;


				msg.surfaceSensorUseTime = 0;



				currentTimeCount = 0;


				LoRaWAN.setDutyCycle(false);
				LoRaWAN.sendPacket(loraPortNumber[ilp_histo], (uint8_t*)&msg, sizeof(data_histo_t));

				message_histo_clear(&msg);




			}

			currentTimeCount++;

		}


		manager_lowpower_manage();

	}





}
