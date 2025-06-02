/*
 * tagApplication.cpp
 *
 *  Created on: 22 janv. 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"
#include "Arduino.h"
#include "TimerMillis.h"
#include "LoRaWAN.h"
#include "RTC.h"
#include "STM32L0.h"
//#include "GNSS.h"

#include "../Manager/_manager.h"
#include "../LoraApplication/_LoraApplication.h"
#include "../Tools/shell.h"
#include "tagApplication.h"

//� remettre au propre
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

#define SURFACE_TRIGGER_SENSOR_THRESOLD 	0
#define SURFACE_TRIGGER_SENSOR_GRADIENT		1
#define SURFACE_TRIGGER_PRESSURE_1BA		2
#define SURFACE_TRIGGER_PRESSURE_30BA 		3


const char *const TriggerStr[4] = { "THR", "GRAD", "1BA", "30BA"};

const char* const AppModeStr[tagApp_size] = { "Boot", "Surface", "Subsurface", "surfacing", "dive","preDive"};

#define SERIAL_RESET_DELAY	(1800L)
static int16_t countTick = SERIAL_RESET_DELAY;

static uint8_t diveZone = 0;


static tagApp_mode_t applicationMode = tagApp_boot;

const uint16_t diveZoneLimis[DIVE_NB_ZONES - 1] = DIVE_ZONE_LIMITS_M;

data_histo_t mainMessage;

#ifdef ACTIVE_SURFACE_TRIG_DEBUG
data_SurfaceDebug_t surfaceDebugMessage;
#endif

manager_depthCalibration_t calibratedDepthHandle;
manager_gnss_t gnssManager;
manager_surfaceSensor_t surfaceSensorManager;
manager_diveProfile_t diveProfile;

// application process definition

void _process_boot(void);
void _process_surface(void);
void _process_subsurface(void);
void _process_surfacing(void);
void _process_dive(void);
void _process_preDive(void);

void _changeAppMode(tagApp_mode_t mode);

//application processes jump Table (constant array of function pointer)
void (* const appProcess[tagApp_size])(void) = {
		_process_boot, // [tagApp_unknow] = _process_unknow,
		_process_surface, //[tagApp_surface] = _process_surface,
		_process_subsurface, //[tagApp_subsurface] = _process_subsurface,
		_process_surfacing, //[tagApp_surfacing] = _process_surfacing,
		_process_dive, //[tagApp_dive] = _process_dive
		_process_preDive

};


// Event Iimer Main Application
TimerMillis appMainTimer;
const uint32_t appMainTimer_period = 1000;
void _appTickCallback(void);
volatile bool flagTickMain = false;

//Event Timer SurfaceSensor
TimerMillis surfaceTimer;
const uint32_t surfaceTimer_period = deltaT_surface_detect;
void _surfaceTickCallback(void);
volatile bool flagTickSurface = false;
volatile bool first_subsurface = true;

uint32_t tsSurfacingTime; //timestamp of the surfacing time
uint32_t tsBackToSurfaceTime; //timestamp when we go back to the surface
uint32_t tsSubsurfacingTime; //timestamp enter in subsurface
uint32_t tsDiveStartTime; //timestamp enter in subsurface
uint32_t tsNextGnssReload;



bool _checkSurfacingDeep(void){
	return false;

}
bool _checkSurfacingSensor(void){
	return true;
}


//////////// Implementation /////////////////

void _startDiving(void){
	// RTC.attachInterrupt(alarmMatch);

	_changeAppMode(tagApp_preDive);
	tsDiveStartTime = manager_appTick_get();

	mainMessage.surfaceTime_s = tsDiveStartTime - tsSurfacingTime;


	//stop the GPS
	manager_gnssV2_setOFF(&gnssManager);


	manager_depth_calibrate30BA();
	first_subsurface = true;
	diveZone = 0;

}



bool _stopDiving(uint8_t triggerId){
	int32_t depth;
	pressureSensor_result_t res;


	//res = manager_depth_get_mm(pressureSensor_1BA, &depth);
	depth = manager_depthCalibration_getCurrentCalibratedValue_mm(&calibratedDepthHandle);
	//depth = manager_depthCalibration_getCurrentValue_mm(&calibratedDepthHandle);
	uint32_t cTime = manager_appTick_get();

	//check for Surface trigger Error
	if((triggerId == SURFACE_TRIGGER_SENSOR_THRESOLD) || (triggerId == SURFACE_TRIGGER_SENSOR_GRADIENT)){

		//if( (res == psr_OVERLIMIT) || ( (res == psr_OK)&&(depth>(SURFACESENSOR_ACTIVATION_DEPTH_CM*10)) )){
		if( depth>(SURFACESENSOR_ACTIVATION_DEPTH_CM*10) ){
			LOG("Trigger Error ");LOGLN(TriggerStr[triggerId]);
			//if(triggerId == SURFACE_TRIGGER_SENSOR_THRESOLD) mainMessage.surfaceSensor_falseTHR_count++;
			//if(triggerId == SURFACE_TRIGGER_SENSOR_GRADIENT) mainMessage.surfaceSensor_falseGRAD_count++;

#ifdef ACTIVE_SURFACE_TRIG_DEBUG

			data_SurfaceDebug_trigDef_t trigData;
			trigData.trigSource = triggerId;
			trigData.trigTime = cTime - tsDiveStartTime;
			trigData.pressure1BA = depth;
			trigData.surfaceSensor_filter0 = manager_surfaceSensor_getFilter0();
			trigData.surfaceSensor_filterN = manager_surfaceSensor_getFilterN();

			message_surfaceTriggerDebug_buildFalseTrigMessage(&surfaceDebugMessage, &trigData);

#endif

			manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
			return false;
		}

	}
//	mainMessage.diveTime_s = cTime - tsDiveStartTime;

#ifdef ACTIVE_SURFACE_TRIG_DEBUG

	surfaceDebugMessage.currentDepth_mm = manager_depthCalibration_getCurrentValue_mm(&calibratedDepthHandle);
	surfaceDebugMessage.currentDepthCalibrated_mm = manager_depthCalibration_getCurrentCalibratedValue_mm(&calibratedDepthHandle);
	surfaceDebugMessage.minPressure_mPa = manager_depthCalibration_getMinPressure_mPa(&calibratedDepthHandle);
	surfaceDebugMessage.surfaceTriggerId = triggerId;
	surfaceDebugMessage.surfaceSensor_filter0 = manager_surfaceSensor_getFilter0();
	surfaceDebugMessage.surfaceSensor_filterN = manager_surfaceSensor_getFilterN();
	surfaceDebugMessage.surfaceSensor_lastChange = manager_surfaceSensor_getLastDive();


#endif

	LOG("Surface trigged by : "); LOGLN(TriggerStr[triggerId]);
	//surfaceTimer.stop();															//de-commenter ici pour arret du capteur surface des le premier surfacage

	float VDDA = STM32L0.getVDDA();
	float _vbat = 0;
	digitalWrite(myVBat_en, HIGH);
	//_vbat = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
	_vbat = 1.27f * VDDA * analogRead(myVBat) / 1024.0f;
	digitalWrite(myVBat_en, LOW);
	LOG("BAT = ");LOGLN(_vbat);

	mainMessage.battLevel_mV = (uint16_t)(_vbat * 1000);

	manager_diveProfile_result_t diveResut;
	manager_diveProfile_end(&diveProfile);
	manager_diveProfile_get(&diveProfile,&diveResut);

	for(uint8_t i =0;i< DIVEPROFILE_OUTPUTSIZE;i++)
		mainMessage.profile[i] = diveResut.profile[i];

	mainMessage.temperature = manager_depthCalibration_getTemperature(&calibratedDepthHandle);

	_changeAppMode(tagApp_surfacing);
	return true;
}


void _toSubSurface(void){
	_changeAppMode(tagApp_subsurface);
	tsSubsurfacingTime = manager_appTick_get();
	//mainMessage.pureSurfaceTime_s += tsSubsurfacingTime - tsBackToSurfaceTime;
}


void _firstSubSurface(uint8_t triggerId){

	//first stop the timer
	surfaceTimer.stop();
	manager_surfaceSensor_stop(&surfaceSensorManager);

#ifdef ACTIVE_SURFACE_TRIG_DEBUG
	surfaceDebugMessage.diveTriggerId = triggerId;
	surfaceDebugMessage.diveSensor_filter0 = manager_surfaceSensor_getFilter0();
	surfaceDebugMessage.diveSensor_filterN = manager_surfaceSensor_getFilterN();
	surfaceDebugMessage.diveSensor_lastChange = manager_surfaceSensor_getLastSurface();
#endif

	//LOGLN("STOP CAPTEUR SURFACE - prochains trigs surface/plongee avec 1BA");

	_toSubSurface();


	first_subsurface = false;

}



void _appTickCallback(void){

	//may be tick
	flagTickMain = true;
	STM32L0.wakeup();
}

void _surfaceTickCallback(void){

	flagTickSurface = true;
	STM32L0.wakeup();
}


void _changeAppMode(tagApp_mode_t mode){

	applicationMode = mode;
	LOG("Mode change to :");
	LOGLN(AppModeStr[mode]);
}



void _process_boot(void){

	/*while(LoRaWAN.busy()){
		LOGLN("Lora Busy");
		delay(500);
	}*/
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

	/*int8_t retrySetDR = 200;
	while( (LoRaWAN.setDataRate(DATARATE) == 0) && (retrySetDR > 0)){
		LOGLN("Retry Set Datarate");
		delay(100);
		retrySetDR--;
	}*/
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

	manager_gnssV2_setOFF(&gnssManager);

	//preload GNSS Data
	//LOGLN("BOOT ----  PRELOAD GPS DATA");
	//manager_gnssV2_setON(&gnssManager);
	//Start GNSS for 2 minutes
	/*LOG("120");
	for(uint8_t i = 0;i<120;i++){
		LOG("-");LOG(120-i);
		if(i%10 == 0) LOGLN("");
		delay(1000);
	}
	manager_gnssV2_process(&gnssManager);*/



	/*for(uint8_t i = 0;i<10;i++){
		//GNSS ON
		manager_gnssV2_setON(&gnssManager);

	//	while(gnssManager.state){
			manager_gnssV2_process(&gnssManager);
			delay(1000);
	//	}
	}
	manager_gnssV2_setOFF(&gnssManager);*/



	//init the surfacingTime
	tsSurfacingTime = manager_appTick_get();
	tsBackToSurfaceTime = tsSurfacingTime;
	tsNextGnssReload = tsSurfacingTime + GNSS_RELOAD_TIME_S;
	//go to surface
	_changeAppMode(tagApp_surface);

	//activate the surface Timer for the first dive
	surfaceTimer.restart( 0, surfaceTimer_period);
	manager_surfaceSensor_start(&surfaceSensorManager);

}

void _process_surface(void){
	LOGLN("--- SURFACE ----");

	uint32_t currentTime = manager_appTick_get();

	//check subsurface triggers
	int32_t depth_mm;
	depth_mm = manager_depthCalibration_getCurrentCalibratedValue_mm(&calibratedDepthHandle);

	if (first_subsurface) {
		if( (currentTime - tsSurfacingTime) > SURFACESENSOR_SURFACE_TIMEOUT ){
			surfaceTimer.stop();
			manager_surfaceSensor_stop(&surfaceSensorManager);
		}

		// security if the surface sensor doesn't trig
		if( depth_mm > (SUBSURFACE_ENTER_DEPTH_CM *10) ){
			LOGLN("First surface with 1BA");
			_firstSubSurface(SURFACE_TRIGGER_PRESSURE_1BA);
			//reset the surface sensor
			manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);



		}



		// premiere replongee detectee au capteur de surface

/*		if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
			//first stop the timer
			surfaceTimer.stop();

			_changeAppMode(tagApp_subsurface);
			tsSubsurfacingTime = manager_appTick_get();
			LOGLN("Premiere replongee: detection par capteur surface");
			mainMessage.pureSurfaceTime_s += tsSubsurfacingTime - tsBackToSurfaceTime;

#ifdef ACTIVE_SURFACE_TRIG_DEBUG
			surfaceDebugMessage.diveSensor_filter0 = manager_surfaceSensor_getFilter0();
			surfaceDebugMessage.diveSensor_filterN = manager_surfaceSensor_getFilterN();
			surfaceDebugMessage.diveSensor_lastChange = manager_surfaceSensor_getLastSurface();
#endif

			LOGLN("STOP CAPTEUR SURFACE - prochains trigs surface/plongee avec 1BA");

			first_subsurface = false;

			return;

		}*/

		//do nothing
	}
	else {

		// replongees suivantes detectees au 1BA (tant qu'on est pas repasse en dive)

		//if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){				//decom ici pour tt subsurface-surface au capteur surface (et commenter les 2lignes suivantes)
		//if(manager_depth_get_mm(pressureSensor_1BA,&depth_mm) == psr_OK){

		manager_depthCalibration_getCurrentValue_mm(&calibratedDepthHandle);
		if( depth_mm > (SUBSURFACE_ENTER_DEPTH_CM *10) ){//enter in subsurface
			_toSubSurface();
			LOGLN("detection subsurface par 1BA");
			return;
		}
		//}
	}

	//reload the GNSS each hour during prolonged surface
	if(currentTime > tsNextGnssReload){
		tsNextGnssReload += GNSS_RELOAD_TIME_S;
		//manager_gnssV2_setON(&gnssManager);
	}


}
void _process_subsurface(void) {
	LOGLN("--- SUBSURFACE ---");
	uint32_t currentTime = manager_appTick_get();
	//check the subsurfacing time
	if( (currentTime-tsSubsurfacingTime) > (SUBSURFACE_MAXTIME_S)  ){
		LOG("TimeOut");
		_startDiving();
		return;
	}

	int32_t depth;

	pressureSensor_result_t res;

	//res = manager_depth_get_mm(pressureSensor_1BA, &depth);
	manager_depthCalibration_getCurrentValue_mm(&calibratedDepthHandle);
	depth = manager_depthCalibration_getCurrentCalibratedValue_mm(&calibratedDepthHandle);
	

	//if(res == psr_OK){
		//surface detection only with the 1BA
	if(depth < (SURFACE_DEPTH_CM *10) ){
		//back to the surface
		LOGLN("back to surf");
//		mainMessage.backToSurfaceCount++;
		tsBackToSurfaceTime = currentTime;
		_changeAppMode(tagApp_surface);
		return;
	}



	/*else if(res == psr_OVERLIMIT){
		//active the 30BA sensor
		res = manager_depth_get_mm(pressureSensor_30BA, &depth);
	}*/

//	if(res == psr_OK){
		//at lesat one sensor
	if(depth > (SUBSURFACE_DEPTH_CM *10) ){
		_startDiving();
		return;
	}


	//nothing special to do in subsurface


}

void _process_surfacing(void){
	LOGLN("--- SURFACING ---");

	tsSurfacingTime = manager_appTick_get();
	tsBackToSurfaceTime = tsSurfacingTime;
	tsNextGnssReload = tsSurfacingTime + GNSS_RELOAD_TIME_S;

	//Collect GPS information of the last session before Start
	mainMessage.latitude = gnssManager.latitude;
	mainMessage.longitude = gnssManager.longitude;
	mainMessage.ehpe = gnssManager.ehpe;
	mainMessage.gnssNoFixCount = gnssManager.noFixCount;
	mainMessage.gnssZeroSatTimeout = gnssManager.zeroSatTimeoutCount;
	mainMessage.gnssUseTime = gnssManager.totalUseTime;
	mainMessage.ttf = gnssManager.ttf;

	mainMessage.gnssNbSat = gnssManager.nbSat;
	mainMessage.gnssNbSatPowered = gnssManager.nbSatPowered;


	mainMessage.surfaceSensorUseTime = surfaceSensorManager.useTime;


	//start the GPS
	//manager_gnssV2_setON(&gnssManager);

	//Send Lora Message
	if(LoRaWAN.busy()){
		LOGLN("Lora is busy");
	}
	else{

		if(!LoRaWAN.joined()){
			char _DevEUI[17];

			LoRaWAN.getDevEui(_DevEUI,17);
			LOGLN(_DevEUI);
			if(LoRaWAN.joinOTAA(APPEUI, APPKEY, _DevEUI)){

				uint8_t count = 100;

				while(count--){
					delay(100);
					if( !LoRaWAN.busy() && LoRaWAN.joined())
						break; //exit of the loop

				}
			}
		}

		//could not be joined
		if(LoRaWAN.joined()){

			LOGLN("join accepted");

			//send the main message
			LOG("Send Packet of ");LOG(sizeof(data_histo_t));LOGLN(" bytes");


			LoRaWAN.setDutyCycle(false);

			LoRaWAN.sendPacket(loraPortNumber[ilp_histo], (uint8_t*)&mainMessage, sizeof(data_histo_t));


#ifdef ACTIVE_SURFACE_TRIG_DEBUG
			//send the debug trame

			message_surfaceTriggerDebug_print(&surfaceDebugMessage);
			uint8_t count = 50;
			while( (count--) && LoRaWAN.busy() ){
				delay(500);
				LOGLN("Lora Busy");
			}


			LOG("Send Packet of ");LOG(sizeof(data_SurfaceDebug_t));LOGLN(" bytes");

			LoRaWAN.sendPacket(loraPortNumber[ilp_SurfaceDebug], (uint8_t*)&surfaceDebugMessage, sizeof(data_SurfaceDebug_t));
			message_surfaceTriggerDebug_clearMessage(&surfaceDebugMessage);
#endif


		}
		else
			LOGLN("join error");

	}

	//always change to surface mode
	message_histo_print(&mainMessage);
	message_histo_clear(&mainMessage);
	manager_diveProfile_start(&diveProfile);
	manager_depthCalibration_clear(&calibratedDepthHandle);
	_changeAppMode(tagApp_surface);

}

void _process_preDive(void){

	//manager_surfaceSensor_refreshWetReference();
	_changeAppMode(tagApp_dive);

}

void _process_dive(void){
	LOGLN("----- DIVE ------");

	//check surface triggers
	int32_t depth_mm;



	manager_depthCalibration_getCurrentValue_mm(&calibratedDepthHandle);
	depth_mm = manager_depthCalibration_getCurrentCalibratedValue_mm(&calibratedDepthHandle);

	if( depth_mm > (30 * 10) ){
		//stop the surface sensor
		if(surfaceSensorManager.state == true){

			surfaceTimer.stop();
			manager_surfaceSensor_stop(&surfaceSensorManager);
		}
	}


	if(depth_mm < (25 * 10)){
		//start the surface sensor
		if(surfaceSensorManager.state == false){
			manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
			surfaceTimer.restart( 0, surfaceTimer_period);
			manager_surfaceSensor_start(&surfaceSensorManager);
		}

	}


	if(depth_mm < (SURFACE_DEPTH_CM * 10)){
		//Surfacing  Now
		#ifdef TRIG_ON_1BA
		_stopDiving(SURFACE_TRIGGER_PRESSURE_1BA);

		return;
		#endif
	}

	diveZone = message_histo_getDiveZone(depth_mm/10);
	message_histo_addDepthPoint_diveZone(&mainMessage,diveZone);
	LOG("Zone : ");LOGLN(diveZone);

	if(depth_mm > 0)
		manager_diveProfile_addPoint(&diveProfile, depth_mm);
	else
		manager_diveProfile_addPoint(&diveProfile, 0);


}


void tagApplication_init(void){
	//init system time
	manager_rtc_setDefaultTime();
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	LOGINIT();
	applicationMode = tagApp_boot;

	delay(5000);
	digitalWrite(LED_BUILTIN, HIGH);
	for(uint8_t i=0;i<20;i++){
		delay(1000);
		LOG(".");
	}

	manager_depthCalibration_init(&calibratedDepthHandle);
	manager_surfaceSensor_initialize(&surfaceSensorManager);
	manager_appTick_init();


	LOGLN("");
	LOGLN("Staring app");

	if(!appMainTimer.start(_appTickCallback, 0, appMainTimer_period)){
		LOGLN("App Timer Error");
	}
	else
		LOGLN("App Timer Init");
	if(!surfaceTimer.start(_surfaceTickCallback, 0, surfaceTimer_period)){
		LOGLN("Surf Timer Error");
	}
	else{
		LOGLN("Surf Timer Init");
		surfaceTimer.stop();
	}

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
	manager_depth_init();

	message_histo_init(&mainMessage);
	manager_diveProfile_start(&diveProfile);
	manager_gnssV2_init(&gnssManager);



	//LoraWan init
	LoRaWAN.begin(EU868);
	LoRaWAN.setADR(false);
	LoRaWAN.setDataRate(0);//always join in SF12 first
	LoRaWAN.setTxPower(10);
	LoRaWAN.setSubBand(1);
	LoRaWAN.setDutyCycle(false);
	LoRaWAN.setSaveSession(true);


	pinMode(myVBat_en, OUTPUT);
	digitalWrite(myVBat_en, LOW); // start with battery voltage monirot off
	pinMode(myVBat, INPUT);
	//analogReadResolution(12);

#ifndef DISABLE_IWDG
	STM32L0.wdtEnable(18000);
#endif


	//manager_lowpower_needSleep();
	LOGLN("Init End");

}


void tagApplication_process(void){
	//(".");

	if(flagTickSurface){
		flagTickSurface = false;

		if(applicationMode == tagApp_dive){
			manager_surfaceSensor_process();

			//set flagTickMain to true to enable instant send
			if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)){
				//LOGLN("[DEBUG] TRIGGER THR ");
				if(_stopDiving(SURFACE_TRIGGER_SENSOR_THRESOLD) == true)
					flagTickMain = true;
			}else if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)){
				if(_stopDiving(SURFACE_TRIGGER_SENSOR_GRADIENT) == true)
					flagTickMain = true;
				//LOGLN("[DEBUG] TRIGGER GRAD ");
			}
		}

		if((applicationMode == tagApp_surface) && (first_subsurface)){
			manager_surfaceSensor_process();
			if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
				LOGLN("Premiere replongee: detection par capteur surface");
				_firstSubSurface(SURFACE_TRIGGER_SENSOR_THRESOLD);
				flagTickMain = true;
			}
		}
	}




	if(flagTickMain){
#ifndef DISABLE_IWDG
		STM32L0.wdtReset();
#endif
		flagTickMain = false;

		//LOGLN(countTick);
/*		if(countTick-- <0 ){
			LOGLN("RESET SERIAL");
			delay(100);
			Serial.end();
			delay(100);
			Serial.begin(9600);
			countTick = SERIAL_RESET_DELAY;
		}
*/


		/*if(STM32L0.getVBUS()==1){
			if(!Serial){
				Serial.begin(9600);
			}
		}
		else{
			if(Serial){
				Serial.end();
			}
		}*/



		manager_depthCalibration_process(&calibratedDepthHandle);

		appProcess[applicationMode]();

		//post process the GNSS state
		//manager_gnssV2_process(&gnssManager); // for to reprocess the state of the GNSS if busy when you try to put it on or off


		manager_appTick_tick();
	}



	//shell_receive();
	//LOG("LOCK STOP = ");LOGLN(stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_STOP]);
	manager_lowpower_manage();

}



