/*
 * test_surfaceSensor.cpp
 *
 *  Created on: 06 mar. 2020
 *      Author: agoharza
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
#include "test_surfaceSensor.h"

//à remettre au propre
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

#define SURFACE_TRIGGER_SENSOR_THRESOLD 	0
#define SURFACE_TRIGGER_SENSOR_GRADIENT		1
#define SURFACE_TRIGGER_PRESSURE_1BA		2
#define SURFACE_TRIGGER_PRESSURE_30BA 		3


const char *const test_ss_TriggerStr[4] = { "THR", "GRAD", "1BA", "30BA"};

const char* const test_ss_AppModeStr[tagApp_size] = { "Boot", "Surface", "Subsurface", "surfacing", "dive"};

#define SERIAL_RESET_DELAY	(1800L)
static int16_t countTick = SERIAL_RESET_DELAY;

static uint8_t diveZone = 0;


static tagApp_mode_t applicationMode = tagApp_boot;

const uint16_t diveZoneLimis[DIVE_NB_ZONES - 1] = DIVE_ZONE_LIMITS_M;

data_histo_t test_ss_mainMessage;

bool test_ss_gnss_in_use = false;

bool test_ss_first_subsurface = true;

void _test_ss_process_boot(void);
void _test_ss_process_surface(void);
void _test_ss_process_subsurface(void);
void _test_ss_process_surfacing(void);
void _test_ss_process_dive(void);

void _test_ss_changeAppMode(tagApp_mode_t mode);


TimerMillis test_ss_appMainTimer;
const uint32_t appMainTimer_period = 1000;
void _test_ss_appTickCallback(void);
volatile bool test_ss_flagTickMain = false;

TimerMillis test_ss_surfaceTimer;
const uint32_t test_ss_surfaceTimer_period = deltaT_surface_detect;
void _test_ss_surfaceTickCallback(void);
volatile bool test_ss_flagTickSurface = false;

uint32_t test_ss_tsSurfacingTime; //timestamp of the surfacing time
uint32_t test_ss_tsBackToSurfaceTime; //timestamp when we go back to the surface
uint32_t test_ss_tsSubsurfacingTime; //timestamp enter in subsurface
uint32_t test_ss_tsDiveStartTime; //timestamp enter in subsurface

static float lastEHPE = 10000;

//application processes jump Table (constant array of function pointer)
void (* const test_ss_appProcess[tagApp_size])(void) = {
	_test_ss_process_boot, // [tagApp_unknow] = _process_unknow,
	_test_ss_process_surface, //[tagApp_surface] = _process_surface,
	_test_ss_process_subsurface, //[tagApp_subsurface] = _process_subsurface,
	_test_ss_process_surfacing, //[tagApp_surfacing] = _process_surfacing,
	_test_ss_process_dive //[tagApp_dive] = _process_dive


};



bool _test_ss_checkSurfacingDeep(void){
	return false;

}
bool _test_ss_checkSurfacingSensor(void){
	return true;
}


//////////// Implementation /////////////////

void _test_ss_startDiving(void){
	 // RTC.attachInterrupt(alarmMatch);
	//digitalWrite(LED_BUILTIN, HIGH);						// De-commenter ici pour extinction LED en dive

	_test_ss_changeAppMode(tagApp_dive);
	test_ss_tsDiveStartTime = RTC.getEpoch();


	//stop the GPS
	if(test_ss_gnss_in_use){
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF) test_ss_gnss_in_use = false;
		else
			LOGLN("STOP GPS - delayed");

		//if(GNSS.suspend()) test_ss_gnss_in_use = false;
	}


	manager_depth_calibrate30BA();

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);												//decom ici pour ss au 1ba
	test_ss_surfaceTimer.restart( 0, test_ss_surfaceTimer_period);
	LOGLN("RESTART CAPTEUR SURFACE");

	diveZone = 0;

	test_ss_first_subsurface = true;

}

bool _test_ss_stopDiving(uint8_t triggerId){
	int32_t depth;
	pressureSensor_result_t res;

	res = manager_depth_get_mm(pressureSensor_1BA, &depth);
	//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);

	//check for Surface trigger Error
	/*if((triggerId == SURFACE_TRIGGER_SENSOR_THRESOLD) || (triggerId == SURFACE_TRIGGER_SENSOR_GRADIENT)){

		if( (res == psr_OVERLIMIT) || ( (res == psr_OK)&&(depth>(SURFACESENSOR_ACTIVATION_DEPTH_CM*10)) )){
			LOG("Trigger Error ");LOGLN(test_ss_TriggerStr[triggerId]);
			//if(triggerId == SURFACE_TRIGGER_SENSOR_THRESOLD) test_ss_mainMessage.surfaceSensor_falseTHR_count++;
			//if(triggerId == SURFACE_TRIGGER_SENSOR_GRADIENT) test_ss_mainMessage.surfaceSensor_falseGRAD_count++;
			manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
			return false;
		}

	}*/

/*	test_ss_mainMessage.diveTime_s = RTC.getEpoch() - tsDiveStartTime;
	test_ss_mainMessage.surfaceTriggerId = triggerId;
	test_ss_mainMessage.pressure1BA = depth;
	test_ss_mainMessage.surfaceSensor_filter0 = manager_surfaceSensor_getFilter0();
	test_ss_mainMessage.surfaceSensor_filterN = manager_surfaceSensor_getFilterN();
	test_ss_mainMessage.surfaceSensor_lastChange = manager_surfaceSensor_getLastChange();*/



	LOG("Surface trigged by : "); LOGLN(test_ss_TriggerStr[triggerId]);
	//test_ss_surfaceTimer.stop();											// CA a enlever pour ne pas desactiver a la surface

	float VDDA = STM32L0.getVDDA();
	float _vbat = 0;
	digitalWrite(myVBat_en, HIGH);
	//_vbat = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
	_vbat = 1.27f * VDDA * analogRead(myVBat) / 1024.0f;
	digitalWrite(myVBat_en, LOW);
	LOG("BAT = ");LOGLN(_vbat);

	test_ss_mainMessage.battLevel_mV = (uint16_t)(_vbat * 1000);

	_test_ss_changeAppMode(tagApp_surfacing);
	return true;
}


void _test_ss_appTickCallback(void){

	//may be tick
	test_ss_flagTickMain = true;
	STM32L0.wakeup();
}

void _test_ss_surfaceTickCallback(void){

	test_ss_flagTickSurface = true;
	STM32L0.wakeup();
}


void _test_ss_changeAppMode(tagApp_mode_t mode){

	applicationMode = mode;
	LOG("Mode change to :");
	LOGLN(test_ss_AppModeStr[mode]);
}



void _test_ss_process_boot(void){

	//init the surfacingTime
	test_ss_tsSurfacingTime = RTC.getEpoch();
	test_ss_tsBackToSurfaceTime = test_ss_tsSurfacingTime;
	//go to surface
	_test_ss_changeAppMode(tagApp_surface);


}

void _test_ss_process_surface(void){
	LOGLN("--- SURFACE ----");

	//check subsurface triggers  ----------------------------------------- Ajouter un booleen juste pour le premier passage en subsurface (on le remet a false en dive)
	int32_t depth_mm;

	if (test_ss_first_subsurface) {

		//LOGLN("profondeur:");
		//LOGLN(manager_depth_get_mm(pressureSensor_1BA,&depth_mm));
		if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
				_test_ss_changeAppMode(tagApp_subsurface);
				test_ss_tsSubsurfacingTime = RTC.getEpoch();
				LOGLN("Premiere replongee: detection par capteur surface");
				//test_ss_mainMessage.pureSurfaceTime_s += test_ss_tsSubsurfacingTime - test_ss_tsBackToSurfaceTime;

				test_ss_surfaceTimer.stop();																				//decom ici pour ss au 1ba
				LOGLN("STOP CAPTEUR SURFACE - prochains trigs surface/plongee avec 1BA");

				//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
				//test_ss_surfaceTimer.restart( 0, test_ss_surfaceTimer_period);

				test_ss_first_subsurface = false;
				//digitalWrite(LED_BUILTIN, HIGH);									// De-commenter ici pour extinction LED en subsurface

				return;

			}
	}
	else {

		//if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
		if(manager_depth_get_mm(pressureSensor_1BA,&depth_mm) == psr_OK){												//decom ici pour ss au 1ba
			if( depth_mm > (SURFACE_DEPTH_CM *10) ){//enter in subsurface												//decom ici pour ss au 1ba
				_test_ss_changeAppMode(tagApp_subsurface);
				test_ss_tsSubsurfacingTime = RTC.getEpoch();
				LOGLN("detection subsurface par 1BA");
				//LOGLN("detection subsurface par capteur surface");
				//test_ss_mainMessage.pureSurfaceTime_s += test_ss_tsSubsurfacingTime - test_ss_tsBackToSurfaceTime;

				//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);

				//digitalWrite(LED_BUILTIN, HIGH);							// De-commenter ici pour extinction LED en subsurface

				return;
			}																													//decom ici pour ss au 1ba
		}
	}

	if(manager_gnss_getState() == GNSS_ON){
		GNSSLocation _location;
		//check the GNSS status
		GNSS.location(_location);

		manager_gnss_logSattelites();
		static const char *fixTypeString[] = {
		            "NONE",
		            "TIME",
		            "2D",
		            "3D",
		};

		LOGLN(fixTypeString[_location.fixType()]);

		if( (_location.fixType() == GNSSLocation::TYPE_2D) || (_location.fixType() == GNSSLocation::TYPE_3D) ){
			//GPS is Fix
			float_t _ehpe = _location.ehpe();
			LOG("EHPE : ");LOGLN(_ehpe);

			if(_ehpe < lastEHPE){
				//store the GPS value
				int32_t value;
				_location.latitude(value);
				test_ss_mainMessage.latitude = value;
				LOG("Lat : ");LOG(value);

				_location.longitude(value);
				test_ss_mainMessage.longitude = value;
				LOG(" ,Long : ");LOG(value);

				test_ss_mainMessage.ehpe = (uint32_t)(_ehpe*1000);

				LOGLN("");
				lastEHPE = _ehpe;
			}


			if(_ehpe < GNSS_EHPE_OK ){
				LOGLN("Good GNSS Fix");
				test_ss_mainMessage.ttf = RTC.getEpoch() - test_ss_tsSurfacingTime;
				manager_gnss_setState(GNSS_OFF);
				if(manager_gnss_getState() == GNSS_OFF) test_ss_gnss_in_use = false;
				else LOGLN("Stop GPS - delayed");

				//if(GNSS.suspend()) test_ss_gnss_in_use = false;

				LOG("ttf = ");LOG(test_ss_mainMessage.ttf);LOGLN(" s");
			}


		}//no fix

		uint32_t currentT = RTC.getEpoch();
		if( (RTC.getEpoch() - test_ss_tsSurfacingTime) > GNSS_MAX_ACTIVE_TIME_S){
			//stop gnss with no position
			LOGLN("GNSS time over");
			LOGLN(currentT);
			LOGLN(test_ss_tsSurfacingTime);
			/*test_ss_mainMessage.latitude = GNSS_UNDEFINED_VALUE;
			test_ss_mainMessage.longitude  = GNSS_UNDEFINED_VALUE;
			test_ss_mainMessage.ehpe = GNSS_UNDEFINED_VALUE;
			 */
			manager_gnss_setState(GNSS_OFF);
			if(manager_gnss_getState() == GNSS_OFF) test_ss_gnss_in_use = false;
			else LOGLN("Stop GPS - delayed");

			//if(GNSS.suspend()) test_ss_gnss_in_use = false;
		}

	}//if(test_ss_gnss_in_use)


}

void _test_ss_process_subsurface(void) {
	LOGLN("--- SUBSURFACE ---");
	uint32_t currentTime = RTC.getEpoch();

	//check the subsurfacing time
	/*if( (currentTime-tsSubsurfacingTime) > (SUBSURFACE_MAXTIME_S)  ){
		LOG("TimeOut");
		_startDiving();
		return;
	}*/

	int32_t depth;

	pressureSensor_result_t res;


	res = manager_depth_get_mm(pressureSensor_1BA, &depth);

	// Subsurface: on detecte les re-surfacages au 1BA (capteur surface desactive apres premier passage surface -> subsurface)

	//if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)){
	if(res == psr_OK){																						//decom ici pour ss au 1ba
		//surface detection only with the 1BA
		if(depth < (SURFACE_DEPTH_CM *10) ){
			//back to the surface
			//test_ss_mainMessage.backToSurfaceCount++;
			test_ss_tsBackToSurfaceTime = currentTime;
			_test_ss_changeAppMode(tagApp_surface);
			//digitalWrite(LED_BUILTIN, LOW);							// De-commenter ici pour allumage LED pour detection subsurface -> surface au 1BA
			return;
		}
	}
	if(res == psr_OVERLIMIT){
		//active the 30BA sensor
		res = manager_depth_get_mm(pressureSensor_30BA, &depth);
	}

	//if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
	if(res == psr_OK){																					//decom ici pour ss au 1ba
		//at least one sensor
		if(depth > (SUBSURFACE_DEPTH_CM *10) ){
			_test_ss_startDiving();
			return;
		}
	}

	//nothing special to do in subsurface


}

void _test_ss_process_surfacing(void){
	LOGLN("--- SURFACING ---");
	//digitalWrite(LED_BUILTIN, LOW);
	//LOGLN("Surfacing");
	//build the message

	//start the GPS
	if(!test_ss_gnss_in_use){

		//if(GNSS.resume()){
		manager_gnss_setState(GNSS_ON);
		//reset the state even if the GPS et not working
		lastEHPE = 10000;
		test_ss_tsSurfacingTime = RTC.getEpoch();
		test_ss_tsBackToSurfaceTime = test_ss_tsSurfacingTime;

		if(manager_gnss_getState()){
			test_ss_gnss_in_use = true;
			LOGLN("Start GPS");
		}
		else
			LOGLN("Start GPS - delayed");
	}


	//always change to surface mode
	_test_ss_changeAppMode(tagApp_surface);

}

void _test_ss_process_dive(void){
	LOGLN("----- DIVE ------");

	//check surface triggers
	int32_t depth_mm;

	pressureSensor_result_t res = manager_depth_get_mm(pressureSensor_1BA, &depth_mm) ;


	// TRIG SURFACE UNIQUEMENT AU CAPTEUR SURFACE

	if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)){
		_test_ss_stopDiving(SURFACE_TRIGGER_SENSOR_THRESOLD);
		//digitalWrite(LED_BUILTIN, LOW);
		return;

	}

	if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)){
		_test_ss_stopDiving(SURFACE_TRIGGER_SENSOR_GRADIENT);
		//digitalWrite(LED_BUILTIN, LOW);
		return;

	}



	/*if(res == psr_OK ){
		//surface detection is only done by 1BA
		if(depth_mm < (SURFACE_DEPTH_CM * 10)){
			//Surfacing  Now
			#ifdef TRIG_ON_1BA
			_stopDiving(SURFACE_TRIGGER_PRESSURE_1BA);

			return;
			#endif
		}
	}
	else if(res == psr_OVERLIMIT){
		//active the 30BA
		res = manager_depth_get_mm(pressureSensor_30BA, &depth_mm);

	}*/

	//Execute dive mode
/*	if(res == psr_OK){
		//at least one of the sensor
		//message_histo_addDepthPoint_cm(&test_ss_mainMessage, depth_mm /10);
		diveZone = message_histo_getDiveZone(depth_mm/10);
		message_histo_addDepthPoint_diveZone(&test_ss_mainMessage,diveZone);
		LOG("Zone : ");LOGLN(diveZone);
	}*/


}


void test_ss_tagApplication_init(void){
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

	LOGLN("");
	LOGLN("Staring app");

	if(!test_ss_appMainTimer.start(_test_ss_appTickCallback, 0, appMainTimer_period)){
		LOGLN("App Timer Error");
	}
	else
		LOGLN("App Timer Init");
	if(!test_ss_surfaceTimer.start(_test_ss_surfaceTickCallback, 0, test_ss_surfaceTimer_period)){
		LOGLN("Surf Timer Error");
	}
	else{
		LOGLN("Surf Timer Init");
		//test_ss_surfaceTimer.stop();
	}

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
	manager_depth_init();

	manager_gnss_init();

    test_ss_gnss_in_use = false;


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


void test_ss_tagApplication_process(void){
	//(".");
	// manager_surfaceSensor_process();

	if(test_ss_flagTickSurface){
		test_ss_flagTickSurface = false;

		if(applicationMode == tagApp_dive){
			manager_surfaceSensor_process();

			//set flagTickMain to true to enable instant send
			if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)){
				//LOGLN("[DEBUG] TRIGGER THR ");
				if(_test_ss_stopDiving(SURFACE_TRIGGER_SENSOR_THRESOLD) == true)
					test_ss_flagTickMain = true;
			}else if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)){
				if(_test_ss_stopDiving(SURFACE_TRIGGER_SENSOR_GRADIENT) == true)
					test_ss_flagTickMain = true;
				//LOGLN("[DEBUG] TRIGGER GRAD ");
			}
		}

	}

	if((applicationMode == tagApp_surface) && (test_ss_first_subsurface)){
		manager_surfaceSensor_process();
	}


	if(test_ss_flagTickMain){
		#ifndef DISABLE_IWDG
		STM32L0.wdtReset();
		#endif
		test_ss_flagTickMain = false;

		LOGLN(countTick);
		if(countTick-- <0 ){
			LOGLN("RESET SERIAL");
			delay(100);
			Serial.end();
			delay(100);
			Serial.begin(9600);
			countTick = SERIAL_RESET_DELAY;
		}



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

		test_ss_appProcess[applicationMode]();

	}



	manager_gnss_processState(); // for to reprocess the state of the GNSS if busy when you try to put it on or off
	shell_receive();
	//LOG("LOCK STOP = ");LOGLN(stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_STOP]);
	manager_lowpower_manage();

}

void test_surfaceSensor(void){
	test_ss_tagApplication_init();
	while(1) {
		test_ss_tagApplication_process();
	}
}
