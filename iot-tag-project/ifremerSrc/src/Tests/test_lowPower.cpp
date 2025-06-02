/*
 * test_lowPower.cpp
 *
 *  Created on: 19 févr. 2020
 *      Author: jfezande
 */


/*
 * Séquence de test de l'ensemble des capteurs
 *
 * Chaque test débute avec un code. Le code est transmis en activant la LED
 * On envoi le code 0xA5 suivi du code de la séquence
 *
 * Nous utilisons le mode STOP pour tous les tests puis un standby pour terminer
 *
 *
 */


#include "Arduino.h"
#include "TimerMillis.h"
#include "STM32L0.h"
#include "LoRaWAN.h"
#include "SPIFlash.h"

#include "../_board.h"
#include "../_appDirectives.h"
#include "../Manager/_manager.h"

#include "test_lowPower.h"

#define SEQ_NONE				0x00
#define SEQ_PRESSURE			0x01
#define SEQ_SURFACE				0x02
#define SEQ_LORAJOIN			0x03
#define SEQ_LORASEND			0x04
#define SEQ_GNSS_FIRSTFIX		0x05
#define SEQ_GNSS_REFIX			0x06
#define SEQ_GNSS_EN_OFF			0x07
#define SEQ_GNSS_BACK_OFF		0x08




typedef enum {
	state_Wait,
	state_LedCode,
	state_Sequence

}testState_t;

uint8_t currentSequence = SEQ_NONE;
testState_t state = state_Wait;
uint16_t sequenceState = 0;

volatile bool tickFlagLp = false;
extern SPIFlash SPIFlash;

TimerMillis _localTimer;

void _onTick(void){
	tickFlagLp = true;
	STM32L0.wakeup();
}

static uint8_t currentBit = 0;

void LedCode(uint8_t code){
	const uint8_t headerByte = 0xA5;

	uint8_t value;

	if(currentBit < 8){
		//on envoie le header
		if(currentBit == 0){
			//on démarre le timer
			_localTimer.restart(500,500);

		}
		value = (headerByte>>(7-currentBit)) & 0x01;
	}
	else{
		if(currentBit == 16){ //fin de l'envoi du header
			currentBit = 0;
			digitalWrite(LED_BUILTIN,HIGH);
			state = state_Sequence;
			LOGLN("");
			return;
		}
		value = (code>>(8+7-currentBit)) & 0x01;
	}

	LOG(value);
	if(value == 1){
		digitalWrite(LED_BUILTIN,LOW);
	}
	else{
		digitalWrite(LED_BUILTIN,HIGH);
	}
	currentBit++;
}

void SequenceEnd(void){
	LOG("End seq : ");LOGLN(currentSequence);
	currentSequence++;
	sequenceState = 0;
	state = state_Wait;
	_localTimer.restart(1000,1000);

}


void Seq_LoraJoin(void){

	switch(sequenceState){
	case 0:

		_localTimer.restart(500,500);
		LoRaWAN.begin(EU868);
		LoRaWAN.setADR(false);
		LoRaWAN.setDataRate(0);//always join in SF12 first
		LoRaWAN.setTxPower(10);
		LoRaWAN.setSubBand(1);
		LoRaWAN.setDutyCycle(false);
		sequenceState++;
	case 1:
		char _DevEUI[17];
		LoRaWAN.getDevEui(_DevEUI,17);
		LOGLN(_DevEUI);
 		LoRaWAN.joinOTAA(APPEUI, APPKEY, _DevEUI );
		sequenceState++;
 		break;
	case 2:
		if(LoRaWAN.joined()){
			sequenceState++;
		}
		else{
			if(!LoRaWAN.busy()){
				sequenceState = 1; //retry join
			}
		}
		break;
	case 3 :
		SequenceEnd();
		break;

	}

}

void Seq_Pressure(void){
	int32_t value;

	if(sequenceState == 0){
		manager_depth_init();
		_localTimer.restart(1000,1000);
	}

	if(sequenceState > 60){
		SequenceEnd();
		return;
	}

	uint32_t timingSurface = armv6m_systick_micros();

	manager_depth_readPressure_mPa_1BA(&value);
	manager_depth_readPressure_mPa_30BA(&value);

	timingSurface = armv6m_systick_micros() - timingSurface;
	LOG("Press Process Time = ");LOG(timingSurface);LOGLN("us");

	sequenceState++;
	/*switch(sequenceState){
		case 0:
			manager_depth_init();
			_localTimer.restart(500,500);
			sequenceState++;
			break;
		case 1:
			manager_depth_readPressure_mPa_1BA(&value);
			sequenceState++;
			break;
		case 2:
			manager_depth_readPressure_mPa_30BA(&value);
			sequenceState++;
			break;
		case 3:
			SequenceEnd();
			break;
	}*/

}

void Seq_LoraSend(void){


	const char *Text = "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF"; //64 character
	LoRaWAN.setDataRate(DATARATE);//send SF9

	LoRaWAN.sendPacket(11,(uint8_t*)Text,64,false);
	SequenceEnd();

}

void Seq_SurfaceSensor(void){
	if(sequenceState == 0){
		manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
		_localTimer.restart(10,10);
	}else{
		if(sequenceState > 2000){ //20s
			SequenceEnd();
			_localTimer.restart(500,500);
			return;
		}
		uint8_t result;
		uint32_t timingSurface = armv6m_systick_micros();
		manager_surfaceSensor_process();
		result = manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold);
		result = manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient);

		timingSurface = armv6m_systick_micros() - timingSurface;
		LOG("Surf Process Time = ");LOG(timingSurface);LOGLN("us");

	}

	sequenceState++;
}

void Seq_Gnss_FirstFix(void){
	GNSSLocation _location;
	//check the GNSS status

	//test the fix of the GNSS
	//suspend
	//wait for 10 seconds
	//and refix
	//then GNSS_en = 0 && BACKUP = 1 for 10 seconds
	//then restart And fix
	//then GNSS_en = 0 && BACKUP = 0 for 10 seconds

	switch(sequenceState){
	case 0:
		_localTimer.restart(500,500);
		manager_gnss_init();
		sequenceState++;
		break;
	case 1:
		manager_gnss_setState(GNSS_ON);
		sequenceState++;
		break;
	case 2:
		if(manager_gnss_getState() == GNSS_ON) sequenceState++;
		else manager_gnss_processState();
		break;
	case 3:
		GNSS.location(_location);
		if( (_location.fixType() == GNSSLocation::TYPE_2D) || (_location.fixType() == GNSSLocation::TYPE_3D) ){
			float_t _ehpe = _location.ehpe();
			LOG("EHPE : ");LOGLN(_ehpe);
			if(_ehpe < GNSS_EHPE_OK ){
				sequenceState++;
			}
		}
		break;
	case 4:
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF){
			_localTimer.restart(10000,10000);
			sequenceState++;
		}
		else manager_gnss_processState();
		break;
	case 5 :
		SequenceEnd();
		break;
	}


}

void Seq_Gnss_ReFix(void){
	GNSSLocation _location;
	//check the GNSS status

	//test the fix of the GNSS
	//suspend
	//wait for 10 seconds
	//and refix
	//then GNSS_en = 0 && BACKUP = 1 for 10 seconds
	//then restart And fix
	//then GNSS_en = 0 && BACKUP = 0 for 10 seconds

	switch(sequenceState){
	case 0:
		_localTimer.restart(500,500);
		//manager_gnss_init();
		sequenceState++;
		break;
	case 1:
		manager_gnss_setState(GNSS_ON);
		sequenceState++;
		break;
	case 2:
		if(manager_gnss_getState() == GNSS_ON) sequenceState++;
		else manager_gnss_processState();
		break;
	case 3:
		GNSS.location(_location);
		if( (_location.fixType() == GNSSLocation::TYPE_2D) || (_location.fixType() == GNSSLocation::TYPE_3D) ){
			float_t _ehpe = _location.ehpe();
			LOG("EHPE : ");LOGLN(_ehpe);
			if(_ehpe < GNSS_EHPE_OK ){
				sequenceState++;
			}
		}
		break;
	case 4:
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF){
			manager_gnss_Vdd_En(false);
			_localTimer.restart(10000,10000);
			sequenceState++;
		}
		else manager_gnss_processState();
		break;
	case 5 :
		SequenceEnd();
		break;
	}


}

void Seq_Gnss_EnOFF(void){
	GNSSLocation _location;
	//check the GNSS status

	//test the fix of the GNSS
	//suspend
	//wait for 10 seconds
	//and refix
	//then GNSS_en = 0 && BACKUP = 1 for 10 seconds
	//then restart And fix
	//then GNSS_en = 0 && BACKUP = 0 for 10 seconds

	switch(sequenceState){
	case 0:
		_localTimer.restart(500,500);
		//manager_gnss_init();
		manager_gnss_Vdd_En(true);
		sequenceState++;
		break;
	case 1:
		manager_gnss_setState(GNSS_ON);
		sequenceState++;
		break;
	case 2:
		if(manager_gnss_getState() == GNSS_ON) sequenceState++;
		else manager_gnss_processState();
		break;
	case 3:
		GNSS.location(_location);
		if( (_location.fixType() == GNSSLocation::TYPE_2D) || (_location.fixType() == GNSSLocation::TYPE_3D) ){
			float_t _ehpe = _location.ehpe();
			LOG("EHPE : ");LOGLN(_ehpe);
			if(_ehpe < GNSS_EHPE_OK ){
				sequenceState++;
			}
		}
		break;
	case 4:
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF){
			manager_gnss_Vdd_En(false);
			manager_gnss_Backup_En(false);
			_localTimer.restart(10000,10000);
			sequenceState++;
		}
		else manager_gnss_processState();
		break;
	case 5 :
		SequenceEnd();
		break;
	}

}

void Seq_Gnss_BackOFF(void){
	GNSSLocation _location;
	//check the GNSS status

	//test the fix of the GNSS
	//suspend
	//wait for 10 seconds
	//and refix
	//then GNSS_en = 0 && BACKUP = 1 for 10 seconds
	//then restart And fix
	//then GNSS_en = 0 && BACKUP = 0 for 10 seconds

	switch(sequenceState){
	case 0:
		_localTimer.restart(500,500);
		//manager_gnss_init();
		manager_gnss_Vdd_En(true);
		manager_gnss_Backup_En(true);
		sequenceState++;
		break;
	case 1:
		manager_gnss_setState(GNSS_ON);
		sequenceState++;
		break;
	case 2:
		if(manager_gnss_getState() == GNSS_ON) sequenceState++;
		else manager_gnss_processState();
		break;
	case 3:
		GNSS.location(_location);
		if( (_location.fixType() == GNSSLocation::TYPE_2D) || (_location.fixType() == GNSSLocation::TYPE_3D) ){
			float_t _ehpe = _location.ehpe();
			LOG("EHPE : ");LOGLN(_ehpe);
			if(_ehpe < GNSS_EHPE_OK ){
				sequenceState++;
			}
		}
		break;
	case 4:
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF){
			manager_gnss_Vdd_En(false);
			manager_gnss_Backup_En(false);
			_localTimer.restart(10000,10000);
			sequenceState++;
		}
		else manager_gnss_processState();
		break;
	case 5 :
		SequenceEnd();
		break;
	}


}



void SequenceExecute(void){

	switch(currentSequence){
	case SEQ_NONE:
		_localTimer.stop();
		delay(5000);
		_localTimer.restart(10000,10000);
		SequenceEnd();
		break;
	case SEQ_LORAJOIN:
		//SequenceEnd();
		Seq_LoraJoin();
		break;
	case SEQ_LORASEND:
		Seq_LoraSend();
		break;
	case SEQ_PRESSURE:
		Seq_Pressure();
		break;
	case SEQ_SURFACE:
		Seq_SurfaceSensor();
		break;
	case SEQ_GNSS_FIRSTFIX:
		Seq_Gnss_FirstFix();
		break;
	case SEQ_GNSS_REFIX:
		Seq_Gnss_ReFix();
		break;
	case SEQ_GNSS_EN_OFF:
		Seq_Gnss_EnOFF();
		break;
	case SEQ_GNSS_BACK_OFF:
		Seq_Gnss_BackOFF();
		break;
	default:
		_localTimer.stop();
		STM32L0.stop(10000);
		break;
	}
}

void test_lowPower_launch(void){
	Serial.begin(9600);
	delay(10000);
	LOGLN("start test sequence");
	digitalWrite(LED_BUILTIN, HIGH);
	pinMode(LED_BUILTIN, OUTPUT);

	LOGLN("Flash Init");

	SPIFlash.init();
	LOGLN("Power down");
	SPIFlash.powerDown();

	_localTimer.start(_onTick,1000,1000);

	while(1){

		if(tickFlagLp){
			tickFlagLp = false;

			switch(state){
			case state_Wait:
				LOGLN("Wait");
				_localTimer.restart(1000,1000);
				state = state_LedCode;
				break;
			case state_LedCode:
				LedCode(currentSequence);
				break;
			case state_Sequence:
				SequenceExecute();
				break;
			}

		}


		STM32L0.stop(1000000);

	}



}
