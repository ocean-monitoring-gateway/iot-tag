/*
 * message_histo.h
 *
 *  Created on: 23 janv. 2020
 *      Author: jfezande
 */

#ifndef LORAAPPLICATION_MESSAGEDEFINTION_MESSAGE_HISTO_H_
#define LORAAPPLICATION_MESSAGEDEFINTION_MESSAGE_HISTO_H_

#include "Arduino.h"
#include "../_appDirectives.h"

#define HISTO_MESSAGE_VERSION	0x0106


typedef struct __attribute__((packed, aligned(1))){
	//version 1.6
	uint16_t version; //version du format de message
	uint16_t diveId;
	//uint16_t stopTimeHisto[HISTO_SIZE];
	uint16_t diveDeepHisto[HISTO_SIZE];
	//uint32_t diveTime_s;


	//of the previous dive
	int32_t latitude;
	int32_t longitude;
	uint32_t ehpe;
	uint32_t ttf;
	uint32_t surfaceTime_s;
	//uint32_t pureSurfaceTime_s;

	uint32_t surfaceSensorUseTime;
	uint16_t gnssUseTime;

	uint8_t gnssNoFixCount;
	uint8_t gnssZeroSatTimeout;


	uint8_t gnssNbSat;
	uint8_t gnssNbSatPowered;



	uint8_t profile[20];
	uint16_t temperature;

	//power
	uint16_t battLevel_mV;

}data_histo_t;

void message_histo_init(data_histo_t *msg);
void message_histo_clear(data_histo_t *msg);
uint8_t message_histo_getDiveZone(int32_t value);
void message_histo_addDepthPoint_cm(data_histo_t *msg, int32_t value);
void message_histo_addDepthPoint_diveZone(data_histo_t *msg, uint8_t value);

void message_histo_print(data_histo_t *msg);

#endif /* LORAAPPLICATION_MESSAGEDEFINTION_MESSAGE_HISTO_H_ */
