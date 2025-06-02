/*
 * message_histo.c
 *
 *  Created on: 23 janv. 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"
#include "message_histo.h"



const int32_t histo_depth_limits[HISTO_SIZE - 1] = HISTO_DEPTH_LIMITS_CM;
const int32_t histo_stopTime_limits[HISTO_SIZE - 1] = HISTO_DEPTH_LIMITS_CM;


void message_histo_init(data_histo_t *msg){
	msg->version = HISTO_MESSAGE_VERSION;

	message_histo_clear(msg);
	msg->diveId = 0;
	msg->gnssNoFixCount = 0;

	msg->latitude = GNSS_UNDEFINED_VALUE;
	msg->longitude = GNSS_UNDEFINED_VALUE;
	msg->ehpe = GNSS_UNDEFINED_VALUE;
	msg->ttf = 0xFFFFFFFF;

}

void message_histo_clear(data_histo_t *msg){


	for(uint8_t i=0; i<HISTO_SIZE;i++){
		msg->diveDeepHisto[i] = 0;
		//msg->stopTimeHisto[i] = 0;
	}

	//msg->pureSurfaceTime_s = 0;
	//msg->backToSurfaceCount = 0;

	//msg->surfaceSensor_falseGRAD_count = 0;
	//msg->surfaceSensor_falseTHR_count = 0;
	msg->diveId++;
	msg->gnssNoFixCount++;


}

//value : depth in cm
uint8_t message_histo_getDiveZone(int32_t value){
	if(value >= histo_depth_limits[HISTO_SIZE-2]){
		return HISTO_SIZE-1;
	}

	for(uint8_t i=0; i<HISTO_SIZE-2;i++){
		if(value < histo_depth_limits[i]  ){
			return i;
		}
	}
	return 0;
}

void message_histo_addDepthPoint_cm(data_histo_t *msg, int32_t value){

	LOG("Add point ");LOGLN(value);
	LOG(" to zone ");

	if(value >= histo_depth_limits[HISTO_SIZE-2]){
		//over the last depth limits
		LOGLN(HISTO_SIZE-1);
		msg->diveDeepHisto[HISTO_SIZE-1]++;
		return;
	}

	for(uint8_t i=0; i<HISTO_SIZE-2;i++){
		if(value < histo_depth_limits[i]  ){
			LOGLN(i);
			msg->diveDeepHisto[i]++;
			return;
		}

	}

}

void message_histo_addDepthPoint_diveZone(data_histo_t *msg, uint8_t value){

	if(value<HISTO_SIZE){
		msg->diveDeepHisto[value]++;
	}

}

void message_histo_print(data_histo_t *msg){
	LOGLN("Histo : {");
	LOG("ID : ");LOGLN(msg->diveId);
	LOG("Depth = [");
	for(uint8_t i=0; i<HISTO_SIZE-1;i++){
		LOG(msg->diveDeepHisto[i]);LOG(";");
	}
	LOG(msg->diveDeepHisto[HISTO_SIZE-1]);
	LOGLN("]");
//	LOG("Dive Time : ");LOG(msg->diveTime_s);LOGLN("s");
	LOG("Surface Time : ");LOG(msg->surfaceTime_s);LOGLN("s");
	//LOG("Back to surface : ");LOG(msg->backToSurfaceCount);

	//	LOG("Trigger : ");LOGLN(TriggerStr[msg->surfaceTriggerId]);

	LOG("GNSS : { ");LOG(msg->latitude);LOG(",");LOG(msg->longitude);LOG("; E=");LOG(msg->ehpe);LOG(";TTF=");LOG(msg->ttf);LOGLN("s }");
	//LOG("Surface : GRAD_err=");LOG(msg->surfaceSensor_falseGRAD_count);LOG(",THR_err=");LOGLN(msg->surfaceSensor_falseTHR_count);
	LOGLN("}");


}

