/*
 * message_surfaceTriggerDebug.cpp
 *
 *  Created on: 29 juin 2020
 *      Author: jfezande
 */

#include "stdlib.h"
#include "message_surfaceTriggerDebug.h"


void message_surfaceTriggerDebug_buildFalseTrigMessage(data_SurfaceDebug_t *msg, data_SurfaceDebug_trigDef_t *trig){

	uint8_t index;
	uint8_t triggerId = trig->trigSource;

	//calculate the current index
	index = (msg->surfaceSensor_falseGRAD_count + msg->surfaceSensor_falseTHR_count)%MAX_TRIG_STORED;

	//don't use memcopy because of the alignement of the data_SurfaceDebug_t struct
	msg->lastTrigs[index].trigSource = trig->trigSource;
	msg->lastTrigs[index].pressure1BA = trig->pressure1BA;
	msg->lastTrigs[index].surfaceSensor_filter0 = trig->surfaceSensor_filter0;
	msg->lastTrigs[index].surfaceSensor_filterN = trig->surfaceSensor_filterN;
	msg->lastTrigs[index].trigTime = trig->trigTime;

	//increment the trigger counter
	if(triggerId == 0) msg->surfaceSensor_falseTHR_count++;
	if(triggerId == 1) msg->surfaceSensor_falseGRAD_count++;

}

void message_surfaceTriggerDebug_clearMessage(data_SurfaceDebug_t *msg){

	//clear the message
	memset(msg,0,sizeof(data_SurfaceDebug_t));


}

void message_surfaceTriggerDebug_print(data_SurfaceDebug_t *msg){
	LOGLN("-- Message surface Trigger Debug --");
	LOGLN("Sutface Trig");
	LOG("Filter0 : ");LOGLN(msg->surfaceSensor_filter0);
	LOG("FilterN : ");LOGLN(msg->surfaceSensor_filterN);
	LOG("LastChg : ");LOGLN(msg->surfaceSensor_lastChange);

	LOGLN("Dive trig");
	LOG("Filter0 : ");LOGLN(msg->diveSensor_filter0);
	LOG("FilterN : ");LOGLN(msg->diveSensor_filterN);
	LOG("LastChg : ");LOGLN(msg->diveSensor_lastChange);




}
