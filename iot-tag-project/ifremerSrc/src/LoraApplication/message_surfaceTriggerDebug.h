/*
 * message_surfaceTriggerDebug.h
 *
 *  Created on: 29 juin 2020
 *      Author: jfezande
 */

#ifndef SRC_LORAAPPLICATION_MESSAGE_SURFACETRIGGERDEBUG_H_
#define SRC_LORAAPPLICATION_MESSAGE_SURFACETRIGGERDEBUG_H_

#include "Arduino.h"
#include "../_appDirectives.h"


#define MAX_TRIG_STORED		5

typedef struct {
	uint32_t trigTime;
	uint16_t surfaceSensor_filter0;
	uint16_t surfaceSensor_filterN;

	int32_t pressure1BA;

	// force the struct format alignement with 32bits align
	//	uint8_t trigSource;
	uint32_t trigSource;
}data_SurfaceDebug_trigDef_t;

typedef struct __attribute__((packed, aligned(1))){
	uint16_t surfaceSensor_falseTHR_count;
	uint16_t surfaceSensor_falseGRAD_count;
	uint16_t surfaceSensor_filter0;
	uint16_t surfaceSensor_filterN;
	uint16_t surfaceSensor_lastChange;

	uint16_t diveSensor_filter0;
	uint16_t diveSensor_filterN;
	uint16_t diveSensor_lastChange;

	int32_t currentDepth_mm;
	int32_t currentDepthCalibrated_mm;
	int32_t minPressure_mPa;

	//surface trigger
	uint8_t surfaceTriggerId;
	uint8_t diveTriggerId;

	data_SurfaceDebug_trigDef_t lastTrigs[MAX_TRIG_STORED];

}data_SurfaceDebug_t;

void message_surfaceTriggerDebug_buildFalseTrigMessage(data_SurfaceDebug_t *msg, data_SurfaceDebug_trigDef_t *trig);
void message_surfaceTriggerDebug_clearMessage(data_SurfaceDebug_t *msg);
void message_surfaceTriggerDebug_print(data_SurfaceDebug_t *msg);




#endif /* SRC_LORAAPPLICATION_MESSAGE_SURFACETRIGGERDEBUG_H_ */
