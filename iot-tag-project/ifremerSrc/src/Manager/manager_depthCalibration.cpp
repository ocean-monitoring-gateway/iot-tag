/*
 * manager_depthCalibration.c
 *
 *  Created on: 25 juin 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"

#include "manager_depth.h"
#include "manager_depthCalibration.h"

#define DEFAULT_SURFACE_MPa 	(101325*1000)



void _refreshMin(manager_depthCalibration_t *handle){
	int32_t result = INT32_MAX;

	for(uint8_t i=0;i<NB_WINDOWS;i++){
		if(handle->minPressure[i] < result)
			result = handle->minPressure[i];
	}

	handle->minPressure_mPa = result;

}

// return 1 if the window changes
void _tickIndex(manager_depthCalibration_t *handle){
	handle->tickIndex++;
	if(handle->tickIndex >= TICK_PER_WINDOWS){
		//change the window
		handle->tickIndex = 0;

		handle->index++;
		if(handle->index >= NB_WINDOWS)
			handle->index = 0;

		//clear the new window
		handle->minPressure[handle->index] = INT32_MAX;
		_refreshMin(handle);

	}

}



void manager_depthCalibration_init(manager_depthCalibration_t *handle){

	for(uint8_t i=0;i<NB_WINDOWS;i++){
		handle->minPressure[i] = INT32_MAX;
	}
	handle->index = 0;
	handle->tickIndex = 0;

	handle->readErrorCount = 0;
	handle->minPressure_mPa = INT32_MAX;

}

//use empty when no measure is provide but conserve the timeline
void manager_depthCalibration_emptyTick(manager_depthCalibration_t *handle){
	_tickIndex(handle);

}


void manager_depthCalibration_clear(manager_depthCalibration_t *handle){
	handle->temperatureAcc = 0;
	handle->temperatureCount = 0;
}

int16_t manager_depthCalibration_getTemperature(manager_depthCalibration_t *handle){

	return (int16_t) ( handle->temperatureAcc / handle->temperatureCount);
}



void manager_depthCalibration_pushValue(manager_depthCalibration_t *handle, int32_t value){

	if( value < handle->minPressure[handle->index]){
		handle->minPressure[handle->index] = value;

		if( value < handle->minPressure_mPa )
			handle->minPressure_mPa = value;
	}

	//change indexes values
	_tickIndex(handle);
}

void manager_depthCalibration_process(manager_depthCalibration_t *handle){
	int32_t pressure, temperature;

	pressureSensor_result_t res;

	res = manager_depth_readPressure_mPa_Temp_1BA(&pressure, &temperature);

	if(res == psr_OK){
		handle->currentPressure_mPa = pressure;
		handle->currentTemperature = temperature;

		manager_depthCalibration_pushValue(handle, pressure);

		handle->temperatureAcc += temperature;
		handle->temperatureCount++;


	}
	else{
		LOG("Error in pressure meas with 1BA");
		//manager_depthCalibration_emptyTick(handle);
		if(res == psr_OVERLIMIT){
			//active the 30BA sensor
			LOGLN("1BA psr_OVERLIMIT, using 30BA sensor");
			res = manager_depth_readPressure_mPa_Temp_30BA(&pressure, &temperature);
			if(res == psr_OK){
					handle->currentPressure_mPa = pressure;
					handle->currentTemperature = temperature;

					manager_depthCalibration_pushValue(handle, pressure);

					handle->temperatureAcc += temperature;
					handle->temperatureCount++;

			}
			if(res == psr_ERROR){
				LOGLN("30BA measurement error !");
				manager_depthCalibration_emptyTick(handle);
				handle->readErrorCount++;
			}
		}
		if(res == psr_ERROR){
			handle->readErrorCount++;
			manager_depthCalibration_emptyTick(handle);
		}
	}

	LOG("min mPa : ");LOGLN(handle->minPressure_mPa);

	/* --- (DEBUG) --- */
	/*
	int32_t pressure_debug;

	LOGLN("--- (DEBUG) ---");
	res = manager_depth_readPressure_mPa_1BA(&pressure_debug);
	LOG("get_mPa pressureSensor_1BA : ");
	LOGLN(pressure_debug);
	if(res == psr_OK){
		//surface detection only with the 1BA
		LOGLN("1BA psr_OK");
	}
	else if(res == psr_OVERLIMIT){
		//active the 30BA sensor
		LOGLN("1BA psr_OVERLIMIT");
	}

	res = manager_depth_readPressure_mPa_30BA(&pressure_debug);
	LOG("get_mm pressureSensor_30BA : ");
	LOGLN(pressure_debug);
	if(res == psr_OK){
		//surface detection only with the 1BA
		LOGLN("30BA psr_OK");
	}
	else if(res == psr_OVERLIMIT){
		//active the 30BA sensor
		LOGLN("30BA psr_OVERLIMIT");
	}

	LOGLN("--- (end of DEBUG) ---");
	*/
	/* --- (end of DEBUG) --- */


}

int32_t manager_depthCalibration_getCurrentValue_mPa(manager_depthCalibration_t *handle){
	return handle->currentPressure_mPa;
}

int32_t manager_depthCalibration_getMinPressure_mPa(manager_depthCalibration_t *handle){
	return handle->minPressure_mPa;
}


int32_t manager_depthCalibration_getCurrentValue_mm(manager_depthCalibration_t *handle){
	int32_t value;
	value = manager_depth_convert_mPa2mm(handle->currentPressure_mPa, DEFAULT_SURFACE_MPa);
	LOG("d  mm : ");LOGLN(value);
	return value;

}

int32_t manager_depthCalibration_getCurrentCalibratedValue_mm(manager_depthCalibration_t *handle){
	int32_t value;
	value = manager_depth_convert_mPa2mm(handle->currentPressure_mPa, handle->minPressure_mPa);
	LOG("dc mm : ");LOGLN(value);
	return value;
}





