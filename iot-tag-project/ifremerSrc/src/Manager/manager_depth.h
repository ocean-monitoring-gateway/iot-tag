/*
 * manager_depth.h
 *
 *  Created on: 23 janv. 2020
 *      Author: jfezande
 */

#ifndef MANAGER_MANAGER_DEPTH_H_
#define MANAGER_MANAGER_DEPTH_H_

#include "Arduino.h"

typedef enum{
	pressureSensor_1BA,
	pressureSensor_30BA,

	pressureSensor_size
}pressureSensor_range_t;

typedef enum{
	psr_OK,
	psr_ERROR,
	psr_OVERLIMIT

}pressureSensor_result_t;

bool manager_depth_init(void);
pressureSensor_result_t manager_depth_get_mm(pressureSensor_range_t range, int32_t *pValue);
pressureSensor_result_t manager_depth_get_mm_temp(pressureSensor_range_t range, int32_t *pPressure, int32_t *pTemp);

void manager_depth_calibrate30BA(void);

int32_t manager_depth_getOffset30BA(void);

pressureSensor_result_t manager_depth_readPressure_mPa_1BA(int32_t *pValue);
pressureSensor_result_t manager_depth_readPressure_mPa_30BA(int32_t *pValue);

pressureSensor_result_t manager_depth_readPressure_mPa_Temp_1BA(int32_t *pPressure, int32_t *pTemp);
pressureSensor_result_t manager_depth_readPressure_mPa_Temp_30BA(int32_t *pPressure, int32_t *pTemp);

int32_t manager_depth_convert_mPa2mm(int32_t value, int32_t surfacePressure);


#endif /* MANAGER_MANAGER_DEPTH_H_ */
