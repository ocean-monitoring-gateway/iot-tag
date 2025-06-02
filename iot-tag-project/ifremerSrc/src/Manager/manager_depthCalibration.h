/*
 * manager_depthCalibration.h
 *
 *  Created on: 25 juin 2020
 *      Author: jfezande
 */

#ifndef SRC_MANAGER_MANAGER_DEPTHCALIBRATION_H_
#define SRC_MANAGER_MANAGER_DEPTHCALIBRATION_H_


#define NB_WINDOWS 12
#define TICK_PER_WINDOWS (60*10)
//#define TICK_PER_WINDOWS (60)

typedef struct {
	int32_t minPressure[NB_WINDOWS]; //par tranche de 10 minutes
	int32_t currentPressure_mPa;
	int32_t currentTemperature;

	int32_t minPressure_mPa;

	uint16_t readErrorCount;

	uint16_t tickIndex; //tick index restart after TICK_PER_WINDOWS tick
	uint8_t index;

	int32_t temperatureAcc;
	uint16_t temperatureCount;

} manager_depthCalibration_t;


void manager_depthCalibration_init(manager_depthCalibration_t *handle);
void manager_depthCalibration_emptyTick(manager_depthCalibration_t *handle);
void manager_depthCalibration_pushValue(manager_depthCalibration_t *handle, int32_t value);
//int32_t manager_depthCalibration_getMinPressure(manager_depthCalibration_t *handle);

void manager_depthCalibration_clear(manager_depthCalibration_t *handle);
int16_t manager_depthCalibration_getTemperature(manager_depthCalibration_t *handle);


//call process in the main loop at periodic interval (tick)
void manager_depthCalibration_process(manager_depthCalibration_t *handle);

int32_t manager_depthCalibration_getCurrentValue_mPa(manager_depthCalibration_t *handle);
int32_t manager_depthCalibration_getMinPressure_mPa(manager_depthCalibration_t *handle);

int32_t manager_depthCalibration_getCurrentValue_mm(manager_depthCalibration_t *handle);
int32_t manager_depthCalibration_getCurrentCalibratedValue_mm(manager_depthCalibration_t *handle);


#endif /* SRC_MANAGER_MANAGER_DEPTHCALIBRATION_H_ */
