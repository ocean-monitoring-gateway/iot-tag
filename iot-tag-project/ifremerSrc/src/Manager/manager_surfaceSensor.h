/*
 * manager_surfaceSensor.h
 *
 *  Created on: 28 janv. 2020
 *      Author: jfezande
 */

#ifndef MANAGER_MANAGER_SURFACESENSOR_H_
#define MANAGER_MANAGER_SURFACESENSOR_H_

#define sensorPin  A3 // Analog pin for surface detection
#define switchPin  9


#define SURFACESENSOR_INITSTATE_SURFACE		(true)
#define SURFACESENSOR_INITSTATE_DIVE		(false)

typedef enum {
	surfaceSensorAlgo_threshold,
	surfaceSensorAlgo_gradient
}surfaceSensorAlgo_t;

typedef struct {
	bool state;

	uint32_t useTime;


	uint32_t startTime;


}manager_surfaceSensor_t;


void manager_surfaceSensor_init(bool initState);
void manager_surfaceSensor_process(void);
bool manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_t algo);
bool manager_surfaceSensor_diveDetection(surfaceSensorAlgo_t algo);
uint16_t manager_surfaceSensor_getFilter0(void);
uint16_t manager_surfaceSensor_getFilterN(void);
//uint16_t manager_surfaceSensor_getLastChange(void);
uint16_t manager_surfaceSensor_getLastSurface(void);
uint16_t manager_surfaceSensor_getLastDive(void);

void manager_surfaceSensor_initialize(manager_surfaceSensor_t *pManager);
void manager_surfaceSensor_start(manager_surfaceSensor_t *pManager);
void manager_surfaceSensor_stop(manager_surfaceSensor_t *pManager);

#endif /* MANAGER_MANAGER_SURFACESENSOR_H_ */
;
