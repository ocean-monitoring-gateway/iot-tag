/*
 * manager_gnssV2.h
 *
 *  Created on: 17 juil. 2020
 *      Author: jfezande
 */

#ifndef SRC_MANAGER_MANAGER_GNSSV2_H_
#define SRC_MANAGER_MANAGER_GNSSV2_H_

#include "Arduino.h"
#include "../_manager_gnss_config.h"


#define GNSS_ON		true
#define GNSS_OFF 	false

typedef struct {
	bool state;
	bool neededState;

	uint8_t noFixCount;
	uint8_t zeroSatTimeoutCount;
	uint8_t fixDelayLocked;

	//gestion du temps d'utilisation
	uint32_t dailyUseTime;
	uint32_t totalUseTime;


	//current point
	int32_t latitude;
	int32_t longitude;
	uint32_t ehpe; //ehpe x 1000
	uint32_t hdop;
	uint32_t ttf;
	uint8_t nbSat; //max number of sat during session
	uint8_t nbSatPowered; //max number of sat with a minimum reception level during session


	// session parameters
	uint32_t startTime;
	uint32_t lastFixTime;
	uint32_t lastEHPE; //best EHPE session


}manager_gnss_t;



void manager_gnssV2_init(manager_gnss_t *pManager);

void manager_gnssV2_setON(manager_gnss_t *pManager);
void manager_gnssV2_setOFF(manager_gnss_t *pManager);

void manager_gnssV2_process(manager_gnss_t *pManager);





#endif /* SRC_MANAGER_MANAGER_GNSSV2_H_ */
