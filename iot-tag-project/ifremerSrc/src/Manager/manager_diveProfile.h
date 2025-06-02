/*
 * manager_diveProfile.h
 *
 *  Created on: 10 août 2020
 *      Author: jfezande
 */



#ifndef SRC_MANAGER_MANAGER_DIVEPROFILE_H_
#define SRC_MANAGER_MANAGER_DIVEPROFILE_H_

#include "Arduino.h"

#define DIVEPROFILE_MEANSIZE 120
#define DIVEPROFILE_INITIAL_STEP 15 //en secondes
#define DIVEPROFILE_OUTPUTSIZE 20


typedef struct {
	uint32_t currentMeanAcc; //accumulateur pour le calcul de la moyenne courante
	uint8_t meanNbPts; // nb de points dans la moyenne courante
	uint16_t currentStep; // pas en cours en secondes

	uint8_t means[DIVEPROFILE_MEANSIZE]; // tableau des moyennes
	uint8_t meanIndex; //index dans le tableau des moyennes

}manager_diveProfile_t;

typedef struct {
	uint8_t profile[DIVEPROFILE_OUTPUTSIZE];
	uint8_t timeStep; //N x INITIALSTEP
}manager_diveProfile_result_t;

void manager_diveProfile_start(manager_diveProfile_t *p);
void manager_diveProfile_end(manager_diveProfile_t *p);

void manager_diveProfile_addPoint(manager_diveProfile_t *p, int16_t value);

void manager_diveProfile_get(manager_diveProfile_t *p, manager_diveProfile_result_t *pRes);




#endif /* SRC_MANAGER_MANAGER_DIVEPROFILE_H_ */
