/*
 * manager_diveProfile.cpp
 *
 *  Created on: 10 août 2020
 *      Author: jfezande
 */



#include "manager_diveProfile.h"


uint8_t _convertProfilValue(uint32_t value){
	//valeur en mm ramener en dm
	uint32_t mean = value;
	mean /= 100;

	if(mean >= 0xFF) mean = 0xFF; //saturée à la valeur maximale (25,5 dm)

	return (uint8_t)mean;

}

void manager_diveProfile_start(manager_diveProfile_t *p){

	p->currentMeanAcc = 0;
	p->meanNbPts = 0;
	p->currentStep = DIVEPROFILE_INITIAL_STEP;
	p->meanIndex = 0;


}
void manager_diveProfile_end(manager_diveProfile_t *p){

	//calculer la moyenne en cours
	uint32_t mean;
	mean = p->currentMeanAcc / p->meanNbPts;
	p->means[p->meanIndex] = _convertProfilValue(mean);


}

void manager_diveProfile_addPoint(manager_diveProfile_t *p, int16_t value){

	//la moyenne contient en 1 et currentStep points

	if(p->meanNbPts == p->currentStep){
		//on calacule la moyenne
		uint32_t mean;
		mean = p->currentMeanAcc / p->meanNbPts;


		p->means[p->meanIndex] = _convertProfilValue(mean);
		p->meanNbPts = 0;
		p->currentMeanAcc = 0;

		p->meanIndex++;

		if(p->meanIndex == DIVEPROFILE_MEANSIZE){
			//le tableau des moyennes est plein
			//on réduit le nb de point par 2
			for(uint8_t i = 0; i <DIVEPROFILE_MEANSIZE/2; i++){
				uint32_t val = (p->means[2*i] + p->means[2*i+1]) / 2;
				p->means[i] = val;
			}

			p->currentStep *= 2; //double le pas

			p->meanIndex = DIVEPROFILE_MEANSIZE/2; // on ramène l'index des moyennes sur le premier élément de la seconde partie du tableau.

		}
	}



	if(value > 0) //valeur négative équivalente à la surface donc 0
		p->currentMeanAcc += value;

	p->meanNbPts++;


}


void manager_diveProfile_get(manager_diveProfile_t *p, manager_diveProfile_result_t *pRes){

	//génération et remplissage du tableau final
	memset(pRes->profile, 0, DIVEPROFILE_OUTPUTSIZE);

	//définir le nombre de points par valeur de sortie
	uint8_t oversampleRatio = (p->meanIndex / DIVEPROFILE_OUTPUTSIZE) + 1;


	pRes->timeStep = (p->currentStep/DIVEPROFILE_INITIAL_STEP)*oversampleRatio;

	bool exit = false;

	for(uint8_t i = 0; i<DIVEPROFILE_OUTPUTSIZE;i++){
		//pour chaque point du tableau de sortie, on calcule une moyenne en fonction du nombre de points par moyenne calculé précédemment
		uint32_t localAcc = 0;
		uint8_t j;
		for(j=0;j<oversampleRatio; j++){
			localAcc += p->means[i*oversampleRatio + j];

			if(i*oversampleRatio + j == p->meanIndex){
				exit = true;
				break;
			}
		}

		if(exit)
			localAcc /= j+1;
		else
			localAcc /= oversampleRatio;

		pRes->profile[i] = (uint8_t)localAcc;

		if(exit) break;
	}



}




