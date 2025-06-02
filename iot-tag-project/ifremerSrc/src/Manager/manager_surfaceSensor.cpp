/*
 * manager_surfaceSensor.cpp
 *
 *  Created on: 28 janv. 2020
 *      Author: jfezande
 */

#include "../_appDirectives.h"


#include "Arduino.h"
#include "RTC.h"

#include "manager_appTick.h"
#include "manager_surfaceSensor.h"

uint8_t flag_surface_gradient;
uint8_t flag_surface_threshold;
uint8_t flag_dive_threshold;

uint16_t conductivity[sizeMeasuresTab];
uint16_t val_end_prec;
uint16_t val_last_dive;
uint16_t val_last_surface = 0;
uint16_t i_conduct;

uint16_t _filter0 = 0;
uint16_t _filterN = 0;

uint8_t flag_surface_gradient_old;
bool out_water;

uint16_t startCount;


void _conduct_filter(uint16_t unfilt_measures[], uint16_t filt_measures[]);

uint8_t _surface_detection_gradient(uint16_t filt_measures[], uint16_t flag_out_prec, uint16_t val_end_prec, uint16_t val_last_dive);
uint8_t _surface_detection_threshold(uint16_t filt_measures[], uint16_t initial_measure, const uint16_t delta_threshold, uint16_t val_last_change);


void manager_surfaceSensor_initialize(manager_surfaceSensor_t *pManager){
	pManager->state = false;
	pManager->useTime = 0;


}

void manager_surfaceSensor_start(manager_surfaceSensor_t *pManager){
	if(!pManager->state){
		LOGLN("[SURFACE SENSOR] : Start");
		pManager->startTime = manager_appTick_get();
		pManager->state = true;
	}
}

void manager_surfaceSensor_stop(manager_surfaceSensor_t *pManager){
	if(pManager->state){
		LOGLN("[SURFACE SENSOR] : Stop");
		pManager->useTime += (manager_appTick_get() - pManager->startTime);
		pManager->state = false;
		LOG("[SURFACE SENSOR] : Use Time :");LOGLN(pManager->useTime);
	}
}





void manager_surfaceSensor_init(bool initState){
	out_water = initState;
	flag_surface_gradient_old = 0;
	flag_surface_gradient = 0;
	flag_surface_threshold = 0;
	flag_dive_threshold = 0;

	//val_last_surface = 5120;
	//val_last_surface = 0;

	pinMode(switchPin, OUTPUT);
	digitalWrite(switchPin, LOW);

	startCount = 0;
}



bool manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_t algo){

	//LOGLN("test on rentre dans surface detect");
	switch(algo){
	case surfaceSensorAlgo_threshold:

		if (flag_surface_threshold==1) return true;

		break;
	case surfaceSensorAlgo_gradient:
		if (flag_surface_gradient==1) return true;
		break;
	}

	return false;
}


bool manager_surfaceSensor_diveDetection(surfaceSensorAlgo_t algo){

	//LOGLN(flag_dive_threshold);
	//LOGLN("test on rentre dans dive detect");
	switch(algo){
	case surfaceSensorAlgo_threshold:

		if (flag_dive_threshold==2) return true;

		break;
	case surfaceSensorAlgo_gradient:
		if (flag_surface_gradient==2) return true;
		break;
	}

	return false;

}


void manager_surfaceSensor_process(void){

	// variables declaration
	uint16_t filtered_measures[sizeMeasuresTab];
	uint16_t current_measure;
	uint16_t initial_measure;
	//uint8_t flag_dive_threshold;

	// Read Analog measure
	digitalWrite(switchPin, HIGH);
	current_measure = 10*analogRead(sensorPin);  //x10 -> precision 0.1 sans utiliser de float
	//Serial.println(current_measure);
	digitalWrite(switchPin, LOW);

	if(i_conduct < sizeMeasuresTab) {
		// Initialize conductivity measure array
		conductivity[i_conduct] = current_measure;
		i_conduct ++;
		return;
	}

	const uint16_t Nmeas = 100;

	if(startCount == (Nmeas/2)){
		val_last_surface = current_measure;			// COQUILLE DANS LA VERSION PRECEDENTE (ca aurait du etre val_last_surface)
		val_last_dive    = current_measure;
	}

	if(startCount < Nmeas){//let at least N mesaure before start the algorythm
		startCount++;
		return;
	}

#warning Attention vieux patch


	// update conductivity measure array: save the last N-1 (=sizeMeasuresTab-1) measures and update the last value of the array with the current measure
	for (int i_translate_measures = 0; i_translate_measures < sizeMeasuresTab - 1; i_translate_measures++) {
		conductivity[i_translate_measures] = conductivity[i_translate_measures + 1];
	}
	conductivity[sizeMeasuresTab-1] = current_measure;

	// Measure Array Filtering: see conduct_filter function for filters tuning
	_conduct_filter(conductivity, filtered_measures);

	// Choose which filtered measure of the array will define the beginning of the surface detection measure interval (in this case: first measure for full interval treatment)
	initial_measure = filtered_measures[0];
	_filter0 = filtered_measures[0];
	_filterN = filtered_measures[sizeMeasuresTab-1];

	if (out_water) {
		/*  CASE: OUT OF WATER - surface already detected - mode detection de plongee
		 *  In this case, diving detection via threshold is activated (see function surface_detection_threshold)
		 */
		/*LOGLN(_filter0);
		LOGLN(_filterN);
		LOGLN(val_last_surface);*/
		// diving detection via threshold
		flag_dive_threshold = _surface_detection_threshold(filtered_measures, initial_measure, delta_threshold_out, val_last_surface);

		if (flag_dive_threshold == 2){      // flag_dive_threshold == 2 -> diving, else -> surface
			LOGLN("plongee detectee");
			// reinitialisation du tableau de mesures
			val_last_dive = filtered_measures[sizeMeasuresTab - 1];
			i_conduct = 0;
			// Passage dans le mode detection de surface
			// digitalWrite(LED_BUILTIN, HIGH);
			out_water = false;
		}
	}

	else {
		/*  CASE: IN WATER - surface detection activated - mode detection de surface
		 *  In this case, surface detection by threshold and by gradient are activated (see function surface_detection_threshold and surface_detection_gradient)
		 */

		// Surface detection mode :

		// initialisation valeur seuil mini du dernier surfacage
		if (val_last_dive == 0){
			val_last_dive = initial_measure;
		}

		// 1 - via gradient
#warning A revoir gestion du gradient

		flag_surface_gradient = _surface_detection_gradient(filtered_measures, flag_surface_gradient_old, val_end_prec, val_last_dive);
		//flag_surface_gradient = _surface_detection_gradient(filtered_measures, flag_surface_gradient_old, val_end_prec, val_end_prec);
		val_end_prec = filtered_measures[sizeMeasuresTab - 1]; // update
		flag_surface_gradient_old = flag_surface_gradient;

		// 2 - via seuil
#warning A revoir pour la valeur initial
		flag_surface_threshold = _surface_detection_threshold(filtered_measures, initial_measure, delta_threshold_in, val_last_dive);
		//flag_surface_threshold = _surface_detection_threshold(filtered_measures, initial_measure, delta_threshold_in, val_last_surface);

		if ((flag_surface_threshold == 1) || (flag_surface_gradient == 1)) {
			Serial.println("Trig Surface");
			//flag_dive_threshold = 0;
			for(uint8_t i = 0; i<sizeMeasuresTab;i++){
				Serial.print(filtered_measures[i]);Serial.print(";");
			}
			Serial.println("");

			// reinitialisation des mesures et mise a jour valeur seuil mini du dernier surfacage
			//val_last_surface = (filtered_measures[0] + filtered_measures[sizeMeasuresTab - 1])/2;
			val_last_surface = filtered_measures[sizeMeasuresTab - 1];
			i_conduct = 0;

			// passage en mode detection de replongee
			// digitalWrite(LED_BUILTIN, LOW);
			out_water = true;
			// delay(10);
		}
	}

}
uint16_t manager_surfaceSensor_getFilter0(void){
	return _filter0;
}

uint16_t manager_surfaceSensor_getFilterN(void){
	return _filterN;
}

uint16_t manager_surfaceSensor_getLastSurface(void){
	return val_last_surface;
}

uint16_t manager_surfaceSensor_getLastDive(void){
	return val_last_dive;
}



void _conduct_filter(uint16_t unfilt_measures[], uint16_t filt_measures[]) {
	// Fonction de filtrage des tableaux de mesures du capteur de surface

	uint16_t wnoise_measures[sizeMeasuresTab];

	// ----------- suppression des pics/bruits de mesure ---------
	// Initialisation
	for(int i_init_noise = 0; i_init_noise < sizeMeasuresTab; i_init_noise ++){
		wnoise_measures[i_init_noise] = unfilt_measures[i_init_noise];
	}

	if(wnoise_measures[0] == 0){
		wnoise_measures[0] = unfilt_measures[1];
	}
	if(wnoise_measures[sizeMeasuresTab-1] == 0){
		wnoise_measures[sizeMeasuresTab-1] = unfilt_measures[sizeMeasuresTab-2];
	}

	// Boucle de suppression des bruits
	for(int i_noise = 1; i_noise<sizeMeasuresTab-2; i_noise ++){
		// Pics positifs
		if (wnoise_measures[i_noise] > wnoise_measures[i_noise - 1]){
			if (wnoise_measures[i_noise] > wnoise_measures[i_noise + 1]){
				wnoise_measures[i_noise] = wnoise_measures[i_noise - 1];
			}
			if (wnoise_measures[i_noise + 1] > wnoise_measures[i_noise + 2]){
				wnoise_measures[i_noise] = wnoise_measures[i_noise - 1];
			}
		}
		// Pics negatifs
		if (wnoise_measures[i_noise] < wnoise_measures[i_noise -1]){
			if (wnoise_measures[i_noise] < wnoise_measures[i_noise + 1]){
				wnoise_measures[i_noise] = wnoise_measures[i_noise - 1];
			}
			if (wnoise_measures[i_noise + 1] < wnoise_measures[i_noise + 2]){
				wnoise_measures[i_noise] = wnoise_measures[i_noise - 1];
			}
		}
	}

	// Boucle de filtrage
	// initialisation du filtrage: recopie de mesures debruitees wnoise
	filt_measures[0] = wnoise_measures[0];
	// filtrage
	for (int i_filt = 1; i_filt < sizeMeasuresTab - 1; i_filt ++){
		filt_measures[i_filt] =  filt_measures[i_filt - 1]*1/3 + ((wnoise_measures[i_filt + 1] + wnoise_measures[i_filt] + wnoise_measures[i_filt - 1])*2/9);
	}
	filt_measures[sizeMeasuresTab-1] = filt_measures[sizeMeasuresTab-2]*1/3 + ((2*wnoise_measures[sizeMeasuresTab-1] + wnoise_measures[sizeMeasuresTab-2])*2/9);

}


uint8_t _surface_detection_gradient(uint16_t filt_measures[], uint16_t flag_out_prec, uint16_t val_end_prec, uint16_t val_last_dive){

	int32_t dX_filt[sizeMeasuresTab - 1];
	int32_t d2X_filt[sizeMeasuresTab - 2];
	uint16_t val_end = filt_measures[sizeMeasuresTab - 1];
	uint8_t sum_dx_pos = 0;
	uint8_t sum_dx_neg = 0;
	// uint16_t sum_d2x_pos = 0;
	uint8_t flag_out = 0;
	uint8_t sum_tab_flag_out = 0;
	/*  Serial.println(filt_measures[0]);
Serial.println(filt_measures[1]);
  Serial.println(filt_measures[2]);
Serial.println(filt_measures[3]);
  Serial.println(filt_measures[4]);
Serial.println(filt_measures[5]);
  Serial.println(filt_measures[6]);
Serial.println(filt_measures[7]);
  Serial.println(filt_measures[8]);
Serial.println(filt_measures[9]); */

	// Calcul derivee
	for (int i_dx = 0; i_dx < sizeMeasuresTab - 1; i_dx ++){
		dX_filt[i_dx] = (1/1)*(filt_measures[i_dx + 1] - filt_measures[i_dx]);
		if((dX_filt[i_dx]) > 1){
			sum_dx_pos = sum_dx_pos + 1;
		}
		if((dX_filt[i_dx]) <= 1){
			sum_dx_neg = sum_dx_neg + 1;
		}
	}

	// Calcul derivee seconde
	for (int i_d2x = 0; i_d2x < sizeMeasuresTab - 2; i_d2x ++){
		d2X_filt[i_d2x] = (1/(1))*(dX_filt[i_d2x + 1] - dX_filt[i_d2x]);
		/*if((d2X_filt[i_d2x]) > 0){
      sum_d2x_pos = sum_d2x_pos + 1;
    }*/
	}


	// Conditions de detection de la surface
	if((val_end != val_end_prec) && !(flag_out_prec == 2 && filt_measures[0] <= 1) && filt_measures[0] >= val_last_dive) {
		for (int i_flag_out = sizeMeasuresTab - 2; i_flag_out < sizeMeasuresTab; i_flag_out++) {
			if (filt_measures[i_flag_out] >  filt_measures[0]) {
				sum_tab_flag_out = sum_tab_flag_out + 1;
			}
		}
	}

	if ((sum_tab_flag_out == 2) && (sum_dx_pos == sizeMeasuresTab-1) && (d2X_filt[sizeMeasuresTab - 3] > 0) && (d2X_filt[sizeMeasuresTab - 4] > 0)){ //&& (sum_d2x_pos >= sizeMeasuresTab-5)
		flag_out = 1;  // surface detectee
	}
	else if((filt_measures[0] == filt_measures[sizeMeasuresTab - 1]) || (sum_dx_neg == sizeMeasuresTab - 1)){
		flag_out = 2; // pour cette valeur de flag -> mesures constantes apres filtrage
	}

	return flag_out;
}




uint8_t _surface_detection_threshold(uint16_t filt_measures[], uint16_t initial_measure, const uint16_t delta_threshold, uint16_t val_last_change){
	// fonction de detection de la surface ou de rentree dans l'eau a partir d'un seuil = valeur initiale + delta

	uint8_t flag_out = 0;
	uint8_t sum_over_threshold = 0; // detection sortie eau
	uint8_t sum_under_threshold = 0;  // detection rentree eau
	uint32_t threshold = (67*initial_measure + 33*val_last_change)/100;


	for (int i_threshold = 5; i_threshold < sizeMeasuresTab; i_threshold++) {
		if (filt_measures[i_threshold] >= threshold + delta_threshold) {
			sum_over_threshold++;
		}
		if (filt_measures[i_threshold] <= threshold - delta_threshold) {
			sum_under_threshold++;
		}
	}

	if (sum_over_threshold == sizeMeasuresTab - 5){
		//Serial.print("thr : ");Serial.println(threshold);
		//Serial.print("initial : ");Serial.println(initial_measure);
		//Serial.print("val last: ");Serial.println(val_last_change);

		flag_out = 1; // surface
	}
	if (sum_under_threshold == sizeMeasuresTab - 5){
		flag_out = 2; //rentree
	}

	return flag_out;
}







