/*
 * _appDirectives.h
 *
 *  Created on: 22 janv. 2020
 *      Author: jfezande
 */

#ifndef APPDIRECTIVES_H_
#define APPDIRECTIVES_H_

#include "_board.h"

//use to set all application directives
//should be call in any application files

// DEBUG AND TEST DEFINES
#define USE_SERIAL_DEBUG
#define DISABLE_IWDG


#define USE_PRESSURE_INJECTOR

#define	LOGINIT()	Serial.begin(115200)

//invert comment to no compile "log"
#define LOG(x) 		Serial.print(x)
#define LOGLN(x) 	Serial.println(x)

//#define LOG(x)
//#define LOGLN(x)


//UTIL DEFINE
#define GNSS_UNDEFINED_VALUE	(0x80000000)

//APPLICATION PARAMETER DEFINES

#define TRIG_ON_1BA // Active surfacage avec capteur de pression 1BA
#define SURFACE_DEPTH_CM				(4) // Seuil à laquelle on considère le surfaçage après une plongée avec le capteur de pression
#define SUBSURFACE_ENTER_DEPTH_CM		(18) // Seuil de sortie du mode de surface

#define SUBSURFACE_DEPTH_CM		(25) // Seuil de sortie du mode de subsurface pour une plongée
#define SUBSURFACE_MAXTIME_S	(1*60UL) // Seuil temporelle pour passer en plongée lorsque la tortue est en mode subsurface. Emission a // nouveau possible si on dépasse ce seuil

#define SURFACESENSOR_ACTIVATION_DEPTH_CM	(10) // Seuil d'activation du capteur de surface
#define ACTIVE_SURFACE_TRIG_DEBUG // Activation surface debug ??

#define LIMIT_OF_1BA_SENSOR_mPA	(200000000LL) //2000hPa Limite pour laquelle on utilise le capteur 30BA
//#define LIMIT_OF_1BA_SENSOR_mPA	(125000000LL)     //2.5 m --> Limite pour laquelle on utilise le capteur 30BA

#define DIVE_NB_ZONES			(3) //the number of dive zones
#define DIVE_ZONE_LIMITS_M		{1,2} // the limit between the dive zone (DIVE_NB_ZONES -1)

#define GNSS_EHPE_OK			(25.0f) // acceptation level
//#define GNSS_MAX_ACTIVE_TIME_S	(2*60UL) //2 minutes
#define GNSS_MAX_ACTIVE_TIME_S	(60) //20 secondes
#define GNSS_RELOAD_TIME_S (3600)    //1h - time before reactivating the GPS for long surfacing

#define SURFACESENSOR_SURFACE_TIMEOUT	(30)


// Surface detection tuning parameters
#define delta_threshold_out 	800
#define delta_threshold_in 		400
#define deltaT_surface_detect 	20
#define sizeMeasuresTab  		10



//#define	APPEUI		"70B3D57ED0028732"
//#define APPKEY		"88BD861922D678C17A70A359D2B62A05"

//Orange Network
#define	APPEUI		"70B3D57ED0B28732"
#define APPKEY		"88BD861922C678C17A70A359D2B62A05"

//Local RAK network
//#define APPEUI	"c1ea4e684392fec2"
//#define APPKEY		"018eb0c20c872a01f85606f03a076f80"

//chirpStack
//#define	APPEUI		"70B3D57ED0B28732"
//#define APPKEY		"CA644B5C0679CD8509B0E1D246B0DC22"


#define DATARATE	3


//MESSAGE FORMAT
#define HISTO_SIZE	5
//#define HISTO_DEPTH_LIMITS_CM 		{ 50, 80, 100, 120 } //4 depth limit values <-- ORIGINAL VALUES
#define HISTO_DEPTH_LIMITS_CM 		{ 300, 1000, 1500, 2000 } //4 depth limit values
#define HISTO_STOPTIME_LIMITS_S		{ 60, 120, 140, 160 } //4 stop time limit values

extern const char *const TriggerStr[4];

#endif /* APPDIRECTIVES_H_ */
