/*
 * ifremerLoraMsg.h
 *
 *  Created on: 16 janv. 2020
 *      Author: jfezande
 */

#ifndef IFREMERLORA_IFREMERLORAMSG_H_
#define IFREMERLORA_IFREMERLORAMSG_H_


#include "_MessageDefintion.h"

typedef enum {
	ilp_CayenneLPP,
	ilp_StringMsg,
	ilp_histo,

	//admin
	ilp_keys,

	//test
	ilp_GnssTracking,
	ilp_TTFF,
	ilp_SurfaceDebug,
	ilp_GnssTrackingV2

}ifremerLoraPort_t;

const uint8_t loraPortNumber[]={
	[ilp_CayenneLPP] = 10,
	[ilp_StringMsg] = 11,
	[ilp_histo] = 12,

	[ilp_keys] = 50,

	// test are over 100
	[ilp_GnssTracking] = 100,
	[ilp_TTFF] = 101,
	[ilp_SurfaceDebug] = 102,
	[ilp_GnssTrackingV2] = 103

};

typedef struct __attribute__((packed, aligned(1))){
	float lat;
	float lng;
	float ehpe;
	uint32_t ttf;
}data_gnssTracking_t;


typedef struct __attribute__((packed, aligned(1))){
	int32_t lat;
	int32_t lng;
	uint32_t ehpe;
	uint32_t hdop;
	uint32_t ttf;
	uint16_t nbSat;
	uint8_t noFixCount;
	uint8_t zeroSatTimeout;

}data_gnssTrackingV2_t;


typedef struct __attribute__((packed, aligned(1))){
	uint16_t ttff;
	//all ehpe is x10
	uint16_t ehpe[7]; //index 0 for ehpe at TTFF, 1 at TTFF +10s, ...
}data_TTFF_t;







#endif /* IFREMERLORA_IFREMERLORAMSG_H_ */
