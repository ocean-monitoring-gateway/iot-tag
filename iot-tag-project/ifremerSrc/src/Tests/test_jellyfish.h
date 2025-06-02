/*
 * test_jellyfish.h
 *
 *  Created on: 6 avr. 2020
 *      Author: jnguyen
 */

#ifndef SRC_TESTS_TEST_JELLYFISH_H_
#define SRC_TESTS_TEST_JELLYFISH_H_

#include <STM32L0.h>
#include "SPIFlash.h"
#include <RTC.h>
#include "flash_functions.h"
#include "LSM303AGR_A.h"
#include "LSM303AGR_M.h"
#include "utils.h"
#include "TimerMillis.h"
#include "../Manager/_manager.h"


typedef struct __attribute__((packed, aligned(1))){
	float lat_jelly;
	float lng_jelly;
	float ehpe_jelly;
	uint32_t ttf_jelly;
	uint8_t nb_sat;
}data_gnssTracking_jellyfish_t;

typedef struct __attribute__((packed, aligned(1))){
	//version 1.0

	int16_t accel_data[3];
	int16_t mag_data[3];
	int32_t depth_mm_1;
	int32_t depth_mm_30;
	int32_t temp_1;
	int32_t temp_30;

}dataFlash_t;

typedef struct __attribute__((packed, aligned(1))){
	//version 1.0

	//Accel param
	float a_Res;
	float accel_bias[3];
	int8_t A_ODR ;
	int8_t A_scale;
	//Mag param
	float mag_bias[3];
	float mag_scale[3];
	float m_Res;
	int8_t M_ODR;

}dataParamFlash_t;

typedef enum {

	app_boot = 0,
	app_surface,
	app_subsurface,
	app_surfacing,
	app_dive,

	//last
	app_size

}appMode_t;


//LSM303AGR accel definitions
#define LSM303AGR_A_intPin1_ A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_A_intPin2_ 3   // interrupt2 pin definitions, significant motion
//LIS2MDL magnetometer definitions
#define LSM303AGR_M_intPin_  A2 // interrupt for magnetometer data ready

// VEML6030 interrupt detects excursions below a low threshold abd above a high threshold
#define VEML6030Int A3

void blink_led(int nb_blink);
//void initLSM303AGR();
//void init_test_accelero_1(LSM303AGR_M LSM303AGR_M, uint8_t magFreq, LSM303AGR_A LSM303AGR_A, uint8_t Ascale, uint8_t accelFreq, SPIFlash SPIFlash, uint8_t MODR, uint8_t AODR);
void test_jellyfish();
void print_RTC (uint8_t index);
void get_RTC (uint8_t index);

void LoRa_configuration_connection();
bool fixGPS();
void callback_tick();
void callback_pressureTimer();
void myinthandler_1();
void myinthandler_2();
void myinthandler_3();

void surface_init();
void surface_process();
void appProcess_boot();
void appProcess_surface();
void appProcess_subsurface();
void appProcess_surfacing();
void appProcess_dive();
void start_Diving();
bool stop_Diving(uint8_t triggerId);
void appTickCallback();
void surfaceTickCallback();
void changeAppMode(appMode_t mode);

#endif /* SRC_TESTS_TEST_JELLYFISH_H_ */
