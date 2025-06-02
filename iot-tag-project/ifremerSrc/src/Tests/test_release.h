/*
 * test_release.h
 *
 *  Created on: 8 juin 2020
 *      Author: jnguyen
 */

#ifndef SRC_TESTS_TEST_RELEASE_H_
#define SRC_TESTS_TEST_RELEASE_H_


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
	float lat_release;
	float lng_release;
}data_gnssTracking_release_t;

typedef struct __attribute__((packed, aligned(1))){
	//version 1.0

	int16_t accel_data_release[3];
	int16_t mag_data_release[3];
	int32_t depth_mm_1_release;
	int32_t depth_mm_30_release;
	int32_t temp_1_release;
	int32_t temp_30_release;

}dataFlash_release_t;

typedef struct __attribute__((packed, aligned(1))){
	//version 1.0

	//Accel param
	float a_Res_release;
	float accel_bias_release[3];
	int8_t A_ODR_release;
	int8_t A_scale_release;
	//Mag param
	float mag_bias_release[3];
	float mag_scale_release[3];
	float m_Res_release;
	int8_t M_ODR_release;

}dataParamFlash_release_t;

typedef enum {

	app_boot_release = 0,
	app_surface_release,
	app_subsurface_release,
	app_surfacing_release,
	app_dive_release,

	//last
	app_size_release

}appMode_release_t;


//LSM303AGR accel definitions
#define LSM303AGR_A_intPin1_release A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_A_intPin2_release 3   // interrupt2 pin definitions, significant motion
//LIS2MDL magnetometer definitions
#define LSM303AGR_M_intPin_release  A2 // interrupt for magnetometer data ready

// VEML6030 interrupt detects excursions below a low threshold abd above a high threshold
#define VEML6030Int_release A3

void blink_led(int nb_blink);
void test_release();
void print_RTC_release (uint8_t index);
void get_RTC_release (uint8_t index);

bool fixGPS_release();
void callback_tick_release();
void callback_pressureTimer_release();
void myinthandler_1_release();
void myinthandler_2_release();
void myinthandler_3_release();

void surface_init_release();
void surface_process_release();
void appProcess_boot_release();
void appProcess_surface_release();
void appProcess_subsurface_release();
void appProcess_surfacing_release();
void appProcess_dive_release();
void start_Diving_release();
bool stop_Diving_release(uint8_t triggerId);
void appTickCallback_release();
void surfaceTickCallback_release();
void changeAppMode_release(appMode_release_t mode);


#endif /* SRC_TESTS_TEST_RELEASE_H_ */
