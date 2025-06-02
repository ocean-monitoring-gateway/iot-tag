/*
 * test_accelero.h
 *
 *  Created on: 15 avr. 2020
 *      Author: jnguyen
 */

#ifndef SRC_TESTS_TEST_ACCELERO_H_
#define SRC_TESTS_TEST_ACCELERO_H_

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
	//version 1.0

	int16_t accel_data_test[3];
	int16_t mag_data_test[3];
	int32_t depth_mm_1_test;
	int32_t depth_mm_30_test;

}dataFlash_test_t;

typedef struct __attribute__((packed, aligned(1))){
	//version 1.0

	//Accel param
	float a_Res_test;
	float accel_bias_test[3];
	int8_t A_ODR_test;
	int8_t A_scale_test;
	//Mag param
	float mag_bias_test[3];
	float mag_scale_test[3];
	float m_Res_test;
	int8_t M_ODR_test;

}dataParamFlash_test_t;


//LSM303AGR accel definitions
#define LSM303AGR_A_intPin1_test A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_A_intPin2_test 3   // interrupt2 pin definitions, significant motion
//LIS2MDL magnetometer definitions
#define LSM303AGR_M_intPin_test  A2 // interrupt for magnetometer data ready


void blink_led_test(int nb_blink);
//void initLSM303AGR();
//void init_test_accelero_1(LSM303AGR_M LSM303AGR_M, uint8_t magFreq, LSM303AGR_A LSM303AGR_A, uint8_t Ascale, uint8_t accelFreq, SPIFlash SPIFlash, uint8_t MODR, uint8_t AODR);
void test_accelero();
void print_RTC_test (uint8_t index);
void get_RTC_test (uint8_t index);

void callbackPressureTimer_test();
void myinthandler_1_test();
void myinthandler_2_test();
void myinthandler_3_test();

#endif /* SRC_TESTS_TEST_ACCELERO_H_ */
