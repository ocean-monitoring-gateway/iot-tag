/*
 * test_release_pressure_flash.h
 *
 *  Created on: 7 juil. 2020
 *      Author: jnguyen
 */

#ifndef SRC_TESTS_TEST_RELEASE_PRESSURE_FLASH_H_
#define SRC_TESTS_TEST_RELEASE_PRESSURE_FLASH_H_

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

	int32_t depth_mm_1_r;
	int32_t depth_mm_30_r;
	int32_t temp_1_r;
	int32_t temp_30_r;

}dataFlash_r_t;

//LSM303AGR accel definitions
#define LSM303AGR_A_intPin1_r A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_A_intPin2_r 3   // interrupt2 pin definitions, significant motion

void blink_led(int nb_blink);
void test_release_pressure_flash();

void callback_pressureTimer_r();

void myinthandler_1_r();
void myinthandler_2_r();

#endif /* SRC_TESTS_TEST_RELEASE_PRESSURE_FLASH_H_ */
