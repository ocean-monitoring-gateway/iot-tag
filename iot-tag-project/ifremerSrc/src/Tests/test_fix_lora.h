/*
 * test_fix_lora.h
 *
 *  Created on: 1 juil. 2020
 *      Author: jnguyen
 */

#ifndef SRC_TESTS_TEST_FIX_LORA_H_
#define SRC_TESTS_TEST_FIX_LORA_H_


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
	float lat_fix_lora;
	float lng_fix_lora;
	float ehpe_fix_lora;
	uint8_t nb_sat_fix_lora;
}data_gnssTracking_fix_lora_t;


void blink_led(int nb_blink);
void test_fix_lora();
bool fixGPS_fix_lora();
void callback_tick_fix_lora();

#endif /* SRC_TESTS_TEST_FIX_LORA_H_ */
