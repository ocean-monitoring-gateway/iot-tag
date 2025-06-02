/*
 * test_donnees_lora.h
 *
 *  Created on: 1 juil. 2020
 *      Author: jnguyen
 */

#ifndef SRC_TESTS_TEST_DONNEES_LORA_H_
#define SRC_TESTS_TEST_DONNEES_LORA_H_


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
	int32_t depth_mm_1_donnees_lora;
	int32_t depth_mm_30_donnees_lora;
	int32_t temp_1_donnees_lora;
	int32_t temp_30_donnees_lora;
}data_donnees_lora_t;


void blink_led(int nb_blink);
void test_donnees_lora();
void callback_tick_donnees_lora();
void callback_pressureTimer_donnees_lora();

#endif /* SRC_TESTS_TEST_DONNEES_LORA_H_ */
