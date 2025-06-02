/* 09/01/2020 Pierre Gogendeau  

Libraries of the for testing the accelerometer 

*/

#ifndef _TEST_ACCELERO_H_
#define _TEST_ACCELERO_H_

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

	int16_t accelData[3];
	int16_t magData[3];
	int32_t depth_mm;

}data_flash_t;

typedef struct __attribute__((packed, aligned(1))){
	//version 1.0

	//Accel param
	float aRes;
	float accelBias[3];
	int8_t AODR ;
	int8_t Ascale;
	//Mag param
	float magBias[3];
	float magScale[3];
	float mRes;
	int8_t MODR;

}data_param_flash_t;

//LSM303AGR accel definitions
#define LSM303AGR_A_intPin1 A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_A_intPin2 3   // interrupt2 pin definitions, significant motion
//LIS2MDL magnetometer definitions
#define LSM303AGR_M_intPin  A2 // interrupt for magnetometer data ready


void blink_led(int nb_blink);
void initLSM303AGR();
//void init_test_accelero_1(LSM303AGR_M LSM303AGR_M, uint8_t magFreq, LSM303AGR_A LSM303AGR_A, uint8_t Ascale, uint8_t accelFreq, SPIFlash SPIFlash, uint8_t MODR, uint8_t AODR);
void test_accelero_1();
void printRTC (uint8_t index);
void getRTC (uint8_t index);
void printAscale();
void printAODR();

void myinthandler1();
void myinthandler2();
void myinthandler3();
void callbackPressureTimer();

#endif
