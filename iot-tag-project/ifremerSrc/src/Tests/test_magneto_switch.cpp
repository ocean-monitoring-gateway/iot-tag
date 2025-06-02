/*
 * test_magneto_switch.cpp
 *
 *  Created on: 28 févr. 2020
 *      Author: jfezande
 */


#include "Arduino.h"
#include "../_board.h"
#include "../_appDirectives.h"

#include "STM32L0.h"

#include "LSM303AGR_M.h"

extern LSM303AGR_M LSM303AGR_M;

volatile bool magnetoFlag = false;

void magnetoInterrupt()
{
	magnetoFlag = true;
	STM32L0.wakeup();
}


void test_magneto_switch(void){

	#define LSM303AGR_M_intPin  A2 // interrupt for magnetometer data ready

	uint8_t LSM303AGR_Mstatus;

	Serial.begin(9600);
	delay(5000);

	I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
	delay(1000);
	I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
	delay(1000);


	float param_magBias[3] = {0.0f, 0.0f, 0.0f}, param_magScale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
	int16_t magData[4];              // Stores the 16-bit signed sensor output

	pinMode(LSM303AGR_M_intPin, INPUT);    // set up interrupt pins

	attachInterrupt(LSM303AGR_M_intPin ,   magnetoInterrupt, RISING);  // define data ready interrupt for intPin  output of LIS2MDL

	LSM303AGR_M.selfTest();
	LSM303AGR_M.reset(); // software reset LIS2MDL to default registers
	LSM303AGR_M.init(MODR_10Hz);
	//LSM303AGR_M.offsetBias(param_magBias, param_magScale);


	while(1){

		if(magnetoFlag){


			LSM303AGR_Mstatus = LSM303AGR_M.status();

			if (LSM303AGR_Mstatus & 0x08) // if all axes have new data ready
			{
			//LOGLN(LSM303AGR_M.status());

				LSM303AGR_M.readData(magData);  // read data register to clear interrupt before main loop
				LOG(magData[0]);LOG(";");
				LOG(magData[1]);LOG(";");
				LOG(magData[2]);LOGLN("");
			}
//			delay(1000);
		}
	}




}
